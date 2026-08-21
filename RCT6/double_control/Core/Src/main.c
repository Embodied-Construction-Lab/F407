#include "main.h"
#include "oled_ssd1306.h"
#include "pca9685.h"
#include "truck_control.h"
#include "truck_receiver.h"
#include <stdio.h>
#include <string.h>

#define RX_SIZE 256U
#define TX_SIZE 512U
#define STATUS_PERIOD_MS 100U
#define OLED_ROW_PERIOD_MS 40U
#define OLED_RETRY_MS 1000U
#define OLED_LINE_SIZE 22U
#define FIRMWARE_ID "double-control-f103-oled-1"

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

static uint8_t rx_dma[RX_SIZE];
static TruckReceiver receiver;
static TruckCommand command;
static TruckOutputs requested_outputs;
static TruckOutputs outputs;
static TruckEscController drive_esc;
static uint8_t pca_ready, received, failsafe = 1U, uart_ready;
static uint8_t rx_mask, pwm_cache_valid;
static uint8_t button_override, oled_ready, oled_row;
static uint16_t pwm_cache[TRUCK_OUTPUT_CHANNELS];
static HAL_StatusTypeDef control_i2c_status = HAL_ERROR;
static uint32_t last_control_tick, last_status_tick;
static uint32_t valid_frames, invalid_frames, button_tick;
static uint32_t last_oled_tick, last_oled_retry;
static volatile uint8_t uart_restart, tx_busy;
static char tx_buffer[TX_SIZE];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);

static HAL_StatusTypeDef StartReceive(void)
{
  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
      &huart2, rx_dma, sizeof(rx_dma));
  if ((status == HAL_OK) && (huart2.hdmarx != NULL))
  {
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    uart_ready = 1U;
  }
  else uart_ready = 0U;
  return status;
}

static HAL_StatusTypeDef ApplyOutputs(const TruckOutputs *target)
{
  uint8_t channel;
  if (target == NULL) return HAL_ERROR;
  if (pca_ready == 0U)
  {
    if (Pca9685_Init(&hi2c1) != HAL_OK)
    {
      pwm_cache_valid = 0U;
      return HAL_ERROR;
    }
    pca_ready = 1U;
    pwm_cache_valid = 0U;
  }
  for (channel = 0U; channel < TRUCK_OUTPUT_CHANNELS; ++channel)
  {
    if (!TruckControl_IsActiveChannel(channel)) continue;
    if ((pwm_cache_valid != 0U) &&
        (pwm_cache[channel] == target->pwm_count[channel])) continue;
    if (Pca9685_SetPwm(&hi2c1, channel, 0U,
                       target->pwm_count[channel]) != HAL_OK)
    {
      pca_ready = 0U;
      pwm_cache_valid = 0U;
      return HAL_ERROR;
    }
    pwm_cache[channel] = target->pwm_count[channel];
  }
  pwm_cache_valid = 1U;
  return HAL_OK;
}

static int16_t AngleTenths(float angle)
{
  float scaled;
  if (angle < TRUCK_STEERING_MIN_DEG) angle = TRUCK_STEERING_MIN_DEG;
  else if (angle > TRUCK_STEERING_MAX_DEG) angle = TRUCK_STEERING_MAX_DEG;
  scaled = angle * 10.0f;
  return (int16_t)((scaled >= 0.0f) ? scaled + 0.5f : scaled - 0.5f);
}

static void FormatTenths(char *text, size_t size, int16_t value)
{
  const uint16_t n = (uint16_t)((value < 0) ? -(int32_t)value : value);
  (void)snprintf(text, size, "%s%u.%u", (value < 0) ? "-" : "",
                 (unsigned int)(n / 10U), (unsigned int)(n % 10U));
}

static void FormatAxis(char *text, size_t size, float value, uint8_t is_signed)
{
  int32_t milli;
  uint32_t n;
  if (is_signed != 0U)
  {
    if (value < -1.0f) value = -1.0f;
  }
  else if (value < 0.0f) value = 0.0f;
  if (value > 1.0f) value = 1.0f;
  milli = (int32_t)((value >= 0.0f) ? value * 1000.0f + 0.5f :
                    value * 1000.0f - 0.5f);
  n = (uint32_t)((milli < 0) ? -milli : milli);
  (void)snprintf(text, size, "%s%lu.%03lu", (milli < 0) ? "-" : "",
                 (unsigned long)(n / 1000U), (unsigned long)(n % 1000U));
}

static void SendStatus(const char *state, HAL_StatusTypeDef i2c_status)
{
  char angle[9], throttle[8], brake[8];
  int length;
  if ((state == NULL) || (tx_busy != 0U)) return;
  FormatTenths(angle, sizeof(angle), AngleTenths(command.steering));
  FormatAxis(throttle, sizeof(throttle), command.throttle, 1U);
  FormatAxis(brake, sizeof(brake), command.brake, 0U);
  length = snprintf(
      tx_buffer, sizeof(tx_buffer),
      "{\"type\":\"truck\",\"timestamp_ms\":%lu,\"status\":\"%s\","
      "\"i2c\":%u,\"fw\":\"" FIRMWARE_ID "\",\"rx_mask\":%u,"
      "\"input\":%u,\"up\":%u,\"down\":%u,\"feedback_valid\":0,"
      "\"control_steering_deg\":%s,\"current_steering_deg\":null,"
      "\"sensor_steering_deg\":null,\"motor_speed_percent\":%d,"
      "\"bucket_angle_deg\":0.0,\"received_steering_deg\":%s,"
      "\"received_throttle\":%s,\"received_brake\":%s,"
      "\"throttle_percent\":%d,\"brake_percent\":%d,"
      "\"drive_percent\":%d,\"lift_percent\":%d,\"esc\":%u,"
      "\"pwm\":[%u,%u,%u]}\r\n",
      (unsigned long)HAL_GetTick(), state,
      (i2c_status == HAL_OK) ? 1U : 0U, (unsigned int)rx_mask,
      (unsigned int)command.format, (unsigned int)command.up,
      (unsigned int)command.down, angle, (int)outputs.drive_percent, angle,
      throttle, brake, (int)outputs.throttle_percent,
      (int)outputs.brake_percent, (int)outputs.drive_percent,
      (int)outputs.lift_percent, (unsigned int)drive_esc.state,
      (unsigned int)outputs.pwm_count[TRUCK_CHANNEL_STEERING],
      (unsigned int)outputs.pwm_count[TRUCK_CHANNEL_LIFT],
      (unsigned int)outputs.pwm_count[TRUCK_CHANNEL_DRIVE]);
  if ((length <= 0) || ((size_t)length >= sizeof(tx_buffer))) return;
  tx_busy = 1U;
  if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx_buffer,
                           (uint16_t)length) != HAL_OK) tx_busy = 0U;
}

static void PeriodicStatus(const char *state, HAL_StatusTypeDef i2c_status,
                           uint32_t now)
{
  if ((uint32_t)(now - last_status_tick) < STATUS_PERIOD_MS) return;
  last_status_tick = now;
  SendStatus(state, i2c_status);
}

static void UpdateEsc(uint32_t now)
{
  const int16_t drive = TruckEsc_Update(
      &drive_esc, requested_outputs.drive_percent, now);
  outputs = requested_outputs;
  TruckControl_SetDrivePercent(&outputs, drive);
}

static void ProcessFrames(void)
{
  char frame[TRUCK_FRAME_MAX_LEN];
  uint8_t valid = 0U, invalid = 0U;
  const uint8_t analog_mask = (uint8_t)TRUCK_FIELD_CONTROL;
  const uint8_t button_mask = (uint8_t)(TRUCK_FIELD_UP | TRUCK_FIELD_DOWN);

  while (TruckReceiver_Pop(&receiver, frame, sizeof(frame)))
  {
    TruckCommand update, normalized;
    TruckCommandFormat input_format;
    uint8_t mask;
    const uint32_t now = HAL_GetTick();
    if (!TruckReceiver_ParseJsonUpdate(frame, &update, &mask) ||
        (((mask & analog_mask) != 0U) &&
         ((mask & analog_mask) != analog_mask)))
    {
      invalid_frames++;
      invalid = 1U;
      continue;
    }
    input_format = update.format;
    if (update.format == TRUCK_COMMAND_LOGI_RAW)
    {
      TruckControl_NormalizeRawCommand(&update, &normalized);
      update = normalized;
    }
    if ((mask & TRUCK_FIELD_STEERING) != 0U) command.steering = update.steering;
    if ((mask & TRUCK_FIELD_THROTTLE) != 0U) command.throttle = update.throttle;
    if ((mask & TRUCK_FIELD_BRAKE) != 0U) command.brake = update.brake;
    if ((mask & TRUCK_FIELD_UP) != 0U)
    {
      command.up = update.up;
      if (update.up != 0U) command.down = 0U;
    }
    if ((mask & TRUCK_FIELD_DOWN) != 0U)
    {
      command.down = update.down;
      if (update.down != 0U) command.up = 0U;
    }
    if (((mask & analog_mask) == 0U) && ((mask & button_mask) != 0U))
    {
      button_override = ((command.up != 0U) || (command.down != 0U));
      button_tick = now;
    }
    else if ((mask & button_mask) != 0U) button_override = 0U;
    command.format = input_format;
    rx_mask = mask;
    valid_frames++;
    valid = 1U;
  }
  if (valid != 0U)
  {
    const uint32_t now = HAL_GetTick();
    TruckControl_MapCommand(&command, &requested_outputs);
    UpdateEsc(now);
    control_i2c_status = ApplyOutputs(&outputs);
    last_control_tick = now;
    received = 1U;
    failsafe = 0U;
    PeriodicStatus("applied", control_i2c_status, now);
  }
  else if (invalid != 0U)
    PeriodicStatus("invalid_json", HAL_ERROR, HAL_GetTick());
}

static void ServiceButton(void)
{
  uint32_t now;
  if (button_override == 0U) return;
  now = HAL_GetTick();
  if ((uint32_t)(now - button_tick) <= TRUCK_BUTTON_OVERRIDE_TIMEOUT_MS) return;
  button_override = 0U;
  command.up = 0U;
  command.down = 0U;
  TruckControl_MapCommand(&command, &requested_outputs);
  UpdateEsc(now);
  control_i2c_status = ApplyOutputs(&outputs);
  SendStatus("button_timeout", control_i2c_status);
}

static void ServiceEsc(void)
{
  const uint16_t previous = outputs.pwm_count[TRUCK_CHANNEL_DRIVE];
  const TruckEscState state = drive_esc.state;
  UpdateEsc(HAL_GetTick());
  if ((outputs.pwm_count[TRUCK_CHANNEL_DRIVE] != previous) ||
      (drive_esc.state != state))
  {
    control_i2c_status = ApplyOutputs(&outputs);
    SendStatus("esc_state", control_i2c_status);
  }
}

static void ServiceFailsafe(void)
{
  if ((received == 0U) || (failsafe != 0U) ||
      !TruckControl_IsTimedOut(HAL_GetTick(), last_control_tick)) return;
  TruckControl_SetNeutral(&requested_outputs);
  TruckEsc_Reset(&drive_esc);
  outputs = requested_outputs;
  control_i2c_status = ApplyOutputs(&outputs);
  failsafe = 1U;
  SendStatus("failsafe", control_i2c_status);
}

static void ServiceUart(void)
{
  if (uart_restart == 0U) return;
  uart_restart = 0U;
  (void)HAL_UART_AbortReceive(&huart2);
  if (StartReceive() != HAL_OK) uart_restart = 1U;
}

static void DrawOledRow(uint8_t row)
{
  char line[OLED_LINE_SIZE], angle[9];
  OledSsd1306_ClearRow(row);
  switch (row)
  {
    case 0U:
      (void)snprintf(line, sizeof(line), "CTRL:%s", (received == 0U) ? "NO" :
          ((failsafe != 0U) ? "FS" : "OK"));
      break;
    case 1U:
      FormatTenths(angle, sizeof(angle), AngleTenths(command.steering));
      (void)snprintf(line, sizeof(line), "A:%s T:%d", angle,
                     (int)outputs.throttle_percent);
      break;
    case 2U:
      (void)snprintf(line, sizeof(line), "B:%d D:%d",
                     (int)outputs.brake_percent, (int)outputs.drive_percent);
      break;
    case 3U:
      (void)snprintf(line, sizeof(line), "L:%d U:%u D:%u",
                     (int)outputs.lift_percent, (unsigned int)command.up,
                     (unsigned int)command.down);
      break;
    case 4U:
      (void)snprintf(line, sizeof(line), "PS:%u PD:%u",
          (unsigned int)outputs.pwm_count[TRUCK_CHANNEL_STEERING],
          (unsigned int)outputs.pwm_count[TRUCK_CHANNEL_DRIVE]);
      break;
    case 5U:
      (void)snprintf(line, sizeof(line), "PL:%u ESC:%u",
          (unsigned int)outputs.pwm_count[TRUCK_CHANNEL_LIFT],
          (unsigned int)drive_esc.state);
      break;
    case 6U:
      (void)snprintf(line, sizeof(line), "U2:%s PCA:%s",
                     (uart_ready != 0U) ? "OK" : "NO",
                     (pca_ready != 0U) ? "OK" : "NO");
      break;
    default:
      (void)snprintf(line, sizeof(line), "I2:%s R:%lu E:%lu",
                     (oled_ready != 0U) ? "OK" : "NO",
                     (unsigned long)valid_frames,
                     (unsigned long)invalid_frames);
      break;
  }
  OledSsd1306_WriteText(row, 0U, line);
}

static void ServiceOled(uint32_t now)
{
  if (oled_ready == 0U)
  {
    if ((uint32_t)(now - last_oled_retry) < OLED_RETRY_MS) return;
    last_oled_retry = now;
    if (OledSsd1306_Init(&hi2c2) != HAL_OK) return;
    oled_ready = 1U;
    oled_row = 0U;
    last_oled_tick = now - OLED_ROW_PERIOD_MS;
  }
  if ((uint32_t)(now - last_oled_tick) < OLED_ROW_PERIOD_MS) return;
  last_oled_tick = now;
  DrawOledRow(oled_row);
  if (OledSsd1306_UpdateRow(oled_row) != HAL_OK)
  {
    oled_ready = 0U;
    last_oled_retry = now;
    return;
  }
  oled_row = (uint8_t)((oled_row + 1U) % OLED_SSD1306_ROWS);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  TruckReceiver_Init(&receiver);
  memset(&command, 0, sizeof(command));
  command.format = TRUCK_COMMAND_DIRECT;
  TruckControl_SetNeutral(&requested_outputs);
  TruckEsc_Init(&drive_esc);
  outputs = requested_outputs;
  control_i2c_status = ApplyOutputs(&outputs);
  last_oled_retry = HAL_GetTick() - OLED_RETRY_MS;
  last_status_tick = HAL_GetTick() - STATUS_PERIOD_MS;
  if (StartReceive() != HAL_OK) Error_Handler();
  SendStatus("ready", control_i2c_status);

  while (1)
  {
    const uint32_t now = HAL_GetTick();
    ProcessFrames();
    ServiceButton();
    ServiceEsc();
    ServiceFailsafe();
    ServiceUart();
    PeriodicStatus((received == 0U) ? "waiting" :
        ((failsafe != 0U) ? "failsafe" : "applied"),
        control_i2c_status, now);
    ServiceOled(now);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  osc.HSIState = RCC_HSI_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void InitI2c(I2C_HandleTypeDef *i2c, I2C_TypeDef *instance)
{
  i2c->Instance = instance;
  i2c->Init.ClockSpeed = 100000;
  i2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
  i2c->Init.OwnAddress1 = 0;
  i2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  i2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  i2c->Init.OwnAddress2 = 0;
  i2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  i2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(i2c) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void) { InitI2c(&hi2c1, I2C1); }
static void MX_I2C2_Init(void) { InitI2c(&hi2c2, I2C2); }

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance == USART2)
  {
    TruckReceiver_FeedFromIsr(&receiver, rx_dma, size);
    if (StartReceive() != HAL_OK) uart_restart = 1U;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) tx_busy = 0U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart_ready = 0U;
    tx_busy = 0U;
    uart_restart = 1U;
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
