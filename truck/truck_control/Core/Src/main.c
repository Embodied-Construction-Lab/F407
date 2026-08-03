/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "oled_ssd1306.h"
#include "pca9685.h"
#include "pwm_timing.h"
#include "truck_control.h"
#include "truck_receiver.h"

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define USART2_DMA_RX_SIZE 256U
#define STATUS_TX_BUFFER_SIZE 512U
#define CONTROL_STATUS_PERIOD_MS 100U
#define ADC_SAMPLE_PERIOD_MS 20U
#define ADC_REFERENCE_MV 3300U
#define ADC_FULL_SCALE_COUNT 4095U
#define STEERING_CENTER_MV 1630
#define STEERING_MIN_ANGLE_TENTHS (-300)
#define STEERING_MAX_ANGLE_TENTHS 300
#define OLED_UPDATE_PERIOD_MS 200U
#define OLED_RETRY_PERIOD_MS 1000U
#define OLED_LINE_SIZE 22U
#define TRUCK_FIRMWARE_ID "truck-control-bidir-1"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

static uint8_t usart2_dma_rx[USART2_DMA_RX_SIZE];
static TruckReceiver truck_receiver;
static TruckCommand latest_command;
static TruckOutputs requested_outputs;
static TruckOutputs truck_outputs;
static TruckEscController drive_esc;
static uint8_t pca9685_ready;
static uint8_t control_received;
static uint8_t control_failsafe_active;
static uint8_t system_ready;
static uint8_t usart2_rx_ready;
static uint8_t last_json_valid;
static uint8_t oled_ready;
static uint8_t last_rx_field_mask;
static uint8_t pca_pwm_cache_valid;
static uint8_t adc_conversion_active;
static uint8_t steering_feedback_valid;
static uint16_t pca_pwm_cache[TRUCK_OUTPUT_CHANNELS];
static uint16_t steering_adc_raw;
static uint16_t steering_voltage_mv;
static int16_t vehicle_steering_angle_tenths;
static HAL_StatusTypeDef last_control_i2c_status = HAL_ERROR;
static uint32_t last_valid_control_tick;
static uint32_t last_periodic_status_tick;
static uint32_t valid_frame_count;
static uint32_t invalid_frame_count;
static uint32_t last_oled_update_tick;
static uint32_t last_oled_retry_tick;
static uint32_t last_adc_sample_tick;
static volatile uint8_t usart2_restart_requested;
static volatile uint8_t usart2_tx_busy;
static char status_tx_buffer[STATUS_TX_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_ADC1_Init(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

static HAL_StatusTypeDef USART2_StartReceiveToIdle(void);
static HAL_StatusTypeDef ApplyServoTargets(
    const TruckOutputs *outputs);
static void SendControlStatus(const char *status,
                              HAL_StatusTypeDef i2c_status);
static void SendPeriodicControlStatus(const char *status,
                                      HAL_StatusTypeDef i2c_status,
                                      uint32_t now_ms);
static void FormatUnitValue(char *buffer, size_t size, float value);
static void FormatSignedUnitValue(char *buffer, size_t size, float value);
static void ProcessReceivedFrames(void);
static void ServiceEscState(void);
static void ServiceControlFailsafe(void);
static void ServiceUsart2Restart(void);
static void ServiceSteeringFeedback(uint32_t now_ms);
static void ServiceControlStatus(uint32_t now_ms);
static void ServiceOled(uint32_t now_ms);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef USART2_StartReceiveToIdle(void)
{
  HAL_StatusTypeDef status;

  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_dma_rx,
                                       sizeof(usart2_dma_rx));
  if (status == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    usart2_rx_ready = 1U;
  }
  else
  {
    usart2_rx_ready = 0U;
  }
  return status;
}

static HAL_StatusTypeDef ApplyServoTargets(
    const TruckOutputs *outputs)
{
  uint8_t channel;

  if (outputs == NULL)
  {
    return HAL_ERROR;
  }

  if (pca9685_ready == 0U)
  {
    if (Pca9685_Init(&hi2c1) != HAL_OK)
    {
      pca_pwm_cache_valid = 0U;
      return HAL_ERROR;
    }
    pca9685_ready = 1U;
    pca_pwm_cache_valid = 0U;
  }

  for (channel = 0U; channel < TRUCK_OUTPUT_CHANNELS; ++channel)
  {
    if (!TruckControl_IsActiveChannel(channel))
    {
      continue;
    }
    if ((pca_pwm_cache_valid != 0U) &&
        (pca_pwm_cache[channel] == outputs->pwm_count[channel]))
    {
      continue;
    }

    if (Pca9685_SetPwm(&hi2c1, channel, 0U,
                       outputs->pwm_count[channel]) != HAL_OK)
    {
      pca9685_ready = 0U;
      pca_pwm_cache_valid = 0U;
      return HAL_ERROR;
    }
    pca_pwm_cache[channel] = outputs->pwm_count[channel];
  }

  pca_pwm_cache_valid = 1U;
  return HAL_OK;
}

static int16_t SteeringCommandToTenths(float angle_deg)
{
  float scaled;

  if (angle_deg < TRUCK_STEERING_MIN_DEG)
  {
    angle_deg = TRUCK_STEERING_MIN_DEG;
  }
  else if (angle_deg > TRUCK_STEERING_MAX_DEG)
  {
    angle_deg = TRUCK_STEERING_MAX_DEG;
  }

  scaled = angle_deg * 10.0f;
  return (int16_t)((scaled >= 0.0f) ?
                   (scaled + 0.5f) : (scaled - 0.5f));
}

static void FormatSignedTenths(char *buffer,
                               size_t size,
                               int16_t tenths)
{
  const uint16_t magnitude = (uint16_t)((tenths < 0) ?
      -(int32_t)tenths : tenths);

  (void)snprintf(buffer, size, "%s%u.%u",
                 (tenths < 0) ? "-" : "",
                 (unsigned int)(magnitude / 10U),
                 (unsigned int)(magnitude % 10U));
}

static void SendControlStatus(const char *status,
                              HAL_StatusTypeDef i2c_status)
{
  char command_angle[9];
  char current_angle[9];
  char received_throttle[8];
  char received_brake[8];
  int length;

  if ((status == NULL) || (usart2_tx_busy != 0U))
  {
    return;
  }

  FormatSignedTenths(command_angle, sizeof(command_angle),
                     SteeringCommandToTenths(latest_command.steering));
  if (steering_feedback_valid != 0U)
  {
    FormatSignedTenths(current_angle, sizeof(current_angle),
                       vehicle_steering_angle_tenths);
  }
  else
  {
    (void)snprintf(current_angle, sizeof(current_angle), "null");
  }
  FormatSignedUnitValue(received_throttle, sizeof(received_throttle),
                        latest_command.throttle);
  FormatUnitValue(received_brake, sizeof(received_brake),
                  latest_command.brake);

  length = snprintf(
      status_tx_buffer, sizeof(status_tx_buffer),
      "{\"type\":\"truck\",\"timestamp_ms\":%lu,"
      "\"status\":\"%s\",\"i2c\":%u,"
      "\"fw\":\"" TRUCK_FIRMWARE_ID "\","
      "\"rx_mask\":%u,\"feedback_valid\":%u,"
      "\"control_steering_deg\":%s,"
      "\"current_steering_deg\":%s,"
      "\"sensor_steering_deg\":%s,"
      "\"motor_speed_percent\":%d,"
      "\"bucket_angle_deg\":0.0,"
      "\"received_steering_deg\":%s,"
      "\"received_throttle\":%s,"
      "\"received_brake\":%s,"
      "\"throttle_percent\":%d,\"brake_percent\":%d,"
      "\"drive_percent\":%d,\"esc\":%u,"
      "\"pwm\":[%u,%u,%u]}\r\n",
      (unsigned long)HAL_GetTick(),
      status,
      (i2c_status == HAL_OK) ? 1U : 0U,
      (unsigned int)last_rx_field_mask,
      (unsigned int)steering_feedback_valid,
      command_angle,
      current_angle,
      current_angle,
      (int)truck_outputs.drive_percent,
      command_angle,
      received_throttle,
      received_brake,
      (int)truck_outputs.throttle_percent,
      (int)truck_outputs.brake_percent,
      (int)truck_outputs.drive_percent,
      (unsigned int)drive_esc.state,
      (unsigned int)truck_outputs.pwm_count[0],
      (unsigned int)truck_outputs.pwm_count[1],
      (unsigned int)truck_outputs.pwm_count[2]);
  if ((length <= 0) || ((size_t)length >= sizeof(status_tx_buffer)))
  {
    return;
  }

  usart2_tx_busy = 1U;
  if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)status_tx_buffer,
                           (uint16_t)length) != HAL_OK)
  {
    usart2_tx_busy = 0U;
  }
}

static void SendPeriodicControlStatus(const char *status,
                                      HAL_StatusTypeDef i2c_status,
                                      uint32_t now_ms)
{
  if ((uint32_t)(now_ms - last_periodic_status_tick) <
      CONTROL_STATUS_PERIOD_MS)
  {
    return;
  }

  last_periodic_status_tick = now_ms;
  SendControlStatus(status, i2c_status);
}

static void UpdateEscFilteredOutputs(uint32_t now_ms)
{
  int16_t filtered_drive;

  truck_outputs = requested_outputs;
  filtered_drive = TruckEsc_Update(&drive_esc,
                                    requested_outputs.drive_percent,
                                    now_ms);
  TruckControl_SetDrivePercent(&truck_outputs, filtered_drive);
}

static void ProcessReceivedFrames(void)
{
  char frame[TRUCK_FRAME_MAX_LEN];
  uint8_t received_valid_frame = 0U;
  uint8_t received_invalid_frame = 0U;
  uint32_t now_ms = HAL_GetTick();

  while (TruckReceiver_Pop(&truck_receiver, frame, sizeof(frame)))
  {
    TruckCommand command;

    if (!TruckReceiver_ParseJson(frame, &command))
    {
      invalid_frame_count++;
      last_json_valid = 0U;
      received_invalid_frame = 1U;
      continue;
    }

    valid_frame_count++;
    last_json_valid = 1U;
    last_rx_field_mask = (uint8_t)TRUCK_FIELD_ALL;
    now_ms = HAL_GetTick();
    latest_command = command;
    received_valid_frame = 1U;
  }

  if (received_valid_frame != 0U)
  {
    HAL_StatusTypeDef i2c_status;

    now_ms = HAL_GetTick();
    TruckControl_MapCommand(&latest_command, &requested_outputs);
    UpdateEscFilteredOutputs(now_ms);
    i2c_status = ApplyServoTargets(&truck_outputs);
    last_control_i2c_status = i2c_status;
    last_valid_control_tick = now_ms;
    control_received = 1U;
    control_failsafe_active = 0U;
    SendPeriodicControlStatus("applied", i2c_status, now_ms);
  }
  else if (received_invalid_frame != 0U)
  {
    SendPeriodicControlStatus("invalid_json", HAL_ERROR, HAL_GetTick());
  }
}

static void ServiceEscState(void)
{
  const uint16_t previous_drive_pwm =
      truck_outputs.pwm_count[TRUCK_CHANNEL_DRIVE];
  const TruckEscState previous_state = drive_esc.state;
  HAL_StatusTypeDef i2c_status;

  UpdateEscFilteredOutputs(HAL_GetTick());
  if ((truck_outputs.pwm_count[TRUCK_CHANNEL_DRIVE] ==
       previous_drive_pwm) &&
      (drive_esc.state == previous_state))
  {
    return;
  }

  i2c_status = ApplyServoTargets(&truck_outputs);
  last_control_i2c_status = i2c_status;
  SendControlStatus("esc_state", i2c_status);
}

static void ServiceControlFailsafe(void)
{
  HAL_StatusTypeDef i2c_status;

  if ((control_received == 0U) || (control_failsafe_active != 0U) ||
      !TruckControl_IsTimedOut(HAL_GetTick(), last_valid_control_tick))
  {
    return;
  }

  TruckControl_SetNeutral(&requested_outputs);
  TruckEsc_Reset(&drive_esc);
  truck_outputs = requested_outputs;
  i2c_status = ApplyServoTargets(&truck_outputs);
  last_control_i2c_status = i2c_status;
  control_failsafe_active = 1U;
  SendControlStatus("failsafe", i2c_status);
}

static void ServiceUsart2Restart(void)
{
  if (usart2_restart_requested == 0U)
  {
    return;
  }

  usart2_restart_requested = 0U;
  (void)HAL_UART_AbortReceive(&huart2);
  if (USART2_StartReceiveToIdle() != HAL_OK)
  {
    usart2_restart_requested = 1U;
  }
}

static void ServiceSteeringFeedback(uint32_t now_ms)
{
  uint32_t voltage_mv;
  int32_t angle_tenths;

  if (adc_conversion_active == 0U)
  {
    if ((uint32_t)(now_ms - last_adc_sample_tick) < ADC_SAMPLE_PERIOD_MS)
    {
      return;
    }

    last_adc_sample_tick = now_ms;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    adc_conversion_active = 1U;
    return;
  }

  if ((ADC1->SR & ADC_SR_EOC) == 0U)
  {
    return;
  }

  steering_adc_raw = (uint16_t)ADC1->DR;
  adc_conversion_active = 0U;

  voltage_mv = ((uint32_t)steering_adc_raw * ADC_REFERENCE_MV +
                ADC_FULL_SCALE_COUNT / 2U) / ADC_FULL_SCALE_COUNT;
  steering_voltage_mv = (uint16_t)voltage_mv;

  /*
   * Calibration:
   * 1330 mV -> -30.0 deg, 1630 mV -> 0.0 deg,
   * 1930 mV -> +30.0 deg. Therefore 1 mV equals 0.1 degree.
   */
  angle_tenths = (int32_t)steering_voltage_mv - STEERING_CENTER_MV;
  if (angle_tenths < STEERING_MIN_ANGLE_TENTHS)
  {
    angle_tenths = STEERING_MIN_ANGLE_TENTHS;
  }
  else if (angle_tenths > STEERING_MAX_ANGLE_TENTHS)
  {
    angle_tenths = STEERING_MAX_ANGLE_TENTHS;
  }

  vehicle_steering_angle_tenths = (int16_t)angle_tenths;
  steering_feedback_valid = 1U;
}

static void ServiceControlStatus(uint32_t now_ms)
{
  const char *status;

  if (control_received == 0U)
  {
    status = "waiting";
  }
  else if (control_failsafe_active != 0U)
  {
    status = "failsafe";
  }
  else
  {
    status = "applied";
  }

  SendPeriodicControlStatus(status, last_control_i2c_status, now_ms);
}

static void FormatUnitValue(char *buffer, size_t size, float value)
{
  uint32_t milli;

  if (value < 0.0f)
  {
    value = 0.0f;
  }
  else if (value > 1.0f)
  {
    value = 1.0f;
  }
  milli = (uint32_t)(value * 1000.0f + 0.5f);

  (void)snprintf(buffer, size, "%lu.%03lu",
                 (unsigned long)(milli / 1000U),
                 (unsigned long)(milli % 1000U));
}

static void FormatSignedUnitValue(char *buffer, size_t size, float value)
{
  int32_t milli;
  uint32_t magnitude;

  if (value < -1.0f)
  {
    value = -1.0f;
  }
  else if (value > 1.0f)
  {
    value = 1.0f;
  }

  milli = (int32_t)((value >= 0.0f) ?
                    (value * 1000.0f + 0.5f) :
                    (value * 1000.0f - 0.5f));
  magnitude = (uint32_t)((milli < 0) ? -milli : milli);

  (void)snprintf(buffer, size, "%s%lu.%03lu",
                 (milli < 0) ? "-" : "",
                 (unsigned long)(magnitude / 1000U),
                 (unsigned long)(magnitude % 1000U));
}

static void OledDrawTelemetry(void)
{
  char line[OLED_LINE_SIZE];
  char command_angle[9];
  char current_angle[9];
  char throttle[8];
  char brake[8];

  FormatSignedTenths(command_angle, sizeof(command_angle),
                     SteeringCommandToTenths(latest_command.steering));
  if (steering_feedback_valid != 0U)
  {
    FormatSignedTenths(current_angle, sizeof(current_angle),
                       vehicle_steering_angle_tenths);
  }
  else
  {
    (void)snprintf(current_angle, sizeof(current_angle), "--.-");
  }
  (void)snprintf(line, sizeof(line), "CUR:%s deg", current_angle);
  OledSsd1306_WriteText(0U, 0U, line);

  (void)snprintf(line, sizeof(line), "CMD:%s deg", command_angle);
  OledSsd1306_WriteText(1U, 0U, line);

  FormatSignedUnitValue(throttle, sizeof(throttle),
                        latest_command.throttle);
  (void)snprintf(line, sizeof(line), "THR:%s (%d%%)", throttle,
                 (int)truck_outputs.throttle_percent);
  OledSsd1306_WriteText(2U, 0U, line);

  FormatUnitValue(brake, sizeof(brake), latest_command.brake);
  (void)snprintf(line, sizeof(line), "BRK:%s (%d%%)", brake,
                 (int)truck_outputs.brake_percent);
  OledSsd1306_WriteText(3U, 0U, line);

  (void)snprintf(line, sizeof(line), "DRV:%d%% PWM:%u",
                 (int)truck_outputs.drive_percent,
                 (unsigned int)
                     truck_outputs.pwm_count[TRUCK_CHANNEL_DRIVE]);
  OledSsd1306_WriteText(4U, 0U, line);

  if (steering_feedback_valid != 0U)
  {
    (void)snprintf(line, sizeof(line), "PA1:%u.%03uV",
                   (unsigned int)(steering_voltage_mv / 1000U),
                   (unsigned int)(steering_voltage_mv % 1000U));
  }
  else
  {
    (void)snprintf(line, sizeof(line), "PA1:----V");
  }
  OledSsd1306_WriteText(5U, 0U, line);

  (void)snprintf(line, sizeof(line), "I1:%s I2:%s",
                 (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) ?
                 "OK" : "NO",
                 (HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) ?
                 "OK" : "NO");
  OledSsd1306_WriteText(6U, 0U, line);

  (void)snprintf(line, sizeof(line), "U2:%s PCA:%s",
                 usart2_rx_ready ? "OK" : "NO",
                 pca9685_ready ? "OK" : "NO");
  OledSsd1306_WriteText(7U, 0U, line);
}

static void ServiceOled(uint32_t now_ms)
{
  if (oled_ready == 0U)
  {
    if ((uint32_t)(now_ms - last_oled_retry_tick) < OLED_RETRY_PERIOD_MS)
    {
      return;
    }

    last_oled_retry_tick = now_ms;
    if (OledSsd1306_Init(&hi2c2) != HAL_OK)
    {
      return;
    }
    oled_ready = 1U;
    last_oled_update_tick = now_ms - OLED_UPDATE_PERIOD_MS;
  }

  if ((uint32_t)(now_ms - last_oled_update_tick) < OLED_UPDATE_PERIOD_MS)
  {
    return;
  }
  last_oled_update_tick = now_ms;

  OledSsd1306_Clear();
  OledDrawTelemetry();

  if (OledSsd1306_Update() != HAL_OK)
  {
    oled_ready = 0U;
    last_oled_retry_tick = now_ms;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  TruckReceiver_Init(&truck_receiver);
  latest_command.steering = 0.0f;
  latest_command.throttle = 0.0f;
  latest_command.brake = 0.0f;
  TruckControl_SetNeutral(&requested_outputs);
  TruckEsc_Init(&drive_esc);
  truck_outputs = requested_outputs;
  if (USART2_StartReceiveToIdle() != HAL_OK)
  {
    Error_Handler();
  }
  last_oled_retry_tick = HAL_GetTick();
  oled_ready = (OledSsd1306_Init(&hi2c2) == HAL_OK) ? 1U : 0U;
  last_oled_update_tick = HAL_GetTick() - OLED_UPDATE_PERIOD_MS;
  last_periodic_status_tick = HAL_GetTick() - CONTROL_STATUS_PERIOD_MS;
  last_adc_sample_tick = HAL_GetTick() - ADC_SAMPLE_PERIOD_MS;
  system_ready = 1U;
  last_control_i2c_status = ApplyServoTargets(&truck_outputs);
  SendControlStatus("ready", last_control_i2c_status);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ProcessReceivedFrames();
    ServiceEscState();
    ServiceControlFailsafe();
    ServiceUsart2Restart();
    ServiceSteeringFeedback(HAL_GetTick());
    ServiceControlStatus(HAL_GetTick());
    ServiceOled(HAL_GetTick());

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_ADC1_CLK_ENABLE();

  /* APB2 is 84 MHz; divide by four to keep ADC clock at 21 MHz. */
  ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;

  ADC1->CR1 = 0U;
  ADC1->CR2 = ADC_CR2_EOCS;
  ADC1->SMPR1 = 0U;
  ADC1->SMPR2 = ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_2;
  ADC1->SQR1 = 0U;
  ADC1->SQR2 = 0U;
  ADC1->SQR3 = 1U;
  ADC1->CR2 |= ADC_CR2_ADON;

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    usart2_tx_busy = 0U;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2)
  {
    TruckReceiver_FeedFromIsr(&truck_receiver, usart2_dma_rx, Size);
    if (USART2_StartReceiveToIdle() != HAL_OK)
    {
      usart2_restart_requested = 1U;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    usart2_rx_ready = 0U;
    usart2_restart_requested = 1U;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
