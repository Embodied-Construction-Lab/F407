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
#include "lcd_spi_154.h"
#include "truck_control.h"
#include "truck_receiver.h"
#include "steering_feedback.h"

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART1_DMA_RX_SIZE 256U
#define STATUS_TX_BUFFER_SIZE 768U
#define ADC_SAMPLE_PERIOD_MS 20U
#define ADC_CONVERSION_TIMEOUT_MS 5U
#define CONTROL_STATUS_PERIOD_MS 100U
#define OLED_UPDATE_PERIOD_MS 500U
#define OLED_RETRY_PERIOD_MS 1000U
#define TFT_ROW_UPDATE_PERIOD_MS 50U
#define TFT_LINE_CHARS 30U
#define PWM_MIN_PULSE_US 500U
#define PWM_MAX_PULSE_US 2500U
#define TRUCK_FIRMWARE_ID "cmd-zgt6-tft-adc-2"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
static uint8_t uart1_dma_rx[UART1_DMA_RX_SIZE];
static TruckReceiver truck_receiver;
static SteeringFeedback steering_feedback;
static uint8_t adc_conversion_active;
static uint32_t last_adc_sample_tick;
static uint32_t last_adc_start_tick;
static TruckCommand latest_command;
static TruckCommand applied_command;
static TruckOutputs requested_outputs;
static TruckOutputs truck_outputs;
static TruckEscController drive_esc;
static uint8_t timer_ready;
static uint8_t uart1_rx_ready;
static uint8_t control_received;
static uint8_t analog_control_received;
static uint8_t control_failsafe_active;
static uint8_t button_override_active;
static uint8_t button_override_up;
static uint8_t button_override_down;
static uint8_t last_rx_field_mask;
static uint8_t oled_ready;
static uint8_t tft_ready;
static uint8_t tft_next_row;
static uint8_t last_json_valid;
static uint32_t last_valid_control_tick;
static uint32_t last_analog_control_tick;
static uint32_t button_override_tick;
static uint32_t last_periodic_status_tick;
static uint32_t last_oled_update_tick;
static uint32_t last_oled_retry_tick;
static uint32_t last_tft_update_tick;
static uint32_t valid_frame_count;
static uint32_t invalid_frame_count;
static volatile uint8_t uart1_restart_requested;
static volatile uint8_t uart1_tx_busy;
static volatile uint32_t uart1_rx_events;
static volatile uint32_t uart1_rx_bytes;
static volatile uint32_t uart1_error_count;
static char status_tx_buffer[STATUS_TX_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t TimerPulseUs(uint16_t pulse_us)
{
  if (pulse_us < PWM_MIN_PULSE_US)
  {
    pulse_us = PWM_MIN_PULSE_US;
  }
  else if (pulse_us > PWM_MAX_PULSE_US)
  {
    pulse_us = PWM_MAX_PULSE_US;
  }
  return pulse_us;
}

static HAL_StatusTypeDef ApplyServoTargets(const TruckOutputs *outputs)
{
  if ((outputs == NULL) || (timer_ready == 0U))
  {
    return HAL_ERROR;
  }

  /* TIM1 counts in microseconds: CH1 steering, CH2 lift, CH3 drive. */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
      TimerPulseUs(outputs->pulse_us[TRUCK_CHANNEL_STEERING]));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
      TimerPulseUs(outputs->pulse_us[TRUCK_CHANNEL_LIFT]));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
      TimerPulseUs(outputs->pulse_us[TRUCK_CHANNEL_DRIVE]));
  return HAL_OK;
}

static HAL_StatusTypeDef StartTimerOutputs(const TruckOutputs *neutral)
{
  if (neutral == NULL)
  {
    return HAL_ERROR;
  }

  /* Load neutral values before enabling any output. */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
      TimerPulseUs(neutral->pulse_us[TRUCK_CHANNEL_STEERING]));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
      TimerPulseUs(neutral->pulse_us[TRUCK_CHANNEL_LIFT]));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
      TimerPulseUs(neutral->pulse_us[TRUCK_CHANNEL_DRIVE]));

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    return HAL_ERROR;
  }
  timer_ready = 1U;
  return HAL_OK;
}

static HAL_StatusTypeDef Uart1StartReceiveToIdle(void)
{
  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
      &huart1, uart1_dma_rx, sizeof(uart1_dma_rx));

  uart1_rx_ready = (status == HAL_OK) ? 1U : 0U;
  if (status == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
  return status;
}

static void ServiceSteeringFeedback(uint32_t now_ms)
{
  if (adc_conversion_active == 0U)
  {
    if ((uint32_t)(now_ms - last_adc_sample_tick) < ADC_SAMPLE_PERIOD_MS)
    {
      return;
    }
    last_adc_sample_tick = now_ms;
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR);
    ADC1->CR2 |= ADC_CR2_SWSTART;
    last_adc_start_tick = now_ms;
    adc_conversion_active = 1U;
    return;
  }
  if (((ADC1->SR & ADC_SR_OVR) != 0U) ||
      (((ADC1->SR & ADC_SR_EOC) == 0U) &&
       ((uint32_t)(now_ms - last_adc_start_tick) >= ADC_CONVERSION_TIMEOUT_MS)))
  {
    __HAL_ADC_DISABLE(&hadc1);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR);
    __HAL_ADC_ENABLE(&hadc1);
    adc_conversion_active = 0U;
    SteeringFeedback_Invalidate(&steering_feedback);
    last_adc_sample_tick = now_ms;
    return;
  }
  if ((ADC1->SR & ADC_SR_EOC) != 0U)
  {
    const uint16_t raw = (uint16_t)ADC1->DR;
    adc_conversion_active = 0U;
    /* EOC may have waited while the display was busy: never rejuvenate DR. */
    (void)SteeringFeedback_Update(&steering_feedback, raw, last_adc_start_tick);
  }
}

static void SendControlStatus(const char *status, HAL_StatusTypeDef pwm_status)
{
  char feedback_json[STEERING_FEEDBACK_JSON_CAPACITY];
  int length;

  if ((status == NULL) || (uart1_tx_busy != 0U))
  {
    return;
  }

  if (!SteeringFeedback_FormatJson(&steering_feedback, HAL_GetTick(),
                                    feedback_json, sizeof(feedback_json)))
  {
    return;
  }

  length = snprintf(status_tx_buffer, sizeof(status_tx_buffer),
      "{\"type\":\"truck\",\"status\":\"%s\",\"i2c\":0,\"timer\":%u,"
      "\"fw\":\"" TRUCK_FIRMWARE_ID "\",\"rx_mask\":%u,"
      "\"timestamp_ms\":%lu,\"control_steering_deg\":%d,%s,"
      "\"rx_up\":%u,\"rx_down\":%u,\"up\":%u,\"down\":%u,"
      "\"button_override\":%u,\"steering_deg\":%d,"
      "\"throttle\":%d,\"brake\":%d,\"drive\":%d,\"lift\":%d,"
      "\"esc\":%u,\"pwm_unit\":\"us\",\"pwm\":[%u,%u,%u,%u]}\r\n",
      status, ((pwm_status == HAL_OK) && (timer_ready != 0U)) ? 1U : 0U,
      (unsigned int)last_rx_field_mask,
      (unsigned long)HAL_GetTick(), (int)truck_outputs.steering_deg,
      feedback_json,
      (unsigned int)latest_command.up, (unsigned int)latest_command.down,
      (unsigned int)applied_command.up, (unsigned int)applied_command.down,
      (unsigned int)button_override_active,
      (int)truck_outputs.steering_deg,
      (int)truck_outputs.throttle_percent,
      (int)truck_outputs.brake_percent,
      (int)truck_outputs.drive_percent,
      (int)truck_outputs.lift_percent,
      (unsigned int)drive_esc.state,
      (unsigned int)truck_outputs.pulse_us[0],
      (unsigned int)truck_outputs.pulse_us[1],
      (unsigned int)truck_outputs.pulse_us[2],
      (unsigned int)truck_outputs.pulse_us[3]);
  if ((length <= 0) || ((size_t)length >= sizeof(status_tx_buffer)))
  {
    return;
  }

  uart1_tx_busy = 1U;
  if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)status_tx_buffer,
                           (uint16_t)length) != HAL_OK)
  {
    uart1_tx_busy = 0U;
  }
}

static void SendPeriodicStatus(const char *status,
                               HAL_StatusTypeDef pwm_status,
                               uint32_t now_ms)
{
  if ((uint32_t)(now_ms - last_periodic_status_tick) <
      CONTROL_STATUS_PERIOD_MS)
  {
    return;
  }
  last_periodic_status_tick = now_ms;
  SendControlStatus(status, pwm_status);
}

static void UpdateEscFilteredOutputs(uint32_t now_ms)
{
  const int16_t filtered_drive = TruckEsc_Update(
      &drive_esc, requested_outputs.drive_percent, now_ms);

  truck_outputs = requested_outputs;
  TruckControl_SetDrivePercent(&truck_outputs, filtered_drive);
}

static void BuildAppliedCommand(void)
{
  applied_command = latest_command;
  if (button_override_active != 0U)
  {
    applied_command.up = button_override_up;
    applied_command.down = button_override_down;
  }
}

static void ApplyCommandUpdate(const TruckCommand *update,
                               uint8_t field_mask,
                               uint32_t now_ms)
{
  const uint8_t analog_mask = TRUCK_FIELD_STEERING |
      TRUCK_FIELD_THROTTLE | TRUCK_FIELD_BRAKE;
  const uint8_t button_mask = TRUCK_FIELD_UP | TRUCK_FIELD_DOWN;
  const uint8_t button_only =
      (((field_mask & analog_mask) == 0U) &&
       ((field_mask & button_mask) != 0U)) ? 1U : 0U;

  if (button_only == 0U)
  {
    /* A motion frame is a snapshot: omitted fields are neutral, not stale. */
    latest_command.steering = 0.0f;
    latest_command.throttle = 0.0f;
    latest_command.brake = 0.0f;
    latest_command.up = 0U;
    latest_command.down = 0U;
    last_analog_control_tick = now_ms;
    analog_control_received = 1U;
  }
  else if ((analog_control_received != 0U) &&
           TruckControl_IsTimedOut(now_ms, last_analog_control_tick))
  {
    /* A button update cannot revive axes from an expired motion frame. */
    analog_control_received = 0U;
    latest_command.steering = 0.0f;
    latest_command.throttle = 0.0f;
    latest_command.brake = 0.0f;
  }

  if ((field_mask & TRUCK_FIELD_STEERING) != 0U)
  {
    latest_command.steering = update->steering;
  }
  if ((field_mask & TRUCK_FIELD_THROTTLE) != 0U)
  {
    latest_command.throttle = update->throttle;
  }
  if ((field_mask & TRUCK_FIELD_BRAKE) != 0U)
  {
    latest_command.brake = update->brake;
  }

  if (button_only != 0U)
  {
    if (button_override_active == 0U)
    {
      button_override_up = 0U;
      button_override_down = 0U;
    }
    if ((field_mask & TRUCK_FIELD_UP) != 0U)
    {
      button_override_up = update->up;
      if (update->up != 0U)
      {
        button_override_down = 0U;
      }
    }
    if ((field_mask & TRUCK_FIELD_DOWN) != 0U)
    {
      button_override_down = update->down;
      if (update->down != 0U)
      {
        button_override_up = 0U;
      }
    }
    button_override_active =
        ((button_override_up != 0U) || (button_override_down != 0U)) ?
        1U : 0U;
    button_override_tick = now_ms;
  }
  else
  {
    if ((field_mask & TRUCK_FIELD_UP) != 0U)
    {
      latest_command.up = update->up;
    }
    if ((field_mask & TRUCK_FIELD_DOWN) != 0U)
    {
      latest_command.down = update->down;
    }
  }
  BuildAppliedCommand();
}

static void ProcessReceivedFrames(void)
{
  char frame[TRUCK_FRAME_MAX_LEN];
  uint8_t received_valid_frame = 0U;
  uint8_t received_invalid_frame = 0U;
  uint32_t now_ms = HAL_GetTick();

  while (TruckReceiver_Pop(&truck_receiver, frame, sizeof(frame)))
  {
    TruckCommand update;
    uint8_t field_mask;

    if (!TruckReceiver_ParseJsonUpdate(frame, &update, &field_mask))
    {
      received_invalid_frame = 1U;
      invalid_frame_count++;
      last_json_valid = 0U;
      continue;
    }
    valid_frame_count++;
    last_json_valid = 1U;
    last_rx_field_mask = field_mask;
    ApplyCommandUpdate(&update, field_mask, now_ms);
    received_valid_frame = 1U;
  }

  if (received_valid_frame != 0U)
  {
    HAL_StatusTypeDef pwm_status;

    now_ms = HAL_GetTick();
    TruckControl_MapCommand(&applied_command, &requested_outputs);
    UpdateEscFilteredOutputs(now_ms);
    pwm_status = ApplyServoTargets(&truck_outputs);
    last_valid_control_tick = now_ms;
    control_received = 1U;
    control_failsafe_active = 0U;
    SendPeriodicStatus("applied", pwm_status, now_ms);
  }
  else if (received_invalid_frame != 0U)
  {
    SendPeriodicStatus("invalid_json", HAL_ERROR, HAL_GetTick());
  }
}

static void ServiceButtonOverride(void)
{
  HAL_StatusTypeDef pwm_status;

  if ((button_override_active == 0U) ||
      ((uint32_t)(HAL_GetTick() - button_override_tick) <=
       TRUCK_BUTTON_OVERRIDE_TIMEOUT_MS))
  {
    return;
  }

  button_override_active = 0U;
  button_override_up = 0U;
  button_override_down = 0U;
  BuildAppliedCommand();
  TruckControl_MapCommand(&applied_command, &requested_outputs);
  UpdateEscFilteredOutputs(HAL_GetTick());
  pwm_status = ApplyServoTargets(&truck_outputs);
  SendControlStatus("button_timeout", pwm_status);
}

static void ServiceEscState(void)
{
  const uint16_t previous_drive_pwm =
      truck_outputs.pulse_us[TRUCK_CHANNEL_DRIVE];
  const TruckEscState previous_state = drive_esc.state;

  UpdateEscFilteredOutputs(HAL_GetTick());
  if ((truck_outputs.pulse_us[TRUCK_CHANNEL_DRIVE] != previous_drive_pwm) ||
      (drive_esc.state != previous_state))
  {
    SendControlStatus("esc_state", ApplyServoTargets(&truck_outputs));
  }
}

static void ServiceAnalogTimeout(void)
{
  if ((analog_control_received == 0U) ||
      !TruckControl_IsTimedOut(HAL_GetTick(), last_analog_control_tick))
  {
    return;
  }

  /* Button-only traffic must not keep an old drive command alive. */
  analog_control_received = 0U;
  latest_command.steering = 0.0f;
  latest_command.throttle = 0.0f;
  latest_command.brake = 0.0f;
  BuildAppliedCommand();
  TruckControl_MapCommand(&applied_command, &requested_outputs);
  UpdateEscFilteredOutputs(HAL_GetTick());
  SendControlStatus("axes_timeout", ApplyServoTargets(&truck_outputs));
}

static void ServiceControlFailsafe(void)
{
  HAL_StatusTypeDef pwm_status;

  if ((control_received == 0U) || (control_failsafe_active != 0U) ||
      !TruckControl_IsTimedOut(HAL_GetTick(), last_valid_control_tick))
  {
    return;
  }

  /* Clear stale axes so a later button-only frame cannot revive old drive. */
  latest_command.steering = 0.0f;
  latest_command.throttle = 0.0f;
  latest_command.brake = 0.0f;
  latest_command.up = 0U;
  latest_command.down = 0U;
  button_override_active = 0U;
  button_override_up = 0U;
  button_override_down = 0U;
  analog_control_received = 0U;
  BuildAppliedCommand();
  TruckControl_SetNeutral(&requested_outputs);
  TruckEsc_Reset(&drive_esc);
  truck_outputs = requested_outputs;
  pwm_status = ApplyServoTargets(&truck_outputs);
  control_failsafe_active = 1U;
  SendControlStatus("failsafe", pwm_status);
}

static void ServiceUart1Restart(void)
{
  if (uart1_restart_requested == 0U)
  {
    return;
  }

  uart1_restart_requested = 0U;
  (void)HAL_UART_AbortReceive(&huart1);
  if (Uart1StartReceiveToIdle() != HAL_OK)
  {
    uart1_restart_requested = 1U;
  }
}

static void ServiceOled(uint32_t now_ms)
{
  char line[22];

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
  OledSsd1306_WriteText(0U, 0U, uart1_rx_ready ? "U1:OK" : "U1:NO");
  OledSsd1306_WriteText(1U, 0U, timer_ready ? "T1:OK" : "T1:NO");
  (void)snprintf(line, sizeof(line), "STR:%d", (int)truck_outputs.steering_deg);
  OledSsd1306_WriteText(2U, 0U, line);
  (void)snprintf(line, sizeof(line), "DRV:%d", (int)truck_outputs.drive_percent);
  OledSsd1306_WriteText(3U, 0U, line);
  (void)snprintf(line, sizeof(line), "LFT:%d", (int)truck_outputs.lift_percent);
  OledSsd1306_WriteText(4U, 0U, line);
  (void)snprintf(line, sizeof(line), "P0:%u P1:%u",
                 (unsigned int)truck_outputs.pulse_us[0],
                 (unsigned int)truck_outputs.pulse_us[1]);
  OledSsd1306_WriteText(5U, 0U, line);
  (void)snprintf(line, sizeof(line), "P2:%u",
                 (unsigned int)truck_outputs.pulse_us[2]);
  OledSsd1306_WriteText(6U, 0U, line);
  (void)snprintf(line, sizeof(line), "FS:%u UP:%u DN:%u",
                 (unsigned int)control_failsafe_active,
                 (unsigned int)applied_command.up,
                 (unsigned int)applied_command.down);
  OledSsd1306_WriteText(7U, 0U, line);

  if (OledSsd1306_Update() != HAL_OK)
  {
    oled_ready = 0U;
    last_oled_retry_tick = now_ms;
  }
}

static void TftWriteLine(uint8_t row, uint32_t color, const char *text)
{
  char padded[TFT_LINE_CHARS + 1U];
  size_t length = 0U;

  memset(padded, ' ', TFT_LINE_CHARS);
  padded[TFT_LINE_CHARS] = '\0';
  if (text != NULL)
  {
    length = strlen(text);
    if (length > TFT_LINE_CHARS)
    {
      length = TFT_LINE_CHARS;
    }
    memcpy(padded, text, length);
  }

  LCD_SetColor(color);
  LCD_DisplayString(0U, (uint16_t)(8U + (uint16_t)row * 28U), padded);
}

static void TftDrawStatusRow(uint8_t row, uint32_t now_ms)
{
  char line[48];
  const char *control_state;
  uint32_t state_color;

  if (control_received == 0U)
  {
    control_state = "WAIT";
    state_color = LCD_YELLOW;
  }
  else if (control_failsafe_active != 0U)
  {
    control_state = "FAILSAFE";
    state_color = LCD_RED;
  }
  else
  {
    control_state = "ACTIVE";
    state_color = LCD_GREEN;
  }

  switch (row)
  {
    case 0U:
      (void)snprintf(line, sizeof(line), "CTRL:%s TIM1:%s",
                     control_state, timer_ready ? "ON" : "OFF");
      TftWriteLine(row, state_color, line);
      break;

    case 1U:
      (void)snprintf(line, sizeof(line), "U1:%s EV:%lu ERR:%lu",
                     uart1_rx_ready ? "DMA" : "OFF",
                     (unsigned long)uart1_rx_events,
                     (unsigned long)uart1_error_count);
      TftWriteLine(row, uart1_rx_ready ? LCD_GREEN : LCD_RED, line);
      break;

    case 2U:
      (void)snprintf(line, sizeof(line), "JSON V:%lu B:%lu Q:%lu",
                     (unsigned long)valid_frame_count,
                     (unsigned long)invalid_frame_count,
                     (unsigned long)truck_receiver.dropped_frames);
      TftWriteLine(row,
                   (last_json_valid != 0U) ? LCD_GREEN :
                   ((invalid_frame_count != 0U) ? LCD_RED : LCD_YELLOW),
                   line);
      break;

    case 3U:
      if (control_received != 0U)
      {
        (void)snprintf(line, sizeof(line), "AGE:%lums MASK:%02X BY:%lu",
                       (unsigned long)(now_ms - last_valid_control_tick),
                       (unsigned int)last_rx_field_mask,
                       (unsigned long)uart1_rx_bytes);
      }
      else
      {
        (void)snprintf(line, sizeof(line), "AGE:---- MASK:00 BY:%lu",
                       (unsigned long)uart1_rx_bytes);
      }
      TftWriteLine(row,
                   (control_failsafe_active != 0U) ? LCD_RED :
                   ((control_received != 0U) ? LCD_CYAN : LCD_YELLOW),
                   line);
      break;

    case 4U:
      (void)snprintf(line, sizeof(line), "S:%d TH:%d BR:%d",
                     (int)truck_outputs.steering_deg,
                     (int)truck_outputs.throttle_percent,
                     (int)truck_outputs.brake_percent);
      TftWriteLine(row, LCD_WHITE, line);
      break;

    case 5U:
      (void)snprintf(line, sizeof(line), "D:%+d L:%+d U:%u DN:%u",
                     (int)truck_outputs.drive_percent,
                     (int)truck_outputs.lift_percent,
                     (unsigned int)applied_command.up,
                     (unsigned int)applied_command.down);
      TftWriteLine(row, LCD_WHITE, line);
      break;

    case 6U:
      (void)snprintf(line, sizeof(line), "PWM S:%u L:%u D:%u us",
                     (unsigned int)truck_outputs.pulse_us[TRUCK_CHANNEL_STEERING],
                     (unsigned int)truck_outputs.pulse_us[TRUCK_CHANNEL_LIFT],
                     (unsigned int)truck_outputs.pulse_us[TRUCK_CHANNEL_DRIVE]);
      TftWriteLine(row, LCD_YELLOW, line);
      break;

    default:
      (void)snprintf(line, sizeof(line), "CCR %lu/%lu/%lu ESC:%u",
                     (unsigned long)TIM1->CCR1,
                     (unsigned long)TIM1->CCR2,
                     (unsigned long)TIM1->CCR3,
                     (unsigned int)drive_esc.state);
      TftWriteLine(row,
                   ((TIM1->CCR1 == truck_outputs.pulse_us[TRUCK_CHANNEL_STEERING]) &&
                    (TIM1->CCR2 == truck_outputs.pulse_us[TRUCK_CHANNEL_LIFT]) &&
                    (TIM1->CCR3 == truck_outputs.pulse_us[TRUCK_CHANNEL_DRIVE])) ?
                       LCD_GREEN : LCD_RED,
                   line);
      break;
  }
}

static void ServiceTft(uint32_t now_ms)
{
  if ((tft_ready == 0U) ||
      ((uint32_t)(now_ms - last_tft_update_tick) <
       TFT_ROW_UPDATE_PERIOD_MS))
  {
    return;
  }

  last_tft_update_tick = now_ms;
  TftDrawStatusRow(tft_next_row, now_ms);
  tft_next_row = (uint8_t)((tft_next_row + 1U) % 8U);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  TruckReceiver_Init(&truck_receiver);
  SteeringFeedback_Init(&steering_feedback);
  latest_command.steering = 0.0f;
  latest_command.throttle = 0.0f;
  latest_command.brake = 0.0f;
  latest_command.up = 0U;
  latest_command.down = 0U;
  BuildAppliedCommand();
  TruckControl_SetNeutral(&requested_outputs);
  TruckEsc_Init(&drive_esc);
  truck_outputs = requested_outputs;

  if (Uart1StartReceiveToIdle() != HAL_OK)
  {
    Error_Handler();
  }
  if (StartTimerOutputs(&truck_outputs) != HAL_OK)
  {
    Error_Handler();
  }
  SPI_LCD_Init();
  tft_ready = 1U;
  tft_next_row = 0U;
  last_tft_update_tick = HAL_GetTick() - TFT_ROW_UPDATE_PERIOD_MS;
  last_oled_retry_tick = HAL_GetTick() - OLED_RETRY_PERIOD_MS;
  last_periodic_status_tick = HAL_GetTick();
  SendControlStatus("ready", HAL_OK);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ServiceSteeringFeedback(HAL_GetTick());
    ProcessReceivedFrames();
    ServiceButtonOverride();
    ServiceAnalogTimeout();
    ServiceEscState();
    ServiceControlFailsafe();
    ServiceUart1Restart();
    ServiceOled(HAL_GetTick());
    ServiceTft(HAL_GetTick());
    SendPeriodicStatus(control_failsafe_active ? "failsafe" :
                       (control_received ? "applied" : "ready"),
                       timer_ready ? HAL_OK : HAL_ERROR, HAL_GetTick());
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* HAL_ADC_Init configures ADC1 but does not set CR2.ADON. */
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR);
  __HAL_ADC_ENABLE(&hadc1);

  /* USER CODE END ADC1_Init 2 */

}

static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
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
  hi2c2.Init.ClockSpeed = 100000;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart1_tx_busy = 0U;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    uart1_rx_events++;
    uart1_rx_bytes += Size;
    TruckReceiver_FeedFromIsr(&truck_receiver, uart1_dma_rx, Size);
    if (Uart1StartReceiveToIdle() != HAL_OK)
    {
      uart1_restart_requested = 1U;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart1_error_count++;
    uart1_rx_ready = 0U;
    uart1_tx_busy = 0U;
    uart1_restart_requested = 1U;
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
