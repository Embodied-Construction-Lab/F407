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
#include "pca9685.h"
#include "pwm_timing.h"
#include "truck_control.h"
#include "truck_receiver.h"

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define USART2_DMA_RX_SIZE 256U
#define STATUS_TX_BUFFER_SIZE 320U
#define CONTROL_STATUS_PERIOD_MS 100U
#define ADC_SAMPLE_PERIOD_MS 20U
#define ADC_REFERENCE_MV 3300U
#define ADC_FULL_SCALE_COUNT 4095U
#define STEERING_CENTER_MV 1630
#define STEERING_MIN_ANGLE_TENTHS (-300)
#define STEERING_MAX_ANGLE_TENTHS 300
#define OLED_UPDATE_PERIOD_MS 500U
#define OLED_RETRY_PERIOD_MS 1000U
#define OLED_LINE_SIZE 22U
#define TFT_ROW_UPDATE_PERIOD_MS 50U
#define TFT_LINE_CHARS 30U
#define TRUCK_FIRMWARE_ID "remote-zgt6-tft-1"

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
static TruckCommand applied_command;
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
static uint8_t tft_ready;
static uint8_t tft_next_row;
static uint8_t button_override_active;
static uint8_t button_override_up;
static uint8_t button_override_down;
static uint8_t last_rx_field_mask;
static uint8_t pca_pwm_cache_valid;
static uint8_t adc_conversion_active;
static uint8_t steering_feedback_valid;
static uint16_t pca_pwm_cache[TRUCK_OUTPUT_CHANNELS];
static uint16_t steering_adc_raw;
static uint16_t steering_voltage_mv;
static int16_t vehicle_steering_angle_tenths;
static uint32_t last_valid_control_tick;
static uint32_t button_override_tick;
static uint32_t last_periodic_status_tick;
static uint32_t valid_frame_count;
static uint32_t invalid_frame_count;
static uint32_t last_oled_update_tick;
static uint32_t last_oled_retry_tick;
static uint32_t last_tft_update_tick;
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
static void ProcessReceivedFrames(void);
static void ServiceButtonOverride(void);
static void ServiceEscState(void);
static void ServiceControlFailsafe(void);
static void ServiceUsart2Restart(void);
static void ServiceSteeringFeedback(uint32_t now_ms);
static void ServiceOled(uint32_t now_ms);
static void ServiceTft(uint32_t now_ms);

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

static void SendControlStatus(const char *status,
                              HAL_StatusTypeDef i2c_status)
{
  int length;

  if ((status == NULL) || (usart2_tx_busy != 0U))
  {
    return;
  }

  length = snprintf(
      status_tx_buffer, sizeof(status_tx_buffer),
      "{\"type\":\"truck\",\"status\":\"%s\",\"i2c\":%u,"
      "\"fw\":\"" TRUCK_FIRMWARE_ID "\","
      "\"rx_mask\":%u,\"rx_up\":%u,\"rx_down\":%u,"
      "\"up\":%u,\"down\":%u,\"button_override\":%u,"
      "\"steering_deg\":%d,\"throttle\":%d,\"brake\":%d,"
      "\"drive\":%d,\"lift\":%d,\"esc\":%u,"
      "\"pwm\":[%u,%u,%u,%u]}\r\n",
      status,
      (i2c_status == HAL_OK) ? 1U : 0U,
      (unsigned int)last_rx_field_mask,
      (unsigned int)latest_command.up,
      (unsigned int)latest_command.down,
      (unsigned int)applied_command.up,
      (unsigned int)applied_command.down,
      (unsigned int)button_override_active,
      (int)truck_outputs.steering_deg,
      (int)truck_outputs.throttle_percent,
      (int)truck_outputs.brake_percent,
      (int)truck_outputs.drive_percent,
      (int)truck_outputs.lift_percent,
      (unsigned int)drive_esc.state,
      (unsigned int)truck_outputs.pwm_count[0],
      (unsigned int)truck_outputs.pwm_count[1],
      (unsigned int)truck_outputs.pwm_count[2],
      (unsigned int)truck_outputs.pwm_count[3]);
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
  const uint8_t is_button_only =
      (((field_mask & analog_mask) == 0U) &&
       ((field_mask & button_mask) != 0U)) ? 1U : 0U;

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

  if (is_button_only != 0U)
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
      invalid_frame_count++;
      last_json_valid = 0U;
      received_invalid_frame = 1U;
      continue;
    }

    valid_frame_count++;
    last_json_valid = 1U;
    last_rx_field_mask = field_mask;
    now_ms = HAL_GetTick();
    ApplyCommandUpdate(&update, field_mask, now_ms);
    received_valid_frame = 1U;
  }

  if (received_valid_frame != 0U)
  {
    HAL_StatusTypeDef i2c_status;

    now_ms = HAL_GetTick();
    TruckControl_MapRawCommand(&applied_command, &requested_outputs);
    UpdateEscFilteredOutputs(now_ms);
    i2c_status = ApplyServoTargets(&truck_outputs);
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

static void ServiceButtonOverride(void)
{
  HAL_StatusTypeDef i2c_status;

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
  TruckControl_MapRawCommand(&applied_command, &requested_outputs);
  UpdateEscFilteredOutputs(HAL_GetTick());
  i2c_status = ApplyServoTargets(&truck_outputs);
  SendControlStatus("button_timeout", i2c_status);
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

static int32_t TruckAxisToMilli(float value)
{
  float scaled;

  if (value < -1.0f)
  {
    value = -1.0f;
  }
  else if (value > 1.0f)
  {
    value = 1.0f;
  }

  scaled = value * 1000.0f;
  return (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) :
                                          (scaled - 0.5f));
}

static void FormatAxis(char *buffer, size_t size, float value)
{
  const int32_t milli = TruckAxisToMilli(value);
  const uint32_t magnitude = (uint32_t)((milli < 0) ? -milli : milli);

  (void)snprintf(buffer, size, "%s%lu.%03lu",
                 (milli < 0) ? "-" : "",
                 (unsigned long)(magnitude / 1000U),
                 (unsigned long)(magnitude % 1000U));
}

static void OledDrawTelemetry(void)
{
  char line[OLED_LINE_SIZE];
  char axis0[10];
  char axis1[10];
  char axis2[10];
  char angle[8];
  uint16_t pulse_us;
  uint16_t angle_magnitude;

  FormatAxis(axis0, sizeof(axis0), applied_command.steering);
  if (steering_feedback_valid != 0U)
  {
    angle_magnitude = (uint16_t)
        ((vehicle_steering_angle_tenths < 0) ?
         -vehicle_steering_angle_tenths : vehicle_steering_angle_tenths);
    (void)snprintf(angle, sizeof(angle), "%c%u.%u",
                   (vehicle_steering_angle_tenths < 0) ? '-' : '+',
                   (unsigned int)(angle_magnitude / 10U),
                   (unsigned int)(angle_magnitude % 10U));
  }
  else
  {
    (void)snprintf(angle, sizeof(angle), "--.-");
  }
  (void)snprintf(line, sizeof(line), "A0:%.6s ANG:%.5s", axis0, angle);
  OledSsd1306_WriteText(0U, 0U, line);

  FormatAxis(axis1, sizeof(axis1), applied_command.throttle);
  FormatAxis(axis2, sizeof(axis2), applied_command.brake);
  (void)snprintf(line, sizeof(line), "A1:%.6s A2:%.6s", axis1, axis2);
  OledSsd1306_WriteText(1U, 0U, line);

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
  OledSsd1306_WriteText(2U, 0U, line);

  pulse_us = PwmTiming_DefaultCountToPulseUs(
      truck_outputs.pwm_count[TRUCK_CHANNEL_STEERING]);
  (void)snprintf(line, sizeof(line), "PW0:%uus",
                 (unsigned int)pulse_us);
  OledSsd1306_WriteText(3U, 0U, line);

  pulse_us = PwmTiming_DefaultCountToPulseUs(
      truck_outputs.pwm_count[TRUCK_CHANNEL_LIFT]);
  (void)snprintf(line, sizeof(line), "PW1:%uus",
                 (unsigned int)pulse_us);
  OledSsd1306_WriteText(4U, 0U, line);

  pulse_us = PwmTiming_DefaultCountToPulseUs(
      truck_outputs.pwm_count[TRUCK_CHANNEL_DRIVE]);
  (void)snprintf(line, sizeof(line), "PW2:%uus",
                 (unsigned int)pulse_us);
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

static void TftFormatSignedTenths(char *buffer, size_t size,
                                  int16_t value_tenths)
{
  uint16_t magnitude = (uint16_t)((value_tenths < 0) ?
                                  -value_tenths : value_tenths);

  (void)snprintf(buffer, size, "%c%u.%u",
                 (value_tenths < 0) ? '-' : '+',
                 (unsigned int)(magnitude / 10U),
                 (unsigned int)(magnitude % 10U));
}

static void TftDrawStatusRow(uint8_t row)
{
  char line[48];
  char command_angle[9];
  char feedback_angle[9];
  const char *control_state;
  uint32_t state_color;
  int16_t reverse_percent = 0;

  if (control_received == 0U)
  {
    control_state = "WAITING";
    state_color = LCD_YELLOW;
  }
  else if (control_failsafe_active != 0U)
  {
    control_state = "FAILSAFE";
    state_color = LCD_RED;
  }
  else
  {
    control_state = "APPLIED";
    state_color = LCD_GREEN;
  }

  if (truck_outputs.drive_percent < 0)
  {
    reverse_percent = (int16_t)-truck_outputs.drive_percent;
  }

  switch (row)
  {
    case 0U:
      (void)snprintf(line, sizeof(line), "REMOTE CTRL  %s",
                     control_state);
      TftWriteLine(row, state_color, line);
      break;

    case 1U:
      TftFormatSignedTenths(
          command_angle, sizeof(command_angle),
          (int16_t)((truck_outputs.steering_deg - 90) * 10));
      if (steering_feedback_valid != 0U)
      {
        TftFormatSignedTenths(feedback_angle, sizeof(feedback_angle),
                              vehicle_steering_angle_tenths);
      }
      else
      {
        (void)snprintf(feedback_angle, sizeof(feedback_angle), "--.-");
      }
      (void)snprintf(line, sizeof(line), "ANG C:%s F:%s",
                     command_angle, feedback_angle);
      TftWriteLine(row,
                   (steering_feedback_valid != 0U) ?
                   LCD_CYAN : LCD_YELLOW,
                   line);
      break;

    case 2U:
      (void)snprintf(line, sizeof(line), "THR:%d%%  REV:%d%%",
                     (int)truck_outputs.throttle_percent,
                     (int)reverse_percent);
      TftWriteLine(row, LCD_WHITE, line);
      break;

    case 3U:
      (void)snprintf(line, sizeof(line), "DRV:%+d%%  LIFT:%+d%%",
                     (int)truck_outputs.drive_percent,
                     (int)truck_outputs.lift_percent);
      TftWriteLine(row, LCD_WHITE, line);
      break;

    case 4U:
      (void)snprintf(
          line, sizeof(line), "PWM S:%u D:%u",
          (unsigned int)truck_outputs.pwm_count[TRUCK_CHANNEL_STEERING],
          (unsigned int)truck_outputs.pwm_count[TRUCK_CHANNEL_DRIVE]);
      TftWriteLine(row, LCD_YELLOW, line);
      break;

    case 5U:
      if (steering_feedback_valid != 0U)
      {
        (void)snprintf(
            line, sizeof(line), "PWM L:%u PA1:%u.%03uV",
            (unsigned int)truck_outputs.pwm_count[TRUCK_CHANNEL_LIFT],
            (unsigned int)(steering_voltage_mv / 1000U),
            (unsigned int)(steering_voltage_mv % 1000U));
      }
      else
      {
        (void)snprintf(
            line, sizeof(line), "PWM L:%u PA1:----V",
            (unsigned int)truck_outputs.pwm_count[TRUCK_CHANNEL_LIFT]);
      }
      TftWriteLine(row, LCD_YELLOW, line);
      break;

    case 6U:
      (void)snprintf(
          line, sizeof(line), "U2:%s PCA:%s I1:%s I2:%s",
          usart2_rx_ready ? "OK" : "NO",
          pca9685_ready ? "OK" : "NO",
          (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) ?
          "OK" : "NO",
          (HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) ?
          "OK" : "NO");
      TftWriteLine(
          row,
          ((usart2_rx_ready != 0U) && (pca9685_ready != 0U)) ?
          LCD_GREEN : LCD_RED,
          line);
      break;

    default:
      (void)snprintf(line, sizeof(line), "RX:%lu BAD:%lu ESC:%u",
                     (unsigned long)valid_frame_count,
                     (unsigned long)invalid_frame_count,
                     (unsigned int)drive_esc.state);
      TftWriteLine(row,
                   (last_json_valid != 0U) ? LCD_GREEN : LCD_YELLOW,
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
  TftDrawStatusRow(tft_next_row);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  TruckReceiver_Init(&truck_receiver);
  latest_command.steering = 0.0f;
  latest_command.throttle = TRUCK_PEDAL_RELEASED_RAW;
  latest_command.brake = TRUCK_PEDAL_RELEASED_RAW;
  latest_command.up = 0U;
  latest_command.down = 0U;
  BuildAppliedCommand();
  TruckControl_SetNeutral(&requested_outputs);
  TruckEsc_Init(&drive_esc);
  truck_outputs = requested_outputs;

  SPI_LCD_Init();
  tft_ready = 1U;
  tft_next_row = 0U;
  last_tft_update_tick = HAL_GetTick() - TFT_ROW_UPDATE_PERIOD_MS;

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
  SendControlStatus("ready", ApplyServoTargets(&truck_outputs));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ProcessReceivedFrames();
    ServiceButtonOverride();
    ServiceEscState();
    ServiceControlFailsafe();
    ServiceUsart2Restart();
    ServiceSteeringFeedback(HAL_GetTick());
    ServiceOled(HAL_GetTick());
    ServiceTft(HAL_GetTick());

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
