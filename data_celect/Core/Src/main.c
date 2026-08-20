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
#include "arm_angle.h"
#include "collection_timing.h"
#include "control_command.h"
#include "control_mode_supervisor.h"
#include "dwj_reader.h"
#include "ftepc_rs485.h"
#include "imu_oled_format.h"
#include "imu_parser.h"
#include "joystick_servo_map.h"
#include "motion_telemetry.h"
#include "oled_ssd1306.h"
#include "pca9685.h"
#include "stick_receiver.h"
#include "status_led.h"
#include "velocity_control.h"

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  RS485_IDLE = 0,
  RS485_TX_BUSY,
  RS485_RX_BUSY
} Rs485PollState;

typedef enum
{
  DWJ_IDLE = 0,
  DWJ_WAIT_BIG_ARM,
  DWJ_WAIT_SMALL_ARM
} DwjPollState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART2_DMA_RX_SIZE 512U
#define IMU_DMA_RX_SIZE 8192U
#define ENCODER485_SLAVE_ADDRESS 1U
#define ENCODER485_TIMEOUT_MS 50U
#define ENCODER485_REQUEST_LEN 8U
#define ENCODER485_RESPONSE_LEN 17U
#define ENCODER485_PULSE_START_REGISTER 0x0030U
#define ENCODER485_THREE_PULSE_REGISTERS 0x0006U
#define ENCODER485_FUNCTION_READ_HOLDING 0x03U
#define OLED_UPDATE_PERIOD_MS 500U
#define OLED_PAGE_PERIOD_MS 1500U
#define OLED_PAGE_COUNT 2U
#define OLED_LINE_SIZE 22U
#define TELEMETRY_BUFFER_SIZE 1024U
#define DWJ_POLL_PERIOD_MS 50U
#define TELEMETRY_FAULT_RS485 (1UL << 0)
#define TELEMETRY_FAULT_DWJ (1UL << 1)
#define TELEMETRY_FAULT_IMU (1UL << 2)
#define TELEMETRY_FAULT_PCA9685 (1UL << 3)
#define TELEMETRY_FAULT_COMMAND_TIMEOUT (1UL << 4)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
static uint8_t usart2_dma_rx[USART2_DMA_RX_SIZE];
static uint8_t imu_dma_rx[IMU_DMA_RX_SIZE];
static uint16_t imu_dma_last_pos;
static StickReceiver stick_receiver;
static ControlCommand latest_control_command;
static ControlModeSupervisor control_mode_supervisor;
static VelocityControl velocity_controller;
static VelocityControlOutput velocity_output;
static uint8_t velocity_feedback_ready;
static uint8_t control_command_active;
static volatile uint8_t usart2_restart_requested;
static StatusLedTimer status_led_timer;
static JoystickServoTargets servo_targets;
static uint32_t last_valid_control_tick;
static uint32_t last_control_tick;
static uint32_t control_seq;
static uint32_t command_rx_seq;
static uint32_t command_source_stamp_ms;
static uint32_t command_received_stamp_ms;
static uint32_t invalid_command_frames;
static uint8_t control_received;
static uint8_t control_failsafe_active;
static uint8_t pca9685_ready;
static uint8_t pca_output_ok;
static ImuParser imu_parser;
static ImuSample imu_latest;
static uint8_t imu_sample_valid;
static float imu_yaw_deg;
static float imu_yaw_rate_deg_s;
static uint64_t imu_last_ts_ms;
static uint8_t imu_yaw_time_valid;
static Ftepc485_Data encoder_data;
static Ftepc485_Status encoder_status = FTEPC485_ERROR_UART;
static int32_t encoder_last_pulse[FTEPC485_CHANNEL_COUNT];
static uint32_t encoder_last_sample_tick;
static uint8_t encoder_has_last_sample;
static int32_t small_arm_angle_hundredths_deg;
static uint8_t rs485_request[ENCODER485_REQUEST_LEN];
static uint8_t rs485_response[ENCODER485_RESPONSE_LEN];
static Rs485PollState rs485_state;
static uint32_t rs485_state_tick;
static uint32_t last_encoder_poll_tick;
static uint8_t oled_ready;
static uint8_t oled_page;
static uint8_t oled_refresh_page;
static uint32_t last_oled_update_tick;
static uint32_t last_oled_page_tick;
static MotionTelemetry motion_telemetry;
static char telemetry_tx_buffer[TELEMETRY_BUFFER_SIZE];
static uint8_t telemetry_header_sent;
static volatile uint8_t telemetry_tx_busy;
static uint32_t last_telemetry_tick;
static volatile uint32_t sensor_seq;
static volatile uint32_t sensor_stamp_ms;
static uint32_t last_sent_sensor_seq;
static DwjReadings dwj_readings;
static HAL_StatusTypeDef dwj_status = HAL_ERROR;
static DwjPollState dwj_state;
static uint32_t dwj_state_tick;
static uint32_t last_dwj_poll_tick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef USART2_StartReceiveToIdle(void);
static HAL_StatusTypeDef ApplyServoTargets(const JoystickServoTargets *targets);
static void EnterSafeZero(void);
static void ProcessReceivedFrames(void);
static void ServiceControl20Hz(void);
static void Imu_RxStart(void);
static void Imu_UpdateYawFromSample(void);
static void ServiceImu(void);
static void ServiceEncoder485(void);
static void ServiceDwjReader(void);
static void ServiceOled(void);
static void ServiceTelemetry(void);
static void Rs485_StartPoll(uint32_t now_ms);
static void Rs485_HandleResponse(uint32_t now_ms);
static void FormatFixedHundredths(int32_t value, char *buffer,
                                  size_t buffer_size);
static void FillMotionTelemetry(MotionTelemetry *telemetry, uint32_t now_ms);

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
    /* A complete frame is detected by IDLE or DMA transfer-complete. */
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }
  return status;
}

static HAL_StatusTypeDef ApplyServoTargets(const JoystickServoTargets *targets)
{
  uint8_t channel;

  if (targets == NULL)
  {
    return HAL_ERROR;
  }

  if (pca9685_ready == 0U)
  {
    if (Pca9685_Init(&hi2c1) != HAL_OK)
    {
      return HAL_ERROR;
    }
    pca9685_ready = 1U;
  }

  for (channel = 0U; channel < JOYSTICK_PCA_CHANNEL_COUNT; ++channel)
  {
    if (!JoystickServoMap_IsActiveChannel(channel))
    {
      continue;
    }
    if (Pca9685_SetPwm(&hi2c1, channel, 0U,
                       targets->pwm_count[channel]) != HAL_OK)
    {
      pca9685_ready = 0U;
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

static void EnterSafeZero(void)
{
  ControlModeSupervisor_EnterSafeZero(&control_mode_supervisor);
  VelocityControl_Reset(&velocity_controller);
  memset(&velocity_output, 0, sizeof(velocity_output));
  velocity_feedback_ready = 0U;
  control_command_active = 0U;
  control_failsafe_active = 0U;
}

static uint8_t ControlCommandIsValid(uint32_t now_ms)
{
  return ((control_command_active != 0U) &&
          !JoystickServoMap_IsTimedOut(now_ms, last_valid_control_tick))
             ? 1U
             : 0U;
}

static void ProcessReceivedFrames(void)
{
  char frame[STICK_FRAME_MAX_LEN];
  ControlCommand parsed_command;

  while (StickReceiver_Pop(&stick_receiver, frame, sizeof(frame)))
  {
    if (ControlCommand_ParseJson(frame, &parsed_command))
    {
      uint32_t now_ms = HAL_GetTick();
      ControlModeDecision decision;

      if (!StickReceiver_IsNewSequence(parsed_command.command_seq,
                                       command_rx_seq,
                                       control_received != 0U))
      {
        invalid_command_frames++;
        EnterSafeZero();
        continue;
      }

      decision = ControlModeSupervisor_Apply(&control_mode_supervisor,
                                             &parsed_command);
      if (!decision.accepted)
      {
        invalid_command_frames++;
        EnterSafeZero();
        continue;
      }

      if (decision.mode_changed)
      {
        VelocityControl_Reset(&velocity_controller);
        memset(&velocity_output, 0, sizeof(velocity_output));
      }
      latest_control_command = parsed_command;
      command_rx_seq = parsed_command.command_seq;
      command_source_stamp_ms = parsed_command.command_source_stamp_ms;
      command_received_stamp_ms = now_ms;
      last_valid_control_tick = now_ms;
      control_received = 1U;
      control_command_active = 1U;
    }
    else
    {
      invalid_command_frames++;
      EnterSafeZero();
    }
  }
}

static void ServiceControl20Hz(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint32_t elapsed_ms;
  HAL_StatusTypeDef i2c_status;
  uint8_t command_valid;

  if ((uint32_t)(now_ms - last_control_tick) <
      COLLECTION_CONTROL_PERIOD_MS)
  {
    return;
  }
  elapsed_ms = (uint32_t)(now_ms - last_control_tick);
  last_control_tick = now_ms;
  control_seq++;
  command_valid = ControlCommandIsValid(now_ms);
  velocity_feedback_ready = 0U;

  if ((command_valid != 0U) &&
      (control_mode_supervisor.active_mode == CONTROL_MODE_MANUAL_ACTION))
  {
    JoystickServoMap_Compute(latest_control_command.axis.swing,
                             latest_control_command.axis.bucket,
                             latest_control_command.axis.stick,
                             latest_control_command.axis.boom,
                             latest_control_command.manual_z1,
                             latest_control_command.manual_z2,
                             &servo_targets);
    i2c_status = ApplyServoTargets(&servo_targets);
    control_failsafe_active = 0U;
  }
  else if ((command_valid != 0U) &&
           (control_mode_supervisor.active_mode ==
            CONTROL_MODE_VELOCITY_REFERENCE) &&
           (encoder_status == FTEPC485_OK) &&
           (imu_sample_valid != 0U))
  {
    VelocityControlFeedback feedback;

    feedback.boom_length_hundredths_mm =
        encoder_data.length_hundredths_mm[0];
    feedback.stick_length_hundredths_mm =
        encoder_data.length_hundredths_mm[1];
    feedback.bucket_length_hundredths_mm =
        encoder_data.length_hundredths_mm[2];
    feedback.boom_speed_hundredths_mm_s =
        encoder_data.speed_hundredths_mm_s[0];
    feedback.stick_speed_hundredths_mm_s =
        encoder_data.speed_hundredths_mm_s[1];
    feedback.bucket_speed_hundredths_mm_s =
        encoder_data.speed_hundredths_mm_s[2];
    feedback.swing_angle_deg = imu_yaw_deg;
    feedback.swing_speed_deg_s = imu_yaw_rate_deg_s;

    if (VelocityControl_Update(&velocity_controller,
                               &latest_control_command.axis,
                               &feedback,
                               (float)elapsed_ms / 1000.0f,
                               &velocity_output))
    {
      JoystickServoMap_Compute(velocity_output.valve_action.swing,
                               velocity_output.valve_action.bucket,
                               velocity_output.valve_action.stick,
                               velocity_output.valve_action.boom,
                               0.0f, 0.0f, &servo_targets);
      i2c_status = ApplyServoTargets(&servo_targets);
      velocity_feedback_ready = 1U;
      control_failsafe_active = 0U;
    }
    else
    {
      VelocityControl_Reset(&velocity_controller);
      JoystickServoMap_SetNeutral(&servo_targets);
      i2c_status = ApplyServoTargets(&servo_targets);
      control_failsafe_active = 1U;
    }
  }
  else if (control_failsafe_active == 0U)
  {
    if (command_valid == 0U)
    {
      ControlModeSupervisor_EnterSafeZero(&control_mode_supervisor);
    }
    VelocityControl_Reset(&velocity_controller);
    memset(&velocity_output, 0, sizeof(velocity_output));
    JoystickServoMap_SetNeutral(&servo_targets);
    i2c_status = ApplyServoTargets(&servo_targets);
    control_failsafe_active = 1U;
  }
  else
  {
    i2c_status = (pca_output_ok != 0U) ? HAL_OK : HAL_ERROR;
  }
  pca_output_ok = (i2c_status == HAL_OK) ? 1U : 0U;
}

static void Imu_NormalizeYaw(void)
{
  while (imu_yaw_deg >= 360.0f)
  {
    imu_yaw_deg -= 360.0f;
  }

  while (imu_yaw_deg < 0.0f)
  {
    imu_yaw_deg += 360.0f;
  }
}

static void Imu_UpdateYawFromSample(void)
{
  imu_yaw_rate_deg_s = imu_latest.gz;

  if (imu_yaw_time_valid != 0U)
  {
    uint64_t elapsed_ms = imu_latest.ts_ms - imu_last_ts_ms;

    if ((imu_latest.ts_ms > imu_last_ts_ms) && (elapsed_ms < 1000U))
    {
      imu_yaw_deg += imu_yaw_rate_deg_s * ((float)elapsed_ms / 1000.0f);
      Imu_NormalizeYaw();
    }
  }

  imu_last_ts_ms = imu_latest.ts_ms;
  imu_yaw_time_valid = 1U;
}

static void Imu_RxStart(void)
{
  imu_sample_valid = 0U;
  imu_yaw_deg = 0.0f;
  imu_yaw_rate_deg_s = 0.0f;
  imu_last_ts_ms = 0U;
  imu_yaw_time_valid = 0U;
  imu_dma_last_pos = 0U;
  imu_parser_init(&imu_parser);

  if (HAL_UART_Receive_DMA(&huart1, imu_dma_rx, sizeof(imu_dma_rx)) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_TC);
  }
}

static void Imu_FeedBytes(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  if (imu_parser_feed(&imu_parser, data, length, &imu_latest) != 0U)
  {
    imu_sample_valid = 1U;
    Imu_UpdateYawFromSample();
  }
}

static void ServiceImu(void)
{
  uint16_t current_pos;

  if (huart1.hdmarx == NULL)
  {
    return;
  }

  current_pos = (uint16_t)(sizeof(imu_dma_rx) -
                           __HAL_DMA_GET_COUNTER(huart1.hdmarx));
  if (current_pos == imu_dma_last_pos)
  {
    return;
  }

  if (current_pos > imu_dma_last_pos)
  {
    Imu_FeedBytes(&imu_dma_rx[imu_dma_last_pos],
                  (uint16_t)(current_pos - imu_dma_last_pos));
  }
  else
  {
    Imu_FeedBytes(&imu_dma_rx[imu_dma_last_pos],
                  (uint16_t)(sizeof(imu_dma_rx) - imu_dma_last_pos));
    if (current_pos > 0U)
    {
      Imu_FeedBytes(imu_dma_rx, current_pos);
    }
  }

  imu_dma_last_pos = current_pos;
}

static void FormatFixedHundredths(int32_t value, char *buffer,
                                  size_t buffer_size)
{
  uint32_t magnitude;
  const char *sign;

  if (buffer_size == 0U)
  {
    return;
  }

  if (value < 0)
  {
    sign = "-";
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  }
  else
  {
    sign = "";
    magnitude = (uint32_t)value;
  }

  (void)snprintf(buffer, buffer_size, "%s%lu.%02lu", sign,
                 (unsigned long)(magnitude / 100U),
                 (unsigned long)(magnitude % 100U));
}

static void FormatFixedTenths(int32_t value, char *buffer, size_t buffer_size)
{
  uint32_t magnitude;
  const char *sign;

  if (buffer_size == 0U)
  {
    return;
  }

  if (value < 0)
  {
    sign = "-";
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  }
  else
  {
    sign = "";
    magnitude = (uint32_t)value;
  }

  (void)snprintf(buffer, buffer_size, "%s%lu.%01lu", sign,
                 (unsigned long)(magnitude / 10U),
                 (unsigned long)(magnitude % 10U));
}

static void OledWriteFixedRow(uint8_t row,
                              const char *prefix,
                              int32_t value,
                              const char *suffix)
{
  char value_text[18];
  char line[OLED_LINE_SIZE];

  FormatFixedHundredths(value, value_text, sizeof(value_text));
  (void)snprintf(line, sizeof(line), "%s%s%s",
                 prefix, value_text, suffix);
  OledSsd1306_WriteText(row, 0, line);
}

static void OledWriteTenthsRow(uint8_t row,
                               const char *prefix,
                               int32_t value,
                               const char *suffix)
{
  char value_text[18];
  char line[OLED_LINE_SIZE];

  FormatFixedTenths(value, value_text, sizeof(value_text));
  (void)snprintf(line, sizeof(line), "%s%s%s",
                 prefix, value_text, suffix);
  OledSsd1306_WriteText(row, 0, line);
}

static void OledWriteLengthSpeedPage(void)
{
  OledSsd1306_WriteText(0, 0, "LEN mm");
  OledWriteFixedRow(1U, "B:", encoder_data.length_hundredths_mm[0], "");
  OledWriteFixedRow(2U, "S:", encoder_data.length_hundredths_mm[1], "");
  OledWriteFixedRow(3U, "K:", encoder_data.length_hundredths_mm[2], "");
  OledSsd1306_WriteText(4U, 0, "SPD mm/s");
  OledWriteFixedRow(5U, "B:", encoder_data.speed_hundredths_mm_s[0], "");
  OledWriteFixedRow(6U, "S:", encoder_data.speed_hundredths_mm_s[1], "");
  OledWriteFixedRow(7U, "K:", encoder_data.speed_hundredths_mm_s[2], "");
}

static void OledWriteAngleImuPage(void)
{
  char line[OLED_LINE_SIZE];

  OledSsd1306_WriteText(0, 0, "ANG deg");
  OledWriteTenthsRow(1U, "B:", dwj_readings.big_arm.angle_tenths_deg, "");
  OledWriteFixedRow(2U, "S:", small_arm_angle_hundredths_deg, "");
  OledWriteTenthsRow(3U, "K:", dwj_readings.bucket.angle_tenths_deg, "");

  if (imu_sample_valid == 0U)
  {
    OledSsd1306_WriteText(4U, 0, "yaw: --");
    OledSsd1306_WriteText(5U, 0, "rate: --");
  }
  else
  {
    imu_oled_format_row(line, sizeof(line), "yaw", imu_yaw_deg);
    OledSsd1306_WriteText(4U, 0, line);
    imu_oled_format_row(line, sizeof(line), "rate", imu_yaw_rate_deg_s);
    OledSsd1306_WriteText(5U, 0, line);
  }

  (void)snprintf(line, sizeof(line), "R:%u A:%u I:%u",
                 (unsigned int)((encoder_status == FTEPC485_OK) ? 1U : 0U),
                 (unsigned int)((dwj_status == HAL_OK) ? 1U : 0U),
                 (unsigned int)((imu_sample_valid != 0U) ? 1U : 0U));
  OledSsd1306_WriteText(6U, 0, line);
  (void)snprintf(line, sizeof(line), "B:%ld K:%ldmV",
                 (long)dwj_readings.big_arm.mv,
                 (long)dwj_readings.bucket.mv);
  OledSsd1306_WriteText(7U, 0, line);
}

static void Rs485_BuildReadRequest(void)
{
  uint16_t crc;

  rs485_request[0] = ENCODER485_SLAVE_ADDRESS;
  rs485_request[1] = ENCODER485_FUNCTION_READ_HOLDING;
  rs485_request[2] = (uint8_t)(ENCODER485_PULSE_START_REGISTER >> 8);
  rs485_request[3] = (uint8_t)ENCODER485_PULSE_START_REGISTER;
  rs485_request[4] = (uint8_t)(ENCODER485_THREE_PULSE_REGISTERS >> 8);
  rs485_request[5] = (uint8_t)ENCODER485_THREE_PULSE_REGISTERS;
  crc = Ftepc485_Crc16(rs485_request, 6U);
  rs485_request[6] = (uint8_t)crc;
  rs485_request[7] = (uint8_t)(crc >> 8);
}

static void Rs485_StartPoll(uint32_t now_ms)
{
  if (rs485_state != RS485_IDLE)
  {
    return;
  }

  Rs485_BuildReadRequest();
  if (HAL_UART_Transmit_IT(&huart3, rs485_request,
                           sizeof(rs485_request)) == HAL_OK)
  {
    rs485_state = RS485_TX_BUSY;
    rs485_state_tick = now_ms;
  }
  else
  {
    encoder_status = FTEPC485_ERROR_UART;
  }
}

static void Rs485_ApplyPulses(const int32_t pulses[FTEPC485_CHANNEL_COUNT],
                              uint32_t now_ms)
{
  Ftepc485_ConvertPulses(pulses, &encoder_data);

  if (encoder_has_last_sample == 0U)
  {
    for (uint8_t channel = 0U; channel < FTEPC485_CHANNEL_COUNT; ++channel)
    {
      encoder_data.speed_hundredths_mm_s[channel] = 0;
      encoder_last_pulse[channel] = pulses[channel];
    }
    encoder_last_sample_tick = now_ms;
    encoder_has_last_sample = 1U;
  }
  else
  {
    uint32_t delta_ms = now_ms - encoder_last_sample_tick;
    if (delta_ms == 0U)
    {
      delta_ms = 1U;
    }

    for (uint8_t channel = 0U; channel < FTEPC485_CHANNEL_COUNT; ++channel)
    {
      int32_t delta_pulse = pulses[channel] - encoder_last_pulse[channel];
      encoder_data.speed_hundredths_mm_s[channel] =
          Ftepc485_DeltaToSpeedHundredthsMmS(channel,
                                             delta_pulse,
                                             delta_ms);
      encoder_last_pulse[channel] = pulses[channel];
    }
    encoder_last_sample_tick = now_ms;
  }

  small_arm_angle_hundredths_deg =
      ArmAngle_Channel2HundredthsDeg(encoder_data.length_hundredths_mm[1]);
}

static void Rs485_HandleResponse(uint32_t now_ms)
{
  int32_t pulses[FTEPC485_CHANNEL_COUNT];

  encoder_status = Ftepc485_ParseThreePulseResponse(
      rs485_response,
      sizeof(rs485_response),
      ENCODER485_SLAVE_ADDRESS,
      pulses);
  if (encoder_status == FTEPC485_OK)
  {
    Rs485_ApplyPulses(pulses, now_ms);
    sensor_stamp_ms = now_ms;
    sensor_seq++;
  }
}

static void ServiceEncoder485(void)
{
  uint32_t now_ms = HAL_GetTick();

  if ((rs485_state != RS485_IDLE) &&
      ((uint32_t)(now_ms - rs485_state_tick) >= ENCODER485_TIMEOUT_MS))
  {
    (void)HAL_UART_Abort(&huart3);
    rs485_state = RS485_IDLE;
    encoder_status = FTEPC485_ERROR_UART;
  }

  if ((uint32_t)(now_ms - last_encoder_poll_tick) >=
      COLLECTION_SENSOR_PERIOD_MS)
  {
    last_encoder_poll_tick = now_ms;
    Rs485_StartPoll(now_ms);
  }
}

static void ServiceDwjReader(void)
{
  uint32_t now_ms = HAL_GetTick();
  int16_t raw;
  int32_t mv;

  if (dwj_state == DWJ_IDLE)
  {
    if ((uint32_t)(now_ms - last_dwj_poll_tick) < DWJ_POLL_PERIOD_MS)
    {
      return;
    }

    last_dwj_poll_tick = now_ms;
    if (Dwj_StartConversion(&hi2c1, DWJ_ADS1115_CHANNEL_AIN0) == HAL_OK)
    {
      dwj_state = DWJ_WAIT_BIG_ARM;
      dwj_state_tick = now_ms;
    }
    else
    {
      dwj_status = HAL_ERROR;
    }
    return;
  }

  if ((uint32_t)(now_ms - dwj_state_tick) < DWJ_ADS1115_CONVERSION_DELAY_MS)
  {
    return;
  }

  if (dwj_state == DWJ_WAIT_BIG_ARM)
  {
    if (Dwj_ReadConversionRawMv(&hi2c1, &raw, &mv) != HAL_OK)
    {
      dwj_state = DWJ_IDLE;
      dwj_status = HAL_ERROR;
      return;
    }

    Dwj_UpdateBigArmReading(raw, mv, &dwj_readings.big_arm);
    if (Dwj_StartConversion(&hi2c1, DWJ_ADS1115_CHANNEL_AIN1) == HAL_OK)
    {
      dwj_state = DWJ_WAIT_SMALL_ARM;
      dwj_state_tick = now_ms;
    }
    else
    {
      dwj_state = DWJ_IDLE;
      dwj_status = HAL_ERROR;
    }
    return;
  }

  if (dwj_state == DWJ_WAIT_SMALL_ARM)
  {
    if (Dwj_ReadConversionRawMv(&hi2c1, &raw, &mv) == HAL_OK)
    {
      Dwj_UpdateBucketReading(raw, mv, &dwj_readings.bucket);
      dwj_status = HAL_OK;
    }
    else
    {
      dwj_status = HAL_ERROR;
    }
    dwj_state = DWJ_IDLE;
  }
}

static void ServiceOled(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (oled_ready == 0U)
  {
    return;
  }

  if ((uint32_t)(now_ms - last_oled_page_tick) >= OLED_PAGE_PERIOD_MS)
  {
    last_oled_page_tick = now_ms;
    oled_page = (uint8_t)((oled_page + 1U) % OLED_PAGE_COUNT);
  }

  if (oled_refresh_page >= OLED_SSD1306_ROWS)
  {
    if ((uint32_t)(now_ms - last_oled_update_tick) < OLED_UPDATE_PERIOD_MS)
    {
      return;
    }
    last_oled_update_tick = now_ms;

    OledSsd1306_Clear();
    if (oled_page == 0U)
    {
      OledWriteLengthSpeedPage();
    }
    else
    {
      OledWriteAngleImuPage();
    }
    oled_refresh_page = 0U;
  }

  if (OledSsd1306_UpdatePage(oled_refresh_page) != HAL_OK)
  {
    oled_ready = 0U;
    oled_refresh_page = OLED_SSD1306_ROWS;
    return;
  }
  oled_refresh_page++;
}

static void FillMotionTelemetry(MotionTelemetry *telemetry, uint32_t now_ms)
{
  ManualAction action;
  uint8_t command_valid;
  uint8_t command_timed_out;
  uint32_t fault_flags = 0U;

  if (telemetry == NULL)
  {
    return;
  }

  command_valid = ControlCommandIsValid(now_ms);
  /* Also marks a fail-closed rejected command so a new serial owner can
   * distinguish command_rx_seq history from the boot value and resume safely. */
  command_timed_out = ((control_received != 0U) && (command_valid == 0U))
                          ? 1U
                          : 0U;

  telemetry->control_seq = control_seq;
  telemetry->control_stamp_ms = last_control_tick;
  telemetry->sensor_seq = sensor_seq;
  telemetry->sensor_stamp_ms = sensor_stamp_ms;
  telemetry->sensor_is_new = (sensor_seq != last_sent_sensor_seq) ? 1U : 0U;
  telemetry->command_rx_seq = command_rx_seq;
  telemetry->command_source_stamp_ms = command_source_stamp_ms;
  telemetry->command_received_stamp_ms = command_received_stamp_ms;
  telemetry->command_age_ms = (control_received != 0U)
                                  ? (now_ms - command_received_stamp_ms)
                                  : 0U;
  if ((command_valid != 0U) &&
      (control_mode_supervisor.active_mode == CONTROL_MODE_MANUAL_ACTION))
  {
    ManualAction_FromStick(latest_control_command.axis.swing,
                           latest_control_command.axis.bucket,
                           latest_control_command.axis.stick,
                           latest_control_command.axis.boom, &action);
  }
  else
  {
    ManualAction_SetZero(&action);
  }
  telemetry->command_action_boom = action.boom;
  telemetry->command_action_stick = action.stick;
  telemetry->command_action_bucket = action.bucket;
  telemetry->command_action_swing = action.swing;

  telemetry->boom_pos_mm =
      (float)encoder_data.length_hundredths_mm[0] / 100.0f;
  telemetry->stick_pos_mm =
      (float)encoder_data.length_hundredths_mm[1] / 100.0f;
  telemetry->bucket_pos_mm =
      (float)encoder_data.length_hundredths_mm[2] / 100.0f;
  telemetry->boom_vel_mmps =
      (float)encoder_data.speed_hundredths_mm_s[0] / 100.0f;
  telemetry->stick_vel_mmps =
      (float)encoder_data.speed_hundredths_mm_s[1] / 100.0f;
  telemetry->bucket_vel_mmps =
      (float)encoder_data.speed_hundredths_mm_s[2] / 100.0f;
  telemetry->boom_angle_deg =
      (float)dwj_readings.big_arm.angle_tenths_deg / 10.0f;
  telemetry->arm_angle_deg =
      (float)small_arm_angle_hundredths_deg / 100.0f;
  telemetry->bucket_angle_deg =
      (float)dwj_readings.bucket.angle_tenths_deg / 10.0f;
  telemetry->swing_angle_deg = imu_yaw_deg;
  telemetry->swing_vel_degps = imu_yaw_rate_deg_s;

  if ((command_valid != 0U) &&
      (control_mode_supervisor.active_mode ==
       CONTROL_MODE_VELOCITY_REFERENCE))
  {
    telemetry->boom_v_ref_mmps = -velocity_output.target.boom;
    telemetry->stick_v_ref_mmps = -velocity_output.target.stick;
    telemetry->bucket_v_ref_mmps = velocity_output.target.bucket;
    telemetry->swing_v_ref_degps = velocity_output.target.swing;
    telemetry->pid_out_boom = velocity_output.valve_action.boom;
    telemetry->pid_out_stick = velocity_output.valve_action.stick;
    telemetry->pid_out_bucket = velocity_output.valve_action.bucket;
    telemetry->pid_out_swing = velocity_output.valve_action.swing;
  }
  else
  {
    telemetry->boom_v_ref_mmps = 0.0f;
    telemetry->stick_v_ref_mmps = 0.0f;
    telemetry->bucket_v_ref_mmps = 0.0f;
    telemetry->swing_v_ref_degps = 0.0f;
    telemetry->pid_out_boom = 0.0f;
    telemetry->pid_out_stick = 0.0f;
    telemetry->pid_out_bucket = 0.0f;
    telemetry->pid_out_swing = 0.0f;
  }
  telemetry->valve_boom_deg = servo_targets.big_arm_deg;
  telemetry->valve_stick_deg = servo_targets.small_arm_deg;
  telemetry->valve_bucket_deg = servo_targets.bucket_deg;
  telemetry->swing_percent = servo_targets.swing_percent;
  telemetry->pump_percent = servo_targets.pump_percent;
  telemetry->pwm_boom = servo_targets.pwm_count[JOYSTICK_CHANNEL_BIG_ARM];
  telemetry->pwm_stick = servo_targets.pwm_count[JOYSTICK_CHANNEL_SMALL_ARM];
  telemetry->pwm_bucket = servo_targets.pwm_count[JOYSTICK_CHANNEL_BUCKET];
  telemetry->pwm_swing = servo_targets.pwm_count[JOYSTICK_CHANNEL_SWING];
  telemetry->pwm_pump = servo_targets.pwm_count[JOYSTICK_CHANNEL_PUMP];

  if ((command_valid != 0U) &&
      (control_mode_supervisor.active_mode == CONTROL_MODE_MANUAL_ACTION))
  {
    telemetry->control_mode = MOTION_CONTROL_MODE_MANUAL;
  }
  else if ((command_valid != 0U) &&
           (control_mode_supervisor.active_mode ==
            CONTROL_MODE_VELOCITY_REFERENCE))
  {
    telemetry->control_mode = MOTION_CONTROL_MODE_VELOCITY;
  }
  else
  {
    telemetry->control_mode = MOTION_CONTROL_MODE_SAFE_ZERO;
  }
  telemetry->homing_complete = 1U;
  telemetry->command_valid = command_valid;
  telemetry->command_timed_out = command_timed_out;
  telemetry->control_enabled =
      ((command_valid != 0U) && (pca_output_ok != 0U) &&
       ((control_mode_supervisor.active_mode == CONTROL_MODE_MANUAL_ACTION) ||
        ((control_mode_supervisor.active_mode ==
          CONTROL_MODE_VELOCITY_REFERENCE) &&
         (velocity_feedback_ready != 0U))))
          ? 1U
          : 0U;
  telemetry->estop = 0U;
  telemetry->limit_mask =
      (control_mode_supervisor.active_mode ==
       CONTROL_MODE_VELOCITY_REFERENCE)
          ? velocity_output.limit_mask
          : 0U;
  telemetry->rs485_ok = (encoder_status == FTEPC485_OK) ? 1U : 0U;
  telemetry->dwj_ok = (dwj_status == HAL_OK) ? 1U : 0U;
  telemetry->imu_ok = (imu_sample_valid != 0U) ? 1U : 0U;

  if (telemetry->rs485_ok == 0U)
  {
    fault_flags |= TELEMETRY_FAULT_RS485;
  }
  if (telemetry->dwj_ok == 0U)
  {
    fault_flags |= TELEMETRY_FAULT_DWJ;
  }
  if (telemetry->imu_ok == 0U)
  {
    fault_flags |= TELEMETRY_FAULT_IMU;
  }
  if (pca_output_ok == 0U)
  {
    fault_flags |= TELEMETRY_FAULT_PCA9685;
  }
  if (command_timed_out != 0U)
  {
    fault_flags |= TELEMETRY_FAULT_COMMAND_TIMEOUT;
  }
  telemetry->fault_flags = fault_flags;
  telemetry->dropped_command_frames =
      stick_receiver.dropped_frames + invalid_command_frames;
}

static void ServiceTelemetry(void)
{
  int length;
  uint32_t now_ms;

  if (telemetry_tx_busy != 0U)
  {
    return;
  }

  if (telemetry_header_sent == 0U)
  {
    length = MotionTelemetry_BuildHeader(telemetry_tx_buffer,
                                         sizeof(telemetry_tx_buffer));
    if (length <= 0)
    {
      return;
    }

    if (HAL_UART_Transmit_IT(&huart2,
                             (uint8_t *)telemetry_tx_buffer,
                             (uint16_t)length) == HAL_OK)
    {
      telemetry_tx_busy = 1U;
      telemetry_header_sent = 1U;
    }
    return;
  }

  now_ms = HAL_GetTick();
  if ((uint32_t)(now_ms - last_telemetry_tick) <
      COLLECTION_TELEMETRY_PERIOD_MS)
  {
    return;
  }
  last_telemetry_tick = now_ms;

  FillMotionTelemetry(&motion_telemetry, now_ms);
  length = MotionTelemetry_BuildRow(telemetry_tx_buffer,
                                    sizeof(telemetry_tx_buffer),
                                    &motion_telemetry);
  if (length <= 0)
  {
    return;
  }

  if (HAL_UART_Transmit_IT(&huart2,
                           (uint8_t *)telemetry_tx_buffer,
                           (uint16_t)length) == HAL_OK)
  {
    telemetry_tx_busy = 1U;
    last_sent_sensor_seq = motion_telemetry.sensor_seq;
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  /* PA1 LED is active-low, so start from the off state. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  StatusLedTimer_Init(&status_led_timer, HAL_GetTick());
  StickReceiver_Init(&stick_receiver);
  ControlModeSupervisor_Init(&control_mode_supervisor);
  VelocityControl_Init(&velocity_controller);
  memset(&velocity_output, 0, sizeof(velocity_output));
  JoystickServoMap_SetNeutral(&servo_targets);
  pca_output_ok = (ApplyServoTargets(&servo_targets) == HAL_OK) ? 1U : 0U;
  control_failsafe_active = 1U;
  Imu_RxStart();
  oled_ready = (OledSsd1306_Init(&hi2c2) == HAL_OK) ? 1U : 0U;
  last_encoder_poll_tick = HAL_GetTick();
  last_oled_update_tick = HAL_GetTick();
  last_oled_page_tick = HAL_GetTick();
  oled_refresh_page = OLED_SSD1306_ROWS;
  last_telemetry_tick = HAL_GetTick();
  last_dwj_poll_tick = HAL_GetTick();
  last_control_tick = HAL_GetTick();
  if (USART2_StartReceiveToIdle() != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (StatusLedTimer_Poll(&status_led_timer, HAL_GetTick()))
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
    }

    ServiceImu();
    ProcessReceivedFrames();
    ServiceControl20Hz();
    ServiceEncoder485();
    ServiceDwjReader();
    ServiceImu();
    ServiceTelemetry();
    ServiceOled();

    if (usart2_restart_requested != 0U)
    {
      usart2_restart_requested = 0U;
      (void)HAL_UART_AbortReceive(&huart2);
      if (USART2_StartReceiveToIdle() != HAL_OK)
      {
        usart2_restart_requested = 1U;
      }
    }
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
  huart1.Init.BaudRate = 460800;
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
  huart2.Init.BaudRate = 460800;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    Rs485_HandleResponse(HAL_GetTick());
    rs485_state = RS485_IDLE;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    telemetry_tx_busy = 0U;
  }
  else if ((huart->Instance == USART3) && (rs485_state == RS485_TX_BUSY))
  {
    if (HAL_UART_Receive_IT(&huart3, rs485_response,
                            sizeof(rs485_response)) == HAL_OK)
    {
      rs485_state = RS485_RX_BUSY;
      rs485_state_tick = HAL_GetTick();
    }
    else
    {
      rs485_state = RS485_IDLE;
      encoder_status = FTEPC485_ERROR_UART;
    }
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2)
  {
    StickReceiver_FeedFromIsr(&stick_receiver, usart2_dma_rx, Size);
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
    telemetry_tx_busy = 0U;
    usart2_restart_requested = 1U;
  }
  else if (huart->Instance == USART1)
  {
    (void)HAL_UART_AbortReceive(&huart1);
    Imu_RxStart();
  }
  else if (huart->Instance == USART3)
  {
    (void)HAL_UART_Abort(&huart3);
    rs485_state = RS485_IDLE;
    encoder_status = FTEPC485_ERROR_UART;
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
