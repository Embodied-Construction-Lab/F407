#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "collection_timing.h"
#include "control_command.h"
#include "control_mode_supervisor.h"
#include "dwj_reader.h"
#include "joystick_servo_map.h"
#include "motion_telemetry.h"
#include "safety_limits.h"
#include "stick_receiver.h"
#include "velocity_control.h"

typedef struct { uint32_t unused; } UART_HandleTypeDef;
typedef struct { uint32_t unused; } GPIO_TypeDef;
#include "ftepc_rs485.h"

static uint32_t tick, last_control_tick, control_seq, last_valid_control_tick;
static uint8_t control_command_active, velocity_feedback_ready;
static uint8_t control_failsafe_active, pca_output_ok, imu_sample_valid;
static float imu_yaw_deg, imu_yaw_rate_deg_s;
static ControlCommand latest_control_command;
static ControlModeSupervisor control_mode_supervisor;
static VelocityControl velocity_controller;
static VelocityControlOutput velocity_output;
static Ftepc485_Data encoder_data;
static Ftepc485_Status encoder_status;
static JoystickServoTargets servo_targets, applied_targets;
static unsigned applied_count;
static unsigned failed_writes_remaining;
static uint8_t control_received;
static uint32_t command_rx_seq, command_source_stamp_ms, command_received_stamp_ms;
static uint32_t sensor_seq, sensor_stamp_ms, last_sent_sensor_seq;
static uint32_t invalid_command_frames;
static int32_t small_arm_angle_hundredths_deg;
static DwjReadings dwj_readings;
static HAL_StatusTypeDef dwj_status;
static StickReceiver stick_receiver;

uint32_t HAL_GetTick(void) { return tick; }
static HAL_StatusTypeDef ApplyServoTargets(const JoystickServoTargets *targets)
{
  ++applied_count;
  if (failed_writes_remaining != 0U)
  {
    --failed_writes_remaining;
    return HAL_ERROR;
  }
  applied_targets = *targets;
  return HAL_OK;
}

/* The test compiles the active main.c service, not a second manual controller. */
#include "control_service_under_test.inc"

static void start_manual(void)
{
  memset(&latest_control_command, 0, sizeof(latest_control_command));
  StickReceiver_Init(&stick_receiver);
  control_received = 1U;
  command_rx_seq = 1U;
  invalid_command_frames = 0U;
  memset(&encoder_data, 0, sizeof(encoder_data));
  encoder_data.length_hundredths_mm[0] = 12000;
  encoder_data.length_hundredths_mm[1] = 12000;
  encoder_data.length_hundredths_mm[2] = 10000;
  encoder_status = FTEPC485_OK;
  imu_sample_valid = 1U;
  imu_yaw_deg = 0.0f;
  imu_yaw_rate_deg_s = 0.0f;
  control_mode_supervisor.active_mode = CONTROL_MODE_MANUAL_ACTION;
  latest_control_command.mode = CONTROL_MODE_MANUAL_ACTION;
  control_command_active = 1U;
  control_failsafe_active = 0U;
  pca_output_ok = 1U;
  applied_count = 0U;
  failed_writes_remaining = 0U;
  last_control_tick = 0U;
  last_valid_control_tick = 0U;
  tick = COLLECTION_CONTROL_PERIOD_MS;
  VelocityControl_Init(&velocity_controller);
}

static void assert_targets_equal(const JoystickServoTargets *expected)
{
  for (unsigned channel = 0; channel < JOYSTICK_PCA_CHANNEL_COUNT; ++channel)
  {
    assert(applied_targets.pwm_count[channel] == expected->pwm_count[channel]);
  }
  assert(applied_count != 0U);
}

static void assert_neutral(void)
{
  JoystickServoTargets expected;
  JoystickServoMap_SetNeutral(&expected);
  assert_targets_equal(&expected);
}

/* Test order is the authority's [boom, stick, bucket, swing]. */
static void set_axis(unsigned axis, float action)
{
  switch (axis)
  {
    case 0: latest_control_command.axis.boom = action; break;
    case 1: latest_control_command.axis.stick = action; break;
    case 2: latest_control_command.axis.bucket = action; break;
    default: latest_control_command.axis.swing = action; break;
  }
}

static void set_endpoint(unsigned axis, unsigned upper)
{
  switch (axis)
  {
    case 0:
      encoder_data.length_hundredths_mm[0] = upper ?
          SAFETY_LIMIT_BOOM_MAX_HUNDREDTHS_MM :
          SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM;
      break;
    case 1:
      encoder_data.length_hundredths_mm[1] = upper ?
          SAFETY_LIMIT_STICK_MAX_HUNDREDTHS_MM :
          SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM;
      break;
    case 2:
      encoder_data.length_hundredths_mm[2] = upper ?
          SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM :
          SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM;
      break;
    default:
      imu_yaw_deg = upper ? SAFETY_LIMIT_SWING_MAX_DEG :
          SAFETY_LIMIT_SWING_MIN_DEG;
      break;
  }
}

static void assert_original_mapping(void)
{
  JoystickServoTargets expected;
  JoystickServoMap_Compute(latest_control_command.axis.swing,
      latest_control_command.axis.bucket, latest_control_command.axis.stick,
      latest_control_command.axis.boom, latest_control_command.manual_z1,
      latest_control_command.manual_z2, &expected);
  assert_targets_equal(&expected);
}

static void receive_manual(float boom, uint32_t seq)
{
  char frame[STICK_FRAME_MAX_LEN];
  int length = snprintf(frame, sizeof(frame),
      "{\"schema_version\":\"stm32_manual_command.v1\","
      "\"X1\":0,\"Y1\":0,\"Z1\":0,\"X2\":0,\"Y2\":%.3f,\"Z2\":0,"
      "\"command_seq\":%u,\"command_source_stamp_ms\":%u}\n",
      (double)boom, (unsigned)seq, (unsigned)tick);
  assert(length > 0 && length < (int)sizeof(frame));
  StickReceiver_FeedFromIsr(&stick_receiver, (const uint8_t *)frame,
                             (uint16_t)length);
  ProcessReceivedFrames();
}

static void test_velocity_telemetry(void)
{
  for (int direction = -1; direction <= 1; ++direction)
  {
    MotionTelemetry telemetry;
    const float speed = direction * 0.01f;
    start_manual();
    control_mode_supervisor.active_mode = CONTROL_MODE_VELOCITY_REFERENCE;
    latest_control_command.mode = CONTROL_MODE_VELOCITY_REFERENCE;
    latest_control_command.axis = (ControlAxisCommand){speed, speed, speed,
                                                     direction * 0.1f};
    ServiceControl20Hz();
    FillMotionTelemetry(&telemetry, tick);
    assert(telemetry.control_enabled == 1U);
    assert(fabsf(telemetry.boom_v_ref_mmps - direction * 10.0f) < 0.001f);
    assert(fabsf(telemetry.stick_v_ref_mmps - direction * 10.0f) < 0.001f);
    assert(fabsf(telemetry.bucket_v_ref_mmps - direction * 10.0f) < 0.001f);
    assert(fabsf(telemetry.swing_v_ref_degps - direction * 5.729578f) < 0.001f);

    /* Report the permitted target after the physical travel limit, not input. */
    if (direction != 0)
    {
      for (unsigned axis = 0; axis < 4; ++axis)
      {
        set_endpoint(axis, direction > 0);
      }
      tick += COLLECTION_CONTROL_PERIOD_MS;
      ServiceControl20Hz();
      FillMotionTelemetry(&telemetry, tick);
      assert(telemetry.boom_v_ref_mmps == 0.0f);
      assert(telemetry.stick_v_ref_mmps == 0.0f);
      assert(telemetry.bucket_v_ref_mmps == 0.0f);
      assert(telemetry.swing_v_ref_degps == 0.0f);
      assert_neutral();
    }
  }
}

static void receive_velocity(float boom, uint32_t seq)
{
  char frame[STICK_FRAME_MAX_LEN];
  int length = snprintf(frame, sizeof(frame),
      "{\"schema_version\":\"stm32_velocity_command.v2\","
      "\"boom_mps\":%.3f,\"stick_mps\":0,\"bucket_mps\":0,\"swing_radps\":0,"
      "\"command_seq\":%u,\"command_source_stamp_ms\":%u}\n",
      (double)boom, (unsigned)seq, (unsigned)tick);
  assert(length > 0 && length < (int)sizeof(frame));
  StickReceiver_FeedFromIsr(&stick_receiver, (const uint8_t *)frame,
                           (uint16_t)length);
  ProcessReceivedFrames();
}

static void test_command_lifecycle(void)
{
  /* Both command modes require a fresh neutral handover. */
  for (unsigned mode = 0; mode < 2; ++mode)
  {
    MotionTelemetry telemetry;
    void (*receive)(float, uint32_t) = mode ? receive_velocity : receive_manual;
    const float demand = mode ? 0.01f : 0.6f;
    start_manual();
    ControlModeSupervisor_Init(&control_mode_supervisor);
    control_command_active = 0U;
    control_received = 0U;
    command_rx_seq = 0U;
    ServiceControl20Hz();
    FillMotionTelemetry(&telemetry, tick);
    assert_neutral();
    assert(telemetry.control_enabled == 0U);
    assert(telemetry.estop == 0U); /* Reserved wire field. */

    /* Startup cannot accept a nonzero command until the explicit zero handover. */
    receive(demand, 1U);
    assert(control_command_active == 0U);
    receive(0.0f, 1U);
    tick += COLLECTION_CONTROL_PERIOD_MS;
    ServiceControl20Hz();
    FillMotionTelemetry(&telemetry, tick);
    assert(telemetry.control_enabled == 1U);
    receive(demand, 2U);
    tick += COLLECTION_CONTROL_PERIOD_MS;
    ServiceControl20Hz();
    assert(applied_targets.pump_percent != 0.0f);

    /* A missed command deadline returns both modes to neutral. */
    tick = last_valid_control_tick + JOYSTICK_CONTROL_TIMEOUT_MS + 1U;
    ServiceControl20Hz();
    FillMotionTelemetry(&telemetry, tick);
    assert_neutral();
    assert(telemetry.control_enabled == 0U);
    assert(control_mode_supervisor.active_mode == CONTROL_MODE_SAFE_ZERO);
    receive(demand, 3U);
    assert(control_command_active == 0U);

    /* Model an MCU restart: volatile mode/command state is reset, not resumed. */
    ControlModeSupervisor_Init(&control_mode_supervisor);
    control_command_active = 0U;
    control_received = 0U;
    command_rx_seq = 0U;
    receive(demand, 100U);
    assert(control_command_active == 0U);
    assert(control_mode_supervisor.active_mode == CONTROL_MODE_SAFE_ZERO);
  }
}

static void test_failed_neutral_is_retried(void)
{
  for (unsigned mode = 0; mode < 2; ++mode)
  {
    MotionTelemetry telemetry;
    start_manual();
    if (mode != 0U)
    {
      control_mode_supervisor.active_mode = CONTROL_MODE_VELOCITY_REFERENCE;
      latest_control_command.mode = CONTROL_MODE_VELOCITY_REFERENCE;
    }
    latest_control_command.axis.boom = mode ? 0.01f : 0.6f;
    ServiceControl20Hz();
    assert(applied_targets.pump_percent != 0.0f);

    /* A failed I2C write leaves the PCA holding its preceding moving output. */
    tick = last_valid_control_tick + JOYSTICK_CONTROL_TIMEOUT_MS + 1U;
    failed_writes_remaining = 2U;
    ServiceControl20Hz();
    assert(pca_output_ok == 0U);
    assert(applied_targets.pump_percent != 0.0f);
    tick += COLLECTION_CONTROL_PERIOD_MS;
    ServiceControl20Hz();
    assert(pca_output_ok == 0U);
    tick += COLLECTION_CONTROL_PERIOD_MS;
    ServiceControl20Hz();
    assert_neutral();
    assert(pca_output_ok == 1U);
    FillMotionTelemetry(&telemetry, tick);
    assert(telemetry.control_enabled == 0U);
    assert(telemetry.control_mode == MOTION_CONTROL_MODE_SAFE_ZERO);

    /* Once neutral succeeds, a fault does not cause redundant I2C writes. */
    unsigned writes = applied_count;
    tick += COLLECTION_CONTROL_PERIOD_MS;
    ServiceControl20Hz();
    assert(applied_count == writes);
  }
}

int main(void)
{
  test_failed_neutral_is_retried();
  test_velocity_telemetry();

  /* At both endpoints, block only the outward direction and allow retreat. */
  for (unsigned axis = 0; axis < 4; ++axis)
  {
    for (unsigned upper = 0; upper < 2; ++upper)
    {
      float outward = (axis == 2 ? -0.6f : 0.6f) * (upper ? 1.0f : -1.0f);
      start_manual();
      set_endpoint(axis, upper);
      set_axis(axis, outward);
      ServiceControl20Hz();
      assert_neutral();
      set_axis(axis, -outward);
      tick += COLLECTION_CONTROL_PERIOD_MS;
      ServiceControl20Hz();
      assert_original_mapping();
    }
  }

  /* In-range four-axis and drive mapping remains unchanged. */
  start_manual();
  latest_control_command.axis = (ControlAxisCommand){0.3f, -0.4f, 0.5f, -0.6f};
  latest_control_command.manual_z1 = 0.2f;
  latest_control_command.manual_z2 = -0.3f;
  ServiceControl20Hz();
  assert_original_mapping();

  /* Losing either limit sensor while moving must replace output with neutral. */
  for (unsigned missing = 0; missing < 2; ++missing)
  {
    start_manual();
    latest_control_command.axis.boom = 0.6f;
    ServiceControl20Hz();
    assert(applied_targets.pump_percent != 0.0f);
    encoder_status = missing == 0 ? FTEPC485_ERROR_UART : FTEPC485_OK;
    imu_sample_valid = missing == 1 ? 0U : 1U;
    tick += COLLECTION_CONTROL_PERIOD_MS;
    ServiceControl20Hz();
    assert_neutral();
    assert(control_failsafe_active == 1U);
  }

  /* An explicit all-zero manual stop always reaches the servo output. */
  start_manual();
  encoder_status = FTEPC485_ERROR_UART;
  imu_sample_valid = 0U;
  control_failsafe_active = 1U;
  ServiceControl20Hz();
  assert_neutral();
  assert(control_failsafe_active == 0U);

  /* The mode's zero deadzone includes both drive inputs, including its edge. */
  const float zero_inputs[] = {-0.15f, -0.1f, 0.1f, 0.15f};
  for (unsigned sample = 0; sample < sizeof(zero_inputs) / sizeof(zero_inputs[0]);
       ++sample)
  {
    start_manual();
    encoder_status = FTEPC485_ERROR_UART;
    imu_sample_valid = 0U;
    control_failsafe_active = 1U;
    latest_control_command.axis = (ControlAxisCommand){
        zero_inputs[sample], zero_inputs[sample],
        zero_inputs[sample], zero_inputs[sample]};
    latest_control_command.manual_z1 = zero_inputs[sample];
    latest_control_command.manual_z2 = -zero_inputs[sample];
    assert(ControlCommand_IsZero(&latest_control_command));
    ServiceControl20Hz();
    assert_neutral();
    assert(control_failsafe_active == 0U);
  }

  /* A drive request outside that deadzone cannot bypass missing sensors. */
  start_manual();
  encoder_status = FTEPC485_ERROR_UART;
  imu_sample_valid = 0U;
  latest_control_command.manual_z1 = 0.15001f;
  ServiceControl20Hz();
  assert_neutral();
  assert(control_failsafe_active == 1U);

  test_command_lifecycle();
  puts("manual control service tests passed");
  return 0;
}
