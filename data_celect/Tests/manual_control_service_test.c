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
static uint32_t manual_limit_mask;
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

static void set_manual_endpoint(unsigned axis, unsigned upper)
{
  set_endpoint(axis, upper);
  if (axis == 1 && !upper)
  {
    encoder_data.length_hundredths_mm[1] = MANUAL_LIMIT_STICK_MIN_HUNDREDTHS_MM;
  }
  if (axis == 2)
  {
    encoder_data.length_hundredths_mm[2] = upper ?
        MANUAL_LIMIT_BUCKET_MAX_HUNDREDTHS_MM : MANUAL_LIMIT_BUCKET_MIN_HUNDREDTHS_MM;
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

static void receive_velocity_reference(float boom, float bucket, uint32_t seq)
{
  char frame[STICK_FRAME_MAX_LEN];
  int length = snprintf(frame, sizeof(frame),
      "{\"schema_version\":\"stm32_velocity_command.v2\","
      "\"boom_mps\":%.3f,\"stick_mps\":0,\"bucket_mps\":%.6f,\"swing_radps\":0,"
      "\"command_seq\":%u,\"command_source_stamp_ms\":%u}\n",
      (double)boom, (double)bucket, (unsigned)seq, (unsigned)tick);
  assert(length > 0 && length < (int)sizeof(frame));
  StickReceiver_FeedFromIsr(&stick_receiver, (const uint8_t *)frame,
                           (uint16_t)length);
  ProcessReceivedFrames();
}

static void receive_velocity(float boom, uint32_t seq)
{
  receive_velocity_reference(boom, 0.0f, seq);
}

static void test_act_to_rl_bucket_handoff(void)
{
  /* Raw positions and commands from the two 2026-11-07 CarryMaterial runs. */
  const int32_t positions[] = {17985, 17493, 17078, 16580};
  for (unsigned index = 0; index < sizeof(positions) / sizeof(positions[0]); ++index)
  {
    MotionTelemetry telemetry;
    start_manual();
    encoder_data.length_hundredths_mm[2] = positions[index];
    latest_control_command.axis.bucket = -0.162f;
    ServiceControl20Hz();
    assert_original_mapping();
    assert(manual_limit_mask == 0U);
    assert(applied_targets.bucket_deg < JOYSTICK_VALVE_NEUTRAL_DEG);

    receive_velocity(0.0f, 2U);
    tick += COLLECTION_CONTROL_PERIOD_MS;
    ServiceControl20Hz();
    assert_neutral();
    for (unsigned braking = 0; braking < 2; ++braking)
    {
      /* A slow retract target with faster measured retraction requests an
         extension-direction braking output; both limit layers must permit it. */
      const float demand = braking ? -0.002442f : 0.0342f;
      encoder_data.speed_hundredths_mm_s[2] = braking ? -2225 : 0;
      receive_velocity_reference(0.0f, demand, 3U + braking);
      tick += COLLECTION_CONTROL_PERIOD_MS;
      ServiceControl20Hz();
      FillMotionTelemetry(&telemetry, tick);
      assert(telemetry.control_mode == MOTION_CONTROL_MODE_VELOCITY);
      assert(telemetry.control_enabled == 1U);
      assert(fabsf(telemetry.bucket_v_ref_mmps - demand * 1000.0f) < 0.001f);
      assert(telemetry.limit_mask == 0U);
      assert(velocity_output.valve_action.bucket < 0.0f);
      assert(applied_targets.bucket_deg < JOYSTICK_VALVE_NEUTRAL_DEG);
      assert(applied_targets.pump_percent == JOYSTICK_FIXED_PUMP_PERCENT);
    }
    receive_velocity(0.0f, 5U);
    tick += COLLECTION_CONTROL_PERIOD_MS;
    ServiceControl20Hz();
    assert_neutral();
  }
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

static void test_small_velocity_reaches_pwm_and_keeps_limits(void)
{
  const unsigned channels[] = {JOYSTICK_CHANNEL_BIG_ARM, JOYSTICK_CHANNEL_SMALL_ARM,
                              JOYSTICK_CHANNEL_BUCKET, JOYSTICK_CHANNEL_SWING};
  JoystickServoTargets neutral;
  JoystickServoMap_SetNeutral(&neutral);
  for (unsigned axis = 0; axis < 4; ++axis)
  {
    for (int direction = -1; direction <= 1; direction += 2)
    {
      const float speed = direction * (axis == 3 ? 0.02f : 0.001f);
      start_manual();
      control_mode_supervisor.active_mode = CONTROL_MODE_VELOCITY_REFERENCE;
      latest_control_command.mode = CONTROL_MODE_VELOCITY_REFERENCE;
      set_axis(axis, speed);
      ServiceControl20Hz();
      /* Exercise the actual main.c -> PID -> limits -> mapping -> PWM path. */
      assert(applied_targets.pwm_count[channels[axis]] != neutral.pwm_count[channels[axis]]);
      const int valve_direction = direction * (axis == 2 ? -1 : 1);
      assert(((int)applied_targets.pwm_count[channels[axis]] -
              (int)neutral.pwm_count[channels[axis]]) * valve_direction > 0);
      assert(applied_targets.pump_percent == (axis == 3 ? 0.0f : JOYSTICK_FIXED_PUMP_PERCENT));
      assert(applied_targets.left_drive_percent == 0.0f);
      assert(applied_targets.right_drive_percent == 0.0f);

      set_endpoint(axis, direction > 0);
      tick += COLLECTION_CONTROL_PERIOD_MS;
      ServiceControl20Hz();
      assert_neutral();
      set_axis(axis, -speed);
      tick += COLLECTION_CONTROL_PERIOD_MS;
      ServiceControl20Hz();
      assert(applied_targets.pwm_count[channels[axis]] != neutral.pwm_count[channels[axis]]);

      /* Explicit zero is neutral even with residual measured motion. */
      encoder_data.speed_hundredths_mm_s[0] = 1000;
      encoder_data.speed_hundredths_mm_s[1] = -1000;
      encoder_data.speed_hundredths_mm_s[2] = 1000;
      imu_yaw_rate_deg_s = 10.0f;
      set_axis(axis, 0.0f);
      tick += COLLECTION_CONTROL_PERIOD_MS;
      ServiceControl20Hz();
      assert_neutral();
    }
  }
}

static void test_feedback_reversal_still_respects_output_limits(void)
{
  for (int direction = -1; direction <= 1; direction += 2)
  {
    start_manual();
    control_mode_supervisor.active_mode = CONTROL_MODE_VELOCITY_REFERENCE;
    latest_control_command.mode = CONTROL_MODE_VELOCITY_REFERENCE;
    latest_control_command.axis = (ControlAxisCommand){direction * .001f,
        direction * .001f, direction * .001f, direction * .02f};
    encoder_data.speed_hundredths_mm_s[0] = direction * 10000;
    encoder_data.speed_hundredths_mm_s[1] = direction * 10000;
    encoder_data.speed_hundredths_mm_s[2] = direction * 10000;
    imu_yaw_rate_deg_s = direction * 100.0f;
    ServiceControl20Hz();
    assert(velocity_output.valve_action.boom * direction < 0.0f);
    assert(velocity_output.valve_action.stick * direction < 0.0f);
    assert(velocity_output.valve_action.bucket * direction > 0.0f);
    assert(velocity_output.valve_action.swing * direction < 0.0f);
    /* Target is inward, but large measured speed makes PID request outward. */
    for (unsigned axis = 0; axis < 4; ++axis)
    {
      set_endpoint(axis, direction < 0);
    }
    tick += COLLECTION_CONTROL_PERIOD_MS;
    ServiceControl20Hz();
    assert(velocity_output.target.boom != 0.0f);
    assert(velocity_output.target.stick != 0.0f);
    assert(velocity_output.target.bucket != 0.0f);
    assert(velocity_output.target.swing != 0.0f);
    assert(velocity_output.limit_mask == 15U);
    assert_neutral();
  }
}

/* Only the RL bucket upper bound is aligned with the manual/ACT envelope. */
_Static_assert(SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM == 8000, "RL boom min changed");
_Static_assert(SAFETY_LIMIT_BOOM_MAX_HUNDREDTHS_MM == 19000, "RL boom max changed");
_Static_assert(SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM == 6000, "RL stick min changed");
_Static_assert(SAFETY_LIMIT_STICK_MAX_HUNDREDTHS_MM == 22000, "RL stick max changed");
_Static_assert(SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM == 6000, "RL bucket min changed");
_Static_assert(SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM == 18000, "RL bucket max changed");
_Static_assert(MANUAL_LIMIT_STICK_MIN_HUNDREDTHS_MM == 4700, "manual stick min changed");
_Static_assert(MANUAL_LIMIT_BUCKET_MIN_HUNDREDTHS_MM == 4800, "manual bucket min changed");
_Static_assert(MANUAL_LIMIT_BUCKET_MAX_HUNDREDTHS_MM == 18000, "manual bucket max changed");

_Static_assert(SAFETY_LIMIT_MASK_BOOM == VELOCITY_LIMIT_BOOM, "boom mask mismatch");
_Static_assert(SAFETY_LIMIT_MASK_STICK == VELOCITY_LIMIT_STICK, "stick mask mismatch");
_Static_assert(SAFETY_LIMIT_MASK_BUCKET == VELOCITY_LIMIT_BUCKET, "bucket mask mismatch");
_Static_assert(SAFETY_LIMIT_MASK_SWING == VELOCITY_LIMIT_SWING, "swing mask mismatch");

static void test_manual_limit_telemetry(void)
{
  MotionTelemetry telemetry;
  start_manual();
  /* At the historical manual bucket upper bound, block only extension. */
  encoder_data.length_hundredths_mm[2] = 18000;
  latest_control_command.axis = (ControlAxisCommand){0.218f, 0.0f, -0.162f, 0.0f};
  ServiceControl20Hz();
  FillMotionTelemetry(&telemetry, tick);
  assert(telemetry.control_mode == MOTION_CONTROL_MODE_MANUAL);
  assert(telemetry.control_enabled == 1U);
  assert(telemetry.command_action_bucket < 0.0f); /* Before position clipping. */
  assert(telemetry.valve_bucket_deg == JOYSTICK_VALVE_NEUTRAL_DEG);
  assert(telemetry.valve_boom_deg > JOYSTICK_VALVE_NEUTRAL_DEG);
  assert(telemetry.limit_mask == VELOCITY_LIMIT_BUCKET);

  /* A command toward the allowed range clears the bit and still moves. */
  latest_control_command.axis = (ControlAxisCommand){0.0f, 0.0f, 0.2f, 0.0f};
  tick += COLLECTION_CONTROL_PERIOD_MS;
  ServiceControl20Hz();
  FillMotionTelemetry(&telemetry, tick);
  assert(telemetry.limit_mask == 0U);
  assert(telemetry.valve_bucket_deg != JOYSTICK_VALVE_NEUTRAL_DEG);

  latest_control_command.axis.bucket = -0.2f;
  tick += COLLECTION_CONTROL_PERIOD_MS;
  ServiceControl20Hz();
  FillMotionTelemetry(&telemetry, tick);
  assert(telemetry.limit_mask == VELOCITY_LIMIT_BUCKET);
  /* Explicit neutral and a lost sensor must not leak the previous mask. */
  latest_control_command.axis.bucket = 0.0f;
  tick += COLLECTION_CONTROL_PERIOD_MS;
  ServiceControl20Hz();
  FillMotionTelemetry(&telemetry, tick);
  assert(telemetry.limit_mask == 0U);
  assert_neutral();
  latest_control_command.axis.bucket = -0.2f;
  tick += COLLECTION_CONTROL_PERIOD_MS;
  ServiceControl20Hz();
  encoder_status = FTEPC485_ERROR_UART;
  tick += COLLECTION_CONTROL_PERIOD_MS;
  ServiceControl20Hz();
  FillMotionTelemetry(&telemetry, tick);
  assert(telemetry.limit_mask == 0U);
  assert_neutral();

  /* All clipped axes can be reported together, with the existing wire bits. */
  start_manual();
  for (unsigned axis = 0; axis < 4; ++axis)
  {
    set_manual_endpoint(axis, 1U);
    set_axis(axis, axis == 2 ? -0.6f : 0.6f);
  }
  ServiceControl20Hz();
  FillMotionTelemetry(&telemetry, tick);
  assert(telemetry.limit_mask == 15U);
  assert_neutral();
  /* Zero handovers clear an old manual mask before the next control tick. */
  receive_velocity(0.0f, 2U);
  receive_manual(0.0f, 3U);
  FillMotionTelemetry(&telemetry, tick);
  assert(telemetry.control_mode == MOTION_CONTROL_MODE_MANUAL);
  assert(telemetry.limit_mask == 0U);
  for (unsigned axis = 0; axis < 4; ++axis)
  {
    set_axis(axis, axis == 2 ? -0.6f : 0.6f);
  }
  tick += COLLECTION_CONTROL_PERIOD_MS;
  ServiceControl20Hz();
  FillMotionTelemetry(&telemetry, tick);
  assert(telemetry.limit_mask == 15U);
  tick = last_valid_control_tick + JOYSTICK_CONTROL_TIMEOUT_MS + 1U;
  ServiceControl20Hz();
  FillMotionTelemetry(&telemetry, tick);
  assert(telemetry.control_mode == MOTION_CONTROL_MODE_SAFE_ZERO);
  assert(telemetry.limit_mask == 0U);
}

static void test_manual_act_within_historical_range(void)
{
  MotionTelemetry telemetry;
  start_manual();
  /* Actual ACT input is inside the user-selected historical manual range. */
  encoder_data.length_hundredths_mm[2] = 16255;
  latest_control_command.axis = (ControlAxisCommand){0.218f, 0.0f, -0.162f, 0.0f};
  ServiceControl20Hz();
  FillMotionTelemetry(&telemetry, tick);
  assert(telemetry.control_mode == MOTION_CONTROL_MODE_MANUAL);
  assert(telemetry.control_enabled == 1U);
  assert(telemetry.limit_mask == 0U);
  assert(telemetry.valve_bucket_deg < JOYSTICK_VALVE_NEUTRAL_DEG);
  assert_original_mapping();

  /* Velocity mode also permits extension below the shared 180 mm upper bound. */
  start_manual();
  encoder_data.length_hundredths_mm[2] = 16255;
  control_mode_supervisor.active_mode = CONTROL_MODE_VELOCITY_REFERENCE;
  latest_control_command.mode = CONTROL_MODE_VELOCITY_REFERENCE;
  latest_control_command.axis.bucket = 0.001f;
  ServiceControl20Hz();
  FillMotionTelemetry(&telemetry, tick);
  assert(telemetry.limit_mask == 0U);
  assert(fabsf(telemetry.bucket_v_ref_mmps - 1.0f) < 0.001f);
  assert(applied_targets.bucket_deg < JOYSTICK_VALVE_NEUTRAL_DEG);
}

static void test_manual_lower_extension_does_not_change_rl(void)
{
  for (unsigned axis = 1; axis <= 2; ++axis)
  {
    MotionTelemetry telemetry;
    start_manual();
    encoder_data.length_hundredths_mm[axis] = 5000;
    /* 50 mm is inside manual's 47/48 mm range, outside RL's 60 mm range. */
    set_axis(axis, axis == 1 ? -0.2f : 0.2f);
    ServiceControl20Hz();
    FillMotionTelemetry(&telemetry, tick);
    assert(telemetry.limit_mask == 0U);
    assert(applied_targets.pump_percent != 0.0f);
    assert_original_mapping();

    start_manual();
    encoder_data.length_hundredths_mm[axis] = 5000;
    control_mode_supervisor.active_mode = CONTROL_MODE_VELOCITY_REFERENCE;
    latest_control_command.mode = CONTROL_MODE_VELOCITY_REFERENCE;
    set_axis(axis, -0.001f);
    ServiceControl20Hz();
    FillMotionTelemetry(&telemetry, tick);
    assert(telemetry.limit_mask == (1UL << axis));
    assert_neutral();
  }
}

static void test_bucket_upper_bound_in_both_modes(void)
{
  const int32_t positions[] = {17999, 18000, 18001};
  for (unsigned velocity = 0; velocity < 2; ++velocity)
  {
    for (unsigned index = 0; index < sizeof(positions) / sizeof(positions[0]); ++index)
    {
      /* Extend, return, and (velocity only) brake fast retraction. */
      for (unsigned scenario = 0; scenario < (velocity ? 3U : 2U); ++scenario)
      {
        MotionTelemetry telemetry;
        const unsigned blocked = positions[index] >= 18000 && scenario != 1;
        const float speed = scenario == 0 ? 0.0342f : -0.002442f;
        start_manual();
        encoder_data.length_hundredths_mm[2] = positions[index];
        if (velocity)
        {
          control_mode_supervisor.active_mode = CONTROL_MODE_VELOCITY_REFERENCE;
          latest_control_command.mode = CONTROL_MODE_VELOCITY_REFERENCE;
          encoder_data.speed_hundredths_mm_s[2] = scenario == 2 ? -2225 : 0;
        }
        latest_control_command.axis.bucket = velocity ? speed :
            (scenario == 0 ? -0.6f : 0.6f);
        ServiceControl20Hz();
        FillMotionTelemetry(&telemetry, tick);
        assert(telemetry.limit_mask == (blocked ? SAFETY_LIMIT_MASK_BUCKET : 0U));
        if (velocity)
        {
          const float reference = blocked && scenario == 0 ? 0.0f : speed * 1000.0f;
          assert(fabsf(telemetry.bucket_v_ref_mmps - reference) < 0.001f);
        }
        if (blocked)
        {
          assert_neutral();
        }
        else
        {
          const int direction = scenario == 1 ? 1 : -1;
          assert((applied_targets.bucket_deg - JOYSTICK_VALVE_NEUTRAL_DEG) * direction > 0.0f);
        }
      }
    }
  }
}

static void test_swing_140_degree_envelope_in_both_modes(void)
{
  /* Test deployed yaw values, not macros: the enlarged working interval must
     apply to ACT/manual and velocity commands through the production service. */
  const struct { float yaw; unsigned block_positive; unsigned block_negative; } cases[] = {
      {0.0f, 0, 0}, {100.0f, 0, 0}, {-100.0f, 0, 0},
      {120.0f, 0, 0}, {-120.0f, 0, 0},
      {139.9f, 0, 0}, {140.0f, 1, 0}, {140.1f, 1, 0},
      {-139.9f, 0, 0}, {-140.0f, 0, 1}, {-140.1f, 0, 1},
      {219.9f, 0, 1}, {220.0f, 0, 1}, {220.1f, 0, 0},
      {179.9f, 1, 0}, {180.0f, 1, 0}, {180.1f, 0, 1},
      {-179.9f, 0, 1}, {-180.0f, 0, 1}, {-180.1f, 1, 0},
      {359.9f, 0, 0}, {360.0f, 0, 0}};
  for (unsigned velocity = 0; velocity < 2; ++velocity)
  {
    for (unsigned index = 0; index < sizeof(cases) / sizeof(cases[0]); ++index)
    {
      for (int direction = -1; direction <= 1; direction += 2)
      {
        MotionTelemetry telemetry;
        const unsigned blocked = direction > 0 ? cases[index].block_positive :
                                                cases[index].block_negative;
        start_manual();
        if (velocity)
        {
          control_mode_supervisor.active_mode = CONTROL_MODE_VELOCITY_REFERENCE;
          latest_control_command.mode = CONTROL_MODE_VELOCITY_REFERENCE;
        }
        imu_yaw_deg = cases[index].yaw;
        latest_control_command.axis.swing = direction * (velocity ? 0.1f : 0.5f);
        ServiceControl20Hz();
        FillMotionTelemetry(&telemetry, tick);
        assert(telemetry.limit_mask == (blocked ? SAFETY_LIMIT_MASK_SWING : 0U));
        if (blocked)
        {
          assert_neutral();
        }
        else
        {
          assert(applied_targets.swing_percent * direction > 0.0f);
        }
      }
    }
  }
}

int main(void)
{
  test_act_to_rl_bucket_handoff();
  test_bucket_upper_bound_in_both_modes();
  test_swing_140_degree_envelope_in_both_modes();
  test_manual_lower_extension_does_not_change_rl();
  test_manual_act_within_historical_range();
  test_manual_limit_telemetry();
  test_small_velocity_reaches_pwm_and_keeps_limits();
  test_feedback_reversal_still_respects_output_limits();
  test_failed_neutral_is_retried();
  test_velocity_telemetry();

  /* At both endpoints, block only the outward direction and allow retreat. */
  for (unsigned axis = 0; axis < 4; ++axis)
  {
    for (unsigned upper = 0; upper < 2; ++upper)
    {
      float outward = (axis == 2 ? -0.6f : 0.6f) * (upper ? 1.0f : -1.0f);
      start_manual();
      set_manual_endpoint(axis, upper);
      set_axis(axis, outward);
      ServiceControl20Hz();
      assert_neutral();
      MotionTelemetry telemetry;
      FillMotionTelemetry(&telemetry, tick);
      assert(telemetry.limit_mask == (1UL << axis));
      set_axis(axis, -outward);
      tick += COLLECTION_CONTROL_PERIOD_MS;
      ServiceControl20Hz();
      assert_original_mapping();
      FillMotionTelemetry(&telemetry, tick);
      assert(telemetry.limit_mask == 0U);
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
