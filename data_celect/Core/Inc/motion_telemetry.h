#ifndef MOTION_TELEMETRY_H
#define MOTION_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MOTION_TELEMETRY_SCHEMA_VERSION "stm32_control_telemetry.v2"

typedef enum
{
  MOTION_CONTROL_MODE_MANUAL = 1,
  MOTION_CONTROL_MODE_SAFE_ZERO = 3
} MotionControlMode;

typedef struct
{
  uint32_t control_seq;
  uint32_t control_stamp_ms;
  uint32_t sensor_seq;
  uint32_t sensor_stamp_ms;
  uint8_t sensor_is_new;
  uint32_t command_rx_seq;
  uint32_t command_source_stamp_ms;
  uint32_t command_received_stamp_ms;
  uint32_t command_age_ms;
  float command_action_boom;
  float command_action_stick;
  float command_action_bucket;
  float command_action_swing;

  float boom_pos_mm;
  float stick_pos_mm;
  float bucket_pos_mm;
  float boom_vel_mmps;
  float stick_vel_mmps;
  float bucket_vel_mmps;
  float boom_angle_deg;
  float arm_angle_deg;
  float bucket_angle_deg;
  float swing_angle_deg;
  float swing_vel_degps;

  float boom_v_ref_mmps;
  float stick_v_ref_mmps;
  float bucket_v_ref_mmps;
  float swing_v_ref_degps;
  float pid_out_boom;
  float pid_out_stick;
  float pid_out_bucket;
  float pid_out_swing;
  float valve_boom_deg;
  float valve_stick_deg;
  float valve_bucket_deg;
  float swing_percent;
  float pump_percent;
  uint16_t pwm_boom;
  uint16_t pwm_stick;
  uint16_t pwm_bucket;
  uint16_t pwm_swing;
  uint16_t pwm_pump;

  uint8_t control_mode;
  uint8_t homing_complete;
  uint8_t command_valid;
  uint8_t command_timed_out;
  uint8_t control_enabled;
  uint8_t estop;
  uint32_t limit_mask;
  uint8_t rs485_ok;
  uint8_t dwj_ok;
  uint8_t imu_ok;
  uint32_t fault_flags;
  uint32_t dropped_command_frames;
} MotionTelemetry;

int MotionTelemetry_BuildHeader(char *buffer, uint32_t buffer_size);
int MotionTelemetry_BuildRow(char *buffer, uint32_t buffer_size,
                             const MotionTelemetry *telemetry);

#ifdef __cplusplus
}
#endif

#endif
