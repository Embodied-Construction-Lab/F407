#include "motion_telemetry.h"

#include <stdio.h>

int MotionTelemetry_BuildHeader(char *buffer, uint32_t buffer_size)
{
  int written;

  if ((buffer == 0) || (buffer_size == 0U))
  {
    return -1;
  }

  written = snprintf(
      buffer, buffer_size,
      "schema_version,control_seq,control_stamp_ms,"
      "sensor_seq,sensor_stamp_ms,sensor_is_new,"
      "command_rx_seq,command_source_stamp_ms,"
      "command_received_stamp_ms,command_age_ms,"
      "boom_pos_mm,stick_pos_mm,bucket_pos_mm,"
      "boom_vel_mmps,stick_vel_mmps,bucket_vel_mmps,"
      "boom_angle_deg,arm_angle_deg,bucket_angle_deg,"
      "swing_angle_deg,swing_vel_degps,"
      "boom_v_ref_mmps,stick_v_ref_mmps,bucket_v_ref_mmps,"
      "swing_v_ref_degps,pid_out_boom,pid_out_stick,"
      "pid_out_bucket,pid_out_swing,valve_boom_deg,"
      "valve_stick_deg,valve_bucket_deg,swing_percent,pump_percent,"
      "pwm_boom,pwm_stick,pwm_bucket,pwm_swing,pwm_pump,"
      "control_mode,homing_complete,command_valid,command_timed_out,"
      "control_enabled,estop,limit_mask,rs485_ok,dwj_ok,imu_ok,"
      "fault_flags,dropped_command_frames\n");
  if ((written < 0) || ((uint32_t)written >= buffer_size))
  {
    return -1;
  }

  return written;
}

int MotionTelemetry_BuildRow(char *buffer, uint32_t buffer_size,
                             const MotionTelemetry *t)
{
  int written;

  if ((buffer == 0) || (t == 0) || (buffer_size == 0U))
  {
    return -1;
  }

  written = snprintf(
      buffer, buffer_size,
      "%s,%lu,%lu,%lu,%lu,%u,%lu,%lu,%lu,%lu,"
      "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
      "%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,"
      "%.3f,%.3f,%.3f,%.3f,%.3f,"
      "%u,%u,%u,%u,%u,"
      "%u,%u,%u,%u,%u,%u,%lu,%u,%u,%u,%lu,%lu\n",
      MOTION_TELEMETRY_SCHEMA_VERSION,
      (unsigned long)t->control_seq,
      (unsigned long)t->control_stamp_ms,
      (unsigned long)t->sensor_seq,
      (unsigned long)t->sensor_stamp_ms,
      (unsigned int)t->sensor_is_new,
      (unsigned long)t->command_rx_seq,
      (unsigned long)t->command_source_stamp_ms,
      (unsigned long)t->command_received_stamp_ms,
      (unsigned long)t->command_age_ms,
      (double)t->boom_pos_mm,
      (double)t->stick_pos_mm,
      (double)t->bucket_pos_mm,
      (double)t->boom_vel_mmps,
      (double)t->stick_vel_mmps,
      (double)t->bucket_vel_mmps,
      (double)t->boom_angle_deg,
      (double)t->arm_angle_deg,
      (double)t->bucket_angle_deg,
      (double)t->swing_angle_deg,
      (double)t->swing_vel_degps,
      (double)t->boom_v_ref_mmps,
      (double)t->stick_v_ref_mmps,
      (double)t->bucket_v_ref_mmps,
      (double)t->swing_v_ref_degps,
      (double)t->pid_out_boom,
      (double)t->pid_out_stick,
      (double)t->pid_out_bucket,
      (double)t->pid_out_swing,
      (double)t->valve_boom_deg,
      (double)t->valve_stick_deg,
      (double)t->valve_bucket_deg,
      (double)t->swing_percent,
      (double)t->pump_percent,
      (unsigned int)t->pwm_boom,
      (unsigned int)t->pwm_stick,
      (unsigned int)t->pwm_bucket,
      (unsigned int)t->pwm_swing,
      (unsigned int)t->pwm_pump,
      (unsigned int)t->control_mode,
      (unsigned int)t->homing_complete,
      (unsigned int)t->command_valid,
      (unsigned int)t->command_timed_out,
      (unsigned int)t->control_enabled,
      (unsigned int)t->estop,
      (unsigned long)t->limit_mask,
      (unsigned int)t->rs485_ok,
      (unsigned int)t->dwj_ok,
      (unsigned int)t->imu_ok,
      (unsigned long)t->fault_flags,
      (unsigned long)t->dropped_command_frames);
  if ((written < 0) || ((uint32_t)written >= buffer_size))
  {
    return -1;
  }

  return written;
}
