#include "motion_telemetry.h"

#include <stdio.h>

int MotionTelemetry_BuildHeader(char *buffer, uint32_t buffer_size)
{
  int written;

  if ((buffer == 0) || (buffer_size == 0U))
  {
    return -1;
  }

  written = snprintf(buffer, buffer_size,
                     "t,s_boom,s_stick,s_bucket,"
                     "v_boom,v_stick,v_bucket,"
                     "a_boom,a_stick,a_bucket,yaw,yaw_rate,"
                     "deg_boom,deg_stick,deg_bucket,"
                     "swing_speed,pump_speed\n");
  if ((written < 0) || ((uint32_t)written >= buffer_size))
  {
    return -1;
  }

  return written;
}

int MotionTelemetry_BuildRow(char *buffer, uint32_t buffer_size,
                             const MotionTelemetry *telemetry)
{
  int written;

  if ((buffer == 0) || (telemetry == 0) || (buffer_size == 0U))
  {
    return -1;
  }

  written = snprintf(buffer, buffer_size,
                     "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                     "%.3f,%.3f,%.3f,%.3f,%.3f,"
                     "%.3f,%.3f,%.3f,%.3f,%.3f\n",
                     (unsigned long)telemetry->t_ms,
                     telemetry->s_boom,
                     telemetry->s_stick,
                     telemetry->s_bucket,
                     telemetry->v_boom,
                     telemetry->v_stick,
                     telemetry->v_bucket,
                     telemetry->a_boom,
                     telemetry->a_stick,
                     telemetry->a_bucket,
                     telemetry->yaw,
                     telemetry->yaw_rate,
                     telemetry->deg_boom,
                     telemetry->deg_stick,
                     telemetry->deg_bucket,
                     telemetry->swing_speed,
                     telemetry->pump_speed);
  if ((written < 0) || ((uint32_t)written >= buffer_size))
  {
    return -1;
  }

  return written;
}
