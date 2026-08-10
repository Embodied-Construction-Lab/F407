#ifndef MOTION_TELEMETRY_H
#define MOTION_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  uint32_t t_ms;
  float s_boom;
  float s_stick;
  float s_bucket;
  float v_boom;
  float v_stick;
  float v_bucket;
  float a_boom;
  float a_stick;
  float a_bucket;
  float yaw;
  float yaw_rate;
  float deg_boom;
  float deg_stick;
  float deg_bucket;
  float swing_speed;
  float pump_speed;
  uint8_t rs485_ok;
  uint8_t adc_ok;
  uint8_t imu_ok;
} MotionTelemetry;

int MotionTelemetry_BuildHeader(char *buffer, uint32_t buffer_size);
int MotionTelemetry_BuildRow(char *buffer, uint32_t buffer_size,
                             const MotionTelemetry *telemetry);

#ifdef __cplusplus
}
#endif

#endif
