#ifndef GYRO_BIAS_CALIBRATOR_H
#define GYRO_BIAS_CALIBRATOR_H

#include <stdint.h>

#define GYRO_BIAS_CALIBRATION_DURATION_MS 3000U
#define GYRO_BIAS_CALIBRATION_MIN_SAMPLES 50U
#define GYRO_BIAS_STATIONARY_MAX_ABS_DEG_S 0.30f
#define GYRO_BIAS_STATIONARY_MAX_RANGE_DEG_S 0.08f

typedef struct
{
  double sample_sum_deg_s;
  float sample_min_deg_s;
  float sample_max_deg_s;
  float bias_deg_s;
  uint64_t window_start_ms;
  uint64_t last_sample_ms;
  uint32_t sample_count;
  uint8_t ready;
} GyroBiasCalibrator;

void GyroBiasCalibrator_Init(GyroBiasCalibrator *calibrator);
uint8_t GyroBiasCalibrator_AddSample(GyroBiasCalibrator *calibrator,
                                     float raw_rate_deg_s,
                                     uint64_t timestamp_ms);
uint8_t GyroBiasCalibrator_IsReady(const GyroBiasCalibrator *calibrator);
float GyroBiasCalibrator_BiasDegS(const GyroBiasCalibrator *calibrator);
float GyroBiasCalibrator_Correct(const GyroBiasCalibrator *calibrator,
                                 float raw_rate_deg_s);

#endif
