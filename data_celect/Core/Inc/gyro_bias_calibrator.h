#ifndef GYRO_BIAS_CALIBRATOR_H
#define GYRO_BIAS_CALIBRATOR_H

#include <stdint.h>

/* Startup requires an explicitly stationary machine. These are noise-window
   criteria, not a deadband: after readiness every finite rate is corrected. */
#define GYRO_BIAS_CALIBRATION_DURATION_MS 10000U
#define GYRO_BIAS_CALIBRATION_MIN_SAMPLES 100U
#define GYRO_BIAS_MAX_SAMPLE_GAP_MS 500U
#define GYRO_BIAS_STATIONARY_MAX_ABS_DEG_S 0.30f
#define GYRO_BIAS_STATIONARY_MAX_STDDEV_DEG_S 0.08f
#define GYRO_BIAS_STATIONARY_MAX_MEAN_DEG_S 0.10f

typedef struct
{
  double weighted_sum_deg_ms;
  double weighted_square_sum_deg2_ms;
  uint64_t window_start_ms;
  uint64_t last_sample_ms;
  uint64_t weight_ms;
  uint32_t sample_count;
  float bias_deg_s;
  uint8_t ready;
} GyroBiasCalibrator;

void GyroBiasCalibrator_Init(GyroBiasCalibrator *calibrator);
void GyroBiasCalibrator_ResetPending(GyroBiasCalibrator *calibrator);
uint8_t GyroBiasCalibrator_AddSample(GyroBiasCalibrator *calibrator,
                                    float raw_rate_deg_s, uint64_t timestamp_ms);
uint8_t GyroBiasCalibrator_IsReady(const GyroBiasCalibrator *calibrator);
float GyroBiasCalibrator_BiasDegS(const GyroBiasCalibrator *calibrator);
float GyroBiasCalibrator_Correct(const GyroBiasCalibrator *calibrator,
                                float raw_rate_deg_s);

#endif
