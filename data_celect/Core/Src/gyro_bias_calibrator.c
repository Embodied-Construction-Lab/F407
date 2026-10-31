#include "gyro_bias_calibrator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

void GyroBiasCalibrator_Init(GyroBiasCalibrator *calibrator)
{
  if (calibrator != NULL)
  {
    memset(calibrator, 0, sizeof(*calibrator));
  }
}

void GyroBiasCalibrator_ResetPending(GyroBiasCalibrator *calibrator)
{
  if ((calibrator != NULL) && (calibrator->ready == 0U))
  {
    GyroBiasCalibrator_Init(calibrator);
  }
}

static void start_window(GyroBiasCalibrator *calibrator, uint64_t timestamp_ms)
{
  GyroBiasCalibrator_Init(calibrator);
  calibrator->window_start_ms = timestamp_ms;
  calibrator->last_sample_ms = timestamp_ms;
  calibrator->sample_count = 1U;
}

uint8_t GyroBiasCalibrator_AddSample(GyroBiasCalibrator *calibrator,
                                    float raw_rate_deg_s, uint64_t timestamp_ms)
{
  uint64_t delta_ms;
  double mean;
  double variance;
  if ((calibrator == NULL) || !isfinite(raw_rate_deg_s))
  {
    GyroBiasCalibrator_ResetPending(calibrator);
    return 0U;
  }
  if (calibrator->ready != 0U)
  {
    /* A sensor restart fences one sample, then accepts its new timebase.
       It never clears the learned bias or permanently rejects lower stamps. */
    uint8_t advancing = timestamp_ms > calibrator->last_sample_ms;
    calibrator->last_sample_ms = timestamp_ms;
    return advancing;
  }
  if (fabsf(raw_rate_deg_s) > GYRO_BIAS_STATIONARY_MAX_ABS_DEG_S)
  {
    GyroBiasCalibrator_ResetPending(calibrator);
    return 0U;
  }
  if (calibrator->sample_count == 0U)
  {
    start_window(calibrator, timestamp_ms);
    return 1U;
  }
  if ((timestamp_ms <= calibrator->last_sample_ms) ||
      (timestamp_ms - calibrator->last_sample_ms >= GYRO_BIAS_MAX_SAMPLE_GAP_MS))
  {
    start_window(calibrator, timestamp_ms);
    return 0U;
  }
  delta_ms = timestamp_ms - calibrator->last_sample_ms;
  calibrator->last_sample_ms = timestamp_ms;
  calibrator->weight_ms += delta_ms;
  ++calibrator->sample_count;
  /* Match the right-endpoint rate*dt rule used by heading integration. */
  calibrator->weighted_sum_deg_ms += (double)raw_rate_deg_s * delta_ms;
  calibrator->weighted_square_sum_deg2_ms +=
      (double)raw_rate_deg_s * raw_rate_deg_s * delta_ms;
  if (calibrator->weight_ms < GYRO_BIAS_CALIBRATION_DURATION_MS)
  {
    return 1U;
  }
  mean = calibrator->weighted_sum_deg_ms / calibrator->weight_ms;
  variance = calibrator->weighted_square_sum_deg2_ms / calibrator->weight_ms - mean * mean;
  if (variance < 0.0) { variance = 0.0; }
  if ((calibrator->sample_count < GYRO_BIAS_CALIBRATION_MIN_SAMPLES) ||
      (fabs(mean) > GYRO_BIAS_STATIONARY_MAX_MEAN_DEG_S) ||
      (variance > (double)GYRO_BIAS_STATIONARY_MAX_STDDEV_DEG_S *
                      GYRO_BIAS_STATIONARY_MAX_STDDEV_DEG_S))
  {
    start_window(calibrator, timestamp_ms);
    return 0U;
  }
  calibrator->bias_deg_s = (float)mean;
  calibrator->ready = 1U;
  return 1U;
}

uint8_t GyroBiasCalibrator_IsReady(const GyroBiasCalibrator *calibrator)
{
  return (uint8_t)((calibrator != NULL) && (calibrator->ready != 0U));
}

float GyroBiasCalibrator_BiasDegS(const GyroBiasCalibrator *calibrator)
{
  return GyroBiasCalibrator_IsReady(calibrator) ? calibrator->bias_deg_s : 0.0f;
}

float GyroBiasCalibrator_Correct(const GyroBiasCalibrator *calibrator, float rate)
{
  return (GyroBiasCalibrator_IsReady(calibrator) && isfinite(rate))
             ? rate - calibrator->bias_deg_s : 0.0f;
}
