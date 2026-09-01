#include "gyro_bias_calibrator.h"

#include <math.h>
#include <stddef.h>

static void start_window(GyroBiasCalibrator *calibrator,
                         float raw_rate_deg_s,
                         uint64_t timestamp_ms)
{
  calibrator->sample_sum_deg_s = raw_rate_deg_s;
  calibrator->sample_square_sum_deg_s2 =
      (double)raw_rate_deg_s * raw_rate_deg_s;
  calibrator->window_start_ms = timestamp_ms;
  calibrator->last_sample_ms = timestamp_ms;
  calibrator->sample_count = 1U;
}

void GyroBiasCalibrator_Init(GyroBiasCalibrator *calibrator)
{
  if (calibrator == NULL)
  {
    return;
  }

  calibrator->sample_sum_deg_s = 0.0;
  calibrator->sample_square_sum_deg_s2 = 0.0;
  calibrator->bias_deg_s = 0.0f;
  calibrator->window_start_ms = 0U;
  calibrator->last_sample_ms = 0U;
  calibrator->sample_count = 0U;
  calibrator->ready = 0U;
}

uint8_t GyroBiasCalibrator_AddSample(GyroBiasCalibrator *calibrator,
                                     float raw_rate_deg_s,
                                     uint64_t timestamp_ms)
{
  uint64_t duration_ms;
  double mean_deg_s;
  double variance_deg_s2;

  if ((calibrator == NULL) || (isfinite(raw_rate_deg_s) == 0))
  {
    return 0U;
  }
  if (calibrator->ready != 0U)
  {
    if (timestamp_ms <= calibrator->last_sample_ms)
    {
      return 0U;
    }
    calibrator->last_sample_ms = timestamp_ms;
    return 1U;
  }
  if (fabsf(raw_rate_deg_s) > GYRO_BIAS_STATIONARY_MAX_ABS_DEG_S)
  {
    GyroBiasCalibrator_Init(calibrator);
    return 0U;
  }
  if (calibrator->sample_count == 0U)
  {
    start_window(calibrator, raw_rate_deg_s, timestamp_ms);
    return 1U;
  }
  if (timestamp_ms <= calibrator->last_sample_ms)
  {
    GyroBiasCalibrator_Init(calibrator);
    return 0U;
  }

  calibrator->sample_sum_deg_s += raw_rate_deg_s;
  calibrator->sample_square_sum_deg_s2 +=
      (double)raw_rate_deg_s * raw_rate_deg_s;
  calibrator->last_sample_ms = timestamp_ms;
  ++calibrator->sample_count;

  duration_ms = timestamp_ms - calibrator->window_start_ms;
  if (duration_ms < GYRO_BIAS_CALIBRATION_DURATION_MS)
  {
    return 1U;
  }

  mean_deg_s = calibrator->sample_sum_deg_s / calibrator->sample_count;
  variance_deg_s2 =
      calibrator->sample_square_sum_deg_s2 / calibrator->sample_count -
      mean_deg_s * mean_deg_s;
  if (variance_deg_s2 < 0.0)
  {
    variance_deg_s2 = 0.0;
  }
  if ((calibrator->sample_count < GYRO_BIAS_CALIBRATION_MIN_SAMPLES) ||
      (mean_deg_s > GYRO_BIAS_STATIONARY_MAX_MEAN_DEG_S) ||
      (mean_deg_s < -GYRO_BIAS_STATIONARY_MAX_MEAN_DEG_S) ||
      (variance_deg_s2 >
       (double)GYRO_BIAS_STATIONARY_MAX_STDDEV_DEG_S *
           GYRO_BIAS_STATIONARY_MAX_STDDEV_DEG_S))
  {
    GyroBiasCalibrator_Init(calibrator);
    return 0U;
  }

  calibrator->bias_deg_s = (float)mean_deg_s;
  calibrator->ready = 1U;
  return 1U;
}

uint8_t GyroBiasCalibrator_IsReady(const GyroBiasCalibrator *calibrator)
{
  return (uint8_t)((calibrator != NULL) && (calibrator->ready != 0U));
}

float GyroBiasCalibrator_BiasDegS(const GyroBiasCalibrator *calibrator)
{
  if (GyroBiasCalibrator_IsReady(calibrator) == 0U)
  {
    return 0.0f;
  }
  return calibrator->bias_deg_s;
}

float GyroBiasCalibrator_Correct(const GyroBiasCalibrator *calibrator,
                                 float raw_rate_deg_s)
{
  if ((GyroBiasCalibrator_IsReady(calibrator) == 0U) ||
      (isfinite(raw_rate_deg_s) == 0))
  {
    return 0.0f;
  }
  return raw_rate_deg_s - calibrator->bias_deg_s;
}
