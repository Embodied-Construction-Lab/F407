#include "gyro_bias_calibrator.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

static void assert_float_near(float actual, float expected, float tolerance)
{
  assert(actual > expected - tolerance);
  assert(actual < expected + tolerance);
}

static void add_stationary_window(GyroBiasCalibrator *calibrator,
                                  float bias_deg_s)
{
  uint64_t timestamp_ms;

  for (timestamp_ms = 1000U; timestamp_ms <= 4100U; timestamp_ms += 50U)
  {
    float noise_deg_s = ((timestamp_ms / 50U) % 2U == 0U) ? 0.001f : -0.001f;
    assert(GyroBiasCalibrator_AddSample(
        calibrator, bias_deg_s + noise_deg_s, timestamp_ms));
  }
}

static void test_stationary_bias_is_removed_without_masking_rotation(void)
{
  GyroBiasCalibrator calibrator;

  GyroBiasCalibrator_Init(&calibrator);
  add_stationary_window(&calibrator, 0.013f);

  assert(GyroBiasCalibrator_IsReady(&calibrator));
  assert_float_near(GyroBiasCalibrator_BiasDegS(&calibrator), 0.013f,
                    0.0002f);
  assert_float_near(GyroBiasCalibrator_Correct(&calibrator, 0.013f), 0.0f,
                    0.0002f);
  assert_float_near(GyroBiasCalibrator_Correct(&calibrator, 5.013f), 5.0f,
                    0.0002f);
  assert(GyroBiasCalibrator_AddSample(&calibrator, 5.013f, 4150U));
  assert(!GyroBiasCalibrator_AddSample(&calibrator, 5.013f, 4150U));
}

static void test_motion_resets_calibration_window(void)
{
  GyroBiasCalibrator calibrator;
  uint64_t timestamp_ms;

  GyroBiasCalibrator_Init(&calibrator);
  for (timestamp_ms = 1000U; timestamp_ms <= 4000U; timestamp_ms += 50U)
  {
    float moving_rate_deg_s =
        ((timestamp_ms / 50U) % 2U == 0U) ? 0.12f : -0.12f;
    (void)GyroBiasCalibrator_AddSample(&calibrator, moving_rate_deg_s,
                                       timestamp_ms);
  }
  assert(!GyroBiasCalibrator_IsReady(&calibrator));

  for (timestamp_ms = 4050U; timestamp_ms <= 7100U; timestamp_ms += 50U)
  {
    assert(GyroBiasCalibrator_AddSample(&calibrator, 0.013f,
                                        timestamp_ms));
  }
  assert(GyroBiasCalibrator_IsReady(&calibrator));
}

static void test_large_motion_and_bad_timestamp_are_rejected(void)
{
  GyroBiasCalibrator calibrator;

  GyroBiasCalibrator_Init(&calibrator);
  assert(GyroBiasCalibrator_AddSample(&calibrator, 0.01f, 1000U));
  assert(!GyroBiasCalibrator_AddSample(&calibrator, 1.0f, 1050U));
  assert(!GyroBiasCalibrator_IsReady(&calibrator));
  assert(GyroBiasCalibrator_AddSample(&calibrator, 0.01f, 1100U));
  assert(!GyroBiasCalibrator_AddSample(&calibrator, 0.01f, 1100U));
  assert(!GyroBiasCalibrator_IsReady(&calibrator));
}

static void test_invalid_inputs_and_unready_outputs_fail_closed(void)
{
  GyroBiasCalibrator calibrator;

  GyroBiasCalibrator_Init(NULL);
  GyroBiasCalibrator_Init(&calibrator);
  assert(!GyroBiasCalibrator_IsReady(NULL));
  assert_float_near(GyroBiasCalibrator_BiasDegS(&calibrator), 0.0f,
                    0.0001f);
  assert_float_near(GyroBiasCalibrator_Correct(&calibrator, 2.0f), 0.0f,
                    0.0001f);
  assert(!GyroBiasCalibrator_AddSample(NULL, 0.0f, 1000U));
  assert(!GyroBiasCalibrator_AddSample(&calibrator, NAN, 1000U));
}

static void test_stable_increasing_samples_calibrate(void)
{
  GyroBiasCalibrator calibrator;
  uint64_t timestamp_ms;

  GyroBiasCalibrator_Init(&calibrator);
  for (timestamp_ms = 1000U; timestamp_ms <= 4050U; timestamp_ms += 50U)
  {
    float ramp_deg_s = 0.005f + (float)(timestamp_ms - 1000U) * 0.000005f;
    assert(GyroBiasCalibrator_AddSample(&calibrator, ramp_deg_s,
                                        timestamp_ms));
  }
  assert(GyroBiasCalibrator_IsReady(&calibrator));
  assert_float_near(GyroBiasCalibrator_Correct(&calibrator, NAN), 0.0f,
                    0.0001f);
}

static void test_stationary_noise_spikes_do_not_block_calibration(void)
{
  GyroBiasCalibrator calibrator;
  uint64_t timestamp_ms;

  GyroBiasCalibrator_Init(&calibrator);
  for (timestamp_ms = 1000U; timestamp_ms <= 4050U; timestamp_ms += 50U)
  {
    float rate_deg_s = 0.013f;
    if (timestamp_ms == 1500U)
    {
      rate_deg_s = -0.15f;
    }
    else if (timestamp_ms == 2500U)
    {
      rate_deg_s = 0.16f;
    }
    assert(GyroBiasCalibrator_AddSample(&calibrator, rate_deg_s,
                                        timestamp_ms));
  }

  assert(GyroBiasCalibrator_IsReady(&calibrator));
  assert_float_near(GyroBiasCalibrator_BiasDegS(&calibrator), 0.0127f,
                    0.001f);
}

static void test_sustained_slow_rotation_is_not_learned_as_bias(void)
{
  GyroBiasCalibrator calibrator;
  uint64_t timestamp_ms;

  GyroBiasCalibrator_Init(&calibrator);
  for (timestamp_ms = 1000U; timestamp_ms <= 4050U; timestamp_ms += 50U)
  {
    (void)GyroBiasCalibrator_AddSample(&calibrator, 0.20f, timestamp_ms);
  }

  assert(!GyroBiasCalibrator_IsReady(&calibrator));
}

int main(void)
{
  test_stationary_bias_is_removed_without_masking_rotation();
  test_motion_resets_calibration_window();
  test_large_motion_and_bad_timestamp_are_rejected();
  test_invalid_inputs_and_unready_outputs_fail_closed();
  test_stable_increasing_samples_calibrate();
  test_stationary_noise_spikes_do_not_block_calibration();
  test_sustained_slow_rotation_is_not_learned_as_bias();
  return 0;
}
