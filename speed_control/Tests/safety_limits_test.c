#include "safety_limits.h"

#include <assert.h>

static void assert_float_equal(float actual, float expected)
{
  float delta = actual - expected;

  if (delta < 0.0f)
  {
    delta = -delta;
  }
  assert(delta <= 0.0001f);
}

static void test_linear_axes_stop_only_outward_motion(void)
{
  JoystickProfileSample sample = {
      .x1 = 0.0f,
      .x2 = 0.5f,
      .y1 = 0.5f,
      .y2 = 0.5f,
  };
  SafetyLimitsFeedback feedback = {
      .boom_length_hundredths_mm = SAFETY_LIMIT_BOOM_MAX_HUNDREDTHS_MM,
      .stick_length_hundredths_mm = SAFETY_LIMIT_STICK_MAX_HUNDREDTHS_MM,
      .bucket_length_hundredths_mm = SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM,
      .yaw_deg = 0.0f,
  };

  SafetyLimits_Apply(&sample, &feedback);

  assert_float_equal(sample.x2,
                     (SAFETY_LIMIT_BUCKET_POSITIVE_INCREASES_LENGTH != 0U)
                         ? 0.0f
                         : 0.5f);
  assert_float_equal(sample.y1, 0.0f);
  assert_float_equal(sample.y2, 0.0f);

  sample.x2 = -0.5f;
  sample.y1 = -0.5f;
  sample.y2 = -0.5f;

  SafetyLimits_Apply(&sample, &feedback);

  assert_float_equal(sample.x2,
                     (SAFETY_LIMIT_BUCKET_POSITIVE_INCREASES_LENGTH != 0U)
                         ? -0.5f
                         : 0.0f);
  assert_float_equal(sample.y1, -0.5f);
  assert_float_equal(sample.y2, -0.5f);

  feedback.boom_length_hundredths_mm = SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM;
  feedback.stick_length_hundredths_mm = SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM;
  feedback.bucket_length_hundredths_mm = SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM;
  sample.x2 = -0.5f;
  sample.y1 = -0.5f;
  sample.y2 = -0.5f;

  SafetyLimits_Apply(&sample, &feedback);

  assert_float_equal(sample.x2,
                     (SAFETY_LIMIT_BUCKET_POSITIVE_INCREASES_LENGTH != 0U)
                         ? 0.0f
                         : -0.5f);
  assert_float_equal(sample.y1, 0.0f);
  assert_float_equal(sample.y2, 0.0f);
}

static void test_swing_axis_uses_signed_yaw_limits(void)
{
  JoystickProfileSample sample = {
      .x1 = 0.5f,
      .x2 = 0.0f,
      .y1 = 0.0f,
      .y2 = 0.0f,
  };
  SafetyLimitsFeedback feedback = {
      .boom_length_hundredths_mm = 11000,
      .stick_length_hundredths_mm = 11000,
      .bucket_length_hundredths_mm = 11000,
      .yaw_deg = 100.0f,
  };

  SafetyLimits_Apply(&sample, &feedback);
  assert_float_equal(sample.x1, 0.0f);

  sample.x1 = -0.5f;
  SafetyLimits_Apply(&sample, &feedback);
  assert_float_equal(sample.x1, -0.5f);

  feedback.yaw_deg = 260.0f; /* Normalizes to -100 deg. */
  sample.x1 = -0.5f;

  SafetyLimits_Apply(&sample, &feedback);
  assert_float_equal(sample.x1, 0.0f);

  sample.x1 = 0.5f;
  SafetyLimits_Apply(&sample, &feedback);
  assert_float_equal(sample.x1, 0.5f);
}

int main(void)
{
  test_linear_axes_stop_only_outward_motion();
  test_swing_axis_uses_signed_yaw_limits();
  return 0;
}
