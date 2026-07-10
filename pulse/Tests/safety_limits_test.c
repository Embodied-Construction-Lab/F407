#include "safety_limits.h"

#include <assert.h>

static void assert_float_equal(float actual, float expected)
{
  assert(actual > expected - 0.0001f);
  assert(actual < expected + 0.0001f);
}

static SafetyLimitsFeedback make_feedback(int32_t boom,
                                          int32_t stick,
                                          int32_t bucket,
                                          float yaw)
{
  SafetyLimitsFeedback feedback;

  feedback.boom_length_hundredths_mm = boom;
  feedback.stick_length_hundredths_mm = stick;
  feedback.bucket_length_hundredths_mm = bucket;
  feedback.yaw_deg = yaw;
  return feedback;
}

static void test_linear_axes_allow_motion_through_center(void)
{
  JoystickProfileSample sample;
  SafetyLimitsFeedback feedback;

  sample.x1 = 0.0f;
  sample.x2 = -0.4f;
  sample.y1 = -0.4f;
  sample.y2 = -0.4f;
  feedback = make_feedback(13000, 13000, 13000, 0.0f);

  SafetyLimits_Apply(&sample, &feedback);

  assert_float_equal(sample.x2, -0.4f);
  assert_float_equal(sample.y1, -0.4f);
  assert_float_equal(sample.y2, -0.4f);
}

static void test_linear_axes_stop_at_configured_limits(void)
{
  JoystickProfileSample sample;
  SafetyLimitsFeedback feedback;

  sample.x1 = 0.0f;
  sample.x2 = -0.4f;
  sample.y1 = -0.4f;
  sample.y2 = -0.4f;
  feedback = make_feedback(0, 0, 0, 0.0f);
  SafetyLimits_Apply(&sample, &feedback);
  assert_float_equal(sample.x2, 0.0f);
  assert_float_equal(sample.y1, 0.0f);
  assert_float_equal(sample.y2, 0.0f);

  sample.x2 = 0.4f;
  sample.y1 = 0.4f;
  sample.y2 = 0.4f;
  feedback = make_feedback(22000, 22000, 22000, 0.0f);
  SafetyLimits_Apply(&sample, &feedback);
  assert_float_equal(sample.x2, 0.0f);
  assert_float_equal(sample.y1, 0.0f);
  assert_float_equal(sample.y2, 0.0f);
}

int main(void)
{
  test_linear_axes_allow_motion_through_center();
  test_linear_axes_stop_at_configured_limits();
  return 0;
}
