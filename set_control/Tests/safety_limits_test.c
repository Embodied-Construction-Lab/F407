#include "safety_limits.h"

#include <assert.h>

static void assert_float_equal(float actual, float expected)
{
  assert(actual > expected - 0.0001f);
  assert(actual < expected + 0.0001f);
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

  assert_float_equal(sample.x2, 0.0f);
  assert_float_equal(sample.y1, 0.0f);
  assert_float_equal(sample.y2, 0.0f);

  sample.x2 = -0.5f;
  sample.y1 = -0.5f;
  sample.y2 = -0.5f;

  SafetyLimits_Apply(&sample, &feedback);

  assert_float_equal(sample.x2, -0.5f);
  assert_float_equal(sample.y1, -0.5f);
  assert_float_equal(sample.y2, -0.5f);

  feedback.boom_length_hundredths_mm = SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM;
  feedback.stick_length_hundredths_mm = SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM;
  feedback.bucket_length_hundredths_mm = SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM;
  sample.x2 = -0.5f;
  sample.y1 = -0.5f;
  sample.y2 = -0.5f;

  SafetyLimits_Apply(&sample, &feedback);

  assert_float_equal(sample.x2, 0.0f);
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

static void test_profile_feedback_reports_selected_axis_limits(void)
{
  SafetyLimitsFeedback safety_feedback = {
      .boom_length_hundredths_mm = SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM,
      .stick_length_hundredths_mm = 11000,
      .bucket_length_hundredths_mm = SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM,
      .yaw_deg = 95.0f,
  };
  JoystickProfileFeedback profile_feedback;

  SafetyLimits_GetJoystickProfileFeedback(JOYSTICK_PROFILE_AXIS_Y2,
                                          &safety_feedback,
                                          &profile_feedback);
  assert(profile_feedback.selected_axis_at_min == 1U);
  assert(profile_feedback.selected_axis_at_max == 0U);

  SafetyLimits_GetJoystickProfileFeedback(JOYSTICK_PROFILE_AXIS_X2,
                                          &safety_feedback,
                                          &profile_feedback);
  assert(profile_feedback.selected_axis_at_min == 0U);
  assert(profile_feedback.selected_axis_at_max == 1U);

  SafetyLimits_GetJoystickProfileFeedback(JOYSTICK_PROFILE_AXIS_X1,
                                          &safety_feedback,
                                          &profile_feedback);
  assert(profile_feedback.selected_axis_at_min == 0U);
  assert(profile_feedback.selected_axis_at_max == 1U);
}

int main(void)
{
  test_linear_axes_stop_only_outward_motion();
  test_swing_axis_uses_signed_yaw_limits();
  test_profile_feedback_reports_selected_axis_limits();
  return 0;
}
