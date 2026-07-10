#include "homing_controller.h"

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

static void test_invalid_feedback_holds_zero(void)
{
  HomingController homing;
  SafetyLimitsFeedback feedback;
  HomingControllerStatus status;

  HomingController_Init(&homing, 100U);
  feedback = make_feedback(9000, 9000, 9000, -20.0f);
  status = HomingController_Update(&homing, 150U, &feedback, 0U);

  assert(status == HOMING_CONTROLLER_WAITING_FEEDBACK);
  assert_float_equal(homing.sample.x1, 0.0f);
  assert_float_equal(homing.sample.x2, 0.0f);
  assert_float_equal(homing.sample.y1, 0.0f);
  assert_float_equal(homing.sample.y2, 0.0f);
}

static void test_homing_uses_fixed_speed_toward_center(void)
{
  HomingController homing;
  SafetyLimitsFeedback feedback;
  HomingControllerStatus status;

  HomingController_Init(&homing, 0U);
  feedback = make_feedback(15000, 15000, 15000, -20.0f);
  status = HomingController_Update(&homing, 50U, &feedback, 1U);

  assert(status == HOMING_CONTROLLER_RUNNING);
  assert_float_equal(homing.sample.x1, 0.4f);
  assert_float_equal(homing.sample.x2, 0.4f);
  assert_float_equal(homing.sample.y1, 0.4f);
  assert_float_equal(homing.sample.y2, 0.4f);
}

static void test_timeout_waits_for_first_valid_feedback(void)
{
  HomingController homing;
  SafetyLimitsFeedback feedback;
  HomingControllerStatus status;

  HomingController_Init(&homing, 0U);
  feedback = make_feedback(15000, 15000, 15000, -20.0f);

  status = HomingController_Update(&homing,
                                   HOMING_CONTROLLER_TIMEOUT_MS + 1000U,
                                   &feedback, 0U);
  assert(status == HOMING_CONTROLLER_WAITING_FEEDBACK);

  status = HomingController_Update(&homing,
                                   HOMING_CONTROLLER_TIMEOUT_MS + 1050U,
                                   &feedback, 1U);
  assert(status == HOMING_CONTROLLER_RUNNING);
  assert_float_equal(homing.sample.x1, 0.4f);
}

static void test_axis_at_target_outputs_zero(void)
{
  HomingController homing;
  SafetyLimitsFeedback feedback;

  HomingController_Init(&homing, 0U);
  feedback = make_feedback(16000, 15000, 16000, 0.5f);
  (void)HomingController_Update(&homing, 50U, &feedback, 1U);

  assert_float_equal(homing.sample.x1, 0.0f);
  assert_float_equal(homing.sample.x2, 0.0f);
  assert_float_equal(homing.sample.y1, 0.4f);
  assert_float_equal(homing.sample.y2, 0.0f);
}

static void test_completion_requires_stable_center_time(void)
{
  HomingController homing;
  SafetyLimitsFeedback feedback;
  HomingControllerStatus status;

  HomingController_Init(&homing, 0U);
  feedback = make_feedback(16000, 16000, 16000, 0.0f);

  status = HomingController_Update(&homing, 100U, &feedback, 1U);
  assert(status == HOMING_CONTROLLER_RUNNING);

  status = HomingController_Update(&homing,
                                   100U + HOMING_CONTROLLER_STABLE_TIME_MS,
                                   &feedback, 1U);
  assert(status == HOMING_CONTROLLER_COMPLETE);
  assert_float_equal(homing.sample.x1, 0.0f);
  assert_float_equal(homing.sample.x2, 0.0f);
  assert_float_equal(homing.sample.y1, 0.0f);
  assert_float_equal(homing.sample.y2, 0.0f);
}

int main(void)
{
  test_invalid_feedback_holds_zero();
  test_homing_uses_fixed_speed_toward_center();
  test_timeout_waits_for_first_valid_feedback();
  test_axis_at_target_outputs_zero();
  test_completion_requires_stable_center_time();
  return 0;
}
