#include "homing_controller.h"

#include <stdio.h>

static int failures;

static void expect_true(int condition, const char *message)
{
  if (!condition)
  {
    ++failures;
    (void)printf("FAIL: %s\n", message);
  }
}

static SafetyLimitsFeedback centered_feedback(void)
{
  SafetyLimitsFeedback feedback;

  feedback.boom_length_hundredths_mm =
      HOMING_CONTROLLER_BOOM_TARGET_HUNDREDTHS_MM;
  feedback.stick_length_hundredths_mm =
      HOMING_CONTROLLER_STICK_TARGET_HUNDREDTHS_MM;
  feedback.bucket_length_hundredths_mm =
      HOMING_CONTROLLER_BUCKET_TARGET_HUNDREDTHS_MM;
  feedback.yaw_deg = HOMING_CONTROLLER_YAW_TARGET_DEG;
  return feedback;
}

static void expect_only_axis_active(const AxisControlSample *sample,
                                    char axis)
{
  expect_true((axis == 'x') ? (sample->x1 != 0.0f) : (sample->x1 == 0.0f),
              "unexpected x1 homing command");
  expect_true((axis == 'b') ? (sample->x2 != 0.0f) : (sample->x2 == 0.0f),
              "unexpected x2 homing command");
  expect_true((axis == 's') ? (sample->y1 != 0.0f) : (sample->y1 == 0.0f),
              "unexpected y1 homing command");
  expect_true((axis == 'B') ? (sample->y2 != 0.0f) : (sample->y2 == 0.0f),
              "unexpected y2 homing command");
}

static void test_homing_runs_boom_stick_bucket_swing_order(void)
{
  HomingController homing;
  SafetyLimitsFeedback feedback;

  HomingController_Init(&homing, 0U);
  feedback = centered_feedback();

  feedback.boom_length_hundredths_mm =
      HOMING_CONTROLLER_BOOM_TARGET_HUNDREDTHS_MM + 2000;
  feedback.stick_length_hundredths_mm =
      HOMING_CONTROLLER_STICK_TARGET_HUNDREDTHS_MM + 2000;
  feedback.bucket_length_hundredths_mm =
      HOMING_CONTROLLER_BUCKET_TARGET_HUNDREDTHS_MM + 2000;
  feedback.yaw_deg = HOMING_CONTROLLER_YAW_TARGET_DEG + 20.0f;

  expect_true(HomingController_Update(&homing, 0U, &feedback, 1U) ==
                  HOMING_CONTROLLER_RUNNING,
              "homing should run when all axes are away from target");
  expect_only_axis_active(&homing.sample, 'B');

  feedback.boom_length_hundredths_mm =
      HOMING_CONTROLLER_BOOM_TARGET_HUNDREDTHS_MM;
  (void)HomingController_Update(&homing, 50U, &feedback, 1U);
  expect_only_axis_active(&homing.sample, 's');

  feedback.stick_length_hundredths_mm =
      HOMING_CONTROLLER_STICK_TARGET_HUNDREDTHS_MM;
  (void)HomingController_Update(&homing, 100U, &feedback, 1U);
  expect_only_axis_active(&homing.sample, 'b');

  feedback.bucket_length_hundredths_mm =
      HOMING_CONTROLLER_BUCKET_TARGET_HUNDREDTHS_MM;
  (void)HomingController_Update(&homing, 150U, &feedback, 1U);
  expect_only_axis_active(&homing.sample, 'x');
}

int main(void)
{
  test_homing_runs_boom_stick_bucket_swing_order();

  if (failures != 0)
  {
    (void)printf("%d failures\n", failures);
    return 1;
  }

  (void)printf("homing_controller_test passed\n");
  return 0;
}
