#include "velocity_control.h"
#include "safety_limits.h"

#include <assert.h>
#include <string.h>

static void assert_float_equal(float actual, float expected, float tolerance)
{
  assert(actual > expected - tolerance);
  assert(actual < expected + tolerance);
}

static VelocityControlFeedback in_range_feedback(void)
{
  VelocityControlFeedback feedback = {0};
  feedback.boom_length_hundredths_mm = 12000;
  feedback.stick_length_hundredths_mm = 12000;
  feedback.bucket_length_hundredths_mm = 10000;
  return feedback;
}

static void test_measured_extension_feedback(void)
{
  for (int sign = -1; sign <= 1; sign += 2)
  {
    VelocityControl controller;
    VelocityControlOutput output;
    VelocityControlFeedback feedback = in_range_feedback();
    const ControlAxisCommand reference = {sign * 0.01f, sign * 0.01f,
                                          sign * 0.01f, sign * 0.1f};
    feedback.boom_speed_hundredths_mm_s = sign * 1000;
    feedback.stick_speed_hundredths_mm_s = sign * 1000;
    feedback.bucket_speed_hundredths_mm_s = sign * 1000;
    feedback.swing_speed_deg_s = sign * 5.729578f;
    VelocityControl_Init(&controller);
    assert(VelocityControl_Update(&controller, &reference, &feedback,
                                  0.05f, &output));
    /* Measured physical velocity matches the request: zero feedback error.
       Nonzero feedforward remains necessary to sustain that velocity. */
    assert_float_equal(controller.boom_pid.previous_error, 0.0f, 0.001f);
    assert_float_equal(controller.stick_pid.previous_error, 0.0f, 0.001f);
    assert_float_equal(controller.bucket_pid.previous_error, 0.0f, 0.001f);
    assert_float_equal(controller.swing_pid.previous_error, 0.0f, 0.001f);
    assert(output.valve_action.boom * sign > 0.0f);
    assert(output.valve_action.stick * sign > 0.0f);
    assert(output.valve_action.bucket * sign < 0.0f);
    assert(output.valve_action.swing * sign > 0.0f);

    /* A zero command clears PID even while measured feedback is nonzero. */
    const ControlAxisCommand stop = {0};
    assert(VelocityControl_Update(&controller, &stop, &feedback, 0.05f, &output));
    assert(output.valve_action.boom == 0.0f);
    assert(output.valve_action.stick == 0.0f);
    assert(output.valve_action.bucket == 0.0f);
    assert(output.valve_action.swing == 0.0f);
    assert(controller.boom_pid.has_previous_error == 0U);
    assert(controller.stick_pid.has_previous_error == 0U);
    assert(controller.bucket_pid.has_previous_error == 0U);
    assert(controller.swing_pid.has_previous_error == 0U);
  }
}

static void test_physical_limits_and_retreat(void)
{
  for (unsigned upper = 0; upper < 2; ++upper)
  {
    const float sign = upper ? 1.0f : -1.0f;
    VelocityControl controller;
    VelocityControlOutput output;
    VelocityControlFeedback feedback = in_range_feedback();
    const ControlAxisCommand outward = {sign * 0.01f, sign * 0.01f,
                                        sign * 0.01f, sign * 0.1f};
    const ControlAxisCommand retreat = {-sign * 0.01f, -sign * 0.01f,
                                        -sign * 0.01f, -sign * 0.1f};
    feedback.boom_length_hundredths_mm = upper ?
        SAFETY_LIMIT_BOOM_MAX_HUNDREDTHS_MM : SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM;
    feedback.stick_length_hundredths_mm = upper ?
        SAFETY_LIMIT_STICK_MAX_HUNDREDTHS_MM : SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM;
    feedback.bucket_length_hundredths_mm = upper ?
        SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM : SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM;
    feedback.swing_angle_deg = upper ? SAFETY_LIMIT_SWING_MAX_DEG :
                                      SAFETY_LIMIT_SWING_MIN_DEG;
    VelocityControl_Init(&controller);
    assert(VelocityControl_Update(&controller, &outward, &feedback, 0.05f, &output));
    assert(output.target.boom == 0.0f && output.valve_action.boom == 0.0f);
    assert(output.target.stick == 0.0f && output.valve_action.stick == 0.0f);
    assert(output.target.bucket == 0.0f && output.valve_action.bucket == 0.0f);
    assert(output.target.swing == 0.0f && output.valve_action.swing == 0.0f);
    assert(output.limit_mask == (VELOCITY_LIMIT_BOOM | VELOCITY_LIMIT_STICK |
                                 VELOCITY_LIMIT_BUCKET | VELOCITY_LIMIT_SWING));
    assert(VelocityControl_Update(&controller, &retreat, &feedback, 0.05f, &output));
    assert(output.valve_action.boom * sign < 0.0f);
    assert(output.valve_action.stick * sign < 0.0f);
    assert(output.valve_action.bucket * sign > 0.0f);
    assert(output.valve_action.swing * sign < 0.0f);
    assert(output.limit_mask == 0U);
  }
}

int main(void)
{
  VelocityControl controller;
  VelocityControlFeedback feedback;
  ControlAxisCommand reference;
  VelocityControlOutput output;

  memset(&feedback, 0, sizeof(feedback));
  feedback.boom_length_hundredths_mm = 12000;
  feedback.stick_length_hundredths_mm = 12000;
  feedback.bucket_length_hundredths_mm = 10000;
  reference.boom = 0.01f;
  reference.stick = 0.01f;
  reference.bucket = 0.01f;
  reference.swing = 0.1f;

  VelocityControl_Init(&controller);
  assert(VelocityControl_Update(&controller, &reference, &feedback,
                                0.05f, &output));

  /* Orin physical velocity signs are adapted only inside STM32. */
  assert_float_equal(output.target.boom, 10.0f, 0.001f);
  assert_float_equal(output.target.stick, 10.0f, 0.001f);
  assert_float_equal(output.target.bucket, -10.0f, 0.001f);
  assert_float_equal(output.target.swing, 5.729578f, 0.001f);
  assert_float_equal(output.valve_action.boom, 0.363333f, 0.001f);
  assert_float_equal(output.valve_action.stick, 0.196667f, 0.001f);
  assert_float_equal(output.valve_action.bucket, -0.303973f, 0.001f);
  assert_float_equal(output.valve_action.swing, 0.171887f, 0.001f);

  VelocityControl_Reset(&controller);
  memset(&reference, 0, sizeof(reference));
  assert(VelocityControl_Update(&controller, &reference, &feedback,
                                0.05f, &output));
  assert_float_equal(output.valve_action.boom, 0.0f, 0.0001f);
  assert_float_equal(output.valve_action.stick, 0.0f, 0.0001f);
  assert_float_equal(output.valve_action.bucket, 0.0f, 0.0001f);
  assert_float_equal(output.valve_action.swing, 0.0f, 0.0001f);

  feedback.boom_length_hundredths_mm = 19000;
  reference.boom = 0.01f;
  assert(VelocityControl_Update(&controller, &reference, &feedback,
                                0.05f, &output));
  assert_float_equal(output.target.boom, 0.0f, 0.0001f);
  assert_float_equal(output.valve_action.boom, 0.0f, 0.0001f);
  assert((output.limit_mask & VELOCITY_LIMIT_BOOM) != 0U);
  test_measured_extension_feedback();
  test_physical_limits_and_retreat();
  return 0;
}
