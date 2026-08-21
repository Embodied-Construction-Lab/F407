#include "velocity_control.h"

#include <assert.h>
#include <string.h>

static void assert_float_equal(float actual, float expected, float tolerance)
{
  assert(actual > expected - tolerance);
  assert(actual < expected + tolerance);
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
  assert_float_equal(output.target.boom, -10.0f, 0.001f);
  assert_float_equal(output.target.stick, -10.0f, 0.001f);
  assert_float_equal(output.target.bucket, 10.0f, 0.001f);
  assert_float_equal(output.target.swing, 5.729578f, 0.001f);
  assert_float_equal(output.valve_action.boom, -0.363333f, 0.001f);
  assert_float_equal(output.valve_action.stick, -0.265294f, 0.001f);
  assert_float_equal(output.valve_action.bucket, 0.193934f, 0.001f);
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
  reference.boom = -0.01f;
  assert(VelocityControl_Update(&controller, &reference, &feedback,
                                0.05f, &output));
  assert_float_equal(output.target.boom, 0.0f, 0.0001f);
  assert_float_equal(output.valve_action.boom, 0.0f, 0.0001f);
  assert((output.limit_mask & VELOCITY_LIMIT_BOOM) != 0U);
  return 0;
}
