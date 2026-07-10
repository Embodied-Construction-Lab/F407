#include "pid_controller.h"

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

static void test_feedforward_and_pid_generate_axis_output(void)
{
  PidController controller;
  float output;

  PidController_Init(&controller, 0.02f, 0.0f, 0.0f, 1.0f / 60.0f);
  output = PidController_Update(&controller, 30.0f, 10.0f, 0.05f);

  assert_float_equal(output, 0.9f);
}

static void test_zero_target_forces_zero_output_and_resets_state(void)
{
  PidController controller;
  float output;

  PidController_Init(&controller, 0.02f, 0.10f, 0.0f, 1.0f / 60.0f);
  output = PidController_Update(&controller, 30.0f, 0.0f, 1.0f);
  assert(output > 0.0f);

  output = PidController_Update(&controller, 0.0f, 20.0f, 1.0f);

  assert_float_equal(output, 0.0f);
  assert_float_equal(controller.integral, 0.0f);
  assert(controller.has_previous_error == 0U);
}

static void test_output_is_clamped_to_axis_range(void)
{
  PidController controller;
  float output;

  PidController_Init(&controller, 1.0f, 0.0f, 0.0f, 1.0f);
  output = PidController_Update(&controller, 10.0f, -10.0f, 0.05f);

  assert_float_equal(output, 0.9f);
}

static void test_asymmetric_feedforward_uses_target_direction(void)
{
  PidController controller;
  float output;

  PidController_InitAsymmetricFeedforward(&controller, 0.0f, 0.0f, 0.0f,
                                          0.10f, 0.20f);

  output = PidController_Update(&controller, 2.0f, 0.0f, 0.05f);
  assert_float_equal(output, 0.20f);

  output = PidController_Update(&controller, -2.0f, 0.0f, 0.05f);
  assert_float_equal(output, -0.40f);
}

int main(void)
{
  test_feedforward_and_pid_generate_axis_output();
  test_zero_target_forces_zero_output_and_resets_state();
  test_output_is_clamped_to_axis_range();
  test_asymmetric_feedforward_uses_target_direction();
  return 0;
}
