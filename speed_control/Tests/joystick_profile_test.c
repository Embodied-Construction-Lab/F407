#include "joystick_profile.h"

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

static void test_profile_generates_four_axis_speed_curve_from_time(void)
{
  JoystickProfile profile;

  JoystickProfile_Init(&profile, 100U);
  JoystickProfile_SetAxisEnabled(&profile, 1U, 1U, 1U, 1U);

  JoystickProfile_Update(&profile, 100U);
  assert_float_equal(profile.sample.x1, 0.0f);
  assert_float_equal(profile.sample.x2, 0.0f);
  assert_float_equal(profile.sample.y1, 0.0f);
  assert_float_equal(profile.sample.y2, 0.0f);

  JoystickProfile_Update(&profile, 1100U);
  assert_float_equal(profile.sample.x1, JOYSTICK_PROFILE_X1_MAX_SPEED_DEG_S);
  assert_float_equal(profile.sample.x2, JOYSTICK_PROFILE_X2_MAX_SPEED_MM_S);
  assert_float_equal(profile.sample.y1, JOYSTICK_PROFILE_Y1_MAX_SPEED_MM_S);
  assert_float_equal(profile.sample.y2, JOYSTICK_PROFILE_Y2_MAX_SPEED_MM_S);

  JoystickProfile_Update(&profile, 5100U);
  assert_float_equal(profile.sample.x1, -JOYSTICK_PROFILE_X1_MAX_SPEED_DEG_S);
  assert_float_equal(profile.sample.x2, -JOYSTICK_PROFILE_X2_MAX_SPEED_MM_S);
  assert_float_equal(profile.sample.y1, -JOYSTICK_PROFILE_Y1_MAX_SPEED_MM_S);
  assert_float_equal(profile.sample.y2, -JOYSTICK_PROFILE_Y2_MAX_SPEED_MM_S);
}

static void test_disabled_axis_outputs_zero_speed(void)
{
  JoystickProfile profile;

  JoystickProfile_Init(&profile, 0U);
  JoystickProfile_SetAxisEnabled(&profile, 1U, 0U, 1U, 0U);
  JoystickProfile_Update(&profile, 1000U);

  assert_float_equal(profile.sample.x1, 20.0f);
  assert_float_equal(profile.sample.x2, 0.0f);
  assert_float_equal(profile.sample.y1, JOYSTICK_PROFILE_Y1_MAX_SPEED_MM_S);
  assert_float_equal(profile.sample.y2, 0.0f);
}

int main(void)
{
  test_profile_generates_four_axis_speed_curve_from_time();
  test_disabled_axis_outputs_zero_speed();
  return 0;
}
