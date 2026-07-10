#include "joystick_profile.h"

#include <assert.h>

static void assert_float_equal(float actual, float expected)
{
  assert(actual > expected - 0.0001f);
  assert(actual < expected + 0.0001f);
}

static void test_profile_starts_positive_from_current_position(void)
{
  JoystickProfile profile;
  JoystickProfileFeedback feedback = {
      .selected_axis_at_min = 0U,
      .selected_axis_at_max = 0U,
  };

  JoystickProfile_Init(&profile, 0U);
  JoystickProfile_Update(&profile, &feedback);

  assert(profile.selected_axis == JOYSTICK_PROFILE_AXIS_Y2);
  assert(profile.state == JOYSTICK_PROFILE_STATE_MOVING_POSITIVE);
  assert(profile.amplitude_index == 0U);
  assert(profile.repeat_count == 0U);
  assert_float_equal(profile.sample.x1, 0.0f);
  assert_float_equal(profile.sample.x2, 0.0f);
  assert_float_equal(profile.sample.y1, 0.0f);
  assert_float_equal(profile.sample.y2, 0.4f);
}

static void complete_current_axis(JoystickProfile *profile)
{
  JoystickProfileFeedback feedback = {
      .selected_axis_at_min = 0U,
      .selected_axis_at_max = 0U,
  };
  uint8_t amplitude;
  uint8_t repeat;

  for (amplitude = 0U; amplitude < JOYSTICK_PROFILE_AMPLITUDE_COUNT;
       ++amplitude)
  {
    for (repeat = 0U; repeat < JOYSTICK_PROFILE_REPEATS_PER_AMPLITUDE;
         ++repeat)
    {
      feedback.selected_axis_at_min = 0U;
      feedback.selected_axis_at_max = 1U;
      JoystickProfile_Update(profile, &feedback);

      feedback.selected_axis_at_min = 1U;
      feedback.selected_axis_at_max = 0U;
      JoystickProfile_Update(profile, &feedback);
    }
  }
}

static void test_profile_runs_axes_in_boom_stick_bucket_swing_order(void)
{
  JoystickProfile profile;
  JoystickProfileFeedback feedback = {
      .selected_axis_at_min = 0U,
      .selected_axis_at_max = 0U,
  };

  JoystickProfile_Init(&profile, 0U);
  JoystickProfile_Update(&profile, &feedback);
  assert(profile.selected_axis == JOYSTICK_PROFILE_AXIS_Y2);
  assert_float_equal(profile.sample.y2, 0.4f);

  complete_current_axis(&profile);
  assert(profile.selected_axis == JOYSTICK_PROFILE_AXIS_Y1);
  assert(profile.state == JOYSTICK_PROFILE_STATE_MOVING_POSITIVE);
  assert(profile.amplitude_index == 0U);
  assert(profile.repeat_count == 0U);
  assert_float_equal(profile.sample.y1, 0.4f);

  complete_current_axis(&profile);
  assert(profile.selected_axis == JOYSTICK_PROFILE_AXIS_X2);
  assert_float_equal(profile.sample.x2, 0.4f);

  complete_current_axis(&profile);
  assert(profile.selected_axis == JOYSTICK_PROFILE_AXIS_X1);
  assert_float_equal(profile.sample.x1, 0.4f);

  complete_current_axis(&profile);
  assert(profile.state == JOYSTICK_PROFILE_STATE_COMPLETE);
  assert_float_equal(profile.sample.x1, 0.0f);
  assert_float_equal(profile.sample.x2, 0.0f);
  assert_float_equal(profile.sample.y1, 0.0f);
  assert_float_equal(profile.sample.y2, 0.0f);
}

static void test_profile_skips_disabled_axes(void)
{
  JoystickProfile profile;
  JoystickProfileFeedback feedback = {
      .selected_axis_at_min = 0U,
      .selected_axis_at_max = 0U,
  };

  JoystickProfile_Init(&profile, 0U);
  JoystickProfile_SetAxisEnabled(&profile, 0U, 1U, 0U, 1U);
  JoystickProfile_Update(&profile, &feedback);
  assert(profile.selected_axis == JOYSTICK_PROFILE_AXIS_Y2);
  assert_float_equal(profile.sample.y2, 0.4f);

  complete_current_axis(&profile);
  assert(profile.selected_axis == JOYSTICK_PROFILE_AXIS_X2);
  assert(profile.state == JOYSTICK_PROFILE_STATE_MOVING_POSITIVE);
  assert_float_equal(profile.sample.x2, 0.4f);

  complete_current_axis(&profile);
  assert(profile.state == JOYSTICK_PROFILE_STATE_COMPLETE);
}

static void test_one_amplitude_repeats_five_round_trips_then_advances(void)
{
  JoystickProfile profile;
  JoystickProfileFeedback feedback = {
      .selected_axis_at_min = 1U,
      .selected_axis_at_max = 0U,
  };
  uint8_t repeat;

  JoystickProfile_Init(&profile, 0U);
  JoystickProfile_Update(&profile, &feedback);

  for (repeat = 0U; repeat < 5U; ++repeat)
  {
    feedback.selected_axis_at_min = 0U;
    feedback.selected_axis_at_max = 1U;
    JoystickProfile_Update(&profile, &feedback);
    assert(profile.state == JOYSTICK_PROFILE_STATE_MOVING_NEGATIVE);
    assert_float_equal(profile.sample.y2, -0.4f);

    feedback.selected_axis_at_min = 1U;
    feedback.selected_axis_at_max = 0U;
    JoystickProfile_Update(&profile, &feedback);
  }

  assert(profile.state == JOYSTICK_PROFILE_STATE_MOVING_POSITIVE);
  assert(profile.amplitude_index == 1U);
  assert(profile.repeat_count == 0U);
  assert_float_equal(profile.sample.y2, 0.6f);
}

static void test_negative_motion_continues_until_min_limit(void)
{
  JoystickProfile profile;
  JoystickProfileFeedback feedback = {
      .selected_axis_at_min = 0U,
      .selected_axis_at_max = 0U,
  };

  JoystickProfile_Init(&profile, 0U);
  JoystickProfile_Update(&profile, &feedback);

  feedback.selected_axis_at_max = 1U;
  JoystickProfile_Update(&profile, &feedback);
  assert(profile.state == JOYSTICK_PROFILE_STATE_MOVING_NEGATIVE);
  assert_float_equal(profile.sample.y2, -0.4f);

  feedback.selected_axis_at_max = 0U;
  feedback.selected_axis_at_min = 0U;
  JoystickProfile_Update(&profile, &feedback);
  assert(profile.state == JOYSTICK_PROFILE_STATE_MOVING_NEGATIVE);
  assert_float_equal(profile.sample.y2, -0.4f);
}

static void test_reversed_motion_ignores_same_end_limit_until_other_end(void)
{
  JoystickProfile profile;
  JoystickProfileFeedback feedback = {
      .selected_axis_at_min = 0U,
      .selected_axis_at_max = 0U,
  };

  JoystickProfile_Init(&profile, 0U);
  JoystickProfile_Update(&profile, &feedback);

  feedback.selected_axis_at_max = 1U;
  JoystickProfile_Update(&profile, &feedback);
  assert(profile.state == JOYSTICK_PROFILE_STATE_MOVING_NEGATIVE);
  assert_float_equal(profile.sample.y2, -0.4f);

  JoystickProfile_Update(&profile, &feedback);
  assert(profile.state == JOYSTICK_PROFILE_STATE_MOVING_NEGATIVE);
  assert_float_equal(profile.sample.y2, -0.4f);

  feedback.selected_axis_at_max = 0U;
  feedback.selected_axis_at_min = 1U;
  JoystickProfile_Update(&profile, &feedback);
  assert(profile.state == JOYSTICK_PROFILE_STATE_MOVING_POSITIVE);
  assert_float_equal(profile.sample.y2, 0.4f);
}

static void test_profile_stops_after_twenty_five_round_trips(void)
{
  JoystickProfile profile;
  JoystickProfileFeedback feedback = {
      .selected_axis_at_min = 1U,
      .selected_axis_at_max = 0U,
  };
  uint8_t amplitude;
  uint8_t repeat;

  JoystickProfile_Init(&profile, 0U);
  JoystickProfile_Update(&profile, &feedback);
  JoystickProfile_SetAxisEnabled(&profile, 0U, 0U, 0U, 1U);

  for (amplitude = 0U; amplitude < JOYSTICK_PROFILE_AMPLITUDE_COUNT;
       ++amplitude)
  {
    for (repeat = 0U; repeat < JOYSTICK_PROFILE_REPEATS_PER_AMPLITUDE;
         ++repeat)
    {
      feedback.selected_axis_at_min = 0U;
      feedback.selected_axis_at_max = 1U;
      JoystickProfile_Update(&profile, &feedback);

      feedback.selected_axis_at_min = 1U;
      feedback.selected_axis_at_max = 0U;
      JoystickProfile_Update(&profile, &feedback);
    }
  }

  assert(profile.state == JOYSTICK_PROFILE_STATE_COMPLETE);
  assert_float_equal(profile.sample.x1, 0.0f);
  assert_float_equal(profile.sample.x2, 0.0f);
  assert_float_equal(profile.sample.y1, 0.0f);
  assert_float_equal(profile.sample.y2, 0.0f);
}

int main(void)
{
  test_profile_starts_positive_from_current_position();
  test_profile_runs_axes_in_boom_stick_bucket_swing_order();
  test_profile_skips_disabled_axes();
  test_one_amplitude_repeats_five_round_trips_then_advances();
  test_negative_motion_continues_until_min_limit();
  test_reversed_motion_ignores_same_end_limit_until_other_end();
  test_profile_stops_after_twenty_five_round_trips();
  return 0;
}
