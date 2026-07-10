#include "joystick_profile.h"

#include <assert.h>

static float absolute(float value)
{
  return (value < 0.0f) ? -value : value;
}

static void assert_float_equal(float actual, float expected)
{
  assert(actual > expected - 0.0001f);
  assert(actual < expected + 0.0001f);
}

static void assert_sample_bounded(const JoystickProfileSample *sample,
                                  float limit)
{
  assert(absolute(sample->x1) <= limit + 0.0001f);
  assert(absolute(sample->x2) <= limit + 0.0001f);
  assert(absolute(sample->y1) <= limit + 0.0001f);
  assert(absolute(sample->y2) <= limit + 0.0001f);
}

static float sample_axis_value(const JoystickProfileSample *sample,
                               uint8_t axis)
{
  if (axis == 0U)
  {
    return sample->x1;
  }
  if (axis == 1U)
  {
    return sample->x2;
  }
  if (axis == 2U)
  {
    return sample->y1;
  }
  return sample->y2;
}

static void assert_only_axis_positive(const JoystickProfileSample *sample,
                                      uint8_t active_axis)
{
  uint8_t axis;

  for (axis = 0U; axis < 4U; axis++)
  {
    if (axis == active_axis)
    {
      assert(sample_axis_value(sample, axis) > 0.0001f);
    }
    else
    {
      assert_float_equal(sample_axis_value(sample, axis), 0.0f);
    }
  }
}

static void test_default_segments_keep_requested_order(void)
{
  JoystickProfile profile;

  JoystickProfile_Init(&profile, 100U);

  JoystickProfile_Update(&profile, 100U);
  assert(profile.segment_index == JOYSTICK_PROFILE_SEGMENT_SINGLE_STEP);

  JoystickProfile_Update(&profile,
                         100U + JOYSTICK_PROFILE_SEGMENT_DURATION_MS);
  assert(profile.segment_index == JOYSTICK_PROFILE_SEGMENT_DEADZONE_SWEEP);

  JoystickProfile_Update(&profile,
                         100U + JOYSTICK_PROFILE_SEGMENT_DURATION_MS * 2U);
  assert(profile.segment_index == JOYSTICK_PROFILE_SEGMENT_RAMP_REVERSAL);

  JoystickProfile_Update(&profile,
                         100U + JOYSTICK_PROFILE_SEGMENT_DURATION_MS * 3U);
  assert(profile.segment_index == JOYSTICK_PROFILE_SEGMENT_MODULATED_SINE);

  JoystickProfile_Update(&profile,
                         100U + JOYSTICK_PROFILE_SEGMENT_DURATION_MS * 4U);
  assert(profile.segment_index == JOYSTICK_PROFILE_SEGMENT_COORDINATED);
}

static void test_single_axis_segments_use_big_small_bucket_swing_order(void)
{
  static const uint8_t expected_axis_order[4] = {3U, 2U, 1U, 0U};
  JoystickProfile profile;
  uint32_t slot_ms;
  uint32_t axis_ms;
  uint32_t phase_ms;
  uint8_t index;

  JoystickProfile_Init(&profile, 0U);
  slot_ms = JOYSTICK_PROFILE_SEGMENT_DURATION_MS / 16U;
  axis_ms = JOYSTICK_PROFILE_SEGMENT_DURATION_MS / 4U;

  for (index = 0U; index < 4U; index++)
  {
    JoystickProfile_Update(&profile, (uint32_t)index * 4U * slot_ms);
    assert(profile.segment_index == JOYSTICK_PROFILE_SEGMENT_SINGLE_STEP);
    assert_only_axis_positive(&profile.sample, expected_axis_order[index]);
  }

  phase_ms = axis_ms / 12U;
  for (index = 0U; index < 4U; index++)
  {
    JoystickProfile_Update(&profile,
                           JOYSTICK_PROFILE_SEGMENT_DURATION_MS +
                               ((uint32_t)index * axis_ms) + phase_ms);
    assert(profile.segment_index == JOYSTICK_PROFILE_SEGMENT_DEADZONE_SWEEP);
    assert_only_axis_positive(&profile.sample, expected_axis_order[index]);
  }

  phase_ms = axis_ms / 8U;
  for (index = 0U; index < 4U; index++)
  {
    JoystickProfile_Update(&profile,
                           (JOYSTICK_PROFILE_SEGMENT_DURATION_MS * 2U) +
                               ((uint32_t)index * axis_ms) + phase_ms);
    assert(profile.segment_index == JOYSTICK_PROFILE_SEGMENT_RAMP_REVERSAL);
    assert_only_axis_positive(&profile.sample, expected_axis_order[index]);
  }
}

static void test_disabled_segments_are_skipped_without_idle_time(void)
{
  JoystickProfile profile;

  JoystickProfile_Init(&profile, 0U);
  JoystickProfile_SetSegmentEnabled(&profile, 1U, 0U, 1U, 0U, 1U);

  JoystickProfile_Update(&profile, 0U);
  assert(profile.segment_index == JOYSTICK_PROFILE_SEGMENT_SINGLE_STEP);

  JoystickProfile_Update(&profile, JOYSTICK_PROFILE_STAGE_DURATION_MS / 3U);
  assert(profile.segment_index == JOYSTICK_PROFILE_SEGMENT_RAMP_REVERSAL);

  JoystickProfile_Update(&profile,
                         (JOYSTICK_PROFILE_STAGE_DURATION_MS / 3U) * 2U);
  assert(profile.segment_index == JOYSTICK_PROFILE_SEGMENT_COORDINATED);
}

static void test_all_disabled_segments_output_zero(void)
{
  JoystickProfile profile;

  JoystickProfile_Init(&profile, 0U);
  JoystickProfile_SetSegmentEnabled(&profile, 0U, 0U, 0U, 0U, 0U);
  JoystickProfile_Update(&profile, 5000U);

  assert_float_equal(profile.sample.x1, 0.0f);
  assert_float_equal(profile.sample.x2, 0.0f);
  assert_float_equal(profile.sample.y1, 0.0f);
  assert_float_equal(profile.sample.y2, 0.0f);
}

static void test_segments_remain_bounded_and_axis_switches_apply(void)
{
  JoystickProfile profile;

  JoystickProfile_Init(&profile, 0U);
  JoystickProfile_Update(&profile,
                         JOYSTICK_PROFILE_SEGMENT_DURATION_MS * 4U + 250U);
  assert_sample_bounded(&profile.sample, profile.stage_amplitude);
  assert(absolute(profile.sample.x1) > 0.0001f);
  assert(absolute(profile.sample.x2) > 0.0001f);
  assert(absolute(profile.sample.y1) > 0.0001f);
  assert(absolute(profile.sample.y2) > 0.0001f);

  JoystickProfile_SetAxisEnabled(&profile, 1U, 0U, 1U, 0U);
  JoystickProfile_Update(&profile,
                         JOYSTICK_PROFILE_SEGMENT_DURATION_MS * 4U + 250U);
  assert(absolute(profile.sample.x1) > 0.0001f);
  assert_float_equal(profile.sample.x2, 0.0f);
  assert(absolute(profile.sample.y1) > 0.0001f);
  assert_float_equal(profile.sample.y2, 0.0f);
}

int main(void)
{
  test_default_segments_keep_requested_order();
  test_single_axis_segments_use_big_small_bucket_swing_order();
  test_disabled_segments_are_skipped_without_idle_time();
  test_all_disabled_segments_output_zero();
  test_segments_remain_bounded_and_axis_switches_apply();
  return 0;
}
