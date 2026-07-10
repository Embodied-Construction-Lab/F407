#include "joystick_profile.h"

#include <stddef.h>

static const float experiment_amplitudes[JOYSTICK_PROFILE_AMPLITUDE_COUNT] = {
    0.4f,
    0.6f,
    0.8f,
    0.9f,
    1.0f,
};

static const JoystickProfileAxis experiment_axes[] = {
    JOYSTICK_PROFILE_AXIS_Y2,
    JOYSTICK_PROFILE_AXIS_Y1,
    JOYSTICK_PROFILE_AXIS_X2,
    JOYSTICK_PROFILE_AXIS_X1,
};

static uint8_t axis_count(void)
{
  return (uint8_t)(sizeof(experiment_axes) / sizeof(experiment_axes[0]));
}

static void clear_sample(JoystickProfileSample *sample)
{
  if (sample == NULL)
  {
    return;
  }

  sample->x1 = 0.0f;
  sample->x2 = 0.0f;
  sample->y1 = 0.0f;
  sample->y2 = 0.0f;
}

static void set_selected_axis(JoystickProfile *profile, float value)
{
  clear_sample(&profile->sample);

  if ((profile->selected_axis == JOYSTICK_PROFILE_AXIS_X1) &&
      (profile->x1_enabled != 0U))
  {
    profile->sample.x1 = value;
  }
  else if ((profile->selected_axis == JOYSTICK_PROFILE_AXIS_X2) &&
           (profile->x2_enabled != 0U))
  {
    profile->sample.x2 = value;
  }
  else if ((profile->selected_axis == JOYSTICK_PROFILE_AXIS_Y1) &&
           (profile->y1_enabled != 0U))
  {
    profile->sample.y1 = value;
  }
  else if ((profile->selected_axis == JOYSTICK_PROFILE_AXIS_Y2) &&
           (profile->y2_enabled != 0U))
  {
    profile->sample.y2 = value;
  }
}

static uint8_t is_axis_enabled(const JoystickProfile *profile,
                               JoystickProfileAxis axis)
{
  if (axis == JOYSTICK_PROFILE_AXIS_X1)
  {
    return profile->x1_enabled;
  }
  if (axis == JOYSTICK_PROFILE_AXIS_X2)
  {
    return profile->x2_enabled;
  }
  if (axis == JOYSTICK_PROFILE_AXIS_Y1)
  {
    return profile->y1_enabled;
  }
  if (axis == JOYSTICK_PROFILE_AXIS_Y2)
  {
    return profile->y2_enabled;
  }
  return 0U;
}

static uint8_t current_axis_index(const JoystickProfile *profile)
{
  uint8_t index;

  for (index = 0U; index < axis_count(); ++index)
  {
    if (experiment_axes[index] == profile->selected_axis)
    {
      return index;
    }
  }

  return 0U;
}

static uint8_t select_enabled_axis_from(JoystickProfile *profile,
                                        uint8_t start_index)
{
  uint8_t index;

  for (index = start_index; index < axis_count(); ++index)
  {
    if (is_axis_enabled(profile, experiment_axes[index]) != 0U)
    {
      profile->selected_axis = experiment_axes[index];
      profile->state = JOYSTICK_PROFILE_STATE_MOVING_POSITIVE;
      profile->amplitude_index = 0U;
      profile->repeat_count = 0U;
      return 1U;
    }
  }

  profile->state = JOYSTICK_PROFILE_STATE_COMPLETE;
  clear_sample(&profile->sample);
  return 0U;
}

static uint8_t ensure_selected_axis_enabled(JoystickProfile *profile)
{
  uint8_t index;

  if (is_axis_enabled(profile, profile->selected_axis) != 0U)
  {
    return 1U;
  }

  index = current_axis_index(profile);
  return select_enabled_axis_from(profile, index + 1U);
}

static float current_amplitude(const JoystickProfile *profile)
{
  if (profile->amplitude_index >= JOYSTICK_PROFILE_AMPLITUDE_COUNT)
  {
    return 0.0f;
  }

  return experiment_amplitudes[profile->amplitude_index];
}

static void advance_after_negative_limit(JoystickProfile *profile)
{
  uint8_t next_axis_index;

  profile->repeat_count++;
  if (profile->repeat_count < JOYSTICK_PROFILE_REPEATS_PER_AMPLITUDE)
  {
    profile->state = JOYSTICK_PROFILE_STATE_MOVING_POSITIVE;
    return;
  }

  profile->repeat_count = 0U;
  profile->amplitude_index++;
  if (profile->amplitude_index >= JOYSTICK_PROFILE_AMPLITUDE_COUNT)
  {
    next_axis_index = current_axis_index(profile) + 1U;
    (void)select_enabled_axis_from(profile, next_axis_index);
  }
  else
  {
    profile->state = JOYSTICK_PROFILE_STATE_MOVING_POSITIVE;
  }
}

void JoystickProfile_Init(JoystickProfile *profile, uint32_t start_ms)
{
  if (profile == NULL)
  {
    return;
  }

  profile->start_ms = start_ms;
  profile->selected_axis = JOYSTICK_PROFILE_AXIS_Y2;
  profile->state = JOYSTICK_PROFILE_STATE_MOVING_POSITIVE;
  profile->amplitude_index = 0U;
  profile->repeat_count = 0U;
  profile->x1_enabled = JOYSTICK_PROFILE_X1_ENABLED;
  profile->x2_enabled = JOYSTICK_PROFILE_X2_ENABLED;
  profile->y1_enabled = JOYSTICK_PROFILE_Y1_ENABLED;
  profile->y2_enabled = JOYSTICK_PROFILE_Y2_ENABLED;
  clear_sample(&profile->sample);
  (void)select_enabled_axis_from(profile, 0U);
}

void JoystickProfile_SetAxisEnabled(JoystickProfile *profile,
                                    uint8_t x1_enabled,
                                    uint8_t x2_enabled,
                                    uint8_t y1_enabled,
                                    uint8_t y2_enabled)
{
  if (profile == NULL)
  {
    return;
  }

  profile->x1_enabled = (x1_enabled != 0U) ? 1U : 0U;
  profile->x2_enabled = (x2_enabled != 0U) ? 1U : 0U;
  profile->y1_enabled = (y1_enabled != 0U) ? 1U : 0U;
  profile->y2_enabled = (y2_enabled != 0U) ? 1U : 0U;
  (void)ensure_selected_axis_enabled(profile);
}

void JoystickProfile_Update(JoystickProfile *profile,
                            const JoystickProfileFeedback *feedback)
{
  if (profile == NULL)
  {
    return;
  }

  if (!ensure_selected_axis_enabled(profile))
  {
    return;
  }

  if (profile->state == JOYSTICK_PROFILE_STATE_MOVING_POSITIVE)
  {
    if ((feedback != NULL) && (feedback->selected_axis_at_max != 0U))
    {
      profile->state = JOYSTICK_PROFILE_STATE_MOVING_NEGATIVE;
      set_selected_axis(profile, -current_amplitude(profile));
    }
    else
    {
      set_selected_axis(profile, current_amplitude(profile));
    }
    return;
  }

  if (profile->state == JOYSTICK_PROFILE_STATE_MOVING_NEGATIVE)
  {
    if ((feedback != NULL) && (feedback->selected_axis_at_min != 0U))
    {
      advance_after_negative_limit(profile);
    }

    if (profile->state == JOYSTICK_PROFILE_STATE_COMPLETE)
    {
      clear_sample(&profile->sample);
    }
    else if (profile->state == JOYSTICK_PROFILE_STATE_MOVING_POSITIVE)
    {
      set_selected_axis(profile, current_amplitude(profile));
    }
    else
    {
      set_selected_axis(profile, -current_amplitude(profile));
    }
    return;
  }

  clear_sample(&profile->sample);
}
