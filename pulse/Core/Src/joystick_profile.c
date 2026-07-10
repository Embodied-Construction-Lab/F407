#include "joystick_profile.h"

#include <stddef.h>

static const float stage_amplitudes[JOYSTICK_PROFILE_STAGE_COUNT] = {
    0.10f,
    0.20f,
    0.35f,
    0.50f,
};

static float absolute(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float clamp(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}

static float triangle_wave(uint32_t phase_ms, uint32_t period_ms)
{
  uint32_t phase;
  float ratio;

  if (period_ms == 0U)
  {
    return 0.0f;
  }

  phase = phase_ms % period_ms;
  ratio = (float)phase / (float)period_ms;

  if (ratio < 0.25f)
  {
    return ratio * 4.0f;
  }
  if (ratio < 0.75f)
  {
    return 2.0f - (ratio * 4.0f);
  }
  return (ratio * 4.0f) - 4.0f;
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

static void set_axis(JoystickProfileSample *sample, uint8_t axis, float value)
{
  if (axis == 0U)
  {
    sample->y2 = value;
  }
  else if (axis == 1U)
  {
    sample->y1 = value;
  }
  else if (axis == 2U)
  {
    sample->x2 = value;
  }
  else
  {
    sample->x1 = value;
  }
}

static JoystickProfileSample single_step(uint32_t local_ms,
                                         float amplitude)
{
  JoystickProfileSample sample;
  uint32_t slot_count = 16U;
  uint32_t slot_ms = JOYSTICK_PROFILE_SEGMENT_DURATION_MS / slot_count;
  uint32_t slot;
  uint8_t axis;
  uint8_t state;

  clear_sample(&sample);
  if (slot_ms == 0U)
  {
    return sample;
  }

  slot = local_ms / slot_ms;
  if (slot >= slot_count)
  {
    slot = slot_count - 1U;
  }
  axis = (uint8_t)(slot / 4U);
  state = (uint8_t)(slot % 4U);

  if (state == 0U)
  {
    set_axis(&sample, axis, amplitude);
  }
  else if (state == 2U)
  {
    set_axis(&sample, axis, -amplitude);
  }

  return sample;
}

static JoystickProfileSample deadzone_sweep(uint32_t local_ms,
                                            float amplitude)
{
  JoystickProfileSample sample;
  uint32_t axis_ms = JOYSTICK_PROFILE_SEGMENT_DURATION_MS / 4U;
  uint8_t axis;
  uint32_t phase_ms;
  float limit;

  clear_sample(&sample);
  if (axis_ms == 0U)
  {
    return sample;
  }

  axis = (uint8_t)(local_ms / axis_ms);
  if (axis > 3U)
  {
    axis = 3U;
  }
  phase_ms = local_ms % axis_ms;
  limit = (amplitude < 0.10f) ? amplitude : 0.10f;
  set_axis(&sample, axis, limit * triangle_wave(phase_ms, axis_ms / 3U));
  return sample;
}

static JoystickProfileSample ramp_reversal(uint32_t local_ms,
                                           float amplitude)
{
  JoystickProfileSample sample;
  uint32_t axis_ms = JOYSTICK_PROFILE_SEGMENT_DURATION_MS / 4U;
  uint8_t axis;
  uint32_t phase_ms;
  float ratio;
  float value;

  clear_sample(&sample);
  if (axis_ms == 0U)
  {
    return sample;
  }

  axis = (uint8_t)(local_ms / axis_ms);
  if (axis > 3U)
  {
    axis = 3U;
  }
  phase_ms = local_ms % axis_ms;
  ratio = (float)phase_ms / (float)axis_ms;

  if (ratio < 0.25f)
  {
    value = amplitude * ratio / 0.25f;
  }
  else if (ratio < 0.50f)
  {
    value = amplitude * (1.0f - ((ratio - 0.25f) / 0.25f));
  }
  else if (ratio < 0.75f)
  {
    value = -amplitude * ((ratio - 0.50f) / 0.25f);
  }
  else
  {
    value = -amplitude * (1.0f - ((ratio - 0.75f) / 0.25f));
  }

  set_axis(&sample, axis, value);
  return sample;
}

static JoystickProfileSample modulated_sine(uint32_t local_ms,
                                            float amplitude)
{
  JoystickProfileSample sample;
  float carrier_fast;
  float carrier_slow;
  float envelope;

  carrier_fast = triangle_wave(local_ms, 1200U);
  carrier_slow = triangle_wave(local_ms + 300U, 3600U);
  envelope = 0.35f + (0.65f * absolute(triangle_wave(local_ms, 5000U)));

  sample.x1 = amplitude * envelope * carrier_fast;
  sample.x2 = amplitude * envelope * triangle_wave(local_ms + 250U, 1400U);
  sample.y1 = amplitude * envelope * carrier_slow;
  sample.y2 = amplitude * envelope * triangle_wave(local_ms + 750U, 1800U);
  return sample;
}

static JoystickProfileSample coordinated(uint32_t local_ms,
                                         float amplitude)
{
  JoystickProfileSample sample;

  sample.x1 = amplitude * triangle_wave(local_ms, 2600U);
  sample.x2 = amplitude * triangle_wave(local_ms + 650U, 2600U);
  sample.y1 = amplitude * triangle_wave(local_ms + 1300U, 2600U);
  sample.y2 = amplitude * triangle_wave(local_ms + 1950U, 2600U);
  return sample;
}

static JoystickProfileSample segment_sample(uint8_t segment_index,
                                            uint32_t local_ms,
                                            float amplitude)
{
  if (segment_index == JOYSTICK_PROFILE_SEGMENT_SINGLE_STEP)
  {
    return single_step(local_ms, amplitude);
  }
  if (segment_index == JOYSTICK_PROFILE_SEGMENT_DEADZONE_SWEEP)
  {
    return deadzone_sweep(local_ms, amplitude);
  }
  if (segment_index == JOYSTICK_PROFILE_SEGMENT_RAMP_REVERSAL)
  {
    return ramp_reversal(local_ms, amplitude);
  }
  if (segment_index == JOYSTICK_PROFILE_SEGMENT_MODULATED_SINE)
  {
    return modulated_sine(local_ms, amplitude);
  }
  return coordinated(local_ms, amplitude);
}

static uint8_t segment_enabled(const JoystickProfile *profile,
                               uint8_t segment_index)
{
  if (segment_index == JOYSTICK_PROFILE_SEGMENT_SINGLE_STEP)
  {
    return profile->single_step_enabled;
  }
  if (segment_index == JOYSTICK_PROFILE_SEGMENT_DEADZONE_SWEEP)
  {
    return profile->deadzone_sweep_enabled;
  }
  if (segment_index == JOYSTICK_PROFILE_SEGMENT_RAMP_REVERSAL)
  {
    return profile->ramp_reversal_enabled;
  }
  if (segment_index == JOYSTICK_PROFILE_SEGMENT_MODULATED_SINE)
  {
    return profile->modulated_sine_enabled;
  }
  return profile->coordinated_enabled;
}

static uint8_t enabled_segment_count(const JoystickProfile *profile)
{
  uint8_t count = 0U;
  uint8_t index;

  for (index = 0U; index < JOYSTICK_PROFILE_SEGMENT_COUNT; index++)
  {
    if (segment_enabled(profile, index) != 0U)
    {
      count++;
    }
  }
  return count;
}

static uint8_t enabled_segment_at(const JoystickProfile *profile,
                                  uint8_t enabled_index)
{
  uint8_t count = 0U;
  uint8_t index;

  for (index = 0U; index < JOYSTICK_PROFILE_SEGMENT_COUNT; index++)
  {
    if (segment_enabled(profile, index) == 0U)
    {
      continue;
    }

    if (count == enabled_index)
    {
      return index;
    }
    count++;
  }

  return JOYSTICK_PROFILE_SEGMENT_COORDINATED;
}

static void apply_axis_switches(JoystickProfile *profile)
{
  if (profile->x1_enabled == 0U)
  {
    profile->sample.x1 = 0.0f;
  }
  if (profile->x2_enabled == 0U)
  {
    profile->sample.x2 = 0.0f;
  }
  if (profile->y1_enabled == 0U)
  {
    profile->sample.y1 = 0.0f;
  }
  if (profile->y2_enabled == 0U)
  {
    profile->sample.y2 = 0.0f;
  }
}

static void clamp_sample(JoystickProfileSample *sample, float amplitude)
{
  sample->x1 = clamp(sample->x1, amplitude);
  sample->x2 = clamp(sample->x2, amplitude);
  sample->y1 = clamp(sample->y1, amplitude);
  sample->y2 = clamp(sample->y2, amplitude);
}

void JoystickProfile_Init(JoystickProfile *profile, uint32_t start_ms)
{
  if (profile == NULL)
  {
    return;
  }

  profile->start_ms = start_ms;
  profile->stage_index = 0U;
  profile->segment_index = JOYSTICK_PROFILE_SEGMENT_SINGLE_STEP;
  profile->stage_amplitude = stage_amplitudes[0];
  profile->x1_enabled = JOYSTICK_PROFILE_X1_ENABLED;
  profile->x2_enabled = JOYSTICK_PROFILE_X2_ENABLED;
  profile->y1_enabled = JOYSTICK_PROFILE_Y1_ENABLED;
  profile->y2_enabled = JOYSTICK_PROFILE_Y2_ENABLED;
  profile->single_step_enabled = JOYSTICK_PROFILE_SINGLE_STEP_ENABLED;
  profile->deadzone_sweep_enabled = JOYSTICK_PROFILE_DEADZONE_SWEEP_ENABLED;
  profile->ramp_reversal_enabled = JOYSTICK_PROFILE_RAMP_REVERSAL_ENABLED;
  profile->modulated_sine_enabled = JOYSTICK_PROFILE_MODULATED_SINE_ENABLED;
  profile->coordinated_enabled = JOYSTICK_PROFILE_COORDINATED_ENABLED;
  clear_sample(&profile->sample);
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
  apply_axis_switches(profile);
}

void JoystickProfile_SetSegmentEnabled(JoystickProfile *profile,
                                       uint8_t single_step_enabled,
                                       uint8_t deadzone_sweep_enabled,
                                       uint8_t ramp_reversal_enabled,
                                       uint8_t modulated_sine_enabled,
                                       uint8_t coordinated_enabled)
{
  if (profile == NULL)
  {
    return;
  }

  profile->single_step_enabled = (single_step_enabled != 0U) ? 1U : 0U;
  profile->deadzone_sweep_enabled = (deadzone_sweep_enabled != 0U) ? 1U : 0U;
  profile->ramp_reversal_enabled = (ramp_reversal_enabled != 0U) ? 1U : 0U;
  profile->modulated_sine_enabled = (modulated_sine_enabled != 0U) ? 1U : 0U;
  profile->coordinated_enabled = (coordinated_enabled != 0U) ? 1U : 0U;
}

void JoystickProfile_Update(JoystickProfile *profile, uint32_t now_ms)
{
  uint32_t elapsed_ms;
  uint32_t stage_time_ms;
  uint32_t local_ms;
  uint32_t active_segment_duration_ms;
  uint32_t slot_local_ms;
  uint8_t active_segment_count;
  uint8_t active_segment_slot;

  if (profile == NULL)
  {
    return;
  }

  elapsed_ms = (uint32_t)(now_ms - profile->start_ms);
  profile->stage_index =
      (uint8_t)((elapsed_ms / JOYSTICK_PROFILE_STAGE_DURATION_MS) %
                JOYSTICK_PROFILE_STAGE_COUNT);
  stage_time_ms = elapsed_ms % JOYSTICK_PROFILE_STAGE_DURATION_MS;
  profile->stage_amplitude = stage_amplitudes[profile->stage_index];

  active_segment_count = enabled_segment_count(profile);
  if (active_segment_count == 0U)
  {
    profile->segment_index = JOYSTICK_PROFILE_SEGMENT_SINGLE_STEP;
    clear_sample(&profile->sample);
    return;
  }

  active_segment_duration_ms =
      JOYSTICK_PROFILE_STAGE_DURATION_MS / active_segment_count;
  if (active_segment_duration_ms == 0U)
  {
    clear_sample(&profile->sample);
    return;
  }

  active_segment_slot = (uint8_t)(stage_time_ms / active_segment_duration_ms);
  if (active_segment_slot >= active_segment_count)
  {
    active_segment_slot = active_segment_count - 1U;
  }

  profile->segment_index = enabled_segment_at(profile, active_segment_slot);
  slot_local_ms = stage_time_ms -
                  ((uint32_t)active_segment_slot *
                   active_segment_duration_ms);
  local_ms = (uint32_t)(((uint64_t)slot_local_ms *
                         JOYSTICK_PROFILE_SEGMENT_DURATION_MS) /
                        active_segment_duration_ms);
  if (local_ms >= JOYSTICK_PROFILE_SEGMENT_DURATION_MS)
  {
    local_ms = JOYSTICK_PROFILE_SEGMENT_DURATION_MS - 1U;
  }

  profile->sample = segment_sample(profile->segment_index, local_ms,
                                   profile->stage_amplitude);
  clamp_sample(&profile->sample, profile->stage_amplitude);
  apply_axis_switches(profile);
}
