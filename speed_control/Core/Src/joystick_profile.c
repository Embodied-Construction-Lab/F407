#include "joystick_profile.h"

#include <stddef.h>

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

static float ramp(float from, float to, uint32_t elapsed_ms,
                  uint32_t duration_ms)
{
  float ratio;

  if (duration_ms == 0U)
  {
    return to;
  }
  if (elapsed_ms >= duration_ms)
  {
    return to;
  }

  ratio = (float)elapsed_ms / (float)duration_ms;
  return from + ((to - from) * ratio);
}

static float speed_wave(uint32_t phase_ms, float amplitude)
{
  if (phase_ms < 1000U)
  {
    return ramp(0.0f, amplitude, phase_ms, 1000U);
  }
  if (phase_ms < 3000U)
  {
    return amplitude;
  }
  if (phase_ms < 4000U)
  {
    return ramp(amplitude, 0.0f, phase_ms - 3000U, 1000U);
  }
  if (phase_ms < 5000U)
  {
    return ramp(0.0f, -amplitude, phase_ms - 4000U, 1000U);
  }
  if (phase_ms < 7000U)
  {
    return -amplitude;
  }
  return ramp(-amplitude, 0.0f, phase_ms - 7000U, 1000U);
}

void JoystickProfile_Init(JoystickProfile *profile, uint32_t start_ms)
{
  if (profile == NULL)
  {
    return;
  }

  profile->start_ms = start_ms;
  profile->x1_enabled = JOYSTICK_PROFILE_X1_ENABLED;
  profile->x2_enabled = JOYSTICK_PROFILE_X2_ENABLED;
  profile->y1_enabled = JOYSTICK_PROFILE_Y1_ENABLED;
  profile->y2_enabled = JOYSTICK_PROFILE_Y2_ENABLED;
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
}

void JoystickProfile_Update(JoystickProfile *profile, uint32_t now_ms)
{
  uint32_t phase_ms;

  if (profile == NULL)
  {
    return;
  }

  phase_ms = (uint32_t)(now_ms - profile->start_ms) %
             JOYSTICK_PROFILE_CYCLE_MS;

  profile->sample.x1 = (profile->x1_enabled != 0U)
                           ? speed_wave(phase_ms,
                                        JOYSTICK_PROFILE_X1_MAX_SPEED_DEG_S)
                           : 0.0f;
  profile->sample.x2 = (profile->x2_enabled != 0U)
                           ? speed_wave(phase_ms,
                                        JOYSTICK_PROFILE_X2_MAX_SPEED_MM_S)
                           : 0.0f;
  profile->sample.y1 = (profile->y1_enabled != 0U)
                           ? speed_wave(phase_ms,
                                        JOYSTICK_PROFILE_Y1_MAX_SPEED_MM_S)
                           : 0.0f;
  profile->sample.y2 = (profile->y2_enabled != 0U)
                           ? speed_wave(phase_ms,
                                        JOYSTICK_PROFILE_Y2_MAX_SPEED_MM_S)
                           : 0.0f;
}
