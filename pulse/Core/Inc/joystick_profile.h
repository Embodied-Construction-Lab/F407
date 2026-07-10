#ifndef JOYSTICK_PROFILE_H
#define JOYSTICK_PROFILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef JOYSTICK_PROFILE_X1_ENABLED
#define JOYSTICK_PROFILE_X1_ENABLED 1U
#endif

#ifndef JOYSTICK_PROFILE_X2_ENABLED
#define JOYSTICK_PROFILE_X2_ENABLED 1U
#endif

#ifndef JOYSTICK_PROFILE_Y1_ENABLED
#define JOYSTICK_PROFILE_Y1_ENABLED 1U
#endif

#ifndef JOYSTICK_PROFILE_Y2_ENABLED
#define JOYSTICK_PROFILE_Y2_ENABLED 1U
#endif

#ifndef JOYSTICK_PROFILE_SINGLE_STEP_ENABLED
#define JOYSTICK_PROFILE_SINGLE_STEP_ENABLED 1U
#endif

#ifndef JOYSTICK_PROFILE_DEADZONE_SWEEP_ENABLED
#define JOYSTICK_PROFILE_DEADZONE_SWEEP_ENABLED 1U
#endif

#ifndef JOYSTICK_PROFILE_RAMP_REVERSAL_ENABLED
#define JOYSTICK_PROFILE_RAMP_REVERSAL_ENABLED 1U
#endif

#ifndef JOYSTICK_PROFILE_MODULATED_SINE_ENABLED
#define JOYSTICK_PROFILE_MODULATED_SINE_ENABLED 1U
#endif

#ifndef JOYSTICK_PROFILE_COORDINATED_ENABLED
#define JOYSTICK_PROFILE_COORDINATED_ENABLED 1U
#endif

#define JOYSTICK_PROFILE_STAGE_COUNT 4U
#define JOYSTICK_PROFILE_SEGMENT_COUNT 5U
#define JOYSTICK_PROFILE_STAGE_DURATION_MS 100000U
#define JOYSTICK_PROFILE_SEGMENT_DURATION_MS \
  (JOYSTICK_PROFILE_STAGE_DURATION_MS / JOYSTICK_PROFILE_SEGMENT_COUNT)

typedef enum
{
  JOYSTICK_PROFILE_SEGMENT_SINGLE_STEP = 0,
  JOYSTICK_PROFILE_SEGMENT_DEADZONE_SWEEP,
  JOYSTICK_PROFILE_SEGMENT_RAMP_REVERSAL,
  JOYSTICK_PROFILE_SEGMENT_MODULATED_SINE,
  JOYSTICK_PROFILE_SEGMENT_COORDINATED
} JoystickProfileSegment;

typedef struct
{
  float x1; /* swing */
  float x2; /* bucket */
  float y1; /* small arm */
  float y2; /* big arm */
} JoystickProfileSample;

typedef struct
{
  uint32_t start_ms;
  uint8_t stage_index;
  uint8_t segment_index;
  float stage_amplitude;
  uint8_t x1_enabled;
  uint8_t x2_enabled;
  uint8_t y1_enabled;
  uint8_t y2_enabled;
  uint8_t single_step_enabled;
  uint8_t deadzone_sweep_enabled;
  uint8_t ramp_reversal_enabled;
  uint8_t modulated_sine_enabled;
  uint8_t coordinated_enabled;
  JoystickProfileSample sample;
} JoystickProfile;

void JoystickProfile_Init(JoystickProfile *profile, uint32_t start_ms);
void JoystickProfile_SetAxisEnabled(JoystickProfile *profile,
                                    uint8_t x1_enabled,
                                    uint8_t x2_enabled,
                                    uint8_t y1_enabled,
                                    uint8_t y2_enabled);
void JoystickProfile_SetSegmentEnabled(JoystickProfile *profile,
                                       uint8_t single_step_enabled,
                                       uint8_t deadzone_sweep_enabled,
                                       uint8_t ramp_reversal_enabled,
                                       uint8_t modulated_sine_enabled,
                                       uint8_t coordinated_enabled);
void JoystickProfile_Update(JoystickProfile *profile, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_PROFILE_H */
