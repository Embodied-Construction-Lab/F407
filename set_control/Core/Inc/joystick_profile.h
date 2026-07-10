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

#define JOYSTICK_PROFILE_AMPLITUDE_COUNT 5U
#define JOYSTICK_PROFILE_REPEATS_PER_AMPLITUDE 5U

typedef enum
{
  JOYSTICK_PROFILE_AXIS_X1 = 0,
  JOYSTICK_PROFILE_AXIS_X2,
  JOYSTICK_PROFILE_AXIS_Y1,
  JOYSTICK_PROFILE_AXIS_Y2
} JoystickProfileAxis;

#ifndef JOYSTICK_PROFILE_SELECTED_AXIS
#define JOYSTICK_PROFILE_SELECTED_AXIS JOYSTICK_PROFILE_AXIS_Y2
#endif

typedef enum
{
  JOYSTICK_PROFILE_STATE_MOVING_POSITIVE = 0,
  JOYSTICK_PROFILE_STATE_MOVING_NEGATIVE,
  JOYSTICK_PROFILE_STATE_COMPLETE
} JoystickProfileState;

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
  JoystickProfileAxis selected_axis;
  JoystickProfileState state;
  uint8_t amplitude_index;
  uint8_t repeat_count;
  uint8_t x1_enabled;
  uint8_t x2_enabled;
  uint8_t y1_enabled;
  uint8_t y2_enabled;
  JoystickProfileSample sample;
} JoystickProfile;

typedef struct
{
  uint8_t selected_axis_at_min;
  uint8_t selected_axis_at_max;
} JoystickProfileFeedback;

void JoystickProfile_Init(JoystickProfile *profile, uint32_t start_ms);
void JoystickProfile_SetAxisEnabled(JoystickProfile *profile,
                                    uint8_t x1_enabled,
                                    uint8_t x2_enabled,
                                    uint8_t y1_enabled,
                                    uint8_t y2_enabled);
void JoystickProfile_Update(JoystickProfile *profile,
                            const JoystickProfileFeedback *feedback);

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_PROFILE_H */
