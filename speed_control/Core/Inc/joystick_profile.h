#ifndef JOYSTICK_PROFILE_H
#define JOYSTICK_PROFILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef JOYSTICK_PROFILE_X1_ENABLED
#define JOYSTICK_PROFILE_X1_ENABLED 0U
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

#define JOYSTICK_PROFILE_X1_MAX_SPEED_DEG_S 30.0f
#define JOYSTICK_PROFILE_X2_MAX_SPEED_MM_S 30.0f
#define JOYSTICK_PROFILE_Y1_MAX_SPEED_MM_S 30.0f
#define JOYSTICK_PROFILE_Y2_MAX_SPEED_MM_S 30.0f
#define JOYSTICK_PROFILE_CYCLE_MS 8000U

typedef struct
{
  float x1; /* swing target speed, deg/s */
  float x2; /* bucket target speed, mm/s */
  float y1; /* small arm target speed, mm/s */
  float y2; /* big arm target speed, mm/s */
} JoystickProfileSample;

typedef struct
{
  uint32_t start_ms;
  uint8_t x1_enabled;
  uint8_t x2_enabled;
  uint8_t y1_enabled;
  uint8_t y2_enabled;
  JoystickProfileSample sample;
} JoystickProfile;

void JoystickProfile_Init(JoystickProfile *profile, uint32_t start_ms);
void JoystickProfile_SetAxisEnabled(JoystickProfile *profile,
                                    uint8_t x1_enabled,
                                    uint8_t x2_enabled,
                                    uint8_t y1_enabled,
                                    uint8_t y2_enabled);
void JoystickProfile_Update(JoystickProfile *profile, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_PROFILE_H */
