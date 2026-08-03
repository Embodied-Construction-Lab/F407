#ifndef JOYSTICK_SERVO_MAP_H
#define JOYSTICK_SERVO_MAP_H

#include <stdbool.h>
#include <stdint.h>

#define JOYSTICK_DEAD_ZONE 0.05f
#define JOYSTICK_CONTROL_TIMEOUT_MS 300U
#define JOYSTICK_PCA_CHANNEL_COUNT 8U

#define JOYSTICK_X1_DIRECTION 1.0f
#define JOYSTICK_X2_DIRECTION 1.0f
#define JOYSTICK_Y1_DIRECTION 1.0f
#define JOYSTICK_Y2_DIRECTION 1.0f

typedef enum
{
  JOYSTICK_CHANNEL_BUCKET = 0,
  JOYSTICK_CHANNEL_BIG_ARM = 1,
  JOYSTICK_CHANNEL_SMALL_ARM = 2,
  JOYSTICK_CHANNEL_SWING = 3,
  JOYSTICK_CHANNEL_PUMP = 4,
  JOYSTICK_CHANNEL_UNUSED = 5,
  JOYSTICK_CHANNEL_LEFT_DRIVE = 6,
  JOYSTICK_CHANNEL_RIGHT_DRIVE = 7
} JoystickServoChannel;

typedef struct
{
  float bucket_deg;
  float big_arm_deg;
  float small_arm_deg;
  float swing_percent;
  float pump_percent;
  float left_drive_percent;
  float right_drive_percent;
  uint16_t pwm_count[JOYSTICK_PCA_CHANNEL_COUNT];
} JoystickServoTargets;

void JoystickServoMap_SetNeutral(JoystickServoTargets *targets);
void JoystickServoMap_Compute(float x1, float x2, float y1, float y2,
                              float z1, float z2,
                              JoystickServoTargets *targets);
bool JoystickServoMap_IsActiveChannel(uint8_t channel);
bool JoystickServoMap_IsTimedOut(uint32_t now_ms, uint32_t last_valid_ms);

#endif /* JOYSTICK_SERVO_MAP_H */
