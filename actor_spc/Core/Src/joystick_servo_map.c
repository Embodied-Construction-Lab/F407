#include "joystick_servo_map.h"

#include "servo_control.h"

#include <stddef.h>

static float absolute(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float clamp_axis(float value)
{
  if (value < -1.0f)
  {
    value = -1.0f;
  }
  else if (value > 1.0f)
  {
    value = 1.0f;
  }

  return (absolute(value) <= JOYSTICK_DEAD_ZONE) ? 0.0f : value;
}

static void update_pwm_counts(JoystickServoTargets *targets)
{
  targets->pwm_count[JOYSTICK_CHANNEL_BUCKET] =
      ServoControl_AngleToPulse(targets->bucket_deg);
  targets->pwm_count[JOYSTICK_CHANNEL_BIG_ARM] =
      ServoControl_AngleToPulse(targets->big_arm_deg);
  targets->pwm_count[JOYSTICK_CHANNEL_SMALL_ARM] =
      ServoControl_AngleToPulse(targets->small_arm_deg);
  targets->pwm_count[JOYSTICK_CHANNEL_SWING] =
      ServoControl_SpeedToPulse(targets->swing_percent);
  targets->pwm_count[JOYSTICK_CHANNEL_PUMP] =
      ServoControl_SpeedToPulse(targets->pump_percent);
  targets->pwm_count[JOYSTICK_CHANNEL_UNUSED] = 0U;
  targets->pwm_count[JOYSTICK_CHANNEL_LEFT_DRIVE] =
      ServoControl_SpeedToPulse(targets->left_drive_percent);
  targets->pwm_count[JOYSTICK_CHANNEL_RIGHT_DRIVE] =
      ServoControl_SpeedToPulse(targets->right_drive_percent);
}

void JoystickServoMap_SetNeutral(JoystickServoTargets *targets)
{
  if (targets == NULL)
  {
    return;
  }

  targets->bucket_deg = 90.0f;
  targets->big_arm_deg = 90.0f;
  targets->small_arm_deg = 90.0f;
  targets->swing_percent = 0.0f;
  targets->pump_percent = 0.0f;
  targets->left_drive_percent = 0.0f;
  targets->right_drive_percent = 0.0f;
  update_pwm_counts(targets);
}

void JoystickServoMap_Compute(float x1, float x2, float y1, float y2,
                              float z1, float z2,
                              JoystickServoTargets *targets)
{
  float pump_axis;

  if (targets == NULL)
  {
    return;
  }

  x1 = clamp_axis(x1 * JOYSTICK_X1_DIRECTION);
  x2 = clamp_axis(x2 * JOYSTICK_X2_DIRECTION);
  y1 = clamp_axis(y1 * JOYSTICK_Y1_DIRECTION);
  y2 = clamp_axis(y2 * JOYSTICK_Y2_DIRECTION);
  z1 = clamp_axis(z1);
  z2 = clamp_axis(z2);

  targets->bucket_deg = 90.0f + x2 * 45.0f;
  targets->big_arm_deg = 90.0f + y2 * 45.0f;
  targets->small_arm_deg = 90.0f + y1 * 45.0f;
  targets->swing_percent = x1 * 30.0f;

  pump_axis = 0.0f;
  if (absolute(y1) > pump_axis)
  {
    pump_axis = absolute(y1);
  }
  if (absolute(x2) > pump_axis)
  {
    pump_axis = absolute(x2);
  }
  if (absolute(y2) > pump_axis)
  {
    pump_axis = absolute(y2);
  }
  targets->pump_percent = -pump_axis * 20.0f;
  targets->left_drive_percent =  - z1 * 20.0f;
  targets->right_drive_percent =  + z2 * 20.0f;
  update_pwm_counts(targets);
}

bool JoystickServoMap_IsActiveChannel(uint8_t channel)
{
  return (channel < JOYSTICK_PCA_CHANNEL_COUNT) &&
         (channel != JOYSTICK_CHANNEL_UNUSED);
}

bool JoystickServoMap_IsTimedOut(uint32_t now_ms, uint32_t last_valid_ms)
{
  return (uint32_t)(now_ms - last_valid_ms) >
         JOYSTICK_CONTROL_TIMEOUT_MS;
}
