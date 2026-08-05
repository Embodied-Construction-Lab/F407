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

static float valve_angle_from_axis(float axis,
                                   float positive_min_deg,
                                   float positive_max_deg,
                                   float negative_min_deg,
                                   float negative_max_deg)
{
  float magnitude;

  if (axis > 0.0f)
  {
    magnitude = (axis - JOYSTICK_DEAD_ZONE) /
                (1.0f - JOYSTICK_DEAD_ZONE);
    return positive_min_deg +
           magnitude * (positive_max_deg - positive_min_deg);
  }

  if (axis < 0.0f)
  {
    magnitude = (-axis - JOYSTICK_DEAD_ZONE) /
                (1.0f - JOYSTICK_DEAD_ZONE);
    return negative_max_deg -
           magnitude * (negative_max_deg - negative_min_deg);
  }

  return JOYSTICK_VALVE_NEUTRAL_DEG;
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

  targets->bucket_deg = JOYSTICK_VALVE_NEUTRAL_DEG;
  targets->big_arm_deg = JOYSTICK_VALVE_NEUTRAL_DEG;
  targets->small_arm_deg = JOYSTICK_VALVE_NEUTRAL_DEG;
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

  targets->bucket_deg = valve_angle_from_axis(
      x2,
      JOYSTICK_BUCKET_POSITIVE_MIN_DEG,
      JOYSTICK_BUCKET_POSITIVE_MAX_DEG,
      JOYSTICK_BUCKET_NEGATIVE_MIN_DEG,
      JOYSTICK_BUCKET_NEGATIVE_MAX_DEG);
  targets->big_arm_deg = valve_angle_from_axis(
      y2,
      JOYSTICK_BIG_ARM_POSITIVE_MIN_DEG,
      JOYSTICK_BIG_ARM_POSITIVE_MAX_DEG,
      JOYSTICK_BIG_ARM_NEGATIVE_MIN_DEG,
      JOYSTICK_BIG_ARM_NEGATIVE_MAX_DEG);
  targets->small_arm_deg = valve_angle_from_axis(
      y1,
      JOYSTICK_SMALL_ARM_POSITIVE_MIN_DEG,
      JOYSTICK_SMALL_ARM_POSITIVE_MAX_DEG,
      JOYSTICK_SMALL_ARM_NEGATIVE_MIN_DEG,
      JOYSTICK_SMALL_ARM_NEGATIVE_MAX_DEG);

  targets->swing_percent = x1 * JOYSTICK_SWING_MAX_PERCENT;
  targets->pump_percent = ((x2 != 0.0f) || (y2 != 0.0f) || (y1 != 0.0f))
                              ? JOYSTICK_FIXED_PUMP_PERCENT
                              : 0.0f;
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
