#include "joystick_servo_map.h"
#include "servo_control.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>

static void close_to(float actual, float expected)
{
  assert(fabsf(actual - expected) < 0.0001f);
}

static void neutral(const JoystickServoTargets *targets)
{
  close_to(targets->big_arm_deg, 90.0f);
  close_to(targets->small_arm_deg, 90.0f);
  close_to(targets->bucket_deg, 90.0f);
  close_to(targets->swing_percent, 0.0f);
  close_to(targets->pump_percent, 0.0f);
  close_to(targets->left_drive_percent, 0.0f);
  close_to(targets->right_drive_percent, 0.0f);
  for (unsigned i = 0; i < JOYSTICK_PCA_CHANNEL_COUNT; ++i)
  {
    assert(targets->pwm_count[i] == (i == JOYSTICK_CHANNEL_UNUSED ? 0U :
           ServoControl_SpeedToPulse(0.0f)));
  }
}

static void check_pwm(const JoystickServoTargets *t)
{
  assert(t->pwm_count[JOYSTICK_CHANNEL_BIG_ARM] == ServoControl_AngleToPulse(t->big_arm_deg));
  assert(t->pwm_count[JOYSTICK_CHANNEL_SMALL_ARM] == ServoControl_AngleToPulse(t->small_arm_deg));
  assert(t->pwm_count[JOYSTICK_CHANNEL_BUCKET] == ServoControl_AngleToPulse(t->bucket_deg));
  assert(t->pwm_count[JOYSTICK_CHANNEL_SWING] == ServoControl_SpeedToPulse(t->swing_percent));
  assert(t->pwm_count[JOYSTICK_CHANNEL_PUMP] == ServoControl_SpeedToPulse(t->pump_percent));
  assert(t->pwm_count[JOYSTICK_CHANNEL_UNUSED] == 0U);
  assert(t->pwm_count[JOYSTICK_CHANNEL_LEFT_DRIVE] == ServoControl_SpeedToPulse(0.0f));
  assert(t->pwm_count[JOYSTICK_CHANNEL_RIGHT_DRIVE] == ServoControl_SpeedToPulse(0.0f));
}

static void test_effort_mapping_and_manual_deadzone(void)
{
  const float magnitudes[] = {0.000001f, 0.05f, 0.149f, 0.15f, 0.151f, 0.5f, 1.0f, 2.0f};
  for (unsigned i = 0; i < sizeof(magnitudes) / sizeof(magnitudes[0]); ++i)
  {
    for (int sign = -1; sign <= 1; sign += 2)
    {
      JoystickServoTargets automatic, manual;
      const float value = sign * magnitudes[i];
      const float magnitude = fminf(magnitudes[i], 1.0f);
      VelocityServoMap_Compute(value, value, value, value, &automatic);
      close_to(automatic.big_arm_deg, sign > 0 ? 110.0f + 5.0f * magnitude : 78.0f - 5.0f * magnitude);
      close_to(automatic.small_arm_deg, sign > 0 ? 105.0f + 5.0f * magnitude : 75.0f - 6.0f * magnitude);
      close_to(automatic.bucket_deg, sign > 0 ? 105.0f + 5.0f * magnitude : 76.0f - 5.0f * magnitude);
      close_to(automatic.swing_percent, sign * magnitude * 20.0f);
      close_to(automatic.pump_percent, -30.0f);
      check_pwm(&automatic);
      JoystickServoMap_Compute(value, value, value, value, 0, 0, &manual);
      if (magnitudes[i] <= 0.15f)
      {
        neutral(&manual);
      }
      else
      {
        const float scaled = (magnitude - 0.15f) / 0.85f;
        close_to(manual.big_arm_deg, sign > 0 ? 110.0f + 5.0f * scaled : 78.0f - 5.0f * scaled);
        close_to(manual.small_arm_deg, sign > 0 ? 105.0f + 5.0f * scaled : 75.0f - 6.0f * scaled);
        close_to(manual.bucket_deg, sign > 0 ? 105.0f + 5.0f * scaled : 76.0f - 5.0f * scaled);
        check_pwm(&manual);
      }
    }
  }
}

static void test_invalid_efforts_and_zero(void)
{
  const float invalid[] = {NAN, INFINITY, -INFINITY};
  JoystickServoTargets targets;
  for (unsigned axis = 0; axis < 4; ++axis)
  {
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
      float effort[] = {0.5f, -0.5f, 0.5f, -0.5f};
      effort[axis] = invalid[i];
      VelocityServoMap_Compute(effort[0], effort[1], effort[2], effort[3], &targets);
      neutral(&targets);
    }
  }
  VelocityServoMap_Compute(0.0f, -0.0f, 0.0f, -0.0f, &targets);
  neutral(&targets);
  VelocityServoMap_Compute(0.0f, 0.0f, 0.0f, 0.1f, &targets);
  close_to(targets.swing_percent, 2.0f);
  close_to(targets.pump_percent, 0.0f);
  check_pwm(&targets);
  VelocityServoMap_Compute(0, 0, 0, 0, NULL);
}

int main(void)
{
  test_effort_mapping_and_manual_deadzone();
  test_invalid_efforts_and_zero();
  return 0;
}
