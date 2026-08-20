#include "manual_action.h"
#include "joystick_servo_map.h"

#include <assert.h>

static void assert_float_equal(float actual, float expected)
{
  assert(actual > expected - 0.0001f);
  assert(actual < expected + 0.0001f);
}

int main(void)
{
  ManualAction action;
  JoystickServoTargets targets;

  ManualAction_FromStick(-0.8f, 0.2f, 0.4f, -0.6f, &action);
  assert_float_equal(action.boom, -0.6f);
  assert_float_equal(action.stick, 0.4f);
  assert_float_equal(action.bucket, 0.2f);
  assert_float_equal(action.swing, -0.8f);

  ManualAction_FromStick(0.15f, -0.149f, 0.0f, 0.1f, &action);
  assert_float_equal(action.boom, 0.0f);
  assert_float_equal(action.stick, 0.0f);
  assert_float_equal(action.bucket, 0.0f);
  assert_float_equal(action.swing, 0.0f);

  ManualAction_FromStick(2.0f, -2.0f, 0.0f, 0.0f, &action);
  assert_float_equal(action.bucket, -1.0f);
  assert_float_equal(action.swing, 1.0f);

  JoystickServoMap_Compute(0.0f, 0.0f, 0.0f, 0.0f,
                           0.5f, 0.25f, &targets);
  assert_float_equal(targets.left_drive_percent, -10.0f);
  assert_float_equal(targets.right_drive_percent, 5.0f);
  JoystickServoMap_Compute(0.0f, 0.0f, 0.0f, 0.0f,
                           0.15f, -0.149f, &targets);
  assert_float_equal(targets.left_drive_percent, 0.0f);
  assert_float_equal(targets.right_drive_percent, 0.0f);
  assert(!JoystickServoMap_IsTimedOut(300U, 0U));
  assert(JoystickServoMap_IsTimedOut(301U, 0U));
  return 0;
}
