#include "speed_pid_config.h"

#include <assert.h>

static void assert_float_equal(float actual, float expected)
{
  float delta = actual - expected;

  if (delta < 0.0f)
  {
    delta = -delta;
  }
  assert(delta <= 0.0001f);
}

int main(void)
{
  assert_float_equal(SPEED_PID_X1_KP, 0.02f);
  assert_float_equal(SPEED_PID_X1_KI, 0.0f);
  assert_float_equal(SPEED_PID_X1_KD, 0.0f);
  assert_float_equal(SPEED_PID_X1_FEEDFORWARD, 1.0f / 40.0f);

  assert_float_equal(SPEED_PID_X2_KP, 0.015f);
  assert_float_equal(SPEED_PID_X2_KI, 0.001f);
  assert_float_equal(SPEED_PID_X2_KD, 0.0f);
  assert_float_equal(SPEED_PID_X2_POSITIVE_FEEDFORWARD, 1.0f / 42.0f);
  assert_float_equal(SPEED_PID_X2_NEGATIVE_FEEDFORWARD, 1.0f / 34.0f);

  assert_float_equal(SPEED_PID_Y1_KP, 0.015f);
  assert_float_equal(SPEED_PID_Y1_KI, 0.001f);
  assert_float_equal(SPEED_PID_Y1_KD, 0.0f);
  assert_float_equal(SPEED_PID_Y1_POSITIVE_FEEDFORWARD, 1.0f / 35.0f);
  assert_float_equal(SPEED_PID_Y1_NEGATIVE_FEEDFORWARD, 1.0f / 43.0f);

  assert_float_equal(SPEED_PID_Y2_KP, 0.015f);
  assert_float_equal(SPEED_PID_Y2_KI, 0.0f);
  assert_float_equal(SPEED_PID_Y2_KD, 0.0f);
  assert_float_equal(SPEED_PID_Y2_POSITIVE_FEEDFORWARD, 1.0f / 17.5f);
  assert_float_equal(SPEED_PID_Y2_NEGATIVE_FEEDFORWARD, 1.0f / 32.0f);

  return 0;
}
