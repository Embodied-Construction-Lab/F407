#include "safety_limits.h"

#include <assert.h>
#include <math.h>

static void assert_float_equal(float actual, float expected)
{
  assert(actual > expected - 0.0001f);
  assert(actual < expected + 0.0001f);
}

int main(void)
{
  assert_float_equal(SafetyLimits_LimitSwingCommand(0.5f, 99.0f), 0.5f);
  assert_float_equal(SafetyLimits_LimitSwingCommand(0.5f, 100.0f), 0.0f);
  assert_float_equal(SafetyLimits_LimitSwingCommand(0.5f, 101.0f), 0.0f);
  assert_float_equal(SafetyLimits_LimitSwingCommand(-0.5f, 101.0f), -0.5f);

  assert_float_equal(SafetyLimits_LimitSwingCommand(-0.5f, -99.0f), -0.5f);
  assert_float_equal(SafetyLimits_LimitSwingCommand(-0.5f, -100.0f), 0.0f);
  assert_float_equal(SafetyLimits_LimitSwingCommand(-0.5f, -101.0f), 0.0f);
  assert_float_equal(SafetyLimits_LimitSwingCommand(0.5f, -101.0f), 0.5f);

  assert_float_equal(SafetyLimits_LimitSwingCommand(0.5f, NAN), 0.0f);
  assert_float_equal(SafetyLimits_LimitSwingCommand(NAN, 0.0f), 0.0f);
  return 0;
}
