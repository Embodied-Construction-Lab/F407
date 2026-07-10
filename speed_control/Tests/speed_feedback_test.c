#include "speed_feedback.h"

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
  SpeedFeedback feedback;

  SpeedFeedback_FromRaw(&feedback, 12.5f, 4200, -3500, 1800);

  assert_float_equal(feedback.x1_deg_s, 12.5f);
  assert_float_equal(feedback.x2_mm_s, -42.0f);
  assert_float_equal(feedback.y1_mm_s, -35.0f);
  assert_float_equal(feedback.y2_mm_s, 18.0f);

  return 0;
}
