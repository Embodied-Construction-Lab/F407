#include "safety_limits.h"

#include <stddef.h>

static float limit_linear_axis(float command, int32_t position,
                               int32_t minimum, int32_t maximum,
                               uint8_t positive_increases_length)
{
  float increasing_command =
      (positive_increases_length != 0U) ? command : -command;

  if ((position >= maximum) && (increasing_command > 0.0f))
  {
    return 0.0f;
  }
  if ((position <= minimum) && (increasing_command < 0.0f))
  {
    return 0.0f;
  }
  return command;
}

float SafetyLimits_NormalizeYawDeg(float yaw_deg)
{
  while (yaw_deg > 180.0f)
  {
    yaw_deg -= 360.0f;
  }
  while (yaw_deg < -180.0f)
  {
    yaw_deg += 360.0f;
  }
  return yaw_deg;
}

void SafetyLimits_Apply(AxisControlSample *sample,
                        const SafetyLimitsFeedback *feedback)
{
  float yaw_deg;

  if ((sample == NULL) || (feedback == NULL))
  {
    return;
  }
  yaw_deg = SafetyLimits_NormalizeYawDeg(feedback->yaw_deg);
  if ((yaw_deg >= SAFETY_LIMIT_SWING_MAX_DEG) && (sample->x1 > 0.0f))
  {
    sample->x1 = 0.0f;
  }
  else if ((yaw_deg <= SAFETY_LIMIT_SWING_MIN_DEG) && (sample->x1 < 0.0f))
  {
    sample->x1 = 0.0f;
  }

  sample->x2 = limit_linear_axis(
      sample->x2, feedback->bucket_length_hundredths_mm,
      SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM,
      SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM,
      SAFETY_LIMIT_BUCKET_POSITIVE_INCREASES_LENGTH);
  sample->y1 = limit_linear_axis(
      sample->y1, feedback->stick_length_hundredths_mm,
      SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM,
      SAFETY_LIMIT_STICK_MAX_HUNDREDTHS_MM,
      SAFETY_LIMIT_STICK_POSITIVE_INCREASES_LENGTH);
  sample->y2 = limit_linear_axis(
      sample->y2, feedback->boom_length_hundredths_mm,
      SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM,
      SAFETY_LIMIT_BOOM_MAX_HUNDREDTHS_MM,
      SAFETY_LIMIT_BOOM_POSITIVE_INCREASES_LENGTH);
}
