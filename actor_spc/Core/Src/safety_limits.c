#include "safety_limits.h"

#include <stddef.h>

static float limit_linear_axis(float command,
                               int32_t position_hundredths_mm,
                               int32_t min_hundredths_mm,
                               int32_t max_hundredths_mm,
                               uint8_t positive_increases_length)
{
  float increasing_command;

  increasing_command = (positive_increases_length != 0U) ? command : -command;

  if ((position_hundredths_mm >= max_hundredths_mm) &&
      (increasing_command > 0.0f))
  {
    return 0.0f;
  }
  if ((position_hundredths_mm <= min_hundredths_mm) &&
      (increasing_command < 0.0f))
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

void SafetyLimits_ApplyAxisEnable(AxisControlSample *sample)
{
  if (sample == NULL)
  {
    return;
  }

  if (SAFETY_LIMIT_AXIS_X1_ENABLED == 0U)
  {
    sample->x1 = 0.0f;
  }
  if (SAFETY_LIMIT_AXIS_X2_ENABLED == 0U)
  {
    sample->x2 = 0.0f;
  }
  if (SAFETY_LIMIT_AXIS_Y1_ENABLED == 0U)
  {
    sample->y1 = 0.0f;
  }
  if (SAFETY_LIMIT_AXIS_Y2_ENABLED == 0U)
  {
    sample->y2 = 0.0f;
  }
}

void SafetyLimits_Apply(AxisControlSample *sample,
                        const SafetyLimitsFeedback *feedback)
{
  float yaw_deg;

  if ((sample == NULL) || (feedback == NULL))
  {
    return;
  }

  SafetyLimits_ApplyAxisEnable(sample);

  yaw_deg = SafetyLimits_NormalizeYawDeg(feedback->yaw_deg);
  if (SAFETY_LIMIT_SWING_POSITIVE_INCREASES_YAW == 0U)
  {
    yaw_deg = -yaw_deg;
  }

  if ((yaw_deg >= SAFETY_LIMIT_SWING_MAX_DEG) && (sample->x1 > 0.0f))
  {
    sample->x1 = 0.0f;
  }
  else if ((yaw_deg <= SAFETY_LIMIT_SWING_MIN_DEG) && (sample->x1 < 0.0f))
  {
    sample->x1 = 0.0f;
  }

  sample->x2 = limit_linear_axis(sample->x2,
                                 feedback->bucket_length_hundredths_mm,
                                 SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM,
                                 SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM,
                                 SAFETY_LIMIT_BUCKET_POSITIVE_INCREASES_LENGTH);
  sample->y1 = limit_linear_axis(sample->y1,
                                 feedback->stick_length_hundredths_mm,
                                 SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM,
                                 SAFETY_LIMIT_STICK_MAX_HUNDREDTHS_MM,
                                 SAFETY_LIMIT_STICK_POSITIVE_INCREASES_LENGTH);
  sample->y2 = limit_linear_axis(sample->y2,
                                 feedback->boom_length_hundredths_mm,
                                 SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM,
                                 SAFETY_LIMIT_BOOM_MAX_HUNDREDTHS_MM,
                                 SAFETY_LIMIT_BOOM_POSITIVE_INCREASES_LENGTH);
  SafetyLimits_ApplyAxisEnable(sample);
}
