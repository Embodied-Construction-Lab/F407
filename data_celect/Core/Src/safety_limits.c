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

static uint32_t apply_bounds(AxisControlSample *sample,
                              const SafetyLimitsFeedback *feedback,
                              int32_t stick_min, int32_t bucket_min,
                              int32_t bucket_max)
{
  float yaw_deg;
  AxisControlSample requested;

  if ((sample == NULL) || (feedback == NULL))
  {
    return 0U;
  }
  requested = *sample;
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
      bucket_min, bucket_max,
      SAFETY_LIMIT_BUCKET_POSITIVE_INCREASES_LENGTH);
  sample->y1 = limit_linear_axis(
      sample->y1, feedback->stick_length_hundredths_mm,
      stick_min,
      SAFETY_LIMIT_STICK_MAX_HUNDREDTHS_MM,
      SAFETY_LIMIT_STICK_POSITIVE_INCREASES_LENGTH);
  sample->y2 = limit_linear_axis(
      sample->y2, feedback->boom_length_hundredths_mm,
      SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM,
      SAFETY_LIMIT_BOOM_MAX_HUNDREDTHS_MM,
      SAFETY_LIMIT_BOOM_POSITIVE_INCREASES_LENGTH);
  return ((sample->y2 != requested.y2) ? SAFETY_LIMIT_MASK_BOOM : 0U) |
         ((sample->y1 != requested.y1) ? SAFETY_LIMIT_MASK_STICK : 0U) |
         ((sample->x2 != requested.x2) ? SAFETY_LIMIT_MASK_BUCKET : 0U) |
         ((sample->x1 != requested.x1) ? SAFETY_LIMIT_MASK_SWING : 0U);
}

uint32_t SafetyLimits_Apply(AxisControlSample *sample,
                            const SafetyLimitsFeedback *feedback)
{
  return apply_bounds(sample, feedback, SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM,
                      SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM,
                      SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM);
}

uint32_t SafetyLimits_ApplyManual(AxisControlSample *sample,
                                  const SafetyLimitsFeedback *feedback)
{
  return apply_bounds(sample, feedback, MANUAL_LIMIT_STICK_MIN_HUNDREDTHS_MM,
                      MANUAL_LIMIT_BUCKET_MIN_HUNDREDTHS_MM,
                      MANUAL_LIMIT_BUCKET_MAX_HUNDREDTHS_MM);
}
