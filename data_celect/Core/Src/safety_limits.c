#include "safety_limits.h"

#include <math.h>
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

float SafetyLimits_LimitSwingCommand(float command,
                                     float unwrapped_yaw_deg)
{
  float increasing_yaw_command;

  if (!isfinite(command) || !isfinite(unwrapped_yaw_deg))
  {
    return 0.0f;
  }
  increasing_yaw_command =
      (SAFETY_LIMIT_SWING_POSITIVE_INCREASES_YAW != 0U) ? command : -command;
  if ((unwrapped_yaw_deg >= SAFETY_LIMIT_SWING_MAX_DEG) &&
      (increasing_yaw_command > 0.0f))
  {
    return 0.0f;
  }
  if ((unwrapped_yaw_deg <= SAFETY_LIMIT_SWING_MIN_DEG) &&
      (increasing_yaw_command < 0.0f))
  {
    return 0.0f;
  }
  return command;
}

void SafetyLimits_Apply(AxisControlSample *sample,
                        const SafetyLimitsFeedback *feedback)
{
  if ((sample == NULL) || (feedback == NULL))
  {
    return;
  }
  sample->x1 = SafetyLimits_LimitSwingCommand(sample->x1,
                                               feedback->swing_unwrapped_deg);

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
