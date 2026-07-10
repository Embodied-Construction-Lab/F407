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

static void get_linear_axis_profile_feedback(
    int32_t position_hundredths_mm,
    int32_t min_hundredths_mm,
    int32_t max_hundredths_mm,
    uint8_t positive_increases_length,
    JoystickProfileFeedback *profile_feedback)
{
  uint8_t at_physical_min;
  uint8_t at_physical_max;

  at_physical_min =
      (position_hundredths_mm <= min_hundredths_mm) ? 1U : 0U;
  at_physical_max =
      (position_hundredths_mm >= max_hundredths_mm) ? 1U : 0U;

  if (positive_increases_length != 0U)
  {
    profile_feedback->selected_axis_at_min = at_physical_min;
    profile_feedback->selected_axis_at_max = at_physical_max;
  }
  else
  {
    profile_feedback->selected_axis_at_min = at_physical_max;
    profile_feedback->selected_axis_at_max = at_physical_min;
  }
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

void SafetyLimits_Apply(JoystickProfileSample *sample,
                        const SafetyLimitsFeedback *feedback)
{
  float yaw_deg;

  if ((sample == NULL) || (feedback == NULL))
  {
    return;
  }

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
}

void SafetyLimits_GetJoystickProfileFeedback(
    JoystickProfileAxis axis,
    const SafetyLimitsFeedback *safety_feedback,
    JoystickProfileFeedback *profile_feedback)
{
  float yaw_deg;

  if (profile_feedback == NULL)
  {
    return;
  }

  profile_feedback->selected_axis_at_min = 0U;
  profile_feedback->selected_axis_at_max = 0U;
  if (safety_feedback == NULL)
  {
    return;
  }

  if (axis == JOYSTICK_PROFILE_AXIS_X1)
  {
    yaw_deg = SafetyLimits_NormalizeYawDeg(safety_feedback->yaw_deg);
    if (SAFETY_LIMIT_SWING_POSITIVE_INCREASES_YAW == 0U)
    {
      yaw_deg = -yaw_deg;
    }
    profile_feedback->selected_axis_at_min =
        (yaw_deg <= SAFETY_LIMIT_SWING_MIN_DEG) ? 1U : 0U;
    profile_feedback->selected_axis_at_max =
        (yaw_deg >= SAFETY_LIMIT_SWING_MAX_DEG) ? 1U : 0U;
  }
  else if (axis == JOYSTICK_PROFILE_AXIS_X2)
  {
    get_linear_axis_profile_feedback(
        safety_feedback->bucket_length_hundredths_mm,
        SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM,
        SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM,
        SAFETY_LIMIT_BUCKET_POSITIVE_INCREASES_LENGTH,
        profile_feedback);
  }
  else if (axis == JOYSTICK_PROFILE_AXIS_Y1)
  {
    get_linear_axis_profile_feedback(
        safety_feedback->stick_length_hundredths_mm,
        SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM,
        SAFETY_LIMIT_STICK_MAX_HUNDREDTHS_MM,
        SAFETY_LIMIT_STICK_POSITIVE_INCREASES_LENGTH,
        profile_feedback);
  }
  else if (axis == JOYSTICK_PROFILE_AXIS_Y2)
  {
    get_linear_axis_profile_feedback(
        safety_feedback->boom_length_hundredths_mm,
        SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM,
        SAFETY_LIMIT_BOOM_MAX_HUNDREDTHS_MM,
        SAFETY_LIMIT_BOOM_POSITIVE_INCREASES_LENGTH,
        profile_feedback);
  }
}
