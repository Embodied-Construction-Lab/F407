#include "speed_feedback.h"

#include <stddef.h>

void SpeedFeedback_FromRaw(SpeedFeedback *feedback,
                           float yaw_rate_deg_s,
                           int32_t bucket_speed_hundredths_mm_s,
                           int32_t stick_speed_hundredths_mm_s,
                           int32_t boom_speed_hundredths_mm_s)
{
  if (feedback == NULL)
  {
    return;
  }

  feedback->x1_deg_s = yaw_rate_deg_s;
  feedback->x2_mm_s = -(float)bucket_speed_hundredths_mm_s / 100.0f;
  feedback->y1_mm_s = (float)stick_speed_hundredths_mm_s / 100.0f;
  feedback->y2_mm_s = (float)boom_speed_hundredths_mm_s / 100.0f;
}
