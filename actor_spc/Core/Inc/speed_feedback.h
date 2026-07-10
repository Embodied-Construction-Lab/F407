#ifndef SPEED_FEEDBACK_H
#define SPEED_FEEDBACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  float x1_deg_s;
  float x2_mm_s;
  float y1_mm_s;
  float y2_mm_s;
} SpeedFeedback;

void SpeedFeedback_FromRaw(SpeedFeedback *feedback,
                           float yaw_rate_deg_s,
                           int32_t bucket_speed_hundredths_mm_s,
                           int32_t stick_speed_hundredths_mm_s,
                           int32_t boom_speed_hundredths_mm_s);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_FEEDBACK_H */
