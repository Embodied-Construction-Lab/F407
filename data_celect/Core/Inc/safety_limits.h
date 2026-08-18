#ifndef SAFETY_LIMITS_H
#define SAFETY_LIMITS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM 8000
#define SAFETY_LIMIT_BOOM_MAX_HUNDREDTHS_MM 19000
#define SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM 6000
#define SAFETY_LIMIT_STICK_MAX_HUNDREDTHS_MM 22000
#define SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM 6000
#define SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM 16000
#define SAFETY_LIMIT_SWING_MIN_DEG (-110.0f)
#define SAFETY_LIMIT_SWING_MAX_DEG 110.0f

#define SAFETY_LIMIT_SWING_POSITIVE_INCREASES_YAW 1U
#define SAFETY_LIMIT_BUCKET_POSITIVE_INCREASES_LENGTH 0U
#define SAFETY_LIMIT_STICK_POSITIVE_INCREASES_LENGTH 1U
#define SAFETY_LIMIT_BOOM_POSITIVE_INCREASES_LENGTH 1U

typedef struct
{
  float x1;
  float x2;
  float y1;
  float y2;
} AxisControlSample;

typedef struct
{
  int32_t boom_length_hundredths_mm;
  int32_t stick_length_hundredths_mm;
  int32_t bucket_length_hundredths_mm;
  float yaw_deg;
} SafetyLimitsFeedback;

float SafetyLimits_NormalizeYawDeg(float yaw_deg);
void SafetyLimits_Apply(AxisControlSample *sample,
                        const SafetyLimitsFeedback *feedback);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_LIMITS_H */
