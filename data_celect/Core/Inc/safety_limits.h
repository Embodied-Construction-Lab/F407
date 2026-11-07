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
/* Shared automatic/manual/ACT upper working bound for mode handoff. */
#define SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM 18000
#define SAFETY_LIMIT_SWING_MIN_DEG (-140.0f)
#define SAFETY_LIMIT_SWING_MAX_DEG 140.0f

/* User-selected working bounds from ICRA manual demonstrations, not physical
   hard stops. Other manual bounds retain the existing values above. */
#define MANUAL_LIMIT_STICK_MIN_HUNDREDTHS_MM 4700
#define MANUAL_LIMIT_BUCKET_MIN_HUNDREDTHS_MM 4800
#define MANUAL_LIMIT_BUCKET_MAX_HUNDREDTHS_MM SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM

/* Telemetry bit order is [boom, stick, bucket, swing] in both modes. */
#define SAFETY_LIMIT_MASK_BOOM (1UL << 0)
#define SAFETY_LIMIT_MASK_STICK (1UL << 1)
#define SAFETY_LIMIT_MASK_BUCKET (1UL << 2)
#define SAFETY_LIMIT_MASK_SWING (1UL << 3)

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
/* Velocity mode keeps the existing directional limits and clipped-axis mask. */
uint32_t SafetyLimits_Apply(AxisControlSample *sample,
                            const SafetyLimitsFeedback *feedback);
/* Manual/ACT mode has a separate historical working envelope. */
uint32_t SafetyLimits_ApplyManual(AxisControlSample *sample,
                                  const SafetyLimitsFeedback *feedback);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_LIMITS_H */
