#ifndef SAFETY_LIMITS_H
#define SAFETY_LIMITS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM 14000
#define SAFETY_LIMIT_BOOM_MAX_HUNDREDTHS_MM 19000
#define SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM 6000
#define SAFETY_LIMIT_STICK_MAX_HUNDREDTHS_MM 22000
#define SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM 6000
#define SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM 16000

#define SAFETY_LIMIT_SWING_MIN_DEG (-60.0f)
#define SAFETY_LIMIT_SWING_MAX_DEG 60.0f

/*回中方向   */
#define SAFETY_LIMIT_SWING_POSITIVE_INCREASES_YAW 1U
#define SAFETY_LIMIT_BUCKET_POSITIVE_INCREASES_LENGTH 0U
#define SAFETY_LIMIT_STICK_POSITIVE_INCREASES_LENGTH 1U
#define SAFETY_LIMIT_BOOM_POSITIVE_INCREASES_LENGTH 1U

#ifndef SAFETY_LIMIT_AXIS_X1_ENABLED
#define SAFETY_LIMIT_AXIS_X1_ENABLED 1U
#endif

#ifndef SAFETY_LIMIT_AXIS_X2_ENABLED
#define SAFETY_LIMIT_AXIS_X2_ENABLED 1U
#endif

#ifndef SAFETY_LIMIT_AXIS_Y1_ENABLED
#define SAFETY_LIMIT_AXIS_Y1_ENABLED 1U
#endif

#ifndef SAFETY_LIMIT_AXIS_Y2_ENABLED
#define SAFETY_LIMIT_AXIS_Y2_ENABLED 1U
#endif

typedef struct
{
  float x1; /* swing target speed/output */
  float x2; /* bucket target speed/output */
  float y1; /* small arm target speed/output */
  float y2; /* big arm target speed/output */
} AxisControlSample;

typedef struct
{
  int32_t boom_length_hundredths_mm;
  int32_t stick_length_hundredths_mm;
  int32_t bucket_length_hundredths_mm;
  float yaw_deg;
} SafetyLimitsFeedback;

float SafetyLimits_NormalizeYawDeg(float yaw_deg);
void SafetyLimits_ApplyAxisEnable(AxisControlSample *sample);
void SafetyLimits_Apply(AxisControlSample *sample,
                        const SafetyLimitsFeedback *feedback);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_LIMITS_H */
