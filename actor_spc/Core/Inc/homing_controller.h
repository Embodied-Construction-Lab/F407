#ifndef HOMING_CONTROLLER_H
#define HOMING_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "safety_limits.h"

#include <stdint.h>

#ifndef HOMING_CONTROLLER_SPEED
#define HOMING_CONTROLLER_SPEED 0.8f
#endif

#ifndef HOMING_CONTROLLER_BOOM_TARGET_HUNDREDTHS_MM
#define HOMING_CONTROLLER_BOOM_TARGET_HUNDREDTHS_MM 16000
#endif

#ifndef HOMING_CONTROLLER_STICK_TARGET_HUNDREDTHS_MM
#define HOMING_CONTROLLER_STICK_TARGET_HUNDREDTHS_MM 16000
#endif

#ifndef HOMING_CONTROLLER_BUCKET_TARGET_HUNDREDTHS_MM
#define HOMING_CONTROLLER_BUCKET_TARGET_HUNDREDTHS_MM 16000
#endif

#ifndef HOMING_CONTROLLER_LINEAR_TOLERANCE_HUNDREDTHS_MM
#define HOMING_CONTROLLER_LINEAR_TOLERANCE_HUNDREDTHS_MM 500
#endif

#ifndef HOMING_CONTROLLER_YAW_TARGET_DEG
#define HOMING_CONTROLLER_YAW_TARGET_DEG 0.0f
#endif

#ifndef HOMING_CONTROLLER_YAW_TOLERANCE_DEG
#define HOMING_CONTROLLER_YAW_TOLERANCE_DEG 3.0f
#endif

#ifndef HOMING_CONTROLLER_STABLE_TIME_MS
#define HOMING_CONTROLLER_STABLE_TIME_MS 500U
#endif

#ifndef HOMING_CONTROLLER_TIMEOUT_MS
#define HOMING_CONTROLLER_TIMEOUT_MS 20000U
#endif

typedef enum
{
  HOMING_CONTROLLER_WAITING_FEEDBACK = 0,
  HOMING_CONTROLLER_RUNNING,
  HOMING_CONTROLLER_COMPLETE,
  HOMING_CONTROLLER_TIMEOUT
} HomingControllerStatus;

typedef struct
{
  uint32_t start_ms;
  uint32_t stable_since_ms;
  uint8_t stable_timer_active;
  HomingControllerStatus status;
  AxisControlSample sample;
} HomingController;

void HomingController_Init(HomingController *homing, uint32_t start_ms);
HomingControllerStatus HomingController_Update(
    HomingController *homing,
    uint32_t now_ms,
    const SafetyLimitsFeedback *feedback,
    uint8_t feedback_valid);

#ifdef __cplusplus
}
#endif

#endif /* HOMING_CONTROLLER_H */
