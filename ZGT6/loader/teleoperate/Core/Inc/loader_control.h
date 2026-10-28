#ifndef LOADER_CONTROL_H
#define LOADER_CONTROL_H

#include "stick_receiver.h"

#include <stdint.h>

/* PCA9685 output channels; adjust these limits only after hardware calibration. */
#define LOADER_CH_BOOM      0U
#define LOADER_CH_BUCKET    1U
#define LOADER_CH_STEERING  2U
#define LOADER_CH_DRIVE     3U
#define LOADER_CH_PUMP      4U
#define LOADER_CHANNEL_COUNT 5U

#define LOADER_DEAD_ZONE 0.15f
#define LOADER_CONTROL_TIMEOUT_MS 300U
#define LOADER_VALVE_NEUTRAL_DEG 90.0f

#define LOADER_BOOM_POS_MIN_DEG 110.0f
#define LOADER_BOOM_POS_MAX_DEG 115.0f
#define LOADER_BOOM_NEG_MIN_DEG 73.0f
#define LOADER_BOOM_NEG_MAX_DEG 78.0f
#define LOADER_BUCKET_POS_MIN_DEG 105.0f
#define LOADER_BUCKET_POS_MAX_DEG 110.0f
#define LOADER_BUCKET_NEG_MIN_DEG 71.0f
#define LOADER_BUCKET_NEG_MAX_DEG 76.0f
#define LOADER_STEERING_POS_MIN_DEG 105.0f
#define LOADER_STEERING_POS_MAX_DEG 112.0f
#define LOADER_STEERING_NEG_MIN_DEG 70.0f
#define LOADER_STEERING_NEG_MAX_DEG 78.0f

#define LOADER_DRIVE_MAX_PERCENT 20.0f
#define LOADER_FIXED_PUMP_PERCENT (-30.0f)

typedef struct
{
  float valve_deg[3];
  float drive_percent;
  float pump_percent;
  uint16_t pwm_count[LOADER_CHANNEL_COUNT];
} LoaderTargets;

void LoaderControl_SetNeutral(LoaderTargets *targets);
void LoaderControl_Compute(const StickData *stick, LoaderTargets *targets);

#endif
