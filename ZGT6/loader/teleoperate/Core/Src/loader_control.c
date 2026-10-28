#include "loader_control.h"

#include "servo_control.h"

#include <stddef.h>

static float clamp_axis(float axis)
{
  if (axis < -1.0f) axis = -1.0f;
  if (axis > 1.0f) axis = 1.0f;
  if ((axis >= -LOADER_DEAD_ZONE) && (axis <= LOADER_DEAD_ZONE)) axis = 0.0f;
  return axis;
}

static float valve_angle(float axis, float pos_min, float pos_max,
                         float neg_min, float neg_max)
{
  if (axis > 0.0f)
  {
    const float amount = (axis - LOADER_DEAD_ZONE) / (1.0f - LOADER_DEAD_ZONE);
    return pos_min + amount * (pos_max - pos_min);
  }
  if (axis < 0.0f)
  {
    const float amount = (-axis - LOADER_DEAD_ZONE) / (1.0f - LOADER_DEAD_ZONE);
    return neg_max - amount * (neg_max - neg_min);
  }
  return LOADER_VALVE_NEUTRAL_DEG;
}

static void update_counts(LoaderTargets *targets)
{
  targets->pwm_count[LOADER_CH_BOOM] = ServoControl_AngleToPulse(targets->valve_deg[0]);
  targets->pwm_count[LOADER_CH_BUCKET] = ServoControl_AngleToPulse(targets->valve_deg[1]);
  targets->pwm_count[LOADER_CH_STEERING] = ServoControl_AngleToPulse(targets->valve_deg[2]);
  targets->pwm_count[LOADER_CH_DRIVE] = ServoControl_SpeedToPulse(targets->drive_percent);
  targets->pwm_count[LOADER_CH_PUMP] = ServoControl_SpeedToPulse(targets->pump_percent);
}

void LoaderControl_SetNeutral(LoaderTargets *targets)
{
  if (targets == NULL) return;
  targets->valve_deg[0] = LOADER_VALVE_NEUTRAL_DEG;
  targets->valve_deg[1] = LOADER_VALVE_NEUTRAL_DEG;
  targets->valve_deg[2] = LOADER_VALVE_NEUTRAL_DEG;
  targets->drive_percent = 0.0f;
  targets->pump_percent = 0.0f;
  update_counts(targets);
}

void LoaderControl_Compute(const StickData *stick, LoaderTargets *targets)
{
  float x1, x2, y1, y2;
  if ((stick == NULL) || (targets == NULL)) return;
  x1 = clamp_axis(stick->x1);
  x2 = clamp_axis(stick->x2);
  y1 = clamp_axis(stick->y1);
  y2 = clamp_axis(stick->y2);
  targets->valve_deg[0] = valve_angle(y2, LOADER_BOOM_POS_MIN_DEG,
      LOADER_BOOM_POS_MAX_DEG, LOADER_BOOM_NEG_MIN_DEG, LOADER_BOOM_NEG_MAX_DEG);
  targets->valve_deg[1] = valve_angle(x2, LOADER_BUCKET_POS_MIN_DEG,
      LOADER_BUCKET_POS_MAX_DEG, LOADER_BUCKET_NEG_MIN_DEG, LOADER_BUCKET_NEG_MAX_DEG);
  targets->valve_deg[2] = valve_angle(x1, LOADER_STEERING_POS_MIN_DEG,
      LOADER_STEERING_POS_MAX_DEG, LOADER_STEERING_NEG_MIN_DEG,
      LOADER_STEERING_NEG_MAX_DEG);
  targets->drive_percent = y1 * LOADER_DRIVE_MAX_PERCENT;
  targets->pump_percent = ((x1 != 0.0f) || (x2 != 0.0f) || (y2 != 0.0f))
      ? LOADER_FIXED_PUMP_PERCENT : 0.0f;
  update_counts(targets);
}
