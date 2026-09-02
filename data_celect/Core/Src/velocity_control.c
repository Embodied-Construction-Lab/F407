#include "velocity_control.h"

#include "safety_limits.h"
#include "speed_pid_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define METRES_TO_MILLIMETRES 1000.0f
#define RADIANS_TO_DEGREES 57.2957795f
#define SWING_REFERENCE_MAX_RAD_S 0.34f

static float clamp_swing_reference(float value)
{
  if (value > SWING_REFERENCE_MAX_RAD_S)
  {
    return SWING_REFERENCE_MAX_RAD_S;
  }
  if (value < -SWING_REFERENCE_MAX_RAD_S)
  {
    return -SWING_REFERENCE_MAX_RAD_S;
  }
  return value;
}

void VelocityControl_Init(VelocityControl *controller)
{
  if (controller == NULL)
  {
    return;
  }
  PidController_InitAsymmetricFeedforward(
      &controller->boom_pid, SPEED_PID_BOOM_KP, SPEED_PID_BOOM_KI,
      SPEED_PID_BOOM_KD, SPEED_PID_BOOM_POSITIVE_FEEDFORWARD,
      SPEED_PID_BOOM_NEGATIVE_FEEDFORWARD);
  PidController_InitAsymmetricFeedforward(
      &controller->stick_pid, SPEED_PID_STICK_KP, SPEED_PID_STICK_KI,
      SPEED_PID_STICK_KD, SPEED_PID_STICK_POSITIVE_FEEDFORWARD,
      SPEED_PID_STICK_NEGATIVE_FEEDFORWARD);
  PidController_InitAsymmetricFeedforward(
      &controller->bucket_pid, SPEED_PID_BUCKET_KP, SPEED_PID_BUCKET_KI,
      SPEED_PID_BUCKET_KD, SPEED_PID_BUCKET_POSITIVE_FEEDFORWARD,
      SPEED_PID_BUCKET_NEGATIVE_FEEDFORWARD);
  PidController_Init(&controller->swing_pid, SPEED_PID_SWING_KP,
                     SPEED_PID_SWING_KI, SPEED_PID_SWING_KD,
                     SPEED_PID_SWING_FEEDFORWARD);
}

void VelocityControl_Reset(VelocityControl *controller)
{
  if (controller == NULL)
  {
    return;
  }
  PidController_Reset(&controller->boom_pid);
  PidController_Reset(&controller->stick_pid);
  PidController_Reset(&controller->bucket_pid);
  PidController_Reset(&controller->swing_pid);
}

static AxisControlSample target_from_physical(
    const ControlAxisCommand *reference)
{
  AxisControlSample target;

  /* Preserve actor_spc's calibrated physical-to-valve sign contract. */
  target.y2 = -reference->boom * METRES_TO_MILLIMETRES;
  target.y1 = -reference->stick * METRES_TO_MILLIMETRES;
  target.x2 = reference->bucket * METRES_TO_MILLIMETRES;
  target.x1 = clamp_swing_reference(reference->swing) * RADIANS_TO_DEGREES;
  return target;
}

static uint32_t changed_axis_mask(const AxisControlSample *before,
                                  const AxisControlSample *after)
{
  uint32_t mask = 0U;

  if (before->y2 != after->y2)
  {
    mask |= VELOCITY_LIMIT_BOOM;
  }
  if (before->y1 != after->y1)
  {
    mask |= VELOCITY_LIMIT_STICK;
  }
  if (before->x2 != after->x2)
  {
    mask |= VELOCITY_LIMIT_BUCKET;
  }
  if (before->x1 != after->x1)
  {
    mask |= VELOCITY_LIMIT_SWING;
  }
  return mask;
}

bool VelocityControl_Update(VelocityControl *controller,
                            const ControlAxisCommand *physical_reference,
                            const VelocityControlFeedback *feedback,
                            float dt_s,
                            VelocityControlOutput *output)
{
  AxisControlSample requested_target;
  AxisControlSample limited_target;
  AxisControlSample requested_output;
  AxisControlSample limited_output;
  SafetyLimitsFeedback limits_feedback;

  if ((controller == NULL) || (physical_reference == NULL) ||
      (feedback == NULL) || (output == NULL) ||
      !isfinite(physical_reference->boom) ||
      !isfinite(physical_reference->stick) ||
      !isfinite(physical_reference->bucket) ||
      !isfinite(physical_reference->swing) || (dt_s <= 0.0f))
  {
    return false;
  }

  requested_target = target_from_physical(physical_reference);
  limited_target = requested_target;
  limits_feedback.boom_length_hundredths_mm =
      feedback->boom_length_hundredths_mm;
  limits_feedback.stick_length_hundredths_mm =
      feedback->stick_length_hundredths_mm;
  limits_feedback.bucket_length_hundredths_mm =
      feedback->bucket_length_hundredths_mm;
  limits_feedback.swing_unwrapped_deg = feedback->swing_unwrapped_deg;
  SafetyLimits_Apply(&limited_target, &limits_feedback);

  requested_output.y2 = PidController_Update(
      &controller->boom_pid, limited_target.y2,
      (float)feedback->boom_speed_hundredths_mm_s / 100.0f, dt_s);
  requested_output.y1 = PidController_Update(
      &controller->stick_pid, limited_target.y1,
      (float)feedback->stick_speed_hundredths_mm_s / 100.0f, dt_s);
  requested_output.x2 = PidController_Update(
      &controller->bucket_pid, limited_target.x2,
      -(float)feedback->bucket_speed_hundredths_mm_s / 100.0f, dt_s);
  requested_output.x1 = PidController_Update(
      &controller->swing_pid, limited_target.x1,
      feedback->swing_speed_deg_s, dt_s);
  limited_output = requested_output;
  SafetyLimits_Apply(&limited_output, &limits_feedback);

  memset(output, 0, sizeof(*output));
  output->target.boom = limited_target.y2;
  output->target.stick = limited_target.y1;
  output->target.bucket = limited_target.x2;
  output->target.swing = limited_target.x1;
  output->valve_action.boom = limited_output.y2;
  output->valve_action.stick = limited_output.y1;
  output->valve_action.bucket = limited_output.x2;
  output->valve_action.swing = limited_output.x1;
  output->limit_mask = changed_axis_mask(&requested_target, &limited_target) |
                       changed_axis_mask(&requested_output, &limited_output);
  return true;
}
