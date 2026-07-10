#include "pid_controller.h"

#include <stddef.h>

static float clamp_output(float value)
{
  if (value > PID_CONTROLLER_OUTPUT_MAX)
  {
    return PID_CONTROLLER_OUTPUT_MAX;
  }
  if (value < PID_CONTROLLER_OUTPUT_MIN)
  {
    return PID_CONTROLLER_OUTPUT_MIN;
  }
  return value;
}

void PidController_Init(PidController *controller,
                        float kp,
                        float ki,
                        float kd,
                        float feedforward_gain)
{
  PidController_InitAsymmetricFeedforward(controller, kp, ki, kd,
                                          feedforward_gain,
                                          feedforward_gain);
}

void PidController_InitAsymmetricFeedforward(PidController *controller,
                                             float kp,
                                             float ki,
                                             float kd,
                                             float positive_feedforward_gain,
                                             float negative_feedforward_gain)
{
  if (controller == NULL)
  {
    return;
  }

  controller->kp = kp;
  controller->ki = ki;
  controller->kd = kd;
  controller->positive_feedforward_gain = positive_feedforward_gain;
  controller->negative_feedforward_gain = negative_feedforward_gain;
  PidController_Reset(controller);
}

void PidController_Reset(PidController *controller)
{
  if (controller == NULL)
  {
    return;
  }

  controller->integral = 0.0f;
  controller->previous_error = 0.0f;
  controller->has_previous_error = 0U;
}

float PidController_Update(PidController *controller,
                           float target_speed,
                           float measured_speed,
                           float dt_s)
{
  float error;
  float derivative;
  float feedforward_gain;
  float output;

  if (controller == NULL)
  {
    return 0.0f;
  }

  if (target_speed == 0.0f)
  {
    PidController_Reset(controller);
    return 0.0f;
  }

  if (dt_s <= 0.0f)
  {
    dt_s = 0.001f;
  }

  error = target_speed - measured_speed;
  controller->integral += error * dt_s;
  derivative = 0.0f;
  if (controller->has_previous_error != 0U)
  {
    derivative = (error - controller->previous_error) / dt_s;
  }
  controller->previous_error = error;
  controller->has_previous_error = 1U;
  feedforward_gain = (target_speed > 0.0f)
                         ? controller->positive_feedforward_gain
                         : controller->negative_feedforward_gain;

  output = (target_speed * feedforward_gain) +
           (controller->kp * error) +
           (controller->ki * controller->integral) +
           (controller->kd * derivative);
  return clamp_output(output);
}
