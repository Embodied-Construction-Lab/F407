#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PID_CONTROLLER_OUTPUT_MIN (-0.9f)
#define PID_CONTROLLER_OUTPUT_MAX 0.9f

typedef struct
{
  float kp;
  float ki;
  float kd;
  float positive_feedforward_gain;
  float negative_feedforward_gain;
  float integral;
  float previous_error;
  uint8_t has_previous_error;
} PidController;

void PidController_Init(PidController *controller,
                        float kp,
                        float ki,
                        float kd,
                        float feedforward_gain);
void PidController_InitAsymmetricFeedforward(PidController *controller,
                                             float kp,
                                             float ki,
                                             float kd,
                                             float positive_feedforward_gain,
                                             float negative_feedforward_gain);
void PidController_Reset(PidController *controller);
float PidController_Update(PidController *controller,
                           float target_speed,
                           float measured_speed,
                           float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
