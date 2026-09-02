#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "control_command.h"
#include "pid_controller.h"

#include <stdbool.h>
#include <stdint.h>

#define VELOCITY_LIMIT_BOOM (1UL << 0)
#define VELOCITY_LIMIT_STICK (1UL << 1)
#define VELOCITY_LIMIT_BUCKET (1UL << 2)
#define VELOCITY_LIMIT_SWING (1UL << 3)

typedef struct
{
  int32_t boom_length_hundredths_mm;
  int32_t stick_length_hundredths_mm;
  int32_t bucket_length_hundredths_mm;
  int32_t boom_speed_hundredths_mm_s;
  int32_t stick_speed_hundredths_mm_s;
  int32_t bucket_speed_hundredths_mm_s;
  float swing_unwrapped_deg;
  float swing_speed_deg_s;
} VelocityControlFeedback;

typedef struct
{
  PidController boom_pid;
  PidController stick_pid;
  PidController bucket_pid;
  PidController swing_pid;
} VelocityControl;

typedef struct
{
  /* Internal PID targets: linear mm/s and swing deg/s. */
  ControlAxisCommand target;
  /* Normalized valve actions in [boom, stick, bucket, swing] order. */
  ControlAxisCommand valve_action;
  uint32_t limit_mask;
} VelocityControlOutput;

void VelocityControl_Init(VelocityControl *controller);
void VelocityControl_Reset(VelocityControl *controller);
bool VelocityControl_Update(VelocityControl *controller,
                            const ControlAxisCommand *physical_reference,
                            const VelocityControlFeedback *feedback,
                            float dt_s,
                            VelocityControlOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* VELOCITY_CONTROL_H */
