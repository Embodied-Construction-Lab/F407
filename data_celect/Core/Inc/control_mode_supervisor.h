#ifndef CONTROL_MODE_SUPERVISOR_H
#define CONTROL_MODE_SUPERVISOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "control_command.h"

#include <stdbool.h>

typedef struct
{
  ControlMode active_mode;
} ControlModeSupervisor;

typedef struct
{
  bool accepted;
  bool mode_changed;
  bool force_zero;
} ControlModeDecision;

void ControlModeSupervisor_Init(ControlModeSupervisor *supervisor);
ControlModeDecision ControlModeSupervisor_Apply(
    ControlModeSupervisor *supervisor,
    const ControlCommand *command);
void ControlModeSupervisor_EnterSafeZero(ControlModeSupervisor *supervisor);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_MODE_SUPERVISOR_H */
