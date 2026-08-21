#include "control_mode_supervisor.h"

#include <stddef.h>

void ControlModeSupervisor_Init(ControlModeSupervisor *supervisor)
{
  if (supervisor != NULL)
  {
    supervisor->active_mode = CONTROL_MODE_SAFE_ZERO;
  }
}

void ControlModeSupervisor_EnterSafeZero(ControlModeSupervisor *supervisor)
{
  if (supervisor != NULL)
  {
    supervisor->active_mode = CONTROL_MODE_SAFE_ZERO;
  }
}

ControlModeDecision ControlModeSupervisor_Apply(
    ControlModeSupervisor *supervisor,
    const ControlCommand *command)
{
  ControlModeDecision decision = {false, false, true};
  ControlMode previous_mode;

  if ((supervisor == NULL) || (command == NULL))
  {
    return decision;
  }
  if ((command->mode != CONTROL_MODE_MANUAL_ACTION) &&
      (command->mode != CONTROL_MODE_VELOCITY_REFERENCE))
  {
    ControlModeSupervisor_EnterSafeZero(supervisor);
    return decision;
  }

  previous_mode = supervisor->active_mode;
  if (command->mode == previous_mode)
  {
    decision.accepted = true;
    decision.force_zero = ControlCommand_IsZero(command);
    return decision;
  }

  if (ControlCommand_IsZero(command))
  {
    supervisor->active_mode = command->mode;
    decision.accepted = true;
    decision.mode_changed = (previous_mode != command->mode);
    return decision;
  }

  supervisor->active_mode = CONTROL_MODE_SAFE_ZERO;
  decision.mode_changed = (previous_mode != CONTROL_MODE_SAFE_ZERO);
  return decision;
}
