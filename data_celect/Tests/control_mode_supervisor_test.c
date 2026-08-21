#include "control_mode_supervisor.h"

#include <assert.h>
#include <string.h>

static ControlCommand command_for(ControlMode mode, float boom)
{
  ControlCommand command;

  memset(&command, 0, sizeof(command));
  command.mode = mode;
  command.axis.boom = boom;
  return command;
}

int main(void)
{
  ControlModeSupervisor supervisor;
  ControlModeDecision decision;
  ControlCommand command;

  ControlModeSupervisor_Init(&supervisor);
  assert(supervisor.active_mode == CONTROL_MODE_SAFE_ZERO);

  command = command_for(CONTROL_MODE_MANUAL_ACTION, 0.4f);
  decision = ControlModeSupervisor_Apply(&supervisor, &command);
  assert(!decision.accepted);
  assert(decision.force_zero);
  assert(supervisor.active_mode == CONTROL_MODE_SAFE_ZERO);

  command = command_for(CONTROL_MODE_MANUAL_ACTION, 0.0f);
  command.manual_z1 = 0.4f;
  decision = ControlModeSupervisor_Apply(&supervisor, &command);
  assert(!decision.accepted);
  assert(decision.force_zero);
  assert(supervisor.active_mode == CONTROL_MODE_SAFE_ZERO);

  command = command_for(CONTROL_MODE_MANUAL_ACTION, 0.0f);
  decision = ControlModeSupervisor_Apply(&supervisor, &command);
  assert(decision.accepted);
  assert(decision.mode_changed);
  assert(decision.force_zero);
  assert(supervisor.active_mode == CONTROL_MODE_MANUAL_ACTION);

  command = command_for(CONTROL_MODE_MANUAL_ACTION, -0.4f);
  decision = ControlModeSupervisor_Apply(&supervisor, &command);
  assert(decision.accepted);
  assert(!decision.mode_changed);
  assert(!decision.force_zero);

  command = command_for(CONTROL_MODE_VELOCITY_REFERENCE, 0.1f);
  decision = ControlModeSupervisor_Apply(&supervisor, &command);
  assert(!decision.accepted);
  assert(decision.force_zero);
  assert(supervisor.active_mode == CONTROL_MODE_SAFE_ZERO);

  command = command_for(CONTROL_MODE_VELOCITY_REFERENCE, 0.0f);
  decision = ControlModeSupervisor_Apply(&supervisor, &command);
  assert(decision.accepted);
  assert(decision.mode_changed);
  assert(decision.force_zero);
  assert(supervisor.active_mode == CONTROL_MODE_VELOCITY_REFERENCE);
  return 0;
}
