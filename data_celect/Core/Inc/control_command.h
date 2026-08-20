#ifndef CONTROL_COMMAND_H
#define CONTROL_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define CONTROL_MANUAL_SCHEMA_VERSION "stm32_manual_command.v1"
#define CONTROL_VELOCITY_SCHEMA_VERSION "stm32_velocity_command.v1"

typedef enum
{
  CONTROL_MODE_SAFE_ZERO = 0,
  CONTROL_MODE_MANUAL_ACTION = 1,
  CONTROL_MODE_VELOCITY_REFERENCE = 2
} ControlMode;

typedef struct
{
  float boom;
  float stick;
  float bucket;
  float swing;
} ControlAxisCommand;

typedef struct
{
  ControlMode mode;
  ControlAxisCommand axis;
  float manual_z1;
  float manual_z2;
  uint32_t command_seq;
  uint32_t command_source_stamp_ms;
} ControlCommand;

bool ControlCommand_ParseJson(const char *frame, ControlCommand *command);
bool ControlCommand_IsZero(const ControlCommand *command);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_COMMAND_H */
