#include "control_command.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MANUAL_ZERO_DEAD_ZONE 0.15f
#define VELOCITY_ZERO_EPSILON 0.000001f

static bool number_has_json_terminator(const char *end)
{
  while (isspace((unsigned char)*end) != 0)
  {
    ++end;
  }
  return (*end == ',') || (*end == '}');
}

static bool parse_number(const char *frame, const char *key, float *value)
{
  const char *position = strstr(frame, key);
  char *end;
  float parsed;

  if (position == NULL)
  {
    return false;
  }
  position += strlen(key);
  while (isspace((unsigned char)*position) != 0)
  {
    ++position;
  }
  if (*position++ != ':')
  {
    return false;
  }
  while (isspace((unsigned char)*position) != 0)
  {
    ++position;
  }

  errno = 0;
  parsed = strtof(position, &end);
  if ((end == position) || (errno == ERANGE) || !isfinite(parsed) ||
      !number_has_json_terminator(end))
  {
    return false;
  }
  *value = parsed;
  return true;
}

static bool parse_uint32(const char *frame, const char *key, uint32_t *value)
{
  const char *position = strstr(frame, key);
  char *end;
  unsigned long parsed;

  if (position == NULL)
  {
    return false;
  }
  position += strlen(key);
  while (isspace((unsigned char)*position) != 0)
  {
    ++position;
  }
  if (*position++ != ':')
  {
    return false;
  }
  while (isspace((unsigned char)*position) != 0)
  {
    ++position;
  }
  if (*position == '-')
  {
    return false;
  }

  errno = 0;
  parsed = strtoul(position, &end, 10);
  if ((end == position) || (errno == ERANGE) || (parsed > UINT32_MAX) ||
      !number_has_json_terminator(end))
  {
    return false;
  }
  *value = (uint32_t)parsed;
  return true;
}

static bool schema_equals(const char *frame, const char *expected)
{
  const char *position = strstr(frame, "\"schema_version\"");
  const char *value_start;
  const char *value_end;
  size_t value_length;

  if (position == NULL)
  {
    return false;
  }
  position += strlen("\"schema_version\"");
  while (isspace((unsigned char)*position) != 0)
  {
    ++position;
  }
  if (*position++ != ':')
  {
    return false;
  }
  while (isspace((unsigned char)*position) != 0)
  {
    ++position;
  }
  if (*position++ != '"')
  {
    return false;
  }
  value_start = position;
  value_end = strchr(value_start, '"');
  if (value_end == NULL)
  {
    return false;
  }
  value_length = (size_t)(value_end - value_start);
  return (value_length == strlen(expected)) &&
         (strncmp(value_start, expected, value_length) == 0);
}

static bool parse_velocity(const char *frame, ControlCommand *command)
{
  command->mode = CONTROL_MODE_VELOCITY_REFERENCE;
  return parse_number(frame, "\"boom_mps\"", &command->axis.boom) &&
         parse_number(frame, "\"stick_mps\"", &command->axis.stick) &&
         parse_number(frame, "\"bucket_mps\"", &command->axis.bucket) &&
         parse_number(frame, "\"swing_radps\"", &command->axis.swing);
}

static bool parse_manual(const char *frame, ControlCommand *command)
{
  float x1;
  float x2;
  float y1;
  float y2;
  float z1;
  float z2;

  if (!parse_number(frame, "\"X1\"", &x1) ||
      !parse_number(frame, "\"Y1\"", &y1) ||
      !parse_number(frame, "\"Z1\"", &z1) ||
      !parse_number(frame, "\"X2\"", &x2) ||
      !parse_number(frame, "\"Y2\"", &y2) ||
      !parse_number(frame, "\"Z2\"", &z2))
  {
    return false;
  }

  command->mode = CONTROL_MODE_MANUAL_ACTION;
  command->axis.boom = y2;
  command->axis.stick = y1;
  command->axis.bucket = x2;
  command->axis.swing = x1;
  command->manual_z1 = z1;
  command->manual_z2 = z2;
  return true;
}

bool ControlCommand_ParseJson(const char *frame, ControlCommand *command)
{
  ControlCommand parsed;

  if ((frame == NULL) || (command == NULL))
  {
    return false;
  }
  memset(&parsed, 0, sizeof(parsed));

  if (schema_equals(frame, CONTROL_MANUAL_SCHEMA_VERSION))
  {
    if (!parse_manual(frame, &parsed))
    {
      return false;
    }
  }
  else if (schema_equals(frame, CONTROL_VELOCITY_SCHEMA_VERSION))
  {
    if (!parse_velocity(frame, &parsed))
    {
      return false;
    }
  }
  else
  {
    return false;
  }

  if (!parse_uint32(frame, "\"command_seq\"", &parsed.command_seq) ||
      !parse_uint32(frame, "\"command_source_stamp_ms\"",
                    &parsed.command_source_stamp_ms))
  {
    return false;
  }

  *command = parsed;
  return true;
}

static bool axes_within(const ControlAxisCommand *axis, float tolerance)
{
  return (fabsf(axis->boom) <= tolerance) &&
         (fabsf(axis->stick) <= tolerance) &&
         (fabsf(axis->bucket) <= tolerance) &&
         (fabsf(axis->swing) <= tolerance);
}

bool ControlCommand_IsZero(const ControlCommand *command)
{
  if (command == NULL)
  {
    return true;
  }
  if (command->mode == CONTROL_MODE_MANUAL_ACTION)
  {
    return axes_within(&command->axis, MANUAL_ZERO_DEAD_ZONE) &&
           (fabsf(command->manual_z1) <= MANUAL_ZERO_DEAD_ZONE) &&
           (fabsf(command->manual_z2) <= MANUAL_ZERO_DEAD_ZONE);
  }
  if (command->mode == CONTROL_MODE_VELOCITY_REFERENCE)
  {
    return axes_within(&command->axis, VELOCITY_ZERO_EPSILON);
  }
  return true;
}
