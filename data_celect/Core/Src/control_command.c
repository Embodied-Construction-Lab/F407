#include "control_command.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MANUAL_ZERO_DEAD_ZONE 0.15f
#define VELOCITY_ZERO_EPSILON 0.000001f

/* The wire contract is one flat object with exactly one of these field sets. */
static const char *const field_names[] = {
  "schema_version", "command_seq", "command_source_stamp_ms",
  "X1", "Y1", "Z1", "X2", "Y2", "Z2",
  "boom_mps", "stick_mps", "bucket_mps", "swing_radps"
};
#define FIELD_COUNT (sizeof(field_names) / sizeof(field_names[0]))
#define COMMON_FIELDS 0x0007U
#define MANUAL_FIELDS 0x01ffU
#define VELOCITY_FIELDS (COMMON_FIELDS | 0x1e00U)

static void skip_space(const char **cursor)
{
  while ((**cursor == ' ') || (**cursor == '\t') ||
         (**cursor == '\r') || (**cursor == '\n'))
  {
    ++*cursor;
  }
}

static bool read_string(const char **cursor, const char **start, size_t *length)
{
  if (**cursor != '"')
  {
    return false;
  }
  *start = ++*cursor;
  while ((**cursor != '\0') && (**cursor != '"'))
  {
    /* Contract keys/schema names are ASCII literals, never escaped strings. */
    if (((unsigned char)**cursor < 0x20U) || (**cursor == '\\'))
    {
      return false;
    }
    ++*cursor;
  }
  if (**cursor != '"')
  {
    return false;
  }
  *length = (size_t)(*cursor - *start);
  ++*cursor;
  return true;
}

static bool string_equals(const char *start, size_t length, const char *expected)
{
  return (length == strlen(expected)) &&
         (strncmp(start, expected, length) == 0);
}

static bool read_schema(const char **cursor, ControlMode *mode)
{
  const char *start;
  size_t length;
  if (!read_string(cursor, &start, &length))
  {
    return false;
  }
  if (string_equals(start, length, CONTROL_MANUAL_SCHEMA_VERSION))
  {
    *mode = CONTROL_MODE_MANUAL_ACTION;
    return true;
  }
  if (string_equals(start, length, CONTROL_VELOCITY_SCHEMA_VERSION))
  {
    *mode = CONTROL_MODE_VELOCITY_REFERENCE;
    return true;
  }
  return false;
}

static bool digit(char value)
{
  return (value >= '0') && (value <= '9');
}

static bool read_uint32(const char **cursor, uint32_t *value)
{
  uint32_t parsed = 0U;
  const char *start = *cursor;
  if (!digit(**cursor))
  {
    return false;
  }
  do
  {
    uint32_t next = (uint32_t)(**cursor - '0');
    if (parsed > (UINT32_MAX - next) / 10U)
    {
      return false;
    }
    parsed = parsed * 10U + next;
    ++*cursor;
  } while (digit(**cursor));
  if ((*start == '0') && (*cursor - start > 1))
  {
    return false;
  }
  *value = parsed;
  return true;
}

static bool read_float(const char **cursor, float *value)
{
  const char *start = *cursor;
  char *end;
  float parsed;
  if (**cursor == '-')
  {
    ++*cursor;
  }
  if (**cursor == '0')
  {
    ++*cursor;
  }
  else
  {
    if ((**cursor < '1') || (**cursor > '9'))
    {
      return false;
    }
    do { ++*cursor; } while (digit(**cursor));
  }
  if (**cursor == '.')
  {
    ++*cursor;
    if (!digit(**cursor)) { return false; }
    do { ++*cursor; } while (digit(**cursor));
  }
  if ((**cursor == 'e') || (**cursor == 'E'))
  {
    ++*cursor;
    if ((**cursor == '+') || (**cursor == '-')) { ++*cursor; }
    if (!digit(**cursor)) { return false; }
    do { ++*cursor; } while (digit(**cursor));
  }
  errno = 0;
  parsed = strtof(start, &end);
  if ((end != *cursor) || (errno == ERANGE) || !isfinite(parsed))
  {
    return false;
  }
  *value = parsed;
  return true;
}

static bool read_field(const char **cursor, uint32_t *seen,
                       ControlCommand *command, float axes[10])
{
  const char *start;
  size_t length;
  size_t index;
  if (!read_string(cursor, &start, &length)) { return false; }
  for (index = 0U; index < FIELD_COUNT; ++index)
  {
    if (string_equals(start, length, field_names[index])) { break; }
  }
  if ((index == FIELD_COUNT) || ((*seen & (1U << index)) != 0U))
  {
    return false;
  }
  *seen |= 1U << index;
  skip_space(cursor);
  if (**cursor != ':') { return false; }
  ++*cursor;
  skip_space(cursor);
  if (index == 0U) { return read_schema(cursor, &command->mode); }
  if (index == 1U) { return read_uint32(cursor, &command->command_seq); }
  if (index == 2U) { return read_uint32(cursor, &command->command_source_stamp_ms); }
  return read_float(cursor, &axes[index - 3U]);
}

bool ControlCommand_ParseJson(const char *frame, ControlCommand *command)
{
  ControlCommand parsed = {0};
  float axes[10] = {0};
  uint32_t seen = 0U;
  size_t index;
  if ((frame == NULL) || (command == NULL)) { return false; }
  skip_space(&frame);
  if (*frame != '{') { return false; }
  ++frame;
  do
  {
    skip_space(&frame);
    if (!read_field(&frame, &seen, &parsed, axes)) { return false; }
    skip_space(&frame);
    if (*frame == '}') { break; }
    if (*frame != ',') { return false; }
    ++frame;
  } while (true);
  ++frame;
  skip_space(&frame);
  if (*frame != '\0') { return false; }
  if (parsed.mode == CONTROL_MODE_MANUAL_ACTION)
  {
    if (seen != MANUAL_FIELDS) { return false; }
    for (index = 0U; index < 6U; ++index)
    {
      if (fabsf(axes[index]) > 1.0f) { return false; }
    }
    parsed.axis.boom = axes[4];
    parsed.axis.stick = axes[1];
    parsed.axis.bucket = axes[3];
    parsed.axis.swing = axes[0];
    parsed.manual_z1 = axes[2];
    parsed.manual_z2 = axes[5];
  }
  else
  {
    if (seen != VELOCITY_FIELDS) { return false; }
    parsed.axis.boom = axes[6];
    parsed.axis.stick = axes[7];
    parsed.axis.bucket = axes[8];
    parsed.axis.swing = axes[9];
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
