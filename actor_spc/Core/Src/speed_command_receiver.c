#include "speed_command_receiver.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

void SpeedCommandReceiver_Init(SpeedCommandReceiver *receiver)
{
  if (receiver != NULL)
  {
    memset(receiver, 0, sizeof(*receiver));
  }
}

void SpeedCommandReceiver_FeedFromIsr(SpeedCommandReceiver *receiver,
                                      const uint8_t *data,
                                      uint16_t size)
{
  uint16_t index;

  if ((receiver == NULL) || (data == NULL))
  {
    return;
  }

  for (index = 0U; index < size; ++index)
  {
    const uint8_t byte = data[index];

    if (byte == (uint8_t)'\r')
    {
      continue;
    }

    if (byte == (uint8_t)'\n')
    {
      if (receiver->dropping_frame != 0U)
      {
        receiver->dropping_frame = 0U;
        receiver->current_length = 0U;
        continue;
      }

      if (receiver->current_length != 0U)
      {
        const uint8_t next_head =
            (uint8_t)((receiver->head + 1U) % SPEED_COMMAND_QUEUE_SLOTS);

        if (next_head == receiver->tail)
        {
          receiver->dropped_frames++;
        }
        else
        {
          memcpy(receiver->frames[receiver->head], receiver->current,
                 receiver->current_length);
          receiver->frames[receiver->head][receiver->current_length] = '\0';
          receiver->lengths[receiver->head] = receiver->current_length;
          receiver->head = next_head;
        }
      }
      receiver->current_length = 0U;
      continue;
    }

    if (receiver->dropping_frame != 0U)
    {
      continue;
    }

    if (receiver->current_length >= (SPEED_COMMAND_FRAME_MAX_LEN - 1U))
    {
      receiver->dropping_frame = 1U;
      receiver->current_length = 0U;
      receiver->dropped_frames++;
      continue;
    }

    receiver->current[receiver->current_length++] = (char)byte;
  }
}

bool SpeedCommandReceiver_Pop(SpeedCommandReceiver *receiver,
                              char *frame,
                              size_t frame_capacity)
{
  uint16_t length;

  if ((receiver == NULL) || (frame == NULL) ||
      (receiver->tail == receiver->head))
  {
    return false;
  }

  length = receiver->lengths[receiver->tail];
  if (frame_capacity <= (size_t)length)
  {
    return false;
  }

  memcpy(frame, receiver->frames[receiver->tail], length + 1U);
  receiver->tail = (uint8_t)((receiver->tail + 1U) % SPEED_COMMAND_QUEUE_SLOTS);
  return true;
}

static void skip_spaces(const char **cursor)
{
  while (isspace((unsigned char)**cursor) != 0)
  {
    ++(*cursor);
  }
}

static bool consume_separator(const char **cursor)
{
  skip_spaces(cursor);
  if (**cursor != ';')
  {
    return false;
  }
  ++(*cursor);
  return true;
}

static bool parse_u32_field(const char **cursor, uint32_t *value)
{
  char *end;
  unsigned long parsed;

  skip_spaces(cursor);
  if (**cursor == '-')
  {
    return false;
  }

  errno = 0;
  parsed = strtoul(*cursor, &end, 10);
  if ((end == *cursor) || (errno == ERANGE) ||
      (parsed > (unsigned long)UINT32_MAX))
  {
    return false;
  }

  *cursor = end;
  *value = (uint32_t)parsed;
  return consume_separator(cursor);
}

static bool parse_float_field(const char **cursor,
                              float *value,
                              uint8_t require_separator)
{
  char *end;
  float parsed;

  skip_spaces(cursor);
  errno = 0;
  parsed = strtof(*cursor, &end);
  if ((end == *cursor) || (errno == ERANGE) || !isfinite(parsed))
  {
    return false;
  }

  *cursor = end;
  *value = parsed;
  if (require_separator != 0U)
  {
    return consume_separator(cursor);
  }
  return true;
}

bool SpeedCommandReceiver_ParseFrame(const char *frame,
                                     SpeedCommand *command)
{
  SpeedCommand parsed;
  const char *cursor;

  if ((frame == NULL) || (command == NULL))
  {
    return false;
  }

  cursor = frame;
  if (!parse_u32_field(&cursor, &parsed.t_ms) ||
      !parse_float_field(&cursor, &parsed.v_boom, 1U) ||
      !parse_float_field(&cursor, &parsed.v_stick, 1U) ||
      !parse_float_field(&cursor, &parsed.v_bucket, 1U) ||
      !parse_float_field(&cursor, &parsed.yaw_rate, 0U))
  {
    return false;
  }

  skip_spaces(&cursor);
  if (*cursor == ';')
  {
    ++cursor;
    skip_spaces(&cursor);
  }
  if (*cursor != '\0')
  {
    return false;
  }

  *command = parsed;
  return true;
}
