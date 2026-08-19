#include "stick_receiver.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

void StickReceiver_Init(StickReceiver *receiver)
{
  if (receiver != NULL)
  {
    memset(receiver, 0, sizeof(*receiver));
  }
}

void StickReceiver_FeedFromIsr(StickReceiver *receiver,
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
            (uint8_t)((receiver->head + 1U) % STICK_QUEUE_SLOTS);

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

    if (receiver->current_length >= (STICK_FRAME_MAX_LEN - 1U))
    {
      receiver->dropping_frame = 1U;
      receiver->current_length = 0U;
      receiver->dropped_frames++;
      continue;
    }

    receiver->current[receiver->current_length++] = (char)byte;
  }
}

bool StickReceiver_Pop(StickReceiver *receiver,
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
  receiver->tail = (uint8_t)((receiver->tail + 1U) % STICK_QUEUE_SLOTS);
  return true;
}

static bool parse_number(const char *frame, const char *key, float *value)
{
  const char *position = frame;
  const size_t key_length = strlen(key);

  while ((position = strstr(position, key)) != NULL)
  {
    char *end;
    float parsed;

    position += key_length;
    while (isspace((unsigned char)*position) != 0)
    {
      ++position;
    }
    if (*position != ':')
    {
      continue;
    }
    ++position;
    while (isspace((unsigned char)*position) != 0)
    {
      ++position;
    }

    errno = 0;
    parsed = strtof(position, &end);
    if ((end == position) || (errno == ERANGE) || !isfinite(parsed))
    {
      return false;
    }

    *value = parsed;
    return true;
  }

  return false;
}

static bool parse_uint32(const char *frame, const char *key, uint32_t *value)
{
  const char *position = frame;
  const size_t key_length = strlen(key);

  while ((position = strstr(position, key)) != NULL)
  {
    char *end;
    unsigned long parsed;

    position += key_length;
    while (isspace((unsigned char)*position) != 0)
    {
      ++position;
    }
    if (*position != ':')
    {
      continue;
    }
    ++position;
    while (isspace((unsigned char)*position) != 0)
    {
      ++position;
    }

    errno = 0;
    parsed = strtoul(position, &end, 10);
    if ((end == position) || (errno == ERANGE) ||
        (parsed > UINT32_MAX))
    {
      return false;
    }

    *value = (uint32_t)parsed;
    return true;
  }

  return false;
}

static bool parse_direct_data(const char *frame, StickData *stick)
{
  return parse_number(frame, "\"X1\"", &stick->x1) &&
         parse_number(frame, "\"Y1\"", &stick->y1) &&
         parse_number(frame, "\"Z1\"", &stick->z1) &&
         parse_number(frame, "\"X2\"", &stick->x2) &&
         parse_number(frame, "\"Y2\"", &stick->y2) &&
         parse_number(frame, "\"Z2\"", &stick->z2);
}

static bool schema_is_supported(const char *frame)
{
  const char *schema_key = "\"schema_version\"";
  const char *position = strstr(frame, schema_key);
  const char *value_start;
  const char *value_end;
  size_t value_length;

  if (position == NULL)
  {
    return false;
  }
  position += strlen(schema_key);
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
  return (value_length == strlen(STICK_COMMAND_SCHEMA_VERSION)) &&
         (strncmp(value_start, STICK_COMMAND_SCHEMA_VERSION, value_length) == 0);
}

bool StickReceiver_ParseJson(const char *frame, StickData *stick)
{
  StickData parsed;

  if ((frame == NULL) || (stick == NULL))
  {
    return false;
  }

  memset(&parsed, 0, sizeof(parsed));
  if (!schema_is_supported(frame) || !parse_direct_data(frame, &parsed))
  {
    return false;
  }

  parsed.has_command_seq =
      parse_uint32(frame, "\"command_seq\"", &parsed.command_seq) ? 1U : 0U;
  parsed.has_command_source_stamp_ms =
      parse_uint32(frame, "\"command_source_stamp_ms\"",
                   &parsed.command_source_stamp_ms) ? 1U : 0U;

  if ((parsed.has_command_seq == 0U) ||
      (parsed.has_command_source_stamp_ms == 0U))
  {
    return false;
  }

  *stick = parsed;
  return true;
}

bool StickReceiver_IsNewSequence(uint32_t candidate,
                                 uint32_t previous,
                                 bool has_previous)
{
  return !has_previous || ((int32_t)(candidate - previous) > 0);
}
