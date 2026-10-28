#include "truck_receiver.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

void TruckReceiver_Init(TruckReceiver *receiver)
{
  if (receiver != NULL)
  {
    memset(receiver, 0, sizeof(*receiver));
  }
}

void TruckReceiver_FeedFromIsr(TruckReceiver *receiver,
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
            (uint8_t)((receiver->head + 1U) % TRUCK_FRAME_QUEUE_SLOTS);

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
    if (receiver->current_length >= (TRUCK_FRAME_MAX_LEN - 1U))
    {
      receiver->dropping_frame = 1U;
      receiver->current_length = 0U;
      receiver->dropped_frames++;
      continue;
    }
    receiver->current[receiver->current_length++] = (char)byte;
  }
}

bool TruckReceiver_Pop(TruckReceiver *receiver,
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
  receiver->tail =
      (uint8_t)((receiver->tail + 1U) % TRUCK_FRAME_QUEUE_SLOTS);
  return true;
}

static bool TruckReceiver_ParseNumber(const char *frame,
                                      const char *key,
                                      float *value)
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

static uint8_t TruckReceiver_FlagFromFloat(float value)
{
  return (value >= 0.5f) ? 1U : 0U;
}

static bool TruckReceiver_ParseFlag(const char *frame,
                                    const char *key,
                                    uint8_t *value)
{
  const char *position;
  float numeric;

  if (TruckReceiver_ParseNumber(frame, key, &numeric))
  {
    *value = TruckReceiver_FlagFromFloat(numeric);
    return true;
  }

  position = strstr(frame, key);
  if (position == NULL)
  {
    return false;
  }
  position += strlen(key);
  while (isspace((unsigned char)*position) != 0)
  {
    ++position;
  }
  if (*position != ':')
  {
    return false;
  }
  ++position;
  while (isspace((unsigned char)*position) != 0)
  {
    ++position;
  }

  if (strncmp(position, "true", 4U) == 0)
  {
    *value = 1U;
    return true;
  }
  if (strncmp(position, "false", 5U) == 0)
  {
    *value = 0U;
    return true;
  }
  return false;
}

bool TruckReceiver_ParseJsonUpdate(const char *frame,
                                   TruckCommand *update,
                                   uint8_t *field_mask)
{
  TruckCommand parsed = {0};
  uint8_t mask = 0U;

  if ((frame == NULL) || (update == NULL) || (field_mask == NULL))
  {
    return false;
  }

  if (strstr(frame, "\"steering\"") != NULL)
  {
    if (!TruckReceiver_ParseNumber(frame, "\"steering\"", &parsed.steering))
    {
      return false;
    }
    mask |= TRUCK_FIELD_STEERING;
  }
  if (strstr(frame, "\"throttle\"") != NULL)
  {
    if (!TruckReceiver_ParseNumber(frame, "\"throttle\"", &parsed.throttle))
    {
      return false;
    }
    mask |= TRUCK_FIELD_THROTTLE;
  }
  if (strstr(frame, "\"brake\"") != NULL)
  {
    if (!TruckReceiver_ParseNumber(frame, "\"brake\"", &parsed.brake))
    {
      return false;
    }
    mask |= TRUCK_FIELD_BRAKE;
  }
  if (strstr(frame, "\"up\"") != NULL)
  {
    if (!TruckReceiver_ParseFlag(frame, "\"up\"", &parsed.up))
    {
      return false;
    }
    mask |= TRUCK_FIELD_UP;
  }
  if (strstr(frame, "\"down\"") != NULL)
  {
    if (!TruckReceiver_ParseFlag(frame, "\"down\"", &parsed.down))
    {
      return false;
    }
    mask |= TRUCK_FIELD_DOWN;
  }

  if (mask == 0U)
  {
    return false;
  }

  *update = parsed;
  *field_mask = mask;
  return true;
}

bool TruckReceiver_ParseJson(const char *frame, TruckCommand *command)
{
  TruckCommand parsed;
  uint8_t field_mask;

  if ((command == NULL) ||
      !TruckReceiver_ParseJsonUpdate(frame, &parsed, &field_mask) ||
      (field_mask != (uint8_t)TRUCK_FIELD_ALL))
  {
    return false;
  }
  *command = parsed;
  return true;
}
