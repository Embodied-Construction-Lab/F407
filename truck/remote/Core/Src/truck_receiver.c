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

bool TruckReceiver_ParseJson(const char *frame, TruckCommand *command)
{
  TruckCommand parsed = {0};
  float up;
  float down;

  if ((frame == NULL) || (command == NULL))
  {
    return false;
  }

  if (!TruckReceiver_ParseNumber(frame, "\"steering\"", &parsed.steering) ||
      !TruckReceiver_ParseNumber(frame, "\"throttle\"", &parsed.throttle) ||
      !TruckReceiver_ParseNumber(frame, "\"brake\"", &parsed.brake) ||
      !TruckReceiver_ParseNumber(frame, "\"up\"", &up) ||
      !TruckReceiver_ParseNumber(frame, "\"down\"", &down))
  {
    return false;
  }

  parsed.up = TruckReceiver_FlagFromFloat(up);
  parsed.down = TruckReceiver_FlagFromFloat(down);

  *command = parsed;
  return true;
}
