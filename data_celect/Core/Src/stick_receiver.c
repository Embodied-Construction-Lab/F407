#include "stick_receiver.h"

#include "control_command.h"

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

bool StickReceiver_ParseJson(const char *frame, StickData *stick)
{
  ControlCommand command;

  if ((frame == NULL) || (stick == NULL))
  {
    return false;
  }
  if (!ControlCommand_ParseJson(frame, &command) ||
      (command.mode != CONTROL_MODE_MANUAL_ACTION))
  {
    return false;
  }
  memset(stick, 0, sizeof(*stick));
  stick->x1 = command.axis.swing;
  stick->x2 = command.axis.bucket;
  stick->y1 = command.axis.stick;
  stick->y2 = command.axis.boom;
  stick->command_seq = command.command_seq;
  stick->command_source_stamp_ms = command.command_source_stamp_ms;
  stick->has_command_seq = 1U;
  stick->has_command_source_stamp_ms = 1U;
  return true;
}

bool StickReceiver_IsNewSequence(uint32_t candidate,
                                 uint32_t previous,
                                 bool has_previous)
{
  return !has_previous || ((int32_t)(candidate - previous) > 0);
}
