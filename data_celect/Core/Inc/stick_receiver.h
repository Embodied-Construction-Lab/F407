#ifndef STICK_RECEIVER_H
#define STICK_RECEIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define STICK_FRAME_MAX_LEN 384U
#define STICK_QUEUE_SLOTS 4U

typedef struct
{
  float x1;
  float x2;
  float y1;
  float y2;
  float z1;
  float z2;
  uint32_t command_seq;
  uint32_t command_source_stamp_ms;
  uint8_t has_command_seq;
  uint8_t has_command_source_stamp_ms;
} StickData;

typedef struct
{
  char current[STICK_FRAME_MAX_LEN];
  char frames[STICK_QUEUE_SLOTS][STICK_FRAME_MAX_LEN];
  uint16_t lengths[STICK_QUEUE_SLOTS];
  uint16_t current_length;
  volatile uint8_t head;
  volatile uint8_t tail;
  uint8_t dropping_frame;
  volatile uint32_t dropped_frames;
} StickReceiver;

void StickReceiver_Init(StickReceiver *receiver);
void StickReceiver_FeedFromIsr(StickReceiver *receiver,
                               const uint8_t *data,
                               uint16_t size);
bool StickReceiver_Pop(StickReceiver *receiver,
                       char *frame,
                       size_t frame_capacity);
bool StickReceiver_ParseJson(const char *frame, StickData *stick);

#endif /* STICK_RECEIVER_H */
