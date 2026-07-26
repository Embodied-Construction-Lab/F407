#ifndef TRUCK_RECEIVER_H
#define TRUCK_RECEIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TRUCK_FRAME_MAX_LEN 256U
#define TRUCK_FRAME_QUEUE_SLOTS 16U

typedef struct
{
  float steering;
  float drive_axis;
  float lift_axis;
} TruckCommand;

typedef struct
{
  char current[TRUCK_FRAME_MAX_LEN];
  char frames[TRUCK_FRAME_QUEUE_SLOTS][TRUCK_FRAME_MAX_LEN];
  uint16_t lengths[TRUCK_FRAME_QUEUE_SLOTS];
  uint16_t current_length;
  volatile uint8_t head;
  volatile uint8_t tail;
  uint8_t dropping_frame;
  volatile uint32_t dropped_frames;
} TruckReceiver;

void TruckReceiver_Init(TruckReceiver *receiver);
void TruckReceiver_FeedFromIsr(TruckReceiver *receiver,
                               const uint8_t *data,
                               uint16_t size);
bool TruckReceiver_Pop(TruckReceiver *receiver,
                       char *frame,
                       size_t frame_capacity);
bool TruckReceiver_ParseJson(const char *frame, TruckCommand *command);

#endif /* TRUCK_RECEIVER_H */
