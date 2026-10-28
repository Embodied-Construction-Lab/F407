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
  float throttle;
  float brake;
  uint8_t up;
  uint8_t down;
} TruckCommand;

typedef enum
{
  TRUCK_FIELD_STEERING = (1U << 0),
  TRUCK_FIELD_THROTTLE = (1U << 1),
  TRUCK_FIELD_BRAKE = (1U << 2),
  TRUCK_FIELD_UP = (1U << 3),
  TRUCK_FIELD_DOWN = (1U << 4),
  TRUCK_FIELD_ALL = TRUCK_FIELD_STEERING | TRUCK_FIELD_THROTTLE |
                    TRUCK_FIELD_BRAKE | TRUCK_FIELD_UP |
                    TRUCK_FIELD_DOWN
} TruckCommandField;

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
bool TruckReceiver_ParseJsonUpdate(const char *frame,
                                   TruckCommand *update,
                                   uint8_t *field_mask);

#endif /* TRUCK_RECEIVER_H */
