#ifndef SPEED_COMMAND_RECEIVER_H
#define SPEED_COMMAND_RECEIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SPEED_COMMAND_FRAME_MAX_LEN 96U
#define SPEED_COMMAND_QUEUE_SLOTS 4U

typedef struct
{
  uint32_t t_ms;
  float v_boom;
  float v_stick;
  float v_bucket;
  float yaw_rate;
} SpeedCommand;

typedef struct
{
  char current[SPEED_COMMAND_FRAME_MAX_LEN];
  char frames[SPEED_COMMAND_QUEUE_SLOTS][SPEED_COMMAND_FRAME_MAX_LEN];
  uint16_t lengths[SPEED_COMMAND_QUEUE_SLOTS];
  uint16_t current_length;
  volatile uint8_t head;
  volatile uint8_t tail;
  uint8_t dropping_frame;
  volatile uint32_t dropped_frames;
} SpeedCommandReceiver;

void SpeedCommandReceiver_Init(SpeedCommandReceiver *receiver);
void SpeedCommandReceiver_FeedFromIsr(SpeedCommandReceiver *receiver,
                                      const uint8_t *data,
                                      uint16_t size);
bool SpeedCommandReceiver_Pop(SpeedCommandReceiver *receiver,
                              char *frame,
                              size_t frame_capacity);
bool SpeedCommandReceiver_ParseFrame(const char *frame,
                                     SpeedCommand *command);

#endif /* SPEED_COMMAND_RECEIVER_H */
