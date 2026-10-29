#ifndef IMU_PARSER_H
#define IMU_PARSER_H

#include <stddef.h>
#include <stdint.h>

#define IMU_FRAME_LEN 66U
#define IMU_SYNC_LEN (2U * IMU_FRAME_LEN)

typedef struct
{
  float gz;
  uint64_t ts_ms;
} ImuSample;

typedef struct
{
  uint8_t bytes[IMU_SYNC_LEN];
  size_t length;
  uint8_t locked;
} ImuParser;

void imu_parser_init(ImuParser *parser);
uint8_t imu_parser_feed(ImuParser *parser, const uint8_t *data,
                        size_t length, ImuSample *latest);

#endif
