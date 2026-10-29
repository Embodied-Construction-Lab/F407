#ifndef IMU_PARSER_H
#define IMU_PARSER_H

#include <stddef.h>
#include <stdint.h>

#define IMU_FRAME_LEN 66U

typedef struct
{
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float gx_deg_s;
  float gy_deg_s;
  float gz_deg_s;
  uint64_t ts_ms;
} ImuSample;

typedef struct
{
  uint8_t bytes[IMU_FRAME_LEN];
  size_t length;
} ImuParser;

void imu_parser_init(ImuParser *parser);
uint8_t imu_parser_feed(ImuParser *parser, const uint8_t *data,
                        size_t length, ImuSample *latest);

#endif
