#include "imu_parser.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static void put_f32(uint8_t *frame, size_t offset, float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  for (size_t index = 0; index < 4U; index++)
  {
    frame[offset + index] = (uint8_t)(bits >> (index * 8U));
  }
}

static void make_frame(uint8_t *frame, float base)
{
  memset(frame, 0, IMU_FRAME_LEN);
  frame[0] = 0xABU;
  frame[1] = 0x54U;
  frame[2] = 0x65U;
  frame[3] = 0x00U;
  put_f32(frame, 11U, base + 1.0f);
  put_f32(frame, 15U, base + 2.0f);
  put_f32(frame, 19U, base + 3.0f);
  put_f32(frame, 23U, base + 4.0f);
  put_f32(frame, 27U, base + 5.0f);
  put_f32(frame, 31U, base + 6.0f);
  frame[56] = 0x12U;
  frame[57] = 0x34U;
}

int main(void)
{
  uint8_t stream[3U + 3U * IMU_FRAME_LEN];
  ImuParser parser;
  ImuSample sample = {0};
  uint8_t produced = 0U;

  stream[0] = 0x01U;
  stream[1] = 0x02U;
  stream[2] = 0x03U;
  make_frame(&stream[3], 10.0f);
  make_frame(&stream[3 + IMU_FRAME_LEN], 20.0f);
  make_frame(&stream[3 + 2U * IMU_FRAME_LEN], 30.0f);

  imu_parser_init(&parser);
  for (size_t offset = 0; offset < sizeof(stream); offset += 8U)
  {
    size_t count = sizeof(stream) - offset;
    if (count > 8U) count = 8U;
    produced |= imu_parser_feed(&parser, &stream[offset], count, &sample);
  }

  assert(produced != 0U);
  assert(sample.roll_deg == 31.0f);
  assert(sample.pitch_deg == 32.0f);
  assert(sample.yaw_deg == 33.0f);
  assert(sample.gx_deg_s == 34.0f);
  assert(sample.gy_deg_s == 35.0f);
  assert(sample.gz_deg_s == 36.0f);
  assert(sample.ts_ms == 0x3412U);

  /* A damaged frame must not replace the last good sample. */
  imu_parser_init(&parser);
  make_frame(stream, 0.0f);
  put_f32(stream, 11U, NAN);
  assert(imu_parser_feed(&parser, stream, IMU_FRAME_LEN,
                         &sample) == 0U);
  assert(sample.roll_deg == 31.0f);
  make_frame(stream, 40.0f);
  assert(imu_parser_feed(&parser, stream, IMU_FRAME_LEN,
                         &sample) != 0U);
  assert(sample.roll_deg == 41.0f);
  return 0;
}
