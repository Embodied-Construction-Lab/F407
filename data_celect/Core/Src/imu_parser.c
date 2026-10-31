#include "imu_parser.h"

#include <string.h>

static uint8_t has_header(const uint8_t *data)
{
  return (uint8_t)((data[0] == 0xABU) &&
                   (data[1] == 0x54U) &&
                   (data[2] == 0x65U) &&
                   (data[3] == 0x00U));
}

static uint32_t read_u32(const uint8_t *data)
{
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static uint64_t read_u64(const uint8_t *data)
{
  return ((uint64_t)data[0]) |
         ((uint64_t)data[1] << 8) |
         ((uint64_t)data[2] << 16) |
         ((uint64_t)data[3] << 24) |
         ((uint64_t)data[4] << 32) |
         ((uint64_t)data[5] << 40) |
         ((uint64_t)data[6] << 48) |
         ((uint64_t)data[7] << 56);
}

static float read_f32(const uint8_t *data)
{
  uint32_t bits = read_u32(data);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

static void decode_frame(const uint8_t *frame, ImuSample *sample)
{
  sample->gz = read_f32(&frame[31]);
  sample->ts_ms = read_u64(&frame[56]);
}

static void discard_bytes(ImuParser *parser, size_t count)
{
  parser->length -= count;
  memmove(parser->bytes, &parser->bytes[count], parser->length);
}

static uint8_t process_buffer(ImuParser *parser, ImuSample *latest)
{
  uint8_t produced = 0U;

  for (;;)
  {
    if (parser->length >= 4U && has_header(parser->bytes) == 0U)
    {
      parser->locked = 0U;
      discard_bytes(parser, 1U);
      continue;
    }

    if (parser->length < IMU_SYNC_LEN)
    {
      break;
    }

    if (has_header(&parser->bytes[IMU_FRAME_LEN]) == 0U)
    {
      parser->locked = 0U;
      discard_bytes(parser, 1U);
      continue;
    }

    decode_frame(parser->bytes, latest);
    produced = 1U;
    parser->locked = 1U;
    discard_bytes(parser, IMU_FRAME_LEN);
  }

  return produced;
}

void imu_parser_init(ImuParser *parser)
{
  memset(parser, 0, sizeof(*parser));
}

uint8_t imu_parser_feed(ImuParser *parser, const uint8_t *data,
                        size_t length, ImuSample *latest)
{
  uint8_t produced = 0U;

  while (length > 0U)
  {
    if (parser->length == IMU_SYNC_LEN)
    {
      produced |= process_buffer(parser, latest);
    }

    parser->bytes[parser->length++] = *data++;
    --length;
    produced |= process_buffer(parser, latest);
  }

  return produced;
}
