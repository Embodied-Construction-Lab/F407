#include "imu_parser.h"

#include <math.h>
#include <string.h>

static uint8_t has_header(const uint8_t *data)
{
  return (uint8_t)(data[0] == 0xABU && data[1] == 0x54U &&
                   data[2] == 0x65U && data[3] == 0x00U);
}

static uint32_t read_u32(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static float read_f32(const uint8_t *data)
{
  uint32_t bits = read_u32(data);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

static uint64_t read_u64(const uint8_t *data)
{
  uint64_t value = 0U;
  for (uint8_t index = 0U; index < 8U; index++)
  {
    value |= (uint64_t)data[index] << (8U * index);
  }
  return value;
}

static uint8_t decode_frame(const uint8_t *frame, ImuSample *sample)
{
  ImuSample candidate;
  candidate.roll_deg = read_f32(&frame[11]);
  candidate.pitch_deg = read_f32(&frame[15]);
  candidate.yaw_deg = read_f32(&frame[19]);
  candidate.gx_deg_s = read_f32(&frame[23]);
  candidate.gy_deg_s = read_f32(&frame[27]);
  candidate.gz_deg_s = read_f32(&frame[31]);
  candidate.ts_ms = read_u64(&frame[56]);

  /* Reject obviously damaged concatenations. CRC16 parameters are not supplied. */
  if (!isfinite(candidate.roll_deg) || !isfinite(candidate.pitch_deg) ||
      !isfinite(candidate.yaw_deg) || !isfinite(candidate.gx_deg_s) ||
      !isfinite(candidate.gy_deg_s) || !isfinite(candidate.gz_deg_s))
  {
    return 0U;
  }

  *sample = candidate;
  return 1U;
}

static void discard_bytes(ImuParser *parser, size_t count)
{
  parser->length -= count;
  memmove(parser->bytes, &parser->bytes[count], parser->length);
}

void imu_parser_init(ImuParser *parser)
{
  memset(parser, 0, sizeof(*parser));
}

uint8_t imu_parser_feed(ImuParser *parser, const uint8_t *data,
                        size_t length, ImuSample *latest)
{
  uint8_t produced = 0U;

  while (length-- > 0U)
  {
    parser->bytes[parser->length++] = *data++;

    while (parser->length >= 4U && !has_header(parser->bytes))
    {
      discard_bytes(parser, 1U);
    }

    if (parser->length == IMU_FRAME_LEN)
    {
      if (decode_frame(parser->bytes, latest))
      {
        produced = 1U;
        discard_bytes(parser, IMU_FRAME_LEN);
      }
      else
      {
        discard_bytes(parser, 1U);
      }
    }
  }

  return produced;
}
