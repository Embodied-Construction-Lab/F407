#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "imu_parser.h"

typedef struct
{
  ImuSample samples[16];
  size_t count;
  double integral_deg;
} Capture;

static void capture_sample(const ImuSample *sample, void *context)
{
  Capture *capture = context;
  assert(capture->count < sizeof(capture->samples) / sizeof(capture->samples[0]));
  if (capture->count > 0U)
  {
    uint64_t before = capture->samples[capture->count - 1U].ts_ms;
    assert(sample->ts_ms > before);
    capture->integral_deg += (double)sample->gz * (double)(sample->ts_ms - before) / 1000.0;
  }
  capture->samples[capture->count++] = *sample;
}

static void frame(uint8_t *destination, float rate, uint64_t stamp)
{
  uint32_t rate_bits;
  memset(destination, 0, IMU_FRAME_LEN);
  destination[0] = 0xABU;
  destination[1] = 0x54U;
  destination[2] = 0x65U;
  memcpy(&rate_bits, &rate, sizeof(rate_bits));
  for (unsigned int byte = 0U; byte < 4U; ++byte)
  {
    destination[31U + byte] = (uint8_t)(rate_bits >> (8U * byte));
  }
  for (unsigned int byte = 0U; byte < 8U; ++byte)
  {
    destination[56U + byte] = (uint8_t)(stamp >> (8U * byte));
  }
}

static void assert_complete(const Capture *capture)
{
  static const float rates[] = {0.0f, 1.0f, -1.0f, 1.0f, -1.0f};
  assert(capture->count == sizeof(rates) / sizeof(rates[0]));
  for (size_t index = 0U; index < capture->count; ++index)
  {
    assert(capture->samples[index].gz == rates[index]);
    assert(capture->samples[index].ts_ms == 1000U + 10U * index);
  }
  assert(fabs(capture->integral_deg) < 1e-12);
}

int main(void)
{
  /* One following header/frame is needed by the existing two-header sync. */
  uint8_t stream[6U * IMU_FRAME_LEN];
  static const float rates[] = {0.0f, 1.0f, -1.0f, 1.0f, -1.0f, 7.0f};
  static const size_t chunks[] = {1U, 7U, 65U, 66U, 67U, 132U};
  ImuParser parser;
  Capture capture = {0};
  for (size_t index = 0U; index < 6U; ++index)
  {
    frame(stream + index * IMU_FRAME_LEN, rates[index], 1000U + 10U * index);
  }
  imu_parser_init(&parser);
  assert(imu_parser_feed(&parser, stream, sizeof(stream), capture_sample, &capture) == 1U);
  assert_complete(&capture);
  assert(parser.length == IMU_FRAME_LEN);
  assert(imu_parser_feed(&parser, stream, 0U, capture_sample, &capture) == 0U);
  assert_complete(&capture);

  for (size_t part = 0U; part < sizeof(chunks) / sizeof(chunks[0]); ++part)
  {
    Capture split = {0};
    imu_parser_init(&parser);
    for (size_t offset = 0U; offset < sizeof(stream);)
    {
      size_t length = sizeof(stream) - offset;
      if (length > chunks[part]) { length = chunks[part]; }
      (void)imu_parser_feed(&parser, stream + offset, length, capture_sample, &split);
      offset += length;
    }
    assert_complete(&split);
  }

  /* Garbage and a partial trailing frame must not invent callbacks. */
  {
    const uint8_t noise[] = {0x00U, 0xABU, 0x54U, 0x12U, 0x34U};
    Capture recovered = {0};
    imu_parser_init(&parser);
    assert(imu_parser_feed(&parser, noise, sizeof(noise), capture_sample, &recovered) == 0U);
    assert(imu_parser_feed(&parser, stream, IMU_FRAME_LEN, capture_sample, &recovered) == 0U);
    assert(recovered.count == 0U);
    assert(imu_parser_feed(&parser, stream + IMU_FRAME_LEN,
                           sizeof(stream) - IMU_FRAME_LEN, capture_sample, &recovered) == 1U);
    assert_complete(&recovered);
  }
  puts("imu parser callback tests passed");
  return 0;
}
