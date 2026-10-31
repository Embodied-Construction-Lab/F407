#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "imu_parser.h"

#define IMU_DMA_RX_SIZE 8192U
#define DMA_IT_HT 1U
#define DMA_IT_TC 2U
typedef struct { uint32_t remaining; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; } UART_HandleTypeDef;
static DMA_HandleTypeDef dma = {IMU_DMA_RX_SIZE};
static UART_HandleTypeDef huart1 = {&dma};
static uint8_t imu_dma_rx[IMU_DMA_RX_SIZE];
static uint16_t imu_dma_last_pos;
static ImuParser imu_parser;
static ImuSample imu_latest;
static uint8_t imu_sample_valid, imu_yaw_time_valid;
static float imu_yaw_deg, imu_yaw_rate_deg_s;
static uint64_t imu_last_ts_ms;
static uint32_t tick;
/* Production code uses this after adding receive-age expiry. */
static uint32_t imu_last_received_tick;
uint32_t HAL_GetTick(void) { return tick; }
#define __HAL_DMA_GET_COUNTER(handle) ((handle)->remaining)
#define __HAL_DMA_DISABLE_IT(handle, flag) ((void)(handle), (void)(flag))
static HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart,
                                             uint8_t *buffer, uint16_t length)
{
  (void)uart; (void)buffer; (void)length;
  return HAL_OK;
}
#include "imu_service_under_test.inc"

static void feed(float speed, uint64_t stamp)
{
  uint8_t frames[IMU_SYNC_LEN] = {0};
  for (unsigned i = 0; i < 2U; ++i)
  {
    uint8_t *frame = frames + i * IMU_FRAME_LEN;
    frame[0] = 0xAB; frame[1] = 0x54; frame[2] = 0x65;
    memcpy(frame + 31, &speed, sizeof(speed));
    for (unsigned b = 0; b < 8U; ++b) frame[56+b] = (uint8_t)(stamp >> (8U*b));
  }
  imu_parser_init(&imu_parser);
  Imu_FeedBytes(frames, sizeof(frames));
}

int main(int argc, char **argv)
{
  (void)argv;
  Imu_RxStart();
  imu_last_received_tick = 0U; /* Also avoid an unused symbol in the pre-fix build. */
  assert(imu_sample_valid == 0U);
  tick = 100U;
  feed(1.0f, 1000U);
  assert(imu_sample_valid == 1U);
  tick = 200U;
  feed(1.0f, 1100U);
  assert(fabsf(imu_yaw_deg - .1f) < .0001f);
  if (argc > 1)
  {
    tick += 501U;
    ServiceImu(); /* No DMA progress: stale validity still must expire. */
    assert(imu_sample_valid == 0U);
    assert(fabsf(imu_yaw_deg - .1f) < .0001f);
    feed(1.0f, 1800U);
    assert(imu_sample_valid == 1U);
    assert(fabsf(imu_yaw_deg - .1f) < .0001f); /* No integration across gap. */
    tick = UINT32_MAX - 200U;
    feed(1.0f, 1900U);
    tick = 301U;
    ServiceImu();
    assert(imu_sample_valid == 0U);
  }
  else
  {
    const float invalid[] = {NAN, INFINITY, -INFINITY};
    for (unsigned i = 0; i < 3U; ++i)
    {
      float before = imu_yaw_deg;
      feed(invalid[i], 1200U+i*100U);
      assert(imu_sample_valid == 0U);
      assert(isfinite(imu_yaw_rate_deg_s));
      assert(imu_yaw_deg == before);
      feed(1.0f, 1250U+i*100U);
      assert(imu_sample_valid == 1U);
      assert(imu_yaw_deg == before);
    }
    /* Huge but finite corrupted floats must not cause unbounded subtraction. */
    imu_yaw_deg = FLT_MAX;
    Imu_NormalizeYaw();
    assert(isfinite(imu_yaw_deg) && imu_yaw_deg >= 0.0f && imu_yaw_deg < 360.0f);
  }
  puts("imu service tests passed");
  return 0;
}
