#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "imu_parser.h"
#include "gyro_bias_calibrator.h"
#include "control_command.h"

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
static GyroBiasCalibrator imu_gyro_bias;
static volatile uint8_t imu_restart_requested;
static uint8_t imu_sample_valid, imu_yaw_time_valid;
static float imu_yaw_deg, imu_yaw_rate_deg_s;
static uint64_t imu_last_ts_ms;
static uint32_t tick, imu_last_received_tick, aborts;
static uint8_t command_valid;
static ControlCommand latest_control_command;
uint32_t HAL_GetTick(void) { return tick; }
static uint8_t ControlCommandIsValid(uint32_t now) { (void)now; return command_valid; }
#define __HAL_DMA_GET_COUNTER(handle) ((handle)->remaining)
#define __HAL_DMA_DISABLE_IT(handle, flag) ((void)(handle), (void)(flag))
static HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart,
                                             uint8_t *buffer, uint16_t length)
{
  (void)uart; (void)buffer; (void)length;
  dma.remaining = IMU_DMA_RX_SIZE;
  return HAL_OK;
}
static HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
  (void)uart;
  ++aborts;
  return HAL_OK;
}
#include "imu_service_under_test.inc"

static void frame(uint8_t *bytes, float speed, uint64_t stamp)
{
  memset(bytes, 0, IMU_FRAME_LEN);
  bytes[0] = 0xAB; bytes[1] = 0x54; bytes[2] = 0x65;
  memcpy(bytes + 31, &speed, sizeof(speed));
  for (unsigned b = 0; b < 8U; ++b) bytes[56+b] = (uint8_t)(stamp >> (8U*b));
}

static void feed(float speed, uint64_t stamp)
{
  uint8_t bytes[IMU_SYNC_LEN];
  frame(bytes, speed, stamp);
  frame(bytes + IMU_FRAME_LEN, speed, stamp + 10U);
  imu_parser_init(&imu_parser);
  Imu_FeedBytes(bytes, sizeof(bytes));
}

static void boot(void)
{
  tick = 0U;
  command_valid = 0U;
  imu_restart_requested = 0U;
  imu_yaw_deg = 0.0f;
  GyroBiasCalibrator_Init(&imu_gyro_bias);
  Imu_RxStart();
}

static void calibrate(float bias)
{
  for (uint64_t stamp = 0; stamp <= 10000U; stamp += 10U)
  {
    tick = (uint32_t)stamp;
    feed(bias, stamp);
    if (stamp < 10000U) { assert(imu_sample_valid == 0U); }
  }
  assert(imu_sample_valid == 1U);
}

static void static_bias_and_motion(void)
{
  boot();
  calibrate(.02f);
  for (uint64_t stamp = 10010U; stamp <= 40000U; stamp += 10U)
  {
    tick = (uint32_t)stamp;
    feed(.02f, stamp);
  }
  assert(fabsf(imu_yaw_rate_deg_s) < .00001f);
  assert(fabsf(imu_yaw_deg) < .001f);
  /* No corrected-speed deadband: very slow real motion must remain visible. */
  for (uint64_t stamp = 40100U; stamp <= 50100U; stamp += 100U) feed(.025f, stamp);
  assert(imu_yaw_deg > .049f && imu_yaw_deg < .052f);
  for (uint64_t stamp = 50200U; stamp <= 60200U; stamp += 100U) feed(.015f, stamp);
  assert(fabsf(imu_yaw_deg) < .001f || fabsf(imu_yaw_deg - 360.f) < .001f);
  feed(90.02f, 60300U);
  for (uint64_t stamp = 60400U; stamp <= 61200U; stamp += 100U) feed(90.02f, stamp);
  assert(fabsf(imu_yaw_deg - 90.f) < .002f);
  for (uint64_t stamp = 61300U; stamp <= 62200U; stamp += 100U) feed(-89.98f, stamp);
  assert(fabsf(imu_yaw_deg) < .003f || fabsf(imu_yaw_deg - 360.f) < .003f);
}

static void invalid_and_restart(void)
{
  boot(); calibrate(.02f);
  tick = 10100U; feed(1.02f, 10100U);
  assert(fabsf(imu_yaw_deg - .1f) < .0001f);
  const float invalid[] = {NAN, INFINITY, -INFINITY};
  for (unsigned i = 0; i < 3U; ++i)
  {
    float before = imu_yaw_deg;
    feed(invalid[i], 10200U+i*200U);
    assert(imu_sample_valid == 0U);
    assert(isfinite(imu_yaw_rate_deg_s));
    feed(1.02f, 10300U+i*200U);
    assert(imu_sample_valid == 1U);
    assert(imu_yaw_deg == before);
  }
  float before = imu_yaw_deg;
  imu_restart_requested = 1U;
  ServiceImu();
  assert(aborts == 1U);
  assert(GyroBiasCalibrator_IsReady(&imu_gyro_bias));
  assert(imu_yaw_deg == before);
  feed(1.02f, 11000U);
  assert(imu_yaw_deg == before);
  /* Duplicate and sensor reset fence the interval but recover next samples. */
  feed(1.02f, 11000U); assert(imu_sample_valid == 0U);
  feed(1.02f, 11100U); assert(imu_sample_valid == 1U);
  assert(imu_yaw_deg == before);
  feed(1.02f, 50U); assert(imu_sample_valid == 0U);
  feed(1.02f, 150U); assert(imu_sample_valid == 1U);
  assert(imu_yaw_deg == before);
  feed(1.02f, 250U);
  assert(fabsf(imu_yaw_deg - (before + .1f)) < .0001f);
  before = imu_yaw_deg;
  feed(1.02f, 10000U); /* A long timestamp gap is not integrated. */
  assert(imu_yaw_deg == before);
  imu_yaw_deg = FLT_MAX;
  Imu_NormalizeYaw();
  assert(isfinite(imu_yaw_deg) && imu_yaw_deg >= 0.f && imu_yaw_deg < 360.f);
}

static void expiry(void)
{
  boot(); calibrate(.02f);
  tick = 10100U; feed(1.02f, 10100U);
  float before = imu_yaw_deg;
  tick += 501U; ServiceImu();
  assert(imu_sample_valid == 0U);
  assert(imu_yaw_deg == before);
  feed(1.02f, 10800U);
  assert(imu_sample_valid == 1U && imu_yaw_deg == before);
  tick = UINT32_MAX - 200U; feed(.02f, 10900U);
  tick = 301U; ServiceImu();
  assert(imu_sample_valid == 0U);
}

static void requested_motion_resets_pending(void)
{
  boot();
  for (uint64_t stamp = 0; stamp <= 8000U; stamp += 10U) feed(.02f, stamp);
  command_valid = 1U;
  latest_control_command.mode = CONTROL_MODE_MANUAL_ACTION;
  latest_control_command.axis.swing = .2f;
  feed(.02f, 8010U);
  assert(imu_gyro_bias.sample_count == 0U && imu_sample_valid == 0U);
  command_valid = 0U;
  for (uint64_t stamp = 8020U; stamp < 18020U; stamp += 10U) feed(.02f, stamp);
  assert(!GyroBiasCalibrator_IsReady(&imu_gyro_bias));
  feed(.02f, 18020U);
  assert(GyroBiasCalibrator_IsReady(&imu_gyro_bias));
}

static void batch_native_samples(void)
{
  boot(); calibrate(.02f);
  float before = imu_yaw_deg;
  uint8_t bytes[3U * IMU_FRAME_LEN];
  frame(bytes, 1.02f, 10010U);
  frame(bytes + IMU_FRAME_LEN, -.98f, 10020U);
  frame(bytes + 2U*IMU_FRAME_LEN, .02f, 10030U);
  imu_parser_init(&imu_parser);
  Imu_FeedBytes(bytes, sizeof(bytes));
  assert(fabsf(imu_yaw_deg - before) < .0001f || fabsf(imu_yaw_deg - 360.f - before) < .0001f);
}

int main(int argc, char **argv)
{
  if (argc > 1 && strcmp(argv[1], "expiry") == 0) expiry();
  else if (argc > 1 && strcmp(argv[1], "drift") == 0) static_bias_and_motion();
  else
  {
    invalid_and_restart();
    requested_motion_resets_pending();
    batch_native_samples();
  }
  puts("imu service tests passed");
  return 0;
}
