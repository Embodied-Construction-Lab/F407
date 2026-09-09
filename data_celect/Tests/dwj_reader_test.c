#include <assert.h>
#include <stdio.h>
#include "dwj_reader.h"

static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
static I2C_HandleTypeDef hi2c1;
static enum { DWJ_IDLE, DWJ_WAIT_BIG_ARM, DWJ_WAIT_SMALL_ARM } dwj_state;
static uint32_t dwj_state_tick, last_dwj_poll_tick;
static HAL_StatusTypeDef dwj_status;
static DwjReadings dwj_readings;
#define DWJ_POLL_PERIOD_MS 50U
#include "dwj_service_under_test.inc"
static uint8_t selected_register;
static uint16_t config_word;
static unsigned conversion_reads;
static HAL_StatusTypeDef bus_status = HAL_OK;
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *i2c, uint16_t address,
    uint8_t *data, uint16_t length, uint32_t timeout)
{
  (void)i2c; (void)address; (void)length; (void)timeout;
  if (length == 3) { tick += 4; } /* Simulate conversion-start I2C cost. */
  selected_register = data[0];
  return bus_status;
}
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *i2c, uint16_t address,
    uint8_t *data, uint16_t length, uint32_t timeout)
{
  uint16_t word = config_word;
  (void)i2c; (void)address; (void)length; (void)timeout;
  if (selected_register == 0) { ++conversion_reads; word = 8888; }
  data[0] = (uint8_t)(word >> 8); data[1] = (uint8_t)word;
  return bus_status;
}
int main(void)
{
  I2C_HandleTypeDef i2c = {0};
  int16_t raw = -1;
  int32_t mv = -1;
  config_word = 0x5383; /* AIN1 still busy: conversion register contains AIN0. */
  assert(Dwj_ReadConversionRawMv(&i2c, DWJ_ADS1115_CHANNEL_AIN1, &raw, &mv) == HAL_BUSY);
  assert(conversion_reads == 0 && raw == -1 && mv == -1);
  config_word = 0xc383; /* Ready but wrong MUX. */
  assert(Dwj_ReadConversionRawMv(&i2c, DWJ_ADS1115_CHANNEL_AIN1, &raw, &mv) == HAL_ERROR);
  assert(conversion_reads == 0);
  config_word = 0xd383;
  assert(Dwj_ReadConversionRawMv(&i2c, DWJ_ADS1115_CHANNEL_AIN1, &raw, &mv) == HAL_OK);
  assert(conversion_reads == 1 && raw == 8888 && mv == 1111);
  bus_status = HAL_ERROR;
  assert(Dwj_ReadConversionRawMv(&i2c, DWJ_ADS1115_CHANNEL_AIN1, &raw, &mv) == HAL_ERROR);
  assert(conversion_reads == 1);
  bus_status = HAL_OK;
  tick = 100;
  ServiceDwjReader();
  assert(dwj_state == DWJ_WAIT_BIG_ARM && dwj_state_tick == 104);
  tick = 113;
  config_word = 0x4383;
  ServiceDwjReader();
  assert(dwj_state == DWJ_WAIT_BIG_ARM && dwj_readings.big_arm.raw == 0);
  tick = 134;
  ServiceDwjReader();
  assert(dwj_state == DWJ_IDLE && dwj_status == HAL_ERROR);
  tick = 200;
  ServiceDwjReader();
  tick = 213;
  config_word = 0xc383;
  ServiceDwjReader();
  assert(dwj_state == DWJ_WAIT_SMALL_ARM && dwj_state_tick == 217);
  assert(dwj_readings.big_arm.mv == 1111);
  tick = 226;
  config_word = 0x5383;
  ServiceDwjReader();
  assert(dwj_state == DWJ_WAIT_SMALL_ARM && dwj_readings.bucket.raw == 0);
  tick = 227;
  config_word = 0xd383;
  ServiceDwjReader();
  assert(dwj_state == DWJ_IDLE && dwj_status == HAL_OK);
  assert(dwj_readings.bucket.mv == 1111);
  /* A HAL bus-busy error must not masquerade as ADC conversion-in-progress. */
  bus_status = HAL_BUSY;
  assert(Dwj_ReadConversionRawMv(&i2c, DWJ_ADS1115_CHANNEL_AIN1, &raw, &mv) == HAL_ERROR);
  bus_status = HAL_OK;
  dwj_state = DWJ_WAIT_SMALL_ARM;
  dwj_state_tick = UINT32_MAX - 10U;
  tick = 19U; /* 30 ms elapsed across tick rollover. */
  ServiceDwjReader();
  assert(dwj_state == DWJ_IDLE && dwj_status == HAL_ERROR);
  puts("dwj reader tests passed");
  return 0;
}
