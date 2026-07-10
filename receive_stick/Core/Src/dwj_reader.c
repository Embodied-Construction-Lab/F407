#include "dwj_reader.h"

#define ADS1115_REG_CONVERSION 0x00U
#define ADS1115_REG_CONFIG 0x01U
#define ADS1115_CONFIG_OS_SINGLE 0x8000U
#define ADS1115_CONFIG_MUX_SINGLE_0 0x4000U
#define ADS1115_CONFIG_PGA_4V096 0x0200U
#define ADS1115_CONFIG_MODE_SINGLE 0x0100U
#define ADS1115_CONFIG_DR_128SPS 0x0080U
#define ADS1115_CONFIG_COMP_DISABLED 0x0003U
#define ADS1115_FULL_SCALE_MV 4096
#define ADS1115_COUNTS_PER_FULL_SCALE 32768
#define ADS1115_I2C_TIMEOUT_MS 10U

#define BIG_ARM_MV_MIN 632
#define BIG_ARM_MV_MAX 1322
#define BIG_ARM_ANGLE_MIN 90
#define BIG_ARM_ANGLE_MAX 20

#define BUCKET_MV_MIN 610
#define BUCKET_MV_MAX 1975
#define BUCKET_ANGLE_MIN 80
#define BUCKET_ANGLE_MAX 230

static uint16_t ads1115_build_config(DwjAds1115Channel channel)
{
  uint16_t mux = (uint16_t)(ADS1115_CONFIG_MUX_SINGLE_0 +
                            ((uint16_t)channel << 12));

  return (uint16_t)(ADS1115_CONFIG_OS_SINGLE |
                    mux |
                    ADS1115_CONFIG_PGA_4V096 |
                    ADS1115_CONFIG_MODE_SINGLE |
                    ADS1115_CONFIG_DR_128SPS |
                    ADS1115_CONFIG_COMP_DISABLED);
}

static int32_t ads1115_raw_to_mv(int16_t raw)
{
  if (raw <= 0)
  {
    return 0;
  }

  return ((int32_t)raw * ADS1115_FULL_SCALE_MV) /
         ADS1115_COUNTS_PER_FULL_SCALE;
}

int16_t Dwj_VoltageToAngleTenths(int32_t mv,
                                 int32_t mv_min,
                                 int32_t mv_max,
                                 int16_t min_angle_deg,
                                 int16_t max_angle_deg)
{
  int32_t numerator;
  int32_t denominator;
  int32_t angle_tenths;

  if (mv_min == mv_max)
  {
    return (int16_t)(min_angle_deg * 10);
  }

  if (mv < mv_min)
  {
    mv = mv_min;
  }
  else if (mv > mv_max)
  {
    mv = mv_max;
  }

  numerator = (mv - mv_min) * (int32_t)(max_angle_deg - min_angle_deg) * 10;
  denominator = mv_max - mv_min;
  angle_tenths = ((int32_t)min_angle_deg * 10) + (numerator / denominator);

  return (int16_t)angle_tenths;
}

HAL_StatusTypeDef Dwj_StartConversion(I2C_HandleTypeDef *hi2c,
                                      DwjAds1115Channel channel)
{
  uint8_t tx[3];

  if ((hi2c == 0) ||
      (Dwj_BuildStartConversionCommand(channel, tx) != HAL_OK))
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Master_Transmit(hi2c,
                                 DWJ_ADS1115_I2C_ADDRESS,
                                 tx,
                                 sizeof(tx),
                                 ADS1115_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef Dwj_ReadConversionRawMv(I2C_HandleTypeDef *hi2c,
                                          int16_t *raw,
                                          int32_t *mv)
{
  uint8_t reg[1];
  uint8_t rx[2];
  HAL_StatusTypeDef status;

  if ((hi2c == 0) || (raw == 0) || (mv == 0))
  {
    return HAL_ERROR;
  }

  Dwj_BuildReadConversionCommand(reg);
  status = HAL_I2C_Master_Transmit(hi2c,
                                   DWJ_ADS1115_I2C_ADDRESS,
                                   reg,
                                   1U,
                                   ADS1115_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_I2C_Master_Receive(hi2c,
                                  DWJ_ADS1115_I2C_ADDRESS,
                                  rx,
                                  sizeof(rx),
                                  ADS1115_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  Dwj_DecodeRawMv(rx, raw, mv);
  return HAL_OK;
}

HAL_StatusTypeDef Dwj_BuildStartConversionCommand(DwjAds1115Channel channel,
                                                  uint8_t tx[3])
{
  uint16_t config;

  if ((tx == 0) || (channel > DWJ_ADS1115_CHANNEL_AIN3))
  {
    return HAL_ERROR;
  }

  config = ads1115_build_config(channel);
  tx[0] = ADS1115_REG_CONFIG;
  tx[1] = (uint8_t)(config >> 8);
  tx[2] = (uint8_t)(config & 0xFFU);

  return HAL_OK;
}

void Dwj_BuildReadConversionCommand(uint8_t tx[1])
{
  if (tx == 0)
  {
    return;
  }

  tx[0] = ADS1115_REG_CONVERSION;
}

void Dwj_DecodeRawMv(const uint8_t rx[2], int16_t *raw, int32_t *mv)
{
  if ((rx == 0) || (raw == 0) || (mv == 0))
  {
    return;
  }

  *raw = (int16_t)(((uint16_t)rx[0] << 8) | rx[1]);
  *mv = ads1115_raw_to_mv(*raw);
}

void Dwj_UpdateBigArmReading(int16_t raw,
                             int32_t mv,
                             DwjPotentiometerReading *reading)
{
  if (reading == 0)
  {
    return;
  }

  reading->raw = raw;
  reading->mv = mv;
  reading->angle_tenths_deg = Dwj_VoltageToAngleTenths(mv,
                                                       BIG_ARM_MV_MIN,
                                                       BIG_ARM_MV_MAX,
                                                       BIG_ARM_ANGLE_MIN,
                                                       BIG_ARM_ANGLE_MAX);
}

void Dwj_UpdateBucketReading(int16_t raw,
                             int32_t mv,
                             DwjPotentiometerReading *reading)
{
  if (reading == 0)
  {
    return;
  }

  reading->raw = raw;
  reading->mv = mv;
  reading->angle_tenths_deg = Dwj_VoltageToAngleTenths(mv,
                                                       BUCKET_MV_MIN,
                                                       BUCKET_MV_MAX,
                                                       BUCKET_ANGLE_MIN,
                                                       BUCKET_ANGLE_MAX);
}
