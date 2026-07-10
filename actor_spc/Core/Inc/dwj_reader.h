#ifndef DWJ_READER_H
#define DWJ_READER_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DWJ_ADS1115_I2C_ADDRESS_7BIT 0x48U
#define DWJ_ADS1115_I2C_ADDRESS (DWJ_ADS1115_I2C_ADDRESS_7BIT << 1)
#define DWJ_ADS1115_CONVERSION_DELAY_MS 9U

typedef enum
{
  DWJ_ADS1115_CHANNEL_AIN0 = 0,
  DWJ_ADS1115_CHANNEL_AIN1 = 1,
  DWJ_ADS1115_CHANNEL_AIN2 = 2,
  DWJ_ADS1115_CHANNEL_AIN3 = 3
} DwjAds1115Channel;

typedef struct
{
  int16_t raw;
  int32_t mv;
  int16_t angle_tenths_deg;
} DwjPotentiometerReading;

typedef struct
{
  DwjPotentiometerReading big_arm;
  DwjPotentiometerReading bucket;
} DwjReadings;

int16_t Dwj_VoltageToAngleTenths(int32_t mv,
                                 int32_t mv_min,
                                 int32_t mv_max,
                                 int16_t min_angle_deg,
                                 int16_t max_angle_deg);
HAL_StatusTypeDef Dwj_StartConversion(I2C_HandleTypeDef *hi2c,
                                      DwjAds1115Channel channel);
HAL_StatusTypeDef Dwj_ReadConversionRawMv(I2C_HandleTypeDef *hi2c,
                                          int16_t *raw,
                                          int32_t *mv);
HAL_StatusTypeDef Dwj_BuildStartConversionCommand(DwjAds1115Channel channel,
                                                  uint8_t tx[3]);
void Dwj_BuildReadConversionCommand(uint8_t tx[1]);
void Dwj_DecodeRawMv(const uint8_t rx[2], int16_t *raw, int32_t *mv);
void Dwj_UpdateBigArmReading(int16_t raw,
                             int32_t mv,
                             DwjPotentiometerReading *reading);
void Dwj_UpdateBucketReading(int16_t raw,
                             int32_t mv,
                             DwjPotentiometerReading *reading);

#ifdef __cplusplus
}
#endif

#endif
