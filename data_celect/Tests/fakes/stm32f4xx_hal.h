#ifndef STM32F4XX_HAL_H
#define STM32F4XX_HAL_H

#include <stdint.h>

typedef enum
{
  HAL_OK = 0,
  HAL_ERROR = 1
} HAL_StatusTypeDef;

typedef struct
{
  uint32_t unused;
} I2C_HandleTypeDef;

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *i2c,
                                        uint16_t address,
                                        uint32_t trials,
                                        uint32_t timeout_ms);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *i2c,
                                          uint16_t address,
                                          uint8_t *data,
                                          uint16_t length,
                                          uint32_t timeout_ms);
void HAL_Delay(uint32_t delay_ms);

#endif
