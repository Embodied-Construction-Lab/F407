#ifndef PCA9685_H
#define PCA9685_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

#define PCA9685_I2C_ADDRESS 0x40U
#define PCA9685_CHANNELS 16U

HAL_StatusTypeDef Pca9685_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef Pca9685_SetPwm(I2C_HandleTypeDef *hi2c,
                                 uint8_t channel,
                                 uint16_t on_count,
                                 uint16_t off_count);

#endif /* PCA9685_H */
