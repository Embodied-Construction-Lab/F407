#include "pca9685.h"

#include "pwm_timing.h"

#define PCA9685_MODE1 0x00U
#define PCA9685_MODE2 0x01U
#define PCA9685_PRESCALE 0xFEU
#define PCA9685_LED0_ON_L 0x06U
#define PCA9685_I2C_TIMEOUT_MS 20U
#define PCA9685_MODE1_AUTO_INCREMENT 0x20U
#define PCA9685_MODE1_RESTART 0x80U

static HAL_StatusTypeDef write_register(I2C_HandleTypeDef *hi2c,
                                        uint8_t reg,
                                        uint8_t value)
{
  uint8_t data[2] = {reg, value};
  return HAL_I2C_Master_Transmit(hi2c,
                                 (uint16_t)(PCA9685_I2C_ADDRESS << 1U),
                                 data, sizeof(data),
                                 PCA9685_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_register(I2C_HandleTypeDef *hi2c,
                                       uint8_t reg,
                                       uint8_t *value)
{
  HAL_StatusTypeDef status;

  status = HAL_I2C_Master_Transmit(hi2c,
                                   (uint16_t)(PCA9685_I2C_ADDRESS << 1U),
                                   &reg, 1U, PCA9685_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }
  return HAL_I2C_Master_Receive(hi2c,
                                (uint16_t)(PCA9685_I2C_ADDRESS << 1U),
                                value, 1U, PCA9685_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef Pca9685_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t old_mode;
  uint8_t sleep_mode;
  uint8_t prescale;
  HAL_StatusTypeDef status;

  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  status = write_register(hi2c, PCA9685_MODE1, 0x00U);
  if (status != HAL_OK)
  {
    return status;
  }
  status = write_register(hi2c, PCA9685_MODE2, 0x04U);
  if (status != HAL_OK)
  {
    return status;
  }
  HAL_Delay(5U);

  status = read_register(hi2c, PCA9685_MODE1, &old_mode);
  if (status != HAL_OK)
  {
    return status;
  }

  sleep_mode = (uint8_t)((old_mode & 0x7FU) | 0x10U);
  prescale = PwmTiming_CalculatePrescale(PWM_OSCILLATOR_HZ,
                                         PWM_TARGET_FREQUENCY_HZ);
  status = write_register(hi2c, PCA9685_MODE1, sleep_mode);
  if (status != HAL_OK)
  {
    return status;
  }
  status = write_register(hi2c, PCA9685_PRESCALE, prescale);
  if (status != HAL_OK)
  {
    return status;
  }
  status = write_register(hi2c, PCA9685_MODE1, old_mode);
  if (status != HAL_OK)
  {
    return status;
  }
  HAL_Delay(5U);
  return write_register(hi2c, PCA9685_MODE1,
                        (uint8_t)(old_mode | PCA9685_MODE1_RESTART |
                                  PCA9685_MODE1_AUTO_INCREMENT));
}

HAL_StatusTypeDef Pca9685_SetPwm(I2C_HandleTypeDef *hi2c,
                                 uint8_t channel,
                                 uint16_t on_count,
                                 uint16_t off_count)
{
  uint8_t data[5];

  if ((hi2c == NULL) || (channel >= PCA9685_CHANNELS))
  {
    return HAL_ERROR;
  }

  data[0] = (uint8_t)(PCA9685_LED0_ON_L + 4U * channel);
  data[1] = (uint8_t)(on_count & 0xFFU);
  data[2] = (uint8_t)(on_count >> 8U);
  data[3] = (uint8_t)(off_count & 0xFFU);
  data[4] = (uint8_t)(off_count >> 8U);
  return HAL_I2C_Master_Transmit(hi2c,
                                 (uint16_t)(PCA9685_I2C_ADDRESS << 1U),
                                 data, sizeof(data),
                                 PCA9685_I2C_TIMEOUT_MS);
}
