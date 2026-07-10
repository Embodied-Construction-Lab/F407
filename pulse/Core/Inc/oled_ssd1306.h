#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stdint.h>

#define OLED_SSD1306_WIDTH 128U
#define OLED_SSD1306_HEIGHT 64U
#define OLED_SSD1306_ROWS (OLED_SSD1306_HEIGHT / 8U)

HAL_StatusTypeDef OledSsd1306_Init(I2C_HandleTypeDef *i2c);
void OledSsd1306_Clear(void);
void OledSsd1306_WriteText(uint8_t row, uint8_t column, const char *text);
HAL_StatusTypeDef OledSsd1306_Update(void);

#ifdef __cplusplus
}
#endif

#endif
