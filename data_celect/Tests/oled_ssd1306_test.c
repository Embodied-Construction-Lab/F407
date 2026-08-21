#include "oled_ssd1306.h"

#include <assert.h>
#include <stddef.h>
#include <string.h>

#define MAX_CAPTURED_TRANSMISSIONS 16U

typedef struct
{
  uint16_t length;
  uint8_t bytes[17];
} CapturedTransmission;

static CapturedTransmission captured[MAX_CAPTURED_TRANSMISSIONS];
static size_t captured_count;

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *i2c,
                                        uint16_t address,
                                        uint32_t trials,
                                        uint32_t timeout_ms)
{
  (void)i2c;
  (void)address;
  (void)trials;
  (void)timeout_ms;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *i2c,
                                          uint16_t address,
                                          uint8_t *data,
                                          uint16_t length,
                                          uint32_t timeout_ms)
{
  (void)i2c;
  (void)address;
  (void)timeout_ms;
  if (captured_count < MAX_CAPTURED_TRANSMISSIONS)
  {
    captured[captured_count].length = length;
    assert(length <= sizeof(captured[captured_count].bytes));
    memcpy(captured[captured_count].bytes, data, length);
  }
  captured_count++;
  return HAL_OK;
}

void HAL_Delay(uint32_t delay_ms)
{
  (void)delay_ms;
}

static void ResetCapture(void)
{
  memset(captured, 0, sizeof(captured));
  captured_count = 0U;
}

int main(void)
{
  I2C_HandleTypeDef i2c = {0};

  assert(OledSsd1306_Init(&i2c) == HAL_OK);
  ResetCapture();

  assert(OledSsd1306_UpdatePage(3U) == HAL_OK);
  assert(captured_count == 11U);
  assert(captured[0].length == 2U);
  assert(captured[0].bytes[0] == 0x00U);
  assert(captured[0].bytes[1] == 0xB3U);
  assert(captured[1].bytes[1] == 0x00U);
  assert(captured[2].bytes[1] == 0x10U);
  for (size_t index = 3U; index < captured_count; ++index)
  {
    assert(captured[index].length == 17U);
    assert(captured[index].bytes[0] == 0x40U);
  }

  ResetCapture();
  assert(OledSsd1306_UpdatePage(OLED_SSD1306_ROWS) == HAL_ERROR);
  assert(captured_count == 0U);
  return 0;
}
