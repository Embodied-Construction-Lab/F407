#include "oled_ssd1306.h"

#include <string.h>

#define OLED_SSD1306_I2C_ADDR_0 (0x3CU << 1)
#define OLED_SSD1306_I2C_ADDR_1 (0x3DU << 1)
#define OLED_SSD1306_CONTROL_COMMAND 0x00U
#define OLED_SSD1306_CONTROL_DATA 0x40U
#define OLED_SSD1306_CHAR_WIDTH 6U

static I2C_HandleTypeDef *oled_i2c;
static uint16_t oled_address;
static uint8_t oled_buffer[OLED_SSD1306_WIDTH * OLED_SSD1306_ROWS];

static HAL_StatusTypeDef OledSsd1306_WriteCommand(uint8_t command)
{
  uint8_t data[2] = {OLED_SSD1306_CONTROL_COMMAND, command};

  return HAL_I2C_Master_Transmit(oled_i2c, oled_address, data, sizeof(data), 50);
}

static HAL_StatusTypeDef OledSsd1306_WriteData(const uint8_t *data,
                                               uint16_t length)
{
  uint8_t packet[17];
  uint16_t offset = 0;

  packet[0] = OLED_SSD1306_CONTROL_DATA;
  while (offset < length)
  {
    uint16_t chunk = (uint16_t)(length - offset);
    if (chunk > (sizeof(packet) - 1U))
    {
      chunk = (uint16_t)(sizeof(packet) - 1U);
    }

    memcpy(&packet[1], &data[offset], chunk);
    if (HAL_I2C_Master_Transmit(oled_i2c, oled_address, packet,
                                (uint16_t)(chunk + 1U), 50) != HAL_OK)
    {
      return HAL_ERROR;
    }
    offset = (uint16_t)(offset + chunk);
  }

  return HAL_OK;
}

static void OledSsd1306_GetGlyph(char character, uint8_t glyph[5])
{
  static const uint8_t digits[10][5] = {
      {0x3E, 0x51, 0x49, 0x45, 0x3E},
      {0x00, 0x42, 0x7F, 0x40, 0x00},
      {0x42, 0x61, 0x51, 0x49, 0x46},
      {0x21, 0x41, 0x45, 0x4B, 0x31},
      {0x18, 0x14, 0x12, 0x7F, 0x10},
      {0x27, 0x45, 0x45, 0x45, 0x39},
      {0x3C, 0x4A, 0x49, 0x49, 0x30},
      {0x01, 0x71, 0x09, 0x05, 0x03},
      {0x36, 0x49, 0x49, 0x49, 0x36},
      {0x06, 0x49, 0x49, 0x29, 0x1E},
  };

  memset(glyph, 0, 5);

  if ((character >= '0') && (character <= '9'))
  {
    memcpy(glyph, digits[character - '0'], 5);
    return;
  }

  switch (character)
  {
  case ' ':
    break;
  case '-':
    memset(glyph, 0x08, 5);
    break;
  case '.':
    glyph[1] = 0x60;
    glyph[2] = 0x60;
    break;
  case ':':
    glyph[1] = 0x36;
    glyph[2] = 0x36;
    break;
  case '/':
    glyph[0] = 0x20;
    glyph[1] = 0x10;
    glyph[2] = 0x08;
    glyph[3] = 0x04;
    glyph[4] = 0x02;
    break;
  case 'C':
    glyph[0] = 0x3E;
    glyph[1] = 0x41;
    glyph[2] = 0x41;
    glyph[3] = 0x41;
    glyph[4] = 0x22;
    break;
  case 'A':
    glyph[0] = 0x7E;
    glyph[1] = 0x11;
    glyph[2] = 0x11;
    glyph[3] = 0x11;
    glyph[4] = 0x7E;
    break;
  case 'E':
    glyph[0] = 0x7F;
    glyph[1] = 0x49;
    glyph[2] = 0x49;
    glyph[3] = 0x49;
    glyph[4] = 0x41;
    break;
  case 'I':
    glyph[0] = 0x00;
    glyph[1] = 0x41;
    glyph[2] = 0x7F;
    glyph[3] = 0x41;
    glyph[4] = 0x00;
    break;
  case 'L':
    glyph[0] = 0x7F;
    glyph[1] = 0x40;
    glyph[2] = 0x40;
    glyph[3] = 0x40;
    glyph[4] = 0x40;
    break;
  case 'M':
    glyph[0] = 0x7F;
    glyph[1] = 0x02;
    glyph[2] = 0x0C;
    glyph[3] = 0x02;
    glyph[4] = 0x7F;
    break;
  case 'O':
    glyph[0] = 0x3E;
    glyph[1] = 0x41;
    glyph[2] = 0x41;
    glyph[3] = 0x41;
    glyph[4] = 0x3E;
    break;
  case 'R':
    glyph[0] = 0x7F;
    glyph[1] = 0x09;
    glyph[2] = 0x19;
    glyph[3] = 0x29;
    glyph[4] = 0x46;
    break;
  case 'S':
    glyph[0] = 0x46;
    glyph[1] = 0x49;
    glyph[2] = 0x49;
    glyph[3] = 0x49;
    glyph[4] = 0x31;
    break;
  case 'U':
    glyph[0] = 0x3F;
    glyph[1] = 0x40;
    glyph[2] = 0x40;
    glyph[3] = 0x40;
    glyph[4] = 0x3F;
    break;
  case 'V':
    glyph[0] = 0x1F;
    glyph[1] = 0x20;
    glyph[2] = 0x40;
    glyph[3] = 0x20;
    glyph[4] = 0x1F;
    break;
  case 'a':
    glyph[0] = 0x20;
    glyph[1] = 0x54;
    glyph[2] = 0x54;
    glyph[3] = 0x54;
    glyph[4] = 0x78;
    break;
  case 'e':
    glyph[0] = 0x38;
    glyph[1] = 0x54;
    glyph[2] = 0x54;
    glyph[3] = 0x54;
    glyph[4] = 0x18;
    break;
  case 'g':
    glyph[0] = 0x18;
    glyph[1] = 0xA4;
    glyph[2] = 0xA4;
    glyph[3] = 0xA4;
    glyph[4] = 0x7C;
    break;
  case 'l':
    glyph[1] = 0x41;
    glyph[2] = 0x7F;
    glyph[3] = 0x40;
    break;
  case 'm':
    glyph[0] = 0x7C;
    glyph[1] = 0x04;
    glyph[2] = 0x18;
    glyph[3] = 0x04;
    glyph[4] = 0x78;
    break;
  case 'n':
    glyph[0] = 0x7C;
    glyph[1] = 0x08;
    glyph[2] = 0x04;
    glyph[3] = 0x04;
    glyph[4] = 0x78;
    break;
  case 'r':
    glyph[0] = 0x7C;
    glyph[1] = 0x08;
    glyph[2] = 0x04;
    glyph[3] = 0x04;
    glyph[4] = 0x08;
    break;
  case 's':
    glyph[0] = 0x48;
    glyph[1] = 0x54;
    glyph[2] = 0x54;
    glyph[3] = 0x54;
    glyph[4] = 0x20;
    break;
  case 't':
    glyph[0] = 0x04;
    glyph[1] = 0x3F;
    glyph[2] = 0x44;
    glyph[3] = 0x40;
    glyph[4] = 0x20;
    break;
  case 'w':
    glyph[0] = 0x3C;
    glyph[1] = 0x40;
    glyph[2] = 0x30;
    glyph[3] = 0x40;
    glyph[4] = 0x3C;
    break;
  case 'x':
    glyph[0] = 0x44;
    glyph[1] = 0x28;
    glyph[2] = 0x10;
    glyph[3] = 0x28;
    glyph[4] = 0x44;
    break;
  case 'y':
    glyph[0] = 0x0C;
    glyph[1] = 0x90;
    glyph[2] = 0x90;
    glyph[3] = 0x90;
    glyph[4] = 0x7C;
    break;
  case 'z':
    glyph[0] = 0x44;
    glyph[1] = 0x64;
    glyph[2] = 0x54;
    glyph[3] = 0x4C;
    glyph[4] = 0x44;
    break;
  default:
    break;
  }
}

HAL_StatusTypeDef OledSsd1306_Init(I2C_HandleTypeDef *i2c)
{
  static const uint8_t init_commands[] = {
      0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40, 0x81,
      0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3, 0x00, 0xD5,
      0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB, 0x40, 0x8D, 0x14,
      0xAF,
  };
  const uint16_t addresses[] = {OLED_SSD1306_I2C_ADDR_0,
                                OLED_SSD1306_I2C_ADDR_1};

  if (i2c == 0)
  {
    return HAL_ERROR;
  }

  oled_i2c = i2c;
  oled_address = 0;

  for (uint32_t index = 0; index < (sizeof(addresses) / sizeof(addresses[0]));
       index++)
  {
    if (HAL_I2C_IsDeviceReady(oled_i2c, addresses[index], 2, 20) == HAL_OK)
    {
      oled_address = addresses[index];
      break;
    }
  }

  if (oled_address == 0U)
  {
    return HAL_ERROR;
  }

  HAL_Delay(50);
  for (uint32_t index = 0; index < sizeof(init_commands); index++)
  {
    if (OledSsd1306_WriteCommand(init_commands[index]) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  OledSsd1306_Clear();
  return OledSsd1306_Update();
}

void OledSsd1306_Clear(void)
{
  memset(oled_buffer, 0, sizeof(oled_buffer));
}

void OledSsd1306_WriteText(uint8_t row, uint8_t column, const char *text)
{
  uint16_t x;

  if ((row >= OLED_SSD1306_ROWS) || (text == 0))
  {
    return;
  }

  x = (uint16_t)column * OLED_SSD1306_CHAR_WIDTH;
  while ((*text != '\0') && ((x + OLED_SSD1306_CHAR_WIDTH) <= OLED_SSD1306_WIDTH))
  {
    uint8_t glyph[5];
    uint16_t offset = ((uint16_t)row * OLED_SSD1306_WIDTH) + x;

    OledSsd1306_GetGlyph(*text, glyph);
    for (uint32_t index = 0; index < 5U; index++)
    {
      oled_buffer[offset + index] = glyph[index];
    }
    oled_buffer[offset + 5U] = 0x00;

    text++;
    x = (uint16_t)(x + OLED_SSD1306_CHAR_WIDTH);
  }
}

HAL_StatusTypeDef OledSsd1306_Update(void)
{
  if ((oled_i2c == 0) || (oled_address == 0U))
  {
    return HAL_ERROR;
  }

  for (uint8_t page = 0; page < OLED_SSD1306_ROWS; page++)
  {
    if (OledSsd1306_WriteCommand((uint8_t)(0xB0U + page)) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (OledSsd1306_WriteCommand(0x00) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (OledSsd1306_WriteCommand(0x10) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (OledSsd1306_WriteData(&oled_buffer[page * OLED_SSD1306_WIDTH],
                              OLED_SSD1306_WIDTH) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}
