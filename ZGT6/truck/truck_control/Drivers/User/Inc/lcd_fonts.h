#ifndef LCD_FONTS_H
#define LCD_FONTS_H

#include <stdint.h>

typedef struct
{
  const uint8_t *pTable;
  uint16_t Width;
  uint16_t Height;
  uint16_t Sizes;
  uint16_t Table_Rows;
} pFONT;

extern pFONT ASCII_Font16;

#endif /* LCD_FONTS_H */

