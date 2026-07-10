#include "imu_oled_format.h"

#include <math.h>
#include <stdint.h>

static void append_char(char **cursor, size_t *remaining, char value)
{
  if (*remaining <= 1U)
  {
    return;
  }
  **cursor = value;
  (*cursor)++;
  (*remaining)--;
  **cursor = '\0';
}

static void append_text(char **cursor, size_t *remaining, const char *text)
{
  while (*text != '\0')
  {
    append_char(cursor, remaining, *text++);
  }
}

void imu_oled_format_row(char *output, size_t capacity,
                         const char *label, float value)
{
  char *cursor = output;
  size_t remaining = capacity;
  float magnitude;
  uint32_t thousandths;
  uint32_t whole;
  uint32_t fraction;

  if (capacity == 0U)
  {
    return;
  }
  output[0] = '\0';

  append_text(&cursor, &remaining, label);
  append_char(&cursor, &remaining, ':');

  if (isnan(value))
  {
    append_text(&cursor, &remaining, " nan");
    return;
  }
  if (isinf(value))
  {
    append_text(&cursor, &remaining, signbit(value) ? "-inf" : " inf");
    return;
  }

  if (signbit(value))
  {
    append_char(&cursor, &remaining, '-');
    magnitude = -value;
  }
  else
  {
    append_char(&cursor, &remaining, ' ');
    magnitude = value;
  }

  if (magnitude > 9999.999f)
  {
    append_text(&cursor, &remaining, "ovf");
    return;
  }

  thousandths = (uint32_t)(magnitude * 1000.0f + 0.5f);
  whole = thousandths / 1000U;
  fraction = thousandths % 1000U;

  if (whole >= 1000U)
  {
    append_char(&cursor, &remaining, (char)('0' + ((whole / 1000U) % 10U)));
  }
  if (whole >= 100U)
  {
    append_char(&cursor, &remaining, (char)('0' + ((whole / 100U) % 10U)));
  }
  if (whole >= 10U)
  {
    append_char(&cursor, &remaining, (char)('0' + ((whole / 10U) % 10U)));
  }
  append_char(&cursor, &remaining, (char)('0' + (whole % 10U)));
  append_char(&cursor, &remaining, '.');
  append_char(&cursor, &remaining, (char)('0' + (fraction / 100U)));
  append_char(&cursor, &remaining, (char)('0' + ((fraction / 10U) % 10U)));
  append_char(&cursor, &remaining, (char)('0' + (fraction % 10U)));
}
