#include "motion_telemetry.h"

#include <stdarg.h>
#include <stdio.h>

static int append_format(char *buffer, uint32_t buffer_size,
                         uint32_t *offset, const char *format, ...)
{
  va_list args;
  int written;

  if ((buffer == 0) || (offset == 0) || (format == 0) ||
      (*offset >= buffer_size))
  {
    return -1;
  }

  va_start(args, format);
  written = vsnprintf(&buffer[*offset], buffer_size - *offset, format, args);
  va_end(args);

  if ((written < 0) || ((uint32_t)written >= (buffer_size - *offset)))
  {
    return -1;
  }

  *offset += (uint32_t)written;
  return 0;
}

static int32_t round_thousandths(float value)
{
  if (value >= 0.0f)
  {
    return (int32_t)((value * 1000.0f) + 0.5f);
  }
  return (int32_t)((value * 1000.0f) - 0.5f);
}

static int append_fixed3(char *buffer, uint32_t buffer_size,
                         uint32_t *offset, float value)
{
  int32_t scaled;
  int32_t whole;
  int32_t fraction;
  const char *sign;

  scaled = round_thousandths(value);
  sign = (scaled < 0) ? "-" : "";
  if (scaled < 0)
  {
    scaled = -scaled;
  }

  whole = scaled / 1000;
  fraction = scaled % 1000;
  return append_format(buffer, buffer_size, offset, ",%s%ld.%03ld",
                       sign, (long)whole, (long)fraction);
}

int MotionTelemetry_BuildHeader(char *buffer, uint32_t buffer_size)
{
  int written;

  if ((buffer == 0) || (buffer_size == 0U))
  {
    return -1;
  }

  written = snprintf(buffer, buffer_size,
                     "t,x1,x2,y1,y2,"
                     "s_boom,s_stick,s_bucket,"
                     "v_boom,v_stick,v_bucket,"
                     "a_boom,a_stick,a_bucket,yaw,yaw_rate,"
                     "rs485_ok,adc_ok,imu_ok\n");
  if ((written < 0) || ((uint32_t)written >= buffer_size))
  {
    return -1;
  }

  return written;
}

int MotionTelemetry_BuildRow(char *buffer, uint32_t buffer_size,
                             const MotionTelemetry *telemetry)
{
  uint32_t offset;

  if ((buffer == 0) || (telemetry == 0) || (buffer_size == 0U))
  {
    return -1;
  }

  offset = 0U;
  if (append_format(buffer, buffer_size, &offset, "%lu",
                    (unsigned long)telemetry->t_ms) != 0)
  {
    return -1;
  }

  if ((append_fixed3(buffer, buffer_size, &offset, telemetry->x1) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->x2) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->y1) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->y2) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->s_boom) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->s_stick) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->s_bucket) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->v_boom) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->v_stick) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->v_bucket) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->a_boom) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->a_stick) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->a_bucket) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->yaw) != 0) ||
      (append_fixed3(buffer, buffer_size, &offset, telemetry->yaw_rate) != 0))
  {
    return -1;
  }

  if (append_format(buffer, buffer_size, &offset, ",%u,%u,%u\n",
                    telemetry->rs485_ok, telemetry->adc_ok,
                    telemetry->imu_ok) != 0)
  {
    return -1;
  }

  return (int)offset;
}
