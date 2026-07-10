#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static int test_snprintf(char *buffer, size_t buffer_size,
                         const char *format, ...)
{
  va_list args;
  int written;

  if ((strstr(format, "%f") != NULL) || (strstr(format, "%.3f") != NULL))
  {
    return -1;
  }

  va_start(args, format);
  written = vsnprintf(buffer, buffer_size, format, args);
  va_end(args);
  return written;
}

#define snprintf test_snprintf
#include "../Core/Src/motion_telemetry.c"
#undef snprintf

int main(void)
{
  char buffer[256];
  MotionTelemetry telemetry = {
      .t_ms = 36608U,
      .x1 = 0.0f,
      .x2 = -0.125f,
      .y1 = 0.4f,
      .y2 = 1.0f,
      .s_boom = 1.25f,
      .s_stick = 2.5f,
      .s_bucket = -3.75f,
      .v_boom = 10.0f,
      .v_stick = -20.0f,
      .v_bucket = 30.5f,
      .a_boom = 0.0f,
      .a_stick = 45.125f,
      .a_bucket = -0.001f,
      .yaw = 180.0f,
      .yaw_rate = -1.5f,
      .rs485_ok = 1U,
      .adc_ok = 1U,
      .imu_ok = 1U,
  };

  int header_len = MotionTelemetry_BuildHeader(buffer, sizeof(buffer));
  assert(header_len > 0);
  assert(strcmp(buffer,
                "t,x1,x2,y1,y2,"
                "s_boom,s_stick,s_bucket,"
                "v_boom,v_stick,v_bucket,"
                "a_boom,a_stick,a_bucket,yaw,yaw_rate,"
                "rs485_ok,adc_ok,imu_ok\n") == 0);

  int row_len = MotionTelemetry_BuildRow(buffer, sizeof(buffer), &telemetry);
  assert(row_len > 0);
  assert(strcmp(buffer,
                "36608,0.000,-0.125,0.400,1.000,"
                "1.250,2.500,-3.750,10.000,-20.000,30.500,"
                "0.000,45.125,-0.001,180.000,-1.500,1,1,1\n") == 0);

  assert(MotionTelemetry_BuildRow(buffer, 4U, &telemetry) == -1);
  assert(MotionTelemetry_BuildRow(buffer, sizeof(buffer), 0) == -1);
  return 0;
}
