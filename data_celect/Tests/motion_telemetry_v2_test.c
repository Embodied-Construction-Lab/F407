#include "motion_telemetry.h"

#include <assert.h>
#include <string.h>

int main(void)
{
  char buffer[1024];
  MotionTelemetry telemetry = {0};
  int header_length;
  int row_length;

  telemetry.control_seq = 7U;
  telemetry.sensor_seq = 4U;
  telemetry.command_action_boom = -0.6f;
  telemetry.command_action_stick = 0.4f;
  telemetry.command_action_bucket = 0.2f;
  telemetry.command_action_swing = -0.8f;
  telemetry.command_valid = 1U;

  header_length = MotionTelemetry_BuildHeader(buffer, sizeof(buffer));
  assert(header_length > 0);
  assert(strncmp(buffer, "schema_version,control_seq", 26U) == 0);
  assert(strstr(buffer, "command_action_boom,command_action_stick,") != 0);
  assert(strstr(buffer, "command_action_bucket,command_action_swing,") != 0);

  row_length = MotionTelemetry_BuildRow(buffer, sizeof(buffer), &telemetry);
  assert(row_length > 0);
  assert(strncmp(buffer, "stm32_control_telemetry.v2,7,", 29U) == 0);
  assert(strstr(buffer, ",-0.600,0.400,0.200,-0.800,") != 0);
  assert(row_length < (int)sizeof(buffer));
  /* 8N1 costs 10 bits/byte. Even a full 1023-byte row at 20 Hz uses
     204600 bit/s, below the fixed USART2 rate of 460800 bit/s. */
  assert(((sizeof(buffer) - 1U) * 20U * 10U) < 460800U);
  assert(MotionTelemetry_BuildRow(buffer, 32U, &telemetry) == -1);
  return 0;
}
