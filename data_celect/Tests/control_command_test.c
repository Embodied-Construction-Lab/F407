#include "control_command.h"

#include <assert.h>

static void assert_float_equal(float actual, float expected)
{
  assert(actual > expected - 0.0001f);
  assert(actual < expected + 0.0001f);
}

int main(void)
{
  ControlCommand command;
  const char *manual =
      "{\"schema_version\":\"stm32_manual_command.v1\","
      "\"X1\":-0.8,\"Y1\":0.4,\"Z1\":0.7,"
      "\"X2\":0.2,\"Y2\":-0.6,\"Z2\":-0.5,"
      "\"command_seq\":40,\"command_source_stamp_ms\":9019}";
  const char *velocity =
      "{\"schema_version\":\"stm32_velocity_command.v1\","
      "\"boom_mps\":0.12,\"stick_mps\":-0.08,"
      "\"bucket_mps\":0.04,\"swing_radps\":-0.15,"
      "\"command_seq\":41,\"command_source_stamp_ms\":9021}";
  const char *corrupt_velocity =
      "{\"schema_version\":\"stm32_velocity_command.v1\","
      "\"boom_mps\":0.12oops,\"stick_mps\":0,"
      "\"bucket_mps\":0,\"swing_radps\":0,"
      "\"command_seq\":42,\"command_source_stamp_ms\":9022}";

  assert(ControlCommand_ParseJson(manual, &command));
  assert(command.mode == CONTROL_MODE_MANUAL_ACTION);
  assert(command.command_seq == 40U);
  assert_float_equal(command.axis.boom, -0.6f);
  assert_float_equal(command.axis.stick, 0.4f);
  assert_float_equal(command.axis.bucket, 0.2f);
  assert_float_equal(command.axis.swing, -0.8f);
  assert_float_equal(command.manual_z1, 0.7f);
  assert_float_equal(command.manual_z2, -0.5f);

  assert(ControlCommand_ParseJson(velocity, &command));
  assert(command.mode == CONTROL_MODE_VELOCITY_REFERENCE);
  assert(command.command_seq == 41U);
  assert(command.command_source_stamp_ms == 9021U);
  assert_float_equal(command.axis.boom, 0.12f);
  assert_float_equal(command.axis.stick, -0.08f);
  assert_float_equal(command.axis.bucket, 0.04f);
  assert_float_equal(command.axis.swing, -0.15f);
  assert(!ControlCommand_ParseJson(corrupt_velocity, &command));
  return 0;
}
