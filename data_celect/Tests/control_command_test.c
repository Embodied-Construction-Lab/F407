#include "control_command.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

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
      "{\"schema_version\":\"stm32_velocity_command.v2\","
      "\"boom_mps\":0.12,\"stick_mps\":-0.08,"
      "\"bucket_mps\":0.04,\"swing_radps\":-0.15,"
      "\"command_seq\":41,\"command_source_stamp_ms\":9021}";
  const char *corrupt_velocity =
      "{\"schema_version\":\"stm32_velocity_command.v2\","
      "\"boom_mps\":0.12oops,\"stick_mps\":0,"
      "\"bucket_mps\":0,\"swing_radps\":0,"
      "\"command_seq\":42,\"command_source_stamp_ms\":9022}";
  const char *legacy_velocity =
      "{\"schema_version\":\"stm32_velocity_command.v1\","
      "\"boom_mps\":0.01,\"stick_mps\":0,\"bucket_mps\":0,\"swing_radps\":0,"
      "\"command_seq\":43,\"command_source_stamp_ms\":9023}";

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
  /* v1 used the opposite cylinder convention: never silently reinterpret it. */
  assert(!ControlCommand_ParseJson(legacy_velocity, &command));
  /* A truncated or ambiguous frame must not become a motion command. */
  {
    char bad[512];
    strcpy(bad, manual);
    strcat(bad, "oops");
    assert(!ControlCommand_ParseJson(bad, &command));
    snprintf(bad, sizeof(bad), "%.*s,", (int)strlen(manual) - 1, manual);
    assert(!ControlCommand_ParseJson(bad, &command));
    snprintf(bad, sizeof(bad), "%.*s,\"Y2\":0}",
             (int)strlen(manual) - 1, manual);
    assert(!ControlCommand_ParseJson(bad, &command));
  }
  {
    const char *bad_axis[] = {
      "5", "-1.01", "true", "null", "\"0\"", "NaN", "Infinity",
      "+0.5", ".5", "1.", "01", "0x1p0", "1e", "1e999", "1e-999",
      "[0]", "{\"value\":0}"
    };
    const char *bad_seq[] = {"-1", "+1", "01", "1.0", "1e0", "4294967296", "true"};
    char frame[512];
    size_t index;
    ControlCommand unchanged = command;
    for (index = 0U; index < sizeof(bad_axis) / sizeof(bad_axis[0]); ++index)
    {
      snprintf(frame, sizeof(frame),
               "{\"schema_version\":\"stm32_manual_command.v1\","
               "\"X1\":0,\"Y1\":0,\"Z1\":0,\"X2\":0,\"Y2\":%s,\"Z2\":0,"
               "\"command_seq\":1,\"command_source_stamp_ms\":2}", bad_axis[index]);
      assert(!ControlCommand_ParseJson(frame, &command));
      assert(memcmp(&command, &unchanged, sizeof(command)) == 0);
    }
    for (index = 0U; index < sizeof(bad_seq) / sizeof(bad_seq[0]); ++index)
    {
      snprintf(frame, sizeof(frame),
               "{\"schema_version\":\"stm32_velocity_command.v2\","
               "\"boom_mps\":0,\"stick_mps\":0,\"bucket_mps\":0,\"swing_radps\":0,"
               "\"command_seq\":%s,\"command_source_stamp_ms\":2}", bad_seq[index]);
      assert(!ControlCommand_ParseJson(frame, &command));
    }
    assert(!ControlCommand_ParseJson("{}", &command));
    assert(!ControlCommand_ParseJson(manual + 1, &command));
    snprintf(frame, sizeof(frame), "%.*s,\"unexpected\":0}",
             (int)strlen(manual) - 1, manual);
    assert(!ControlCommand_ParseJson(frame, &command));
    snprintf(frame, sizeof(frame), "%.*s,\"boom_mps\":0}",
             (int)strlen(manual) - 1, manual);
    assert(!ControlCommand_ParseJson(frame, &command));
    snprintf(frame, sizeof(frame), "%.*s,}", (int)strlen(manual) - 1, manual);
    assert(!ControlCommand_ParseJson(frame, &command));
    assert(ControlCommand_ParseJson(
      " \t{\"command_seq\":4294967295,\"command_source_stamp_ms\":0,"
      "\"boom_mps\":1.2e-2,\"stick_mps\":-0.08,\"bucket_mps\":-0,"
      "\"swing_radps\":3.4E-1,\"schema_version\":\"stm32_velocity_command.v2\"}\r\n", &command));
    assert(command.command_seq == UINT32_MAX);
    assert_float_equal(command.axis.boom, 0.012f);
    assert_float_equal(command.axis.swing, 0.34f);
  }
  return 0;
}
