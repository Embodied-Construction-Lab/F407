#include "stick_receiver.h"

#include <assert.h>

int main(void)
{
  StickData stick;
  const char *valid =
      "{\"schema_version\":\"stm32_manual_command.v1\","
      "\"X1\":-0.8,\"Y1\":0.4,\"Z1\":0,"
      "\"X2\":0.2,\"Y2\":-0.6,\"Z2\":0,"
      "\"command_seq\":7,\"command_source_stamp_ms\":1234}";
  const char *unknown_schema =
      "{\"schema_version\":\"unknown.v1\","
      "\"X1\":0,\"Y1\":0,\"Z1\":0,\"X2\":0,\"Y2\":0,\"Z2\":0,"
      "\"command_seq\":8,\"command_source_stamp_ms\":1235}";
  const char *missing_sequence =
      "{\"schema_version\":\"stm32_manual_command.v1\","
      "\"X1\":0,\"Y1\":0,\"Z1\":0,\"X2\":0,\"Y2\":0,\"Z2\":0}";

  assert(StickReceiver_ParseJson(valid, &stick));
  assert(stick.command_seq == 7U);
  assert(stick.command_source_stamp_ms == 1234U);
  assert(!StickReceiver_ParseJson(unknown_schema, &stick));
  assert(!StickReceiver_ParseJson(missing_sequence, &stick));
  assert(StickReceiver_IsNewSequence(0U, 0U, false));
  assert(StickReceiver_IsNewSequence(8U, 7U, true));
  assert(!StickReceiver_IsNewSequence(7U, 7U, true));
  assert(!StickReceiver_IsNewSequence(6U, 7U, true));
  return 0;
}
