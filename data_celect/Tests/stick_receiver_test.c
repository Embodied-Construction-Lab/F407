#include "stick_receiver.h"

#include <assert.h>
#include <string.h>

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
  {
    StickReceiver receiver;
    char frame[STICK_FRAME_MAX_LEN];
    const uint8_t nul_and_tail[] = {0U, 'b', 'a', 'd', '\n'};
    StickReceiver_Init(&receiver);
    /* A complete JSON prefix plus embedded NUL is still a corrupt wire frame. */
    StickReceiver_FeedFromIsr(&receiver, (const uint8_t *)valid, (uint16_t)strlen(valid));
    StickReceiver_FeedFromIsr(&receiver, nul_and_tail, sizeof(nul_and_tail));
    assert(!StickReceiver_Pop(&receiver, frame, sizeof(frame)));
    assert(receiver.dropped_frames == 1U);
    StickReceiver_FeedFromIsr(&receiver, (const uint8_t *)valid, (uint16_t)strlen(valid));
    StickReceiver_FeedFromIsr(&receiver, (const uint8_t *)"\n", 1U);
    assert(StickReceiver_Pop(&receiver, frame, sizeof(frame)));
    assert(StickReceiver_ParseJson(frame, &stick));
    assert(stick.command_seq == 7U);
    assert(!StickReceiver_Pop(&receiver, frame, sizeof(frame)));
  }
  return 0;
}
