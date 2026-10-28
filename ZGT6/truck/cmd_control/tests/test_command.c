#include "truck_control.h"
#include "truck_receiver.h"

#include <assert.h>
#include <string.h>

static void test_bridge_command(void)
{
  const char *frame =
      "{\"type\":\"truck_control\",\"timestamp\":1.25,"
      "\"steering\":30,\"throttle\":1,\"brake\":0,"
      "\"up\":1,\"down\":0}";
  TruckCommand command;
  TruckOutputs outputs;

  assert(TruckReceiver_ParseJson(frame, &command));
  TruckControl_MapCommand(&command, &outputs);
  assert(outputs.steering_deg == 30);
  assert(outputs.throttle_percent == 100);
  assert(outputs.drive_percent == 20);
  assert(outputs.lift_percent == 50);
  assert(outputs.pulse_us[TRUCK_CHANNEL_STEERING] == 1833);
  assert(outputs.pulse_us[TRUCK_CHANNEL_LIFT] == 2000);
  assert(outputs.pulse_us[TRUCK_CHANNEL_DRIVE] == 1700);
}

static void test_optional_and_reverse(void)
{
  TruckCommand command;
  TruckOutputs outputs;
  uint8_t mask;

  assert(TruckReceiver_ParseJsonUpdate(
      "{\"type\":\"truck_control\",\"steering\":-30,"
      "\"throttle\":-1}", &command, &mask));
  assert(mask == (TRUCK_FIELD_STEERING | TRUCK_FIELD_THROTTLE));
  assert(command.brake == 0.0f && command.up == 0U && command.down == 0U);
  TruckControl_MapCommand(&command, &outputs);
  assert(outputs.steering_deg == -30);
  assert(outputs.throttle_percent == -100);
  assert(outputs.drive_percent == -20);
  assert(outputs.lift_percent == 0);
  assert(outputs.pulse_us[TRUCK_CHANNEL_STEERING] == 1167);
  assert(outputs.pulse_us[TRUCK_CHANNEL_DRIVE] == 1300);

  command.brake = 0.01f;
  TruckControl_MapCommand(&command, &outputs);
  assert(outputs.drive_percent == 0);
  assert(outputs.pulse_us[TRUCK_CHANNEL_DRIVE] == 1500);

  assert(TruckReceiver_ParseJsonUpdate(
      "{\"type\":\"button\",\"up\":true}", &command, &mask));
  assert(mask == TRUCK_FIELD_UP && command.up == 1U);
  assert(!TruckReceiver_ParseJsonUpdate(
      "{\"type\":\"logi_raw\",\"steering\":0}", &command, &mask));
  assert(!TruckReceiver_ParseJsonUpdate(
      "{\"steering\":0", &command, &mask));
  assert(!TruckReceiver_ParseJsonUpdate(
      "{\"steering\":1foo}", &command, &mask));
}

static void test_split_uart_frame(void)
{
  const char *first = "{\"steering\":0,\"throttle\":0";
  const char *last = ",\"brake\":0}\n";
  TruckReceiver receiver;
  TruckCommand command;
  char frame[TRUCK_FRAME_MAX_LEN];

  TruckReceiver_Init(&receiver);
  TruckReceiver_FeedFromIsr(&receiver, (const uint8_t *)first,
                            (uint16_t)strlen(first));
  assert(!TruckReceiver_Pop(&receiver, frame, sizeof(frame)));
  TruckReceiver_FeedFromIsr(&receiver, (const uint8_t *)last,
                            (uint16_t)strlen(last));
  assert(TruckReceiver_Pop(&receiver, frame, sizeof(frame)));
  assert(TruckReceiver_ParseJson(frame, &command));
  assert(!TruckReceiver_Pop(&receiver, frame, sizeof(frame)));
}

int main(void)
{
  test_bridge_command();
  test_optional_and_reverse();
  test_split_uart_frame();
  return 0;
}
