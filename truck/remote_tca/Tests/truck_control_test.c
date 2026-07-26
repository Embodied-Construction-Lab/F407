#include "pwm_timing.h"
#include "truck_control.h"
#include "truck_receiver.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_tca_json_and_neutral(void)
{
  const char *json =
      "{\"type\":\"logi_raw\",\"timestamp\":1780000000.0,"
      "\"steering\":0.0,\"axis0\":0.0,\"axis1\":0.0}";
  TruckCommand command;
  TruckOutputs outputs;

  assert(TruckReceiver_ParseJson(json, &command));
  assert(command.drive_axis == 0.0f);
  assert(command.lift_axis == 0.0f);
  TruckControl_MapRawCommand(&command, &outputs);
  assert(outputs.steering_deg == 90);
  assert(outputs.drive_percent == 0);
  assert(outputs.lift_percent == 0);
  assert(outputs.pwm_count[TRUCK_CHANNEL_STEERING] == 306U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_LIFT] == 306U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_DRIVE] == 306U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_UNUSED] == 0U);
}

static void test_tca_drive_and_lift_axes(void)
{
  TruckCommand command = {0};
  TruckOutputs outputs;

  command.steering = -1.0f;
  command.drive_axis = -1.0f;
  command.lift_axis = -1.0f;
  TruckControl_MapRawCommand(&command, &outputs);
  assert(outputs.steering_deg == 0);
  assert(outputs.drive_percent == 50);
  assert(outputs.lift_percent == 90);
  assert(outputs.pwm_count[TRUCK_CHANNEL_STEERING] == 102U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_LIFT] == 490U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_DRIVE] == 408U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_UNUSED] == 0U);

  command.drive_axis = 1.0f;
  command.lift_axis = 0.0f;
  TruckControl_MapRawCommand(&command, &outputs);
  assert(outputs.drive_percent == -50);
  assert(outputs.lift_percent == 0);
  assert(outputs.pwm_count[TRUCK_CHANNEL_DRIVE] == 204U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_LIFT] == 306U);

  command.lift_axis = 1.0f;
  TruckControl_MapRawCommand(&command, &outputs);
  assert(outputs.lift_percent == -50);
  assert(outputs.pwm_count[TRUCK_CHANNEL_LIFT] == 204U);
}

static void test_bridge_tca_legacy_fields(void)
{
  const char *forward_up =
      "{\"type\":\"logi_raw\",\"steering\":0.25,"
      "\"throttle\":-1.0,\"brake\":1.0,\"up\":1,\"down\":0}";
  const char *reverse_down =
      "{\"type\":\"logi_raw\",\"steering\":0.25,"
      "\"throttle\":1.0,\"brake\":-1.0,\"up\":0,\"down\":1}";
  TruckCommand command;

  assert(TruckReceiver_ParseJson(forward_up, &command));
  assert(command.drive_axis == -1.0f);
  assert(command.lift_axis == -1.0f);

  assert(TruckReceiver_ParseJson(reverse_down, &command));
  assert(command.drive_axis == 1.0f);
  assert(command.lift_axis == 1.0f);
}

static void test_split_dma_frame(void)
{
  const char *json =
      "{\"type\":\"logi_raw\",\"steering\":0.25,"
      "\"axis0\":-0.5,\"axis1\":0.75}\n";
  TruckReceiver receiver;
  TruckCommand command;
  char frame[TRUCK_FRAME_MAX_LEN];
  const size_t split = 17U;

  TruckReceiver_Init(&receiver);
  TruckReceiver_FeedFromIsr(&receiver, (const uint8_t *)json,
                            (uint16_t)split);
  assert(!TruckReceiver_Pop(&receiver, frame, sizeof(frame)));
  TruckReceiver_FeedFromIsr(&receiver,
                            (const uint8_t *)&json[split],
                            (uint16_t)(strlen(json) - split));
  assert(TruckReceiver_Pop(&receiver, frame, sizeof(frame)));
  assert(TruckReceiver_ParseJson(frame, &command));
  assert(command.steering == 0.25f);
  assert(command.drive_axis == -0.5f);
  assert(command.lift_axis == 0.75f);
}

static void test_pwm_timing_and_timeout(void)
{
  assert(PwmTiming_CalculatePrescale(PWM_OSCILLATOR_HZ,
                                     PWM_TARGET_FREQUENCY_HZ) == 129U);
  assert(PwmTiming_DefaultPulseUsToCount(500U) == 102U);
  assert(PwmTiming_DefaultPulseUsToCount(1500U) == 306U);
  assert(PwmTiming_DefaultPulseUsToCount(2500U) == 510U);
  assert(TruckControl_IsActiveChannel(TRUCK_CHANNEL_STEERING));
  assert(!TruckControl_IsActiveChannel(TRUCK_CHANNEL_UNUSED));
  assert(!TruckControl_IsTimedOut(1300U, 1000U));
  assert(TruckControl_IsTimedOut(1301U, 1000U));
}

int main(void)
{
  test_tca_json_and_neutral();
  test_tca_drive_and_lift_axes();
  test_bridge_tca_legacy_fields();
  test_split_dma_frame();
  test_pwm_timing_and_timeout();
  puts("F407 truck remote tests passed");
  return 0;
}
