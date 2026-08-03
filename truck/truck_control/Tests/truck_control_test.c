#include "pwm_timing.h"
#include "servo_control.h"
#include "truck_control.h"
#include "truck_receiver.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_direct_command_json_and_neutral(void)
{
  const char *json =
      "{\"type\":\"truck_control\",\"timestamp\":1780000000.0,"
      "\"steering\":0.0,\"throttle\":0.0,\"brake\":0.0}";
  TruckCommand command;
  TruckOutputs outputs;

  assert(TruckReceiver_ParseJson(json, &command));
  TruckControl_MapCommand(&command, &outputs);
  assert(outputs.steering_deg == 0);
  assert(outputs.throttle_percent == 0);
  assert(outputs.brake_percent == 0);
  assert(outputs.drive_percent == 0);
  assert(outputs.lift_percent == 0);
  assert(outputs.pwm_count[TRUCK_CHANNEL_STEERING] == 307U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_LIFT] == 307U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_DRIVE] == 307U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_UNUSED] == 0U);
}

static void test_steering_throttle_and_brake_mapping(void)
{
  TruckCommand command = {0};
  TruckOutputs outputs;

  command.steering = -30.0f;
  command.throttle = 1.0f;
  command.brake = 0.0f;
  TruckControl_MapCommand(&command, &outputs);
  assert(outputs.steering_deg == -30);
  assert(outputs.throttle_percent == 100);
  assert(outputs.brake_percent == 0);
  assert(outputs.drive_percent == 20);
  assert(outputs.lift_percent == 0);
  assert(outputs.pwm_count[TRUCK_CHANNEL_STEERING] == 239U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_LIFT] == 307U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_DRIVE] == 348U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_UNUSED] == 0U);

  command.steering = 30.0f;
  command.throttle = 1.0f;
  command.brake = 1.0f;
  TruckControl_MapCommand(&command, &outputs);
  assert(outputs.steering_deg == 30);
  assert(outputs.pwm_count[TRUCK_CHANNEL_STEERING] == 375U);
  assert(outputs.throttle_percent == 100);
  assert(outputs.brake_percent == 100);
  assert(outputs.drive_percent == 0);
  assert(outputs.lift_percent == 0);
  assert(outputs.pwm_count[TRUCK_CHANNEL_DRIVE] == 307U);
  assert(outputs.pwm_count[TRUCK_CHANNEL_LIFT] == 307U);

  command.steering = 0.0f;
  command.throttle = -1.0f;
  command.brake = 0.0f;
  TruckControl_MapCommand(&command, &outputs);
  assert(outputs.throttle_percent == -100);
  assert(outputs.brake_percent == 0);
  assert(outputs.drive_percent == -20);
  assert(outputs.pwm_count[TRUCK_CHANNEL_DRIVE] == 266U);

  command.throttle = -0.5f;
  command.brake = 0.1f;
  TruckControl_MapCommand(&command, &outputs);
  assert(outputs.throttle_percent == -50);
  assert(outputs.brake_percent == 10);
  assert(outputs.drive_percent == 0);
  assert(outputs.pwm_count[TRUCK_CHANNEL_DRIVE] == 307U);

  command.steering = 50.0f;
  command.throttle = 2.0f;
  command.brake = -1.0f;
  TruckControl_MapCommand(&command, &outputs);
  assert(outputs.steering_deg == 30);
  assert(outputs.throttle_percent == 100);
  assert(outputs.brake_percent == 0);
  assert(outputs.drive_percent == 20);
}

static void test_lift_limit_is_applied_by_output_setter(void)
{
  TruckOutputs outputs;

  TruckControl_SetNeutral(&outputs);
  TruckControl_SetLiftPercent(&outputs, 100);
  assert(outputs.lift_percent == TRUCK_LIFT_UP_MAX_PERCENT);
  assert(outputs.pwm_count[TRUCK_CHANNEL_LIFT] ==
         ServoControl_SpeedToPulse((float)TRUCK_LIFT_UP_MAX_PERCENT));

  TruckControl_SetLiftPercent(&outputs, -100);
  assert(outputs.lift_percent == -TRUCK_LIFT_DOWN_MAX_PERCENT);
  assert(outputs.pwm_count[TRUCK_CHANNEL_LIFT] ==
         ServoControl_SpeedToPulse(
             -(float)TRUCK_LIFT_DOWN_MAX_PERCENT));
}

static void test_split_dma_frame(void)
{
  const char *json =
      "{\"type\":\"truck_control\",\"steering\":-12.5,"
      "\"throttle\":-0.5,\"brake\":0.0}\n";
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
  assert(command.steering == -12.5f);
  assert(command.throttle == -0.5f);
  assert(command.brake == 0.0f);
}

static void test_incomplete_json_is_rejected(void)
{
  TruckCommand update;
  uint8_t field_mask;

  assert(TruckReceiver_ParseJsonUpdate("{\"steering\":10}", &update,
                                       &field_mask));
  assert(field_mask == TRUCK_FIELD_STEERING);
  assert(update.steering == 10.0f);
  assert(!TruckReceiver_ParseJson("{\"steering\":10}", &update));

  assert(!TruckReceiver_ParseJsonUpdate("{\"type\":\"truck_control\"}",
                                        &update, &field_mask));
}

static void test_pwm_timing_and_timeout(void)
{
  assert(PwmTiming_CalculatePrescale(PWM_OSCILLATOR_HZ,
                                     PWM_TARGET_FREQUENCY_HZ) == 133U);
  assert(PwmTiming_DefaultPulseUsToCount(500U) == 102U);
  assert(PwmTiming_DefaultPulseUsToCount(1500U) == 307U);
  assert(PwmTiming_DefaultPulseUsToCount(2500U) == 512U);
  assert(PwmTiming_DefaultCountToPulseUs(307U) == 1499U);
  assert(TruckControl_IsActiveChannel(TRUCK_CHANNEL_STEERING));
  assert(!TruckControl_IsActiveChannel(TRUCK_CHANNEL_UNUSED));
  assert(!TruckControl_IsTimedOut(1300U, 1000U));
  assert(TruckControl_IsTimedOut(1301U, 1000U));
}

static void test_nonblocking_esc_reverse_sequence(void)
{
  TruckEscController esc;

  TruckEsc_Init(&esc);
  assert(esc.state == TRUCK_ESC_NEUTRAL);
  assert(TruckEsc_Update(&esc, 50, 100U) == 50);
  assert(esc.state == TRUCK_ESC_FORWARD);

  assert(TruckEsc_Update(&esc, -50, 200U) == -50);
  assert(esc.state == TRUCK_ESC_BRAKE_FOR_REVERSE);
  assert(TruckEsc_Update(&esc, -50, 499U) == -50);
  assert(esc.state == TRUCK_ESC_BRAKE_FOR_REVERSE);

  assert(TruckEsc_Update(&esc, -50, 500U) == 0);
  assert(esc.state == TRUCK_ESC_REVERSE_NEUTRAL);
  assert(TruckEsc_Update(&esc, -50, 699U) == 0);
  assert(esc.state == TRUCK_ESC_REVERSE_NEUTRAL);

  assert(TruckEsc_Update(&esc, -50, 700U) == -50);
  assert(esc.state == TRUCK_ESC_REVERSE);
  assert(TruckEsc_Update(&esc, 0, 710U) == 0);
  assert(esc.state == TRUCK_ESC_NEUTRAL);
}

static void test_esc_reverse_abort(void)
{
  TruckEscController esc;

  TruckEsc_Init(&esc);
  assert(TruckEsc_Update(&esc, -25, 1000U) == -25);
  assert(esc.state == TRUCK_ESC_BRAKE_FOR_REVERSE);
  assert(TruckEsc_Update(&esc, 0, 1100U) == 0);
  assert(esc.state == TRUCK_ESC_NEUTRAL);
}

int main(void)
{
  test_direct_command_json_and_neutral();
  test_steering_throttle_and_brake_mapping();
  test_lift_limit_is_applied_by_output_setter();
  test_split_dma_frame();
  test_incomplete_json_is_rejected();
  test_pwm_timing_and_timeout();
  test_nonblocking_esc_reverse_sequence();
  test_esc_reverse_abort();
  puts("F407 truck_control tests passed");
  return 0;
}
