#include "loader_control.h"
#include "servo_control.h"
#include "stick_receiver.h"

#include <assert.h>
#include <math.h>
#include <string.h>

static int near(float actual, float expected)
{
  return fabsf(actual - expected) < 0.02f;
}

int main(void)
{
  const char *json =
      "{\"X1\":0.5,\"Y1\":1,\"Z1\":0,\"X2\":-1,\"Y2\":1,\"Z2\":0}\n";
  StickReceiver receiver;
  StickData stick;
  LoaderTargets targets;
  char frame[STICK_FRAME_MAX_LEN];
  size_t split = 17U;

  StickReceiver_Init(&receiver);
  StickReceiver_FeedFromIsr(&receiver, (const uint8_t *)json, (uint16_t)split);
  assert(!StickReceiver_Pop(&receiver, frame, sizeof(frame)));
  StickReceiver_FeedFromIsr(&receiver, (const uint8_t *)json + split,
                            (uint16_t)(strlen(json) - split));
  assert(StickReceiver_Pop(&receiver, frame, sizeof(frame)));
  assert(StickReceiver_ParseJson(frame, &stick));
  assert(!StickReceiver_ParseJson("{\"X1\":0}", &stick));

  LoaderControl_Compute(&stick, &targets);
  assert(near(targets.valve_deg[LOADER_CH_BOOM], 115.0f));
  assert(near(targets.valve_deg[LOADER_CH_BUCKET], 71.0f));
  assert(targets.valve_deg[LOADER_CH_STEERING] > 105.0f);
  assert(targets.valve_deg[LOADER_CH_STEERING] < 112.0f);
  assert(near(targets.drive_percent, 20.0f));
  assert(near(targets.pump_percent, -30.0f));
  assert(targets.pwm_count[LOADER_CH_DRIVE] ==
         ServoControl_SpeedToPulse(20.0f));

  stick.x1 = 0.0f;
  stick.x2 = 0.0f;
  stick.y2 = 0.0f;
  stick.y1 = 0.1f;
  LoaderControl_Compute(&stick, &targets);
  assert(near(targets.drive_percent, 0.0f));
  assert(near(targets.pump_percent, 0.0f));
  stick.y1 = -1.0f;
  LoaderControl_Compute(&stick, &targets);
  assert(near(targets.drive_percent, -20.0f));
  assert(near(targets.pump_percent, 0.0f));

  LoaderControl_SetNeutral(&targets);
  assert(near(targets.valve_deg[LOADER_CH_BOOM], 90.0f));
  assert(near(targets.valve_deg[LOADER_CH_BUCKET], 90.0f));
  assert(near(targets.valve_deg[LOADER_CH_STEERING], 90.0f));
  assert(near(targets.drive_percent, 0.0f));
  assert(near(targets.pump_percent, 0.0f));
  assert(targets.pwm_count[LOADER_CH_DRIVE] ==
         ServoControl_SpeedToPulse(0.0f));
  return 0;
}
