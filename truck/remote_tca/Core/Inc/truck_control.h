#ifndef TRUCK_CONTROL_H
#define TRUCK_CONTROL_H

#include "truck_receiver.h"

#include <stdbool.h>
#include <stdint.h>

#define TRUCK_OUTPUT_CHANNELS 4U
#define TRUCK_CONTROL_TIMEOUT_MS 300U
#define TRUCK_STEERING_DEAD_ZONE 0.02f
#define TRUCK_TCA_AXIS_DEAD_ZONE 0.02f
#define TRUCK_DRIVE_MAX_PERCENT 50.0f
#define TRUCK_LIFT_UP_MAX_PERCENT 90.0f
#define TRUCK_LIFT_DOWN_MAX_PERCENT 50.0f

#define TRUCK_STEERING_DIRECTION 1.0f

typedef enum
{
  TRUCK_CHANNEL_STEERING = 0,
  TRUCK_CHANNEL_LIFT = 1,
  TRUCK_CHANNEL_DRIVE = 2,
  TRUCK_CHANNEL_UNUSED = 3
} TruckPcaChannel;

typedef struct
{
  uint16_t pwm_count[TRUCK_OUTPUT_CHANNELS];
  int16_t steering_deg;
  int16_t drive_percent;
  int16_t lift_percent;
} TruckOutputs;

void TruckControl_SetNeutral(TruckOutputs *outputs);
void TruckControl_MapRawCommand(const TruckCommand *command,
                                TruckOutputs *outputs);
bool TruckControl_IsActiveChannel(uint8_t channel);
bool TruckControl_IsTimedOut(uint32_t now_ms, uint32_t last_valid_ms);

#endif /* TRUCK_CONTROL_H */
