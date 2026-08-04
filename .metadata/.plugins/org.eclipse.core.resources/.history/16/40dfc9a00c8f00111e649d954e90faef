#ifndef TRUCK_CONTROL_H
#define TRUCK_CONTROL_H

#include "truck_receiver.h"

#include <stdbool.h>
#include <stdint.h>

#define TRUCK_OUTPUT_CHANNELS 4U
#define TRUCK_CONTROL_TIMEOUT_MS 300U
#define TRUCK_STEERING_MIN_DEG (-30.0f)
#define TRUCK_STEERING_MAX_DEG 30.0f
#define TRUCK_STEERING_SERVO_CENTER_DEG 90.0f
#define TRUCK_STEERING_SERVO_DEG_PER_VEHICLE_DEG 1.0f
#define TRUCK_DRIVE_MAX_PERCENT 90.0f
#define TRUCK_LIFT_UP_MAX_PERCENT 50
#define TRUCK_LIFT_DOWN_MAX_PERCENT 50
#define TRUCK_ESC_BRAKE_TIME_MS 300U
#define TRUCK_ESC_REVERSE_NEUTRAL_TIME_MS 200U

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
  int16_t throttle_percent;
  int16_t brake_percent;
  int16_t drive_percent;
  int16_t lift_percent;
} TruckOutputs;

typedef enum
{
  TRUCK_ESC_NEUTRAL = 0,
  TRUCK_ESC_FORWARD,
  TRUCK_ESC_BRAKE_FOR_REVERSE,
  TRUCK_ESC_REVERSE_NEUTRAL,
  TRUCK_ESC_REVERSE
} TruckEscState;

typedef struct
{
  TruckEscState state;
  uint32_t state_started_ms;
  int16_t output_percent;
} TruckEscController;

void TruckControl_SetNeutral(TruckOutputs *outputs);
void TruckControl_MapCommand(const TruckCommand *command,
                             TruckOutputs *outputs);
void TruckControl_SetDrivePercent(TruckOutputs *outputs,
                                  int16_t drive_percent);
void TruckControl_SetLiftPercent(TruckOutputs *outputs,
                                 int16_t lift_percent);
void TruckEsc_Init(TruckEscController *controller);
void TruckEsc_Reset(TruckEscController *controller);
int16_t TruckEsc_Update(TruckEscController *controller,
                        int16_t requested_percent,
                        uint32_t now_ms);
bool TruckControl_IsActiveChannel(uint8_t channel);
bool TruckControl_IsTimedOut(uint32_t now_ms, uint32_t last_valid_ms);

#endif /* TRUCK_CONTROL_H */
