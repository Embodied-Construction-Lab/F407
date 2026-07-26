#include "truck_control.h"

#include "servo_control.h"

#include <stddef.h>

static float TruckControl_Absolute(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float TruckControl_Clamp(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static float TruckControl_ClampSteeringAxis(float value)
{
  value = TruckControl_Clamp(value, -1.0f, 1.0f);

  return (TruckControl_Absolute(value) <= TRUCK_STEERING_DEAD_ZONE) ?
      0.0f : value;
}

static float TruckControl_ClampTcaAxis(float value)
{
  value = TruckControl_Clamp(value, -1.0f, 1.0f);
  return (TruckControl_Absolute(value) <= TRUCK_TCA_AXIS_DEAD_ZONE) ?
      0.0f : value;
}

void TruckControl_SetNeutral(TruckOutputs *outputs)
{
  if (outputs == NULL)
  {
    return;
  }

  outputs->pwm_count[TRUCK_CHANNEL_STEERING] =
      ServoControl_AngleToPulse(90.0f);
  outputs->pwm_count[TRUCK_CHANNEL_DRIVE] =
      ServoControl_SpeedToPulse(0.0f);
  outputs->pwm_count[TRUCK_CHANNEL_UNUSED] = 0U;
  outputs->pwm_count[TRUCK_CHANNEL_LIFT] =
      ServoControl_SpeedToPulse(0.0f);
  outputs->steering_deg = 90;
  outputs->drive_percent = 0;
  outputs->lift_percent = 0;
}

void TruckControl_MapRawCommand(const TruckCommand *command,
                                TruckOutputs *outputs)
{
  float steering_angle;
  float drive_axis;
  float lift_axis;
  float drive;
  float lift_speed = 0.0f;

  if ((command == NULL) || (outputs == NULL))
  {
    return;
  }

  steering_angle =
      (TruckControl_ClampSteeringAxis(
           command->steering * TRUCK_STEERING_DIRECTION) + 1.0f) * 90.0f;
  drive_axis = TruckControl_ClampTcaAxis(command->drive_axis);
  lift_axis = TruckControl_ClampTcaAxis(command->lift_axis);

  /* TCA axis 0: -1..0 is forward, 0..1 is reverse. */
  drive = -drive_axis * TRUCK_DRIVE_MAX_PERCENT;

  /* TCA axis 1: -1..0 is lift up, 0..1 is lift down. */
  if (lift_axis < 0.0f)
  {
    lift_speed = -lift_axis * TRUCK_LIFT_UP_MAX_PERCENT;
  }
  else if (lift_axis > 0.0f)
  {
    lift_speed = -lift_axis * TRUCK_LIFT_DOWN_MAX_PERCENT;
  }

  outputs->pwm_count[TRUCK_CHANNEL_STEERING] =
      ServoControl_AngleToPulse(steering_angle);
  outputs->pwm_count[TRUCK_CHANNEL_DRIVE] =
      ServoControl_SpeedToPulse(drive);
  outputs->pwm_count[TRUCK_CHANNEL_UNUSED] = 0U;
  outputs->pwm_count[TRUCK_CHANNEL_LIFT] =
      ServoControl_SpeedToPulse(lift_speed);

  outputs->steering_deg = (int16_t)(steering_angle + 0.5f);
  outputs->drive_percent = (int16_t)((drive >= 0.0f) ?
                                      (drive + 0.5f) : (drive - 0.5f));
  outputs->lift_percent = (int16_t)((lift_speed >= 0.0f) ?
                                     (lift_speed + 0.5f) :
                                     (lift_speed - 0.5f));
}

bool TruckControl_IsActiveChannel(uint8_t channel)
{
  return (channel < TRUCK_OUTPUT_CHANNELS) &&
         (channel != TRUCK_CHANNEL_UNUSED);
}

bool TruckControl_IsTimedOut(uint32_t now_ms, uint32_t last_valid_ms)
{
  return (uint32_t)(now_ms - last_valid_ms) > TRUCK_CONTROL_TIMEOUT_MS;
}
