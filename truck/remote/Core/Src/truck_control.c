#include "truck_control.h"

#include "servo_control.h"

#include <stddef.h>

#define PEDAL_DEAD_ZONE_PERCENT 2.0f

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

static float TruckControl_PedalToPercent(float raw_axis)
{
  const float minimum =
      (TRUCK_PEDAL_RELEASED_RAW < TRUCK_PEDAL_PRESSED_RAW) ?
      TRUCK_PEDAL_RELEASED_RAW : TRUCK_PEDAL_PRESSED_RAW;
  const float maximum =
      (TRUCK_PEDAL_RELEASED_RAW > TRUCK_PEDAL_PRESSED_RAW) ?
      TRUCK_PEDAL_RELEASED_RAW : TRUCK_PEDAL_PRESSED_RAW;
  const float denominator =
      TRUCK_PEDAL_RELEASED_RAW - TRUCK_PEDAL_PRESSED_RAW;
  float percent;

  if ((denominator > -0.0001f) && (denominator < 0.0001f))
  {
    return 0.0f;
  }

  raw_axis = TruckControl_Clamp(raw_axis, minimum, maximum);
  percent = (TRUCK_PEDAL_RELEASED_RAW - raw_axis) * 100.0f / denominator;
  percent = TruckControl_Clamp(percent, 0.0f, 100.0f);

  if (percent <= PEDAL_DEAD_ZONE_PERCENT)
  {
    percent = 0.0f;
  }
  return percent;
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
  outputs->throttle_percent = 0;
  outputs->brake_percent = 0;
  outputs->drive_percent = 0;
  outputs->lift_percent = 0;
}

void TruckControl_MapRawCommand(const TruckCommand *command,
                                TruckOutputs *outputs)
{
  float steering_angle;
  float throttle;
  float brake;
  float drive;
  float lift_speed = 0.0f;

  if ((command == NULL) || (outputs == NULL))
  {
    return;
  }

  steering_angle =
      (TruckControl_ClampSteeringAxis(
           command->steering * TRUCK_STEERING_DIRECTION) + 1.0f) * 90.0f;
  throttle = TruckControl_PedalToPercent(command->throttle);
  brake = TruckControl_PedalToPercent(command->brake);
  drive = ((brake > 0.0f) ? -brake : throttle) *
      TRUCK_DRIVE_MAX_PERCENT / 100.0f;

  if ((command->up != 0U) && (command->down == 0U))
  {
    lift_speed = 90.0f;
  }
  else if ((command->down != 0U) && (command->up == 0U))
  {
    lift_speed = -50.0f;
  }

  outputs->pwm_count[TRUCK_CHANNEL_STEERING] =
      ServoControl_AngleToPulse(steering_angle);
  outputs->pwm_count[TRUCK_CHANNEL_DRIVE] =
      ServoControl_SpeedToPulse(drive);
  outputs->pwm_count[TRUCK_CHANNEL_UNUSED] = 0U;
  outputs->pwm_count[TRUCK_CHANNEL_LIFT] =
      ServoControl_SpeedToPulse(lift_speed);

  outputs->steering_deg = (int16_t)(steering_angle + 0.5f);
  outputs->throttle_percent = (int16_t)(throttle + 0.5f);
  outputs->brake_percent = (int16_t)(brake + 0.5f);
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
