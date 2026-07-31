#include "truck_control.h"

#include "servo_control.h"

#include <stddef.h>

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

void TruckControl_SetDrivePercent(TruckOutputs *outputs,
                                  int16_t drive_percent)
{
  if (outputs == NULL)
  {
    return;
  }
  if (drive_percent > 100)
  {
    drive_percent = 100;
  }
  else if (drive_percent < -100)
  {
    drive_percent = -100;
  }

  outputs->drive_percent = drive_percent;
  outputs->pwm_count[TRUCK_CHANNEL_DRIVE] =
      ServoControl_SpeedToPulse((float)drive_percent);
}

void TruckControl_SetLiftPercent(TruckOutputs *outputs,
                                 int16_t lift_percent)
{
  if (outputs == NULL)
  {
    return;
  }
  if (lift_percent > TRUCK_LIFT_UP_MAX_PERCENT)
  {
    lift_percent = TRUCK_LIFT_UP_MAX_PERCENT;
  }
  else if (lift_percent < -TRUCK_LIFT_DOWN_MAX_PERCENT)
  {
    lift_percent = -TRUCK_LIFT_DOWN_MAX_PERCENT;
  }

  outputs->lift_percent = lift_percent;
  outputs->pwm_count[TRUCK_CHANNEL_LIFT] =
      ServoControl_SpeedToPulse((float)lift_percent);
}

void TruckControl_MapCommand(const TruckCommand *command,
                             TruckOutputs *outputs)
{
  float steering_angle;
  float steering_servo_angle;
  float throttle;
  float brake;
  float drive;

  if ((command == NULL) || (outputs == NULL))
  {
    return;
  }

  steering_angle = TruckControl_Clamp(command->steering,
                                      TRUCK_STEERING_MIN_DEG,
                                      TRUCK_STEERING_MAX_DEG);
  steering_servo_angle = TRUCK_STEERING_SERVO_CENTER_DEG +
      steering_angle * TRUCK_STEERING_SERVO_DEG_PER_VEHICLE_DEG;
  throttle = TruckControl_Clamp(command->throttle, 0.0f, 1.0f) *
      100.0f;
  brake = TruckControl_Clamp(command->brake, 0.0f, 1.0f) * 100.0f;
  drive = ((brake > 0.0f) ? -brake : throttle) *
      TRUCK_DRIVE_MAX_PERCENT / 100.0f;

  outputs->pwm_count[TRUCK_CHANNEL_STEERING] =
      ServoControl_AngleToPulse(steering_servo_angle);
  outputs->pwm_count[TRUCK_CHANNEL_UNUSED] = 0U;
  outputs->pwm_count[TRUCK_CHANNEL_LIFT] =
      ServoControl_SpeedToPulse(0.0f);
  outputs->steering_deg =
      (int16_t)((steering_angle >= 0.0f) ?
                (steering_angle + 0.5f) : (steering_angle - 0.5f));
  outputs->throttle_percent = (int16_t)(throttle + 0.5f);
  outputs->brake_percent = (int16_t)(brake + 0.5f);
  TruckControl_SetDrivePercent(
      outputs, (int16_t)((drive >= 0.0f) ?
                         (drive + 0.5f) : (drive - 0.5f)));
  outputs->lift_percent = 0;
}

void TruckEsc_Init(TruckEscController *controller)
{
  if (controller != NULL)
  {
    controller->state = TRUCK_ESC_NEUTRAL;
    controller->state_started_ms = 0U;
    controller->output_percent = 0;
  }
}

void TruckEsc_Reset(TruckEscController *controller)
{
  TruckEsc_Init(controller);
}

int16_t TruckEsc_Update(TruckEscController *controller,
                        int16_t requested_percent,
                        uint32_t now_ms)
{
  if (controller == NULL)
  {
    return 0;
  }
  if (requested_percent > 100)
  {
    requested_percent = 100;
  }
  else if (requested_percent < -100)
  {
    requested_percent = -100;
  }

  switch (controller->state)
  {
    case TRUCK_ESC_NEUTRAL:
      if (requested_percent > 0)
      {
        controller->state = TRUCK_ESC_FORWARD;
        controller->output_percent = requested_percent;
      }
      else if (requested_percent < 0)
      {
        controller->state = TRUCK_ESC_BRAKE_FOR_REVERSE;
        controller->state_started_ms = now_ms;
        controller->output_percent = requested_percent;
      }
      else
      {
        controller->output_percent = 0;
      }
      break;

    case TRUCK_ESC_FORWARD:
      if (requested_percent < 0)
      {
        controller->state = TRUCK_ESC_BRAKE_FOR_REVERSE;
        controller->state_started_ms = now_ms;
        controller->output_percent = requested_percent;
      }
      else if (requested_percent == 0)
      {
        controller->state = TRUCK_ESC_NEUTRAL;
        controller->output_percent = 0;
      }
      else
      {
        controller->output_percent = requested_percent;
      }
      break;

    case TRUCK_ESC_BRAKE_FOR_REVERSE:
      if (requested_percent >= 0)
      {
        controller->state = (requested_percent > 0) ?
            TRUCK_ESC_FORWARD : TRUCK_ESC_NEUTRAL;
        controller->output_percent = requested_percent;
      }
      else if ((uint32_t)(now_ms - controller->state_started_ms) >=
               TRUCK_ESC_BRAKE_TIME_MS)
      {
        controller->state = TRUCK_ESC_REVERSE_NEUTRAL;
        controller->state_started_ms = now_ms;
        controller->output_percent = 0;
      }
      else
      {
        controller->output_percent = requested_percent;
      }
      break;

    case TRUCK_ESC_REVERSE_NEUTRAL:
      if (requested_percent >= 0)
      {
        controller->state = (requested_percent > 0) ?
            TRUCK_ESC_FORWARD : TRUCK_ESC_NEUTRAL;
        controller->output_percent = requested_percent;
      }
      else if ((uint32_t)(now_ms - controller->state_started_ms) >=
               TRUCK_ESC_REVERSE_NEUTRAL_TIME_MS)
      {
        controller->state = TRUCK_ESC_REVERSE;
        controller->output_percent = requested_percent;
      }
      else
      {
        controller->output_percent = 0;
      }
      break;

    case TRUCK_ESC_REVERSE:
      if (requested_percent < 0)
      {
        controller->output_percent = requested_percent;
      }
      else
      {
        controller->state = (requested_percent > 0) ?
            TRUCK_ESC_FORWARD : TRUCK_ESC_NEUTRAL;
        controller->output_percent = requested_percent;
      }
      break;

    default:
      TruckEsc_Reset(controller);
      break;
  }

  return controller->output_percent;
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
