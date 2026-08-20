#include "servo_control.h"

#include "pwm_timing.h"

static float clamp_float(float value, float minimum, float maximum)
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

uint16_t ServoControl_AngleToPulse(float angle_deg)
{
  const float clamped = clamp_float(angle_deg, 0.0f, 180.0f);
  float pulse_us;

  if (clamped <= 90.0f)
  {
    pulse_us = (float)SERVO_PULSE_MIN_US +
        ((float)(SERVO_PULSE_MID_US - SERVO_PULSE_MIN_US) *
         clamped / 90.0f);
  }
  else
  {
    pulse_us = (float)SERVO_PULSE_MID_US +
        ((float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MID_US) *
         (clamped - 90.0f) / 90.0f);
  }

  return PwmTiming_DefaultPulseUsToCount((uint16_t)(pulse_us + 0.5f));
}

uint16_t ServoControl_SpeedToPulse(float speed_percent)
{
  const float clamped = clamp_float(speed_percent, -100.0f, 100.0f);
  float pulse_us;

  if (clamped >= 0.0f)
  {
    pulse_us = (float)ESC_PULSE_MID_US +
        ((float)(ESC_PULSE_MAX_US - ESC_PULSE_MID_US) *
         clamped / 100.0f);
  }
  else
  {
    pulse_us = (float)ESC_PULSE_MID_US +
        ((float)(ESC_PULSE_MID_US - ESC_PULSE_MIN_US) *
         clamped / 100.0f);
  }
  return PwmTiming_DefaultPulseUsToCount((uint16_t)(pulse_us + 0.5f));
}
