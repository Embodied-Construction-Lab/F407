#include "pwm_timing.h"

uint8_t PwmTiming_CalculatePrescale(uint32_t oscillator_hz,
                                    uint32_t frequency_hz)
{
  uint32_t divisor;
  uint32_t rounded_ratio;

  if ((oscillator_hz == 0U) || (frequency_hz == 0U))
  {
    return 0U;
  }

  divisor = PWM_RESOLUTION_COUNTS * frequency_hz;
  rounded_ratio = (oscillator_hz + divisor / 2U) / divisor;
  if (rounded_ratio <= 1U)
  {
    return 0U;
  }
  if (rounded_ratio > 256U)
  {
    return 255U;
  }
  return (uint8_t)(rounded_ratio - 1U);
}

uint16_t PwmTiming_PulseUsToCount(uint32_t oscillator_hz,
                                  uint8_t prescale,
                                  uint16_t pulse_us)
{
  uint64_t numerator;
  uint64_t denominator;
  uint64_t count;

  if (oscillator_hz == 0U)
  {
    return 0U;
  }

  numerator = (uint64_t)pulse_us * oscillator_hz;
  denominator = 1000000ULL * ((uint32_t)prescale + 1U);
  count = (numerator + denominator / 2U) / denominator;
  if (count >= PWM_RESOLUTION_COUNTS)
  {
    count = PWM_RESOLUTION_COUNTS - 1U;
  }
  return (uint16_t)count;
}

uint16_t PwmTiming_DefaultPulseUsToCount(uint16_t pulse_us)
{
  const uint8_t prescale = PwmTiming_CalculatePrescale(
      PWM_OSCILLATOR_HZ, PWM_TARGET_FREQUENCY_HZ);
  return PwmTiming_PulseUsToCount(PWM_OSCILLATOR_HZ, prescale, pulse_us);
}

uint16_t PwmTiming_DefaultCountToPulseUs(uint16_t count)
{
  const uint8_t prescale = PwmTiming_CalculatePrescale(
      PWM_OSCILLATOR_HZ, PWM_TARGET_FREQUENCY_HZ);
  uint64_t numerator;
  uint64_t pulse_us;

  if (count >= PWM_RESOLUTION_COUNTS)
  {
    count = PWM_RESOLUTION_COUNTS - 1U;
  }

  numerator = (uint64_t)count * 1000000ULL *
      ((uint32_t)prescale + 1U);
  pulse_us = (numerator + PWM_OSCILLATOR_HZ / 2U) /
      PWM_OSCILLATOR_HZ;
  if (pulse_us > UINT16_MAX)
  {
    pulse_us = UINT16_MAX;
  }
  return (uint16_t)pulse_us;
}
