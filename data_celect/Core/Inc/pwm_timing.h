#ifndef PWM_TIMING_H
#define PWM_TIMING_H

#include <stdint.h>

#define PWM_OSCILLATOR_HZ 27443200U
#define PWM_TARGET_FREQUENCY_HZ 50U
#define PWM_RESOLUTION_COUNTS 4096U

uint8_t PwmTiming_CalculatePrescale(uint32_t oscillator_hz,
                                    uint32_t frequency_hz);
uint16_t PwmTiming_PulseUsToCount(uint32_t oscillator_hz,
                                  uint8_t prescale,
                                  uint16_t pulse_us);
uint16_t PwmTiming_DefaultPulseUsToCount(uint16_t pulse_us);

#endif /* PWM_TIMING_H */
