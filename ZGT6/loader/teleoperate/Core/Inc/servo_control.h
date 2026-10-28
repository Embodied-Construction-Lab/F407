#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>

#define SERVO_PULSE_MIN_US 500U
#define SERVO_PULSE_MID_US 1500U
#define SERVO_PULSE_MAX_US 2500U
#define ESC_PULSE_MIN_US 500U
#define ESC_PULSE_MID_US 1500U
#define ESC_PULSE_MAX_US 2500U

uint16_t ServoControl_AngleToPulse(float angle_deg);
uint16_t ServoControl_SpeedToPulse(float speed_percent);

#endif /* SERVO_CONTROL_H */
