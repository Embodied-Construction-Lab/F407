#ifndef SERVO_DEBUG_H
#define SERVO_DEBUG_H

#include "joystick_servo_map.h"
#include "stick_receiver.h"

#include <stdbool.h>
#include <stddef.h>

int ServoDebug_FormatAction(char *buffer,
                            size_t capacity,
                            const StickData *stick,
                            const JoystickServoTargets *targets,
                            bool i2c_ok);

#endif /* SERVO_DEBUG_H */
