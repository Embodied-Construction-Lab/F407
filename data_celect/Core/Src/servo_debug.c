#include "servo_debug.h"

#include <stdio.h>

int ServoDebug_FormatAction(char *buffer,
                            size_t capacity,
                            const StickData *stick,
                            const JoystickServoTargets *targets,
                            bool i2c_ok)
{
  int length;

  if ((buffer == NULL) || (capacity == 0U) ||
      (stick == NULL) || (targets == NULL))
  {
    return -1;
  }

  length = snprintf(
      buffer, capacity,
      "[servo] X1=%.2f X2=%.2f Y1=%.2f Y2=%.2f Z1=%.2f Z2=%.2f\r\n"
      "[servo] bucket=%.1fdeg big_arm=%.1fdeg small_arm=%.1fdeg "
      "pump=%.1f left=%.1f right=%.1f I2C=%s\r\n",
      (double)stick->x1, (double)stick->x2,
      (double)stick->y1, (double)stick->y2,
      (double)stick->z1, (double)stick->z2,
      (double)targets->bucket_deg,
      (double)targets->big_arm_deg,
      (double)targets->small_arm_deg,
      (double)targets->pump_percent,
      (double)targets->left_drive_percent,
      (double)targets->right_drive_percent,
      i2c_ok ? "OK" : "ERROR");
  if ((length < 0) || ((size_t)length >= capacity))
  {
    return -1;
  }
  return length;
}
