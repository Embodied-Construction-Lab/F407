#ifndef IMU_OLED_FORMAT_H
#define IMU_OLED_FORMAT_H

#include "imu_parser.h"

#include <stddef.h>

void imu_oled_format_row(char *output, size_t capacity,
                         const char *label, float value);

#endif
