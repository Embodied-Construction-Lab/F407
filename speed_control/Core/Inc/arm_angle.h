#ifndef ARM_ANGLE_H
#define ARM_ANGLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ARM_ANGLE_OA_MM (420.0f)
#define ARM_ANGLE_OB_MM (120.0f)
#define ARM_ANGLE_BASE_LENGTH_MM (250.0f)

int32_t ArmAngle_FromExtensionHundredthsMm(int32_t extension_hundredths_mm);
int32_t ArmAngle_Channel2HundredthsDeg(int32_t channel2_length_hundredths_mm);

#ifdef __cplusplus
}
#endif

#endif
