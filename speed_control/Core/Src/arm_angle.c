#include "arm_angle.h"

#define ARM_ANGLE_PI (3.14159265358979323846f)
#define ARM_ANGLE_RAD_TO_DEG (57.29577951308232f)
#define ARM_ANGLE_DEG_180_HUNDREDTHS (18000)

static float ArmAngle_Clamp(float value, float minimum, float maximum)
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

static float ArmAngle_Sqrt(float value)
{
  float estimate;

  if (value <= 0.0f)
  {
    return 0.0f;
  }

  estimate = (value > 1.0f) ? value : 1.0f;
  for (uint8_t iteration = 0; iteration < 8U; iteration++)
  {
    estimate = 0.5f * (estimate + (value / estimate));
  }

  return estimate;
}

static float ArmAngle_Acos(float value)
{
  float x = ArmAngle_Clamp(value, -1.0f, 1.0f);
  uint8_t negative = (x < 0.0f) ? 1U : 0U;
  float result;

  if (negative != 0U)
  {
    x = -x;
  }

  result = -0.0187293f;
  result = (result * x) + 0.0742610f;
  result = (result * x) - 0.2121144f;
  result = (result * x) + 1.5707288f;
  result *= ArmAngle_Sqrt(1.0f - x);

  if (negative != 0U)
  {
    result = ARM_ANGLE_PI - result;
  }

  return result;
}

static int32_t ArmAngle_DegToHundredths(float angle_deg)
{
  float scaled = angle_deg * 100.0f;

  if (scaled >= 0.0f)
  {
    return (int32_t)(scaled + 0.5f);
  }

  return (int32_t)(scaled - 0.5f);
}

int32_t ArmAngle_FromExtensionHundredthsMm(int32_t extension_hundredths_mm)
{
  float extension_mm = (float)extension_hundredths_mm / 100.0f;
  float length_mm = ARM_ANGLE_BASE_LENGTH_MM + extension_mm;
  float numerator = (ARM_ANGLE_OA_MM * ARM_ANGLE_OA_MM) +
                    (ARM_ANGLE_OB_MM * ARM_ANGLE_OB_MM) -
                    (length_mm * length_mm);
  float denominator = 2.0f * ARM_ANGLE_OA_MM * ARM_ANGLE_OB_MM;
  float cos_theta = ArmAngle_Clamp(numerator / denominator, -1.0f, 1.0f);
  float angle_deg = ArmAngle_Acos(cos_theta) * ARM_ANGLE_RAD_TO_DEG;

  return ArmAngle_DegToHundredths(angle_deg);
}

int32_t ArmAngle_Channel2HundredthsDeg(int32_t channel2_length_hundredths_mm)
{
  int32_t reference_angle =
      ArmAngle_FromExtensionHundredthsMm(channel2_length_hundredths_mm);

  return ARM_ANGLE_DEG_180_HUNDREDTHS - reference_angle;
}
