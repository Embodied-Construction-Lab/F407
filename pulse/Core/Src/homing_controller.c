#include "homing_controller.h"

#include <stddef.h>

static void clear_sample(JoystickProfileSample *sample)
{
  if (sample == NULL)
  {
    return;
  }

  sample->x1 = 0.0f;
  sample->x2 = 0.0f;
  sample->y1 = 0.0f;
  sample->y2 = 0.0f;
}

static int32_t absolute_i32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static float absolute_f(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float linear_axis_command(int32_t position_hundredths_mm,
                                 int32_t target_hundredths_mm,
                                 uint8_t positive_increases_length)
{
  int32_t error;
  float increasing_command;

  error = target_hundredths_mm - position_hundredths_mm;
  if (absolute_i32(error) <= HOMING_CONTROLLER_LINEAR_TOLERANCE_HUNDREDTHS_MM)
  {
    return 0.0f;
  }

  increasing_command = (error > 0) ? HOMING_CONTROLLER_SPEED :
                                    -HOMING_CONTROLLER_SPEED;
  return (positive_increases_length != 0U) ? increasing_command :
                                            -increasing_command;
}

static float yaw_axis_command(float yaw_deg)
{
  float error;
  float increasing_command;

  error = SafetyLimits_NormalizeYawDeg(HOMING_CONTROLLER_YAW_TARGET_DEG -
                                       SafetyLimits_NormalizeYawDeg(yaw_deg));
  if (absolute_f(error) <= HOMING_CONTROLLER_YAW_TOLERANCE_DEG)
  {
    return 0.0f;
  }

  increasing_command = (error > 0.0f) ? HOMING_CONTROLLER_SPEED :
                                       -HOMING_CONTROLLER_SPEED;
  return (SAFETY_LIMIT_SWING_POSITIVE_INCREASES_YAW != 0U) ?
             increasing_command :
             -increasing_command;
}

static uint8_t sample_is_zero(const JoystickProfileSample *sample)
{
  return ((sample->x1 == 0.0f) && (sample->x2 == 0.0f) &&
          (sample->y1 == 0.0f) && (sample->y2 == 0.0f)) ? 1U : 0U;
}

void HomingController_Init(HomingController *homing, uint32_t start_ms)
{
  if (homing == NULL)
  {
    return;
  }

  homing->start_ms = start_ms;
  homing->stable_since_ms = start_ms;
  homing->stable_timer_active = 0U;
  homing->status = HOMING_CONTROLLER_WAITING_FEEDBACK;
  clear_sample(&homing->sample);
}

HomingControllerStatus HomingController_Update(
    HomingController *homing,
    uint32_t now_ms,
    const SafetyLimitsFeedback *feedback,
    uint8_t feedback_valid)
{
  if (homing == NULL)
  {
    return HOMING_CONTROLLER_WAITING_FEEDBACK;
  }

  if (homing->status == HOMING_CONTROLLER_COMPLETE)
  {
    clear_sample(&homing->sample);
    return homing->status;
  }

  if (feedback_valid == 0U || feedback == NULL)
  {
    homing->status = HOMING_CONTROLLER_WAITING_FEEDBACK;
    homing->start_ms = now_ms;
    homing->stable_timer_active = 0U;
    clear_sample(&homing->sample);
    return homing->status;
  }

  if ((uint32_t)(now_ms - homing->start_ms) > HOMING_CONTROLLER_TIMEOUT_MS)
  {
    homing->status = HOMING_CONTROLLER_TIMEOUT;
    clear_sample(&homing->sample);
    return homing->status;
  }

  homing->sample.x1 = yaw_axis_command(feedback->yaw_deg);
  homing->sample.x2 = linear_axis_command(
      feedback->bucket_length_hundredths_mm,
      HOMING_CONTROLLER_BUCKET_TARGET_HUNDREDTHS_MM,
      SAFETY_LIMIT_BUCKET_POSITIVE_INCREASES_LENGTH);
  homing->sample.y1 = linear_axis_command(
      feedback->stick_length_hundredths_mm,
      HOMING_CONTROLLER_STICK_TARGET_HUNDREDTHS_MM,
      SAFETY_LIMIT_STICK_POSITIVE_INCREASES_LENGTH);
  homing->sample.y2 = linear_axis_command(
      feedback->boom_length_hundredths_mm,
      HOMING_CONTROLLER_BOOM_TARGET_HUNDREDTHS_MM,
      SAFETY_LIMIT_BOOM_POSITIVE_INCREASES_LENGTH);

  if (sample_is_zero(&homing->sample) == 0U)
  {
    homing->stable_timer_active = 0U;
    homing->status = HOMING_CONTROLLER_RUNNING;
    return homing->status;
  }

  if (homing->stable_timer_active == 0U)
  {
    homing->stable_since_ms = now_ms;
    homing->stable_timer_active = 1U;
    homing->status = HOMING_CONTROLLER_RUNNING;
    return homing->status;
  }

  if ((uint32_t)(now_ms - homing->stable_since_ms) >=
      HOMING_CONTROLLER_STABLE_TIME_MS)
  {
    homing->status = HOMING_CONTROLLER_COMPLETE;
    return homing->status;
  }

  homing->status = HOMING_CONTROLLER_RUNNING;
  return homing->status;
}
