#include "steering_feedback.h"

#include <stdio.h>
#include <string.h>

void SteeringFeedback_Init(SteeringFeedback *feedback)
{
  if (feedback != NULL)
  {
    memset(feedback, 0, sizeof(*feedback));
  }
}

void SteeringFeedback_Invalidate(SteeringFeedback *feedback)
{
  if (feedback != NULL)
  {
    feedback->valid = false;
  }
}

bool SteeringFeedback_Update(SteeringFeedback *feedback, uint16_t raw,
                             uint32_t now_ms)
{
  int32_t angle;
  if (feedback == NULL)
  {
    return false;
  }
  feedback->raw = raw;
  feedback->sample_ms = now_ms;
  feedback->has_sample = true;
  feedback->valid = false;
  if (raw > STEERING_FEEDBACK_ADC_MAX)
  {
    feedback->voltage_mv = 0U;
    return false;
  }
  feedback->voltage_mv = (uint16_t)(
      ((uint32_t)raw * STEERING_FEEDBACK_REFERENCE_MV +
       STEERING_FEEDBACK_ADC_MAX / 2U) / STEERING_FEEDBACK_ADC_MAX);
  /* Rail samples are not evidence of a connected, healthy potentiometer. */
  if ((raw == 0U) || (raw == STEERING_FEEDBACK_ADC_MAX))
  {
    return false;
  }
  angle = ((int32_t)feedback->voltage_mv - STEERING_FEEDBACK_CENTER_MV) *
          STEERING_FEEDBACK_TENTHS_PER_MV;
  if (angle > STEERING_FEEDBACK_LIMIT_TENTHS)
  {
    angle = STEERING_FEEDBACK_LIMIT_TENTHS;
  }
  else if (angle < -STEERING_FEEDBACK_LIMIT_TENTHS)
  {
    angle = -STEERING_FEEDBACK_LIMIT_TENTHS;
  }
  feedback->angle_tenths = (int16_t)angle;
  feedback->valid = true;
  return true;
}

bool SteeringFeedback_IsFresh(const SteeringFeedback *feedback,
                              uint32_t now_ms)
{
  return (feedback != NULL) && feedback->has_sample && feedback->valid &&
      ((uint32_t)(now_ms - feedback->sample_ms) <= STEERING_FEEDBACK_TIMEOUT_MS);
}

bool SteeringFeedback_FormatJson(const SteeringFeedback *feedback,
                                 uint32_t now_ms, char *buffer, size_t capacity)
{
  char angle[12] = "null";
  char age[12] = "null";
  int length;
  const bool valid = SteeringFeedback_IsFresh(feedback, now_ms);
  if ((feedback == NULL) || (buffer == NULL) || (capacity == 0U))
  {
    return false;
  }
  if (valid)
  {
    const int value = feedback->angle_tenths;
    const unsigned int magnitude = (unsigned int)((value < 0) ? -value : value);
    (void)snprintf(angle, sizeof(angle), "%s%u.%u", (value < 0) ? "-" : "",
                   magnitude / 10U, magnitude % 10U);
  }
  if (feedback->has_sample)
  {
    (void)snprintf(age, sizeof(age), "%lu",
                   (unsigned long)(uint32_t)(now_ms - feedback->sample_ms));
  }
  length = snprintf(buffer, capacity,
      "\"feedback_valid\":%u,\"sensor_steering_deg\":%s,"
      "\"current_steering_deg\":%s,\"steering_adc_raw\":%u,"
      "\"steering_voltage_mv\":%u,\"steering_sample_age_ms\":%s",
      valid ? 1U : 0U, angle, angle, (unsigned int)feedback->raw,
      (unsigned int)feedback->voltage_mv, age);
  return (length >= 0) && ((size_t)length < capacity);
}
