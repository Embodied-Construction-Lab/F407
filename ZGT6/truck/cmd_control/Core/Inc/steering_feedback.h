#ifndef STEERING_FEEDBACK_H
#define STEERING_FEEDBACK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Copied from sibling truck_control; NOT calibrated for this vehicle. */
#define STEERING_FEEDBACK_REFERENCE_MV 3300U
#define STEERING_FEEDBACK_ADC_MAX 4095U
#define STEERING_FEEDBACK_CENTER_MV 1630
#define STEERING_FEEDBACK_TENTHS_PER_MV 1
#define STEERING_FEEDBACK_LIMIT_TENTHS 300
#define STEERING_FEEDBACK_TIMEOUT_MS 100U
#define STEERING_FEEDBACK_JSON_CAPACITY 240U

typedef struct
{
  uint16_t raw;
  uint16_t voltage_mv;
  int16_t angle_tenths;
  uint32_t sample_ms;
  bool has_sample;
  bool valid;
} SteeringFeedback;

void SteeringFeedback_Init(SteeringFeedback *feedback);
void SteeringFeedback_Invalidate(SteeringFeedback *feedback);
bool SteeringFeedback_Update(SteeringFeedback *feedback, uint16_t raw,
                             uint32_t now_ms);
bool SteeringFeedback_IsFresh(const SteeringFeedback *feedback,
                              uint32_t now_ms);
bool SteeringFeedback_FormatJson(const SteeringFeedback *feedback,
                                 uint32_t now_ms, char *buffer, size_t capacity);

#endif
