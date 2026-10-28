#include "steering_feedback.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint16_t raw_for_mv(uint32_t mv)
{
  return (uint16_t)((mv * 4095U + 1650U) / 3300U);
}

static void emit(const SteeringFeedback *feedback, uint32_t now_ms)
{
  char json[STEERING_FEEDBACK_JSON_CAPACITY];
  assert(SteeringFeedback_FormatJson(feedback, now_ms, json, sizeof(json)));
  printf("{\"type\":\"truck\",%s}\n", json);
}

int main(void)
{
  SteeringFeedback feedback;
  char json[STEERING_FEEDBACK_JSON_CAPACITY];
  SteeringFeedback_Init(&feedback);
  assert(!SteeringFeedback_IsFresh(&feedback, 0U));
  emit(&feedback, 0U);
  assert(SteeringFeedback_Update(&feedback, raw_for_mv(1630U), 10U));
  assert(feedback.voltage_mv == 1630U && feedback.angle_tenths == 0);
  assert(SteeringFeedback_IsFresh(&feedback, 110U));
  assert(!SteeringFeedback_IsFresh(&feedback, 111U));
  assert(SteeringFeedback_Update(&feedback, raw_for_mv(1330U), 20U));
  assert(feedback.angle_tenths == -300);
  assert(SteeringFeedback_Update(&feedback, raw_for_mv(1930U), 30U));
  assert(feedback.angle_tenths == 300);
  assert(SteeringFeedback_Update(&feedback, raw_for_mv(1730U), 40U));
  assert(feedback.angle_tenths == 100);
  emit(&feedback, 50U);
  emit(&feedback, 141U);
  assert(SteeringFeedback_Update(&feedback, raw_for_mv(1629U), 50U));
  assert(feedback.angle_tenths == -1);
  assert(SteeringFeedback_FormatJson(&feedback, 50U, json, sizeof(json)));
  assert(strstr(json, "\"sensor_steering_deg\":-0.1") != NULL);
  assert(SteeringFeedback_Update(&feedback, raw_for_mv(2100U), 60U));
  assert(feedback.angle_tenths == 300);
  assert(SteeringFeedback_Update(&feedback, raw_for_mv(1100U), 70U));
  assert(feedback.angle_tenths == -300);
  assert(!SteeringFeedback_Update(&feedback, 0U, 80U));
  assert(!SteeringFeedback_IsFresh(&feedback, 80U));
  assert(!SteeringFeedback_Update(&feedback, 4095U, 90U));
  assert(!SteeringFeedback_Update(&feedback, 4096U, 100U));
  assert(SteeringFeedback_Update(&feedback, raw_for_mv(1630U), 110U));
  SteeringFeedback_Invalidate(&feedback);
  emit(&feedback, 111U);
  assert(!SteeringFeedback_IsFresh(&feedback, 111U));
  assert(SteeringFeedback_Update(&feedback, raw_for_mv(1630U), UINT32_MAX - 10U));
  assert(SteeringFeedback_IsFresh(&feedback, 20U));
  assert(!SteeringFeedback_IsFresh(&feedback, 100U));
  assert(!SteeringFeedback_FormatJson(&feedback, 20U, json, 2U));
  assert(!SteeringFeedback_Update(NULL, 2000U, 0U));
  assert(!SteeringFeedback_IsFresh(NULL, 0U));
  return 0;
}
