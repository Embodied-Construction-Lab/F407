#include "collection_timing.h"

#include <assert.h>

int main(void)
{
  assert(COLLECTION_CONTROL_PERIOD_MS == 50U);
  assert(COLLECTION_TELEMETRY_PERIOD_MS == 50U);
  assert(COLLECTION_SENSOR_PERIOD_MS == 100U);
  return 0;
}
