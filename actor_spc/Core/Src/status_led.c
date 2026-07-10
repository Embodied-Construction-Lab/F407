#include "status_led.h"

#include <stddef.h>

void StatusLedTimer_Init(StatusLedTimer *timer, uint32_t now_ms)
{
  if (timer != NULL)
  {
    timer->last_toggle_ms = now_ms;
  }
}

bool StatusLedTimer_Poll(StatusLedTimer *timer, uint32_t now_ms)
{
  if ((timer == NULL) ||
      ((uint32_t)(now_ms - timer->last_toggle_ms) <
       STATUS_LED_TOGGLE_INTERVAL_MS))
  {
    return false;
  }

  timer->last_toggle_ms = now_ms;
  return true;
}
