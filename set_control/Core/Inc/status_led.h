#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>
#include <stdint.h>

#define STATUS_LED_TOGGLE_INTERVAL_MS 500U

typedef struct
{
  uint32_t last_toggle_ms;
} StatusLedTimer;

void StatusLedTimer_Init(StatusLedTimer *timer, uint32_t now_ms);
bool StatusLedTimer_Poll(StatusLedTimer *timer, uint32_t now_ms);

#endif /* STATUS_LED_H */
