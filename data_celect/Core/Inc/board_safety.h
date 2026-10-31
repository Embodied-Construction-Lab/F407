#ifndef BOARD_SAFETY_H
#define BOARD_SAFETY_H

#include <stdint.h>

/* Enable only after the physical input and its active polarity are verified. */
#ifndef BOARD_ESTOP_INPUT_ENABLED
#define BOARD_ESTOP_INPUT_ENABLED 0
#endif

/* Explicit architecture for a confirmed, independent power-cut button.
 * It has no GPIO feedback. After using it, the operator must restart the MCU,
 * Orin and excavator before a new locally authorized control session.
 * This flag never claims to measure button state or prevent hydraulic drift. */
#ifndef BOARD_EXTERNAL_POWER_CUTOFF_ENABLED
#define BOARD_EXTERNAL_POWER_CUTOFF_ENABLED 0
#endif

#if ((BOARD_ESTOP_INPUT_ENABLED != 0) && (BOARD_ESTOP_INPUT_ENABLED != 1)) || \
    ((BOARD_EXTERNAL_POWER_CUTOFF_ENABLED != 0) && \
     (BOARD_EXTERNAL_POWER_CUTOFF_ENABLED != 1))
#error "Board safety mode flags must be 0 or 1"
#endif

#if BOARD_ESTOP_INPUT_ENABLED && BOARD_EXTERNAL_POWER_CUTOFF_ENABLED
#error "Select GPIO estop or external power cutoff, not both"
#endif

#if BOARD_ESTOP_INPUT_ENABLED
#if !defined(BOARD_ESTOP_GPIO_PORT) || !defined(BOARD_ESTOP_GPIO_PIN) || \
    !defined(BOARD_ESTOP_ACTIVE_STATE)
#error "Configure the verified estop GPIO port, pin and active state explicitly"
#endif
#endif

typedef struct
{
  uint8_t available;
  /* available describes GPIO measurement only. Unconfigured is fail-closed.
   * External cutoff reports available=0, estop=0 with explicit mode flags;
   * consumers must interpret the physical button state as unknown. */
  uint8_t estop;
} BoardSafetyInput;

#endif
