/* SysTime.h
 *
 * Thin BSW time service: exposes the free-running STM0 counter behind an
 * iLLD-free interface so ASW code can take timestamps without touching
 * iLLD headers (see src/asw/README.md, rule 2).
 *
 * CPU0 only — the value is read from STM0 (each core owns its own STM).
 */

#ifndef SYSTIME_H
#define SYSTIME_H

#include <stdint.h>
#include "Ifx_Types.h"

/* Lower 32 bit of the STM0 counter (100 MHz after PLL init, 1 tick =
 * 10 ns); wraps after ~42.9 s. Deltas computed with unsigned
 * subtraction stay correct across a single wrap. */
uint32_t SysTime_getTicks(void);
float32  SysTime_getTimeElapsedS(uint32 *lastTicks);

#endif /* SYSTIME_H */
