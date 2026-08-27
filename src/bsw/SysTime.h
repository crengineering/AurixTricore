/* SysTime.h
 *
 * Thin BSW time service: exposes the free-running STM0 counter behind an
 * iLLD-free interface so ASW code can take timestamps without touching
 * iLLD headers (see src/asw/README.md, rule 2).
 *
 * ⚠️ Deliberate, load-bearing exception to "each core uses its own STM"
 * (CLAUDE.md rule 2): this is pinned to MODULE_STM0 for EVERY caller on
 * EVERY core, on purpose, since T12 (docs/REFACTORING_PLAN.md) made
 * NavTask_step (CPU1) a caller too. `dt` has to come from ONE shared time
 * base -- if CPU1 measured elapsed time against its own STM1 instead, its
 * `dt` and CPU0's would drift apart from two independently free-running
 * counters, which is a subtler and worse bug than the read itself. The read
 * is safe (cross-core STM reads are read-only and do not need coherency
 * handling, docs/REFACTORING_PLAN.md Risk 3); do not "fix" it by giving
 * CPU1 its own SysTime built on MODULE_STM1.
 */

#ifndef SYSTIME_H
#define SYSTIME_H

#include <stdint.h>

/* Lower 32 bit of the STM0 counter (100 MHz after PLL init, 1 tick =
 * 10 ns); wraps after ~42.9 s. Deltas computed with unsigned
 * subtraction stay correct across a single wrap. */
uint32_t SysTime_getTicks(void);
float    SysTime_getTimeElapsedS(uint32_t *lastTicks);

#endif /* SYSTIME_H */
