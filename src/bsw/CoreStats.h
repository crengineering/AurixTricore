/**********************************************************************************************************************
 * \file CoreStats.h
 * \brief Per-core execution-time and load measurement for all six TriCores.
 *
 * Every core runs the same Scheduler_run() loop, so the measurement lives there:
 * each task dispatch is bracketed with STM reads and the busy time accumulated.
 * Once per window the accumulator is converted into
 *   - execUs   : microseconds of work done in the window (what actually ran)
 *   - loadPmil : that time as a fraction of the window, in per mille (0..1000)
 * and the raw peak of a single dispatch is kept as execMaxUs, which is what
 * matters for scheduling headroom — a task that averages 200 us but spikes to
 * 15 ms will blow a 20 ms period even though its average looks harmless.
 *
 * Storage: g_coreStats lives in the LMU shared block (SharedRam.h/.c),
 * non-cached alias, one 16-byte slot per core -- NOT a plain per-file global
 * any more. This is deliberately no longer "the sanctioned cross-core
 * pattern"; that claim was wrong (it described the reader only, and was never
 * Infineon's actual rule) and must not be cited for a new crossing. The real
 * rule -- LMU + non-cached alias, single writer per object, all fields
 * 32-bit, ordered publish -- is documented in SharedRam.h; see
 * docs/REFACTORING_PLAN.md §2.4. This struct is kept here, alongside
 * CoreStats_init(), because it is still "per-core execution stats", not
 * because its storage is special.
 *
 * Each core still touches only its own array slot, so there is still no
 * shared-write race and no lock is needed -- that discipline is unchanged.
 * A torn read here costs one wrong load/alive number for a single 100 ms
 * window, never a flight, so it is tolerated rather than guarded against.
 *
 * aliveCounter increments every window regardless of load, so a core that has
 * hung (see the I2C hang of 2026-07-30) shows a frozen counter rather than
 * simply reporting 0% and looking idle.
 *********************************************************************************************************************/
#ifndef CORESTATS_H
#define CORESTATS_H

#include "Ifx_Types.h"

#define CORESTATS_NUM_CORES   (6u)

/** Rolling statistics for one core. Written by that core only.
 *  All fields 32-bit: this struct lives in the LMU shared block, which is
 *  64 bits wide with no sub-word write (SharedRam.h). loadPmil and
 *  aliveCounter were uint16 before the T9 move; a halfword store there would
 *  have been a read-modify-write of a doubleword another core might be
 *  writing at the same time. Each slot is exactly 16 bytes = two full
 *  doublewords, so two cores' slots never share a physical line. */
typedef struct
{
    uint32 execUs;        /**< busy time in the last window [us]            */
    uint32 execMaxUs;     /**< longest single task dispatch since boot [us] */
    uint32 loadPmil;      /**< busy fraction of the window [per mille]      */
    uint32 aliveCounter;  /**< +1 per window; frozen => core is stuck       */
} CoreStats_t;

/* Defined (with #pragma section) in SharedRam.c, not here -- see
 * SharedRam.h. Off __at(SHARED_LMU_ADDR) as of T3 (docs/MEMORY_PLACEMENT.md);
 * address is locator-assigned, not a literal -- read it from the `.map`. */
extern volatile CoreStats_t g_coreStats[CORESTATS_NUM_CORES];

/** Reset one core's slot. Call from that core before its scheduler loop. */
void CoreStats_init(uint8 coreId);

#endif /* CORESTATS_H */
