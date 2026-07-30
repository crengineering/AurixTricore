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
 * Cross-core access: g_coreStats is one plain global written only by its owning
 * core and read by CPU0 for the XCP block. Each core touches its own array slot,
 * so there is no shared-write race and no lock is needed. TC3xx DSPRs are
 * globally addressable and other cores' accesses bypass the data cache, so CPU0
 * sees fresh values. Same pattern as the existing cpuSyncEvent global.
 *
 * aliveCounter increments every window regardless of load, so a core that has
 * hung (see the I2C hang of 2026-07-30) shows a frozen counter rather than
 * simply reporting 0% and looking idle.
 *********************************************************************************************************************/
#ifndef CORESTATS_H
#define CORESTATS_H

#include "Ifx_Types.h"

#define CORESTATS_NUM_CORES   (6u)

/** Rolling statistics for one core. Written by that core only. */
typedef struct
{
    uint32 execUs;        /**< busy time in the last window [us]            */
    uint32 execMaxUs;     /**< longest single task dispatch since boot [us] */
    uint16 loadPmil;      /**< busy fraction of the window [per mille]      */
    uint16 aliveCounter;  /**< +1 per window; frozen => core is stuck       */
} CoreStats_t;

extern volatile CoreStats_t g_coreStats[CORESTATS_NUM_CORES];

/** Reset one core's slot. Call from that core before its scheduler loop. */
void CoreStats_init(uint8 coreId);

#endif /* CORESTATS_H */
