/**********************************************************************************************************************
 * \file ImuEdge.h
 * \brief The DRDY edge handoff -- imuDrdyIsr (CPU1, T15) to NavTask_step
 *        (CPU1), in the LMU shared block. See docs/REFACTORING_PLAN.md §3.6.
 *
 * A SEPARATE object from g_imuDrdyLastTicks/g_imuDrdyCount (ImuInt.h), which
 * stay exactly where they are and keep their plain-global form on purpose:
 * ImuInt.h already documents them as "do not fold into a struct -- a struct
 * member does not get its own symbol in the map file", because they are read
 * BY NAME over XCP (tools/xcp_read.py, tools/imu_int_stats.py). Folding them
 * into this object would silently break that tooling. g_imuEdge instead is
 * PURELY the producer/consumer contract NavTask_step needs to detect a new
 * sample and take its exact dt -- nothing outside ImuInt.c/NavTask.c ever
 * reads it, so it can be whatever shape the contract needs.
 *
 * Placed in the LMU per SharedRam.h even though, after T15's ISR retarget,
 * producer (imuDrdyIsr) and consumer (NavTask_step) both run on CPU1: this is
 * an ISR-vs-task race, not a cross-core cache-coherency one, but it is the
 * SAME class of "must never see a torn combination of two fields written
 * together" problem the LMU discipline already solves, and reusing one
 * audited protocol (SharedRam.h rules 2/3, the same generation-counter shape
 * as NavState_get()/ahrs_refreshMag()) beats inventing a second one for an
 * ISR/task boundary instead of a core/core one.
 *
 * SharedRam.h rule 2: both fields are 32-bit, the object is 8 bytes and
 * 8-byte aligned (ImuEdgePlace.c), and it has exactly ONE writer (imuDrdyIsr).
 * Rule 3: the writer stores `ticks`, Ifx__dsync()s, then increments `seq` --
 * `seq` doubles as the object's own "is this a NEW edge" generation counter,
 * so there is no separate `gen` field. */
#ifndef IMUEDGE_H
#define IMUEDGE_H

#include "Ifx_Types.h"

typedef struct
{
    uint32 ticks;   /**< STM0 tick of the most recent DRDY edge, written LAST
                      *   by the reader's reckoning but FIRST by the writer's
                      *   -- see the file comment: payload, then seq          */
    uint32 seq;     /**< incremented once per edge, after `ticks` is stored;
                      *   monotonic (wraps after ~4.29e9 edges -- irrelevant
                      *   at 1014 edges/s, ~48 days)                          */
} ImuEdge_t;

extern volatile ImuEdge_t g_imuEdge;

/** Snapshot {seq, ticks} with the torn-read retry (read, re-read seq, one
 *  retry) -- same shape as NavState_get()/ahrs_refreshMag(). No barrier is
 *  needed on this side: a plain read is already ordered against the writer's
 *  Ifx__dsync()'d store on this in-order core (SharedRam.h). Never writes
 *  g_imuEdge -- the caller keeps its own "last consumed" state entirely on
 *  its side, exactly like NavState_get()'s caller does.
 *
 *  Pure and iLLD-free, so host-testable (test/test_imuint.c): this is what
 *  makes it safe to define here rather than beside the ISR in ImuInt.c. */
static inline void ImuEdge_snapshot(uint32 *seqOut, uint32 *ticksOut)
{
    uint8   attempt;
    boolean ok = FALSE;

    /* Bounded: at most one retry, same reasoning as NavState_get() -- a
     * second mismatch would mean the ISR fired twice inside this handful of
     * instructions, vanishingly unlikely at any DRDY rate this task ever
     * sees. On that limit the LAST attempt's values are used anyway: still a
     * self-consistent (seq, ticks) pair, just possibly one edge behind the
     * very latest at this instant -- exactly NavState_get()'s "keep the
     * previous snapshot" trade-off, applied here instead of returning
     * boolean because the caller compares `seq` itself to decide staleness. */
    for (attempt = 0u; (attempt < 2u) && (ok == FALSE); attempt++)
    {
        uint32 seqBefore = g_imuEdge.seq;

        *seqOut   = seqBefore;
        *ticksOut = g_imuEdge.ticks;

        if (g_imuEdge.seq == seqBefore)
        {
            ok = TRUE;
        }
    }
}

#endif /* IMUEDGE_H */
