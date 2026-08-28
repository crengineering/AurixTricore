/**********************************************************************************************************************
 * \file NavState.h
 * \brief CPU1 -> CPU0 navigation-state snapshot -- the cross-core contract
 *        that NavTask (T11/T12) publishes and Housekeeping (T11) reads.
 *
 * Lives in the LMU shared block (SharedRam.h), non-cached alias, in the
 * linker-managed `shared_lmu` group (docs/MEMORY_PLACEMENT.md T3) -- address
 * is locator-assigned, not a literal; read it from the `.map`. Single writer
 * (the nav core), any number of readers. See docs/REFACTORING_PLAN.md §2.4
 * for the full argument; this header states the contract for this one
 * object.
 *
 * Every field is 32-bit: LMU SRAM has no sub-word write (SharedRam.h rule 2).
 * `imuPresent` was a `uint8 imuPresent; uint8 reserved[3];` pair in the
 * original single-core code (Cpu0_Main.c's Task_Imu) -- that packs into one
 * word here instead, because a bare uint8 store would be a read-modify-write
 * of a doubleword nobody else needs to touch, but must not be trained to
 * think it is safe elsewhere.
 *
 * NOT yet wired to anything as of T10: NavTask/T11 is the first writer,
 * Housekeeping/T11 is the first reader. This file and its host test exist to
 * prove the publish/get protocol before flight code depends on it.
 *********************************************************************************************************************/
#ifndef NAVSTATE_H
#define NAVSTATE_H

#include "Ifx_Types.h"
#include "Ahrs.h"
#include "fusion.h"

typedef struct
{
    uint32       gen;          /**< monotonic, +1 per publish; no even/odd needed
                                 *   (the writer never holds an odd/half-written
                                 *   state visible -- payload is written
                                 *   completely, THEN this increments). Also
                                 *   doubles as the IMU sample sequence number
                                 *   the T12 blocker fix needs: NavTask_step
                                 *   publishes exactly once per IMU sample, at
                                 *   any rate, so a second field would be
                                 *   redundant. */
    Ahrs_Values  ahrs;
    FusionValues fusion;
    float32      dtS;
    uint32       imuPresent;   /**< was uint8 + reserved[3]; see file header */
    /* T12 blocker fix (docs/REFACTORING_PLAN.md 3.7): the raw IMU sample and
     * an ACCUMULATED liveness sum, so NavTask_step (CPU1) never again has to
     * call measurementsSetImu()/PeriphDiag_report() itself -- both write
     * CPU0-owned state (g_xcpData is CPU0 DSPR; PeriphDiag's s_periph[] is a
     * plain non-volatile static) and would become two-writer objects the
     * moment NavTask_step runs on CPU1. Housekeeping_100ms (CPU0) makes
     * those calls instead, from this snapshot -- same pattern as
     * measurementsSetFusion already uses. +32 bytes, zero A2L cost: none of
     * this reaches Xcp_Data, which has only 8 bytes free (docs/CODEMAP.md 3).
     */
    float32      imuAcc[3];    /**< latest sample, sensor frame [g]           */
    float32      imuGyro[3];   /**< latest sample, sensor frame [deg/s]       */
    float32      imuTempC;     /**< latest sample [degC]                      */
    float32      imuLiveness;  /**< RUNNING total of Icm42688_plausible()'s
                                 *   per-sample liveness sum, incremented every
                                 *   NavTask_step tick and NEVER reset by the
                                 *   writer. Housekeeping_100ms diffs two
                                 *   reads of this instead of reading the
                                 *   latest instantaneous value, which is what
                                 *   keeps the stuck-sensor detector sensitive
                                 *   to every tick instead of only the one
                                 *   tick that happens to land on its 100 ms
                                 *   boundary: for a genuinely stuck sensor the
                                 *   per-tick contribution is bit-identical
                                 *   every tick, so consecutive 100 ms windows
                                 *   sum an identical run and the DIFFERENCE
                                 *   is bit-identical too -- the same
                                 *   frozen-value trip PeriphDiag_report()
                                 *   already does, just fed a windowed delta
                                 *   instead of one sample in twenty (or,
                                 *   after T15, one in twenty at 1 kHz). */
} NavState_t;

/** Defined (with #pragma section) in SharedRam.c, not here --
 *  docs/MEMORY_PLACEMENT.md T3. Formerly its own file (NavStatePlace.c,
 *  __at(0xB00F0060u)), required while __at() poisoned cppcheck's symbol
 *  table for the rest of any TU it appeared in; #pragma section has no such
 *  restriction. */
extern volatile NavState_t g_navState;

/** Zero the block. CPU1 only, once at boot. Kept even though `g_navState`
 *  left __at() in T3 (docs/MEMORY_PLACEMENT.md): whether the `shared_lmu`
 *  group's `.bss.shared_lmu.*` sections are covered by the startup's
 *  auto-zero table the way ordinary .bss groups are is NOT verified either
 *  way -- explicit init is correct and cheap regardless, so it stays.
 *  __at() storage (still true for the XCP blocks, T4) is definitely NOT
 *  guaranteed pre-zeroed the way ordinary .bss is (Measurements.c's
 *  g_xcpData needs the same explicit init, for that reason). */
void NavState_init(void);

/** Publish one snapshot. CPU1 (the nav core) ONLY.
 *  Sequence: store payload (ahrs, fusion, dtS, imuPresent, imuAcc, imuGyro,
 *  imuTempC, imuLiveness) -> Ifx__dsync() -> gen++. The barrier is what stops
 *  the TriCore store buffer from posting the gen store ahead of the payload
 *  stores even though the alias is non-cached; volatile alone only orders the
 *  compiler, not the store buffer. Never call this from any core but the nav
 *  core.
 *  \param liveness  the CALLER's running total (NavTask_step accumulates,
 *                   never resets) -- see the imuLiveness field comment. */
void NavState_publish(const Ahrs_Values *ahrs, const FusionValues *fusion,
                       float32 dtS, boolean imuPresent,
                       const float32 imuAcc[3], const float32 imuGyro[3],
                       float32 imuTempC, float32 liveness);

/** Read one consistent snapshot into *out. Any core.
 *  Sequence: read gen -> copy payload -> re-read gen; if they match, the
 *  copy is consistent and this returns TRUE. One retry on a mismatch; a
 *  second mismatch returns FALSE and *out is left as whatever the last
 *  (possibly torn) attempt wrote -- the CALLER must not use *out on FALSE,
 *  and should keep its own last-good snapshot instead. No barrier needed on
 *  this side: TriCore is in-order and the alias is non-cached, so the
 *  compiler-ordered volatile reads execute exactly as written. Never writes
 *  shared state -- this is what makes it safe from any core. */
boolean NavState_get(NavState_t *out);

#endif /* NAVSTATE_H */
