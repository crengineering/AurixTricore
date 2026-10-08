/**********************************************************************************************************************
 * \file NavTask.h
 * \brief THE flight chain: IMU read -> AHRS -> fusion -> NavState.
 *
 * T11 (docs/REFACTORING_PLAN.md): moved out of Cpu0_Main.c's Task_Imu
 * verbatim, publishing via NavState_publish() instead of calling
 * measurementsSetFusion() directly -- Housekeeping_100ms is now the one that
 * reads NavState and publishes to XCP.
 *
 * T12: the actual migration. NavTask_init()/NavTask_step() now run on CPU1
 * (core1_main, after its sync barrier) -- the crossing NavState_publish()
 * exercises is a real cross-core one from here on, not the same-core dry run
 * T11 was. NavTask_step() also stopped calling measurementsSetImu()/
 * PeriphDiag_report() directly (both write CPU0-owned state -- g_xcpData is
 * CPU0 DSPR, PeriphDiag's s_periph[] is a plain non-volatile static); the raw
 * sample and an accumulated liveness now ride in the NavState payload and
 * Housekeeping_100ms makes those calls instead (NavState.h, Housekeeping.c).
 *********************************************************************************************************************/
#ifndef NAVTASK_H
#define NAVTASK_H

#include "Ifx_Types.h"

/** NavTask_step's own registered scheduler period, in microseconds -- the
 *  ONE source both Cpu1_Main.c's `Scheduler_addTask(&g_sched, NavTask_step,
 *  SCHED_US(NAVTASK_DISPATCH_PERIOD_US))` and NavTask.c's
 *  NAVTASK_TIMEDOUT_FAULT_DT_S derive from. flight-reviewer follow-up
 *  (SYS1-001): NAVTASK_TIMEDOUT_FAULT_DT_S must track this exactly, or a
 *  retune silently mis-scales Ahrs.c's fault-hold clock -- a comment saying
 *  so is not enough, so this is the single #define both sites read, not two
 *  numbers that happen to agree today. */
#define NAVTASK_DISPATCH_PERIOD_US   (500u)

/** FusionCal_init, Ahrs_init, Fusion_init, NavState_init -- in that order,
 *  once at boot, before the scheduler starts. */
void NavTask_init(void);

/** The flight chain, one tick: IMU read, AHRS update, fusion update,
 *  NavState_publish. Registered at SCHED_US(NAVTASK_DISPATCH_PERIOD_US)
 *  (T15) -- a 2 kHz poll well above the sensor's measured ~1014.2 Hz DRDY
 *  rate; the body returns almost immediately unless a new edge (or a
 *  timeout, see NavTask.c) is pending, so the actual work rate tracks the
 *  sensor, not the poll period. */
void NavTask_step(void);

/** Whether this tick's inputs are trustworthy enough to feed the fusion
 *  filter: the dt window (bounded, NaN-safe -- an out-of-range or NaN dtS
 *  returns FALSE by construction, it is never accepted by omission the way
 *  "reject if dt < lo || dt > hi" would silently pass a NaN through),
 *  imuPresent, and the AHRS having left calibration (AHRS_RUNNING). Split out
 *  of NavTask_step for the host test -- this is where the NaN/validity logic
 *  lives (docs/REFACTORING_PLAN.md §4). */
boolean NavTask_inputValid(float32 dtS, boolean imuPresent, uint8 ahrsState);

/** What NavTask_classifyDt() found for a measured interval. */
typedef enum
{
    NAVTASK_DT_OK    = 0,  /**< inside [NAVTASK_DT_MIN_S, NAVTASK_DT_MAX_S] */
    NAVTASK_DT_SHORT = 1,  /**< below the window -- a duplicate-edge candidate */
    NAVTASK_DT_LONG  = 2,  /**< above the window -- a real gap                 */
    NAVTASK_DT_NONE  = 3   /**< no interval to classify: dtS <= 0 or NaN, or
                             *   the deliberate 0.0f NavTask_step passes on
                             *   its own no-new-edge timeout                   */
} NavTask_DtClass;

/** Pure classification of a measured interval against the same bounds
 *  navTask_dtValid() enforces, but naming WHICH side of the window (or
 *  neither) rather than a plain yes/no -- SYS1-001: the SHORT case is the
 *  duplicate-DRDY-edge candidate NavTask_step (task 3) treats as "not a
 *  fault", not something to feed to Ahrs_update as invalid input.
 *
 *  NaN-safe by construction (project rule, same idiom as navTask_dtValid()):
 *  written as a chain of POSITIVE range tests, so a NaN dtS -- which
 *  compares false against every relational operator -- falls through every
 *  test and lands on NAVTASK_DT_NONE, the same bucket a genuine "no interval
 *  measured" (dtS == 0.0f) lands on. */
NavTask_DtClass NavTask_classifyDt(float32 dtS);

/** SYS1-001 strand B task 15 (SWE1-FW-009): reject a gyro sample whose
 *  per-axis rate changed from the previous ACCEPTED sample by more than
 *  200 000 deg/s^2 scaled by \p dtS (197 deg/s over one measured 985 us
 *  tick) -- a bound no legitimate airframe motion reaches (10 000 deg/s^2,
 *  20x headroom) but the observed corrupt-sample defect (a near-full-scale
 *  word appearing between two ordinary ticks, ~2 000 000 deg/s^2) sails
 *  through by 10x. Pure (no bus access, no state -- the caller owns which
 *  sample counted as "previous accepted"); split out for the host test, the
 *  same reason NavTask_classifyDt() is.
 *  \param gyro  this tick's gyro, sensor frame, AS DELIVERED (pre-mount) [deg/s]
 *  \param prev  the previous ACCEPTED sample's gyro, same frame
 *  \param dtS   elapsed time since that previous accepted sample [s] -- a
 *               non-finite, zero or negative dtS makes the bound
 *               non-positive, so any nonzero change is rejected (correct:
 *               no elapsed time cannot excuse one)
 *  \return FALSE if any axis is outside the bound (also FALSE for a NaN
 *          gyro/prev/dtS -- NaN compares false against every relational
 *          operator here, so the positive "inside the band" test rejects it
 *          by construction, never by omission). */
boolean NavTask_gyroSlewOk(const float32 gyro[3], const float32 prev[3], float32 dtS);

/* --- debug instrumentation (SYS1-001 task 0) -----------------------------
 * Raw map symbols read by tools/xcp_read.py, same precedent as g_imuDrdy*
 * (ImuInt.h) -- no A2L entry, no Xcp_Data field, no GUI change. Declared
 * here (not just defined in NavTask.c) for MISRA 8.4. */
/** Ticks that reached the ahrsInputOk check with invalid input: a
 *  LONG-classified edge, a present == FALSE tick, or the no-new-edge
 *  timeout. flight-reviewer FAIL correction: this does NOT include
 *  g_dbgNavDtShort any more -- SYS1-001 task 3's `duplicateEdge` handling
 *  consumes a SHORT (duplicate DRDY edge) tick and returns before this
 *  check is ever reached, which is the whole point: a duplicate edge is not
 *  a fault. Read g_dbgNavDtShort (how many duplicate edges occurred) and
 *  g_dbgAhrsRealigns (Ahrs.h, how many times the estimator actually
 *  re-initialised) together instead -- neither should move the other any
 *  more. */
extern volatile uint32 g_dbgNavInvalidTicks;
/** Genuine new edges whose interval was BELOW NAVTASK_DT_MIN_S -- the
 *  duplicate-DRDY-edge candidate named in the SYS1-001 dispatch note. */
extern volatile uint32 g_dbgNavDtShort;
/** Genuine new edges whose interval was ABOVE NAVTASK_DT_MAX_S. */
extern volatile uint32 g_dbgNavDtLong;
/** Smallest deltaTicks seen among g_dbgNavDtShort events, ticks (STM0,
 *  10 ns/tick); 0xFFFFFFFF (sentinel) until the first one. Root-causing
 *  aid: near-zero means a true back-to-back double-fire, near
 *  NAVTASK_DT_MIN_S*1e8 means a merely-early edge. */
extern volatile uint32 g_dbgNavDtShortMinTicks;
/** Icm42688_read() returned FALSE (present == FALSE) on a tick this task
 *  actually reached -- a genuine sensor communication failure, as opposed
 *  to a short/long-dt classification. */
extern volatile uint32 g_dbgImuReadFail;

/** SYS1-001 strand B task 17 (B6.4, SWE1-FW-008 clause g): running max
 *  duration of NavTask_step() itself, STM0 ticks, since boot -- brackets
 *  the WHOLE dispatch (present or absent, new-sample or timed-out), the
 *  same way g_imuSpiBurstMaxTicks (Icm42688.c) brackets exactly the SPI
 *  transfer. Names the number clause (g)'s bound
 *  ("no single NavTask_step dispatch exceeds NAVTASK_DISPATCH_PERIOD_US")
 *  is checked against -- nothing measured this before task 17. */
extern volatile uint32 g_dbgNavStepMaxTicks;

#endif /* NAVTASK_H */
