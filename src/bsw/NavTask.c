/**********************************************************************************************************************
 * \file NavTask.c
 * \brief THE flight chain -- see NavTask.h.
 *********************************************************************************************************************/
#include "NavTask.h"
#include "Icm42688.h"
#include "Ahrs.h"
#include "fusion.h"
#include "FusionCal.h"
#include "NavState.h"
#include "SysTime.h"
#include "ImuInt.h"
#include "ImuEdge.h"

/* Below NAVTASK_DT_MIN_S the tick fired twice inside one STM period (nothing
 * to integrate), above NAVTASK_DT_MAX_S the previous tick was so late that
 * integrating the gap would put a real offset into attitude/position rather
 * than a measurement.
 *
 * T14 (docs/REFACTORING_PLAN.md §3.8, Risk 10): NAVTASK_DT_MIN_S was 0.001f,
 * sized against the 50 Hz task period -- and a SILENT KILLER at the measured
 * 985 us IMU interval, because 985 us < 1 ms rejects EVERY tick, permanently
 * FALSE-ing ahrsInputOk/fusionInputOk with no error anywhere: the estimator
 * just stops. 0.0002f (200 us) still catches a genuine double dispatch and
 * clears the measured 985 us period by 5x, at any rate this task ever runs
 * at. */
#define NAVTASK_DT_MIN_S   (0.0002f)
#define NAVTASK_DT_MAX_S   (0.2f)

/* T15 (docs/REFACTORING_PLAN.md §3.6): STM0 @ ~100 MHz, 1 tick = 10 ns -- the
 * same constant SysTime.c itself uses. There is no shared header for it
 * (Spi.c's SPI_STM_TICKS_PER_MS is the same per-file convention); needed here
 * to turn the ISR's raw edge-tick DELTA into the dt seconds Ahrs_update/
 * Fusion_update take, now that dt comes from ImuEdge.h instead of
 * SysTime_getTimeElapsedS()'s task-dispatch measurement. */
#define NAVTASK_TICKS_TO_S   (1.0e-8f)

/* How stale the last-SEEN edge may be before this task stops waiting for a
 * new one and falls back to probing the bus directly. Two failure modes this
 * covers, both needing the same fallback:
 *   - cold boot, before the very first DRDY edge has ever arrived (edgeTicks
 *     starts at 0, so the very first poll already looks "stale" against a
 *     boot elapsed time this small or larger);
 *   - the sensor going fully silent (power lost, INT1 wire broken) rather
 *     than merely late -- a pure newSample gate would never call
 *     Icm42688_read() again once its own edges stop, and that call is what
 *     drives its hot-plug reconnect probe (Icm42688.c ICM42688_RECOVERY_PERIOD).
 * 20 ms matches T13's own worst-case data freshness (the 50 Hz task period
 * this replaces), so the fallback is never a worse guarantee than before --
 * comfortably inside AHRS_DT_MAX_S/FUSION_DT_MAX (0.2 s) with margin to
 * spare. */
#define NAVTASK_NO_EDGE_TIMEOUT_S   (0.02f)

/* NaN-safe by construction: written as "is dtS INSIDE the window", not "is
 * dtS outside the window". NaN compares false against every relational
 * operator, so the ORIGINAL Cpu0_Main.c form -- `(dt < lo) || (dt > hi)` to
 * mean "reject" -- let a NaN dt through as "not rejected", i.e. accepted.
 * This form rejects it: both comparisons below are false for NaN, so `valid`
 * never becomes TRUE. Same bug class as the GNSS input check fixed in
 * fusion.c (MISRA 10.3 commit) and the plausibility bands in Bmp581/Mmc5983/
 * Icm42688 -- NaN is a real value here, not an edge case. */
static boolean navTask_dtValid(float32 dtS)
{
    boolean valid = FALSE;

    if ((dtS >= NAVTASK_DT_MIN_S) && (dtS <= NAVTASK_DT_MAX_S))
    {
        valid = TRUE;
    }

    return valid;
}

boolean NavTask_inputValid(float32 dtS, boolean imuPresent, uint8 ahrsState)
{
    boolean valid = FALSE;

    if ((navTask_dtValid(dtS) != FALSE)
        && (imuPresent != FALSE)
        && (ahrsState == (uint8)AHRS_RUNNING))
    {
        valid = TRUE;
    }

    return valid;
}

/* Running total of Icm42688_plausible()'s per-sample liveness, published
 * verbatim in NavState_t.imuLiveness (NEVER reset here after boot) -- see
 * that field's comment. Housekeeping_100ms does the resetting-by-diffing;
 * this side just keeps adding. CPU1-only, same as every other NavTask.c
 * static. */
static float32 s_imuLivenessAccum;

void NavTask_init(void)
{
    NavState_init();        /* the publish target, before anything publishes */
    FusionCal_init();        /* estimator tuning defaults, BEFORE the filters */
    Ahrs_init();              /* start the gyro-bias calibration; hold still  */
    Fusion_init();            /* zero every channel state and covariance      */
    s_imuLivenessAccum = 0.0f;
}

/* T15 (docs/REFACTORING_PLAN.md §3.6): registered at SCHED_US(500), a 2 kHz
 * poll well above the sensor's measured ~1014.2 Hz -- deliberately faster
 * than the data it waits for, so the edge sequence counter (not the poll
 * period) is the real clock. See the body below for the gate. */
void NavTask_step(void)
{
    /* CPU1-side record of the last edge this task actually consumed --
     * never written to g_imuEdge, matching NavState_get()'s "the reader never
     * writes shared state" rule. */
    static uint32 s_lastEdgeSeq   = 0u;
    static uint32 s_lastEdgeTicks = 0u;

    uint32  edgeSeq;
    uint32  edgeTicks;
    boolean newSample;
    boolean timedOut;

    ImuEdge_snapshot(&edgeSeq, &edgeTicks);

    /* I5, docs/IMU_INTERRUPT.md 5.5: how stale is the last edge the ISR saw,
     * computed on EVERY poll now (not only on a consumed sample) -- T15 turns
     * this into a continuous liveness signal (docs/REFACTORING_PLAN.md §3.8):
     * a healthy sensor keeps it under ~1 ms; a sustained rise means either
     * this task is falling behind the sensor, or the sensor has stopped
     * producing edges entirely. g_imuDrdyLastTicks (ImuInt.c) starts at 0 and
     * only updates once a real edge has been seen; a giant value before the
     * wire/ISR are alive is expected, not a fault. */
    g_imuDrdyStaleTicks = (uint32)SysTime_getTicks() - edgeTicks;

    /* Written as if/else into a boolean local, not a direct comparison
     * assignment: cppcheck's MISRA 10.3 does not recognise `boolean` as an
     * essentially-Boolean type, so it flags a comparison result stored
     * straight into one as a different essential type category. Same idiom
     * as navTask_dtValid()/NavTask_inputValid() above and ahrsInputOk below. */
    newSample = FALSE;
    if (edgeSeq != s_lastEdgeSeq)
    {
        newSample = TRUE;
    }

    timedOut = FALSE;
    if (((float32)g_imuDrdyStaleTicks * NAVTASK_TICKS_TO_S) > NAVTASK_NO_EDGE_TIMEOUT_S)
    {
        timedOut = TRUE;
    }

    if ((newSample == FALSE) && (timedOut == FALSE))
    {
        /* Nothing to do: no new edge, and not stale enough to suspect the
         * sensor has stopped producing them entirely -- the common case at a
         * 2 kHz poll against a ~1014 Hz sensor. Costs one LMU read, one STM
         * read and two comparisons -- no SPI, no AHRS, no fusion. */
    }
    else
    {
        Icm42688_Sample sample = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 0.0f };
        boolean         present;
        boolean         ahrsInputOk;
        boolean         fusionInputOk;
        FusionValues    fusion;
        Ahrs_Values     ahrs;
        float32         elapsedTime;

        if (newSample != FALSE)
        {
            /* seq is monotonic (ImuEdge.h): a delta of exactly 1 is the
             * common case, and anything larger means edges arrived that this
             * task never individually consumed -- counted, not silently
             * dropped (docs/REFACTORING_PLAN.md §3.3's overrun row). */
            uint32 missed     = edgeSeq - s_lastEdgeSeq;
            /* MISRA 10.8: cast a plain object, not the composite subtraction
             * above it -- casting (edgeTicks - s_lastEdgeTicks) directly to
             * float32 would cast a composite expression across essential
             * type categories (unsigned -> floating). Same idiom Ahrs.c uses
             * (`s_calSum[i] / (float32)n`). */
            uint32 deltaTicks = edgeTicks - s_lastEdgeTicks;

            if (missed > 1u)
            {
                g_imuDrdyMissedEdges += (missed - 1u);
            }
            else
            {
                /* exactly one new edge -- the common case */
            }

            /* The whole reason the ISR exists: dt from the edge timestamps,
             * not from this task's own dispatch interval (the deleted
             * SysTime_getTimeElapsedS() measured the latter, i.e. the
             * jitter -- T15, docs/REFACTORING_PLAN.md §3.6). Correct even
             * across a missed edge -- two edges missed gives dt ~= 2 * period,
             * area preserved. */
            elapsedTime     = (float32)deltaTicks * NAVTASK_TICKS_TO_S;
            s_lastEdgeSeq   = edgeSeq;
            s_lastEdgeTicks = edgeTicks;
        }
        else
        {
            /* Timed out with no new edge at all: no real interval to report,
             * so dt is left outside NAVTASK_DT_MIN_S/MAX_S on purpose --
             * ahrsInputOk below is FALSE regardless of `present`, exactly the
             * "freeze rather than integrate garbage" rule NAVTASK_DT_MIN_S
             * already enforces for every other bad dt. Icm42688_read() is
             * still called (below) so a reconnect keeps being noticed. */
            elapsedTime = 0.0f;
        }

        /* Called whenever this block runs, like the other sensor tasks:
         * Icm42688_read() owns the presence state and uses these calls to
         * probe for a reconnected sensor -- including the timed-out branch
         * above, where there is no new edge at all: a fully silent sensor
         * (power lost, INT1 wire broken) would never call this again if the
         * call were gated on newSample alone, since ITS OWN edges are what
         * would be missing. */
        present = Icm42688_read(&sample);

        /* Attitude first, then navigation: the channel filters need
         * acceleration resolved into NED, and only the AHRS can do that.
         * ahrs.state does not exist yet at this point, so this first gate is
         * dt+presence only -- NavTask_inputValid's AHRS_RUNNING check applies
         * below, once ahrs.state is an output rather than an unknown. */
        /* Written as an if/else into a boolean local, not a direct `&&`
         * assignment: cppcheck's MISRA 10.3 does not recognise `boolean` as
         * an essentially-Boolean type, so it flags a `&&`/comparison result
         * stored straight into one as a different essential type category.
         * Same idiom as navTask_dtValid()/NavTask_inputValid(). */
        ahrsInputOk = FALSE;
        if ((navTask_dtValid(elapsedTime) != FALSE) && (present != FALSE))
        {
            ahrsInputOk = TRUE;
        }
        Ahrs_update(&ahrs, sample.acc, sample.gyro, elapsedTime, ahrsInputOk);

        /* Gate the navigation filter on the attitude being usable, not
         * merely on the IMU answering. While the AHRS is still averaging the
         * gyro bias or waiting to align, its projection is meaningless and
         * integrating it would put a real offset into the velocity before
         * the barometer ever sees it. */
        fusionInputOk = NavTask_inputValid(elapsedTime, present, ahrs.state);
        Fusion_update(&fusion, ahrs.accNed, elapsedTime, fusionInputOk);

        /* Raw sample + an accumulated liveness sum ride along in the SAME
         * publish as the fusion output (T12 blocker,
         * docs/REFACTORING_PLAN.md 3.7): measurementsSetImu() writes
         * g_xcpData (CPU0 DSPR) and PeriphDiag_report() writes PeriphDiag's
         * plain non-volatile s_periph[] -- calling either one from here would
         * make both a two-writer object racing against CPU0's
         * Housekeeping_100ms/PeriphDiag_update. So this task only accumulates
         * and publishes; Housekeeping_100ms (CPU0) makes those calls from
         * the NavState snapshot. Icm42688_plausible()'s instantaneous
         * liveness is summed, never reset, so Housekeeping's diff of two
         * reads sees every consumed tick's contribution -- see
         * NavState_t.imuLiveness. */
        {
            float32 sampleLiveness = 0.0f;

            (void)Icm42688_plausible(&sample, &sampleLiveness);
            s_imuLivenessAccum += sampleLiveness;
        }

        /* Publish for Housekeeping_100ms to pick up (NavState_get) and
         * forward to XCP. Runs on every dispatch that reaches this branch
         * (~1014 Hz in normal operation, up to 2 kHz only in the timed-out
         * fallback) -- cheap (§2.4/§3.7, ~3.9 us), so there is no reason to
         * gate it any further than the block it is already inside. */
        NavState_publish(&ahrs, &fusion, elapsedTime, present,
                          sample.acc, sample.gyro, sample.tempC,
                          s_imuLivenessAccum);
    }
}
