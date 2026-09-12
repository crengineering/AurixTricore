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

/* B4b (SYS1-001 strand B, evidence 952275AD99001303): how long DRDY may stay
 * silent before this task starts actively probing WHO_AM_I
 * (Icm42688_verifyPresence()), deliberately LONGER than
 * NAVTASK_NO_EDGE_TIMEOUT_S above -- that fallback already re-reads the bus
 * every dispatch once silent, which is what catches a genuine SPI failure;
 * this one exists for the case that read keeps SUCCEEDING (a frozen frame),
 * so it must not fire on every ordinary short gap the first fallback already
 * handles cleanly. */
#define NAVTASK_STUCK_VERIFY_TIMEOUT_S   (0.2f)

/* flight-reviewer FAIL, SYS1-001 regression: the no-new-edge (timed out)
 * branch below reports elapsedTime = 0.0f -- deliberately, so it stays
 * outside the dt window and ahrsInputOk/fusionInputOk are never tricked into
 * TRUE by a stale-but-in-range value (see that branch's own comment). But
 * Ahrs_update()'s fault-hold clock (AHRS_FAULT_HOLD_S, Ahrs.c) needs to see
 * SOME real, positive dt on every invalid tick, or a fully silent sensor
 * (dead IMU, broken INT1) never crosses the hold and AHRS_NO_SENSOR never
 * fires -- the estimator is left reporting AHRS_RUNNING on a frozen
 * quaternion forever, which is what the GUI's Attitude view gates "live" on.
 * This is that dt, used ONLY for that one purpose (see its call site) --
 * NOT elapsedTime itself, which must stay 0.0f for the reasons above. Set to
 * this task's own registered dispatch period, NAVTASK_DISPATCH_PERIOD_US
 * (NavTask.h; Cpu1_Main.c registers NavTask_step at
 * SCHED_US(NAVTASK_DISPATCH_PERIOD_US), the SAME macro, not a second number
 * that merely happens to match today): each timed-out dispatch really is
 * ~500 us after the previous one, so summing this in on every such dispatch
 * reconstructs real elapsed wall time to within the scheduler's own jitter
 * -- comfortably good enough against a 50 ms threshold, and errs toward
 * declaring the outage LATE (safe) rather than early if CPU1 ever falls
 * behind its own poll rate. */
#define NAVTASK_TIMEDOUT_FAULT_DT_S   ((float32)NAVTASK_DISPATCH_PERIOD_US * 1.0e-6f)

/* SYS1-001 strand B task 15 (SWE1-FW-009): bound on how fast the gyro may
 * change, per axis, between two ACCEPTED samples -- 197 deg/s over one
 * measured 985 us tick. A quad's airframe angular acceleration is under
 * 10 000 deg/s^2 (20x headroom); the observed defect (a near-full-scale
 * word appearing between two ordinary ticks, evidence rows
 * 7D13E62A0B428BE3/1E1CC203E9702454) implies ~2 000 000 deg/s^2, 10x above
 * this bound. Scaled by dt, not an absolute delta: a genuine LONG gap (the
 * sensor answering late) widens the allowance in proportion, so a real
 * manoeuvre spanning a longer interval is never mistaken for a corrupt
 * sample. */
#define NAVTASK_GYRO_SLEW_DPS_PER_S   (200000.0f)

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

/* SYS1-001 task 1: see NavTask.h for the contract. Kept in the same style as
 * navTask_dtValid()/NavTask_inputValid() -- a chain of POSITIVE tests into a
 * local that starts at the "nothing usable" answer, so a NaN dtS (false
 * against every relational operator) falls straight through to NONE instead
 * of being accepted by omission. */
NavTask_DtClass NavTask_classifyDt(float32 dtS)
{
    NavTask_DtClass result = NAVTASK_DT_NONE;

    if ((dtS >= NAVTASK_DT_MIN_S) && (dtS <= NAVTASK_DT_MAX_S))
    {
        result = NAVTASK_DT_OK;
    }
    else if ((dtS > 0.0f) && (dtS < NAVTASK_DT_MIN_S))
    {
        result = NAVTASK_DT_SHORT;
    }
    else if (dtS > NAVTASK_DT_MAX_S)
    {
        result = NAVTASK_DT_LONG;
    }
    else
    {
        /* dtS <= 0.0f, or NaN -- NAVTASK_DT_NONE, the initial value */
    }

    return result;
}

/* SYS1-001 strand B task 15 (SWE1-FW-009): see NavTask.h for the contract.
 * Pure (no bus access, no state) -- the caller owns "the previous ACCEPTED
 * sample" (NavTask_step's s_lastGyroSensor below), same separation as
 * NavTask_classifyDt()/navTask_dtValid() above. Written as a positive
 * "is inside the band" test per axis, not a negated "outside" one: a NaN
 * delta or a NaN bound (dtS itself NaN) compares false against BOTH
 * relational operators, so the positive form is what rejects it -- the same
 * discipline as every other bound in this file and in Ahrs.c. */
boolean NavTask_gyroSlewOk(const float32 gyro[3], const float32 prev[3], float32 dtS)
{
    boolean       ok    = TRUE;
    const float32 bound = NAVTASK_GYRO_SLEW_DPS_PER_S * dtS;
    uint8         i;

    for (i = 0u; i < 3u; i++)
    {
        const float32 delta = gyro[i] - prev[i];

        if ((delta <= bound) && (delta >= -bound))
        {
            /* this axis is within bound */
        }
        else
        {
            ok = FALSE;
        }
    }

    return ok;
}

/* Running total of Icm42688_plausible()'s per-sample liveness, published
 * verbatim in NavState_t.imuLiveness (NEVER reset here after boot) -- see
 * that field's comment. Housekeeping_100ms does the resetting-by-diffing;
 * this side just keeps adding. CPU1-only, same as every other NavTask.c
 * static. */
static float32 s_imuLivenessAccum;

/* CPU1-side record of the last edge this task actually consumed -- never
 * written to g_imuEdge, matching NavState_get()'s "the reader never writes
 * shared state" rule. File-scope (not NavTask_step-local) since T16:
 * NavTask_init() must seed these from the CURRENT g_imuEdge, not from 0 --
 * see NavTask_init() for why. */
static uint32 s_lastEdgeSeq;
static uint32 s_lastEdgeTicks;

/* Task 15: the gyro (raw, sensor frame, as delivered by Icm42688_read()) of
 * the last tick NavTask_gyroSlewOk() actually accepted -- updated ONLY on
 * acceptance, so a run of rejected ticks keeps comparing against the last
 * KNOWN GOOD reading rather than chaining off a previous bad one (the same
 * reason a rejected sample must not update the reference at all). */
static float32 s_lastGyroSensor[3];

/* cppcheck-suppress-begin misra-c2012-8.7 ; deviation: read over XCP
 * SHORT_UPLOAD by raw address (tools/xcp_read.py), never referenced by C
 * code outside this file -- same class of deviation as g_imuDrdy* (ImuInt.c).
 * SYS1-001 task 0 instrumentation: see NavTask.h for what each one counts. */
volatile uint32 g_dbgNavInvalidTicks;
volatile uint32 g_dbgNavDtShort;
volatile uint32 g_dbgNavDtLong;
volatile uint32 g_dbgNavDtShortMinTicks;
volatile uint32 g_dbgImuReadFail;
/* cppcheck-suppress-end misra-c2012-8.7 */

void NavTask_init(void)
{
    NavState_init();        /* the publish target, before anything publishes */
    FusionCal_init();        /* estimator tuning defaults, BEFORE the filters */
    Ahrs_init();              /* start the gyro-bias calibration; hold still  */
    Fusion_init();            /* zero every channel state and covariance      */
    s_imuLivenessAccum = 0.0f;

    /* SYS1-001 task 0 instrumentation -- see NavTask.h. */
    g_dbgNavInvalidTicks    = 0u;
    g_dbgNavDtShort         = 0u;
    g_dbgNavDtLong          = 0u;
    g_dbgNavDtShortMinTicks = 0xFFFFFFFFu;
    g_dbgImuReadFail        = 0u;

    /* Task 15: no prior accepted sample yet -- zero is a safe reference
     * (the slew bound scaled by even one nominal tick's dt is 197 deg/s,
     * comfortably above any boot-time gyro bias). */
    s_lastGyroSensor[0] = 0.0f;
    s_lastGyroSensor[1] = 0.0f;
    s_lastGyroSensor[2] = 0.0f;

    /* T16 (docs/REFACTORING_PLAN.md §3.6, missedEdges investigation): seed
     * the baseline from whatever the ISR has already produced during
     * Icm42688_init()/BringUp_dumpImu()'s bring-up window (Cpu1_Main.c),
     * not from 0. Those edges were never polled for -- no task existed yet
     * to consume them -- so starting the diff at 0 counted every one of
     * them as "missed" on NavTask_step's very first dispatch: a single,
     * deterministic, boot-time jump in g_imuDrdyMissedEdges indistinguishable,
     * in the published counter, from genuine in-flight loss. Seeding here
     * instead means the counter only ever reports edges missed AFTER this
     * task started actually looking for them. */
    ImuEdge_snapshot(&s_lastEdgeSeq, &s_lastEdgeTicks);
}

/* T15 (docs/REFACTORING_PLAN.md §3.6): registered at
 * SCHED_US(NAVTASK_DISPATCH_PERIOD_US), a 2 kHz poll well above the
 * sensor's measured ~1014.2 Hz -- deliberately faster
 * than the data it waits for, so the edge sequence counter (not the poll
 * period) is the real clock. See the body below for the gate. */
void NavTask_step(void)
{
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
        boolean         duplicateEdge = FALSE;

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
            NavTask_DtClass dtClass;

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
            elapsedTime = (float32)deltaTicks * NAVTASK_TICKS_TO_S;
            dtClass     = NavTask_classifyDt(elapsedTime);

            /* SYS1-001 task 0 instrumentation: name which side of the window
             * a genuine new edge's interval fell on -- see NavTask.h. */
            switch (dtClass)
            {
                case NAVTASK_DT_SHORT:
                    g_dbgNavDtShort++;
                    if (deltaTicks < g_dbgNavDtShortMinTicks)
                    {
                        g_dbgNavDtShortMinTicks = deltaTicks;
                    }
                    break;
                case NAVTASK_DT_LONG:
                    g_dbgNavDtLong++;
                    break;
                case NAVTASK_DT_OK:
                case NAVTASK_DT_NONE:
                default:
                    break;
            }

            if (dtClass == NAVTASK_DT_SHORT)
            {
                /* SYS1-001 task 3: a duplicate DRDY edge, not a fault (the
                 * dispatch's own recommendation, §3 point 2) -- consume the
                 * sequence number so this edge is not seen again, but leave
                 * s_lastEdgeTicks at the last GOOD edge: the next genuine
                 * edge's interval is then measured across the duplicate,
                 * coming out as the full ~985 us period instead of being
                 * truncated by it. No AHRS/fusion update, no publish, and no
                 * fault signalled to Ahrs_update -- previously this ran the
                 * tick through as ahrsInputOk == FALSE, which is exactly the
                 * one-tick glitch that used to re-initialise the estimator
                 * (see Ahrs.c's AHRS_FAULT_HOLD_S for the other half of the
                 * fix). */
                s_lastEdgeSeq = edgeSeq;
                duplicateEdge = TRUE;
            }
            else
            {
                s_lastEdgeSeq   = edgeSeq;
                s_lastEdgeTicks = edgeTicks;
            }
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

        if (duplicateEdge != FALSE)
        {
            /* Nothing else to do for a duplicate edge -- see above. */
        }
        else
        {
            /* Called whenever this block runs, like the other sensor tasks:
             * Icm42688_read() owns the presence state and uses these calls to
             * probe for a reconnected sensor -- including the timed-out
             * branch above, where there is no new edge at all: a fully
             * silent sensor (power lost, INT1 wire broken) would never call
             * this again if the call were gated on newSample alone, since
             * ITS OWN edges are what would be missing. */
            present = Icm42688_read(&sample);
            if (present == FALSE)
            {
                g_dbgImuReadFail++;
            }

            /* B4b trigger 1: DRDY has been silent for a while (longer than
             * the dt-window fallback above, which already re-reads the bus
             * every dispatch) -- ask the sensor directly whether it is still
             * there. Only relevant on the pure-timeout path: a genuine new
             * edge (even a LONG one) means DRDY is not silent at all. Updates
             * `present` in place so a drop is visible to ahrsInputOk/
             * fusionInputOk on THIS same tick, not one tick late. */
            if ((newSample == FALSE)
                && (((float32)g_imuDrdyStaleTicks * NAVTASK_TICKS_TO_S)
                    >= NAVTASK_STUCK_VERIFY_TIMEOUT_S))
            {
                present = Icm42688_verifyPresence(NAVTASK_TIMEDOUT_FAULT_DT_S);
            }
            else
            {
                /* not silent long enough yet -- trigger 1 stays quiet */
            }

            /* Attitude first, then navigation: the channel filters need
             * acceleration resolved into NED, and only the AHRS can do that.
             * ahrs.state does not exist yet at this point, so this first gate
             * is dt+presence only -- NavTask_inputValid's AHRS_RUNNING check
             * applies below, once ahrs.state is an output rather than an
             * unknown. */
            /* Written as an if/else into a boolean local, not a direct `&&`
             * assignment: cppcheck's MISRA 10.3 does not recognise `boolean`
             * as an essentially-Boolean type, so it flags a `&&`/comparison
             * result stored straight into one as a different essential type
             * category. Same idiom as navTask_dtValid()/NavTask_inputValid(). */
            /* Task 15 (SWE1-FW-009): a slew-rejected sample gets the exact
             * same "freeze this tick only" reaction as a bad dt or an absent
             * sensor -- ahrsInputOk FALSE, nothing else. Short-circuits
             * before touching sample.gyro when present == FALSE, so a
             * rejected/absent read (all-zero sample) is never compared
             * against s_lastGyroSensor at all. */
            ahrsInputOk = FALSE;
            if ((navTask_dtValid(elapsedTime) != FALSE) && (present != FALSE)
                && (NavTask_gyroSlewOk(sample.gyro, s_lastGyroSensor, elapsedTime) != FALSE))
            {
                ahrsInputOk = TRUE;
                s_lastGyroSensor[0] = sample.gyro[0];
                s_lastGyroSensor[1] = sample.gyro[1];
                s_lastGyroSensor[2] = sample.gyro[2];
            }
            else
            {
                /* SYS1-001 task 0 instrumentation -- see NavTask.h. Does NOT
                 * include a SHORT (duplicate-edge) tick -- see NavTask.h.
                 * Task 15: also counts a slew-rejected tick here, same
                 * bucket as every other invalid-input case. */
                g_dbgNavInvalidTicks++;
            }

            /* flight-reviewer FAIL, SYS1-001 regression: elapsedTime is
             * deliberately 0.0f above when there was no new edge at all
             * (NAVTASK_TIMEDOUT_FAULT_DT_S's own comment) -- correct for the
             * dt-window/fusion gating this far, but Ahrs_update()'s
             * fault-hold clock needs real, positive time to pass on THIS one
             * call, or a fully silent sensor never crosses AHRS_FAULT_HOLD_S.
             * A genuine new edge (SHORT already handled above, LONG here)
             * already carries the real measured gap in elapsedTime -- only
             * the no-new-edge case needs the override. */
            {
                float32 ahrsDt = elapsedTime;

                if ((ahrsInputOk == FALSE) && (newSample == FALSE))
                {
                    ahrsDt = NAVTASK_TIMEDOUT_FAULT_DT_S;
                }

                Ahrs_update(&ahrs, sample.acc, sample.gyro, ahrsDt, ahrsInputOk);
            }

            /* Gate the navigation filter on the attitude being usable, not
             * merely on the IMU answering. While the AHRS is still averaging
             * the gyro bias or waiting to align, its projection is
             * meaningless and integrating it would put a real offset into
             * the velocity before the barometer ever sees it. */
            fusionInputOk = NavTask_inputValid(elapsedTime, present, ahrs.state);
            Fusion_update(&fusion, ahrs.accNed, elapsedTime, fusionInputOk);

            /* Raw sample + an accumulated liveness sum ride along in the SAME
             * publish as the fusion output (T12 blocker,
             * docs/REFACTORING_PLAN.md 3.7): measurementsSetImu() writes
             * g_xcpData (CPU0 DSPR) and PeriphDiag_report() writes
             * PeriphDiag's plain non-volatile s_periph[] -- calling either
             * one from here would make both a two-writer object racing
             * against CPU0's Housekeeping_100ms/PeriphDiag_update. So this
             * task only accumulates and publishes; Housekeeping_100ms (CPU0)
             * makes those calls from the NavState snapshot.
             * Icm42688_plausible()'s instantaneous liveness is summed, never
             * reset, so Housekeeping's diff of two reads sees every consumed
             * tick's contribution -- see NavState_t.imuLiveness. */
            {
                float32 sampleLiveness = 0.0f;
                boolean plausible;
                float32 plausibleDt = elapsedTime;

                /* Same override as ahrsDt above and for the same reason:
                 * elapsedTime is deliberately 0.0f on the no-new-edge path,
                 * but Icm42688_reportPlausibility()'s hold clock (B4b
                 * trigger 2) needs a real, positive duration on every call. */
                if (newSample == FALSE)
                {
                    plausibleDt = NAVTASK_TIMEDOUT_FAULT_DT_S;
                }

                plausible = Icm42688_plausible(&sample, &sampleLiveness);
                present   = Icm42688_reportPlausibility(plausible, plausibleDt);
                s_imuLivenessAccum += sampleLiveness;
            }

            /* Publish for Housekeeping_100ms to pick up (NavState_get) and
             * forward to XCP. Runs on every dispatch that reaches this branch
             * (~1014 Hz in normal operation, up to 2 kHz only in the
             * timed-out fallback) -- cheap (§2.4/§3.7, ~3.9 us), so there is
             * no reason to gate it any further than the block it is already
             * inside. */
            NavState_publish(&ahrs, &fusion, elapsedTime, present,
                              sample.acc, sample.gyro, sample.tempC,
                              s_imuLivenessAccum);
        }
    }
}
