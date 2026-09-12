#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "fakes/IfxStm.h"        /* FakeStm_* -- T16 baseline-seeding tests */
#include <math.h>                 /* fabsf, task 15's yaw-movement check    */
#include <string.h>               /* memcpy, task 15's gyroBias comparison  */
#include <stdio.h>                /* snprintf, failure messages             */
#include "../src/bsw/NavTask.c"   /* pulls in navTask_dtValid, which is static
                                   * and otherwise unreachable -- same reason
                                   * test_GnssM9N.c #includes GnssM9N.c */

/* T11, docs/REFACTORING_PLAN.md §4: NavTask_inputValid is "where the
 * NaN/validity logic lives" and is the one piece of NavTask_step worth a
 * host test. This test never calls NavTask_init()/NavTask_step() -- only
 * NavTask_inputValid() and, transitively through the #include above,
 * navTask_dtValid() -- but including NavTask.c as one translation unit still
 * means every symbol NavTask_init/NavTask_step reference must resolve at
 * link time. FusionCal_init/Ahrs_init/Ahrs_update/Fusion_init/Fusion_update
 * are the REAL, already-host-tested implementations (linked via the
 * `estimator` library); NavState_init/NavState_publish are the REAL
 * NavState.c. T15 (docs/REFACTORING_PLAN.md §3.6) moved `dt` off
 * SysTime_getTimeElapsedS() (deleted -- MISRA 8.7, no callers left) and onto
 * the ImuEdge.h edge timestamps instead; NavTask.c still calls
 * SysTime_getTicks(), the REAL SysTime.c (iLLD-free, already proven
 * host-safe by test_GnssM9N). Only the genuinely
 * target-only calls -- the IMU bus read, the XCP publish, peripheral
 * diagnostics -- are stubbed below, purely to satisfy the linker, the same
 * pattern fakes/I2c.c and fakes/Spi.c already use for the plausibility
 * tests (see test/CMakeLists.txt). */
/* Settable by a test that needs a recovered/present sensor (SYS1-001 task 2
 * flight-reviewer FAIL follow-up); every existing test relies on the FALSE
 * default and never touches this, so their behaviour is unchanged. */
static boolean s_icm42688Present = FALSE;

boolean Icm42688_read(Icm42688_Sample *sample)
{
    if (s_icm42688Present != FALSE)
    {
        /* A plausible level, 1 g reading -- enough for ahrs_align() to see
         * accNorm inside its trust window and actually reach AHRS_RUNNING,
         * not just leave AHRS_ALIGNING. */
        sample->acc[0]  = 0.0f;
        sample->acc[1]  = 0.0f;
        sample->acc[2]  = -1.0f;
        sample->gyro[0] = 0.0f;
        sample->gyro[1] = 0.0f;
        sample->gyro[2] = 0.0f;
        sample->tempC   = 20.0f;
    }
    return s_icm42688Present;
}

boolean Icm42688_plausible(const Icm42688_Sample *sample, float32 *liveness)
{
    (void)sample;
    if (liveness != NULL_PTR)
    {
        *liveness = 0.0f;
    }
    return FALSE;
}

/* B4b (SYS1-001 strand B): link-only stubs for the two new presence
 * triggers, same treatment as Icm42688_read()/Icm42688_plausible() above --
 * NavTask.c's own wiring (WHEN it calls these, with what dt) is what this
 * file tests; the triggers' OWN internal logic is the real Icm42688.c,
 * covered by test_icm42688.c. Trigger 1 is a plain pass-through: nothing
 * here exercises the no-edge/silent path at the presence level (that is a
 * driver-only concern once NavTask.c has decided to call it, which
 * test_navtask.c's stale-tick tests already cover). Trigger 2 keeps a real,
 * minimal duration accumulator mirroring Icm42688.c's own ICM42688_STUCK_HOLD_S
 * (0.1 s) so ONE integration test below can prove NavTask_step's wiring
 * actually drops presence end-to-end; every OTHER existing test in this file
 * calls Icm42688_plausible() (stubbed FALSE, above) for far fewer than 0.1 s
 * of accumulated dt, so this does not change their behaviour. */
static float32 s_stuckHoldS;

boolean Icm42688_verifyPresence(float32 dtS)
{
    (void)dtS;
    return s_icm42688Present;
}

boolean Icm42688_reportPlausibility(boolean plausible, float32 dtS)
{
    if (s_icm42688Present != FALSE)
    {
        if (plausible != FALSE)
        {
            s_stuckHoldS = 0.0f;
        }
        else if ((dtS > 0.0f) && (dtS < 1.0f))
        {
            s_stuckHoldS += dtS;
            if (s_stuckHoldS >= 0.1f)
            {
                g_dbgImuStuckDrops++;
                s_icm42688Present = FALSE;
            }
        }
        else
        {
            /* not a usable interval */
        }
    }
    else
    {
        s_stuckHoldS = 0.0f;
    }
    return s_icm42688Present;
}

volatile uint32 g_dbgImuStuckDrops;
volatile uint32 g_dbgImuWhoAmIFail;

/* I5, docs/IMU_INTERRUPT.md 5.5: NavTask_step now reads/writes these two.
 * Their real storage is in ImuInt.c, which pulls in ERU/SRC/Port headers
 * with no host fakes -- stubbed here like every other target-only symbol
 * above, purely to satisfy the linker. Never read by this test. */
volatile uint32 g_imuDrdyStaleTicks;
volatile uint32 g_imuDrdyLastTicks;

/* T14: reserved, wired in T15 (ImuInt.h) -- NavTask.c's missed-edge counting
 * (T15) references it directly, so it needs the same link-only stub. */
volatile uint32 g_imuDrdyMissedEdges;

/* T12, docs/REFACTORING_PLAN.md 3.7: NavTask_step no longer calls
 * measurementsSetImu()/PeriphDiag_report() at all -- both moved to
 * Housekeeping_100ms (CPU0), reading the raw sample + accumulated liveness
 * back out of NavState instead (see NavTask.c and Housekeeping.c). Nothing
 * left in NavTask.c references either symbol, so the stubs that used to
 * satisfy the linker for them are gone too. */

void setUp(void)
{
    /* B4b: s_stuckHoldS is file-scope so the one integration test below can
     * observe it end-to-end; reset before every test so accumulation from a
     * PREVIOUS test (e.g. the 150-dispatch no-edge-timeout run) can never
     * carry into the next one. */
    s_stuckHoldS = 0.0f;
}

void tearDown(void)
{
}

/* --- the dt window, both directions, boundary-inclusive (>= lo, <= hi) --- */

void test_dt_window_accepts_typical_50hz_period(void)
{
    TEST_ASSERT_EQUAL(TRUE, NavTask_inputValid(0.02f, TRUE, (uint8)AHRS_RUNNING));
}

void test_dt_window_rejects_below_minimum(void)
{
    TEST_ASSERT_EQUAL(FALSE, NavTask_inputValid(0.00019f, TRUE, (uint8)AHRS_RUNNING));
}

void test_dt_window_accepts_at_minimum_boundary(void)
{
    /* T14 (docs/REFACTORING_PLAN.md §3.8): NAVTASK_DT_MIN_S 0.001f -> 0.0002f
     * -- 0.001f used to be the boundary and is now comfortably inside the
     * window, which is the point: the measured 985 us IMU period must clear
     * it, and 0.001f > 0.000985f is exactly the margin check. */
    TEST_ASSERT_EQUAL(TRUE, NavTask_inputValid(0.0002f, TRUE, (uint8)AHRS_RUNNING));
}

void test_dt_window_accepts_measured_imu_period(void)
{
    /* The number this whole change exists for: docs/IMU_INTERRUPT.md §5.6's
     * measured 985.036 us mean interval must be accepted, not merely 1 ms. */
    TEST_ASSERT_EQUAL(TRUE, NavTask_inputValid(0.000985f, TRUE, (uint8)AHRS_RUNNING));
}

void test_dt_window_rejects_above_maximum(void)
{
    TEST_ASSERT_EQUAL(FALSE, NavTask_inputValid(0.2001f, TRUE, (uint8)AHRS_RUNNING));
}

void test_dt_window_accepts_at_maximum_boundary(void)
{
    TEST_ASSERT_EQUAL(TRUE, NavTask_inputValid(0.2f, TRUE, (uint8)AHRS_RUNNING));
}

/* NaN is a real value here, not an edge case (project memory: it has already
 * killed a reject counter and a whole filter channel). The ORIGINAL
 * Cpu0_Main.c form -- `(dt < lo) || (dt > hi)` meaning "reject" -- let a NaN
 * dt through as accepted, because both comparisons are false for NaN. This
 * is the regression test for that specific class of bug. */
void test_dt_nan_is_rejected(void)
{
    float32 nan = 0.0f / 0.0f;
    TEST_ASSERT_EQUAL(FALSE, NavTask_inputValid(nan, TRUE, (uint8)AHRS_RUNNING));
}

/* --- the AHRS_RUNNING gate --- */

void test_ahrs_not_running_is_rejected(void)
{
    TEST_ASSERT_EQUAL(FALSE, NavTask_inputValid(0.02f, TRUE, (uint8)AHRS_CALIBRATING));
}

/* --- SYS1-001 task 1: NavTask_classifyDt(), the pure SHORT/LONG/NONE/OK
 * classifier task 3 uses to tell a duplicate DRDY edge (SHORT) from every
 * other kind of bad interval. --- */

void test_classify_zero_is_none(void)
{
    /* NavTask_step's own sentinel for "no real interval" (the no-new-edge
     * timeout) -- must not read as SHORT. */
    TEST_ASSERT_EQUAL(NAVTASK_DT_NONE, NavTask_classifyDt(0.0f));
}

void test_classify_tiny_positive_is_short(void)
{
    /* 1e-5 s = 10 us, well below NAVTASK_DT_MIN_S (200 us) but a genuine,
     * nonzero measured interval -- the duplicate-edge candidate. */
    TEST_ASSERT_EQUAL(NAVTASK_DT_SHORT, NavTask_classifyDt(1.0e-5f));
}

void test_classify_measured_imu_period_is_ok(void)
{
    TEST_ASSERT_EQUAL(NAVTASK_DT_OK, NavTask_classifyDt(0.000985f));
}

void test_classify_quarter_second_is_long(void)
{
    TEST_ASSERT_EQUAL(NAVTASK_DT_LONG, NavTask_classifyDt(0.25f));
}

void test_classify_nan_is_none(void)
{
    /* NaN compares false against every relational operator -- it must fall
     * through every test and land on NONE, never be silently accepted as
     * OK. */
    float32 nan = 0.0f / 0.0f;
    TEST_ASSERT_EQUAL(NAVTASK_DT_NONE, NavTask_classifyDt(nan));
}

/* --- imuPresent --- */

void test_imu_absent_is_rejected(void)
{
    TEST_ASSERT_EQUAL(FALSE, NavTask_inputValid(0.02f, FALSE, (uint8)AHRS_RUNNING));
}

/* All three gates must hold simultaneously -- not a design a single failing
 * one can slip past by chance. */
void test_all_three_conditions_required(void)
{
    TEST_ASSERT_EQUAL(FALSE, NavTask_inputValid(5.0f, FALSE, (uint8)AHRS_CALIBRATING));
    TEST_ASSERT_EQUAL(TRUE,  NavTask_inputValid(0.02f, TRUE, (uint8)AHRS_RUNNING));
}

/* --- T16: g_imuDrdyMissedEdges must not count the sensor's own bring-up
 * window (docs/REFACTORING_PLAN.md §3.6, missedEdges investigation) ---
 *
 * Cpu1_Main.c calls Icm42688_init()/BringUp_dumpImu() -- which pulses DRDY
 * for real, advancing g_imuEdge.seq -- BEFORE NavTask_init()/NavTask_step()
 * ever run. Seeding s_lastEdgeSeq/s_lastEdgeTicks at 0 (the pre-T16 code)
 * made NavTask_step's first dispatch see that whole bring-up advance as
 * edges it "missed", even though no task existed yet to consume them. These
 * tests simulate that ordering directly: poke g_imuEdge (as the ISR would
 * have, same technique test_imuedge.c uses), THEN call NavTask_init(), THEN
 * NavTask_step(), and check what the counter reports. */

void test_init_seeds_baseline_without_counting_bringup_edges(void)
{
    g_imuDrdyMissedEdges = 0u;
    FakeStm_reset();

    /* Simulate 30 edges having already happened during bring-up, well
     * before NavTask_init() ever runs. */
    FakeStm_setTicks(900000u);
    g_imuEdge.seq   = 30u;
    g_imuEdge.ticks = 900000u;

    NavTask_init();
    TEST_ASSERT_EQUAL_UINT32(0u, g_imuDrdyMissedEdges);

    /* One genuine new edge after init, 985 us later (98500 STM ticks @
     * 100 MHz) -- comfortably inside NAVTASK_DT_MIN_S/MAX_S. A delta of
     * exactly 1 against the seeded baseline must not be flagged. */
    FakeStm_setTicks(998500u);
    g_imuEdge.seq   = 31u;
    g_imuEdge.ticks = 998500u;

    NavTask_step();
    TEST_ASSERT_EQUAL_UINT32(0u, g_imuDrdyMissedEdges);
}

void test_step_still_counts_genuine_multi_edge_gaps(void)
{
    /* Regression for the counting logic itself, unaffected by T16: once the
     * baseline is seeded, a dispatch that observes the sequence jump by more
     * than 1 must still report the difference. */
    g_imuDrdyMissedEdges = 0u;
    FakeStm_reset();

    FakeStm_setTicks(0u);
    g_imuEdge.seq   = 0u;
    g_imuEdge.ticks = 0u;
    NavTask_init();
    TEST_ASSERT_EQUAL_UINT32(0u, g_imuDrdyMissedEdges);

    /* Three edges' worth of ticks (295500 = 3 * 98500), seq advanced by 3 --
     * two genuine edges never individually consumed. */
    FakeStm_setTicks(295500u);
    g_imuEdge.seq   = 3u;
    g_imuEdge.ticks = 295500u;

    NavTask_step();
    TEST_ASSERT_EQUAL_UINT32(2u, g_imuDrdyMissedEdges);
}

/* --- SYS1-001 task 3: a duplicate DRDY edge (SHORT-classified dt) is
 * consumed without a fault -- the sequence number advances, the timestamp
 * does not, and no AHRS/fusion update or NavState publish happens for it.
 * The next GENUINE edge must then measure the FULL nominal interval, not one
 * truncated by the duplicate. --- */

void test_duplicate_edge_is_consumed_without_publish_and_widens_next_dt(void)
{
    NavState_t snap;
    uint32     genAfterFirst;
    uint32     invalidAfterFirst;

    g_dbgNavDtShort      = 0u;
    g_dbgNavInvalidTicks = 0u;
    FakeStm_reset();

    FakeStm_setTicks(0u);
    g_imuEdge.seq   = 0u;
    g_imuEdge.ticks = 0u;
    NavTask_init();

    /* One genuine edge at the nominal period (98500 ticks = 985 us @
     * 100 MHz) -- establishes a real "last good" timestamp. */
    FakeStm_setTicks(98500u);
    g_imuEdge.seq   = 1u;
    g_imuEdge.ticks = 98500u;
    NavTask_step();
    TEST_ASSERT_TRUE(NavState_get(&snap));
    genAfterFirst      = snap.gen;
    invalidAfterFirst  = g_dbgNavInvalidTicks;

    /* A duplicate edge only 50 ticks (500 ns) later -- deep inside the
     * SHORT band (NAVTASK_DT_MIN_S = 200 us = 20000 ticks). */
    FakeStm_setTicks(98550u);
    g_imuEdge.seq   = 2u;
    g_imuEdge.ticks = 98550u;
    NavTask_step();

    TEST_ASSERT_EQUAL_UINT32(1u, g_dbgNavDtShort);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(invalidAfterFirst, g_dbgNavInvalidTicks,
        "a duplicate edge must not be counted as an invalid AHRS input tick");
    TEST_ASSERT_TRUE(NavState_get(&snap));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(genAfterFirst, snap.gen,
        "a duplicate edge must not publish a new NavState snapshot");

    /* The next GENUINE edge, one full nominal period after the DUPLICATE
     * (98550 + 98500 = 197050) -- s_lastEdgeTicks was never advanced past
     * the first genuine edge (98500), so this must measure as the FULL
     * interval (~985 us), not truncated by the duplicate. */
    FakeStm_setTicks(197050u);
    g_imuEdge.seq   = 3u;
    g_imuEdge.ticks = 197050u;
    NavTask_step();

    TEST_ASSERT_TRUE(NavState_get(&snap));
    TEST_ASSERT_TRUE_MESSAGE(snap.gen != genAfterFirst,
        "the next genuine edge must publish");
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.000985f, snap.dtS);
}

/* --- SYS1-001 flight-reviewer FAIL follow-up: the no-new-edge timeout path
 * must still declare AHRS_NO_SENSOR after a genuine outage, driven through
 * the REAL NavTask_step integration (not a hand-picked dt at the Ahrs.c
 * level, which is what let the original regression pass its own tests). --- */

void test_no_edge_timeout_declares_no_sensor_then_realigns_once(void)
{
    NavState_t snap;
    boolean    sawNoSensor = FALSE;
    uint32     realignsBefore;
    int        i;

    g_dbgNavInvalidTicks = 0u;
    s_icm42688Present    = FALSE;
    FakeStm_reset();

    FakeStm_setTicks(0u);
    g_imuEdge.seq   = 0u;
    g_imuEdge.ticks = 0u;
    NavTask_init();

    /* No new edge ever arrives (g_imuEdge left untouched) while SysTime runs
     * far ahead of it -- every dispatch from here on takes the timed-out
     * branch. 150 dispatches * NAVTASK_TIMEDOUT_FAULT_DT_S (0.5 ms) = 75 ms
     * of accumulated fault-hold time, comfortably over AHRS_FAULT_HOLD_S
     * (50 ms, Ahrs.c). */
    FakeStm_setTicks(50000000u);   /* comfortably past NAVTASK_NO_EDGE_TIMEOUT_S */
    for (i = 0; i < 150; ++i)
    {
        NavTask_step();
        TEST_ASSERT_TRUE(NavState_get(&snap));
        if (snap.ahrs.state == (uint8)AHRS_NO_SENSOR) { sawNoSensor = TRUE; }
    }
    TEST_ASSERT_TRUE_MESSAGE(sawNoSensor,
        "150 no-new-edge dispatches (75 ms of NAVTASK_TIMEDOUT_FAULT_DT_S) "
        "never reached AHRS_NO_SENSOR -- the fault-hold clock is not "
        "advancing on the timeout path");
    TEST_ASSERT_EQUAL_UINT32((uint32)AHRS_NO_SENSOR, (uint32)snap.ahrs.state);
    TEST_ASSERT_TRUE_MESSAGE(g_dbgNavInvalidTicks > 0u,
        "the timeout path must still count as invalid ticks (NavTask.h)");

    /* One genuine edge, present == TRUE, a normal in-window interval
     * relative to the ORIGINAL last-good edge (tick 0) -- picked
     * deliberately so this tick classifies OK, not LONG, isolating the
     * re-align check from the LONG-edge path (covered separately in
     * test_ahrs.c). */
    realignsBefore    = g_dbgAhrsRealigns;
    s_icm42688Present = TRUE;
    g_imuEdge.seq     = 1u;
    g_imuEdge.ticks   = 98500u;   /* 985 us after the original edge at tick 0 */
    NavTask_step();

    TEST_ASSERT_TRUE(NavState_get(&snap));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE((uint32)AHRS_RUNNING, (uint32)snap.ahrs.state,
        "the next good sample must resume RUNNING");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(realignsBefore + 1u, g_dbgAhrsRealigns,
        "recovery from the timeout outage must re-align exactly once");

    s_icm42688Present = FALSE;   /* leave the stub as every other test expects it */
}

/* --- B4b (SYS1-001 strand B, evidence 952275AD99001303) -- trigger 2's
 * WIRING through the real NavTask_step, not just the driver logic
 * (test_icm42688.c covers Icm42688_reportPlausibility() itself; this proves
 * NavTask.c actually calls it, with a real dt, on the path that matters). --- */

void test_stuck_sample_drops_presence_after_100ms_via_navtask_wiring(void)
{
    NavState_t snap;
    int        i;

    g_dbgImuStuckDrops = 0u;
    s_icm42688Present  = TRUE;
    FakeStm_reset();

    FakeStm_setTicks(0u);
    g_imuEdge.seq   = 0u;
    g_imuEdge.ticks = 0u;
    NavTask_init();

    /* No new edge ever arrives -- every dispatch takes the timed-out branch,
     * Icm42688_plausible() is stubbed FALSE (above), so
     * Icm42688_reportPlausibility() accumulates NAVTASK_TIMEDOUT_FAULT_DT_S
     * (0.5 ms) each call. 250 dispatches = 125 ms, comfortably over the
     * 100 ms hold (Icm42688.c ICM42688_STUCK_HOLD_S, mirrored in this file's
     * stub). */
    FakeStm_setTicks(50000000u);
    for (i = 0; i < 250; ++i)
    {
        NavTask_step();
    }

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1u, g_dbgImuStuckDrops,
        "250 dispatches (125 ms) of a continuously-implausible sample must "
        "drop presence exactly once");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(FALSE, s_icm42688Present,
        "the stub's presence flag must reflect the drop");
    TEST_ASSERT_TRUE(NavState_get(&snap));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(FALSE, snap.imuPresent,
        "NavState must publish the drop on the same tick it happens");

    s_icm42688Present = FALSE;   /* leave the stub as every other test expects it */
}

/* ==========================================================================
 * SYS1-001 strand B task 15 (SWE1-FW-009): NavTask_gyroSlewOk() -- the
 * per-axis rate-of-change bound on the gyro -- and its effect wired exactly
 * the way NavTask_step wires it: a rejection makes Ahrs_update()'s `valid`
 * FALSE for that tick and nothing else (SWE1-FW-001's debounce owns the
 * rest, unchanged).
 * ======================================================================== */

#define T15_DT   (1.0f / 1014.2f)   /* measured DRDY rate, docs/IMU_INTERRUPT.md */

void test_gyro_slew_ok_accepts_zero_change(void)
{
    const float32 gyro[3] = { 0.0f, 0.0f, 0.0f };
    const float32 prev[3] = { 0.0f, 0.0f, 0.0f };
    TEST_ASSERT_TRUE(NavTask_gyroSlewOk(gyro, prev, T15_DT));
}

void test_gyro_slew_ok_accepts_at_the_boundary(void)
{
    /* 200 000 deg/s^2 * T15_DT = 197.088 deg/s -- boundary-inclusive
     * (NavTask_gyroSlewOk uses <=/>=, not strict). */
    const float32 bound   = 200000.0f * T15_DT;
    const float32 gyro[3] = { bound, -bound, 0.0f };
    const float32 prev[3] = { 0.0f, 0.0f, 0.0f };
    TEST_ASSERT_TRUE_MESSAGE(NavTask_gyroSlewOk(gyro, prev, T15_DT),
        "a delta exactly AT the bound must be accepted (boundary-inclusive)");
}

void test_gyro_slew_ok_rejects_just_past_the_boundary(void)
{
    const float32 bound   = 200000.0f * T15_DT;
    const float32 gyro[3] = { bound + 1.0f, 0.0f, 0.0f };
    const float32 prev[3] = { 0.0f, 0.0f, 0.0f };
    TEST_ASSERT_FALSE_MESSAGE(NavTask_gyroSlewOk(gyro, prev, T15_DT),
        "a delta past the bound must be rejected");
}

void test_gyro_slew_ok_rejects_nan(void)
{
    float32 nan = 0.0f / 0.0f;
    const float32 gyro[3] = { nan, 0.0f, 0.0f };
    const float32 prev[3] = { 0.0f, 0.0f, 0.0f };
    TEST_ASSERT_FALSE_MESSAGE(NavTask_gyroSlewOk(gyro, prev, T15_DT),
        "NaN compares false against every relational operator -- must be "
        "rejected, not accepted by omission");
}

void test_gyro_slew_ok_rejects_nonzero_change_over_zero_dt(void)
{
    const float32 gyro[3] = { 1.0f, 0.0f, 0.0f };
    const float32 prev[3] = { 0.0f, 0.0f, 0.0f };
    TEST_ASSERT_FALSE_MESSAGE(NavTask_gyroSlewOk(gyro, prev, 0.0f),
        "no elapsed time cannot excuse a nonzero change");
}

void test_gyro_slew_ok_accepts_190dps_per_tick_ramp(void)
{
    /* Acceptance: a 190 deg/s-per-tick ramp (just under the ~197 deg/s bound
     * at this dt) is legitimate fast handling and must be accepted EVERY
     * tick, not just once -- the "does not block real motion" half of the
     * claim, updating the reference each time exactly as NavTask_step does
     * on acceptance. */
    float32 prev[3] = { 0.0f, 0.0f, 0.0f };
    int     i;

    for (i = 0; i < 20; ++i)
    {
        const float32 gyro[3] = { 0.0f, 0.0f, prev[2] + 190.0f };
        char msg[64];

        (void)snprintf(msg, sizeof msg, "tick %d: gyro_z = %g dps", i, (double)gyro[2]);
        TEST_ASSERT_TRUE_MESSAGE(NavTask_gyroSlewOk(gyro, prev, T15_DT), msg);
        prev[0] = gyro[0];
        prev[1] = gyro[1];
        prev[2] = gyro[2];
    }
}

/* --- the reproduction: the evidence-row scenario, composed exactly the way
 * NavTask_step composes NavTask_gyroSlewOk() into Ahrs_update()'s `valid`
 * (test_navtask.c's own Icm42688_read() stub always returns a fixed sample,
 * so this drives Ahrs_update() directly -- the real, already-host-tested
 * implementation, linked via the `estimator` library, exactly as every
 * other Ahrs_update() call in this file's stubs is). --- */

void test_slew_rejection_prevents_corrupt_word_from_moving_yaw(void)
{
    Ahrs_Values   v; memset(&v, 0, sizeof v);
    const float32 accLevel[3]    = { 0.0f, 0.0f, -1.0f };
    const float32 gyroQuiet[3]   = { 0.0f, 0.0f, 0.0f };
    const float32 gyroCorrupt[3] = { 0.0f, 0.0f, -1967.0f };   /* evidence row 7D13E62A0B428BE3 */
    const float32 lastGyro[3]    = { 0.0f, 0.0f, 0.0f };
    float32       yawBefore;
    float32       gyroBiasBefore[3];
    boolean       slewOk;
    int           i;

    Ahrs_init();
    for (i = 0; i < 4000; ++i)
    {
        Ahrs_update(&v, accLevel, gyroQuiet, T15_DT, TRUE);
    }
    TEST_ASSERT_EQUAL(AHRS_RUNNING, v.state);

    yawBefore = v.yawRad;
    memcpy(gyroBiasBefore, v.gyroBias, sizeof gyroBiasBefore);

    slewOk = NavTask_gyroSlewOk(gyroCorrupt, lastGyro, T15_DT);
    TEST_ASSERT_FALSE_MESSAGE(slewOk, "the evidence-row magnitude must fail the slew bound");

    /* Exactly NavTask_step's own wiring: a slew rejection makes `valid`
     * FALSE for this tick and nothing else. */
    Ahrs_update(&v, accLevel, gyroCorrupt, T15_DT, slewOk);

    {
        const float32 yawMovedDeg = fabsf((v.yawRad - yawBefore) * (180.0f / 3.14159265f));
        char          msg[128];

        (void)snprintf(msg, sizeof msg,
            "yaw moved %.4f deg (baseline defect: 1.94 deg, target < 0.05)",
            (double)yawMovedDeg);
        TEST_ASSERT_TRUE_MESSAGE(yawMovedDeg < 0.05f, msg);
    }
    TEST_ASSERT_EQUAL_FLOAT_ARRAY_MESSAGE(gyroBiasBefore, v.gyroBias, 3,
        "both integrals (published via gyroBias) must be bit-unchanged on a rejected tick");
}

void test_40_consecutive_slew_rejections_do_not_realign(void)
{
    Ahrs_Values   v; memset(&v, 0, sizeof v);
    const float32 accLevel[3]    = { 0.0f, 0.0f, -1.0f };
    const float32 gyroQuiet[3]   = { 0.0f, 0.0f, 0.0f };
    const float32 gyroCorrupt[3] = { 0.0f, 0.0f, -1967.0f };
    const float32 lastGyro[3]    = { 0.0f, 0.0f, 0.0f };
    uint32        realignsBefore;
    int           i;

    Ahrs_init();
    for (i = 0; i < 4000; ++i)
    {
        Ahrs_update(&v, accLevel, gyroQuiet, T15_DT, TRUE);
    }
    TEST_ASSERT_EQUAL(AHRS_RUNNING, v.state);
    realignsBefore = g_dbgAhrsRealigns;

    /* 1..40 consecutive rejections: SWE1-FW-001's debounce (AHRS_FAULT_HOLD_S
     * = 0.05 s) must not have crossed at 40 * T15_DT (~39.4 ms), so no
     * re-align -- this also proves the slew check keeps comparing against
     * the same last-KNOWN-GOOD reference rather than chaining off a
     * previous rejected sample: lastGyro is never updated in this loop,
     * exactly as NavTask_step never updates s_lastGyroSensor on a rejection. */
    for (i = 1; i <= 40; ++i)
    {
        char    msg[64];
        boolean slewOk = NavTask_gyroSlewOk(gyroCorrupt, lastGyro, T15_DT);

        Ahrs_update(&v, accLevel, gyroCorrupt, T15_DT, slewOk);
        (void)snprintf(msg, sizeof msg, "after %d consecutive rejections", i);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(realignsBefore, g_dbgAhrsRealigns, msg);
        TEST_ASSERT_EQUAL_MESSAGE(AHRS_RUNNING, v.state, msg);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_dt_window_accepts_typical_50hz_period);
    RUN_TEST(test_dt_window_rejects_below_minimum);
    RUN_TEST(test_dt_window_accepts_at_minimum_boundary);
    RUN_TEST(test_dt_window_accepts_measured_imu_period);
    RUN_TEST(test_dt_window_rejects_above_maximum);
    RUN_TEST(test_dt_window_accepts_at_maximum_boundary);
    RUN_TEST(test_dt_nan_is_rejected);
    RUN_TEST(test_ahrs_not_running_is_rejected);
    RUN_TEST(test_imu_absent_is_rejected);
    RUN_TEST(test_all_three_conditions_required);
    RUN_TEST(test_init_seeds_baseline_without_counting_bringup_edges);
    RUN_TEST(test_step_still_counts_genuine_multi_edge_gaps);
    RUN_TEST(test_classify_zero_is_none);
    RUN_TEST(test_classify_tiny_positive_is_short);
    RUN_TEST(test_classify_measured_imu_period_is_ok);
    RUN_TEST(test_classify_quarter_second_is_long);
    RUN_TEST(test_classify_nan_is_none);
    RUN_TEST(test_duplicate_edge_is_consumed_without_publish_and_widens_next_dt);
    RUN_TEST(test_no_edge_timeout_declares_no_sensor_then_realigns_once);
    RUN_TEST(test_stuck_sample_drops_presence_after_100ms_via_navtask_wiring);
    RUN_TEST(test_gyro_slew_ok_accepts_zero_change);
    RUN_TEST(test_gyro_slew_ok_accepts_at_the_boundary);
    RUN_TEST(test_gyro_slew_ok_rejects_just_past_the_boundary);
    RUN_TEST(test_gyro_slew_ok_rejects_nan);
    RUN_TEST(test_gyro_slew_ok_rejects_nonzero_change_over_zero_dt);
    RUN_TEST(test_gyro_slew_ok_accepts_190dps_per_tick_ramp);
    RUN_TEST(test_slew_rejection_prevents_corrupt_word_from_moving_yaw);
    RUN_TEST(test_40_consecutive_slew_rejections_do_not_realign);
    return UNITY_END();
}
