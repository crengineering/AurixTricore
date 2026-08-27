#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "fakes/IfxStm.h"        /* FakeStm_* -- T16 baseline-seeding tests */
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
boolean Icm42688_read(Icm42688_Sample *sample)
{
    (void)sample;
    return FALSE;
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
    return UNITY_END();
}
