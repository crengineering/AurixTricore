#include "unity.h"
#include "fakes/Ifx_Types.h"
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
 * NavState.c; SysTime_getTimeElapsedS is the REAL SysTime.c (iLLD-free,
 * already proven host-safe by test_GnssM9N). Only the genuinely
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

void measurementsSetImu(boolean present, const float32 acc[3], const float32 gyro[3],
                        float32 temperatureC)
{
    (void)present;
    (void)acc;
    (void)gyro;
    (void)temperatureC;
}

void PeriphDiag_report(PeriphDiag_Id id, boolean readOk, boolean plausible, float32 liveness)
{
    (void)id;
    (void)readOk;
    (void)plausible;
    (void)liveness;
}

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
    TEST_ASSERT_EQUAL(FALSE, NavTask_inputValid(0.0009f, TRUE, (uint8)AHRS_RUNNING));
}

void test_dt_window_accepts_at_minimum_boundary(void)
{
    TEST_ASSERT_EQUAL(TRUE, NavTask_inputValid(0.001f, TRUE, (uint8)AHRS_RUNNING));
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

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_dt_window_accepts_typical_50hz_period);
    RUN_TEST(test_dt_window_rejects_below_minimum);
    RUN_TEST(test_dt_window_accepts_at_minimum_boundary);
    RUN_TEST(test_dt_window_rejects_above_maximum);
    RUN_TEST(test_dt_window_accepts_at_maximum_boundary);
    RUN_TEST(test_dt_nan_is_rejected);
    RUN_TEST(test_ahrs_not_running_is_rejected);
    RUN_TEST(test_imu_absent_is_rejected);
    RUN_TEST(test_all_three_conditions_required);
    return UNITY_END();
}
