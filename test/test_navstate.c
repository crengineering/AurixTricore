#include <string.h>
#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/NavState.h"

/* T10, docs/REFACTORING_PLAN.md §4: the publish/get PROTOCOL, host-testable
 * without any iLLD header.
 *
 * What this test canNOT cover, and why:
 *  - The memory placement (__at(0xB00F0060)) and the DSYNC barrier itself --
 *    hardware/.src checks (see fakes/tasking_shim.h and NavState.src).
 *  - The torn-read retry actually firing, and the final FALSE path. Both are
 *    ONLY reachable when gen changes between NavState_get's own read and its
 *    re-check -- which requires a second thread (or interrupt) actually
 *    publishing mid-copy. This test suite is single-threaded, so every call
 *    observes a quiescent block and necessarily succeeds on the first
 *    attempt: there is no way to force the retry branch without either a
 *    real concurrent writer (flaky by nature, and this project has no
 *    threading test precedent to build on) or instrumenting NavState_get
 *    itself for injection, which would test the harness rather than the
 *    shipped function. Rather than fake this coverage, it is left
 *    unexercised here and flagged in the T10 report -- same honesty the plan
 *    already applies to the placement/barrier checks above. */

static Ahrs_Values  s_ahrs;
static FusionValues s_fusion;

void setUp(void)
{
    /* Fresh, known payload each test: distinct, non-zero values so a
     * get() that returns stale/zeroed data is caught, not missed. */
    uint8 i;

    s_ahrs.q[0] = 1.0f; s_ahrs.q[1] = 0.0f; s_ahrs.q[2] = 0.0f; s_ahrs.q[3] = 0.0f;
    s_ahrs.rollRad  = 0.1f;
    s_ahrs.pitchRad = 0.2f;
    s_ahrs.yawRad   = 0.3f;
    for (i = 0u; i < 3u; i++)
    {
        s_ahrs.rate[i]      = 1.0f + (float32)i;
        s_ahrs.gyroBias[i]  = 2.0f + (float32)i;
        s_ahrs.accNed[i]    = 3.0f + (float32)i;
    }
    s_ahrs.accMagG      = 0.998f;
    s_ahrs.magFieldG    = 0.48f;
    s_ahrs.state        = 2u;
    s_ahrs.accTrusted   = 1u;
    s_ahrs.magTrusted   = 1u;
    s_ahrs.biasDegraded = 0u;

    s_fusion.a_D = 1.0f; s_fusion.a_d = 2.0f; s_fusion.a_v_d = 3.0f;
    s_fusion.accBiasD = 0.01f; s_fusion.baroBias = 0.02f;
    s_fusion.innov = 0.03f; s_fusion.p00 = 0.04f;
    s_fusion.a_N = 4.0f; s_fusion.a_E = 5.0f;
    s_fusion.posN = 6.0f; s_fusion.posE = 7.0f;
    s_fusion.velN = 8.0f; s_fusion.velE = 9.0f;
    s_fusion.accBiasN = 0.05f; s_fusion.accBiasE = 0.06f;
    s_fusion.innovN = 0.07f; s_fusion.innovE = 0.08f;
    s_fusion.pNN = 0.09f;
    s_fusion.originLatDeg = 48.1f; s_fusion.originLonDeg = 11.5f; s_fusion.originAltM = 519.0f;
    s_fusion.rejects = 1u; s_fusion.resets = 2u;
    s_fusion.gnssRejects = 3u; s_fusion.gnssUpdates = 4u; s_fusion.covResets = 5u;
    s_fusion.gnssITow = 123456u; s_fusion.dropped = 6u; s_fusion.gnssDupes = 7u;
    s_fusion.verticalOk = 1u; s_fusion.horizontalOk = 1u; s_fusion.originSet = 1u;
    s_fusion.reserved = 0u;

    NavState_init();
}

void tearDown(void)
{
}

/* get() before any publish() must succeed (gen==0, the init state) and
 * report the zeroed payload NavState_init() wrote. */
void test_get_before_publish_returns_init_state(void)
{
    NavState_t snap;
    boolean    ok;

    memset(&snap, 0xAA, sizeof(snap));   /* poison, so a no-op get() is caught */
    ok = NavState_get(&snap);

    TEST_ASSERT_EQUAL(TRUE, ok);
    TEST_ASSERT_EQUAL_UINT32(0u, snap.gen);
    TEST_ASSERT_EQUAL_UINT32(0u, snap.imuPresent);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, snap.dtS);
    TEST_ASSERT_EQUAL_UINT8(0u, snap.ahrs.state);
}

/* One publish -> one matching get(), gen advanced by exactly one, and every
 * field round-trips -- the core protocol claim. */
void test_publish_then_get_roundtrips_and_advances_gen(void)
{
    NavState_t snap;
    boolean    ok;

    ok = NavState_get(&snap);
    TEST_ASSERT_EQUAL(TRUE, ok);
    TEST_ASSERT_EQUAL_UINT32(0u, snap.gen);

    NavState_publish(&s_ahrs, &s_fusion, 0.02f, TRUE);

    ok = NavState_get(&snap);
    TEST_ASSERT_EQUAL(TRUE, ok);
    TEST_ASSERT_EQUAL_UINT32(1u, snap.gen);
    TEST_ASSERT_EQUAL_UINT32(1u, snap.imuPresent);
    TEST_ASSERT_EQUAL_FLOAT(0.02f, snap.dtS);
    TEST_ASSERT_EQUAL_FLOAT(s_ahrs.pitchRad, snap.ahrs.pitchRad);
    TEST_ASSERT_EQUAL_UINT8(s_ahrs.state, snap.ahrs.state);
    TEST_ASSERT_EQUAL_FLOAT(s_fusion.posN, snap.fusion.posN);
    TEST_ASSERT_EQUAL_UINT32(s_fusion.gnssITow, snap.fusion.gnssITow);
}

/* imuPresent is stored as 0/1 regardless of what non-zero value `boolean`
 * happens to carry in -- the "was uint8, now uint32" field must not become a
 * second place a stray non-{0,1} boolean value leaks through. */
void test_publish_normalises_imu_present_to_0_or_1(void)
{
    NavState_t snap;

    NavState_publish(&s_ahrs, &s_fusion, 0.02f, (boolean)42u);
    (void)NavState_get(&snap);
    TEST_ASSERT_EQUAL_UINT32(1u, snap.imuPresent);

    NavState_publish(&s_ahrs, &s_fusion, 0.02f, FALSE);
    (void)NavState_get(&snap);
    TEST_ASSERT_EQUAL_UINT32(0u, snap.imuPresent);
}

/* Repeated publishes each advance gen by exactly one -- the monotonic
 * counter claim, not just "some number changed". */
void test_gen_increments_by_one_per_publish(void)
{
    NavState_t snap;
    uint8 i;

    for (i = 0u; i < 5u; i++)
    {
        NavState_publish(&s_ahrs, &s_fusion, 0.02f, TRUE);
    }

    (void)NavState_get(&snap);
    TEST_ASSERT_EQUAL_UINT32(5u, snap.gen);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_get_before_publish_returns_init_state);
    RUN_TEST(test_publish_then_get_roundtrips_and_advances_gen);
    RUN_TEST(test_publish_normalises_imu_present_to_0_or_1);
    RUN_TEST(test_gen_increments_by_one_per_publish);
    return UNITY_END();
}
