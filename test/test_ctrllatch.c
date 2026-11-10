#include <math.h>
#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/CtrlLatch.h"

/* CtrlLatch: the CPU1 -> CPU0 handoff of the flight controller's rotor-speed
 * requests. Host-testable: the seqlock protocol as seen by a quiescent reader,
 * the "writer inside" (odd gen) rejection, and CtrlLatch_request()'s staleness /
 * status / range handling. NOT testable here: a publish overlapping the copy
 * (needs a second thread) and the DSYNC barrier (hardware, see tasking_shim.h).
 *
 * CtrlLatch_request() keeps file-static state (last snapshot, stale counter),
 * so the request tests run as ONE ordered scenario. */

static const float32 s_tau[3]  = { 0.1f, -0.2f, 0.3f };
static const float32 s_om[3]   = { 1.0f, 2.0f, 3.0f };
static const float32 s_tauI[3] = { 0.01f, 0.02f, 0.03f };
static const float32 s_phi[3]  = { 0.1f, -0.1f, 2.0f };

void setUp(void)
{
}

void tearDown(void)
{
}

void test_publish_get_roundtrip(void)
{
    const float32 rpm[4] = { 4000.0f, 4100.0f, 4200.0f, 4300.0f };
    CtrlLatch_t   out;
    const uint32  gen0 = g_ctrlLatch.gen;

    CtrlLatch_publish(rpm, s_tau, s_om, s_tauI, s_phi, CTRLLATCH_ST_DRIVE);
    TEST_ASSERT_TRUE(CtrlLatch_get(&out));
    TEST_ASSERT_EQUAL_UINT32(gen0 + 2u, out.gen);            /* odd in between, even when stable */
    TEST_ASSERT_EQUAL_UINT32(0u, out.gen & 1u);
    for (uint32 i = 0u; i < 4u; i++) { TEST_ASSERT_EQUAL_FLOAT(rpm[i], out.rpm[i]); }
    for (uint32 i = 0u; i < 3u; i++)
    {
        TEST_ASSERT_EQUAL_FLOAT(s_tau[i],  out.tau[i]);
        TEST_ASSERT_EQUAL_FLOAT(s_om[i],   out.omSoll[i]);
        TEST_ASSERT_EQUAL_FLOAT(s_tauI[i], out.tauI[i]);
        TEST_ASSERT_EQUAL_FLOAT(s_phi[i],  out.phiSoll[i]);
    }
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_DRIVE, out.status);
}

void test_get_rejects_writer_inside(void)
{
    CtrlLatch_t out;

    g_ctrlLatch.gen = g_ctrlLatch.gen + 1u;                  /* what a reader sees mid-publish */
    TEST_ASSERT_FALSE(CtrlLatch_get(&out));
    g_ctrlLatch.gen = g_ctrlLatch.gen + 1u;
    TEST_ASSERT_TRUE(CtrlLatch_get(&out));
}

/* Must be the first test that calls CtrlLatch_request(): checks the boot state. */
void test_request_scenario(void)
{
    const float32 rpm[4]    = { 4000.4f, 4000.6f, 1300.0f, 8260.0f };
    const float32 rpmBad[4] = { NAN, -5.0f, 70000.0f, 0.2f };
    uint16        rq[4]     = { 1u, 1u, 1u, 1u };
    CtrlLatch_t   snap;

    /* boot: nothing new published since the last request -> unusable, zeros */
    g_ctrlLatch.gen = 0u;
    TEST_ASSERT_FALSE(CtrlLatch_request(rq, &snap));
    for (uint32 i = 0u; i < 4u; i++) { TEST_ASSERT_EQUAL_UINT16(0u, rq[i]); }
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_STALE, snap.status);

    /* fresh publish -> usable, rounded to the nearest rpm */
    CtrlLatch_publish(rpm, s_tau, s_om, s_tauI, s_phi, CTRLLATCH_ST_DRIVE);
    TEST_ASSERT_TRUE(CtrlLatch_request(rq, &snap));
    TEST_ASSERT_EQUAL_UINT16(4000u, rq[0]);
    TEST_ASSERT_EQUAL_UINT16(4001u, rq[1]);
    TEST_ASSERT_EQUAL_UINT16(1300u, rq[2]);
    TEST_ASSERT_EQUAL_UINT16(8260u, rq[3]);
    TEST_ASSERT_EQUAL_FLOAT(rpm[0], snap.rpm[0]);
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_DRIVE, snap.status);

    /* the flight core goes silent: the last request holds for STALE_CALLS - 1 calls ... */
    for (uint32 n = 1u; n < CTRLLATCH_STALE_CALLS; n++)
    {
        TEST_ASSERT_TRUE(CtrlLatch_request(rq, &snap));
        TEST_ASSERT_EQUAL_UINT16(4000u, rq[0]);
    }
    /* ... then it is dead, and stays dead */
    for (uint32 n = 0u; n < 3u; n++)
    {
        TEST_ASSERT_FALSE(CtrlLatch_request(rq, &snap));
        for (uint32 i = 0u; i < 4u; i++) { TEST_ASSERT_EQUAL_UINT16(0u, rq[i]); }
    }
    TEST_ASSERT_EQUAL_FLOAT(rpm[0], snap.rpm[0]);            /* the log still shows the last one */
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_DRIVE | CTRLLATCH_ST_STALE, snap.status);   /* and says why */
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_DRIVE, g_ctrlLatch.status);                 /* reader-side bit only */

    /* it comes back with the next publish */
    CtrlLatch_publish(rpm, s_tau, s_om, s_tauI, s_phi, 0u);
    TEST_ASSERT_TRUE(CtrlLatch_request(rq, &snap));
    TEST_ASSERT_EQUAL_UINT16(4000u, rq[0]);

    /* a writer caught inside counts as "no new publish", not as a request */
    g_ctrlLatch.gen = g_ctrlLatch.gen + 1u;
    TEST_ASSERT_TRUE(CtrlLatch_request(rq, &snap));          /* stale call 1: last one holds */
    g_ctrlLatch.gen = g_ctrlLatch.gen + 1u;

    /* controller says the estimate is unusable / the tilt limit is exceeded */
    CtrlLatch_publish(rpm, s_tau, s_om, s_tauI, s_phi, CTRLLATCH_ST_TILT);
    TEST_ASSERT_FALSE(CtrlLatch_request(rq, &snap));
    for (uint32 i = 0u; i < 4u; i++) { TEST_ASSERT_EQUAL_UINT16(0u, rq[i]); }
    TEST_ASSERT_EQUAL_UINT32(CTRLLATCH_ST_TILT, snap.status);
    CtrlLatch_publish(rpm, s_tau, s_om, s_tauI, s_phi, CTRLLATCH_ST_NAV_INVALID);
    TEST_ASSERT_FALSE(CtrlLatch_request(rq, &snap));

    /* values that are not a speed: NaN and negative -> 0, above uint16 -> 65535, < 0.5 -> 0 */
    CtrlLatch_publish(rpmBad, s_tau, s_om, s_tauI, s_phi, 0u);
    TEST_ASSERT_TRUE(CtrlLatch_request(rq, &snap));
    TEST_ASSERT_EQUAL_UINT16(0u,     rq[0]);
    TEST_ASSERT_EQUAL_UINT16(0u,     rq[1]);
    TEST_ASSERT_EQUAL_UINT16(65535u, rq[2]);
    TEST_ASSERT_EQUAL_UINT16(0u,     rq[3]);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_request_scenario);          /* first: needs CtrlLatch_request's boot state */
    RUN_TEST(test_publish_get_roundtrip);
    RUN_TEST(test_get_rejects_writer_inside);
    return UNITY_END();
}
