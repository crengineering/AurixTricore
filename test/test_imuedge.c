#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/ImuEdge.h"

/* T15, docs/REFACTORING_PLAN.md §3.6: the ISR->NavTask_step edge handoff
 * protocol, host-testable because ImuEdge_snapshot() is pure and iLLD-free
 * (Ifx_Types.h only) -- same idiom as ImuInt_accumulate() (test_imuint.c).
 *
 * What this test canNOT cover, and why -- same limitation test_navstate.c
 * already documents for NavState_get(): the torn-read retry actually firing
 * is only reachable when `seq`/`ticks` change between ImuEdge_snapshot()'s
 * own read and its re-check, which needs a real concurrent writer (the ISR).
 * This suite is single-threaded, so every call observes a quiescent object
 * and succeeds on the first attempt. Left unexercised here, same honesty
 * test_navstate.c applies, rather than faking it by instrumenting the
 * function under test. The memory placement (__at(0xB00F0500)) is a
 * hardware/.map check, not a host one (see the T15 report). */

void setUp(void)
{
    /* Directly poking g_imuEdge, not calling the real ISR (ImuInt.c pulls in
     * ERU/SRC/Port headers with no host fakes) -- this is exactly the
     * producer side's write order (payload, then seq), just without the
     * Ifx__dsync() in between: on the host that wraps a no-op anyway
     * (fakes/tasking_shim.h), and ordering within one thread does not need a
     * barrier regardless. */
    g_imuEdge.ticks = 0u;
    g_imuEdge.seq   = 0u;
}

void tearDown(void)
{
}

void test_snapshot_before_any_write_reads_the_zeroed_object(void)
{
    uint32 seq   = 111u;
    uint32 ticks = 222u;

    ImuEdge_snapshot(&seq, &ticks);

    TEST_ASSERT_EQUAL_UINT32(0u, seq);
    TEST_ASSERT_EQUAL_UINT32(0u, ticks);
}

void test_snapshot_reads_back_exactly_what_was_written(void)
{
    uint32 seq;
    uint32 ticks;

    g_imuEdge.ticks = 98590u;
    g_imuEdge.seq   = 1u;

    ImuEdge_snapshot(&seq, &ticks);

    TEST_ASSERT_EQUAL_UINT32(1u, seq);
    TEST_ASSERT_EQUAL_UINT32(98590u, ticks);
}

/* seq is the caller's ONLY signal that a new edge arrived (NavTask.c compares
 * it against its own last-consumed copy) -- it must track every write, not
 * just the first. */
void test_snapshot_tracks_successive_edges(void)
{
    uint32 seq;
    uint32 ticks;
    uint16 i;

    for (i = 1u; i <= 5u; i++)
    {
        g_imuEdge.ticks = (uint32)i * 98590u;
        g_imuEdge.seq   = i;

        ImuEdge_snapshot(&seq, &ticks);

        TEST_ASSERT_EQUAL_UINT32(i, seq);
        TEST_ASSERT_EQUAL_UINT32((uint32)i * 98590u, ticks);
    }
}

/* ImuEdge_snapshot() must never write g_imuEdge -- the reader keeps its own
 * "last consumed" state entirely on its side (NavTask.c), same rule
 * NavState_get() follows. A snapshot call must therefore be idempotent: two
 * consecutive calls with no write in between return the same values. */
void test_snapshot_does_not_mutate_the_shared_object(void)
{
    uint32 seqA;
    uint32 ticksA;
    uint32 seqB;
    uint32 ticksB;

    g_imuEdge.ticks = 555u;
    g_imuEdge.seq   = 7u;

    ImuEdge_snapshot(&seqA, &ticksA);
    ImuEdge_snapshot(&seqB, &ticksB);

    TEST_ASSERT_EQUAL_UINT32(seqA, seqB);
    TEST_ASSERT_EQUAL_UINT32(ticksA, ticksB);
    TEST_ASSERT_EQUAL_UINT32(7u, g_imuEdge.seq);
    TEST_ASSERT_EQUAL_UINT32(555u, g_imuEdge.ticks);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_snapshot_before_any_write_reads_the_zeroed_object);
    RUN_TEST(test_snapshot_reads_back_exactly_what_was_written);
    RUN_TEST(test_snapshot_tracks_successive_edges);
    RUN_TEST(test_snapshot_does_not_mutate_the_shared_object);
    return UNITY_END();
}
