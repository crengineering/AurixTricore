#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/ImuInt.h"

/* docs/IMU_INTERRUPT.md §7 footer: "the binning/statistics arithmetic
 * factors into a pure ImuInt_accumulate(stats*, dt) with no iLLD include --
 * a fake is not even needed." ImuInt.h's own header comment names this file
 * as the host test for that function. The ISR wrapper in ImuInt.c (the
 * volatile globals, the vector table entry) is hardware-only by nature and
 * is not exercised here -- only the pure state machine is. */

static ImuInt_State s_state;

void setUp(void)
{
    s_state.dtCount      = 0u;
    s_state.dtMin        = 0xFFFFFFFFu;
    s_state.dtMax        = 0u;
    s_state.dtSum        = 0u;
    s_state.histBase     = 0u;
    s_state.histCentered = FALSE;
}

void tearDown(void)
{
}

/* First interval: min/max/sum/count all track it, histogram stays
 * uncentred (fewer than IMUINT_HIST_CENTER_EDGE samples so far). */
void test_first_sample_updates_min_max_sum_but_not_hist(void)
{
    ImuInt_BinResult r = ImuInt_accumulate(&s_state, 100000u);

    TEST_ASSERT_EQUAL_UINT32(1u, s_state.dtCount);
    TEST_ASSERT_EQUAL_UINT32(100000u, s_state.dtMin);
    TEST_ASSERT_EQUAL_UINT32(100000u, s_state.dtMax);
    TEST_ASSERT_EQUAL_UINT32(100000u, (uint32)s_state.dtSum);
    TEST_ASSERT_EQUAL(FALSE, s_state.histCentered);
    TEST_ASSERT_EQUAL(IMUINT_BIN_NONE, r.kind);
    TEST_ASSERT_EQUAL(FALSE, r.justCentered);
}

/* min/max track the true extremes across a mixed run, and the sum is a
 * plain running total -- the mean the host computes from it depends on
 * this being exact, not merely close. */
void test_min_max_sum_track_across_varied_samples(void)
{
    uint32 i;
    static const uint32 dt[5] = { 100000u, 99000u, 101500u, 98000u, 100200u };
    uint64 expectSum = 0u;

    for (i = 0u; i < 5u; i++)
    {
        (void)ImuInt_accumulate(&s_state, dt[i]);
        expectSum += (uint64)dt[i];
    }

    TEST_ASSERT_EQUAL_UINT32(5u, s_state.dtCount);
    TEST_ASSERT_EQUAL_UINT32(98000u, s_state.dtMin);
    TEST_ASSERT_EQUAL_UINT32(101500u, s_state.dtMax);
    TEST_ASSERT_TRUE(expectSum == s_state.dtSum);
}

/* The histogram centres exactly once, on the IMUINT_HIST_CENTER_EDGE-th
 * accumulated interval, at dtMin - (margin bins * bin width) -- and
 * justCentered fires TRUE only on that one call. */
void test_histogram_centres_once_on_the_configured_edge(void)
{
    uint32 i;
    ImuInt_BinResult r = { IMUINT_BIN_NONE, 0u, FALSE };
    const uint32 margin = IMUINT_HIST_MARGIN_BINS * IMUINT_HIST_BIN_TICKS;

    /* Feed a low outlier first so dtMin is known and distinct from the
     * steady-state value that follows. Stop one short of the centring edge
     * so the FALSE check below never collides with the centring call itself. */
    (void)ImuInt_accumulate(&s_state, 90000u);
    for (i = 2u; i < IMUINT_HIST_CENTER_EDGE; i++)
    {
        r = ImuInt_accumulate(&s_state, 100000u);
        TEST_ASSERT_EQUAL(FALSE, r.justCentered);
    }
    TEST_ASSERT_EQUAL_UINT32(IMUINT_HIST_CENTER_EDGE - 1u, s_state.dtCount);

    /* This is the IMUINT_HIST_CENTER_EDGE-th interval -- the one call that
     * must centre the window. */
    r = ImuInt_accumulate(&s_state, 100000u);
    TEST_ASSERT_EQUAL_UINT32(IMUINT_HIST_CENTER_EDGE, s_state.dtCount);
    TEST_ASSERT_EQUAL(TRUE, s_state.histCentered);
    TEST_ASSERT_EQUAL(TRUE, r.justCentered);
    TEST_ASSERT_EQUAL_UINT32(90000u - margin, s_state.histBase);

    /* The very next interval must NOT re-centre or re-signal justCentered. */
    r = ImuInt_accumulate(&s_state, 100000u);
    TEST_ASSERT_EQUAL(FALSE, r.justCentered);
}

/* dtMin small enough that dtMin - margin would underflow uint32: the
 * centring clamps to 0 rather than wrapping around to a huge base. */
void test_histogram_centre_clamps_at_zero_when_min_is_tiny(void)
{
    uint32 i;
    ImuInt_BinResult r = { IMUINT_BIN_NONE, 0u, FALSE };

    (void)ImuInt_accumulate(&s_state, 50u);   /* smaller than the margin */
    for (i = 1u; i < IMUINT_HIST_CENTER_EDGE; i++)
    {
        r = ImuInt_accumulate(&s_state, 100000u);
    }

    TEST_ASSERT_EQUAL(TRUE, r.justCentered);
    TEST_ASSERT_EQUAL_UINT32(0u, s_state.histBase);
}

/* Once centred, a dt inside the window lands in the right bin index. */
void test_dt_inside_window_classifies_to_the_right_bin(void)
{
    uint32 i;
    ImuInt_BinResult r;

    for (i = 0u; i < IMUINT_HIST_CENTER_EDGE; i++)
    {
        (void)ImuInt_accumulate(&s_state, 100000u);
    }
    /* histBase = 100000 - 16*100 = 98400. A dt of 98400 + 250 = 98650 falls
     * in bin (98650-98400)/100 = 2. */
    r = ImuInt_accumulate(&s_state, 98650u);
    TEST_ASSERT_EQUAL(IMUINT_BIN_INDEX, r.kind);
    TEST_ASSERT_EQUAL_UINT32(2u, r.index);
}

/* A dt below the centred window is UNDER, not a wrapped/garbage index. */
void test_dt_below_window_classifies_under(void)
{
    uint32 i;
    ImuInt_BinResult r;

    for (i = 0u; i < IMUINT_HIST_CENTER_EDGE; i++)
    {
        (void)ImuInt_accumulate(&s_state, 100000u);
    }
    r = ImuInt_accumulate(&s_state, 1u);   /* far below histBase */
    TEST_ASSERT_EQUAL(IMUINT_BIN_UNDER, r.kind);
}

/* A dt at/above the top edge of the centred window is OVER. */
void test_dt_above_window_classifies_over(void)
{
    uint32 i;
    ImuInt_BinResult r;
    const uint32 binWidth = IMUINT_HIST_BINS * IMUINT_HIST_BIN_TICKS;

    for (i = 0u; i < IMUINT_HIST_CENTER_EDGE; i++)
    {
        (void)ImuInt_accumulate(&s_state, 100000u);
    }
    /* histBase = 98400; the window covers [98400, 98400+3200). Anything at
     * or past 98400+3200 must be OVER, including a dropped-edge-sized 2 ms
     * outlier. */
    r = ImuInt_accumulate(&s_state, s_state.histBase + binWidth);
    TEST_ASSERT_EQUAL(IMUINT_BIN_OVER, r.kind);

    r = ImuInt_accumulate(&s_state, 2000000u);   /* ~2 ms, a dropped edge */
    TEST_ASSERT_EQUAL(IMUINT_BIN_OVER, r.kind);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_first_sample_updates_min_max_sum_but_not_hist);
    RUN_TEST(test_min_max_sum_track_across_varied_samples);
    RUN_TEST(test_histogram_centres_once_on_the_configured_edge);
    RUN_TEST(test_histogram_centre_clamps_at_zero_when_min_is_tiny);
    RUN_TEST(test_dt_inside_window_classifies_to_the_right_bin);
    RUN_TEST(test_dt_below_window_classifies_under);
    RUN_TEST(test_dt_above_window_classifies_over);
    return UNITY_END();
}
