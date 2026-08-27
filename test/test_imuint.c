#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/ImuInt.h"

/* docs/IMU_INTERRUPT.md SS7 footer: "the binning/statistics arithmetic
 * factors into a pure ImuInt_accumulate(stats*, dt) with no iLLD include --
 * a fake is not even needed." ImuInt.h's own header comment names this file
 * as the host test for that function. The ISR wrapper in ImuInt.c (the
 * volatile globals, the vector table entry) is hardware-only by nature and
 * is not exercised here -- only the pure state machine is.
 *
 * Rewritten after the first hardware run found two bugs the original
 * version of this file did not catch (docs/IMU_INTERRUPT.md SS5.6):
 *   1. IfxScuEru_enableAutoClear() double-counted every edge (fixed in
 *      ImuInt.c, not this pure function -- unrelated to this file).
 *   2. Centring on a running dtMin let one boot-transient outlier corrupt
 *      the histogram window for the whole run. Fixed here: the first
 *      IMUINT_WARMUP_EDGES intervals are discarded outright, and centring
 *      then happens once, immediately, on the first post-warm-up sample's
 *      own value -- see ImuInt_accumulate()'s comment for why. */

static ImuInt_State s_state;

void setUp(void)
{
    s_state.warmupRemaining = IMUINT_WARMUP_EDGES;
    s_state.dtCount         = 0u;
    s_state.dtMin           = 0xFFFFFFFFu;
    s_state.dtMax           = 0u;
    s_state.dtSum           = 0u;
    s_state.histBase        = 0u;
    s_state.histCentered    = FALSE;
}

void tearDown(void)
{
}

/* Feeds exactly IMUINT_WARMUP_EDGES intervals of the given value, to clear
 * warm-up without centring on anything -- the shared setup step most tests
 * below need. */
static void feedWarmup(uint32 dt)
{
    uint32 i;
    for (i = 0u; i < IMUINT_WARMUP_EDGES; i++)
    {
        (void)ImuInt_accumulate(&s_state, dt);
    }
}

/* Every warm-up interval is discarded outright: no min/max/sum/count
 * update, histogram never centres, every result is IMUINT_BIN_NONE -- even
 * when the warm-up samples themselves are wild outliers (a 20 ms boot
 * transient, say). This is the fix: the old behaviour let exactly this kind
 * of sample corrupt the whole run's window (docs/IMU_INTERRUPT.md SS5.6). */
void test_warmup_intervals_are_discarded_entirely_even_if_extreme(void)
{
    uint32 i;
    ImuInt_BinResult r = { IMUINT_BIN_INDEX, 1u, TRUE };  /* poison */

    r = ImuInt_accumulate(&s_state, 74131u);     /* ~741 us, the real short outlier */
    TEST_ASSERT_EQUAL(IMUINT_BIN_NONE, r.kind);
    TEST_ASSERT_EQUAL(FALSE, r.justCentered);

    for (i = 1u; i < IMUINT_WARMUP_EDGES; i++)
    {
        r = ImuInt_accumulate(&s_state, 2039635u); /* ~20.4 ms, the real long outlier */
    }

    TEST_ASSERT_EQUAL(IMUINT_BIN_NONE, r.kind);
    TEST_ASSERT_EQUAL_UINT32(0u, s_state.dtCount);
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, s_state.dtMin);
    TEST_ASSERT_EQUAL_UINT32(0u, s_state.dtMax);
    TEST_ASSERT_TRUE(0u == s_state.dtSum);
    TEST_ASSERT_EQUAL(FALSE, s_state.histCentered);
}

/* The first POST-warm-up interval is the one that starts counting AND
 * centres the histogram, immediately, on its own value -- not a running
 * dtMin gathered over many samples. */
void test_first_post_warmup_sample_starts_counting_and_centres(void)
{
    ImuInt_BinResult r;
    const uint32 margin = IMUINT_HIST_MARGIN_BINS * IMUINT_HIST_BIN_TICKS;

    feedWarmup(2039635u);   /* boot-transient noise, discarded */
    r = ImuInt_accumulate(&s_state, 98590u);   /* first real sample */

    TEST_ASSERT_EQUAL_UINT32(1u, s_state.dtCount);
    TEST_ASSERT_EQUAL_UINT32(98590u, s_state.dtMin);
    TEST_ASSERT_EQUAL_UINT32(98590u, s_state.dtMax);
    TEST_ASSERT_TRUE((uint64)98590u == s_state.dtSum);
    TEST_ASSERT_EQUAL(TRUE, s_state.histCentered);
    TEST_ASSERT_EQUAL(TRUE, r.justCentered);
    TEST_ASSERT_EQUAL_UINT32(98590u - margin, s_state.histBase);

    /* The next interval must not re-centre or re-signal justCentered. */
    r = ImuInt_accumulate(&s_state, 98600u);
    TEST_ASSERT_EQUAL(FALSE, r.justCentered);
}

/* min/max/sum track the true extremes across a mixed POST-warm-up run, and
 * the sum is a plain running total -- the mean the host computes from it
 * depends on this being exact, not merely close. Values chosen close
 * together so none of them individually falls outside the centred window
 * (this test is about min/max/sum, not bin classification). */
void test_min_max_sum_track_across_varied_post_warmup_samples(void)
{
    uint32 i;
    static const uint32 dt[5] = { 98590u, 98550u, 98630u, 98520u, 98610u };
    uint64 expectSum = 0u;

    feedWarmup(98590u);
    for (i = 0u; i < 5u; i++)
    {
        (void)ImuInt_accumulate(&s_state, dt[i]);
        expectSum += (uint64)dt[i];
    }

    TEST_ASSERT_EQUAL_UINT32(5u, s_state.dtCount);
    TEST_ASSERT_EQUAL_UINT32(98520u, s_state.dtMin);
    TEST_ASSERT_EQUAL_UINT32(98630u, s_state.dtMax);
    TEST_ASSERT_TRUE(expectSum == s_state.dtSum);
}

/* Anchor value small enough that anchor - margin would underflow uint32:
 * the centring clamps to 0 rather than wrapping around to a huge base. */
void test_histogram_centre_clamps_at_zero_when_anchor_is_tiny(void)
{
    ImuInt_BinResult r;

    feedWarmup(100000u);
    r = ImuInt_accumulate(&s_state, 50u);   /* smaller than the margin */

    TEST_ASSERT_EQUAL(TRUE, r.justCentered);
    TEST_ASSERT_EQUAL_UINT32(0u, s_state.histBase);
}

/* Once centred, a dt inside the window lands in the right bin index. */
void test_dt_inside_window_classifies_to_the_right_bin(void)
{
    ImuInt_BinResult r;

    feedWarmup(100000u);
    (void)ImuInt_accumulate(&s_state, 100000u);  /* first real sample: centres */
    /* histBase = 100000 - 16*100 = 98400. A dt of 98400 + 250 = 98650 falls
     * in bin (98650-98400)/100 = 2. */
    r = ImuInt_accumulate(&s_state, 98650u);
    TEST_ASSERT_EQUAL(IMUINT_BIN_INDEX, r.kind);
    TEST_ASSERT_EQUAL_UINT32(2u, r.index);
}

/* A dt below the centred window is UNDER, not a wrapped/garbage index. */
void test_dt_below_window_classifies_under(void)
{
    ImuInt_BinResult r;

    feedWarmup(100000u);
    (void)ImuInt_accumulate(&s_state, 100000u);
    r = ImuInt_accumulate(&s_state, 1u);   /* far below histBase */
    TEST_ASSERT_EQUAL(IMUINT_BIN_UNDER, r.kind);
}

/* A dt at/above the top edge of the centred window is OVER. */
void test_dt_above_window_classifies_over(void)
{
    ImuInt_BinResult r;
    const uint32 binWidth = IMUINT_HIST_BINS * IMUINT_HIST_BIN_TICKS;

    feedWarmup(100000u);
    (void)ImuInt_accumulate(&s_state, 100000u);
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
    RUN_TEST(test_warmup_intervals_are_discarded_entirely_even_if_extreme);
    RUN_TEST(test_first_post_warmup_sample_starts_counting_and_centres);
    RUN_TEST(test_min_max_sum_track_across_varied_post_warmup_samples);
    RUN_TEST(test_histogram_centre_clamps_at_zero_when_anchor_is_tiny);
    RUN_TEST(test_dt_inside_window_classifies_to_the_right_bin);
    RUN_TEST(test_dt_below_window_classifies_under);
    RUN_TEST(test_dt_above_window_classifies_over);
    return UNITY_END();
}
