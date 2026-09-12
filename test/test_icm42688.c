#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "fakes/FakeSpi.h"
#include "../src/bsw/Icm42688.h"
#include <math.h>
#include <stdio.h>

/* SYS1-001 strand B task 5 (B4b, evidence 952275AD99001303): 1014.2 Hz is the
 * measured DRDY rate (docs/IMU_INTERRUPT.md) -- the natural per-tick dt for
 * driving Icm42688_reportPlausibility()/Icm42688_verifyPresence() the way
 * NavTask.c actually would. */
#define B5_DT   (1.0f / 1014.2f)

void setUp(void) { FakeSpi_reset(); }
void tearDown(void) {}

void test_nominal_plausible(void)
{
    /* ~1 g on Z, at rest. */
    Icm42688_Sample s = { { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, 20.0f };
    float32 liveness = 0.0f;
    TEST_ASSERT_TRUE(Icm42688_plausible(&s, &liveness));
    TEST_ASSERT_EQUAL_FLOAT(21.0f, liveness);
}

void test_accel_band_edges(void)
{
    float32 liveness = 0.0f;
    /* |a|^2 comfortably below 0.0025 (0.03 g) -> excluded. Not tested bit-exact
     * on the boundary: 0.05f*0.05f is not guaranteed to round to exactly
     * 0.0025f, which would make the test assert on float rounding rather than
     * on the band logic. */
    Icm42688_Sample lo = { { 0.03f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 20.0f };
    TEST_ASSERT_FALSE(Icm42688_plausible(&lo, &liveness));

    Icm42688_Sample loIn = { { 0.1f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 20.0f };
    TEST_ASSERT_TRUE(Icm42688_plausible(&loIn, &liveness));

    /* |a|^2 comfortably above 289 (17.5 g) -> excluded; comfortably below (16 g)
     * -> included. */
    Icm42688_Sample hi = { { 17.5f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 20.0f };
    TEST_ASSERT_FALSE(Icm42688_plausible(&hi, &liveness));

    Icm42688_Sample hiIn = { { 16.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 20.0f };
    TEST_ASSERT_TRUE(Icm42688_plausible(&hiIn, &liveness));
}

void test_temperature_band_edges(void)
{
    float32 liveness = 0.0f;
    Icm42688_Sample cold = { { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, -40.0f };
    TEST_ASSERT_FALSE(Icm42688_plausible(&cold, &liveness));

    Icm42688_Sample hot = { { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, 105.0f };
    TEST_ASSERT_FALSE(Icm42688_plausible(&hot, &liveness));
}

/* NaN on any axis, in either sensor, must fail the band -- not pass it. */
void test_nan_is_not_plausible(void)
{
    Icm42688_Sample s = { { NAN, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, 20.0f };
    float32 liveness = 0.0f;
    TEST_ASSERT_FALSE(Icm42688_plausible(&s, &liveness));
    TEST_ASSERT_TRUE(isnan(liveness));
}

/* ==========================================================================
 * SYS1-001 strand B task 5 (B4b) -- IMU presence detection, evidence
 * 952275AD99001303: a well-formed but FROZEN frame (constant 0x8000/axis =
 * -16g/-2000dps) never trips Icm42688_read()'s own SPI-failure path, so
 * presence never dropped and the WHO_AM_I recovery probe was never reached.
 * ======================================================================== */

/** A level, 1 g plausible burst -- temp(0,0)=0 raw -> 25.0 degC, accel Z
 *  raw = -32768*... no: ACCEL_SCALE = 16/32768, so raw -8192 -> -4 g is too
 *  much; use a small raw count for a clean ~1 g reading: 32768/16 = 2048
 *  counts per g, so -2048 on Z (big-endian 0xF8,0x00) gives -1.0 g exactly. */
static void b5_plausibleBurst(uint8 out[14])
{
    uint8 i;
    for (i = 0u; i < 14u; i++) { out[i] = 0u; }
    out[6] = 0xF8u; out[7] = 0x00u;   /* accel Z = -2048 counts = -1.0 g */
}

static void b5_stuckBurst(uint8 out[14])
{
    uint8 i;
    for (i = 0u; i < 14u; i++) { out[i] = (i % 2u == 0u) ? 0x80u : 0x00u; }
}

void test_stuck_frame_with_drdy_alive_drops_presence_within_100ms(void)
{
    /* "DRDY alive" == this test drives Icm42688_read()/Icm42688_reportPlausibility()
     * every tick, same as NavTask_step would on every genuine new edge --
     * trigger 1 (verifyPresence, the SILENT path) is never involved here.
     *
     * Task 14 (SWE1-FW-009) update: a constant 0x8000-per-axis burst is now
     * rejected a layer earlier, inside Icm42688_read() itself (every one of
     * the six words IS the sentinel), so it no longer "still answers SPI"
     * the way it did when this test and evidence row 952275AD99001303 were
     * first written -- see test_all_six_axes_sentinel_still_drops_presence_
     * after_100ms for that path in detail. What this test still pins down,
     * unchanged, is the OUTCOME evidence row 952275AD99001303 needed fixed:
     * a sensor that keeps answering (whether "answering" means a scaled
     * sample, pre-task-14, or a rejected burst, post-task-14) but never
     * produces a plausible one still drops presence within 100 ms -- the
     * defect was presence NEVER dropping, not which layer catches it. */
    uint8    stuck[14];
    boolean  present;
    int      i;
    const int expectedTicks = (int)(0.1f / B5_DT + 0.5f);   /* ~101 */

    TEST_ASSERT_TRUE(Icm42688_init());
    b5_stuckBurst(stuck);
    FakeSpi_setBurst(stuck);

    present = TRUE;
    for (i = 1; (i <= (expectedTicks + 5)) && (present != FALSE); ++i)
    {
        Icm42688_Sample sample = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 0.0f };
        float32         liveness = 0.0f;
        boolean         plausible;
        boolean         ok;

        ok = Icm42688_read(&sample);
        TEST_ASSERT_FALSE_MESSAGE(ok,
            "a constant 0x8000 triple is now rejected by task 14's sentinel check");
        plausible = Icm42688_plausible(&sample, &liveness);
        TEST_ASSERT_FALSE_MESSAGE(plausible, "the unpopulated sample must fail the plausibility band");
        present = Icm42688_reportPlausibility(plausible, B5_DT);
        if (present == FALSE)
        {
            char msg[96];
            (void)snprintf(msg, sizeof msg, "dropped at tick %d, expected %d +/- 1", i, expectedTicks);
            TEST_ASSERT_TRUE_MESSAGE((i >= expectedTicks - 1) && (i <= expectedTicks + 1), msg);
        }
    }
    TEST_ASSERT_FALSE_MESSAGE(present, "100 ms of a continuously implausible sample must drop presence");
}

void test_silent_drdy_with_good_whoami_keeps_presence(void)
{
    /* "silent" == this test never calls Icm42688_read() at all -- only
     * NavTask.c's own no-edge path would call verifyPresence(), and only
     * that trigger is exercised here. */
    boolean present = TRUE;
    int     i;

    TEST_ASSERT_TRUE(Icm42688_init());   /* WHO_AM_I = 0x47 (FakeSpi default) */

    for (i = 0; i < 2000; ++i)   /* ~2 s -- several 5 Hz probe periods */
    {
        present = Icm42688_verifyPresence(B5_DT);
        TEST_ASSERT_TRUE_MESSAGE(present, "a correctly-answering WHO_AM_I must never drop presence");
    }
}

void test_silent_drdy_with_failing_whoami_drops_within_200ms(void)
{
    boolean present;

    TEST_ASSERT_TRUE(Icm42688_init());
    FakeSpi_setBusOk(FALSE);            /* silent AND now unreachable */

    present = Icm42688_verifyPresence(B5_DT);
    TEST_ASSERT_FALSE_MESSAGE(present,
        "the first probe after init must fire immediately (no extra startup "
        "delay), so a failing WHO_AM_I drops presence well inside 200 ms");
}

void test_single_overrange_sample_does_not_drop_presence(void)
{
    uint8   good[14];
    uint8   over[14];
    boolean present = TRUE;
    int     i;

    b5_plausibleBurst(good);
    /* accel X = +32000 counts ~= 15.6 g on its own; combined with the other
     * plausible axes this still trips the |a| < 17 g band once. */
    for (i = 0; i < 14; ++i) { over[i] = good[i]; }
    over[2] = 0x7Du; over[3] = 0x00u;

    TEST_ASSERT_TRUE(Icm42688_init());
    FakeSpi_setBurst(good);

    for (i = 0; i < 500; ++i)
    {
        Icm42688_Sample sample;
        float32         liveness = 0.0f;
        boolean         plausible;

        if (i == 250) { FakeSpi_setBurst(over); }
        if (i == 251) { FakeSpi_setBurst(good); }

        TEST_ASSERT_TRUE(Icm42688_read(&sample));
        plausible = Icm42688_plausible(&sample, &liveness);
        present   = Icm42688_reportPlausibility(plausible, B5_DT);
        TEST_ASSERT_TRUE_MESSAGE(present, "a single implausible sample amid a healthy stream must not drop presence");
    }
}

void test_replug_reruns_init_exactly_once(void)
{
    Icm42688_Sample sample;
    int             i;

    TEST_ASSERT_TRUE(Icm42688_init());
    TEST_ASSERT_EQUAL_UINT32(1u, FakeSpi_softResetWriteCount());

    /* Unplug: the bus itself now fails outright (the driver's EXISTING
     * read-fail path, unchanged by task 5). Run through more than two full
     * ICM42688_RECOVERY_PERIOD (50-call) cycles -- both recovery attempts
     * fail because the bus is down, so Icm42688_init() must not run again. */
    FakeSpi_setBusOk(FALSE);
    for (i = 0; i < 120; ++i)
    {
        (void)Icm42688_read(&sample);
    }
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1u, FakeSpi_softResetWriteCount(),
        "a bus that keeps failing must never re-run init");

    /* Replug: the bus (and WHO_AM_I) answer correctly again. The recovery
     * probe fires again within one more 50-call window. */
    FakeSpi_setBusOk(TRUE);
    for (i = 0; i < 50; ++i)
    {
        (void)Icm42688_read(&sample);
    }
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(2u, FakeSpi_softResetWriteCount(),
        "a replugged, correctly-answering device must re-run init exactly once");

    /* And exactly once -- continuing to read a healthy, present device must
     * never call init again. */
    for (i = 0; i < 200; ++i)
    {
        (void)Icm42688_read(&sample);
    }
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(2u, FakeSpi_softResetWriteCount(),
        "a device that stays present must never be re-initialised");
}

/* ==========================================================================
 * SYS1-001 strand B task 14 (SWE1-FW-009): sentinel-word rejection in
 * Icm42688_read(), on the raw be16 words, before scaling.
 * ======================================================================== */

/** A plausible, level burst with ONE axis (gyro Z, offset 12) forced to the
 *  0x8000 sentinel -- everything else a normal, in-band reading. */
static void b14_singleSentinelBurst(uint8 out[14])
{
    b5_plausibleBurst(out);
    out[12] = 0x80u; out[13] = 0x00u;
}

/** Same, but the word is 0x8001 -- one LSB off the sentinel, and per the
 *  acceptance a LEGITIMATE (if near-full-scale) reading that must still be
 *  accepted. */
static void b14_almostSentinelBurst(uint8 out[14])
{
    b5_plausibleBurst(out);
    out[12] = 0x80u; out[13] = 0x01u;
}

void test_single_sentinel_word_is_rejected_but_presence_unchanged(void)
{
    uint8           burst[14];
    Icm42688_Sample sample;
    uint32          sentinelBefore;
    int             i;

    b14_singleSentinelBurst(burst);
    TEST_ASSERT_TRUE(Icm42688_init());
    FakeSpi_setBurst(burst);
    sentinelBefore = g_dbgImuSentinelWords;

    TEST_ASSERT_FALSE_MESSAGE(Icm42688_read(&sample),
        "a burst with any be16 word == 0x8000 must be reported invalid");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(sentinelBefore + 1u, g_dbgImuSentinelWords,
        "the sentinel counter must increment exactly once per rejected burst");

    /* Presence itself must be UNCHANGED -- not the read-fail/recovery path.
     * Prove it the same way test_single_overrange_sample_does_not_drop_
     * presence does: go straight back to a healthy burst and confirm this
     * next read succeeds immediately, with no re-init in between (the
     * recovery path would have needed ICM42688_RECOVERY_PERIOD calls while
     * "absent" first). */
    {
        uint8 good[14];
        b5_plausibleBurst(good);
        FakeSpi_setBurst(good);
        TEST_ASSERT_TRUE_MESSAGE(Icm42688_read(&sample),
            "presence must stay TRUE across a single sentinel word -- the "
            "very next healthy burst must read straight through");
    }

    /* And it must not have re-run init: exactly one soft-reset write, from
     * the Icm42688_init() call above. */
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1u, FakeSpi_softResetWriteCount(),
        "a single sentinel word must never trigger a re-init");

    /* Sanity: a run of single-sentinel/good pairs never accumulates toward
     * a drop either, matching "presence unchanged on a single word". */
    {
        uint8 good[14];
        b5_plausibleBurst(good);
        for (i = 0; i < 20; ++i)
        {
            FakeSpi_setBurst(burst);
            (void)Icm42688_read(&sample);
            FakeSpi_setBurst(good);
            TEST_ASSERT_TRUE(Icm42688_read(&sample));
        }
    }
}

void test_0x8001_word_one_lsb_off_sentinel_is_accepted(void)
{
    /* -1999.9 dps / -15.999 g -- deliberately NOT the sentinel, and per the
     * acceptance a legitimate near-full-scale reading. */
    uint8           burst[14];
    Icm42688_Sample sample;
    uint32          sentinelBefore;

    b14_almostSentinelBurst(burst);
    TEST_ASSERT_TRUE(Icm42688_init());
    FakeSpi_setBurst(burst);
    sentinelBefore = g_dbgImuSentinelWords;

    TEST_ASSERT_TRUE_MESSAGE(Icm42688_read(&sample),
        "0x8001 is one LSB off the sentinel and must be accepted");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(sentinelBefore, g_dbgImuSentinelWords,
        "0x8001 must never count as a sentinel word");

    {
        char msg[96];
        (void)snprintf(msg, sizeof msg, "gyro Z = %g dps, expected ~-1999.9",
            (double)sample.gyro[2]);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, -1999.94f, sample.gyro[2], msg);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nominal_plausible);
    RUN_TEST(test_accel_band_edges);
    RUN_TEST(test_temperature_band_edges);
    RUN_TEST(test_nan_is_not_plausible);
    RUN_TEST(test_stuck_frame_with_drdy_alive_drops_presence_within_100ms);
    RUN_TEST(test_silent_drdy_with_good_whoami_keeps_presence);
    RUN_TEST(test_silent_drdy_with_failing_whoami_drops_within_200ms);
    RUN_TEST(test_single_overrange_sample_does_not_drop_presence);
    RUN_TEST(test_replug_reruns_init_exactly_once);
    RUN_TEST(test_single_sentinel_word_is_rejected_but_presence_unchanged);
    RUN_TEST(test_0x8001_word_one_lsb_off_sentinel_is_accepted);
    return UNITY_END();
}
