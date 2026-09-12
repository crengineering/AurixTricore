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
     * trigger 1 (verifyPresence, the SILENT path) is never involved here. */
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
        Icm42688_Sample sample;
        float32         liveness = 0.0f;
        boolean         plausible;
        boolean         ok;

        ok = Icm42688_read(&sample);
        TEST_ASSERT_TRUE_MESSAGE(ok, "the stuck frame must still answer SPI (that is the whole defect)");
        plausible = Icm42688_plausible(&sample, &liveness);
        TEST_ASSERT_FALSE_MESSAGE(plausible, "a constant 0x8000 triple must fail the plausibility band");
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
    return UNITY_END();
}
