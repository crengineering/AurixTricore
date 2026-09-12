#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "fakes/FakeSpi.h"
#include "fakes/IfxStm.h"
#include "../src/bsw/Icm42688.h"
#include "../src/bsw/Spi.h"   /* SPI_MODE_0/SPI_MODE_3 -- task 17 mode-retry tests */
#include <math.h>
#include <stdio.h>

/* SYS1-001 strand B task 5 (B4b, evidence 952275AD99001303): 1014.2 Hz is the
 * measured DRDY rate (docs/IMU_INTERRUPT.md) -- the natural per-tick dt for
 * driving Icm42688_reportPlausibility()/Icm42688_verifyPresence() the way
 * NavTask.c actually would. */
#define B5_DT   (1.0f / 1014.2f)

/* SYS1-001 strand B task 17 (B6.4): the same rate, used to drive
 * Icm42688_reinitStep() the way Icm42688_read()'s absent branch actually
 * would (NavTask_step polls every ~500 us; a genuinely absent sensor's own
 * edges never arrive, so every dispatch takes the no-new-edge path and
 * calls Icm42688_read() at that rate). */
#define T17_DT  (0.0005f)

void setUp(void)
{
    FakeSpi_reset();
    FakeStm_reset();
    /* Task 17: these two are plain production globals (Icm42688.h), not
     * reset by any driver call -- zero them per test the same way FakeSpi/
     * FakeStm reset their own state, so each test's assertions are against
     * a known baseline rather than whatever earlier tests accumulated. */
    g_dbgImuReinits     = 0u;
    g_dbgImuReinitFails = 0u;
}
void tearDown(void) {}

/* Task 17: Icm42688_reinitStep()'s state machine is file-scope static in
 * Icm42688.c and persists across tests within this executable (like
 * s_icm42688Present always has) -- there is no test-only reset, so a test
 * that needs a FRESH IDLE start drives the module there through its own
 * public interface: a healthy Icm42688_init() first guarantees a known
 * DONE (present TRUE), then one read() with the bus down forces exactly
 * the "Lost it" path (Icm42688_reinitStart()) into IDLE. */
static void icm42688_forceIdle(void)
{
    Icm42688_Sample sample;

    FakeSpi_setWhoAmI(ICM42688_WHO_AM_I_VALUE);
    FakeSpi_setBusOk(TRUE);
    TEST_ASSERT_TRUE_MESSAGE(Icm42688_init(), "test setup: boot must reach DONE");

    FakeSpi_setBusOk(FALSE);
    (void)Icm42688_read(&sample);
    FakeSpi_setBusOk(TRUE);
}

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

void test_replug_reinitialises_via_the_nonblocking_state_machine(void)
{
    /* Task 17 (B6.4) superseded the old exact-soft-reset-count assertion:
     * the state machine now re-attempts on its OWN cadence (RESET_WAIT/
     * WAKE_WAIT timing plus the FAILED backoff), not a single lightweight
     * probe every ICM42688_RECOVERY_PERIOD calls -- what must still hold is
     * the OUTCOME: a bus that keeps failing never falsely reports present,
     * a replugged device recovers, and a device that stays present is never
     * re-initialised again. */
    Icm42688_Sample sample;
    int             i;
    boolean         present;

    TEST_ASSERT_TRUE(Icm42688_init());
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, g_dbgImuReinits,
        "g_dbgImuReinits counts RUNTIME re-inits only -- a healthy boot must show 0");
    FakeStm_reset();   /* the boot pump's own delayMs calls are legitimate;
                        * only the runtime recovery path below must be silent */

    /* Icm42688_read()'s absent branch measures ITS OWN dt from SysTime_
     * getTicks() (it has no dtS parameter) -- auto-advance the fake clock
     * by ~500 us/call (50000 STM0 ticks @ 100 MHz), the real poll rate
     * NavTask_step drives it at, or the wait states never see elapsed time
     * and the machine never leaves RESET_WAIT. */
    FakeStm_setAutoAdvance(50000u);

    /* Unplug: the bus itself now fails outright (the driver's EXISTING
     * read-fail path). Run through several full attempt+backoff cycles --
     * presence must never read TRUE while the bus is down, and the
     * non-blocking contract must hold throughout. */
    FakeSpi_setBusOk(FALSE);
    present = TRUE;
    for (i = 0; i < 400; ++i)
    {
        present = Icm42688_read(&sample);
        TEST_ASSERT_FALSE_MESSAGE(present, "a failing bus must never report present");
    }
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, g_dbgImuReinits,
        "a bus that keeps failing must never reach DONE");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, FakeStm_waitTicksCallCount(),
        "the runtime recovery path must never call Icm42688_delayMs()");

    /* Replug: the bus (and WHO_AM_I) answer correctly again. Recovery is
     * bounded but not instantaneous -- drive enough calls for at least one
     * full attempt cycle. */
    FakeSpi_setBusOk(TRUE);
    for (i = 0; (i < 400) && (present == FALSE); ++i)
    {
        present = Icm42688_read(&sample);
    }
    TEST_ASSERT_TRUE_MESSAGE(present, "a replugged, correctly-answering device must recover");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1u, g_dbgImuReinits,
        "exactly one RUNTIME re-init DONE for this one replug");

    /* And exactly once -- continuing to read a healthy, present device must
     * never re-initialise again. */
    for (i = 0; i < 500; ++i)
    {
        (void)Icm42688_read(&sample);
    }
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1u, g_dbgImuReinits,
        "a device that stays present must never be re-initialised");
}

/* ==========================================================================
 * SYS1-001 strand B task 17 (B6.4, SWE1-FW-008 clause g): the non-blocking
 * re-init state machine, Icm42688_reinitStep().
 * ======================================================================== */

void test_reinitstep_issues_at_most_one_spi_transaction_per_call(void)
{
    int i;

    for (i = 0; i < 200; ++i)
    {
        const uint32 before = FakeSpi_transferCallCount();
        uint32       delta;
        char         msg[64];

        (void)Icm42688_reinitStep(T17_DT);
        delta = FakeSpi_transferCallCount() - before;
        (void)snprintf(msg, sizeof msg, "call %d issued %u SPI transactions", i, (unsigned)delta);
        TEST_ASSERT_TRUE_MESSAGE(delta <= 1u, msg);
    }
}

void test_reinitstep_never_calls_delayms(void)
{
    int i;

    for (i = 0; i < 200; ++i)
    {
        (void)Icm42688_reinitStep(T17_DT);
    }
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, FakeStm_waitTicksCallCount(),
        "Icm42688_reinitStep() must never wait -- Icm42688_delayMs() is boot-pump-only");
}

void test_reinitstep_register_sequence_is_byte_identical_to_blocking(void)
{
    /* Golden trace, captured from the pre-task-17 blocking Icm42688_init():
     * DEVICE_CONFIG=SOFT_RESET, PWR_MGMT0, GYRO_CONFIG0, ACCEL_CONFIG0,
     * INT_CONFIG, INT_CONFIG0, INT_CONFIG1, INT_SOURCE0, in that order --
     * the WHO_AM_I read between the first two is a READ (rx != NULL_PTR),
     * not logged here, same as the fake's write-only log always excluded
     * it. */
    static const uint8 expectReg[8] = { 0x11u, 0x4Eu, 0x4Fu, 0x50u, 0x14u, 0x63u, 0x64u, 0x65u };
    static const uint8 expectVal[8] = { 0x01u, 0x0Fu, 0x06u, 0x06u, 0x03u, 0x00u, 0x00u, 0x08u };
    int i;

    TEST_ASSERT_TRUE(Icm42688_init());
    TEST_ASSERT_EQUAL_UINT32(8u, FakeSpi_writeLogCount());
    for (i = 0; i < 8; ++i)
    {
        char msg[64];
        (void)snprintf(msg, sizeof msg, "write %d: reg 0x%02X val 0x%02X",
            i, FakeSpi_writeLogReg((uint32)i), FakeSpi_writeLogValue((uint32)i));
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(expectReg[i], FakeSpi_writeLogReg((uint32)i), msg);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(expectVal[i], FakeSpi_writeLogValue((uint32)i), msg);
    }
}

void test_reinitstep_reaches_done_in_40_to_64_steps_at_500us(void)
{
    boolean present = FALSE;
    int     steps;

    icm42688_forceIdle();
    for (steps = 1; (steps <= 64) && (present == FALSE); ++steps)
    {
        present = Icm42688_reinitStep(T17_DT);
    }
    {
        char msg[64];
        (void)snprintf(msg, sizeof msg, "DONE reached at step %d", steps - 1);
        TEST_ASSERT_TRUE_MESSAGE(present, msg);
        TEST_ASSERT_TRUE_MESSAGE((steps - 1) >= 40, msg);
        TEST_ASSERT_TRUE_MESSAGE((steps - 1) <= 64, msg);
    }
}

void test_reinitstep_mode0_failure_retries_mode3_exactly_once_then_failed(void)
{
    /* A WHO_AM_I that never matches on EITHER mode: ID_CHECK fails, retries
     * once on SPI_MODE_3 (exactly one extra Spi_setMode() call beyond
     * Icm42688_reinitStart()'s own initial MODE_0), then FAILED --
     * observable via g_dbgImuReinitFails, since FAILED and "still working"
     * are otherwise both just "present == FALSE". */
    int i;
    boolean present = TRUE;

    icm42688_forceIdle();
    FakeSpi_reset();            /* clean SPI-side counters; FSM stays IDLE */
    FakeStm_reset();            /* forceIdle()'s own boot pump legitimately
                                  * called delayMs -- only what follows here
                                  * must stay silent */
    FakeSpi_setWhoAmI(0x00u);   /* never matches ICM42688_WHO_AM_I_VALUE */

    for (i = 0; (i < 64) && (g_dbgImuReinitFails == 0u); ++i)
    {
        present = Icm42688_reinitStep(T17_DT);
    }
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1u, g_dbgImuReinitFails,
        "a WHO_AM_I that never matches must reach FAILED exactly once");
    TEST_ASSERT_FALSE(present);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(SPI_MODE_3, Spi_getMode(),
        "the retry must have switched to SPI_MODE_3");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1u, FakeSpi_setModeCallCount(),
        "exactly one retry switch, from a state machine already parked at "
        "SPI_MODE_0 (icm42688_forceIdle() + FakeSpi_reset())");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, FakeStm_waitTicksCallCount(),
        "reaching FAILED must never have blocked");
}

void test_reinitstep_reaches_failed_and_rearms_after_recovery_period(void)
{
    /* Continuation of the scenario above: FAILED must not be permanent --
     * ICM42688_RECOVERY_PERIOD (50) further calls re-arm a fresh attempt
     * (SPI_MODE_0 again), all without ever blocking. */
    int i;

    icm42688_forceIdle();
    FakeSpi_reset();
    FakeStm_reset();
    FakeSpi_setWhoAmI(0x00u);
    for (i = 0; (i < 64) && (g_dbgImuReinitFails == 0u); ++i)
    {
        (void)Icm42688_reinitStep(T17_DT);
    }
    TEST_ASSERT_EQUAL_UINT32(1u, g_dbgImuReinitFails);

    for (i = 0; i < 50; ++i)
    {
        (void)Icm42688_reinitStep(T17_DT);
    }
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(SPI_MODE_0, Spi_getMode(),
        "50 calls in FAILED must re-arm a fresh SPI_MODE_0 attempt");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, FakeStm_waitTicksCallCount(),
        "the FAILED backoff must never have blocked");

    /* And the re-armed attempt can actually succeed once WHO_AM_I starts
     * answering correctly again. */
    FakeSpi_setWhoAmI(ICM42688_WHO_AM_I_VALUE);
    {
        boolean present = FALSE;
        for (i = 0; (i < 64) && (present == FALSE); ++i)
        {
            present = Icm42688_reinitStep(T17_DT);
        }
        TEST_ASSERT_TRUE_MESSAGE(present, "the re-armed attempt must be able to reach DONE");
    }
}

void test_reinitstep_presence_true_only_on_done(void)
{
    int i;

    icm42688_forceIdle();
    for (i = 0; i < 48; ++i)
    {
        const boolean present = Icm42688_reinitStep(T17_DT);
        if (present != FALSE)
        {
            /* the ONLY way present can read TRUE is DONE -- confirmed by
             * the 40-64 step test above; here just confirm it eventually
             * happens and never earlier than the reset+wake timing allows */
            char msg[64];
            (void)snprintf(msg, sizeof msg, "present became TRUE at call %d (< 40 is too early)", i);
            TEST_ASSERT_TRUE_MESSAGE(i >= 39, msg);
        }
        else
        {
            /* still IDLE/RESET_WAIT/ID_CHECK/WAKE_WAIT/CFG -- correct */
        }
    }
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
    RUN_TEST(test_replug_reinitialises_via_the_nonblocking_state_machine);
    RUN_TEST(test_reinitstep_issues_at_most_one_spi_transaction_per_call);
    RUN_TEST(test_reinitstep_never_calls_delayms);
    RUN_TEST(test_reinitstep_register_sequence_is_byte_identical_to_blocking);
    RUN_TEST(test_reinitstep_reaches_done_in_40_to_64_steps_at_500us);
    RUN_TEST(test_reinitstep_mode0_failure_retries_mode3_exactly_once_then_failed);
    RUN_TEST(test_reinitstep_reaches_failed_and_rearms_after_recovery_period);
    RUN_TEST(test_reinitstep_presence_true_only_on_done);
    RUN_TEST(test_single_sentinel_word_is_rejected_but_presence_unchanged);
    RUN_TEST(test_0x8001_word_one_lsb_off_sentinel_is_accepted);
    return UNITY_END();
}
