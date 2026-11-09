#include "unity.h"
#include <math.h>
#include "fakes/Ifx_Types.h"
#include "../src/bsw/ImuFilter.h"
#include "../src/bsw/DshotLatch.h"

/* ImuFilter (rpm-tracked IMU notch, SWE1-FW follow-up of frame run 7): the four-stage
 * notch cascade through the public ImuFilter_step(). g_dshot_bidir_rpm comes from
 * SharedRam.c (linked); the test plays Core 0 and writes rpm / counters directly.
 *
 * Every number here is a plain synthetic sine at the filter's 1 kHz rate -- no sensor,
 * no latch protocol. Amplitudes are measured as max|y| over the last WINDOW samples
 * after SETTLE samples, so the transient of the cascade is excluded. */

#define FS_HZ      1000.0
#define TWO_PI     6.283185307179586
#define SETTLE     2500u
#define WINDOW     500u

/* Core 0 stand-in: one DShot frame per tick unless frozen. */
static boolean s_genRuns;

static void core0_frame(void)
{
    uint32 m;
    if (s_genRuns != FALSE)
    {
        g_dshot_bidir_rpm.gen++;
    }
    for (m = 0u; m < 4u; m++)
    {
        g_dshot_bidir_rpm.dshot_ok_count[m]++;
    }
}

static void set_rpm_all(uint32 rpm)
{
    uint32 m;
    for (m = 0u; m < 4u; m++) { g_dshot_bidir_rpm.rpm[m] = rpm; }
}

/* Run n ticks of a sine of frequency f on every acc and gyro axis; return max|gyroX| over
 * the last WINDOW ticks (the cascade is identical on every axis, gyro X stands for all). */
static float32 run_sine(double f, uint32 n)
{
    Icm42688_Sample s;
    float32 peak = 0.0f;
    uint32 k;

    for (k = 0u; k < n; k++)
    {
        const float32 v = (float32)sin(TWO_PI * f * ((double)k / FS_HZ));
        uint32 i;
        for (i = 0u; i < 3u; i++) { s.acc[i] = v; s.gyro[i] = v; }
        s.tempC = 25.0f;

        core0_frame();
        ImuFilter_step(&s);

        if ((k + WINDOW) >= n)
        {
            const float32 a = fabsf(s.gyro[0]);
            if (a > peak) { peak = a; }
        }
    }
    return peak;
}

/* Flush the cascade's state between scenarios: 1500 bypassed ticks on a zero signal. */
static void flush(void)
{
    uint32 saved[4];
    uint32 m;
    for (m = 0u; m < 4u; m++) { saved[m] = g_dshot_bidir_rpm.rpm[m]; }
    set_rpm_all(0u);
    (void)run_sine(0.0, 1500u);
    for (m = 0u; m < 4u; m++) { g_dshot_bidir_rpm.rpm[m] = saved[m]; }
}

void setUp(void)
{
    s_genRuns = TRUE;
    flush();
}

void tearDown(void)
{
}

/* 1. Four motors at 4800 rpm -> 80 Hz line goes away (> 30 dB). */
static void test_notch_kills_1rev_line(void)
{
    set_rpm_all(4800u);
    const float32 peak = run_sine(80.0, SETTLE + WINDOW);
    TEST_ASSERT_TRUE_MESSAGE(peak < 0.03f, "80 Hz line not attenuated by > 30 dB");
}

/* 2. Same notch, a 5 Hz signal (the control band) passes with < 1 % change. */
static void test_notch_passes_control_band(void)
{
    set_rpm_all(4800u);
    const float32 peak = run_sine(5.0, SETTLE + WINDOW);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, peak);
}

/* 3. Motors at slightly different speeds (78.8 .. 80.0 Hz, the run-7 spread): a line in the
 *    middle is still taken out by the cascade (> 20 dB) -- each stage only partly, together. */
static void test_cascade_covers_motor_spread(void)
{
    g_dshot_bidir_rpm.rpm[0] = 4800u;   /* 80.0 Hz */
    g_dshot_bidir_rpm.rpm[1] = 4729u;   /* 78.8 Hz */
    g_dshot_bidir_rpm.rpm[2] = 4757u;   /* 79.3 Hz */
    g_dshot_bidir_rpm.rpm[3] = 4786u;   /* 79.8 Hz */
    const float32 peak = run_sine(79.4, SETTLE + WINDOW);
    TEST_ASSERT_TRUE_MESSAGE(peak < 0.1f, "79.4 Hz line not attenuated by > 20 dB");
}

/* 4. rpm 0 (threshold bypass): a constant passes EXACTLY, for a long time. Without the
 *    bypass the degenerate biquad (double pole at z = 1) drifts on float rounding. */
static void test_bypass_below_threshold_is_exact(void)
{
    Icm42688_Sample s;
    uint32 k;
    set_rpm_all(0u);
    for (k = 0u; k < 100000u; k++)
    {
        uint32 i;
        for (i = 0u; i < 3u; i++) { s.acc[i] = 1.0f; s.gyro[i] = 1.0f; }
        s.tempC = 25.0f;
        core0_frame();
        ImuFilter_step(&s);
        TEST_ASSERT_EQUAL_FLOAT(1.0f, s.acc[2]);
        TEST_ASSERT_EQUAL_FLOAT(1.0f, s.gyro[0]);
    }
}

/* 5. Bypass -> active hand-over: motors come up to 4800 on a constant 1 g -- no step.
 *    The bypassed stages have tracked the signal, so the notch starts from y == x. */
static void test_activation_has_no_step(void)
{
    Icm42688_Sample s;
    uint32 k;
    set_rpm_all(0u);
    (void)run_sine(0.0, 100u);
    for (k = 0u; k < 3000u; k++)
    {
        uint32 i;
        if (k == 100u) { set_rpm_all(4800u); }
        for (i = 0u; i < 3u; i++) { s.acc[i] = 1.0f; s.gyro[i] = 0.0f; }
        s.tempC = 25.0f;
        core0_frame();
        ImuFilter_step(&s);
        TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 1.0f, s.acc[2]);
    }
}

/* 6. One tick in ~70 sees an unchanged gen (1014 Hz task vs 1000 Hz writer). That tick must
 *    still be filtered, with the previous coefficient set -- the coefficients are static
 *    for exactly this reason. */
static void test_unchanged_gen_tick_is_still_filtered(void)
{
    Icm42688_Sample s;
    float32 peak = 0.0f;
    uint32 k;
    set_rpm_all(4800u);
    for (k = 0u; k < (SETTLE + WINDOW); k++)
    {
        const float32 v = (float32)sin(TWO_PI * 80.0 * ((double)k / FS_HZ));
        uint32 i;
        for (i = 0u; i < 3u; i++) { s.acc[i] = v; s.gyro[i] = v; }
        s.tempC = 25.0f;
        s_genRuns = (boolean)((k % 70u) != 0u);     /* every 70th tick: no new frame */
        core0_frame();
        ImuFilter_step(&s);
        if ((k + WINDOW) >= (SETTLE + WINDOW))
        {
            const float32 a = fabsf(s.gyro[0]);
            if (a > peak) { peak = a; }
        }
    }
    s_genRuns = TRUE;
    TEST_ASSERT_TRUE_MESSAGE(peak < 0.03f, "a tick with unchanged gen let the 80 Hz line through");
}

/* 7. Second harmonic: four motors at 4800 rpm -> the 160 Hz blade-pass line goes away too
 *    (> 30 dB), while the 1/rev line at 80 Hz is still gone and 5 Hz still passes. */
static void test_second_harmonic_kills_blade_pass_line(void)
{
    set_rpm_all(4800u);
    float32 peak = run_sine(160.0, SETTLE + WINDOW);
    TEST_ASSERT_TRUE_MESSAGE(peak < 0.03f, "160 Hz (2/rev) line not attenuated by > 30 dB");
    peak = run_sine(80.0, SETTLE + WINDOW);
    TEST_ASSERT_TRUE_MESSAGE(peak < 0.03f, "80 Hz (1/rev) line lost with the 2/rev stages added");
    peak = run_sine(5.0, SETTLE + WINDOW);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, peak);
}

/* 8. The 2/rev stages sit between the two lines, not on them: 120 Hz (midway at 4800 rpm)
 *    is attenuated by the skirts of both cascades but must still come through at > -6 dB
 *    -- a guard that the second set is placed at 2 * rpm / 60 and not on top of the first. */
static void test_second_harmonic_sits_at_twice_rpm(void)
{
    set_rpm_all(4800u);
    const float32 peak = run_sine(120.0, SETTLE + WINDOW);
    TEST_ASSERT_TRUE_MESSAGE(peak > 0.5f, "120 Hz attenuated by more than 6 dB: 2/rev set misplaced?");
    TEST_ASSERT_TRUE_MESSAGE(peak < 0.99f, "120 Hz untouched: 2/rev set inactive?");
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_notch_kills_1rev_line);
    RUN_TEST(test_notch_passes_control_band);
    RUN_TEST(test_cascade_covers_motor_spread);
    RUN_TEST(test_bypass_below_threshold_is_exact);
    RUN_TEST(test_activation_has_no_step);
    RUN_TEST(test_unchanged_gen_tick_is_still_filtered);
    RUN_TEST(test_second_harmonic_kills_blade_pass_line);
    RUN_TEST(test_second_harmonic_sits_at_twice_rpm);
    return UNITY_END();
}
