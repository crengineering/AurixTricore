#include "unity.h"
#include <math.h>
#include "fakes/Ifx_Types.h"
#include "../src/bsw/ImuFilter.h"
#include "../src/bsw/DshotLatch.h"
#include "../src/bsw/FusionCal.h"

/* ImuFilter (rpm-tracked IMU notch, SWE1-FW follow-up of frame run 7): the four-stage
 * notch cascade through the public ImuFilter_step(). g_dshot_bidir_rpm comes from
 * SharedRam.c (linked); the test plays Core 0 and writes rpm / counters directly.
 *
 * Every number here is a plain synthetic sine at the filter's 1 kHz rate -- no sensor,
 * no latch protocol. Amplitudes are measured as max|y| over the last WINDOW samples
 * after SETTLE samples, so the transient of the cascade is excluded. */

#define FS_HZ      1014.2          /* IMUFILTER_FS_HZ: the sensor's measured DRDY rate */
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
    FusionCal_init();          /* notchQ 5, notchEnable = both harmonics, accLpfHz 15 */
    g_fusionCal.accLpfHz = 0.0f;   /* notch tests run with the acc low-pass OFF; 12/13 switch it on */
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

/* 9. notchEnable 0: the IMU goes through untouched, sample for sample (no stage is
 *    called). Compared per sample, not by peak: a 1 kHz-sampled 80 Hz sine never hits
 *    its crest exactly (max sample 0.998). */
static void test_enable_zero_is_raw_passthrough(void)
{
    Icm42688_Sample s;
    uint32 k;
    set_rpm_all(4800u);
    g_fusionCal.notchEnable = 0u;
    for (k = 0u; k < 1000u; k++)
    {
        const float32 v = (float32)sin(TWO_PI * 80.0 * ((double)k / FS_HZ));
        uint32 i;
        for (i = 0u; i < 3u; i++) { s.acc[i] = v; s.gyro[i] = v; }
        s.tempC = 25.0f;
        core0_frame();
        ImuFilter_step(&s);
        TEST_ASSERT_EQUAL_FLOAT(v, s.acc[1]);
        TEST_ASSERT_EQUAL_FLOAT(v, s.gyro[2]);
    }
}

/* 10. notchEnable bit 1 only: 160 Hz gone, 80 Hz untouched -- the bits select the sets. */
static void test_enable_h2_only(void)
{
    set_rpm_all(4800u);
    g_fusionCal.notchEnable = FCAL_NOTCH_ENABLE_H2;
    float32 peak = run_sine(160.0, SETTLE + WINDOW);
    TEST_ASSERT_TRUE_MESSAGE(peak < 0.03f, "2/rev set off although bit 1 is set");
    peak = run_sine(80.0, SETTLE + WINDOW);
    /* not notched: only the skirt of the 160 Hz stages touches 80 Hz (-0.25 dB) */
    TEST_ASSERT_TRUE_MESSAGE(peak > 0.9f, "80 Hz attenuated although only the 2/rev set is enabled");
}

/* 11. notchQ widens / narrows the stage: at 72 Hz (8 Hz below the 80 Hz notch) Q 2 bites
 *     harder than Q 10. A nonsense Q (0, NaN) falls back to the default via the clamp. */
static void test_q_word_changes_width_and_is_clamped(void)
{
    union { float32 f; uint32 u; } nan_bits;
    set_rpm_all(4800u);

    g_fusionCal.notchQ = 2.0f;
    const float32 wide = run_sine(72.0, SETTLE + WINDOW);
    g_fusionCal.notchQ = 10.0f;
    const float32 narrow = run_sine(72.0, SETTLE + WINDOW);
    TEST_ASSERT_TRUE_MESSAGE(wide < (narrow - 0.2f), "Q 2 should attenuate 72 Hz clearly more than Q 10");

    g_fusionCal.notchQ = 0.0f;
    const float32 q0 = run_sine(72.0, SETTLE + WINDOW);
    nan_bits.u = 0x7FC00000u;
    g_fusionCal.notchQ = nan_bits.f;
    const float32 qnan = run_sine(72.0, SETTLE + WINDOW);
    g_fusionCal.notchQ = FCAL_NOTCH_Q;
    const float32 qdef = run_sine(72.0, SETTLE + WINDOW);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, qdef, q0);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, qdef, qnan);
}

/* 12. Accelerometer low-pass behind the notches (AccLpfHz 15, notches off): an 80 Hz line on
 *     the ACC is ~-30 dB down, 1 Hz passes within 1 %, and the GYRO is untouched by it. */
static void test_acc_lowpass_attenuates_acc_only(void)
{
    Icm42688_Sample s;
    float32 peak_acc = 0.0f, peak_gyro = 0.0f;
    uint32 k;
    set_rpm_all(4800u);
    g_fusionCal.notchEnable = 0u;
    g_fusionCal.accLpfHz = 15.0f;
    for (k = 0u; k < (SETTLE + WINDOW); k++)
    {
        const float32 v = (float32)sin(TWO_PI * 80.0 * ((double)k / FS_HZ));
        uint32 i;
        for (i = 0u; i < 3u; i++) { s.acc[i] = v; s.gyro[i] = v; }
        s.tempC = 25.0f;
        core0_frame();
        ImuFilter_step(&s);
        if ((k + WINDOW) >= (SETTLE + WINDOW))
        {
            if (fabsf(s.acc[0]) > peak_acc)   { peak_acc = fabsf(s.acc[0]); }
            if (fabsf(s.gyro[0]) > peak_gyro) { peak_gyro = fabsf(s.gyro[0]); }
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(peak_acc < 0.05f, "acc 80 Hz not attenuated by the 15 Hz low-pass");
    TEST_ASSERT_TRUE_MESSAGE(peak_gyro > 0.99f, "gyro must not be low-passed");

    /* 1 Hz through the same low-pass: within 1 % (window = 500 samples = half a period,
     * so run long enough that the window holds a crest) */
    peak_acc = 0.0f;
    for (k = 0u; k < 4000u; k++)
    {
        const float32 v = (float32)sin(TWO_PI * 1.0 * ((double)k / FS_HZ));
        uint32 i;
        for (i = 0u; i < 3u; i++) { s.acc[i] = v; s.gyro[i] = 0.0f; }
        s.tempC = 25.0f;
        core0_frame();
        ImuFilter_step(&s);
        if ((k >= 2000u) && (fabsf(s.acc[2]) > peak_acc)) { peak_acc = fabsf(s.acc[2]); }
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, peak_acc);
}

/* 13. AccLpfHz 0 (and NaN) = off: acc passes sample for sample with the notches off. */
static void test_acc_lowpass_off_is_exact_passthrough(void)
{
    union { float32 f; uint32 u; } nan_bits;
    Icm42688_Sample s;
    uint32 k;
    set_rpm_all(4800u);
    g_fusionCal.notchEnable = 0u;
    nan_bits.u = 0x7FC00000u;
    for (k = 0u; k < 2000u; k++)
    {
        const float32 v = (float32)sin(TWO_PI * 80.0 * ((double)k / FS_HZ));
        uint32 i;
        g_fusionCal.accLpfHz = (k < 1000u) ? 0.0f : nan_bits.f;
        for (i = 0u; i < 3u; i++) { s.acc[i] = v; s.gyro[i] = v; }
        s.tempC = 25.0f;
        core0_frame();
        ImuFilter_step(&s);
        TEST_ASSERT_EQUAL_FLOAT(v, s.acc[0]);
    }
}

/* 14. ImuFilter_restart(): after a gap the next sample passes through every stage and the
 *     stages take their history from it -- no ringing off the pre-gap state. Pre-gap: an 80 Hz
 *     line with the notches hot; gap; then a constant 1 g must come out as 1 g from tick 1. */
static void test_restart_takes_state_from_first_sample(void)
{
    Icm42688_Sample s;
    uint32 k;
    set_rpm_all(4800u);
    g_fusionCal.accLpfHz = 15.0f;
    (void)run_sine(80.0, 1000u);              /* notch + low-pass states full of 80 Hz history */
    ImuFilter_restart();
    for (k = 0u; k < 50u; k++)
    {
        uint32 i;
        for (i = 0u; i < 3u; i++) { s.acc[i] = 1.0f; s.gyro[i] = 1.0f; }
        s.tempC = 25.0f;
        core0_frame();
        ImuFilter_step(&s);
        TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 1.0f, s.acc[2]);
        TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 1.0f, s.gyro[0]);
    }
}

/* 15. A mis-decoded rpm above the Nyquist guard (20000 rpm: 2/rev at 667 Hz > fs/2) must not
 *     run a stage unstable: the stage bypasses, the output stays finite; back at 4800 rpm the
 *     cascade works again. */
static void test_rpm_above_nyquist_is_bypassed_and_recovers(void)
{
    Icm42688_Sample s;
    uint32 k;
    set_rpm_all(4800u);
    g_dshot_bidir_rpm.rpm[1] = 20000u;        /* one motor's frame decoded to nonsense */
    for (k = 0u; k < 3000u; k++)
    {
        const float32 v = (float32)sin(TWO_PI * 80.0 * ((double)k / FS_HZ));
        uint32 i;
        for (i = 0u; i < 3u; i++) { s.acc[i] = v; s.gyro[i] = v; }
        s.tempC = 25.0f;
        core0_frame();
        ImuFilter_step(&s);
        TEST_ASSERT_TRUE_MESSAGE(fabsf(s.gyro[0]) < 2.0f, "stage ran away on an above-Nyquist rpm");
        TEST_ASSERT_TRUE_MESSAGE(fabsf(s.acc[0]) < 2.0f, "acc stage ran away on an above-Nyquist rpm");
    }
    set_rpm_all(4800u);
    const float32 peak = run_sine(80.0, SETTLE + WINDOW);
    TEST_ASSERT_TRUE_MESSAGE(peak < 0.03f, "cascade did not recover after the bad rpm went away");
}

/* 16. A poisoned stage state (NaN) is caught by the finite guard: the cascade is back to
 *     normal after the bad samples. Poison through NaN inputs. */
static void test_nan_input_does_not_poison_the_filter(void)
{
    union { float32 f; uint32 u; } nan_bits;
    Icm42688_Sample s;
    uint32 k;
    nan_bits.u = 0x7FC00000u;
    set_rpm_all(4800u);
    (void)run_sine(80.0, 500u);
    for (k = 0u; k < 3u; k++)                  /* three NaN samples in a row */
    {
        uint32 i;
        for (i = 0u; i < 3u; i++) { s.acc[i] = nan_bits.f; s.gyro[i] = nan_bits.f; }
        s.tempC = 25.0f;
        core0_frame();
        ImuFilter_step(&s);
    }
    const float32 peak = run_sine(80.0, SETTLE + WINDOW);
    TEST_ASSERT_TRUE_MESSAGE(peak == peak, "output NaN after a NaN input");
    TEST_ASSERT_TRUE_MESSAGE(peak < 0.03f, "cascade did not recover after NaN samples");
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
    RUN_TEST(test_enable_zero_is_raw_passthrough);
    RUN_TEST(test_enable_h2_only);
    RUN_TEST(test_q_word_changes_width_and_is_clamped);
    RUN_TEST(test_acc_lowpass_attenuates_acc_only);
    RUN_TEST(test_acc_lowpass_off_is_exact_passthrough);
    RUN_TEST(test_restart_takes_state_from_first_sample);
    RUN_TEST(test_rpm_above_nyquist_is_bypassed_and_recovers);
    RUN_TEST(test_nan_input_does_not_poison_the_filter);
    return UNITY_END();
}
