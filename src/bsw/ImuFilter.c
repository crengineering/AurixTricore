/*
 *  Code from crengineering
 *  Last modified: 09.11.2026
 *  Last Modified by: crengineering
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "ImuFilter.h"
#include "DshotLatch.h"
#include "FusionCal.h"
#include <math.h>

typedef struct
{
  float32 b0;
  float32 b1;
  float32 b2;
  float32 a0;
  float32 a1;
  float32 a2;
}ImuFilter_coefficients;

typedef struct
{
  float32 yk2;
  float32 yk1;
  float32 xk2;
  float32 xk1;
}ImuFilter_prev_values;

#define FILTER_Q_MIN      0.5f              /* reader clamp for g_fusionCal.notchQ (default FCAL_NOTCH_Q) */
#define FILTER_HARMONIC_1 1.0f              /* 1/rev: unbalance                         */
#define FILTER_HARMONIC_2 2.0f              /* 2/rev: blade pass of the two-blade prop  */
#define IMUFILTER_FS_HZ  1014.2f            /* measured ICM-42688-P DRDY rate (NavTask.h), never "1 kHz" */
/* Nyquist guard: a stage whose centre would sit above ~0.44 fs is bypassed. Above fs/2
 * sinf(w0) turns negative, a2/a0 > 1 and the biquad runs away -- and a single mis-decoded
 * bidir frame (Dshot.c: exponent 0 with a small mantissa -> tens of millions of eRPM, held
 * as "last valid") is enough to put one there. 0.44 * 1014.2 Hz * 60 / harmonic. */
#define IMUFILTER_RPM_MAX_H1 26700u
#define IMUFILTER_RPM_MAX_H2 13300u
/* finite guard on every stage output: |y| beyond this (or NaN, which fails both
 * comparisons) resets the stage to its input -- same idiom as ahrs_finiteOr() */
#define IMUFILTER_OUT_BOUND  1.0e6f
#define IMUFILTER_PI     3.14159265f
#define IMUFILTER_SEC_PER_MINUTE 60.0f
#define IMUFILTER_RPM_THRESHOLD 1500u

/******************************************************************************/
/*--------------------------Function Declaration------------------------------*/
/******************************************************************************/
static void    ImuFilter_calc_coefficients   (ImuFilter_coefficients coeff[4], const uint32 rpm[4], const float32 harmonic, const float32 q);
static void    ImuFilter_calc_lowpass        (ImuFilter_coefficients *coeff, const float32 fc);
static void    ImuFilter_lowpass_axisPackage (float32 sensor_sample[3], const ImuFilter_coefficients *coeff, const boolean active, ImuFilter_prev_values prev_values[3]);
static float32 ImuFilter_calc_rpm_notch      (const float32 x_k, const ImuFilter_coefficients coeff, const ImuFilter_prev_values prev_values);
static float32 ImuFilter_stage               (const float32 x, const ImuFilter_coefficients *coeff, const boolean active, ImuFilter_prev_values *prev_values);
static void    ImuFilter_filter_axisPackage  (float32 sensor_sample[3], const ImuFilter_coefficients coeff[4], const boolean active[4], ImuFilter_prev_values prev_values[3][4]);
static void    ImuFilter_stage_active        (boolean active[4], const uint32 rpm[4], const boolean set_on, const uint32 rpm_max);
static void    ImuFilter_flatten             (ImuFilter_prev_values *prev_values);

/******************************************************************************/
/*--------------------------Interrupts----------------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*--------------------------Function Implementations--------------------------*/
/******************************************************************************/
/* Notch coefficients for one harmonic: f0 = harmonic * rpm / 60 per motor. Same formula
 * and Q for every harmonic, only the centre frequency differs. */
static void ImuFilter_calc_coefficients(ImuFilter_coefficients coeff[4], const uint32 rpm[4], const float32 harmonic, const float32 q)
{
    for (uint8 i = 0u; i < 4u; i++)
    {
        float32 f0    = (harmonic * (float32)rpm[i]) / IMUFILTER_SEC_PER_MINUTE;
        float32 w0    = (2.0f * IMUFILTER_PI * f0) / (IMUFILTER_FS_HZ);
        float32 alpha = sinf(w0) / (2.0f * q);

        coeff[i].b0 = 1.0f;
        coeff[i].b1 = -2.0f * cosf(w0);
        coeff[i].b2 = 1.0f;

        coeff[i].a0 = 1.0f + alpha;
        coeff[i].a1 = -2.0f * cosf(w0);
        coeff[i].a2 = 1.0f - alpha;
    }
}

/* 2nd-order Butterworth low-pass (RBJ cookbook, Q = 1/sqrt(2)) at fc. Same biquad form
 * as the notch, only the numerator differs, so ImuFilter_calc_rpm_notch() runs it too. */
static void ImuFilter_calc_lowpass(ImuFilter_coefficients *coeff, const float32 fc)
{
    float32 w0    = (2.0f * IMUFILTER_PI * fc) / (IMUFILTER_FS_HZ);
    float32 cosw0 = cosf(w0);
    float32 alpha = sinf(w0) / (2.0f * 0.70710678f);

    coeff->b0 = (1.0f - cosw0) / 2.0f;
    coeff->b1 =  1.0f - cosw0;
    coeff->b2 = (1.0f - cosw0) / 2.0f;

    coeff->a0 = 1.0f + alpha;
    coeff->a1 = -2.0f * cosw0;
    coeff->a2 = 1.0f - alpha;
}

/* Biquad step: y[k] = (b0 x[k] + b1 x[k-1] + b2 x[k-2] - a1 y[k-1] - a2 y[k-2]) / a0 */
static float32 ImuFilter_calc_rpm_notch(const float32 x_k, const ImuFilter_coefficients coeff, const ImuFilter_prev_values prev_values)
{
    return ((coeff.b0 * x_k) + (coeff.b1 * prev_values.xk1) + (coeff.b2 * prev_values.xk2)
          - (coeff.a1 * prev_values.yk1) - (coeff.a2 * prev_values.yk2)) / coeff.a0;
}


/* One biquad stage, state kept by the caller. Inactive: pass through, state follows the
 * signal (so the stage starts without a step when it becomes active). A non-finite output
 * (NaN / inf / absurd -- a bad coefficient set or a poisoned state) is caught here: the
 * stage is reset to its input and the input is passed on, so one bad sample can never leave
 * a stage ringing or NaN until reboot. */
static float32 ImuFilter_stage(const float32 x, const ImuFilter_coefficients *coeff, const boolean active, ImuFilter_prev_values *prev_values)
{
    float32 y = x;

    if (active != FALSE)
    {
        y = ImuFilter_calc_rpm_notch(x, *coeff, *prev_values);
        if ((y >= -IMUFILTER_OUT_BOUND) && (y <= IMUFILTER_OUT_BOUND))
        {
            /* finite: regular history shift below */
        }
        else
        {
            y = x;
            prev_values->xk1 = x;
            prev_values->yk1 = x;
            prev_values->xk2 = x;
            prev_values->yk2 = x;
        }
    }
    prev_values->xk2 = prev_values->xk1;
    prev_values->xk1 = x;
    prev_values->yk2 = prev_values->yk1;
    prev_values->yk1 = y;
    return y;
}

/* Restart helper: make the two-sample history flat (xk2 = xk1, yk2 = yk1). */
static void ImuFilter_flatten(ImuFilter_prev_values *prev_values)
{
    prev_values->xk2 = prev_values->xk1;
    prev_values->yk2 = prev_values->yk1;
}

/* Three axes through the four-stage cascade of one harmonic set. Stage j eats stage j-1's
 * output. Called on EVERY step, also for a disabled set (all active[] FALSE): the stages
 * then track the signal and come back without ringing when the set is enabled again. */
static void ImuFilter_filter_axisPackage(float32 sensor_sample[3], const ImuFilter_coefficients coeff[4], const boolean active[4], ImuFilter_prev_values prev_values[3][4])
{
    uint8 i;
    uint8 j;
    for (i = 0u; i <3u; i++) /* 3 iterations for 3 axes */
    {
       for (j = 0u; j < 4u; j++) /* 4 iterations for 4 motors */
       {
           sensor_sample[i] = ImuFilter_stage(sensor_sample[i], &coeff[j], active[j], &prev_values[i][j]);
       }
   }
}

/* One low-pass stage per axis behind the notch cascade; same stage rules. */
static void ImuFilter_lowpass_axisPackage(float32 sensor_sample[3], const ImuFilter_coefficients *coeff, const boolean active, ImuFilter_prev_values prev_values[3])
{
    uint8 i;
    for (i = 0u; i < 3u; i++)
    {
        sensor_sample[i] = ImuFilter_stage(sensor_sample[i], coeff, active, &prev_values[i]);
    }
}

/* Which stages of a harmonic set run this step: the set is enabled, the motor turns fast
 * enough that its line is out of the control band, and slow enough that the centre stays
 * well below Nyquist (see IMUFILTER_RPM_MAX_*). */
static void ImuFilter_stage_active(boolean active[4], const uint32 rpm[4], const boolean set_on, const uint32 rpm_max)
{
    uint8 j;
    for (j = 0u; j < 4u; j++)
    {
        if ((set_on != FALSE) && (rpm[j] > IMUFILTER_RPM_THRESHOLD) && (rpm[j] < rpm_max))
        {
            active[j] = TRUE;
        }
        else
        {
            active[j] = FALSE;
        }
    }
}

/* Set by ImuFilter_restart(): the next step passes the sample through every stage
 * untouched and glues all states to it. Used after a sensor dropout, so the first good
 * sample reaches the AHRS as it is (its re-alignment needs the real |a|, not a low-pass
 * transient from the stale pre-dropout state). */
static boolean s_restart = FALSE;

/******************************************************************************/
/*--------------------------Global Functions----------------------------------*/
/******************************************************************************/
void ImuFilter_restart(void)
{
    s_restart = TRUE;
}

void ImuFilter_step(Icm42688_Sample *imu_sample)
{
    /* one state block per axis, motor AND harmonic -- the 2/rev stages are their own
     * filters and must not share history with the 1/rev stages */
    static ImuFilter_prev_values s_prev_values_acc_h1[3][4];
    static ImuFilter_prev_values s_prev_values_gyro_h1[3][4];
    static ImuFilter_prev_values s_prev_values_acc_h2[3][4];
    static ImuFilter_prev_values s_prev_values_gyro_h2[3][4];
    static uint32 s_dshot_bidir_gen_previous;

    /* static: the coefficients are only recomputed when Core 0 has published a new
     * rpm set (gen moved). On a tick with unchanged gen the previous set must still
     * be there -- as a plain local it would be uninitialised on that tick. */
    static ImuFilter_coefficients s_coeff_h1[4];
    static ImuFilter_coefficients s_coeff_h2[4];

    /* accelerometer low-pass behind the notches: one stage per axis, coefficients only
     * recomputed when the cal word changes */
    static ImuFilter_prev_values  s_prev_values_acc_lpf[3];
    static ImuFilter_coefficients s_coeff_acc_lpf;
    static float32                s_acc_lpf_hz_previous = -1.0f;

    /* One snapshot of the latch per step (rpm once per motor, then gen): coefficients and
     * stage activation see the same snapshot; reading gen last means a publish between the
     * reads shows up as a changed gen on the next step, never as a dropped set. Each word
     * is atomic; a torn old/new pair is 0.13 Hz at 8000 rpm/s. */
    uint32  gen;
    uint32  rpm[4];
    boolean active_h1[4];
    boolean active_h2[4];
    boolean set_h1;
    boolean set_h2;
    boolean lpf_on;
    uint8   j;

    /* Calibration words (FusionCal.h, XCP-writable, read every tick like Ahrs.c does) */
    const float32 q      = FusionCal_positive(g_fusionCal.notchQ, FILTER_Q_MIN, FCAL_NOTCH_Q);
    const uint32  enable = g_fusionCal.notchEnable;
    const float32 lpf_hz = g_fusionCal.accLpfHz;

    for (j = 0u; j < 4u; j++)
    {
        rpm[j] = g_dshot_bidir_rpm.rpm[j];
    }
    gen = g_dshot_bidir_rpm.gen;   /* AFTER the rpm words: a publish in between is caught next tick */

    /* restart tick (after a gap): every stage passes through and takes its state from
     * this sample -- the sets ARE called below, with nothing active, and the history is
     * flattened afterwards (a bypassed stage keeps one older sample in xk2/yk2) */
    const boolean restart_tick = s_restart;
    set_h1 = FALSE;
    set_h2 = FALSE;
    lpf_on = FALSE;
    if (s_restart == FALSE)
    {
        if ((enable & FCAL_NOTCH_ENABLE_H1) != 0u) { set_h1 = TRUE; }
        if ((enable & FCAL_NOTCH_ENABLE_H2) != 0u) { set_h2 = TRUE; }
        /* in range = on; 0, negatives, NaN (comparisons false) and absurd corners = off */
        if ((lpf_hz >= FCAL_ACC_LPF_HZ_MIN) && (lpf_hz <= FCAL_ACC_LPF_HZ_MAX)) { lpf_on = TRUE; }
    }
    s_restart = FALSE;

    if ((lpf_on != FALSE) && (lpf_hz != s_acc_lpf_hz_previous))
    {
        ImuFilter_calc_lowpass(&s_coeff_acc_lpf, lpf_hz);
        s_acc_lpf_hz_previous = lpf_hz;
    }

    if (gen != s_dshot_bidir_gen_previous)
    {
        /* filter coefficients for first and second harmonic */
        ImuFilter_calc_coefficients(s_coeff_h1, rpm, FILTER_HARMONIC_1, q);
        ImuFilter_calc_coefficients(s_coeff_h2, rpm, FILTER_HARMONIC_2, q);
    }
    s_dshot_bidir_gen_previous = gen;

    ImuFilter_stage_active(active_h1, rpm, set_h1, IMUFILTER_RPM_MAX_H1);
    ImuFilter_stage_active(active_h2, rpm, set_h2, IMUFILTER_RPM_MAX_H2);

    /* notch filter acc, 1/rev then 2/rev (order of linear stages does not matter); both
     * sets run every step, a disabled set just passes through and tracks the signal */
    ImuFilter_filter_axisPackage(imu_sample->acc, s_coeff_h1, active_h1, s_prev_values_acc_h1);
    ImuFilter_filter_axisPackage(imu_sample->acc, s_coeff_h2, active_h2, s_prev_values_acc_h2);

    /* low-pass acc behind the notches (broadband residual, frame run 8c); gyro is not low-passed */
    ImuFilter_lowpass_axisPackage(imu_sample->acc, &s_coeff_acc_lpf, lpf_on, s_prev_values_acc_lpf);

    /* notch filter gyro */
    ImuFilter_filter_axisPackage(imu_sample->gyro, s_coeff_h1, active_h1, s_prev_values_gyro_h1);
    ImuFilter_filter_axisPackage(imu_sample->gyro, s_coeff_h2, active_h2, s_prev_values_gyro_h2);

    if (restart_tick != FALSE)
    {
        /* every stage just passed this sample through: xk1 == yk1 == its input. Copy that
         * into xk2/yk2 too, so no pre-gap sample survives anywhere in the cascade. */
        uint8 i;
        for (i = 0u; i < 3u; i++)
        {
            for (j = 0u; j < 4u; j++)
            {
                ImuFilter_flatten(&s_prev_values_acc_h1[i][j]);
                ImuFilter_flatten(&s_prev_values_acc_h2[i][j]);
                ImuFilter_flatten(&s_prev_values_gyro_h1[i][j]);
                ImuFilter_flatten(&s_prev_values_gyro_h2[i][j]);
            }
            ImuFilter_flatten(&s_prev_values_acc_lpf[i]);
        }
    }
}
