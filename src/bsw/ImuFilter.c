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
#define IMUFILTER_FS_HZ  1000.0f
#define IMUFILTER_PI     3.14159265f
#define IMUFILTER_SEC_PER_MINUTE 60.0f
#define IMUFILTER_RPM_THRESHOLD 1500u

/******************************************************************************/
/*--------------------------Function Declaration------------------------------*/
/******************************************************************************/
static void    ImuFilter_calc_coefficients   (ImuFilter_coefficients coeff[4], const float32 harmonic, const float32 q);
static void    ImuFilter_calc_lowpass        (ImuFilter_coefficients *coeff, const float32 fc);
static void    ImuFilter_lowpass_axisPackage (float32 sensor_sample[3], const ImuFilter_coefficients *coeff, const boolean active, ImuFilter_prev_values prev_values[3]);
static float32 ImuFilter_calc_rpm_notch      (const float32 x_k, const ImuFilter_coefficients coeff, const ImuFilter_prev_values prev_values);
static void    ImuFilter_filter_axisPackage  (float32 sensor_sample[3], const ImuFilter_coefficients coeff[4], ImuFilter_prev_values prev_values[3][4]);

/******************************************************************************/
/*--------------------------Interrupts----------------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*--------------------------Function Implementations--------------------------*/
/******************************************************************************/
/* Notch coefficients for one harmonic: f0 = harmonic * rpm / 60 per motor. Same formula
 * and Q for every harmonic, only the centre frequency differs. */
static void ImuFilter_calc_coefficients(ImuFilter_coefficients coeff[4], const float32 harmonic, const float32 q)
{
    for (uint8 i = 0u; i < 4u; i++)
    {
        float32 f0    = (harmonic * (float32)g_dshot_bidir_rpm.rpm[i]) / IMUFILTER_SEC_PER_MINUTE;
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
    return (coeff.b0 * x_k + coeff.b1 * prev_values.xk1 + coeff.b2 * prev_values.xk2 - coeff.a1 * prev_values.yk1 - coeff.a2 * prev_values.yk2) / (coeff.a0);
}


static void ImuFilter_filter_axisPackage(float32 sensor_sample[3], const ImuFilter_coefficients coeff[4], ImuFilter_prev_values prev_values[3][4])
{
    uint8 i,j;
    /* filter acc */
    for (i = 0u; i <3u; i++) /* 3 iterations for 3 acc values */
    {
       for (j = 0u; j < 4u; j++) /* 4 iterations for 4 motors */
       {
           float32 sample_prev = sensor_sample[i];
           if (g_dshot_bidir_rpm.rpm[j] > IMUFILTER_RPM_THRESHOLD)
           {
               sensor_sample[i] = ImuFilter_calc_rpm_notch(sample_prev, coeff[j], prev_values[i][j]);
           }
           prev_values[i][j].xk2 = prev_values[i][j].xk1;
           prev_values[i][j].xk1 = sample_prev;

           prev_values[i][j].yk2 = prev_values[i][j].yk1;
           prev_values[i][j].yk1 = sensor_sample[i];
       }
   }
}
/* One low-pass stage per axis behind the notch cascade. Inactive: pass through, state
 * glued to the signal -- same rule as a bypassed notch stage. */
static void ImuFilter_lowpass_axisPackage(float32 sensor_sample[3], const ImuFilter_coefficients *coeff, const boolean active, ImuFilter_prev_values prev_values[3])
{
    uint8 i;
    for (i = 0u; i < 3u; i++)
    {
        float32 sample_prev = sensor_sample[i];
        if (active != FALSE)
        {
            sensor_sample[i] = ImuFilter_calc_rpm_notch(sample_prev, *coeff, prev_values[i]);
        }
        prev_values[i].xk2 = prev_values[i].xk1;
        prev_values[i].xk1 = sample_prev;

        prev_values[i].yk2 = prev_values[i].yk1;
        prev_values[i].yk1 = sensor_sample[i];
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

    /* Calibration words (FusionCal.h, XCP-writable, read every tick like Ahrs.c does):
     * Q of every stage, and which harmonics run. A stage that is switched off is simply
     * not called -- its state goes stale, which is harmless (two samples of memory). */
    const float32 q      = FusionCal_positive(g_fusionCal.notchQ, FILTER_Q_MIN, FCAL_NOTCH_Q);
    /* restart tick: every stage in pass-through for this one sample (see s_restart) */
    const uint32  enable = (s_restart != FALSE) ? 0u : g_fusionCal.notchEnable;
    const float32 lpf_hz = g_fusionCal.accLpfHz;
    /* in range = on; 0, negatives, NaN (comparisons false) and absurd corners = off */
    const boolean lpf_on = (boolean)((s_restart == FALSE) && (lpf_hz >= FCAL_ACC_LPF_HZ_MIN) && (lpf_hz <= FCAL_ACC_LPF_HZ_MAX));

    s_restart = FALSE;

    if ((lpf_on != FALSE) && (lpf_hz != s_acc_lpf_hz_previous))
    {
        ImuFilter_calc_lowpass(&s_coeff_acc_lpf, lpf_hz);
        s_acc_lpf_hz_previous = lpf_hz;
    }

    if (g_dshot_bidir_rpm.gen != s_dshot_bidir_gen_previous)
    {
        /* filter coefficients for first and second harmonic */
        ImuFilter_calc_coefficients(s_coeff_h1, FILTER_HARMONIC_1, q);
        ImuFilter_calc_coefficients(s_coeff_h2, FILTER_HARMONIC_2, q);
    }

    /* notch filter acc, 1/rev then 2/rev (order of linear stages does not matter).
     * A set that is switched off is not called: its state goes stale, harmless (two samples
     * of memory) -- except after a dropout, where the restart tick glues it below. */
    if ((enable & FCAL_NOTCH_ENABLE_H1) != 0u)
    {
        ImuFilter_filter_axisPackage(imu_sample->acc, s_coeff_h1, s_prev_values_acc_h1);
    }
    if ((enable & FCAL_NOTCH_ENABLE_H2) != 0u)
    {
        ImuFilter_filter_axisPackage(imu_sample->acc, s_coeff_h2, s_prev_values_acc_h2);
    }

    /* low-pass acc behind the notches (broadband residual, frame run 8c); gyro is not low-passed */
    ImuFilter_lowpass_axisPackage(imu_sample->acc, &s_coeff_acc_lpf, lpf_on, s_prev_values_acc_lpf);

    /* notch filter gyro */
    if ((enable & FCAL_NOTCH_ENABLE_H1) != 0u)
    {
        ImuFilter_filter_axisPackage(imu_sample->gyro, s_coeff_h1, s_prev_values_gyro_h1);
    }
    if ((enable & FCAL_NOTCH_ENABLE_H2) != 0u)
    {
        ImuFilter_filter_axisPackage(imu_sample->gyro, s_coeff_h2, s_prev_values_gyro_h2);
    }

    s_dshot_bidir_gen_previous = g_dshot_bidir_rpm.gen;
}
