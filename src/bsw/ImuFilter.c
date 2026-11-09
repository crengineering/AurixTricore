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

#define FILTER_Q         5.0f
#define FILTER_HARMONICS 2.0f
#define IMUFILTER_FS_HZ  1000.0f
#define IMUFILTER_PI     3.14159265f
#define IMUFILTER_SEC_PER_MINUTE 60.0f
#define IMUFILTER_RPM_THRESHOLD 1500u

/******************************************************************************/
/*--------------------------Function Declaration------------------------------*/
/******************************************************************************/
static void    ImuFilter_calc_coefficients   (ImuFilter_coefficients coeff[4]);
static float32 ImuFilter_calc_rpm_notch      (const float32 x_k, const ImuFilter_coefficients coeff, const ImuFilter_prev_values prev_values);
static void    ImuFilter_filter_axisPackage  (float32 sensor_sample[3], const ImuFilter_coefficients coeff[4], ImuFilter_prev_values prev_values[3][4]);

/******************************************************************************/
/*--------------------------Interrupts----------------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*--------------------------Function Implementations--------------------------*/
/******************************************************************************/
static void ImuFilter_calc_coefficients(ImuFilter_coefficients coeff[4])
{
    for (uint8 i = 0u; i < 4u; i++)
    {
        float32 f0    = (float32)g_dshot_bidir_rpm.rpm[i] / IMUFILTER_SEC_PER_MINUTE;
        float32 w0    = (2.0f * IMUFILTER_PI * f0) / (IMUFILTER_FS_HZ);
        float32 alpha = sinf(w0) / (2 * FILTER_Q);

        coeff[i].b0 = 1.0f;
        coeff[i].b1 = -2.0f * cosf(w0);
        coeff[i].b2 = 1.0f;

        coeff[i].a0 = 1.0f + alpha;
        coeff[i].a1 = -2.0f * cosf(w0);
        coeff[i].a2 = 1.0f - alpha;
    }
}

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
/******************************************************************************/
/*--------------------------Global Functions----------------------------------*/
/******************************************************************************/
void ImuFilter_step(Icm42688_Sample *imu_sample)
{
    static ImuFilter_prev_values s_prev_values_acc[3][4];
    static ImuFilter_prev_values s_prev_values_gyro[3][4];
    static uint32 s_dshot_bidir_gen_previous;

    /* static: the coefficients are only recomputed when Core 0 has published a new
     * rpm set (gen moved). On a tick with unchanged gen the previous set must still
     * be there -- as a plain local it would be uninitialised on that tick. */
    static ImuFilter_coefficients s_coeff[4];


    if (g_dshot_bidir_rpm.gen != s_dshot_bidir_gen_previous)
    {
        /* filter coefficients for frist harmony */
        ImuFilter_calc_coefficients(s_coeff);
    }

    /* notch filter acc */
    ImuFilter_filter_axisPackage(imu_sample->acc, s_coeff, s_prev_values_acc);

    /* notch filter gyro */
    ImuFilter_filter_axisPackage(imu_sample->gyro, s_coeff, s_prev_values_gyro);

    s_dshot_bidir_gen_previous = g_dshot_bidir_rpm.gen;
}
