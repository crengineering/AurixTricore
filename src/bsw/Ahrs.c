/**********************************************************************************************************************
 * \file Ahrs.c
 * \brief Complementary-filter attitude estimator — see Ahrs.h.
 *********************************************************************************************************************/
#include "Ahrs.h"
#include <math.h>

/* --- tuning ------------------------------------------------------------- */

/* Complementary-filter time constant [s]. Above this the gyro dominates, below
 * it the accelerometer pulls the estimate back to gravity. 0.5 s is the usual
 * starting point for a small multirotor: long enough to ride out vibration and
 * short manoeuvres, short enough that gyro drift never accumulates visibly. */
#define AHRS_TAU_S            (0.5f)

/* Accelerometer trust window [g]. Outside it the vector is contaminated by
 * thrust or impact and gravity is no longer the dominant term, so the filter
 * coasts on the gyro. At rest this board measures |a| = 0.99 g. */
#define AHRS_ACC_MIN_G        (0.85f)
#define AHRS_ACC_MAX_G        (1.15f)

/* Gyro-bias calibration: samples to average, and the motion gate. At the 50 Hz
 * IMU task 100 samples is ~2 s. If the board is handled, the average would bake
 * motion into the bias, so motion restarts the window.
 *
 * The gate measures the SPREAD (max - min) across the window, NOT the absolute
 * rate. That distinction matters: this unit has a ~21.7 deg/s bias on Y, so an
 * absolute threshold would read "moving" on every single sample and calibration
 * would never finish. Spread is independent of any constant offset — which is
 * precisely the thing we are trying to measure. */
#define AHRS_CAL_SAMPLES      (100u)
#define AHRS_CAL_SPREAD_DPS   (3.0f)

/* Guard for the Euler kinematics, which are singular at pitch = +/-90 deg
 * (gimbal lock). Clamping cos(theta) keeps the yaw/roll rates finite; a
 * quadcopter that reaches this attitude has bigger problems than estimator
 * accuracy. */
#define AHRS_COS_MIN          (0.017452f)      /* cos(89 deg) */

#define AHRS_DEG_TO_RAD       (0.017453293f)   /* pi / 180 */

/* Sanity bound on dt [s]: a scheduler hiccup or a dropped sample must not be
 * integrated as if it were real elapsed time. The IMU task runs at 50 Hz. */
#define AHRS_DT_MAX_S         (0.2f)
#define AHRS_DT_MIN_S         (0.0001f)

/* --- mounting transform -------------------------------------------------
 * Maps SENSOR axes to BODY axes. Identity for now: the airframe mounting is not
 * decided, so angles come out in the GY-521's own frame (Z up when the chip
 * faces up), which is what the bench test checks.
 *
 * flight_ctrl.h uses NED with z DOWN. For a typical "chip up, X forward, Y
 * right" mounting the body vector is (X, -Y, -Z), i.e. set Y and Z to -1.0f.
 * Set these once the sensor is bolted down, then re-verify on the bench that
 * nose-up gives positive pitch and right-roll gives positive roll. */
#define AHRS_MOUNT_SX         (1.0f)
#define AHRS_MOUNT_SY         (1.0f)
#define AHRS_MOUNT_SZ         (1.0f)

/* --- state --------------------------------------------------------------- */

static Ahrs_State s_ahrsState;
static float32    s_phi[3];          /* roll, pitch, yaw [rad]        */
static float32    s_om[3];           /* p, q, r [rad/s], bias removed */
static float32    s_bias[3];         /* gyro bias [deg/s]             */
static float32    s_calSum[3];       /* bias accumulator              */
static uint16     s_calCount;
static boolean    s_accTrusted;

void Ahrs_init(void)
{
    uint8 i;

    s_ahrsState     = AHRS_CALIBRATING;
    s_calCount  = 0u;
    s_accTrusted = FALSE;

    for (i = 0u; i < 3u; i++)
    {
        s_phi[i]    = 0.0f;
        s_om[i]     = 0.0f;
        s_bias[i]   = 0.0f;
        s_calSum[i] = 0.0f;
    }
}

/* Average the gyro while the board is still. Motion restarts the window,
 * because a biased bias is worse than none. */
static void Ahrs_calibrate(const float32 gyro[3])
{
    /* Block scope (MISRA 8.9): the window extremes are used only here.
     * static because they must survive across calls. */
    static float32 s_calMin[3];
    static float32 s_calMax[3];
    boolean moving = FALSE;
    uint8   i;

    /* Fold the sample into the running window. */
    for (i = 0u; i < 3u; i++)
    {
        if (s_calCount == 0u)
        {
            s_calMin[i] = gyro[i];
            s_calMax[i] = gyro[i];
        }
        else
        {
            if (gyro[i] < s_calMin[i]) { s_calMin[i] = gyro[i]; }
            if (gyro[i] > s_calMax[i]) { s_calMax[i] = gyro[i]; }
        }
        s_calSum[i] += gyro[i];
    }
    s_calCount++;

    /* Spread test — insensitive to the constant offset being measured. */
    for (i = 0u; i < 3u; i++)
    {
        if ((s_calMax[i] - s_calMin[i]) > AHRS_CAL_SPREAD_DPS)
        {
            moving = TRUE;
        }
    }

    if (moving != FALSE)
    {
        s_calCount = 0u;
        for (i = 0u; i < 3u; i++)
        {
            s_calSum[i] = 0.0f;
        }
    }
    else if (s_calCount >= AHRS_CAL_SAMPLES)
    {
        for (i = 0u; i < 3u; i++)
        {
            s_bias[i] = s_calSum[i] / (float32)s_calCount;
        }
        s_ahrsState = AHRS_RUNNING;
    }
    else
    {
        /* keep collecting */
    }
}

void Ahrs_update(const float32 acc[3], const float32 gyro[3], float32 dt, boolean valid)
{
    if (valid == FALSE)
    {
        /* Hold the last estimate rather than integrating garbage. Once the IMU
         * comes back (Mpu6050_read re-inits it), calibration starts over: the
         * device was power-cycled, so the old bias no longer applies. */
        s_ahrsState      = AHRS_NO_SENSOR;
        s_calCount   = 0u;
        s_accTrusted = FALSE;
    }
    else
    {
        if (s_ahrsState == AHRS_NO_SENSOR)
        {
            Ahrs_init();
        }

        if (s_ahrsState == AHRS_CALIBRATING)
        {
            Ahrs_calibrate(gyro);
        }
        else
        {
            /* Sensor -> body frame, bias removed, deg/s -> rad/s. */
            float32 ax = acc[0] * AHRS_MOUNT_SX;
            float32 ay = acc[1] * AHRS_MOUNT_SY;
            float32 az = acc[2] * AHRS_MOUNT_SZ;
            float32 p  = (gyro[0] - s_bias[0]) * AHRS_MOUNT_SX * AHRS_DEG_TO_RAD;
            float32 q  = (gyro[1] - s_bias[1]) * AHRS_MOUNT_SY * AHRS_DEG_TO_RAD;
            float32 r  = (gyro[2] - s_bias[2]) * AHRS_MOUNT_SZ * AHRS_DEG_TO_RAD;

            s_om[0] = p;
            s_om[1] = q;
            s_om[2] = r;

            if ((dt > AHRS_DT_MIN_S) && (dt < AHRS_DT_MAX_S))
            {
                float32 sinPhi = sinf(s_phi[0]);
                float32 cosPhi = cosf(s_phi[0]);
                float32 cosThe = cosf(s_phi[1]);
                float32 tanThe = tanf(s_phi[1]);
                float32 magSq;

                if ((cosThe < AHRS_COS_MIN) && (cosThe > -AHRS_COS_MIN))
                {
                    cosThe = AHRS_COS_MIN;      /* gimbal-lock guard */
                }

                /* Euler kinematics: body rates are NOT Euler-angle rates except
                 * at small angles, so use the full transform. */
                s_phi[0] += dt * (p + (((sinPhi * q) + (cosPhi * r)) * tanThe));
                s_phi[1] += dt * ((cosPhi * q) - (sinPhi * r));
                s_phi[2] += dt * (((sinPhi * q) + (cosPhi * r)) / cosThe);

                /* Gravity reference — only while |a| is near 1 g. */
                magSq = (ax * ax) + (ay * ay) + (az * az);
                if ((magSq > (AHRS_ACC_MIN_G * AHRS_ACC_MIN_G))
                    && (magSq < (AHRS_ACC_MAX_G * AHRS_ACC_MAX_G)))
                {
                    float32 phiAcc = atan2f(ay, az);
                    float32 theAcc = atan2f(-ax, sqrtf((ay * ay) + (az * az)));
                    float32 alpha  = AHRS_TAU_S / (AHRS_TAU_S + dt);
                    float32 beta   = 1.0f - alpha;

                    s_phi[0] = (alpha * s_phi[0]) + (beta * phiAcc);
                    s_phi[1] = (alpha * s_phi[1]) + (beta * theAcc);
                    /* No acc correction for yaw — it is unobservable (Ahrs.h §1). */
                    s_accTrusted = TRUE;
                }
                else
                {
                    s_accTrusted = FALSE;
                }
            }
        }
    }
}

Ahrs_State Ahrs_getState(void)
{
    return s_ahrsState;
}

void Ahrs_getAttitude(float32 phi[3])
{
    phi[0] = s_phi[0];
    phi[1] = s_phi[1];
    phi[2] = s_phi[2];
}

void Ahrs_getRates(float32 om[3])
{
    om[0] = s_om[0];
    om[1] = s_om[1];
    om[2] = s_om[2];
}

void Ahrs_getGyroBias(float32 bias[3])
{
    bias[0] = s_bias[0];
    bias[1] = s_bias[1];
    bias[2] = s_bias[2];
}

boolean Ahrs_isAccelTrusted(void)
{
    return s_accTrusted;
}
