/**********************************************************************************************************************
 * \file Ahrs.c
 * \brief Quaternion Mahony attitude estimator — see Ahrs.h.
 *********************************************************************************************************************/
#include "Ahrs.h"
#include "Nvm.h"
#include "FusionCal.h"
#include <math.h>

/* --- tuning -------------------------------------------------------------- */

/* Mahony proportional gains [1/s], in the "twoKp" form (the factor of two is
 * folded in because the correction multiplies a half-angle error).
 *
 * These set how hard each drift-free vector pulls the gyro integral back. Too
 * high and the estimate inherits the accelerometer's every vibration; too low
 * and gyro drift shows before the correction catches it. 1.0 gives roll/pitch
 * a time constant of roughly 2 s, which rides out a hand movement and still
 * settles quickly.
 *
 * The magnetometer gets HALF the accelerometer's authority on purpose. Indoors
 * it is the less trustworthy of the two by a wide margin — rebar, mains
 * wiring, the bench itself — and yaw is the only thing it fixes, so letting it
 * pull hard buys nothing and costs stability. */
#define AHRS_TWO_KP_ACC       (1.0f)
#define AHRS_TWO_KP_MAG       (0.5f)

/* Integral gain [1/s^2]: this is the gyro-bias estimator. Deliberately small,
 * because the boot calibration below has already removed the bulk (this unit
 * showed +21.7 deg/s on one axis) and all that is left for the integral to
 * track is slow thermal drift. */
#define AHRS_TWO_KI           (0.02f)

/* Accelerometer trust window [g]. Outside it the vector is contaminated by
 * real acceleration and no longer points at gravity, so the filter coasts on
 * the gyro. At rest this board measures |a| = 0.998 g. */
#define AHRS_ACC_MIN_G        (0.85f)
#define AHRS_ACC_MAX_G        (1.15f)

/* Magnetometer trust window [gauss]. Earth's field is 0.25..0.65 G worldwide
 * (~0.48 G in Munich); the band is widened to tolerate a residual hard-iron
 * offset without accepting a value that is wrong by a clean factor — which is
 * what a bad scale constant or a mis-assembled 18-bit word produces. */
#define AHRS_MAG_MIN_G        (0.15f)
#define AHRS_MAG_MAX_G        (2.0f)

/* Gyro-bias calibration: samples to average, and the motion gate. 100 samples
 * is ~2 s at the 50 Hz IMU task.
 *
 * The gate measures the SPREAD (max - min) across the window, NOT the absolute
 * rate — a large constant offset is precisely the thing being measured, so an
 * absolute threshold would read "moving" forever and calibration would never
 * finish. Motion restarts the window, because a biased bias is worse than none. */
#define AHRS_CAL_SAMPLES      (100u)
#define AHRS_CAL_SPREAD_DPS   (3.0f)

/* ...but give up after this many samples and go anyway (10 s at 50 Hz).
 *
 * Without a bound this never finishes on a board that is powered on while
 * moving -- in a vehicle, on a vibrating bench, in someone's hand. Every
 * window that exceeds the spread gate restarts, so the estimator would sit in
 * AHRS_CALIBRATING forever, and Task_Imu gates the whole navigation filter on
 * AHRS_RUNNING: no attitude, no position, no velocity, indefinitely, with
 * nothing in the output saying why.
 *
 * A bias averaged over a moving window is worse than one averaged over a still
 * one, but it is enormously better than no estimate at all -- and the Mahony
 * integral term converges the remainder within seconds once the accelerometer
 * and magnetometer start correcting. The degraded result is flagged rather than
 * hidden: Ahrs_Values.biasDegraded says the boot calibration was taken while
 * the board was moving. */
#define AHRS_CAL_MAX_SAMPLES  (500u)

/* Sanity bounds on dt [s]. The IMU task runs at 50 Hz; the upper bound matters
 * at boot, where the first measured interval is whatever the STM counter held. */
#define AHRS_DT_MIN_S         (0.0001f)
#define AHRS_DT_MAX_S         (0.2f)

#define AHRS_DEG_TO_RAD       (0.017453293f)
#define AHRS_RAD_TO_DEG       (57.29578f)
#define AHRS_GRAVITY          (9.80665f)
#define AHRS_TWO_PI           (6.2831853f)

/* Admissible band for a raw sensor sample. Generous: this rejects garbage and
 * non-finite values, it does not police physics. The ICM-42688-P is configured
 * for +/-16 g and +/-2000 deg/s, so anything past this is a corrupt transfer. */
#define AHRS_INPUT_MAX        (1.0e6f)

/* --- mounting transforms -------------------------------------------------
 * Sensor axes to BODY axes (x forward, y right, z DOWN).
 *
 * A PERMUTATION, not just sign flips. Each body axis names the sensor axis it
 * comes from and the sign to apply, so a remount is three pairs of edits and
 * the bench check in Ahrs.h re-verifies it.
 *
 * IMU — MEASURED on hardware 2026-08-26, not assumed. Holding the board
 * nose-up moved ROLL to +85 degrees and right-wing-down moved PITCH to -87,
 * i.e. the two were swapped. Working back from those two orientations:
 *
 *   nose up          forward axis points up, reads +1 g   -> sensor Y = +1
 *                    => body x (forward) = +sensor y
 *   right wing down  right axis points down, reads -1 g   -> sensor X = -1
 *                    => body y (right)   = +sensor x
 *   level            sensor Z reads +1 g, so Z is UP       -> body z = -sensor z
 *
 * Determinant is +1 (a swap of x and y is -1, times the z flip is -1, giving
 * +1), so the frame stays right-handed. That matters more than it looks: the
 * gyro is a PSEUDOvector and transforms differently from the accelerometer
 * under an improper transform, so a left-handed mapping would leave the two
 * halves of the filter disagreeing about which way is up.
 *
 * ⚠️ The previous AHRS in this tree (deleted at 50ef619) used a pure sign flip
 * (+1, -1, -1). That was correct — for the MPU-6050 on a GY-521, which is what
 * it was written and HW-verified against. It was never revisited when the
 * ICM-42688-P eval board replaced that part, so do not take those constants as
 * evidence for this sensor.
 *
 * MAG — the MMC5983MA is a SEPARATE breakout in its own orientation and gets
 * its own transform. This one is still a HYPOTHESIS: it cannot be measured
 * until the hard-iron offsets are calibrated, because uncalibrated |B| swings
 * by a factor of two with orientation (0.428..0.984 G measured) and swamps any
 * axis check. Run tools/mag_cal.py first, then verify that yaw tracks a
 * physical 90 degree rotation. A wrong mag transform shows as yaw that runs
 * backwards or refuses to settle, while roll and pitch stay perfect. */
#define AHRS_MOUNT_X_SRC      (1u)        /* body forward <- sensor Y */
#define AHRS_MOUNT_X_SGN      ( 1.0f)
#define AHRS_MOUNT_Y_SRC      (0u)        /* body right   <- sensor X */
#define AHRS_MOUNT_Y_SGN      ( 1.0f)
#define AHRS_MOUNT_Z_SRC      (2u)        /* body down    <- -sensor Z */
#define AHRS_MOUNT_Z_SGN      (-1.0f)

#define AHRS_MAG_MOUNT_X_SRC  (1u)
#define AHRS_MAG_MOUNT_X_SGN  ( 1.0f)
#define AHRS_MAG_MOUNT_Y_SRC  (0u)
#define AHRS_MAG_MOUNT_Y_SGN  ( 1.0f)
#define AHRS_MAG_MOUNT_Z_SRC  (2u)
#define AHRS_MAG_MOUNT_Z_SGN  (-1.0f)

/* Apply the IMU mounting transform. Used for the accelerometer AND the gyro —
 * both must go through the same mapping or the filter tears itself apart. */
static void ahrs_mountImu(const float32 in[3], float32 out[3])
{
    out[0] = AHRS_MOUNT_X_SGN * in[AHRS_MOUNT_X_SRC];
    out[1] = AHRS_MOUNT_Y_SGN * in[AHRS_MOUNT_Y_SRC];
    out[2] = AHRS_MOUNT_Z_SGN * in[AHRS_MOUNT_Z_SRC];
}

static void ahrs_mountMag(const float32 in[3], float32 out[3])
{
    out[0] = AHRS_MAG_MOUNT_X_SGN * in[AHRS_MAG_MOUNT_X_SRC];
    out[1] = AHRS_MAG_MOUNT_Y_SGN * in[AHRS_MAG_MOUNT_Y_SRC];
    out[2] = AHRS_MAG_MOUNT_Z_SGN * in[AHRS_MAG_MOUNT_Z_SRC];
}

/* --- state --------------------------------------------------------------- */

static float32 s_q0;               /* quaternion, body -> NED  */
static float32 s_q1;
static float32 s_q2;
static float32 s_q3;

static float32 s_fbI[3];           /* Mahony integral feedback [rad/s]        */
static float32 s_bias[3];          /* boot gyro bias, body frame [deg/s]      */
static float32 s_calSum[3];
static float32 s_calMin[3];
static float32 s_calMax[3];
static uint16  s_calCount;
static uint16  s_calTotal;      /* samples since calibration started */
static boolean s_biasDegraded;  /* deadline hit before a clean window */

/* Named s_ahrsState rather than the obvious s_state: MISRA 5.9 wants
 * internal-linkage identifiers unique across the whole program, and
 * fusion.c and src/asw/CtrlReplay.c each had their own s_state. */
static Ahrs_State s_ahrsState;

static float32 s_magB[3];          /* latched sample, BODY frame, corrected   */
static float32 s_magNorm;

/* Inverse square root. The plain form, not the famous bit-trick approximation:
 * this core has an FPU, the trick's 0.2 percent error would land straight in
 * the attitude, and nothing here is short of cycles. */
static float32 ahrs_invSqrt(float32 x)
{
    float32 r = 0.0f;

    if (x > 0.0f)
    {
        r = 1.0f / sqrtf(x);
    }
    else
    {
        /* zero or NaN: the caller re-aligns */
    }

    return r;
}

/* Rotate a BODY vector into NED: v_ned = R(q) * v_body. */
static void ahrs_bodyToNed(const float32 v[3], float32 out[3])
{
    const float32 r00 = 1.0f - (2.0f * ((s_q2 * s_q2) + (s_q3 * s_q3)));
    const float32 r01 = 2.0f * ((s_q1 * s_q2) - (s_q0 * s_q3));
    const float32 r02 = 2.0f * ((s_q1 * s_q3) + (s_q0 * s_q2));
    const float32 r10 = 2.0f * ((s_q1 * s_q2) + (s_q0 * s_q3));
    const float32 r11 = 1.0f - (2.0f * ((s_q1 * s_q1) + (s_q3 * s_q3)));
    const float32 r12 = 2.0f * ((s_q2 * s_q3) - (s_q0 * s_q1));
    const float32 r20 = 2.0f * ((s_q1 * s_q3) - (s_q0 * s_q2));
    const float32 r21 = 2.0f * ((s_q2 * s_q3) + (s_q0 * s_q1));
    const float32 r22 = 1.0f - (2.0f * ((s_q1 * s_q1) + (s_q2 * s_q2)));

    out[0] = (r00 * v[0]) + (r01 * v[1]) + (r02 * v[2]);
    out[1] = (r10 * v[0]) + (r11 * v[1]) + (r12 * v[2]);
    out[2] = (r20 * v[0]) + (r21 * v[1]) + (r22 * v[2]);
}

/* Rotate a NED vector into BODY: v_body = transpose(R(q)) * v_ned. */
static void ahrs_nedToBody(const float32 v[3], float32 out[3])
{
    const float32 r00 = 1.0f - (2.0f * ((s_q2 * s_q2) + (s_q3 * s_q3)));
    const float32 r01 = 2.0f * ((s_q1 * s_q2) - (s_q0 * s_q3));
    const float32 r02 = 2.0f * ((s_q1 * s_q3) + (s_q0 * s_q2));
    const float32 r10 = 2.0f * ((s_q1 * s_q2) + (s_q0 * s_q3));
    const float32 r11 = 1.0f - (2.0f * ((s_q1 * s_q1) + (s_q3 * s_q3)));
    const float32 r12 = 2.0f * ((s_q2 * s_q3) - (s_q0 * s_q1));
    const float32 r20 = 2.0f * ((s_q1 * s_q3) - (s_q0 * s_q2));
    const float32 r21 = 2.0f * ((s_q2 * s_q3) + (s_q0 * s_q1));
    const float32 r22 = 1.0f - (2.0f * ((s_q1 * s_q1) + (s_q2 * s_q2)));

    out[0] = (r00 * v[0]) + (r10 * v[1]) + (r20 * v[2]);
    out[1] = (r01 * v[0]) + (r11 * v[1]) + (r21 * v[2]);
    out[2] = (r02 * v[0]) + (r12 * v[1]) + (r22 * v[2]);
}

/* Build the quaternion from Euler angles (3-2-1: yaw, then pitch, then roll). */
static void ahrs_setEuler(float32 roll, float32 pitch, float32 yaw)
{
    const float32 cr = cosf(roll * 0.5f);
    const float32 sr = sinf(roll * 0.5f);
    const float32 cp = cosf(pitch * 0.5f);
    const float32 sp = sinf(pitch * 0.5f);
    const float32 cy = cosf(yaw * 0.5f);
    const float32 sy = sinf(yaw * 0.5f);

    s_q0 = (cr * cp * cy) + (sr * sp * sy);
    s_q1 = (sr * cp * cy) - (cr * sp * sy);
    s_q2 = (cr * sp * cy) + (sr * cp * sy);
    s_q3 = (cr * cp * sy) - (sr * sp * cy);
}

void Ahrs_init(void)
{
    uint8 i;

    s_q0 = 1.0f;
    s_q1 = 0.0f;
    s_q2 = 0.0f;
    s_q3 = 0.0f;

    for (i = 0u; i < 3u; i++)
    {
        s_fbI[i]    = 0.0f;
        s_bias[i]   = 0.0f;
        s_calSum[i] = 0.0f;
        s_calMin[i] = 0.0f;
        s_calMax[i] = 0.0f;
        s_magB[i]   = 0.0f;
    }

    s_calCount     = 0u;
    s_calTotal     = 0u;
    s_biasDegraded = FALSE;
    s_ahrsState    = AHRS_CALIBRATING;
    s_magNorm  = 0.0f;
}

/* Average the gyro while the board is still; motion restarts the window. */
static boolean ahrs_calibrate(const float32 gyroBody[3])
{
    boolean done   = FALSE;
    boolean moving = FALSE;
    uint8   i;

    for (i = 0u; i < 3u; i++)
    {
        if (s_calCount == 0u)
        {
            s_calMin[i] = gyroBody[i];
            s_calMax[i] = gyroBody[i];
        }
        else
        {
            if (gyroBody[i] < s_calMin[i])
            {
                s_calMin[i] = gyroBody[i];
            }
            else
            {
                /* not a new minimum */
            }

            if (gyroBody[i] > s_calMax[i])
            {
                s_calMax[i] = gyroBody[i];
            }
            else
            {
                /* not a new maximum */
            }
        }

        s_calSum[i] += gyroBody[i];

        if ((s_calMax[i] - s_calMin[i]) > AHRS_CAL_SPREAD_DPS)
        {
            moving = TRUE;
        }
        else
        {
            /* this axis is quiet */
        }
    }

    s_calCount++;

    s_calTotal++;

    if ((moving != FALSE) && (s_calTotal < AHRS_CAL_MAX_SAMPLES))
    {
        /* Throw the window away rather than bake the motion into the bias. */
        for (i = 0u; i < 3u; i++)
        {
            s_calSum[i] = 0.0f;
        }
        s_calCount = 0u;
    }
    else if ((s_calCount >= AHRS_CAL_SAMPLES)
             || (s_calTotal >= AHRS_CAL_MAX_SAMPLES))
    {
        /* Either a clean window completed, or the deadline expired and this is
         * the best that is going to be available. Divide by what was actually
         * accumulated, not by the nominal count -- at the deadline the window
         * is usually partial, and dividing by 100 regardless would scale the
         * bias down toward zero and look deceptively small. */
        const uint16 n = (s_calCount > 0u) ? s_calCount : 1u;

        if (s_calCount < AHRS_CAL_SAMPLES)
        {
            s_biasDegraded = TRUE;
        }
        else
        {
            s_biasDegraded = FALSE;
        }

        for (i = 0u; i < 3u; i++)
        {
            s_bias[i] = s_calSum[i] / (float32)n;
        }

        done = TRUE;
    }
    else
    {
        /* window still filling */
    }

    return done;
}

/* Snap the quaternion straight onto the first usable accel (and mag) sample.
 *
 * Worth the extra code: starting from identity and letting the Mahony gains
 * walk the estimate in takes several seconds, during which the vertical
 * channel is being fed a badly projected a_D. Aligning analytically means the
 * estimate is right on the first sample and the gains only ever correct drift. */
static void ahrs_align(const float32 accBody[3], float32 accNorm)
{
    const float32 recip = 1.0f / accNorm;
    const float32 ax = accBody[0] * recip;
    const float32 ay = accBody[1] * recip;
    const float32 az = accBody[2] * recip;
    float32 sinPitch = ax;
    float32 roll;
    float32 pitch;
    float32 yaw = 0.0f;

    /* Specific force at rest is [sin(theta), -cos(theta)sin(phi),
     * -cos(theta)cos(phi)] — gravity seen from the body, negated. Invert it. */
    if (sinPitch > 1.0f)
    {
        sinPitch = 1.0f;
    }
    else if (sinPitch < -1.0f)
    {
        sinPitch = -1.0f;
    }
    else
    {
        /* in range */
    }

    pitch = asinf(sinPitch);
    roll  = atan2f(-ay, -az);

    if (s_magNorm > 0.0f)
    {
        /* Tilt-compensated heading: rotate the field back into the horizontal
         * plane using the roll and pitch just found, then take its bearing.
         * Using the RAW field instead is the classic compass bug — the heading
         * then changes when the board is merely tilted. */
        const float32 cr = cosf(roll);
        const float32 sr = sinf(roll);
        const float32 cp = cosf(pitch);
        const float32 sp = sinf(pitch);
        const float32 hx = (s_magB[0] * cp) + (s_magB[1] * sr * sp)
                         + (s_magB[2] * cr * sp);
        const float32 hy = (s_magB[1] * cr) - (s_magB[2] * sr);

        yaw = atan2f(-hy, hx);
    }
    else
    {
        /* No field yet — start at zero and let the mag correction bring yaw in
         * once a sample arrives. Roll and pitch are correct either way. */
    }

    ahrs_setEuler(roll, pitch, yaw);
}

/* The Mahony correction: a rotation-error vector built from whichever
 * drift-free references are usable this sample. */
static void ahrs_errorVector(const float32 accBody[3], float32 accNorm,
                             float32 e[3], boolean *accUsed, boolean *magUsed)
{
    e[0] = 0.0f;
    e[1] = 0.0f;
    e[2] = 0.0f;
    *accUsed = FALSE;
    *magUsed = FALSE;

    if ((accNorm > AHRS_ACC_MIN_G) && (accNorm < AHRS_ACC_MAX_G))
    {
        /* Where the filter BELIEVES the specific force points: minus the NED
         * down axis, expressed in body. The cross product with what the
         * accelerometer actually measured is the rotation that reconciles the
         * two — small-angle, so no trigonometry is needed. */
        const float32 down[3] = { 0.0f, 0.0f, -1.0f };
        const float32 recip = 1.0f / accNorm;
        const float32 ax = accBody[0] * recip;
        const float32 ay = accBody[1] * recip;
        const float32 az = accBody[2] * recip;
        float32 v[3];

        ahrs_nedToBody(down, v);

        const float32 kp = FusionCal_positive(g_fusionCal.twoKpAcc, 0.0f,
                                             AHRS_TWO_KP_ACC);

        e[0] += kp * ((ay * v[2]) - (az * v[1]));
        e[1] += kp * ((az * v[0]) - (ax * v[2]));
        e[2] += kp * ((ax * v[1]) - (ay * v[0]));

        *accUsed = TRUE;
    }
    else
    {
        /* Contaminated by real acceleration — coast on the gyro. */
    }

    if ((s_magNorm > AHRS_MAG_MIN_G) && (s_magNorm < AHRS_MAG_MAX_G))
    {
        /* Rotate the measured field into NED, then flatten it: whatever the
         * horizontal part turns out to be, DEFINE it as pointing north. The
         * difference between that reference and the measurement is then a
         * rotation about the vertical only — which is the point, since the
         * magnetometer must not be allowed to touch roll or pitch. */
        const float32 recip = 1.0f / s_magNorm;
        const float32 mx = s_magB[0] * recip;
        const float32 my = s_magB[1] * recip;
        const float32 mz = s_magB[2] * recip;
        const float32 mn[3] = { mx, my, mz };
        float32 h[3];
        float32 w[3];
        float32 ref[3];

        ahrs_bodyToNed(mn, h);

        ref[0] = sqrtf((h[0] * h[0]) + (h[1] * h[1]));
        ref[1] = 0.0f;
        ref[2] = h[2];

        ahrs_nedToBody(ref, w);

        const float32 kp = FusionCal_positive(g_fusionCal.twoKpMag, 0.0f,
                                             AHRS_TWO_KP_MAG);

        e[0] += kp * ((my * w[2]) - (mz * w[1]));
        e[1] += kp * ((mz * w[0]) - (mx * w[2]));
        e[2] += kp * ((mx * w[1]) - (my * w[0]));

        *magUsed = TRUE;
    }
    else
    {
        /* No usable field — yaw coasts on the gyro and will drift. */
    }
}

void Ahrs_update(Ahrs_Values *out, const float32 acc[3], const float32 gyro[3],
                 float32 dt, boolean valid)
{
    boolean accUsed = FALSE;
    boolean magUsed = FALSE;
    uint8   i;

    if ((valid == FALSE) || (dt <= AHRS_DT_MIN_S) || (dt >= AHRS_DT_MAX_S))
    {
        /* Freeze. A stale sample or a nonsense interval integrated as though
         * it were real is how an estimator ends up confidently wrong. */
        if (valid == FALSE)
        {
            s_ahrsState = AHRS_NO_SENSOR;
            out->accNed[0] = 0.0f;
            out->accNed[1] = 0.0f;
            out->accNed[2] = 0.0f;
            out->rate[0]   = 0.0f;
            out->rate[1]   = 0.0f;
            out->rate[2]   = 0.0f;
            out->accMagG   = 0.0f;
        }
        else
        {
            /* Only the interval was unusable — keep the last projection so a
             * single scheduler hiccup does not blank the channel filters. */
        }
    }
    else if (!((acc[0] > -AHRS_INPUT_MAX) && (acc[0] < AHRS_INPUT_MAX)
            && (acc[1] > -AHRS_INPUT_MAX) && (acc[1] < AHRS_INPUT_MAX)
            && (acc[2] > -AHRS_INPUT_MAX) && (acc[2] < AHRS_INPUT_MAX)
            && (gyro[0] > -AHRS_INPUT_MAX) && (gyro[0] < AHRS_INPUT_MAX)
            && (gyro[1] > -AHRS_INPUT_MAX) && (gyro[1] < AHRS_INPUT_MAX)
            && (gyro[2] > -AHRS_INPUT_MAX) && (gyro[2] < AHRS_INPUT_MAX)))
    {
        /* A NaN or infinite sample is dropped exactly like a failed read. The
         * positive form matters: every comparison against NaN is false, so
         * negating a positive test is what rejects it. Without this, an
         * absurd acceleration propagates into accNed and out to the channel
         * filters as +/-inf. */
    }
    else
    {
        float32 accBody[3];
        float32 gyroBody[3];
        float32 accNorm;
        float32 aBodyMps2[3];
        float32 fNed[3];

        /* Sensor -> body. */
        ahrs_mountImu(acc,  accBody);
        ahrs_mountImu(gyro, gyroBody);

        accNorm = sqrtf((accBody[0] * accBody[0]) + (accBody[1] * accBody[1])
                      + (accBody[2] * accBody[2]));

        if (s_ahrsState == AHRS_NO_SENSOR)
        {
            /* The sensor came back. Re-align rather than resume from a
             * quaternion that is as old as the disconnection was long. */
            s_ahrsState = AHRS_ALIGNING;
        }
        else
        {
            /* normal progression */
        }

        if (s_ahrsState == AHRS_CALIBRATING)
        {
            if (ahrs_calibrate(gyroBody) != FALSE)
            {
                s_ahrsState = AHRS_ALIGNING;
            }
            else
            {
                /* still averaging */
            }
        }
        else
        {
            /* bias already known */
        }

        if (s_ahrsState == AHRS_ALIGNING)
        {
            if ((accNorm > AHRS_ACC_MIN_G) && (accNorm < AHRS_ACC_MAX_G))
            {
                ahrs_align(accBody, accNorm);

                for (i = 0u; i < 3u; i++)
                {
                    s_fbI[i] = 0.0f;
                }

                s_ahrsState = AHRS_RUNNING;
            }
            else
            {
                /* board is being moved; wait for it to settle */
            }
        }
        else
        {
            /* already running, or still calibrating */
        }

        if (s_ahrsState == AHRS_RUNNING)
        {
            float32 e[3];
            float32 wx;
            float32 wy;
            float32 wz;
            float32 recipNorm;

            ahrs_errorVector(accBody, accNorm, e, &accUsed, &magUsed);

            /* Gains are read every tick from the calibration block, so a tuning
             * write takes effect on the next update rather than the next flash. */
            {
                const float32 ki = FusionCal_positive(g_fusionCal.twoKi,
                                                      0.0f, AHRS_TWO_KI);
                for (i = 0u; i < 3u; i++)
                {
                    s_fbI[i] += ki * e[i] * dt;
                }
            }

            /* The integral term IS the gyro-bias estimate: a rotation error
             * that keeps pointing the same way can only be a rate offset. */

            wx = ((gyroBody[0] - s_bias[0]) * AHRS_DEG_TO_RAD) + e[0] + s_fbI[0];
            wy = ((gyroBody[1] - s_bias[1]) * AHRS_DEG_TO_RAD) + e[1] + s_fbI[1];
            wz = ((gyroBody[2] - s_bias[2]) * AHRS_DEG_TO_RAD) + e[2] + s_fbI[2];

            /* q_dot = 0.5 * q (x) [0, w]; the half is folded into h. */
            {
                const float32 h  = 0.5f * dt;
                const float32 qa = s_q0;
                const float32 qb = s_q1;
                const float32 qc = s_q2;
                const float32 qd = s_q3;

                s_q0 += h * ((-qb * wx) - (qc * wy) - (qd * wz));
                s_q1 += h * (( qa * wx) + (qc * wz) - (qd * wy));
                s_q2 += h * (( qa * wy) - (qb * wz) + (qd * wx));
                s_q3 += h * (( qa * wz) + (qb * wy) - (qc * wx));
            }

            /* Renormalise every step. The integration above is first-order, so
             * the norm creeps away on its own; letting it drift turns the
             * rotation matrix into a rotation-plus-scale and the projected
             * acceleration silently gains a gain error. */
            recipNorm = ahrs_invSqrt((s_q0 * s_q0) + (s_q1 * s_q1)
                                   + (s_q2 * s_q2) + (s_q3 * s_q3));

            if (recipNorm > 0.0f)
            {
                s_q0 *= recipNorm;
                s_q1 *= recipNorm;
                s_q2 *= recipNorm;
                s_q3 *= recipNorm;
            }
            else
            {
                /* Norm collapsed — only reachable through a NaN. Re-align. */
                s_ahrsState = AHRS_ALIGNING;
                s_q0 = 1.0f;
                s_q1 = 0.0f;
                s_q2 = 0.0f;
                s_q3 = 0.0f;
            }

            out->rate[0] = (gyroBody[0] - s_bias[0]) * AHRS_DEG_TO_RAD;
            out->rate[1] = (gyroBody[1] - s_bias[1]) * AHRS_DEG_TO_RAD;
            out->rate[2] = (gyroBody[2] - s_bias[2]) * AHRS_DEG_TO_RAD;
        }
        else
        {
            out->rate[0] = 0.0f;
            out->rate[1] = 0.0f;
            out->rate[2] = 0.0f;
        }

        /* The whole reason this file exists: specific force out of the body
         * frame and into NED, with gravity taken back out.
         *
         * At rest and level this reduces to exactly the scaffold it replaces
         * (a_D = g*(1 - acc_z)); at 20 degrees of tilt the scaffold was wrong
         * by 0.6 m/s^2 and this is not. */
        aBodyMps2[0] = accBody[0] * AHRS_GRAVITY;
        aBodyMps2[1] = accBody[1] * AHRS_GRAVITY;
        aBodyMps2[2] = accBody[2] * AHRS_GRAVITY;

        ahrs_bodyToNed(aBodyMps2, fNed);

        out->accNed[0] = fNed[0];
        out->accNed[1] = fNed[1];
        out->accNed[2] = fNed[2] + AHRS_GRAVITY;
        out->accMagG   = accNorm;
    }

    /* --- publish --------------------------------------------------------- */
    {
        float32 sinPitch = 2.0f * ((s_q0 * s_q2) - (s_q3 * s_q1));
        float32 yaw;

        if (sinPitch > 1.0f)
        {
            sinPitch = 1.0f;
        }
        else if (sinPitch < -1.0f)
        {
            sinPitch = -1.0f;
        }
        else
        {
            /* in range */
        }

        out->q[0] = s_q0;
        out->q[1] = s_q1;
        out->q[2] = s_q2;
        out->q[3] = s_q3;

        out->rollRad  = atan2f(2.0f * ((s_q0 * s_q1) + (s_q2 * s_q3)),
                               1.0f - (2.0f * ((s_q1 * s_q1) + (s_q2 * s_q2))));
        out->pitchRad = asinf(sinPitch);

        /* Declination turns MAGNETIC north into TRUE north. It belongs here
         * and not inside the filter: it is a property of the location, not of
         * the board, and folding it into the correction would make the stored
         * hard-iron offsets location-dependent too. */
        yaw = atan2f(2.0f * ((s_q0 * s_q3) + (s_q1 * s_q2)),
                     1.0f - (2.0f * ((s_q2 * s_q2) + (s_q3 * s_q3))));
        yaw += g_xcpNvm.magDeclDeg * AHRS_DEG_TO_RAD;

        /* Wrap to [0, 2pi) with fmodf rather than a pair of while loops: a
         * floating-point loop counter is a MISRA 14.1 violation, and it is a
         * fair rule here -- a loop like that is unbounded if the input is ever
         * NaN or huge, which is exactly the case worth defending against in an
         * estimator. fmodf takes one step whatever the input. */
        yaw = fmodf(yaw, AHRS_TWO_PI);

        if (yaw < 0.0f)
        {
            yaw += AHRS_TWO_PI;
        }
        else
        {
            /* already in range */
        }

        out->yawRad = yaw;

        for (i = 0u; i < 3u; i++)
        {
            /* Report the total: the constant found at boot plus whatever the
             * integral has tracked since. The sign is flipped because the
             * integral is ADDED to the gyro, so it holds minus the bias. */
            out->gyroBias[i] = s_bias[i] - (s_fbI[i] * AHRS_RAD_TO_DEG);
        }

        out->biasDegraded = (s_biasDegraded != FALSE) ? 1u : 0u;
        out->magFieldG  = s_magNorm;
        out->state      = (uint8)s_ahrsState;
        out->accTrusted = (accUsed != FALSE) ? 1u : 0u;
        out->magTrusted = (magUsed != FALSE) ? 1u : 0u;

    }
}

void Ahrs_nedToBody(const float32 vNed[3], float32 vBody[3])
{
    ahrs_nedToBody(vNed, vBody);
}

void Ahrs_setMag(const float32 mag[3], boolean valid)
{
    if (valid != FALSE)
    {
        /* Hard iron first, in SENSOR axes — that is the frame the offsets were
         * measured in, and rotating before subtracting would smear a constant
         * offset across all three. Scale factors follow (soft iron, diagonal
         * only: the full 3x3 needs an ellipsoid fit and buys little here). */
        const float32 sx = (g_xcpNvm.magScaleX > 0.0f) ? g_xcpNvm.magScaleX : 1.0f;
        const float32 sy = (g_xcpNvm.magScaleY > 0.0f) ? g_xcpNvm.magScaleY : 1.0f;
        const float32 sz = (g_xcpNvm.magScaleZ > 0.0f) ? g_xcpNvm.magScaleZ : 1.0f;
        float32 corrected[3];

        corrected[0] = (mag[0] - g_xcpNvm.magOffX) * sx;
        corrected[1] = (mag[1] - g_xcpNvm.magOffY) * sy;
        corrected[2] = (mag[2] - g_xcpNvm.magOffZ) * sz;

        ahrs_mountMag(corrected, s_magB);

        s_magNorm = sqrtf((s_magB[0] * s_magB[0]) + (s_magB[1] * s_magB[1])
                        + (s_magB[2] * s_magB[2]));
    }
    else
    {
        /* Stop trusting a field we are no longer receiving: leaving the last
         * sample latched would let a dead magnetometer keep steering yaw. */
        s_magNorm = 0.0f;
    }
}
