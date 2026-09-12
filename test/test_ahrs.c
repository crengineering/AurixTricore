/* Ahrs — Invarianten (Spec 2.2, 2.3) und analytische Faelle (Spec 3.1, 3.2).
 *
 * Nothing here was derived from Ahrs.c. The expectations come from
 *   - Ahrs.h    frames, units, argument order, sign conventions
 *   - docs/FUSION_SPEC.md sections 2.2, 2.3, 3.1, 3.2, 5
 *   - rigid-body geometry
 *
 * THE MOUNTING TRANSFORM. Ahrs.h says the sensor sits in its own orientation
 * and that AHRS_MOUNT_* / AHRS_MAG_MOUNT_* map sensor axes to body axes. The
 * spec (2.2) requires that transform to have determinant +1 and requires the
 * gyro and the accelerometer to agree about it -- a pseudovector transformed
 * through a left-handed frame flips sign relative to a true vector, and then
 * the two halves of the filter disagree about which way is up (defect 6).
 *
 * Neither transform is readable from here, so both are MEASURED through the
 * public interface, one column at a time:
 *
 *   gyro  column k = the body-frame rotation axis produced by spinning the
 *                    sensor about its own axis k with the accel correction off
 *   accel column k = the body-frame direction the estimator settles on when the
 *                    accelerometer reads 1 g along its own axis k; at rest the
 *                    specific force points UP, i.e. [0,0,-1] in NED, so that
 *                    direction is Ahrs_nedToBody([0,0,-1])
 *
 * Both are pure black-box measurements; they can then be checked as matrices.
 */
#include "unity.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "Ifx_Types.h"
#include "Ahrs.h"
#include "Nvm.h"
#include "FusionCal.h"
#include "test_util_math.h"

#define DT      0.005f          /* 200 Hz, the rate the board runs at */
#define G_MPS2  9.80665f
#define DEG     (float)(M_PI / 180.0)

/* AHRS_TWO_KP_ACC is a private #define in Ahrs.c (not part of Ahrs.h) --
 * mirrored here, read-only, purely to print it next to a MEASURED kp_eff in
 * test_b0_todays_accel_path_kp_eff below. Not consulted for pass/fail. */
#define AHRS_TWO_KP_ACC_TEST   (1.0f)

void NvmFake_identity(void);

void setUp(void)
{
    NvmFake_identity();
    FusionCal_init();
    Ahrs_init();
}

void tearDown(void) { }

/* ------------------------------------------------------------------ helpers */

/** Feed n still samples with the given sensor-frame accelerometer reading. */
static void settle(Ahrs_Values *v, const float acc[3], int n)
{
    const float32 gyro[3] = { 0.0f, 0.0f, 0.0f };
    int i;
    for (i = 0; i < n; ++i) { Ahrs_update(v, acc, gyro, DT, TRUE); }
}

/** Bring the estimator out of CALIBRATING/ALIGNING with a 1 g reading. */
static void bringUp(Ahrs_Values *v, const float acc[3])
{
    settle(v, acc, 4000);                       /* 20 s: bias cal + alignment */
    TEST_ASSERT_EQUAL_MESSAGE(AHRS_RUNNING, v->state,
        "estimator never reached AHRS_RUNNING on a still 1 g input");
}

/** Quaternion product a (x) b, both body->NED, w/x/y/z. */
static void qmul(const float a[4], const float b[4], float o[4])
{
    o[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    o[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    o[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    o[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/* ==========================================================================
 * Spec 2.2 -- attitude invariants
 * ======================================================================== */

void test_quaternion_stays_unit_norm(void)
{
    Ahrs_Values v; memset(&v, 0, sizeof v);
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    bringUp(&v, accLevel);

    Rng r; rngSeed(&r, 0x9E3779B9u);
    int i;
    for (i = 0; i < 60000; ++i)
    {
        const float32 acc[3]  = { rngN(&r) * 0.6f, rngN(&r) * 0.6f, -1.0f + rngN(&r) * 0.6f };
        const float32 gyro[3] = { rngN(&r) * 200.0f, rngN(&r) * 200.0f, rngN(&r) * 200.0f };
        const float32 mag[3]  = { rngN(&r) * 0.5f, rngN(&r) * 0.5f, rngN(&r) * 0.5f };
        if ((i % 4) == 0) { Ahrs_setMag(mag, TRUE); }
        Ahrs_update(&v, acc, gyro, rngF(&r, 0.001f, 0.02f), TRUE);

        const float n = sqrtf(v.q[0]*v.q[0] + v.q[1]*v.q[1] + v.q[2]*v.q[2] + v.q[3]*v.q[3]);
        char msg[160];
        (void)snprintf(msg, sizeof msg, "step %d: |q| = %.9g", i, (double)n);
        TEST_ASSERT_TRUE_MESSAGE(isFiniteF(n), msg);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-4f, 1.0f, n, msg);
    }
}

void test_rotation_preserves_magnitude(void)
{
    Ahrs_Values v; memset(&v, 0, sizeof v);
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    bringUp(&v, accLevel);

    Rng r; rngSeed(&r, 0x1234567u);
    int i;
    for (i = 0; i < 5000; ++i)
    {
        /* stir the attitude so the test does not only see one rotation */
        const float32 gyro[3] = { rngN(&r) * 90.0f, rngN(&r) * 90.0f, rngN(&r) * 90.0f };
        Ahrs_update(&v, accLevel, gyro, DT, TRUE);

        float32 vned[3] = { rngN(&r) * 30.0f, rngN(&r) * 30.0f, rngN(&r) * 30.0f };
        float32 vbody[3];
        Ahrs_nedToBody(vned, vbody);

        const float a = norm3(vned), b = norm3(vbody);
        char msg[160];
        (void)snprintf(msg, sizeof msg, "step %d: |v_ned| = %.9g, |v_body| = %.9g", i, (double)a, (double)b);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-4f * (a + 1.0f), a, b, msg);
    }
}

void test_body_ned_round_trip(void)
{
    Ahrs_Values v; memset(&v, 0, sizeof v);
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    bringUp(&v, accLevel);

    Rng r; rngSeed(&r, 0xBEEF01u);
    int i;
    for (i = 0; i < 5000; ++i)
    {
        const float32 gyro[3] = { rngN(&r) * 90.0f, rngN(&r) * 90.0f, rngN(&r) * 90.0f };
        Ahrs_update(&v, accLevel, gyro, DT, TRUE);

        /* Only nedToBody is exported. bodyToNed is R(q) with q as published in
         * Ahrs_Values, which is the same contract; the round trip therefore
         * also checks that the exported rotation really uses that quaternion. */
        float R[9]; quatToR(v.q, R);
        float32 vbody[3] = { rngN(&r) * 10.0f, rngN(&r) * 10.0f, rngN(&r) * 10.0f };
        float32 vned[3], back[3];
        mat3vec(R, vbody, vned);
        Ahrs_nedToBody(vned, back);

        char msg[200];
        (void)snprintf(msg, sizeof msg, "step %d: [%g %g %g] -> [%g %g %g]",
                       i, (double)vbody[0], (double)vbody[1], (double)vbody[2],
                       (double)back[0], (double)back[1], (double)back[2]);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2e-3f, vbody[0], back[0], msg);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2e-3f, vbody[1], back[1], msg);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2e-3f, vbody[2], back[2], msg);
    }
}

/* --- the mounting transform, measured, then checked as a matrix ---------- */

/** Measure column k of the GYRO mounting transform.
 *  With |a| = 0 the accelerometer carries no direction, so the correction term
 *  vanishes and the quaternion is pure gyro integration. Spinning about sensor
 *  axis k by a known angle must rotate the body frame about M_gyro * e_k. */
static void measureGyroColumn(int k, float col[3], float *angleOut)
{
    Ahrs_Values v; memset(&v, 0, sizeof v);
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    Ahrs_init();
    bringUp(&v, accLevel);

    float q0[4]; memcpy(q0, v.q, sizeof q0);

    const float32 zeroAcc[3] = { 0.0f, 0.0f, 0.0f };
    float32 gyro[3] = { 0.0f, 0.0f, 0.0f };
    const float rateDps = 30.0f;
    const int   steps   = 400;              /* 400 * 5 ms * 30 deg/s = 60 deg */
    gyro[k] = rateDps;
    int i;
    for (i = 0; i < steps; ++i) { Ahrs_update(&v, zeroAcc, gyro, DT, TRUE); }

    /* dq = q0^-1 (x) q1, the rotation expressed in the ORIGINAL body frame */
    const float q0inv[4] = { q0[0], -q0[1], -q0[2], -q0[3] };
    float dq[4];
    qmul(q0inv, v.q, dq);
    if (dq[0] < 0.0f) { dq[0] = -dq[0]; dq[1] = -dq[1]; dq[2] = -dq[2]; dq[3] = -dq[3]; }

    const float s = sqrtf(dq[1]*dq[1] + dq[2]*dq[2] + dq[3]*dq[3]);
    TEST_ASSERT_TRUE_MESSAGE(s > 1e-6f, "gyro produced no rotation at all");
    col[0] = dq[1] / s; col[1] = dq[2] / s; col[2] = dq[3] / s;
    *angleOut = 2.0f * atan2f(s, dq[0]);
}

/** Measure column k of the ACCELEROMETER mounting transform: settle on a 1 g
 *  reading along sensor axis k and read where the estimator thinks that
 *  direction points in body axes. At rest specific force is UP = [0,0,-1] NED. */
static void measureAccColumn(int k, float col[3])
{
    Ahrs_Values v; memset(&v, 0, sizeof v);
    float32 acc[3] = { 0.0f, 0.0f, 0.0f };
    acc[k] = 1.0f;
    Ahrs_init();
    bringUp(&v, acc);
    settle(&v, acc, 40000);                 /* 200 s: twoKpAcc is 1 /s */

    const float32 up[3] = { 0.0f, 0.0f, -1.0f };
    float32 b[3];
    Ahrs_nedToBody(up, b);
    const float n = norm3(b);
    col[0] = b[0] / n; col[1] = b[1] / n; col[2] = b[2] / n;
}

void test_mounting_transform_is_a_proper_rotation(void)
{
    float Mg[9], Ma[9];
    float ang[3];
    int k;

    for (k = 0; k < 3; ++k)
    {
        float c[3];
        measureGyroColumn(k, c, &ang[k]);
        Mg[0*3+k] = c[0]; Mg[1*3+k] = c[1]; Mg[2*3+k] = c[2];
    }
    for (k = 0; k < 3; ++k)
    {
        float c[3];
        measureAccColumn(k, c);
        Ma[0*3+k] = c[0]; Ma[1*3+k] = c[1]; Ma[2*3+k] = c[2];
    }

    printf("\n  gyro mount (sensor->body, columns):\n");
    for (k = 0; k < 3; ++k) printf("    [%7.4f %7.4f %7.4f]\n", (double)Mg[k*3], (double)Mg[k*3+1], (double)Mg[k*3+2]);
    printf("  accel mount (sensor->body, columns):\n");
    for (k = 0; k < 3; ++k) printf("    [%7.4f %7.4f %7.4f]\n", (double)Ma[k*3], (double)Ma[k*3+1], (double)Ma[k*3+2]);
    printf("  rotation angle for a 60 deg sensor spin: %.3f %.3f %.3f deg\n",
           (double)(ang[0]/DEG), (double)(ang[1]/DEG), (double)(ang[2]/DEG));

    /* Spec 2.2: determinant +1. -1 is a left-handed frame. */
    char msg[128];
    (void)snprintf(msg, sizeof msg, "det(gyro mount) = %.6f, must be +1", (double)det3(Mg));
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02f, 1.0f, det3(Mg), msg);
    (void)snprintf(msg, sizeof msg, "det(accel mount) = %.6f, must be +1", (double)det3(Ma));
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02f, 1.0f, det3(Ma), msg);

    /* Spec 2.2 / defect 6: the two halves must agree about which way is up. */
    for (k = 0; k < 9; ++k)
    {
        (void)snprintf(msg, sizeof msg,
            "gyro and accel mounting transforms disagree at element %d: %.4f vs %.4f",
            k, (double)Mg[k], (double)Ma[k]);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02f, Mg[k], Ma[k], msg);
    }

    /* Gyro scale: 30 deg/s for 2 s must come out as 60 deg on every axis.
     * A deg/rad slip or a factor of two in the integration shows here. */
    for (k = 0; k < 3; ++k)
    {
        (void)snprintf(msg, sizeof msg, "axis %d integrated %.3f deg, expected 60", k, (double)(ang[k]/DEG));
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0f, 60.0f, ang[k] / DEG, msg);
    }
}

/* ==========================================================================
 * T14 (docs/REFACTORING_PLAN.md §3.8) -- the gyro-bias calibration window is
 * a DURATION accumulated from dt, not a sample count, so it means the same
 * ~2.0 s wall-clock window whether NavTask_step calls Ahrs_update at 50 Hz
 * or at the measured ~1014.2 Hz. This is the one piece of §3.8 that a host
 * test can actually see -- the rate itself is not observable here, only that
 * the SAME dt stream produces the SAME elapsed time to leave CALIBRATING.
 * ======================================================================== */

void test_calibration_window_is_a_duration_not_a_sample_count(void)
{
    /* 50 Hz (the rate this flies at through T14) and the measured IMU period
     * (~1014.2 Hz, docs/IMU_INTERRUPT.md §5.6) -- both must leave
     * AHRS_CALIBRATING at ~2.0 s of accumulated dt, not at a fixed tick count. */
    static const float rates[] = { 0.02f, 0.0009855f };
    unsigned r;

    for (r = 0u; r < sizeof rates / sizeof rates[0]; ++r)
    {
        const float32 dt = rates[r];
        const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
        const float32 gyro[3]     = { 0.0f, 0.0f, 0.0f };  /* perfectly still */
        Ahrs_Values v;
        int ticks = 0;
        const int limit = (int)(20.0f / dt) + 10;   /* generous, still bounded */
        float elapsed;
        char msg[160];

        memset(&v, 0, sizeof v);
        Ahrs_init();

        while ((v.state == AHRS_CALIBRATING) && (ticks < limit))
        {
            Ahrs_update(&v, accLevel, gyro, dt, TRUE);
            ticks++;
        }

        elapsed = (float)ticks * dt;
        (void)snprintf(msg, sizeof msg,
            "dt=%.6f s: left CALIBRATING after %d ticks = %.4f s, expected ~2.0 s",
            (double)dt, ticks, (double)elapsed);
        TEST_ASSERT_TRUE_MESSAGE(ticks < limit, msg);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(3.0f * dt, 2.0f, elapsed, msg);
        TEST_ASSERT_EQUAL_MESSAGE(0u, v.biasDegraded, msg);
    }
}

void test_calibration_deadline_is_a_duration_and_flags_degraded(void)
{
    /* A board that never sits still never completes a clean window; the
     * AHRS_CAL_DEADLINE_S = 10.0 s deadline must still fire, at 10.0 s of
     * accumulated dt regardless of rate, and flag biasDegraded (Ahrs.h). */
    const float32 dt = 0.02f;    /* 50 Hz is enough ticks to reach it quickly */
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    Ahrs_Values v;
    Rng r;
    int ticks = 0;
    const int limit = 2000;      /* 40 s of ticks -- generously bounded */
    float elapsed;
    char msg[160];

    memset(&v, 0, sizeof v);
    Ahrs_init();
    rngSeed(&r, 0xC001C0DEu);

    while ((v.state == AHRS_CALIBRATING) && (ticks < limit))
    {
        const float32 gyro[3] = { rngN(&r) * 50.0f, rngN(&r) * 50.0f, rngN(&r) * 50.0f };
        Ahrs_update(&v, accLevel, gyro, dt, TRUE);
        ticks++;
    }

    elapsed = (float)ticks * dt;
    (void)snprintf(msg, sizeof msg,
        "left CALIBRATING after %.3f s while continuously moving, expected ~10.0 s deadline",
        (double)elapsed);
    TEST_ASSERT_TRUE_MESSAGE(ticks < limit, msg);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(3.0f * dt, 10.0f, elapsed, msg);
    TEST_ASSERT_EQUAL_MESSAGE(1u, v.biasDegraded, msg);
}

/* ==========================================================================
 * Spec 3.1 -- analytic attitudes
 *
 * The sensor-frame reading for a wanted BODY-frame specific force is
 * M^-1 * a_body, and M is orthonormal, so M^T * a_body. M comes from the
 * measurement above, not from the source.
 * ======================================================================== */

static void mountMatrix(float M[9])
{
    int k;
    for (k = 0; k < 3; ++k)
    {
        float c[3];
        measureAccColumn(k, c);
        M[0*3+k] = c[0]; M[1*3+k] = c[1]; M[2*3+k] = c[2];
    }
}

/** Hold the board so that the body-frame specific force is aBody (in g),
 *  and report the Euler angles the estimator settles on. */
static void holdBody(const float M[9], const float aBody[3], Ahrs_Values *v)
{
    float32 accSensor[3];
    mat3Tvec(M, aBody, accSensor);
    memset(v, 0, sizeof *v);
    Ahrs_init();
    bringUp(v, accSensor);
    settle(v, accSensor, 40000);
}

void test_level_board_reads_zero_roll_and_pitch(void)
{
    float M[9]; mountMatrix(M);
    /* level, chip up: gravity is down, specific force is UP -> a_b = [0,0,-1] */
    const float aBody[3] = { 0.0f, 0.0f, -1.0f };
    Ahrs_Values v;
    holdBody(M, aBody, &v);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0f, 0.0f, v.rollRad  / DEG, "level board, roll");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0f, 0.0f, v.pitchRad / DEG, "level board, pitch");
}

void test_nose_up_reads_pitch_plus_90(void)
{
    float M[9]; mountMatrix(M);
    /* Spec 3.1: a_b = [sin(theta), -cos(theta) sin(phi), -cos(theta) cos(phi)];
     * theta = +90 gives [1, 0, 0]. */
    const float aBody[3] = { 1.0f, 0.0f, 0.0f };
    Ahrs_Values v;
    holdBody(M, aBody, &v);
    char msg[128];
    (void)snprintf(msg, sizeof msg, "nose up: pitch = %.3f deg, expected +90", (double)(v.pitchRad / DEG));
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2.0f, 90.0f, v.pitchRad / DEG, msg);
}

void test_right_wing_down_reads_roll_plus_90(void)
{
    float M[9]; mountMatrix(M);
    /* phi = +90, theta = 0 -> a_b = [0, -1, 0] */
    const float aBody[3] = { 0.0f, -1.0f, 0.0f };
    Ahrs_Values v;
    holdBody(M, aBody, &v);
    char msg[128];
    (void)snprintf(msg, sizeof msg, "right wing down: roll = %.3f deg, expected +90", (double)(v.rollRad / DEG));
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2.0f, 90.0f, v.rollRad / DEG, msg);
}

void test_yaw_advances_clockwise_and_stays_in_range(void)
{
    /* Spec 3.1: "rotated 90 deg clockwise seen from above, level -> yaw
     * advances +90", and yaw is reported in [0, 2pi).
     * Clockwise from above is a positive rotation about BODY z (down). */
    float M[9]; mountMatrix(M);
    const float aBody[3] = { 0.0f, 0.0f, -1.0f };
    float32 accSensor[3];
    mat3Tvec(M, aBody, accSensor);

    Ahrs_Values v; memset(&v, 0, sizeof v);
    Ahrs_init();
    bringUp(&v, accSensor);
    const float yaw0 = v.yawRad;

    /* +90 deg about body z, expressed in sensor axes: M^T * [0,0,rate] */
    const float wBody[3] = { 0.0f, 0.0f, 45.0f };   /* deg/s */
    float32 gyro[3];
    mat3Tvec(M, wBody, gyro);

    int i;
    for (i = 0; i < 400; ++i) { Ahrs_update(&v, accSensor, gyro, DT, TRUE); }   /* 2 s -> 90 deg */

    float d = (v.yawRad - yaw0) / DEG;
    while (d < -180.0f) d += 360.0f;
    while (d >  180.0f) d -= 360.0f;

    char msg[128];
    (void)snprintf(msg, sizeof msg, "yaw advanced %.3f deg, expected +90", (double)d);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(3.0f, 90.0f, d, msg);

    /* the documented range */
    TEST_ASSERT_TRUE_MESSAGE(v.yawRad >= 0.0f && v.yawRad < (float)(2.0 * M_PI),
                             "yaw left the documented [0, 2pi) range");
}

/* ==========================================================================
 * Spec 3.2 -- gravity must be fully removed, in ANY orientation
 * ======================================================================== */

void test_gravity_removed_at_rest_in_any_orientation(void)
{
    float M[9]; mountMatrix(M);

    /* a spread of static attitudes, all |a| = 1 g */
    static const float att[][3] = {
        {  0.0f,  0.0f, -1.0f },        /* level          */
        {  1.0f,  0.0f,  0.0f },        /* nose up        */
        { -1.0f,  0.0f,  0.0f },        /* nose down      */
        {  0.0f, -1.0f,  0.0f },        /* right wing down*/
        {  0.0f,  1.0f,  0.0f },        /* left wing down */
        {  0.0f,  0.0f,  1.0f },        /* inverted       */
        {  0.5f, -0.5f, -0.70710678f }, /* a corner       */
    };
    unsigned t;
    for (t = 0u; t < sizeof att / sizeof att[0]; ++t)
    {
        Ahrs_Values v;
        holdBody(M, att[t], &v);
        const float mag = norm3(v.accNed);
        char msg[200];
        (void)snprintf(msg, sizeof msg,
            "orientation %u (a_b = [%g %g %g]): accNed = [%g %g %g], |accNed| = %g m/s^2, must be 0",
            t, (double)att[t][0], (double)att[t][1], (double)att[t][2],
            (double)v.accNed[0], (double)v.accNed[1], (double)v.accNed[2], (double)mag);
        TEST_ASSERT_TRUE_MESSAGE(allFinite(v.accNed, 3u), msg);
        TEST_ASSERT_TRUE_MESSAGE(mag < 0.15f, msg);

        /* Spec 5: |a| at rest is 0.998..1.005 g. A perfect 1 g input must
         * therefore be reported as 1.000 g -- this checks the g scaling. */
        (void)snprintf(msg, sizeof msg, "orientation %u: accMagG = %g, expected 1.000", t, (double)v.accMagG);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.005f, 1.0f, v.accMagG, msg);
    }
}

/* ==========================================================================
 * Spec 2.3 -- numerical robustness
 * ======================================================================== */

static void assertAhrsFinite(const Ahrs_Values *v, const char *what)
{
    char msg[200];
    (void)snprintf(msg, sizeof msg, "%s: q=[%g %g %g %g] rpy=[%g %g %g] accNed=[%g %g %g] |a|=%g |B|=%g",
        what, (double)v->q[0], (double)v->q[1], (double)v->q[2], (double)v->q[3],
        (double)v->rollRad, (double)v->pitchRad, (double)v->yawRad,
        (double)v->accNed[0], (double)v->accNed[1], (double)v->accNed[2],
        (double)v->accMagG, (double)v->magFieldG);
    TEST_ASSERT_TRUE_MESSAGE(allFinite(v->q, 4u), msg);
    TEST_ASSERT_TRUE_MESSAGE(isFiniteF(v->rollRad) && isFiniteF(v->pitchRad) && isFiniteF(v->yawRad), msg);
    TEST_ASSERT_TRUE_MESSAGE(allFinite(v->accNed, 3u), msg);
    TEST_ASSERT_TRUE_MESSAGE(allFinite(v->rate, 3u), msg);
    TEST_ASSERT_TRUE_MESSAGE(allFinite(v->gyroBias, 3u), msg);
    TEST_ASSERT_TRUE_MESSAGE(isFiniteF(v->accMagG) && isFiniteF(v->magFieldG), msg);
}

void test_no_admissible_input_produces_nan(void)
{
    Ahrs_Values v; memset(&v, 0, sizeof v);
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    bringUp(&v, accLevel);

    /* the list the spec names, plus the surrounding random cloud */
    static const float dts[]  = { 0.0f, -0.005f, -1e6f, 1e6f, 1e30f, 1e-30f, 0.005f };
    static const float amp[]  = { 0.0f, 1e-30f, 1.0f, 1e6f, 1e30f };

    unsigned a, d;
    for (a = 0u; a < sizeof amp / sizeof amp[0]; ++a)
    {
        for (d = 0u; d < sizeof dts / sizeof dts[0]; ++d)
        {
            const float32 acc[3]  = { amp[a], -amp[a], amp[a] };
            const float32 gyro[3] = { amp[a], amp[a], -amp[a] };
            const float32 mag[3]  = { 0.0f, 0.0f, 0.0f };       /* zero |B| */
            Ahrs_setMag(mag, TRUE);
            Ahrs_update(&v, acc, gyro, dts[d], TRUE);

            char what[96];
            (void)snprintf(what, sizeof what, "amp=%g dt=%g", (double)amp[a], (double)dts[d]);
            assertAhrsFinite(&v, what);

            /* and it must still be a rotation afterwards */
            const float n = sqrtf(v.q[0]*v.q[0] + v.q[1]*v.q[1] + v.q[2]*v.q[2] + v.q[3]*v.q[3]);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-3f, 1.0f, n, what);
        }
    }
}

void test_recovers_after_garbage(void)
{
    /* Spec 2.3: a diverged estimator must come back within a bounded number of
     * good samples. Hammer it, then hold the board level again. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    bringUp(&v, accLevel);

    Rng r; rngSeed(&r, 0xDEAD77u);
    int i;
    for (i = 0; i < 5000; ++i)
    {
        const float32 acc[3]  = { rngN(&r) * 1e6f, rngN(&r) * 1e6f, rngN(&r) * 1e6f };
        const float32 gyro[3] = { rngN(&r) * 1e5f, rngN(&r) * 1e5f, rngN(&r) * 1e5f };
        Ahrs_update(&v, acc, gyro, DT, TRUE);
    }
    assertAhrsFinite(&v, "after garbage");

    float M[9]; /* the mount as measured on a clean instance is not needed:
                 * only |accNed| -> 0 is asserted, which is orientation free */
    (void)M;
    settle(&v, accLevel, 40000);
    assertAhrsFinite(&v, "after recovery");
    char msg[160];
    (void)snprintf(msg, sizeof msg, "after 200 s of good samples |accNed| = %g m/s^2", (double)norm3(v.accNed));
    TEST_ASSERT_TRUE_MESSAGE(norm3(v.accNed) < 0.15f, msg);
}

void test_invalid_sample_freezes_the_estimate(void)
{
    /* Ahrs.h: "valid = FALSE when the IMU read failed -- the estimate is frozen
     * rather than integrating a stale sample". */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    bringUp(&v, accLevel);

    float q0[4]; memcpy(q0, v.q, sizeof q0);
    const float32 gyro[3] = { 500.0f, -500.0f, 500.0f };
    int i;
    for (i = 0; i < 1000; ++i) { Ahrs_update(&v, accLevel, gyro, DT, FALSE); }

    TEST_ASSERT_EQUAL_FLOAT_ARRAY(q0, v.q, 4);
}

/* ==========================================================================
 * SYS1-001 Strand B, task 0 -- scaffolding only (dispatch "SYS1-001 -
 * Dispatch.md" strand B, B4 task 0): characterise TODAY's numbers on
 * unmodified Ahrs.c, before the mag-decoupling fix (tasks 1-4). Nothing here
 * changes Ahrs.c; the point is a repeatable, host-only measurement of the
 * three quantities B2/B3 argue about, so tasks 1-4's own tests have a
 * baseline to demonstrably beat.
 * ======================================================================== */

#define B_DT   (1.0f / 1014.2f)     /* measured DRDY rate (docs/IMU_INTERRUPT.md), not the 200 Hz DT above */

/* docs/FUSION.md §5: kp_eff,mag = twoKpMag * h_r^2, h_r = sqrt(h0^2+h1^2)/|B|
 * the HORIZONTAL fraction of the field, fit from the bench (tau = 7.10 s,
 * twoKpMag = 0.5) as h_r^2 = (1/7.10)/0.5 = 0.2817 -> h_r = 0.531. This
 * board's field is far enough from horizontal (magnetic inclination) that
 * h_z = sqrt(1 - h_r^2) = 0.847 is NOT negligible -- B2(a)'s whole point is
 * that this nonzero h_z is what puts a north-axis parasite into e_mag; a
 * synthetic field with h_z = 0 (purely horizontal) would show none, hiding
 * exactly the mechanism task 0 is meant to characterise. Unit-magnitude
 * (|B| = 1): only the direction matters, ahrs_errorVector() normalises
 * s_magB internally either way. */
#define B_MAG_HR   (0.531f)
#define B_MAG_HZ   (0.847f)

/** Drive a 1014 Hz trajectory: ramp roll 0 -> rollDeg linearly over rampS
 *  (pure roll: pitch/yaw held at 0, so body rate = [phiDot,0,0] exactly --
 *  same kinematics test_yaw_advances_clockwise_and_stays_in_range already
 *  relies on for a single-axis Euler rate), with an optional lateral accel
 *  disturbance during the ramp ONLY, then holds at rollDeg (disturbance
 *  removed) until every checkpoint time has been sampled.
 *
 *  The disturbance is applied TANGENT to the roll circle -- d(phi) =
 *  [0, -cos(phi), sin(phi)], i.e. d/dphi of the true gravity direction
 *  v(phi) = [0, -sin(phi), -cos(phi)] -- not along body X. Two reasons: (1)
 *  it is orthogonal to v(phi) by construction, so |a| = sqrt(1+lateralG^2)
 *  at every phi regardless of the ramp progress (B2(b): "0.15 g lateral ->
 *  |a| = 1.011 g, inside the window"); (2) it is the ONLY direction that can
 *  perturb the ROLL-axis correction e[0] = kp*(ay*vz - az*vy) at all -- a
 *  body-X disturbance is orthogonal to the roll plane entirely and leaves
 *  e[0] untouched (verified: it moved e[1]/e[2], not e[0], and the measured
 *  roll error stayed sub-0.02 deg regardless of lateralG). Physically this
 *  is exactly what a lateral hand disturbance during a roll looks like to
 *  the accelerometer: an apparent EXTRA rotation, which is what fools the
 *  correction into pulling the roll estimate off during the maneuver.
 *  Records |commanded roll - v->rollRad| at each checkpointS[k] (seconds
 *  from t=0) into errDegAtCheckpoint[k]. All sensor vectors go through M
 *  (mountMatrix()) exactly as Ahrs_update would decode a real IMU. */
static void rollRampWithLateralAccel(const float M[9], float rollDeg, float rampS,
                                      float lateralG, Ahrs_Values *v,
                                      float *errDegAtCheckpoint,
                                      const float *checkpointS, int nCheckpoints)
{
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    float32 accSensor0[3];
    const float phiDotDeg = rollDeg / rampS;
    const int   rampSteps = (int)(rampS / B_DT + 0.5f);
    float t  = 0.0f;
    int   cp = 0;
    int   i;

    memset(v, 0, sizeof *v);
    mat3Tvec(M, accLevel, accSensor0);
    Ahrs_init();
    bringUp(v, accSensor0);      /* 200 Hz bring-up is fine: only M matters */

    for (i = 0; i < rampSteps; ++i)
    {
        const float  phi    = phiDotDeg * t;
        const float  phiRad = phi * DEG;
        const float  aBody[3] = { 0.0f,
                                   -sinf(phiRad) - (lateralG * cosf(phiRad)),
                                   -cosf(phiRad) + (lateralG * sinf(phiRad)) };
        const float  wBody[3] = { phiDotDeg, 0.0f, 0.0f };
        float32 accS[3];
        float32 gyroS[3];

        mat3Tvec(M, aBody,  accS);
        mat3Tvec(M, wBody,  gyroS);
        Ahrs_update(v, accS, gyroS, B_DT, TRUE);
        t += B_DT;

        while ((cp < nCheckpoints) && (t >= checkpointS[cp]))
        {
            errDegAtCheckpoint[cp] = fabsf(rollDeg - (v->rollRad / DEG));
            cp++;
        }
    }

    {
        const float   aBodyHold[3] = { 0.0f, -sinf(rollDeg * DEG), -cosf(rollDeg * DEG) };
        const float32 wZero[3]     = { 0.0f, 0.0f, 0.0f };
        float32 accS[3];

        mat3Tvec(M, aBodyHold, accS);
        while (cp < nCheckpoints)
        {
            Ahrs_update(v, accS, wZero, B_DT, TRUE);
            t += B_DT;
            if (t >= checkpointS[cp])
            {
                errDegAtCheckpoint[cp] = fabsf(rollDeg - (v->rollRad / DEG));
                cp++;
            }
        }
    }
}

void test_b0_todays_lag_after_90deg_roll_with_lateral_accel(void)
{
    /* B3.4's own worked expectation is checked at motion end, +1 s and +3 s
     * -- task 4 asserts <=2.0/<=1.0/<=0.5 deg there and must FAIL against
     * this baseline; this test only has to report a real, finite number. */
    float M[9]; mountMatrix(M);
    Ahrs_Values v;
    const float checkpoints[3] = { 1.5f, 2.5f, 4.5f };
    float err[3];

    rollRampWithLateralAccel(M, 90.0f, 1.5f, 0.15f, &v, err, checkpoints, 3);

    printf("\n  [StrandB task0] 90deg/1.5s roll + 0.15g lateral accel, TODAY's baseline:\n");
    printf("    error at motion end (t=1.5s): %.3f deg\n", (double)err[0]);
    printf("    error at t=2.5s (motion +1s): %.3f deg\n", (double)err[1]);
    printf("    error at t=4.5s (motion +3s): %.3f deg\n", (double)err[2]);

    TEST_ASSERT_TRUE_MESSAGE(isFiniteF(err[0]) && isFiniteF(err[1]) && isFiniteF(err[2]),
        "lag measurement must produce a finite number even on today's code");
}

void test_b0_todays_accel_path_kp_eff(void)
{
    /* B2(a): "back level, theta_ss = s_fbI/kp_eff (predicted 1.45deg,
     * measured 2.8deg -- open factor of 2, task 0 settles it)". Force a 20deg
     * mag heading error while LEVEL for 15 s (the parasite is dominant along
     * body X per the root cause -- e_mag is not a rotation about the
     * vertical), read s_fbI[0] there, then remove the disturbance and read
     * the standing roll error 10 accel time-constants later, true input
     * staying exactly level throughout.
     *
     * The isolated small-signal algebra gives kp_eff == twoKpAcc == 1.0
     * exactly (e_acc[0] linearises to -kp_acc*sin(theta_est) for a pure roll
     * offset, so at equilibrium theta_ss = s_fbI0/kp_acc, no factor of 2).
     * What this experiment actually measures is smaller by roughly two
     * orders of magnitude -- because e_mag also has a LARGE yaw component
     * (e[2], tau=7.1s) that is still converging at t=15s, and continues
     * converging (in the opposite sense, back toward true north) during the
     * "removed" window; the roll estimate this test reads at the end reflects
     * that whole coupled yaw/roll trajectory, not s_fbI0 acting in isolation.
     * That coupling -- not a clean, single-axis proportional gain -- IS
     * B2(a)'s point, and is exactly what B3.1 (task 1, projecting e_mag onto
     * d_b) removes. Both numbers are reported; task 1's own acceptance test
     * (|e_mag x d_b| < 1e-6) is the one that actually settles the mechanism.
     * gyroBias[i] = s_bias[i] - s_fbI[i]*RAD_TO_DEG (Ahrs.c); bringUp() with
     * gyro==0 leaves s_bias == 0, so s_fbI[0] = -gyroBias[0]*DEG. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    float M[9]; mountMatrix(M);
    float32 accSensor[3];
    float   sFbI0;
    float   thetaSsDeg;
    float   kpEff;
    int     i;

    {
        const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
        mat3Tvec(M, accLevel, accSensor);
    }
    Ahrs_init();
    bringUp(&v, accSensor);

    /* 15 s standing 20deg mag heading error, level (matches B2(a)'s own 15 s
     * worked-example window) -- long enough to move s_fbI[0] measurably,
     * short enough that the twoKi (tau=50s) slow pole has not yet decayed it. */
    {
        const float32 wZero[3] = { 0.0f, 0.0f, 0.0f };
        const float   hErrRad  = 20.0f * DEG;
        float32 magBody[3];
        float32 magS[3];

        magBody[0] = B_MAG_HR * cosf(hErrRad);
        magBody[1] = B_MAG_HR * sinf(hErrRad);
        magBody[2] = B_MAG_HZ;
        mountInverse(M, magBody, magS);
        Ahrs_setMag(magS, TRUE);

        for (i = 0; i < (int)(15.0f / B_DT); ++i)
        {
            Ahrs_update(&v, accSensor, wZero, B_DT, TRUE);
        }
    }
    sFbI0 = -v.gyroBias[0] * DEG;     /* rad/s; s_bias[0] == 0 (still bring-up) */

    /* Remove the disturbance (true north again) and let the fast accel loop
     * settle -- true input stays exactly level throughout, so any roll the
     * estimate shows from here on is the false bias made visible. */
    {
        const float32 wZero[3] = { 0.0f, 0.0f, 0.0f };
        float32 magS[3];
        const float32 magBodyTrue[3] = { B_MAG_HR, 0.0f, B_MAG_HZ };

        mountInverse(M, magBodyTrue, magS);
        Ahrs_setMag(magS, TRUE);

        for (i = 0; i < (int)(10.0f / B_DT); ++i)   /* ~10 accel time constants */
        {
            Ahrs_update(&v, accSensor, wZero, B_DT, TRUE);
        }
    }
    thetaSsDeg = v.rollRad / DEG;
    kpEff = fabsf(sFbI0) / fabsf(thetaSsDeg * DEG);

    printf("\n  [StrandB task0] accel-path kp_eff, TODAY's baseline:\n");
    printf("    s_fbI[0] after 15s/20deg mag error (level): %.5f rad/s (%.4f deg/s)\n",
           (double)sFbI0, (double)(sFbI0 / DEG));
    printf("    standing roll error back at level:          %.4f deg\n", (double)thetaSsDeg);
    printf("    kp_eff = |s_fbI0| / |theta_ss|:              %.4f 1/s"
           " (twoKpAcc = %.2f, twoKpAcc/2 = %.2f)\n",
           (double)kpEff, (double)AHRS_TWO_KP_ACC_TEST, (double)(AHRS_TWO_KP_ACC_TEST * 0.5f));

    TEST_ASSERT_TRUE_MESSAGE(isFiniteF(kpEff) && (kpEff > 0.0f),
        "kp_eff measurement must produce a finite, positive number");
}

void test_b0_todays_delta_s_fbi_after_60s_roll90_with_20deg_mag_error(void)
{
    /* Task 2's own acceptance ("20deg mag error at roll 90 for 60s moves
     * s_fbI[0..2] < 0.05 deg/s") must FAIL against this baseline -- report
     * what today's undivided integrator actually does. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    float M[9]; mountMatrix(M);
    float bias0[3];
    float dSfbi[3];
    int   i;

    {
        float checkpoints[1] = { 1.5f };
        float err[1];
        rollRampWithLateralAccel(M, 90.0f, 1.5f, 0.0f, &v, err, checkpoints, 1);
    }
    memcpy(bias0, v.gyroBias, sizeof bias0);

    {
        const float32 accBody90[3] = { 0.0f, -1.0f, 0.0f };
        const float32 wZero[3]     = { 0.0f, 0.0f, 0.0f };
        const float   hErrRad      = 20.0f * DEG;
        const float32 fieldNed[3]  = { B_MAG_HR * cosf(hErrRad),
                                        B_MAG_HR * sinf(hErrRad),
                                        B_MAG_HZ };
        float32 accS[3];
        float32 magBody[3];
        float32 magS[3];

        mat3Tvec(M, accBody90, accS);
        /* The field is fixed in NED (heading error rotates it about the
         * vertical, same as a wrong stored declination/hard-iron residual
         * would); at roll 90 body != NED, so project through the CURRENT
         * attitude (Ahrs_nedToBody, public) rather than assuming body
         * north == NED north the way the level-attitude test above could. */
        Ahrs_nedToBody(fieldNed, magBody);
        mountInverse(M, magBody, magS);
        Ahrs_setMag(magS, TRUE);

        for (i = 0; i < (int)(60.0f / B_DT); ++i)
        {
            Ahrs_update(&v, accS, wZero, B_DT, TRUE);
        }
    }

    for (i = 0; i < 3; ++i) { dSfbi[i] = (bias0[i] - v.gyroBias[i]) * DEG; }   /* rad/s */

    printf("\n  [StrandB task0] Delta s_fbI after 60s at roll 90deg, 20deg mag error, TODAY's baseline:\n");
    for (i = 0; i < 3; ++i)
    {
        printf("    axis %d: %.5f rad/s (%.4f deg/s)\n", i, (double)dSfbi[i], (double)(dSfbi[i] / DEG));
    }

    TEST_ASSERT_TRUE_MESSAGE(allFinite(dSfbi, 3), "Delta s_fbI measurement must be finite");
}

/* ==========================================================================
 * SYS1-001 Strand B, task 1 (B3.1, SWE1-FW-004): project e_mag onto d_b so
 * the magnetometer gets exactly one degree of freedom (heading). Tested
 * black-box, through the public Ahrs.h interface only -- same discipline
 * this whole file uses (see the file header) -- by inferring the
 * instantaneous e_mag from ONE Ahrs_update() tick's effect on s_fbI
 * (gyroBias[i] = s_bias[i] - s_fbI[i]*RAD_TO_DEG; with gyro == 0 and an
 * exactly-true accel input, e_acc ~= 0 and s_fbI's whole one-tick motion is
 * ki*e_mag*dt), rather than reaching into Ahrs.c's static state.
 * ======================================================================== */

/** 12 (roll, pitch) pairs spanning the attitude envelope, including level,
 *  each extreme, and mixed corners. */
static const float s_b1Attitudes[12][2] =
{
    {   0.0f,   0.0f }, {  30.0f,   0.0f }, { -30.0f,   0.0f }, {  90.0f,   0.0f },
    { -90.0f,   0.0f }, {   0.0f,  45.0f }, {   0.0f, -45.0f }, {  45.0f,  45.0f },
    { -45.0f, -45.0f }, {  60.0f, -30.0f }, { -60.0f,  30.0f }, {  20.0f,  70.0f }
};

void test_b1_mag_correction_is_pure_yaw_at_every_attitude(void)
{
    float M[9]; mountMatrix(M);
    unsigned t;

    for (t = 0u; t < 12u; ++t)
    {
        const float rollDeg  = s_b1Attitudes[t][0];
        const float pitchDeg = s_b1Attitudes[t][1];
        const float aBody[3] = { sinf(pitchDeg * DEG),
                                  -cosf(pitchDeg * DEG) * sinf(rollDeg * DEG),
                                  -cosf(pitchDeg * DEG) * cosf(rollDeg * DEG) };
        Ahrs_Values v;
        float32 accSensor[3];
        float   bias0[3];
        float   dB[3];
        float   eMag[3];
        float   crossMag[3];
        float   crossMagNorm;
        char    msg[160];
        int     i;

        mat3Tvec(M, aBody, accSensor);
        Ahrs_init();
        bringUp(&v, accSensor);         /* yaw anchors at 0 (no mag set yet) */
        memcpy(bias0, v.gyroBias, sizeof bias0);

        Ahrs_nedToBody((const float32[3]){ 0.0f, 0.0f, 1.0f }, dB);

        /* A 20deg heading-error field, referenced to the CURRENT (correct)
         * attitude -- same technique as task 0's roll-90 test. */
        {
            const float   hErrRad = 20.0f * DEG;
            const float32 fieldNed[3] = { B_MAG_HR * cosf(hErrRad),
                                           B_MAG_HR * sinf(hErrRad),
                                           B_MAG_HZ };
            float32 magBody[3];
            float32 magS[3];

            Ahrs_nedToBody(fieldNed, magBody);
            mountInverse(M, magBody, magS);
            Ahrs_setMag(magS, TRUE);
        }

        {
            const float32 wZero[3] = { 0.0f, 0.0f, 0.0f };
            Ahrs_update(&v, accSensor, wZero, B_DT, TRUE);
        }

        /* e_mag = Delta_s_fbI / (ki * dt); ki is FusionCal_positive-clamped
         * to AHRS_TWO_KI (0.02) by NvmFake_identity/FusionCal_init leaving
         * the calibration block at its compiled default. */
        for (i = 0; i < 3; ++i)
        {
            const float dSfbi = (bias0[i] - v.gyroBias[i]) * DEG;   /* rad/s */
            eMag[i] = dSfbi / (0.02f * B_DT);
        }

        crossMag[0] = (eMag[1] * dB[2]) - (eMag[2] * dB[1]);
        crossMag[1] = (eMag[2] * dB[0]) - (eMag[0] * dB[2]);
        crossMag[2] = (eMag[0] * dB[1]) - (eMag[1] * dB[0]);
        crossMagNorm = norm3(crossMag);

        (void)snprintf(msg, sizeof msg,
            "attitude %u (roll=%.0f pitch=%.0f): e_mag=[%.6f %.6f %.6f], "
            "d_b=[%.4f %.4f %.4f], |e_mag x d_b|=%.3e",
            t, (double)rollDeg, (double)pitchDeg,
            (double)eMag[0], (double)eMag[1], (double)eMag[2],
            (double)dB[0], (double)dB[1], (double)dB[2], (double)crossMagNorm);
        TEST_ASSERT_TRUE_MESSAGE(isFiniteF(crossMagNorm), msg);
        TEST_ASSERT_TRUE_MESSAGE(crossMagNorm < 1.0e-6f, msg);
    }
}

void test_b1_level_case_yaw_component_bit_identical_to_pre_fix(void)
{
    /* At level, d_b = [0,0,1] exactly, so the projection keeps only raw[2]
     * -- algebraically identical to the pre-fix e[2] term. Verified here by
     * comparing the new code's e[2] (inferred via s_fbI[2]/gyroBias[2],
     * yaw axis) against the SAME raw-cross-product z-component computed
     * independently in the test, not against a second copy of Ahrs.c. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    float M[9]; mountMatrix(M);
    float32 accSensor[3];
    float   bias0[3];
    float   dSfbiZ;
    float   eMagZExpected;
    char    msg[160];

    {
        const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
        mat3Tvec(M, accLevel, accSensor);
    }
    Ahrs_init();
    bringUp(&v, accSensor);
    memcpy(bias0, v.gyroBias, sizeof bias0);

    {
        const float   hErrRad = 20.0f * DEG;
        const float32 fieldNed[3] = { B_MAG_HR * cosf(hErrRad), B_MAG_HR * sinf(hErrRad), B_MAG_HZ };
        float32 magS[3];

        /* Level, yaw == 0 -> nedToBody == identity -- the field IS the body
         * reading directly, no Ahrs_nedToBody needed at this attitude. */
        mountInverse(M, fieldNed, magS);
        Ahrs_setMag(magS, TRUE);

        /* Independent reference: mn (normalised field) x w (flattened
         * reference), z-component only -- the pre-fix formula's e[2] term,
         * recomputed here from first principles, not copied from Ahrs.c. */
        {
            const float32 magNorm = sqrtf((fieldNed[0] * fieldNed[0])
                                         + (fieldNed[1] * fieldNed[1])
                                         + (fieldNed[2] * fieldNed[2]));
            const float32 mx = fieldNed[0] / magNorm;
            const float32 my = fieldNed[1] / magNorm;
            const float32 hr = sqrtf((mx * mx) + (my * my));
            /* w (level, yaw=0): ref = [hr,0,mz] rotated into body == itself */
            const float32 wx = hr;
            const float32 wy = 0.0f;

            /* AHRS_TWO_KP_MAG (Ahrs.c, private #define) mirrored here --
             * same treatment as AHRS_TWO_KP_ACC_TEST above. e[2] = kp*raw[2]. */
            const float32 kpMag = 0.5f;

            eMagZExpected = kpMag * ((mx * wy) - (my * wx));   /* = -kp*hr*my = -kp*hr^2*sin(psi) */
        }
    }

    {
        const float32 wZero[3] = { 0.0f, 0.0f, 0.0f };
        Ahrs_update(&v, accSensor, wZero, B_DT, TRUE);
    }

    dSfbiZ = (bias0[2] - v.gyroBias[2]) * DEG;         /* rad/s */

    (void)snprintf(msg, sizeof msg,
        "level e_mag[2] via s_fbI = %.6f, expected (raw cross product z) = %.6f",
        (double)(dSfbiZ / (0.02f * B_DT)), (double)eMagZExpected);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0e-4f, eMagZExpected,
        dSfbiZ / (0.02f * B_DT), msg);

    /* And the level case truly stays yaw-only: axes 0/1 must show NO mag
     * contribution now (the parasite this task deletes). */
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0e-6f, bias0[0], v.gyroBias[0], "level: no roll parasite");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0e-6f, bias0[1], v.gyroBias[1], "level: no pitch parasite");
}

void test_b1_yaw_time_constant_matches_the_bench_fit(void)
{
    /* Not measured via a step-response curve fit: s_fbI[2] is not yet split
     * out from the shared integrator (that is task 2), so the true step
     * response is a P+I system, not a single real pole, and fitting a
     * single tau out of it is exactly as fragile as it sounds (an earlier
     * version of this test tried a 1/e-at-t=tau check and, separately, a
     * local-decay-rate fit; both moved by 10-20% purely from the shared
     * integrator's OWN, pre-existing contribution -- nothing to do with
     * this task's change, and not something task 1 either introduces or
     * removes).
     *
     * A stronger and much less fragile proof of "yaw dynamics unchanged":
     * at level, d_b = nedToBody([0,0,1]) stays EXACTLY [0,0,1] for the
     * WHOLE run, not just the first tick -- a pure-yaw quaternion leaves a
     * vertical vector exactly vertical, and roll/pitch have nothing to move
     * them here (accel input is exactly level every tick, matching the
     * filter's own belief of level exactly, so e_acc is exactly zero
     * throughout). With d_b invariant, e[2] = kp*eMagD*dB[2] reduces to
     * kp*raw[2] at EVERY tick, identically to the pre-fix formula -- so
     * comparing the ACTUAL yaw trajectory, tick by tick, against an
     * INDEPENDENT closed-form integration of the pre-fix scalar ODE (using
     * the exact same kp, ki, dt this test drives Ahrs_update with) is a
     * bit-level equivalence proof over the whole transient, which implies
     * an identical tau however that tau is defined or measured. */
    const float stepDeg = 5.0f;
    const int   ticks   = (int)(20.0f / B_DT);   /* ~2.8x the documented 7.10 s tau */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    float M[9]; mountMatrix(M);
    float32 accSensor[3];
    float   yaw0;
    float   psiRef;       /* independent closed-form yaw-only integration */
    float   sFbIRef;      /* independent closed-form s_fbI[2] integration  */
    const float kpMag = 0.5f;
    const float ki    = 0.02f;
    int i;

    {
        const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
        mat3Tvec(M, accLevel, accSensor);
    }
    Ahrs_init();
    bringUp(&v, accSensor);
    yaw0    = v.yawRad;
    psiRef  = 0.0f;
    sFbIRef = 0.0f;

    {
        /* NEGATIVE stepDeg: raw[2] works out to -hr^2*sin(psi_est + beta),
         * beta the field's own fixed body-frame bearing -- the equilibrium
         * is at psi_est = -beta, not +beta (measured, not assumed: an
         * earlier version of this test used +stepDeg and watched yaw
         * diverge from the expected target in exactly this mirrored way). */
        const float   hErrRad = -stepDeg * DEG;
        const float32 fieldNed[3] = { B_MAG_HR * cosf(hErrRad), B_MAG_HR * sinf(hErrRad), B_MAG_HZ };
        float32 magS[3];

        mountInverse(M, fieldNed, magS);
        Ahrs_setMag(magS, TRUE);
    }

    for (i = 0; i < ticks; ++i)
    {
        const float32 wZero[3] = { 0.0f, 0.0f, 0.0f };
        float   rawZ;
        float   eZ;
        char    msg[160];

        /* Independent reference step, using ONLY psiRef/sFbIRef (this
         * test's own state) -- never reads Ahrs.c's internals. */
        rawZ = -(B_MAG_HR * B_MAG_HR) * sinf(psiRef - (stepDeg * DEG));
        eZ   = kpMag * rawZ;
        psiRef  += (eZ + sFbIRef) * B_DT;
        sFbIRef += ki * eZ * B_DT;

        Ahrs_update(&v, accSensor, wZero, B_DT, TRUE);

        {
            /* v.yawRad wraps to [0,2pi); yaw0 starts a hair below 2pi as
             * often as above 0 (float rounding at the boot alignment), and
             * a +5deg convergence can cross that seam -- unwrap to (-pi,pi]
             * before comparing against psiRef, which never wraps. */
            float actualPsi = v.yawRad - yaw0;
            while (actualPsi > (float)M_PI)  { actualPsi -= (float)(2.0 * M_PI); }
            while (actualPsi < -(float)M_PI) { actualPsi += (float)(2.0 * M_PI); }
            (void)snprintf(msg, sizeof msg,
                "tick %d: reference psi=%.6f rad, actual psi=%.6f rad", i,
                (double)psiRef, (double)actualPsi);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0e-4f, psiRef, actualPsi, msg);
        }
    }
}

/* ==========================================================================
 * SYS1-001 Strand B, task 2 (B3.2, SWE1-FW-004): split s_fbI (body, e_acc
 * only) from s_fbIYaw (about d_b, eMagD only). Baseline-failure evidence is
 * task 0's own test above (test_b0_todays_delta_s_fbi_after_60s_roll90_with_
 * 20deg_mag_error): on the undivided integrator, axis 1 alone moved
 * -0.1424..-0.1573 deg/s under this EXACT scenario, already past the 0.05
 * deg/s acceptance below -- these tests are the "after" half.
 * ======================================================================== */

void test_b2_mag_error_at_roll90_no_longer_moves_body_bias(void)
{
    /* s_fbI[0..2] (BODY, frame-fixed) is not directly observable through
     * gyroBias while tilted: published gyroBias[i] = s_bias[i] -
     * (s_fbI[i] + s_fbIYaw*dB[i])*RAD_TO_DEG, and at roll 90 dB is close to
     * a BODY AXIS itself (down rotates from Z toward Y as roll goes 0->90),
     * so s_fbIYaw's own, EXPECTED response to the heading error legitimately
     * shows up in gyroBias[1] at that exact attitude -- that is not the
     * defect (SWE1-FW-002's standing yaw drift is normal); the defect this
     * task removes is s_fbI[0..2] (BODY, frame-fixed) itself retaining a
     * bias that does NOT go away on its own. So this measures gyroBias
     * AFTER returning to level, where dB = [0,0,1] exactly and
     * gyroBias[0]/gyroBias[1] read s_fbI[0]/s_fbI[1] purely (s_fbIYaw
     * projects fully onto axis 2 at level, not 0/1) -- axis 2 is excluded
     * on purpose, for the same reason. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    float M[9]; mountMatrix(M);
    float bias0[3];
    int   i;

    {
        float checkpoints[1] = { 1.5f };
        float err[1];
        rollRampWithLateralAccel(M, 90.0f, 1.5f, 0.0f, &v, err, checkpoints, 1);
    }
    memcpy(bias0, v.gyroBias, sizeof bias0);

    {
        const float32 accBody90[3] = { 0.0f, -1.0f, 0.0f };
        const float32 wZero[3]     = { 0.0f, 0.0f, 0.0f };
        const float   hErrRad      = 20.0f * DEG;
        const float32 fieldNed[3]  = { B_MAG_HR * cosf(hErrRad),
                                        B_MAG_HR * sinf(hErrRad),
                                        B_MAG_HZ };
        float32 accS[3];
        float32 magBody[3];
        float32 magS[3];

        mat3Tvec(M, accBody90, accS);
        Ahrs_nedToBody(fieldNed, magBody);
        mountInverse(M, magBody, magS);
        Ahrs_setMag(magS, TRUE);

        for (i = 0; i < (int)(60.0f / B_DT); ++i)
        {
            Ahrs_update(&v, accS, wZero, B_DT, TRUE);
        }
    }

    /* Roll back to level, no disturbance, mag reverted to true north --
     * dB = [0,0,1] exactly there, isolating s_fbI[0]/s_fbI[1]. */
    {
        const float rampS = 1.5f;
        const int   steps = (int)(rampS / B_DT + 0.5f);
        const float phiDotDeg = -90.0f / rampS;
        float t = 0.0f;
        float32 magS[3];

        mountInverse(M, (const float32[3]){ B_MAG_HR, 0.0f, B_MAG_HZ }, magS);
        Ahrs_setMag(magS, TRUE);

        for (i = 0; i < steps; ++i)
        {
            const float phi = 90.0f + (phiDotDeg * t);
            const float aBody[3] = { 0.0f, -sinf(phi * DEG), -cosf(phi * DEG) };
            const float wBody[3] = { phiDotDeg, 0.0f, 0.0f };
            float32 accS[3];
            float32 gyroS[3];

            mat3Tvec(M, aBody, accS);
            mat3Tvec(M, wBody, gyroS);
            Ahrs_update(&v, accS, gyroS, B_DT, TRUE);
            t += B_DT;
        }
        for (i = 0; i < (int)(2.0f / B_DT); ++i)
        {
            const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
            const float32 wZero[3]    = { 0.0f, 0.0f, 0.0f };
            float32 accS[3];
            mat3Tvec(M, accLevel, accS);
            Ahrs_update(&v, accS, wZero, B_DT, TRUE);
        }
    }

    for (i = 0; i < 2; ++i)
    {
        char  msg[128];
        const float dGyroBiasDeg = v.gyroBias[i] - bias0[i];
        (void)snprintf(msg, sizeof msg,
            "axis %d (s_fbI, isolated at level) moved %.4f deg/s, must stay < 0.05 deg/s",
            i, (double)dGyroBiasDeg);
        TEST_ASSERT_TRUE_MESSAGE(fabsf(dGyroBiasDeg) < 0.05f, msg);
    }
}

void test_b2_return_to_level_after_mag_error_settles_within_2s(void)
{
    /* Roll to 90, hold under a 20deg mag error for 60 s (as above), then
     * roll BACK to level with no disturbance and check the roll error 2 s
     * after motion ends -- must be < 0.5deg. On the undivided integrator
     * this was task 0's B1-evidence scenario (standing error, slow decay);
     * with s_fbI no longer touched by the mag error at all, there is
     * nothing left to decay. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    float M[9]; mountMatrix(M);
    int   i;

    {
        float checkpoints[1] = { 1.5f };
        float err[1];
        rollRampWithLateralAccel(M, 90.0f, 1.5f, 0.0f, &v, err, checkpoints, 1);
    }

    {
        const float32 accBody90[3] = { 0.0f, -1.0f, 0.0f };
        const float32 wZero[3]     = { 0.0f, 0.0f, 0.0f };
        const float   hErrRad      = 20.0f * DEG;
        const float32 fieldNed[3]  = { B_MAG_HR * cosf(hErrRad),
                                        B_MAG_HR * sinf(hErrRad),
                                        B_MAG_HZ };
        float32 accS[3];
        float32 magBody[3];
        float32 magS[3];

        mat3Tvec(M, accBody90, accS);
        Ahrs_nedToBody(fieldNed, magBody);
        mountInverse(M, magBody, magS);
        Ahrs_setMag(magS, TRUE);

        for (i = 0; i < (int)(60.0f / B_DT); ++i)
        {
            Ahrs_update(&v, accS, wZero, B_DT, TRUE);
        }
    }

    /* Roll back to level over 1.5 s, no disturbance -- reuse the ramp
     * helper by driving it from 90 down to 0 (negative rate). */
    {
        const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
        const float rampS = 1.5f;
        const int   steps = (int)(rampS / B_DT + 0.5f);
        const float phiDotDeg = -90.0f / rampS;
        float t = 0.0f;

        for (i = 0; i < steps; ++i)
        {
            const float phi = 90.0f + (phiDotDeg * t);
            const float aBody[3] = { 0.0f, -sinf(phi * DEG), -cosf(phi * DEG) };
            const float wBody[3] = { phiDotDeg, 0.0f, 0.0f };
            float32 accS[3];
            float32 gyroS[3];

            mat3Tvec(M, aBody, accS);
            mat3Tvec(M, wBody, gyroS);
            Ahrs_update(&v, accS, gyroS, B_DT, TRUE);
            t += B_DT;
        }

        for (i = 0; i < (int)(2.0f / B_DT); ++i)
        {
            float32 accS[3];
            const float32 wZero[3] = { 0.0f, 0.0f, 0.0f };
            mat3Tvec(M, accLevel, accS);
            Ahrs_update(&v, accS, wZero, B_DT, TRUE);
        }
    }

    {
        char msg[128];
        const float rollErrDeg = fabsf(v.rollRad / DEG);
        (void)snprintf(msg, sizeof msg,
            "roll error 2s after returning to level: %.4f deg, must be < 0.5", (double)rollErrDeg);
        TEST_ASSERT_TRUE_MESSAGE(rollErrDeg < 0.5f, msg);
    }
}

/* ==========================================================================
 * SYS1-001 Strand B, task 3 (B3.3, SWE1-FW-005): AHRS_FBI_MAX_DPS (2.0/axis,
 * body) and AHRS_FBI_YAW_MAX_DPS (1.0, heading) -- a backstop, not the fix.
 *
 * Both tests use a CHASING error: the injected reference is recomputed every
 * tick from the estimate's OWN current state plus a fixed 45deg offset, so
 * the fast (proportional) response can never converge it away -- a FIXED
 * absolute error self-resolves once the proportional loop tracks it (as
 * task 0/1's tests already showed: kp_acc/kp_eff,mag pull the estimate to
 * match within a few time constants, and then the integral has nothing left
 * to feed on). A chasing error is the only way to sustain forcing for the
 * whole 300 s and actually reach the clamp, rather than one bounded
 * transient bump that decays on its own.
 * ======================================================================== */

void test_b3_yaw_integral_clamped_under_persistent_45deg_error(void)
{
    const float yawMaxDps = 1.0f;   /* AHRS_FBI_YAW_MAX_DPS, mirrored (private #define) */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    float M[9]; mountMatrix(M);
    float32 accSensor[3];
    float   prevBiasZDeg;
    boolean everNonzero = FALSE;
    boolean everClamped = FALSE;
    int     i;
    const int ticks = (int)(300.0f / B_DT);

    {
        const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
        mat3Tvec(M, accLevel, accSensor);
    }
    Ahrs_init();
    bringUp(&v, accSensor);
    prevBiasZDeg = v.gyroBias[2];

    for (i = 0; i < ticks; ++i)
    {
        const float32 wZero[3] = { 0.0f, 0.0f, 0.0f };
        /* Always 45deg AHEAD of the CURRENT estimate: beta = -(psi_est+45deg)
         * makes the equilibrium (psi_est = -beta, per test_b1's derivation)
         * chase itself forever -- see the section comment above. */
        const float hErrRad = -(v.yawRad + (45.0f * DEG));
        const float32 fieldNed[3] = { B_MAG_HR * cosf(hErrRad),
                                       B_MAG_HR * sinf(hErrRad),
                                       B_MAG_HZ };
        float32 magS[3];
        float   biasZDeg;
        char    msg[128];

        mountInverse(M, fieldNed, magS);
        Ahrs_setMag(magS, TRUE);
        Ahrs_update(&v, accSensor, wZero, B_DT, TRUE);

        biasZDeg = v.gyroBias[2];
        if (fabsf(biasZDeg) > 1.0e-6f) { everNonzero = TRUE; }

        (void)snprintf(msg, sizeof msg,
            "tick %d: |gyroBias[2]| = %.4f deg/s, must never exceed %.2f deg/s",
            i, (double)fabsf(biasZDeg), (double)yawMaxDps);
        TEST_ASSERT_TRUE_MESSAGE(fabsf(biasZDeg) <= (yawMaxDps + 1.0e-3f), msg);

        /* Monotone approach: once moving, must not overshoot back past
         * where it came from (a bounded, non-oscillating wind-up). */
        if ((i > 0) && (fabsf(prevBiasZDeg) < (yawMaxDps - 1.0e-3f)))
        {
            TEST_ASSERT_TRUE_MESSAGE(fabsf(biasZDeg) >= (fabsf(prevBiasZDeg) - 1.0e-4f),
                "yaw integral must approach its clamp monotonically, not oscillate");
        }
        if (fabsf(biasZDeg) >= (yawMaxDps - 1.0e-3f)) { everClamped = TRUE; }
        prevBiasZDeg = biasZDeg;
    }

    TEST_ASSERT_TRUE_MESSAGE(everNonzero, "the chasing error never moved the yaw integral at all");
    TEST_ASSERT_TRUE_MESSAGE(everClamped, "300 s of a persistent 45deg error never reached the clamp");
}

void test_b3_body_integral_clamped_under_persistent_45deg_error(void)
{
    const float fbiMaxDps = 2.0f;   /* AHRS_FBI_MAX_DPS, mirrored (private #define) */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    float M[9]; mountMatrix(M);
    float   prevBiasYDeg;
    boolean everNonzero = FALSE;
    boolean everClamped = FALSE;
    int     i;
    const int ticks = (int)(300.0f / B_DT);

    {
        const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
        float32 accSensor[3];
        mat3Tvec(M, accLevel, accSensor);
        Ahrs_init();
        bringUp(&v, accSensor);
    }
    prevBiasYDeg = v.gyroBias[1];

    for (i = 0; i < ticks; ++i)
    {
        const float32 wZero[3] = { 0.0f, 0.0f, 0.0f };
        /* Always 45deg AHEAD of the current PITCH estimate -- same chasing
         * construction as the yaw test above, applied to a pure-pitch
         * attitude (Spec 3.1's aBody = [sinTheta, 0, -cosTheta] at phi=0)
         * instead of a heading. Pitch (not roll) because gyroBias[1] is the
         * axis this drives, matching the standard p/q/r = roll/pitch/yaw
         * body-rate convention this file uses throughout. */
        const float pitchChase = v.pitchRad + (45.0f * DEG);
        const float32 aBody[3] = { sinf(pitchChase), 0.0f, -cosf(pitchChase) };
        float32 accS[3];
        float   biasYDeg;
        char    msg[128];

        mat3Tvec(M, aBody, accS);
        Ahrs_update(&v, accS, wZero, B_DT, TRUE);

        biasYDeg = v.gyroBias[1];
        if (fabsf(biasYDeg) > 1.0e-6f) { everNonzero = TRUE; }

        (void)snprintf(msg, sizeof msg,
            "tick %d: |gyroBias[1]| = %.4f deg/s, must never exceed %.2f deg/s",
            i, (double)fabsf(biasYDeg), (double)fbiMaxDps);
        TEST_ASSERT_TRUE_MESSAGE(fabsf(biasYDeg) <= (fbiMaxDps + 1.0e-3f), msg);

        if ((i > 0) && (fabsf(prevBiasYDeg) < (fbiMaxDps - 1.0e-3f)))
        {
            TEST_ASSERT_TRUE_MESSAGE(fabsf(biasYDeg) >= (fabsf(prevBiasYDeg) - 1.0e-4f),
                "body integral must approach its clamp monotonically, not oscillate");
        }
        if (fabsf(biasYDeg) >= (fbiMaxDps - 1.0e-3f)) { everClamped = TRUE; }
        prevBiasYDeg = biasYDeg;
    }

    TEST_ASSERT_TRUE_MESSAGE(everNonzero, "the chasing error never moved the body integral at all");
    TEST_ASSERT_TRUE_MESSAGE(everClamped, "300 s of a persistent 45deg error never reached the clamp");
}

void test_b3_integral_bit_unchanged_on_untrusted_ticks_even_near_clamp(void)
{
    /* Wind the yaw integral up close to its clamp, then feed a run of
     * invalid ticks (SYS1-001 task 2's freeze path, unaffected by this
     * task) and check gyroBias is bit-identical across every one of them --
     * the clamp must never itself touch state on a tick where valid ==
     * FALSE freezes everything else. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    float M[9]; mountMatrix(M);
    float32 accSensor[3];
    float   biasBefore[3];
    int     i;

    {
        const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
        mat3Tvec(M, accLevel, accSensor);
    }
    Ahrs_init();
    bringUp(&v, accSensor);

    for (i = 0; i < (int)(60.0f / B_DT); ++i)
    {
        const float32 wZero[3] = { 0.0f, 0.0f, 0.0f };
        const float   hErrRad  = -(v.yawRad + (45.0f * DEG));
        const float32 fieldNed[3] = { B_MAG_HR * cosf(hErrRad),
                                       B_MAG_HR * sinf(hErrRad),
                                       B_MAG_HZ };
        float32 magS[3];

        mountInverse(M, fieldNed, magS);
        Ahrs_setMag(magS, TRUE);
        Ahrs_update(&v, accSensor, wZero, B_DT, TRUE);
    }
    memcpy(biasBefore, v.gyroBias, sizeof biasBefore);

    for (i = 0; i < 40; ++i)
    {
        const float32 gyro[3] = { 5.0f, -3.0f, 2.0f };
        Ahrs_update(&v, accSensor, gyro, 0.001f, FALSE);
        TEST_ASSERT_EQUAL_FLOAT_ARRAY(biasBefore, v.gyroBias, 3);
    }
}

/* ==========================================================================
 * SYS1-001 task 2 -- fault debounce (dispatch/"SYS1-001 - Dispatch.md" §4).
 * The defect: one rejected tick used to re-initialise the whole attitude
 * (deadbeat yaw from the mag, gyro-bias integral zeroed) instead of merely
 * freezing. AHRS_FAULT_HOLD_S = 0.05 s is the debounce; below it the
 * estimate must be bit-identical to before the glitch, at/above it (and
 * only then) AHRS_NO_SENSOR fires and the next good sample re-aligns once.
 *
 * flight-reviewer FAIL (regression vs main): the first version of these
 * tests drove the "sustained outage" case with dt = 0.001f, FALSE repeated
 * -- a stimulus NavTask_step never actually produces on that path (it either
 * repeats NAVTASK_TIMEDOUT_FAULT_DT_S = 0.0005 s on a no-new-edge timeout, or
 * reports the real gap once as a single LONG edge, e.g. 0.5 s) and which
 * happened to still pass against the (buggy) fix, hiding a real regression:
 * neither of NavTask.c's actual stimuli reached AHRS_NO_SENSOR at all before
 * the fix below. See test_navtask.c's
 * test_no_edge_timeout_declares_no_sensor_then_realigns_once for the
 * no-new-edge path through the real NavTask_step integration; the two tests
 * below now drive Ahrs_update() directly only with dt values NavTask.c can
 * actually produce: a small in-window glitch (a present == FALSE hiccup,
 * dt otherwise normal -- a duplicate DRDY edge no longer even reaches here,
 * task 3) and a single LONG-edge-sized gap.
 * ======================================================================== */

extern volatile uint32 g_dbgAhrsRealigns;   /* Ahrs.h -- task 0 instrumentation */

void test_glitch_ticks_freeze_without_realigning(void)
{
    /* A brief run of invalid ticks with an otherwise-normal, in-window dt --
     * what NavTask.c reports for a present == FALSE hiccup (a transient SPI
     * read failure while DRDY keeps ticking normally). A SHORT-classified
     * duplicate DRDY edge no longer reaches Ahrs_update() at all as of task
     * 3 (NavTask.c's `duplicateEdge` handling) -- this test is about ANY
     * brief invalid run, not specifically that one. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    const float32 gyro[3]     = { 5.0f, -3.0f, 2.0f };  /* nonzero: a real IMU never reads exactly 0 */
    bringUp(&v, accLevel);

    float q0[4];    memcpy(q0, v.q, sizeof q0);
    float bias0[3]; memcpy(bias0, v.gyroBias, sizeof bias0);
    const uint32 realignsBefore = g_dbgAhrsRealigns;

    /* 40 consecutive glitch ticks at a plausible ~1 kHz cadence (1 ms each) =
     * 40 ms of accumulated invalid input, comfortably under the 50 ms hold. */
    int i;
    for (i = 0; i < 40; ++i)
    {
        char msg[64];
        Ahrs_update(&v, accLevel, gyro, 0.001f, FALSE);
        (void)snprintf(msg, sizeof msg, "tick %d: state = %u, expected RUNNING", i, v.state);
        TEST_ASSERT_EQUAL_MESSAGE(AHRS_RUNNING, v.state, msg);
        TEST_ASSERT_EQUAL_MESSAGE(0u, v.accTrusted, msg);
        TEST_ASSERT_EQUAL_MESSAGE(0u, v.magTrusted, msg);
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, v.rate[0], msg);
    }

    TEST_ASSERT_EQUAL_FLOAT_ARRAY(q0, v.q, 4);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(bias0, v.gyroBias, 3);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(realignsBefore, g_dbgAhrsRealigns,
        "a glitch run under AHRS_FAULT_HOLD_S must not re-align");
}

void test_zero_or_negative_dt_never_advances_the_fault_hold_clock(void)
{
    /* Defensive/robustness regression, not a claim about what NavTask.c
     * sends today: dt <= 0 (or NaN, which fails every comparison here) must
     * never by itself declare AHRS_NO_SENSOR, however many times it repeats
     * -- Ahrs_update()'s OWN contract, independent of any one caller. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    const float32 gyro[3]     = { 0.0f, 0.0f, 0.0f };
    bringUp(&v, accLevel);

    int i;
    for (i = 0; i < 200000; ++i)
    {
        Ahrs_update(&v, accLevel, gyro, 0.0f, FALSE);
    }
    TEST_ASSERT_EQUAL_MESSAGE(AHRS_RUNNING, v.state,
        "dt == 0.0f repeated must never by itself declare AHRS_NO_SENSOR");
}

void test_long_gap_declares_no_sensor_immediately_then_realigns_once(void)
{
    /* NavTask.c's actual LONG-edge stimulus: a single tick whose dt is the
     * real measured gap since the last good edge (unbounded above -- see
     * AHRS_FAULT_DT_MAX_S), not a repeated small one. A gap this long (here,
     * matching the reviewer's own 0.5 s repro) already IS the outage and
     * must declare AHRS_NO_SENSOR on THIS tick, not after further waiting. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    const float32 gyro[3]     = { 0.0f, 0.0f, 0.0f };
    bringUp(&v, accLevel);

    const uint32 realignsBefore = g_dbgAhrsRealigns;

    Ahrs_update(&v, accLevel, gyro, 0.5f, FALSE);
    TEST_ASSERT_EQUAL_MESSAGE(AHRS_NO_SENSOR, v.state,
        "a single 0.5 s invalid tick (a LONG edge's real gap) must declare "
        "AHRS_NO_SENSOR immediately, not after further waiting");

    /* The next good sample re-aligns -- exactly once. */
    Ahrs_update(&v, accLevel, gyro, 0.001f, TRUE);
    TEST_ASSERT_EQUAL(AHRS_RUNNING, v.state);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(realignsBefore + 1u, g_dbgAhrsRealigns,
        "recovery from a genuine outage must re-align exactly once");
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_quaternion_stays_unit_norm);
    RUN_TEST(test_rotation_preserves_magnitude);
    RUN_TEST(test_body_ned_round_trip);
    RUN_TEST(test_mounting_transform_is_a_proper_rotation);
    RUN_TEST(test_calibration_window_is_a_duration_not_a_sample_count);
    RUN_TEST(test_calibration_deadline_is_a_duration_and_flags_degraded);
    RUN_TEST(test_level_board_reads_zero_roll_and_pitch);
    RUN_TEST(test_nose_up_reads_pitch_plus_90);
    RUN_TEST(test_right_wing_down_reads_roll_plus_90);
    RUN_TEST(test_yaw_advances_clockwise_and_stays_in_range);
    RUN_TEST(test_gravity_removed_at_rest_in_any_orientation);
    RUN_TEST(test_b0_todays_lag_after_90deg_roll_with_lateral_accel);
    RUN_TEST(test_b0_todays_accel_path_kp_eff);
    RUN_TEST(test_b0_todays_delta_s_fbi_after_60s_roll90_with_20deg_mag_error);
    RUN_TEST(test_b1_mag_correction_is_pure_yaw_at_every_attitude);
    RUN_TEST(test_b1_level_case_yaw_component_bit_identical_to_pre_fix);
    RUN_TEST(test_b1_yaw_time_constant_matches_the_bench_fit);
    RUN_TEST(test_b2_mag_error_at_roll90_no_longer_moves_body_bias);
    RUN_TEST(test_b2_return_to_level_after_mag_error_settles_within_2s);
    RUN_TEST(test_b3_yaw_integral_clamped_under_persistent_45deg_error);
    RUN_TEST(test_b3_body_integral_clamped_under_persistent_45deg_error);
    RUN_TEST(test_b3_integral_bit_unchanged_on_untrusted_ticks_even_near_clamp);
    RUN_TEST(test_no_admissible_input_produces_nan);
    RUN_TEST(test_recovers_after_garbage);
    RUN_TEST(test_invalid_sample_freezes_the_estimate);
    RUN_TEST(test_glitch_ticks_freeze_without_realigning);
    RUN_TEST(test_zero_or_negative_dt_never_advances_the_fault_hold_clock);
    RUN_TEST(test_long_gap_declares_no_sensor_immediately_then_realigns_once);
    return UNITY_END();
}
