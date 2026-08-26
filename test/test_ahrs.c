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

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_quaternion_stays_unit_norm);
    RUN_TEST(test_rotation_preserves_magnitude);
    RUN_TEST(test_body_ned_round_trip);
    RUN_TEST(test_mounting_transform_is_a_proper_rotation);
    RUN_TEST(test_level_board_reads_zero_roll_and_pitch);
    RUN_TEST(test_nose_up_reads_pitch_plus_90);
    RUN_TEST(test_right_wing_down_reads_roll_plus_90);
    RUN_TEST(test_yaw_advances_clockwise_and_stays_in_range);
    RUN_TEST(test_gravity_removed_at_rest_in_any_orientation);
    RUN_TEST(test_no_admissible_input_produces_nan);
    RUN_TEST(test_recovers_after_garbage);
    RUN_TEST(test_invalid_sample_freezes_the_estimate);
    return UNITY_END();
}
