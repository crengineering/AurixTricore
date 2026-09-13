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
#include "AhrsLatch.h"
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

/* ==========================================================================
 * SYS1-001 B9 (2026-09-13) -- the mag mount table was a HYPOTHESIS never
 * checked away from level (only the horizontal rotation sense was verified,
 * and both the old and the new mapping preserve that). A rigid sensor triad
 * in a locally-uniform field sees a CONSTANT angle between the magnetic
 * field and gravity, regardless of the board's attitude -- that is the
 * check the level-only tests could never make. Six-position window means
 * (measured 2026-09-12, evidence 0F542580C307FA1E,
 * Measurement Data/2026-09-12_6pos_r3_window_means.json), fed through the
 * PRODUCTION mag path (Ahrs_setMag: hard-iron subtraction + the real
 * AHRS_MAG_MOUNT_* transform, read back via g_magLatch) and the production
 * IMU mount (measured, mountMatrix()).
 * ======================================================================== */

void test_mag_mount_field_gravity_angle_is_constant(void)
{
    /* sensor-frame magRaw_G / accRaw_g, in JSON order (level, roll_p90,
     * roll_p90b, roll_m90, roll_m90b, nose_up, nose_down, level_end). */
    static const float32 magRawG[8][3] = {
        {  0.05246f, -0.34019f, -0.41305f },
        { -0.00378f, -0.48328f, -1.01070f },
        { -0.01235f, -0.48752f, -1.01703f },
        { -0.06635f,  0.33103f, -0.62561f },
        { -0.00900f,  0.31426f, -0.63662f },
        { -0.58533f, -0.26785f, -0.59756f },
        {  0.15009f, -0.28623f, -1.07743f },
        {  0.06558f, -0.32375f, -0.41028f },
    };
    static const float32 accRawG[8][3] = {
        { -0.01772f, -0.05938f,  0.99486f },
        { -0.99869f, -0.06778f, -0.00534f },
        { -1.00056f, -0.03295f, -0.02981f },
        {  0.97494f,  0.16589f,  0.11123f },
        {  0.99163f,  0.02788f,  0.07277f },
        { -0.02193f,  0.99352f,  0.05267f },
        { -0.00343f, -0.99390f, -0.14970f },
        { -0.01208f, -0.06262f,  0.99353f },
    };
    static const float32 hardIronG[3] = { -0.1940f, -0.0722f, -0.8510f };
    const unsigned n = 8u;
    float M[9];
    float angleDeg[8];
    float sum, sumSq, mean, variance, std;
    unsigned i;
    char msg[192];

    mountMatrix(M);           /* real, measured IMU mount -- unaffected by B9 */
    g_xcpNvm.magOffX = hardIronG[0];
    g_xcpNvm.magOffY = hardIronG[1];
    g_xcpNvm.magOffZ = hardIronG[2];

    sum = 0.0f; sumSq = 0.0f;
    for (i = 0u; i < n; ++i)
    {
        float32 accBody[3];
        float32 gravityDown[3];
        float32 magBody[3];
        float   dot, nAcc, nMag, cosA;

        /* accel: production mount, forward direction (mat3vec, not the
         * inverse) -- M's column k IS the body direction for sensor axis k
         * (mountMatrix()'s own comment), so M*sensorVec = bodyVec directly. */
        mat3vec(M, accRawG[i], accBody);
        gravityDown[0] = -accBody[0];
        gravityDown[1] = -accBody[1];
        gravityDown[2] = -accBody[2];

        /* mag: the ACTUAL production path -- hard-iron subtraction then the
         * real AHRS_MAG_MOUNT_* transform, read back via the shared latch
         * exactly as NavTask_step would. */
        Ahrs_setMag(magRawG[i], TRUE);
        magBody[0] = g_magLatch.magB[0];
        magBody[1] = g_magLatch.magB[1];
        magBody[2] = g_magLatch.magB[2];

        dot  = (magBody[0] * gravityDown[0]) + (magBody[1] * gravityDown[1]) + (magBody[2] * gravityDown[2]);
        nMag = sqrtf((magBody[0] * magBody[0]) + (magBody[1] * magBody[1]) + (magBody[2] * magBody[2]));
        nAcc = sqrtf((gravityDown[0] * gravityDown[0]) + (gravityDown[1] * gravityDown[1]) + (gravityDown[2] * gravityDown[2]));
        cosA = dot / (nMag * nAcc);
        if (cosA >  1.0f) { cosA =  1.0f; }
        if (cosA < -1.0f) { cosA = -1.0f; }
        angleDeg[i] = acosf(cosA) / DEG;

        sum   += angleDeg[i];
        sumSq += angleDeg[i] * angleDeg[i];
    }

    mean     = sum / (float)n;
    variance = (sumSq / (float)n) - (mean * mean);
    std      = sqrtf((variance > 0.0f) ? variance : 0.0f);

    printf("\n  NEW mag mount: angle(field,gravity) mean=%.2f deg std=%.2f deg\n",
           (double)mean, (double)std);

    (void)snprintf(msg, sizeof msg,
        "NEW mag mount: angle(field,gravity) mean=%.2f deg std=%.2f deg "
        "(want mean in [20,40], std < 6 -- B9: mean 33, spread 3.4-4.9 across 3 datasets)",
        (double)mean, (double)std);
    TEST_ASSERT_TRUE_MESSAGE(std < 6.0f, msg);
    TEST_ASSERT_TRUE_MESSAGE((mean >= 20.0f) && (mean <= 40.0f), msg);

    /* The OLD mapping (AHRS_MAG_MOUNT_* before B9: X_SRC=1/+1, Y_SRC=0/+1,
     * Z_SRC=2/-1 -- identical to the IMU mount) must NOT look constant: it
     * ranked 32nd of 48 in the B9 search (spread 29.5 deg, field pointing
     * UP). The mount is a compile-time table, not a runtime parameter, so
     * this is reproduced directly on the SAME hard-iron-corrected samples
     * rather than by rebuilding with the old constants -- this is what
     * proves the fix is what makes the assertion above pass, not a
     * coincidence of the acceptance band. */
    {
        float oldSum = 0.0f, oldSumSq = 0.0f, oldMean, oldVariance, oldStd;

        for (i = 0u; i < n; ++i)
        {
            float32 accBody[3];
            float32 gravityDown[3];
            float32 corrected[3];
            float32 oldMagBody[3];
            float   dot, nAcc, nMag, cosA, ang;

            mat3vec(M, accRawG[i], accBody);
            gravityDown[0] = -accBody[0];
            gravityDown[1] = -accBody[1];
            gravityDown[2] = -accBody[2];

            corrected[0] = magRawG[i][0] - hardIronG[0];
            corrected[1] = magRawG[i][1] - hardIronG[1];
            corrected[2] = magRawG[i][2] - hardIronG[2];
            oldMagBody[0] =  corrected[1];   /* old X_SRC=1, +1 */
            oldMagBody[1] =  corrected[0];   /* old Y_SRC=0, +1 */
            oldMagBody[2] = -corrected[2];   /* old Z_SRC=2, -1 */

            dot  = (oldMagBody[0] * gravityDown[0]) + (oldMagBody[1] * gravityDown[1]) + (oldMagBody[2] * gravityDown[2]);
            nMag = sqrtf((oldMagBody[0] * oldMagBody[0]) + (oldMagBody[1] * oldMagBody[1]) + (oldMagBody[2] * oldMagBody[2]));
            nAcc = sqrtf((gravityDown[0] * gravityDown[0]) + (gravityDown[1] * gravityDown[1]) + (gravityDown[2] * gravityDown[2]));
            cosA = dot / (nMag * nAcc);
            if (cosA >  1.0f) { cosA =  1.0f; }
            if (cosA < -1.0f) { cosA = -1.0f; }
            ang = acosf(cosA) / DEG;

            oldSum   += ang;
            oldSumSq += ang * ang;
        }

        oldMean     = oldSum / (float)n;
        oldVariance = (oldSumSq / (float)n) - (oldMean * oldMean);
        oldStd      = sqrtf((oldVariance > 0.0f) ? oldVariance : 0.0f);

        printf("  OLD mag mount: angle(field,gravity) mean=%.2f deg std=%.2f deg\n",
               (double)oldMean, (double)oldStd);

        (void)snprintf(msg, sizeof msg,
            "OLD mag mount would give mean=%.2f deg std=%.2f deg -- must NOT look "
            "constant (std > 20), proving the fix is load-bearing",
            (double)oldMean, (double)oldStd);
        TEST_ASSERT_TRUE_MESSAGE(oldStd > 20.0f, msg);
    }
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

/* SYS1-001 strand B task 15 (SWE1-FW-009): AHRS_INPUT_MAX (1.0e6f, rejected
 * only NaN/inf) is now AHRS_GYRO_MAX_DPS (3000) / AHRS_ACC_MAX_INPUT_G (25)
 * -- explicit regression that a literal NaN or infinite sample on EITHER
 * sensor still freezes the estimate exactly as before the rename (q/yaw/
 * gyroBias untouched -- the empty "else if" branch this falls into leaves
 * every static as it was, same freeze-by-omission Ahrs.c documents). */
void test_nan_or_inf_gyro_or_acc_is_still_rejected_after_the_rename(void)
{
    Ahrs_Values v; memset(&v, 0, sizeof v);
    const float32 accLevel[3]  = { 0.0f, 0.0f, -1.0f };
    const float32 gyroQuiet[3] = { 1.0f, -2.0f, 0.5f };
    const float32 nan = 0.0f / 0.0f;
    const float32 inf = 1.0f / 0.0f;
    int i;

    bringUp(&v, accLevel);
    for (i = 0; i < 100; ++i) { Ahrs_update(&v, accLevel, gyroQuiet, DT, TRUE); }

    for (i = 0; i < 6; ++i)
    {
        float32 accBad[3]  = { accLevel[0], accLevel[1], accLevel[2] };
        float32 gyroBad[3] = { gyroQuiet[0], gyroQuiet[1], gyroQuiet[2] };
        float32 yawBefore  = v.yawRad;
        float32 biasBefore[3];
        char    msg[64];

        memcpy(biasBefore, v.gyroBias, sizeof biasBefore);

        /* i = 0..2: NaN on acc axis i; i = 3..5: inf on gyro axis i-3. */
        if (i < 3) { accBad[i] = nan; } else { gyroBad[i - 3] = inf; }

        (void)snprintf(msg, sizeof msg, "case %d (%s)", i, (i < 3) ? "NaN on acc" : "inf on gyro");
        Ahrs_update(&v, accBad, gyroBad, DT, TRUE);
        TEST_ASSERT_TRUE_MESSAGE(isFiniteF(v.yawRad), msg);
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(yawBefore, v.yawRad, msg);
        TEST_ASSERT_EQUAL_FLOAT_ARRAY_MESSAGE(biasBefore, v.gyroBias, 3, msg);
    }
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
                                      const float *checkpointS, int nCheckpoints,
                                      float *intWDtOut)
{
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    float32 accSensor0[3];
    const float phiDotDeg = rollDeg / rampS;
    const int   rampSteps = (int)(rampS / B_DT + 0.5f);
    float t  = 0.0f;
    int   cp = 0;
    int   i;
    float intWDt = 0.0f;

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
        intWDt += ((float)v->accWeightPct / 100.0f) * B_DT;
        t += B_DT;

        while ((cp < nCheckpoints) && (t >= checkpointS[cp]))
        {
            errDegAtCheckpoint[cp] = fabsf(rollDeg - (v->rollRad / DEG));
            cp++;
        }
    }

    if (intWDtOut != NULL)
    {
        *intWDtOut = intWDt;
    }
    else
    {
        /* caller does not need the budget figure */
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

/* Task 11b (review round 1 major finding): test_b0_todays_* ran against the
 * CURRENT build, not a frozen baseline -- at HEAD they were printing
 * POST-fix numbers while still claiming to characterise "today's" (pre-fix)
 * behaviour. Converted to regression tests: the 59e9fca numbers (measured
 * on unmodified Ahrs.c, before any strand B code change, independently
 * reproduced by the reviewer) are now recorded CONSTANTS, and each test
 * asserts HEAD is better by the design margin -- not merely "finite". */
#define B0_BASELINE_LAG_END_DEG      (6.676f)
#define B0_BASELINE_LAG_P1S_DEG      (2.505f)
#define B0_BASELINE_LAG_P3S_DEG      (0.349f)
#define B0_BASELINE_ROLL_ERR_DEG     (-1.2562f)
#define B0_BASELINE_DSFBI_AXIS0_DPS  (-0.0064f)
#define B0_BASELINE_DSFBI_AXIS2_DPS  (0.0055f)

/* Task 12c (B6.6 review round 2): only the TRAPEZOID pre-strand-B baseline
 * was ever recorded above -- the half-sine driver (rollRampHalfSine
 * WithLateralAccel) did not exist at 59e9fca. Measured the same way the
 * reviewer measured the trapezoid figures: unmodified Ahrs.c/Ahrs.h checked
 * out from 59e9fca, compiled standalone against the SAME half-sine
 * trajectory math this file uses today (mount transform unchanged since
 * that commit -- git diff 59e9fca..HEAD on AhrsLatch.h/FusionCal.h/
 * SharedRam.h/Nvm.h is empty), same 90 deg/1.5 s + 0.15 g profile. The old
 * code has no rate gate at all (hard |a| window only), so it is barely
 * sensitive to the PROFILE SHAPE -- unsurprising that its half-sine number
 * lands close to its trapezoid number (6.717 vs 6.676): with |a| staying
 * inside the window throughout either profile, the accelerometer pulls at
 * full gain the whole time regardless of how the rate arrived there. */
#define B0_BASELINE_HALFSINE_END_DEG  (6.717f)
#define B0_BASELINE_HALFSINE_P1S_DEG  (2.520f)
#define B0_BASELINE_HALFSINE_P3S_DEG  (0.351f)

void test_b0_regression_lag_after_90deg_roll_with_lateral_accel(void)
{
    /* Baseline (59e9fca, unmodified Ahrs.c): 6.676/2.505/0.349 deg. Task 11's
     * own acceptance (<=2.0/<=1.0/<=0.5 deg) IS the design margin here --
     * already exercised directly by test_b4_lag_after_90deg_roll_with_
     * lateral_accel_meets_thresholds; this test's job is to also pin the
     * OLD number down as a recorded constant, not just check the new one. */
    float M[9]; mountMatrix(M);
    Ahrs_Values v;
    const float checkpoints[3] = { 1.5f, 2.5f, 4.5f };
    float err[3];
    char  msg[160];

    rollRampWithLateralAccel(M, 90.0f, 1.5f, 0.15f, &v, err, checkpoints, 3, NULL);

    (void)snprintf(msg, sizeof msg,
        "motion-end %.3f deg (baseline %.3f), +1s %.3f deg (baseline %.3f), "
        "+3s %.3f deg (baseline %.3f) -- task 11 target <= 2.0/1.0/0.5",
        (double)err[0], (double)B0_BASELINE_LAG_END_DEG,
        (double)err[1], (double)B0_BASELINE_LAG_P1S_DEG,
        (double)err[2], (double)B0_BASELINE_LAG_P3S_DEG);
    TEST_ASSERT_TRUE_MESSAGE(err[0] < B0_BASELINE_LAG_END_DEG, msg);
    TEST_ASSERT_TRUE_MESSAGE(err[1] < B0_BASELINE_LAG_P1S_DEG, msg);
    TEST_ASSERT_TRUE_MESSAGE(err[2] <= B0_BASELINE_LAG_P3S_DEG, msg);
    TEST_ASSERT_TRUE_MESSAGE(err[0] <= 2.0f, msg);
    TEST_ASSERT_TRUE_MESSAGE(err[1] <= 1.0f, msg);
    TEST_ASSERT_TRUE_MESSAGE(err[2] <= 0.5f, msg);
}

void test_b0_regression_standing_roll_error_after_mag_error_removed(void)
{
    /* Task 11b: the "kp_eff" NUMBER this test used to assert on was vacuous
     * -- "finite and positive" holds trivially on both the broken and the
     * fixed estimator, so it never actually distinguished them (the
     * flight-architect is separately rewording that acceptance clause; no
     * kp_eff number is invented or asserted here). What DOES distinguish
     * them, and is exactly what the reviewer independently reproduced, is
     * the STANDING ROLL ERROR itself: force a 20deg mag heading error while
     * LEVEL for 15 s, remove it, and read the roll the estimate settles to
     * 10 accel time-constants later while the true input stays exactly
     * level throughout. Baseline (59e9fca): -1.2562 deg. At HEAD (B3.1's
     * projection removes the north-axis parasite that caused it): must be
     * under 0.05 deg -- the reviewer measured 0.000. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    float M[9]; mountMatrix(M);
    float32 accSensor[3];
    float   thetaSsDeg;
    int     i;

    {
        const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
        mat3Tvec(M, accLevel, accSensor);
    }
    Ahrs_init();
    bringUp(&v, accSensor);

    /* 15 s standing 20deg mag heading error, level (matches B2(a)'s own 15 s
     * worked-example window). */
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

    {
        char msg[160];
        (void)snprintf(msg, sizeof msg,
            "standing roll error %.4f deg (baseline %.4f), must be < 0.05 deg",
            (double)thetaSsDeg, (double)B0_BASELINE_ROLL_ERR_DEG);
        TEST_ASSERT_TRUE_MESSAGE(fabsf(thetaSsDeg) < fabsf(B0_BASELINE_ROLL_ERR_DEG), msg);
        TEST_ASSERT_TRUE_MESSAGE(fabsf(thetaSsDeg) < 0.05f, msg);
    }
}

void test_b0_regression_delta_s_fbi_after_60s_roll90_with_20deg_mag_error(void)
{
    /* Baseline (59e9fca, undivided integrator): axis0 -0.0064 deg/s, axis2
     * +0.0055 deg/s. Axis1 is deliberately NOT asserted here (baseline
     * -0.1424..-0.1573 deg/s): at roll 90, dB (Ahrs.c) sits close to the
     * BODY Y axis, so s_fbIYaw's own, CORRECT heading response legitimately
     * projects mostly onto gyroBias[1] at this exact attitude -- that is
     * SWE1-FW-002's normal standing yaw behaviour, not the defect, and is
     * exactly why task 2's own tests (test_b2_mag_error_at_roll90_no_longer_
     * moves_body_bias) measure body bias AFTER a clean return to level
     * instead, where dB = [0,0,1] isolates axis 0/1 from axis 2's response. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    float M[9]; mountMatrix(M);
    float bias0[3];
    float dSfbi[3];
    int   i;

    {
        float checkpoints[1] = { 1.5f };
        float err[1];
        rollRampWithLateralAccel(M, 90.0f, 1.5f, 0.0f, &v, err, checkpoints, 1, NULL);
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

    {
        const float axis0Dps = dSfbi[0] / DEG;
        const float axis2Dps = dSfbi[2] / DEG;
        char msg[200];

        (void)snprintf(msg, sizeof msg,
            "axis0 %.5f deg/s (baseline %.4f), axis1 %.5f deg/s (not asserted, "
            "see comment), axis2 %.5f deg/s (baseline %.4f) -- target < 0.001 deg/s",
            (double)axis0Dps, (double)B0_BASELINE_DSFBI_AXIS0_DPS,
            (double)(dSfbi[1] / DEG),
            (double)axis2Dps, (double)B0_BASELINE_DSFBI_AXIS2_DPS);
        TEST_ASSERT_TRUE_MESSAGE(allFinite(dSfbi, 3), msg);
        TEST_ASSERT_TRUE_MESSAGE(fabsf(axis0Dps) < fabsf(B0_BASELINE_DSFBI_AXIS0_DPS), msg);
        TEST_ASSERT_TRUE_MESSAGE(fabsf(axis2Dps) < fabsf(B0_BASELINE_DSFBI_AXIS2_DPS), msg);
        TEST_ASSERT_TRUE_MESSAGE(fabsf(axis0Dps) < 0.001f, msg);
        TEST_ASSERT_TRUE_MESSAGE(fabsf(axis2Dps) < 0.001f, msg);
    }
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
         * reading directly, no Ahrs_nedToBody needed at this attitude.
         * magMountInverse, not mountInverse(M, ...): this is a bit-identical
         * comparison against an exact body-frame target, which needs the
         * REAL mag mount (SYS1-001 B9 changed it away from the IMU mount). */
        magMountInverse(fieldNed, magS);
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

            /* AHRS_TWO_KP_MAG (Ahrs.c, private #define) mirrored here,
             * read-only, purely to reproduce e[2] = kp*raw[2] independently. */
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

        /* Level, yaw == 0: fieldNed IS the wanted body-frame reading, so this
         * needs the REAL mag mount (magMountInverse), not the IMU mount M
         * (SYS1-001 B9) -- same reasoning as the level test above. */
        magMountInverse(fieldNed, magS);
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
        rollRampWithLateralAccel(M, 90.0f, 1.5f, 0.0f, &v, err, checkpoints, 1, NULL);
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
        rollRampWithLateralAccel(M, 90.0f, 1.5f, 0.0f, &v, err, checkpoints, 1, NULL);
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
 * SYS1-001 Strand B, task 4 (B3.4, SWE1-FW-006): continuous accel weight
 * w_acc = w_norm*w_rate replacing the hard |a| window.
 * ======================================================================== */

void test_b4_lag_after_90deg_roll_with_lateral_accel_meets_thresholds(void)
{
    /* Task 0's baseline (unmodified Ahrs.c): 6.68 / 2.51 / 0.35 deg.
     *
     * HISTORY: task 4's original constants (w_rate full trust <= 30 deg/s,
     * zero at 120 deg/s, instantaneous |gyro|) measured ~5.4 / ~2.0 / ~0.28
     * deg here -- FAILING the first two clauses, reported as an open finding
     * rather than silently retuned (see git history, commit a670da3). Task
     * 11 (review round 1) tightened both constants (15/45 deg/s, zero at
     * 60 deg/s) and replaced the instantaneous |gyro| with a 50 ms low-pass
     * (AHRS_GYRO_LP_TAU_S, Ahrs.c) -- measured 0.52/0.20/0.03 deg, all three
     * clauses comfortably passing.
     *
     * Task 12b (B6.5) adds the post-manoeuvre hold-off (AHRS_ACC_HOLDOFF_S)
     * to fix the HALF-SINE case below; the trapezoid's own budget
     * INTEGRAL(w dt) barely changes (0.064 s vs the 0.267 s clause), so this
     * test is asserted against the LIMITS ONLY, per B6.5's own prediction
     * that the trapezoid's +1s figure WORSENS slightly (0.20 -> ~0.26 deg,
     * still comfortably inside <= 1.0) as a side effect of holding the
     * accelerometer out a little longer at the tail of the ramp -- not a
     * regression to chase, and not asserted bit-identical to task 11. */
    float M[9]; mountMatrix(M);
    Ahrs_Values v;
    const float checkpoints[3] = { 1.5f, 2.5f, 4.5f };
    float err[3];
    float intWDt;
    char  msg[128];

    rollRampWithLateralAccel(M, 90.0f, 1.5f, 0.15f, &v, err, checkpoints, 3, &intWDt);

    printf("\n  [StrandB task12b] 90deg/1.5s constant-rate (trapezoid) roll + 0.15g lateral accel, hold-off active:\n");
    printf("    error at motion end (t=1.5s): %.3f deg (target <= 2.0)\n", (double)err[0]);
    printf("    error at t=2.5s (motion +1s): %.3f deg (target <= 1.0)\n", (double)err[1]);
    printf("    error at t=4.5s (motion +3s): %.3f deg (target <= 0.5)\n", (double)err[2]);
    printf("    INTEGRAL(w dt) over the motion: %.4f s (budget <= 0.267 s)\n", (double)intWDt);

    (void)snprintf(msg, sizeof msg, "motion-end error %.3f deg, target <= 2.0", (double)err[0]);
    TEST_ASSERT_TRUE_MESSAGE(err[0] <= 2.0f, msg);
    (void)snprintf(msg, sizeof msg, "+1s error %.3f deg, target <= 1.0", (double)err[1]);
    TEST_ASSERT_TRUE_MESSAGE(err[1] <= 1.0f, msg);
    (void)snprintf(msg, sizeof msg, "+3s error %.3f deg, target <= 0.5", (double)err[2]);
    TEST_ASSERT_TRUE_MESSAGE(err[2] <= 0.5f, msg);
}

/** Same trajectory as rollRampWithLateralAccel, but a half-sine (smooth
 *  accel/decel) velocity profile instead of a constant rate: phi(t) =
 *  rollDeg*(1-cos(pi*t/rampS))/2, so phi(0)=0, phi(rampS)=rollDeg, zero
 *  velocity at both ends, peak velocity rollDeg*pi/(2*rampS) at t=rampS/2
 *  (94.2 deg/s for 90deg/1.5s -- higher than the 60 deg/s constant-rate
 *  case, not lower; this profile is NOT a softer test). Same lateral-
 *  disturbance construction (tangent to the roll circle) as the constant-
 *  rate helper. */
/** Same trajectory as rollRampHalfSineWithLateralAccel (below), plus an
 *  optional out-param (task 12c, B6.6 acceptance (a3)/(v)):
 *  *everZeroWeightOut is set TRUE if accWeightPct ever reads exactly 0
 *  during the ramp -- the only way that can happen is either the |a| edge
 *  (w_norm) or the hold-off (s_accHoldS), and a sub-60 profile never
 *  reaches the hold-off's arming knee, so this is the test's proxy for
 *  "the hold-off never armed". */
static void rollRampHalfSineWithLateralAccelEx(const float M[9], float rollDeg, float rampS,
                                                float lateralG, Ahrs_Values *v,
                                                float *errDegAtCheckpoint,
                                                const float *checkpointS, int nCheckpoints,
                                                float *intWDtOut, boolean *everZeroWeightOut)
{
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    float32 accSensor0[3];
    const int   rampSteps = (int)(rampS / B_DT + 0.5f);
    float   t  = 0.0f;
    int     cp = 0;
    int     i;
    float   intWDt = 0.0f;
    boolean everZeroWeight = FALSE;

    memset(v, 0, sizeof *v);
    mat3Tvec(M, accLevel, accSensor0);
    Ahrs_init();
    bringUp(v, accSensor0);

    for (i = 0; i < rampSteps; ++i)
    {
        const float phi       = rollDeg * (1.0f - cosf((float)M_PI * t / rampS)) / 2.0f;
        const float phiDotDeg = rollDeg * (float)M_PI / (2.0f * rampS)
                               * sinf((float)M_PI * t / rampS);
        const float phiRad    = phi * DEG;
        const float aBody[3]  = { 0.0f,
                                   -sinf(phiRad) - (lateralG * cosf(phiRad)),
                                   -cosf(phiRad) + (lateralG * sinf(phiRad)) };
        const float wBody[3]  = { phiDotDeg, 0.0f, 0.0f };
        float32 accS[3];
        float32 gyroS[3];

        mat3Tvec(M, aBody, accS);
        mat3Tvec(M, wBody, gyroS);
        Ahrs_update(v, accS, gyroS, B_DT, TRUE);
        intWDt += ((float)v->accWeightPct / 100.0f) * B_DT;
        if (v->accWeightPct == 0u) { everZeroWeight = TRUE; }
        t += B_DT;

        while ((cp < nCheckpoints) && (t >= checkpointS[cp]))
        {
            errDegAtCheckpoint[cp] = fabsf(rollDeg - (v->rollRad / DEG));
            cp++;
        }
    }

    if (intWDtOut != NULL)
    {
        *intWDtOut = intWDt;
    }
    else
    {
        /* caller does not need the budget figure */
    }
    if (everZeroWeightOut != NULL)
    {
        *everZeroWeightOut = everZeroWeight;
    }
    else
    {
        /* caller does not need the hold-off proxy */
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

static void rollRampHalfSineWithLateralAccel(const float M[9], float rollDeg, float rampS,
                                              float lateralG, Ahrs_Values *v,
                                              float *errDegAtCheckpoint,
                                              const float *checkpointS, int nCheckpoints,
                                              float *intWDtOut)
{
    rollRampHalfSineWithLateralAccelEx(M, rollDeg, rampS, lateralG, v,
                                        errDegAtCheckpoint, checkpointS, nCheckpoints,
                                        intWDtOut, NULL);
}

void test_b0_regression_halfsine_lag_after_90deg_roll_with_lateral_accel(void)
{
    /* B6.6 (task 12c): the half-sine's own pre-strand-B (59e9fca) baseline,
     * measured standalone -- see B0_BASELINE_HALFSINE_*'s comment above for
     * how. Same "pin the OLD number, assert HEAD is better" contract as
     * test_b0_regression_lag_after_90deg_roll_with_lateral_accel. */
    float M[9]; mountMatrix(M);
    Ahrs_Values v;
    const float checkpoints[3] = { 1.5f, 2.5f, 4.5f };
    float err[3];
    char  msg[160];

    rollRampHalfSineWithLateralAccel(M, 90.0f, 1.5f, 0.15f, &v, err, checkpoints, 3, NULL);

    (void)snprintf(msg, sizeof msg,
        "motion-end %.3f deg (baseline %.3f), +1s %.3f deg (baseline %.3f), "
        "+3s %.3f deg (baseline %.3f) -- target <= 2.0/1.0/0.5",
        (double)err[0], (double)B0_BASELINE_HALFSINE_END_DEG,
        (double)err[1], (double)B0_BASELINE_HALFSINE_P1S_DEG,
        (double)err[2], (double)B0_BASELINE_HALFSINE_P3S_DEG);
    TEST_ASSERT_TRUE_MESSAGE(err[0] < B0_BASELINE_HALFSINE_END_DEG, msg);
    TEST_ASSERT_TRUE_MESSAGE(err[1] < B0_BASELINE_HALFSINE_P1S_DEG, msg);
    TEST_ASSERT_TRUE_MESSAGE(err[2] <= B0_BASELINE_HALFSINE_P3S_DEG, msg);
    TEST_ASSERT_TRUE_MESSAGE(err[0] <= 2.0f, msg);
    TEST_ASSERT_TRUE_MESSAGE(err[1] <= 1.0f, msg);
    TEST_ASSERT_TRUE_MESSAGE(err[2] <= 0.5f, msg);
}

void test_b11_halfsine_profile_meets_thresholds(void)
{
    /* Task 11 acceptance: BOTH the constant-rate profile (test above) AND
     * this half-sine profile (peak ~94 deg/s, not a softer case) must meet
     * <=2.0/1.0/0.5 deg.
     *
     * Task 11 alone FAILED here (reported, not silently forced green):
     * measured 2.84/1.06/0.15 deg -- worse than the constant-rate case
     * (0.52/0.20/0.03) even though this profile's peak rate (94 deg/s)
     * exceeds the constant case's 60 deg/s throughout. Root cause (B6.2/B6.5):
     * NOT accel lag -- with w_acc at 0 for the whole high-rate middle
     * portion, the attitude free-integrates the commanded gyro rate
     * essentially exactly -- but a small, PERSISTENT rate bias baked into
     * the body integrator (s_fbI) during the transition windows at the
     * START and END of the motion, where the 50 ms low-pass has not yet
     * pushed w_acc to 0 (or has already let it back up), so a PARTIALLY
     * weighted eAcc, computed against a DISTURBED accel reading, still
     * charges s_fbI. A half-sine spends 7.7x longer than a trapezoid in
     * that partial-weight band (0.498 s vs 0.064 s of INTEGRAL(w dt)) for
     * the same 90 deg/1.5 s motion.
     *
     * Task 12b (B6.5) closes it with a flat AHRS_ACC_HOLDOFF_S (0.3 s) once
     * the low-passed rate has reached the upper knee (60 deg/s), asymmetric
     * in time rather than a third rate parameter -- see Ahrs.c. Architect's
     * prediction: 1.54/0.77/0.10 deg. */
    float M[9]; mountMatrix(M);
    Ahrs_Values v;
    const float checkpoints[3] = { 1.5f, 2.5f, 4.5f };
    float err[3];
    float intWDt;
    char  msg[128];

    rollRampHalfSineWithLateralAccel(M, 90.0f, 1.5f, 0.15f, &v, err, checkpoints, 3, &intWDt);

    printf("\n  [StrandB task12b] 90deg/1.5s half-sine roll (peak ~94deg/s) + 0.15g lateral accel, hold-off active:\n");
    printf("    error at motion end (t=1.5s): %.3f deg (target <= 2.0, task-11-only measured 2.84)\n", (double)err[0]);
    printf("    error at t=2.5s (motion +1s): %.3f deg (target <= 1.0, task-11-only measured 1.06)\n", (double)err[1]);
    printf("    error at t=4.5s (motion +3s): %.3f deg (target <= 0.5, task-11-only measured 0.15)\n", (double)err[2]);
    printf("    INTEGRAL(w dt) over the motion: %.4f s (budget <= 0.267 s)\n", (double)intWDt);

    (void)snprintf(msg, sizeof msg, "motion-end error %.3f deg, target <= 2.0", (double)err[0]);
    TEST_ASSERT_TRUE_MESSAGE(err[0] <= 2.0f, msg);
    (void)snprintf(msg, sizeof msg, "+1s error %.3f deg, target <= 1.0", (double)err[1]);
    TEST_ASSERT_TRUE_MESSAGE(err[1] <= 1.0f, msg);
    (void)snprintf(msg, sizeof msg, "+3s error %.3f deg, target <= 0.5", (double)err[2]);
    TEST_ASSERT_TRUE_MESSAGE(err[2] <= 0.5f, msg);
}

void test_b11_old_constants_would_not_have_suppressed_60dps_roll(void)
{
    /* Task 11's whole reason, as pure arithmetic rather than a rerun of the
     * whole roll: at a SUSTAINED 60 deg/s (this scenario's rate), the OLD
     * constants (30 deg/s full trust, 90 deg/s span, zero at 120 deg/s) let
     * w_rate = 0.667 through -- comfortably nonzero, which is exactly why
     * the pre-task-11 code (commit a670da3) measured 5.43/2.04/0.28 deg
     * against the <=2.0/1.0/0.5 target and failed the first two clauses.
     * The NEW constants (15/45, zero at 60 deg/s) drop this to exactly 0
     * at steady state. The roll-level proof that the new constants actually
     * fix the lag is test_b4_lag_after_90deg_roll_with_lateral_accel_meets_
     * thresholds and test_b11_halfsine_profile_meets_thresholds above,
     * both green; this test only pins down WHY the old ones could not have
     * worked, as a permanent regression guard against re-widening the span
     * back past 60 deg/s without noticing. */
    /* AHRS_ACC_RATE_FULL_DPS/SPAN_DPS (Ahrs.c, private #defines) mirrored
     * here, read-only, same treatment as every other private-constant
     * mirror in this file (e.g. B_MAG_HR). */
    const float newFullDps = 15.0f;
    const float newSpanDps = 45.0f;
    const float oldFullDps = 30.0f;
    const float oldSpanDps = 90.0f;
    const float sustainedRateDps = 60.0f;
    float wRateOld = 1.0f - ((sustainedRateDps - oldFullDps) / oldSpanDps);
    float wRateNew = 1.0f - ((sustainedRateDps - newFullDps) / newSpanDps);
    char msg[160];

    if (wRateOld > 1.0f) { wRateOld = 1.0f; }
    if (wRateOld < 0.0f) { wRateOld = 0.0f; }
    if (wRateNew > 1.0f) { wRateNew = 1.0f; }
    if (wRateNew < 0.0f) { wRateNew = 0.0f; }

    (void)snprintf(msg, sizeof msg,
        "old constants: w_rate(60deg/s) = %.3f (measured baseline a670da3: "
        "5.43/2.04/0.28 deg, FAILED <=2.0/1.0/0.5); new constants: w_rate = %.3f",
        (double)wRateOld, (double)wRateNew);
    TEST_ASSERT_TRUE_MESSAGE(wRateOld > 0.5f, msg);       /* old: barely suppressed at all */
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, wRateNew, msg); /* new: fully suppressed */
}

/* ==========================================================================
 * SYS1-001 Strand B, task 12b (B6.5, SWE1-FW-006): AHRS_ACC_HOLDOFF_S.
 * ======================================================================== */

void test_b12b_holdoff_never_arms_below_60dps(void)
{
    /* B6.5: the hold-off's arm condition is gyroLpDps >= 60 deg/s
     * (AHRS_ACC_RATE_FULL_DPS + AHRS_ACC_RATE_SPAN_DPS) exactly -- anything
     * sustained BELOW it must never force w_acc to exactly zero via
     * s_accHoldS; only the ordinary w_norm*w_rate ramp may bring it low,
     * and that ramp itself only reaches exactly zero AT 60 deg/s. Two
     * sustained rates just inside the partial-trust band (15..60 deg/s),
     * held long enough (>> 3*AHRS_GYRO_LP_TAU_S) for the low-pass to settle
     * asymptotically toward -- and, being a non-overshooting one-pole, never
     * past -- the sustained value: accWeightPct must stay nonzero at every
     * single tick, which it could not if the hold-off had armed. */
    float M[9]; mountMatrix(M);
    Ahrs_Values v;
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    float32 accSensor[3];
    static const float rates[2] = { 40.0f, 59.0f };
    int r;

    mat3Tvec(M, accLevel, accSensor);

    for (r = 0; r < 2; ++r)
    {
        const float32 gyro[3] = { rates[r], 0.0f, 0.0f };
        const int ticks = (int)(2.0f / DT);
        int i;

        Ahrs_init();
        bringUp(&v, accSensor);

        for (i = 0; i < ticks; ++i)
        {
            char msg[144];

            Ahrs_update(&v, accSensor, gyro, DT, TRUE);

            (void)snprintf(msg, sizeof msg,
                "sustained %.1f deg/s (< 60 deg/s arm threshold), tick %d: "
                "accWeightPct = %u must never be forced to 0 by the hold-off",
                (double)rates[r], i, (unsigned)v.accWeightPct);
            TEST_ASSERT_TRUE_MESSAGE(v.accWeightPct > 0u, msg);
        }
    }
}

/* ==========================================================================
 * SYS1-001 strand B task 12c (B6.6, review round 2 major, SWE1-FW-006):
 * the hold-off's RELEASE condition was a LATCH -- "hold, no countdown while
 * 15 < gyroLp < 60" never releases for a sustained rate inside that band
 * (measured: 60913/60913 ticks at accWeightPct = 0 over a 60 s, 30 deg/s
 * turn). Fixed to a wall-clock duration since the rate was last AT OR ABOVE
 * the arming knee, for every input, no band branch. ======================= */

void test_b12c_arm_then_sustained_turn_recovers_within_bound(void)
{
    /* B6.6 acceptance (a3)/task 12c (i): the exact regression scenario. 0.5 s
     * at 80 deg/s arms the hold; a SUSTAINED 30 deg/s turn follows. Under
     * task 12b's three-way logic this never released (30 deg/s sits inside
     * the 15-60 deg/s "hold, no countdown" band forever). Task 12c must
     * recover to the ramp value (w_rate(30) = 1-(30-15)/45 = 0.667 -> 67%)
     * within 0.5 s of the rate leaving >=60 deg/s, and never drop back to 0
     * for the rest of a 60 s sustained turn. */
    float M[9]; mountMatrix(M);
    Ahrs_Values v;
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    float32 accSensor[3];
    const float32 gyroBurst[3] = { 80.0f, 0.0f, 0.0f };
    const float32 gyroTurn[3]  = { 30.0f, 0.0f, 0.0f };
    const int burstTicks = (int)(0.5f / B_DT);
    const int boundTicks = (int)(0.5f / B_DT);
    const int tailTicks  = (int)(60.0f / B_DT);
    int i;
    int recoverTick = -1;
    float32 gyroBiasAtRecover[3];
    boolean everMovedAfterRecover = FALSE;

    mat3Tvec(M, accLevel, accSensor);
    Ahrs_init();
    bringUp(&v, accSensor);

    for (i = 0; i < burstTicks; ++i)
    {
        Ahrs_update(&v, accSensor, gyroBurst, B_DT, TRUE);
    }
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0u, v.accWeightPct,
        "80 deg/s for 0.5 s must have armed the hold-off");

    for (i = 0; (i < boundTicks) && (recoverTick < 0); ++i)
    {
        Ahrs_update(&v, accSensor, gyroTurn, B_DT, TRUE);
        if (v.accWeightPct > 0u)
        {
            recoverTick = i;
        }
    }
    {
        char msg[128];
        (void)snprintf(msg, sizeof msg,
            "accWeightPct never recovered within 0.5 s of the switch to a "
            "sustained 30 deg/s turn (the task 12b latch this regresses)");
        TEST_ASSERT_TRUE_MESSAGE(recoverTick >= 0, msg);
    }
    {
        char msg[96];
        (void)snprintf(msg, sizeof msg, "recovered accWeightPct = %u, expected 67 +/- 2",
            (unsigned)v.accWeightPct);
        TEST_ASSERT_TRUE_MESSAGE((v.accWeightPct >= 65u) && (v.accWeightPct <= 69u), msg);
    }
    memcpy(gyroBiasAtRecover, v.gyroBias, sizeof gyroBiasAtRecover);

    for (i = 0; i < tailTicks; ++i)
    {
        char  msg[64];
        float diff[3];

        Ahrs_update(&v, accSensor, gyroTurn, B_DT, TRUE);
        (void)snprintf(msg, sizeof msg, "tick %d of the 60 s sustained turn", i);
        TEST_ASSERT_TRUE_MESSAGE(v.accWeightPct > 0u, msg);

        diff[0] = v.gyroBias[0] - gyroBiasAtRecover[0];
        diff[1] = v.gyroBias[1] - gyroBiasAtRecover[1];
        diff[2] = v.gyroBias[2] - gyroBiasAtRecover[2];
        if (norm3(diff) > 1.0e-5f) { everMovedAfterRecover = TRUE; }
    }
    TEST_ASSERT_TRUE_MESSAGE(everMovedAfterRecover,
        "s_fbI (via gyroBias) must resume moving once the accelerometer is trusted again");
}

void test_b12c_never_zero_more_than_035s_after_last_arming_tick(void)
{
    /* B6.6 acceptance (a3)/task 12c (ii), the clause AS WORDED: for ANY
     * input sequence, w_acc must never read exactly 0 while more than
     * 0.35 s has elapsed since the low-passed rate was last >= the arming
     * knee (60 deg/s). "Since the low-pass was last >= 60" is not directly
     * observable from the public interface (s_gyroLpDps is private), so
     * this test carries a SHADOW replica of that one-pole update -- same
     * mirrored-private-constant treatment as
     * test_b11_old_constants_would_not_have_suppressed_60dps_roll's
     * AHRS_ACC_RATE_FULL_DPS/SPAN_DPS mirror above -- seeded at 0, matching
     * Ahrs.c's own seed on the ALIGNING entry this test's bringUp() took.
     * A seeded, randomised sequence of SUSTAINED single-axis segments
     * (each long enough, relative to the 50 ms low-pass tau, for the
     * low-pass to move meaningfully toward its target) exercises arbitrary
     * transitions in and out of the arming region. */
    float M[9]; mountMatrix(M);
    Ahrs_Values v;
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    float32 accSensor[3];
    Rng     r;
    int     seg;
    const float32 lpTauS = 0.05f;      /* mirrors AHRS_GYRO_LP_TAU_S (private) */
    const float32 armDps = 60.0f;      /* mirrors AHRS_ACC_RATE_ZERO_DPS (private) */
    float32 gyroLpShadow = 0.0f;
    float32 timeSinceLastArmS = 1.0e9f; /* "never armed yet" */

    mat3Tvec(M, accLevel, accSensor);
    Ahrs_init();
    bringUp(&v, accSensor);
    rngSeed(&r, 0xB6C60Fu);

    for (seg = 0; seg < 24; ++seg)
    {
        const boolean arming  = ((seg % 2) == 0) ? TRUE : FALSE;
        const float32 rateDps = (arming != FALSE) ? rngF(&r, 70.0f, 150.0f) : rngF(&r, 0.0f, 50.0f);
        const float32 durS    = rngF(&r, 0.2f, 1.5f);
        const int     segTicks = (int)(durS / B_DT);
        const float32 gyro[3] = { rateDps, 0.0f, 0.0f };
        int i;

        for (i = 0; i < segTicks; ++i)
        {
            const float32 lpK = B_DT / lpTauS;   /* B_DT << lpTauS: never capped at 1 */

            Ahrs_update(&v, accSensor, gyro, B_DT, TRUE);

            gyroLpShadow += (rateDps - gyroLpShadow) * lpK;
            if (gyroLpShadow >= armDps)
            {
                timeSinceLastArmS = 0.0f;
            }
            else
            {
                timeSinceLastArmS += B_DT;
            }

            /* Only assert once the shadow low-pass is comfortably clear of
             * the arming knee (a few deg/s of margin, armDps - 2): right at
             * the knee the ORDINARY ramp itself gives a genuinely tiny but
             * nonzero w_acc (e.g. gyroLp = 59.7 -> w_rate = 0.67 %), which
             * the published accWeightPct (uint8, TRUNCATED, not rounded)
             * reads as 0 -- a publish-resolution artifact of the ramp
             * itself, on the way up OR down, not the hold-off this clause
             * bounds. Below the margin the ramp alone is already >= ~4.4 %,
             * so a reported 0 there can only be the hold. */
            if ((v.accWeightPct == 0u) && (gyroLpShadow <= (armDps - 2.0f)))
            {
                char msg[160];

                (void)snprintf(msg, sizeof msg,
                    "segment %d (%.1f deg/s), tick %d: accWeightPct = 0 with "
                    "%.4f s elapsed since the shadow low-pass (%.2f deg/s) "
                    "was last >= 60 deg/s (bound 0.35 s)",
                    seg, (double)rateDps, i, (double)timeSinceLastArmS, (double)gyroLpShadow);
                TEST_ASSERT_TRUE_MESSAGE(timeSinceLastArmS <= 0.35f, msg);
            }
        }
    }
}

void test_b12c_sub60_halfsine_profile_meets_thresholds_and_never_arms(void)
{
    /* B6.6 acceptance (a) sub-60 profile / (a3) (v): a half-sine peaking at
     * 45 deg/s (90 deg in rampS = pi ~= 3.14 s) never reaches the hold-off's
     * 60 deg/s arming knee at all -- the only profile exercising the bare
     * w_rate ramp end to end. Its lateral disturbance is derived from its
     * OWN peak angular acceleration at a 0.43 m lever arm (a_t = alpha*r),
     * the same lever arm that makes the existing 94 deg/s half-sine's 0.15 g
     * self-consistent (alpha = 197 deg/s^2 -> a_t = 0.1507 g): peak alpha
     * here = rollDeg*pi^2/(2*rampS^2) = 90*pi^2/(2*pi^2) = 45 deg/s^2 ->
     * a_t = 45*(pi/180 rad)*0.43 m / 9.80665 m/s^2 = 0.0344 g. */
    float M[9]; mountMatrix(M);
    Ahrs_Values v;
    const float rampS   = (float)M_PI;      /* peaks at rollDeg*pi/(2*rampS) = 45 deg/s */
    const float lateralG = 0.0344f;
    const float checkpoints[3] = { rampS, rampS + 1.0f, rampS + 3.0f };
    float   err[3];
    float   intWDt;
    boolean everZeroWeight = FALSE;
    char    msg[160];

    rollRampHalfSineWithLateralAccelEx(M, 90.0f, rampS, lateralG, &v, err,
                                        checkpoints, 3, &intWDt, &everZeroWeight);

    printf("\n  [StrandB task12c] 90deg/%.3fs half-sine roll (peak 45deg/s, sub-60) + %.4fg lateral accel:\n",
        (double)rampS, (double)lateralG);
    printf("    error at motion end: %.3f deg (target <= 2.0)\n", (double)err[0]);
    printf("    error at +1s: %.3f deg (target <= 1.0)\n", (double)err[1]);
    printf("    error at +3s: %.3f deg (target <= 0.5)\n", (double)err[2]);
    printf("    INTEGRAL(w dt) over the motion: %.4f s\n", (double)intWDt);

    (void)snprintf(msg, sizeof msg, "motion-end error %.3f deg, target <= 2.0", (double)err[0]);
    TEST_ASSERT_TRUE_MESSAGE(err[0] <= 2.0f, msg);
    (void)snprintf(msg, sizeof msg, "+1s error %.3f deg, target <= 1.0", (double)err[1]);
    TEST_ASSERT_TRUE_MESSAGE(err[1] <= 1.0f, msg);
    (void)snprintf(msg, sizeof msg, "+3s error %.3f deg, target <= 0.5", (double)err[2]);
    TEST_ASSERT_TRUE_MESSAGE(err[2] <= 0.5f, msg);
    TEST_ASSERT_FALSE_MESSAGE(everZeroWeight,
        "a sub-60 deg/s profile must never force accWeightPct to 0 via the hold-off");
}

void test_b4_hover_case_bit_identical_to_task3(void)
{
    /* Hover: |gyro| < 30 deg/s and the disturbance < 0.05 g -> w_norm =
     * w_rate = 1 exactly (both comfortably inside the "full trust" band),
     * so w_acc = 1 and eAcc must be bit-identical to task 3's code (which
     * always used kp*1.0, no weight at all). Verified via the published
     * accWeightPct (must read 100) and a bit-exact settle trajectory
     * against the analytic prediction test_gravity_removed_at_rest_in_any_
     * orientation already uses. */
    float M[9]; mountMatrix(M);
    Ahrs_Values v;
    const float32 att[3] = { 0.3f, -0.2f, -0.93f };   /* |a| ~= 1.0, small tilt */
    float32 accSensor[3];
    int i;

    mat3Tvec(M, att, accSensor);
    Ahrs_init();
    bringUp(&v, accSensor);

    for (i = 0; i < 50; ++i)
    {
        const float32 gyro[3] = { 2.0f, -1.0f, 0.5f };   /* well under 30 deg/s */
        Ahrs_update(&v, accSensor, gyro, DT, TRUE);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(100u, v.accWeightPct,
            "hover case (slow, near-1g) must publish full accel weight");
    }
    TEST_ASSERT_TRUE_MESSAGE(isFiniteF(norm3(v.accNed)), "hover trajectory must stay finite");
}

void test_b4_accweightpct_ramps_continuously_with_rate_and_norm(void)
{
    /* Direct check of the two ramps' shape, at the values task 11 names
     * explicitly: full trust inside +/-5% and below 15 deg/s; zero at/beyond
     * the old 15% |a| edge or 60 deg/s (LOW-PASSED, tau = 50 ms -- a fast
     * tumble is held at the sustained rate for several time constants
     * before checking, so this is a steady-state check, not an
     * instantaneous one; the low-pass's own transient shape is covered by
     * test_b11_gyro_lowpass_reaches_steady_state_within_150ms below). */
    float M[9]; mountMatrix(M);
    Ahrs_Values v;
    float32 accSensor[3];
    int i;

    /* Full trust: level, slow, already settled (bringUp uses zero gyro). */
    {
        const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
        const float32 gyroSlow[3] = { 0.0f, 0.0f, 0.0f };
        mat3Tvec(M, accLevel, accSensor);
        Ahrs_init();
        bringUp(&v, accSensor);
        Ahrs_update(&v, accSensor, gyroSlow, DT, TRUE);
        TEST_ASSERT_EQUAL_UINT8(100u, v.accWeightPct);
    }

    /* Zero: at the old 15% edge (|a| = 1.15, AHRS_ACC_MAX_G). */
    {
        const float32 accEdge[3] = { 0.0f, 0.0f, -1.15f };
        const float32 gyroSlow[3] = { 0.0f, 0.0f, 0.0f };
        float32 accS[3];
        mat3Tvec(M, accEdge, accS);
        Ahrs_update(&v, accS, gyroSlow, DT, TRUE);
        TEST_ASSERT_EQUAL_UINT8(0u, v.accWeightPct);
    }

    /* Zero: a SUSTAINED fast tumble (well past the new 60 deg/s edge),
     * level accel, held for 10 time constants (0.5 s) so the low-pass has
     * fully settled before the check. */
    {
        const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
        const float32 gyroFast[3] = { 300.0f, 0.0f, 0.0f };
        float32 accS[3];
        mat3Tvec(M, accLevel, accS);
        for (i = 0; i < (int)(0.5f / DT); ++i)
        {
            Ahrs_update(&v, accS, gyroFast, DT, TRUE);
        }
        TEST_ASSERT_EQUAL_UINT8(0u, v.accWeightPct);
    }
}

void test_b11_gyro_lowpass_reaches_steady_state_within_150ms(void)
{
    /* Task 11's own claim (superseded by task 12b, B6.5): ~150 ms (3 tau)
     * after a fast rotation ends, the low-pass alone decayed back under
     * AHRS_ACC_RATE_FULL_DPS (15 deg/s) and the accel correction
     * re-engaged at full weight. AHRS_ACC_HOLDOFF_S (0.3 s) now ADDS to
     * that -- deliberately, it is the whole point of task 12b -- because
     * the hold keeps re-arming to 0.3 s on every tick the low-pass is still
     * >= 60 deg/s (~80 ms here) and then holds flat (no countdown) through
     * the 15..60 deg/s band (~150 ms total) before it can even START
     * counting down. Full trust is therefore NOT back at 150 ms any more --
     * that is now a regression guard for the hold-off actually taking
     * effect, not a stale expectation -- and returns within a bounded
     * ~450-500 ms instead, still comfortably inside the "0.3 s coasting"
     * cost budget the dispatch names once the pre-existing ~150 ms low-pass
     * recovery is accounted for. */
    float M[9]; mountMatrix(M);
    Ahrs_Values v;
    float32 accSensor[3];
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    const float32 gyroFast[3] = { 300.0f, 0.0f, 0.0f };
    const float32 gyroZero[3] = { 0.0f, 0.0f, 0.0f };
    const int   boundTicks = (int)(0.6f / DT);
    int i;
    int reengageTick = -1;
    char msg[160];

    mat3Tvec(M, accLevel, accSensor);
    Ahrs_init();
    bringUp(&v, accSensor);

    /* Sustain the fast tumble for 0.5 s (well past settling) -- must be
     * fully suppressed while it lasts. */
    for (i = 0; i < (int)(0.5f / DT); ++i)
    {
        Ahrs_update(&v, accSensor, gyroFast, DT, TRUE);
    }
    TEST_ASSERT_EQUAL_UINT8(0u, v.accWeightPct);

    /* Motion stops. Must NOT be at full trust immediately (the whole point
     * of the hold-off). */
    Ahrs_update(&v, accSensor, gyroZero, DT, TRUE);
    TEST_ASSERT_TRUE_MESSAGE(v.accWeightPct < 100u,
        "accel trust must not re-engage on the very tick motion stops");

    /* Task 12b: must NOT be back at full trust by 150 ms any more -- the
     * hold-off is still counting down (or has not even started counting
     * down yet). A regression here means the hold-off stopped doing
     * anything on this profile. */
    for (i = 0; i < (int)(0.15f / DT); ++i)
    {
        Ahrs_update(&v, accSensor, gyroZero, DT, TRUE);
    }
    TEST_ASSERT_TRUE_MESSAGE(v.accWeightPct < 100u,
        "task 12b: accel trust must still be held off at 150 ms after a "
        "manoeuvre that armed the hold (was fully re-engaged pre-task-12b)");

    /* ...but it must come back, and within a bounded time. */
    for (i = (int)(0.15f / DT); (i < boundTicks) && (reengageTick < 0); ++i)
    {
        Ahrs_update(&v, accSensor, gyroZero, DT, TRUE);
        if (v.accWeightPct == 100u)
        {
            reengageTick = i;
        }
    }
    (void)snprintf(msg, sizeof msg,
        "accel trust must fully re-engage within 0.6 s of motion stopping "
        "(measured tick %d = %.3f s)", reengageTick, (double)((float)reengageTick * DT));
    TEST_ASSERT_TRUE_MESSAGE(reengageTick >= 0, msg);
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

/* ==========================================================================
 * SYS1-001 Strand B, task 13 (SWE1-FW-009): the one-shot big-step latch.
 * ======================================================================== */

void test_b13_synthetic_2000dps_z_tick_latches_exactly_one_snapshot(void)
{
    /* The evidence-row defect, reproduced synthetically: a single tick
     * whose raw gyro-z word is -2000 dps (near the +/-2000 dps full scale,
     * same class as the observed 1918-1967 dps) at the measured IMU period
     * -- |gyro|*dt = 2000 * 985.44us = 1.971 deg, comfortably past the
     * 0.5 deg latch threshold. */
    Ahrs_Values v; memset(&v, 0, sizeof v);
    const float32 accLevel[3] = { 0.0f, 0.0f, -1.0f };
    const float32 gyroZero[3] = { 0.0f, 0.0f, 0.0f };
    const float32 gyroBig[3]  = { 0.0f, 0.0f, -2000.0f };
    const float32 gyroBig2[3] = { 0.0f, 0.0f, -1900.0f };  /* a SECOND event */
    uint32 bigStepBefore;

    bringUp(&v, accLevel);
    TEST_ASSERT_EQUAL_UINT32(0u, g_dbgAhrsBigStep);

    bigStepBefore = g_dbgAhrsBigStep;
    Ahrs_update(&v, accLevel, gyroBig, B_DT, TRUE);

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(bigStepBefore + 1u, g_dbgAhrsBigStep,
        "a single near-full-scale gyro-z tick must count exactly one big step");
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(-2000.0f, g_dbgAhrsBigStepSnapshot.gyroRaw[2],
        "the latched snapshot must carry the raw word intact, unmounted, unscaled");
    TEST_ASSERT_EQUAL_FLOAT(0.0f, g_dbgAhrsBigStepSnapshot.gyroRaw[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, g_dbgAhrsBigStepSnapshot.gyroRaw[1]);
    TEST_ASSERT_EQUAL_FLOAT(accLevel[0], g_dbgAhrsBigStepSnapshot.accRaw[0]);
    TEST_ASSERT_EQUAL_FLOAT(accLevel[2], g_dbgAhrsBigStepSnapshot.accRaw[2]);
    TEST_ASSERT_EQUAL_FLOAT(B_DT, g_dbgAhrsBigStepSnapshot.dt);
    {
        /* Copy out of the volatile snapshot first: allFinite() takes a
         * plain float*, and every other read in this test already reads
         * one field at a time (TEST_ASSERT_EQUAL_FLOAT above), which drops
         * the qualifier implicitly the same way an assignment does. */
        const float qCopy[4] = { g_dbgAhrsBigStepSnapshot.q[0],
                                  g_dbgAhrsBigStepSnapshot.q[1],
                                  g_dbgAhrsBigStepSnapshot.q[2],
                                  g_dbgAhrsBigStepSnapshot.q[3] };
        TEST_ASSERT_TRUE_MESSAGE(isFiniteF(g_dbgAhrsBigStepSnapshot.eMagD),
            "snapshot eMagD must be finite");
        TEST_ASSERT_TRUE_MESSAGE(allFinite(qCopy, 4u), "snapshot q must be finite");
    }

    /* A quiet tick afterwards must not touch either the counter or the
     * latch. */
    Ahrs_update(&v, accLevel, gyroZero, B_DT, TRUE);
    TEST_ASSERT_EQUAL_UINT32(bigStepBefore + 1u, g_dbgAhrsBigStep);
    TEST_ASSERT_EQUAL_FLOAT(-2000.0f, g_dbgAhrsBigStepSnapshot.gyroRaw[2]);

    /* A SECOND big-step tick must increment the counter again but must NOT
     * overwrite the already-latched (FIRST) snapshot -- "exactly one
     * snapshot" per the acceptance, however many events follow. */
    Ahrs_update(&v, accLevel, gyroBig2, B_DT, TRUE);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(bigStepBefore + 2u, g_dbgAhrsBigStep,
        "a second big-step tick must still be counted");
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(-2000.0f, g_dbgAhrsBigStepSnapshot.gyroRaw[2],
        "the snapshot must stay latched to the FIRST event, not the second");
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_quaternion_stays_unit_norm);
    RUN_TEST(test_rotation_preserves_magnitude);
    RUN_TEST(test_body_ned_round_trip);
    RUN_TEST(test_mounting_transform_is_a_proper_rotation);
    RUN_TEST(test_mag_mount_field_gravity_angle_is_constant);
    RUN_TEST(test_calibration_window_is_a_duration_not_a_sample_count);
    RUN_TEST(test_calibration_deadline_is_a_duration_and_flags_degraded);
    RUN_TEST(test_level_board_reads_zero_roll_and_pitch);
    RUN_TEST(test_nose_up_reads_pitch_plus_90);
    RUN_TEST(test_right_wing_down_reads_roll_plus_90);
    RUN_TEST(test_yaw_advances_clockwise_and_stays_in_range);
    RUN_TEST(test_gravity_removed_at_rest_in_any_orientation);
    RUN_TEST(test_b0_regression_lag_after_90deg_roll_with_lateral_accel);
    RUN_TEST(test_b0_regression_standing_roll_error_after_mag_error_removed);
    RUN_TEST(test_b0_regression_delta_s_fbi_after_60s_roll90_with_20deg_mag_error);
    RUN_TEST(test_b1_mag_correction_is_pure_yaw_at_every_attitude);
    RUN_TEST(test_b1_level_case_yaw_component_bit_identical_to_pre_fix);
    RUN_TEST(test_b1_yaw_time_constant_matches_the_bench_fit);
    RUN_TEST(test_b2_mag_error_at_roll90_no_longer_moves_body_bias);
    RUN_TEST(test_b2_return_to_level_after_mag_error_settles_within_2s);
    RUN_TEST(test_b3_yaw_integral_clamped_under_persistent_45deg_error);
    RUN_TEST(test_b3_body_integral_clamped_under_persistent_45deg_error);
    RUN_TEST(test_b3_integral_bit_unchanged_on_untrusted_ticks_even_near_clamp);
    RUN_TEST(test_b4_lag_after_90deg_roll_with_lateral_accel_meets_thresholds);
    RUN_TEST(test_b0_regression_halfsine_lag_after_90deg_roll_with_lateral_accel);
    RUN_TEST(test_b11_halfsine_profile_meets_thresholds);
    RUN_TEST(test_b11_old_constants_would_not_have_suppressed_60dps_roll);
    RUN_TEST(test_b12b_holdoff_never_arms_below_60dps);
    RUN_TEST(test_b12c_arm_then_sustained_turn_recovers_within_bound);
    RUN_TEST(test_b12c_never_zero_more_than_035s_after_last_arming_tick);
    RUN_TEST(test_b12c_sub60_halfsine_profile_meets_thresholds_and_never_arms);
    RUN_TEST(test_b4_hover_case_bit_identical_to_task3);
    RUN_TEST(test_b4_accweightpct_ramps_continuously_with_rate_and_norm);
    RUN_TEST(test_b11_gyro_lowpass_reaches_steady_state_within_150ms);
    RUN_TEST(test_nan_or_inf_gyro_or_acc_is_still_rejected_after_the_rename);
    RUN_TEST(test_no_admissible_input_produces_nan);
    RUN_TEST(test_recovers_after_garbage);
    RUN_TEST(test_invalid_sample_freezes_the_estimate);
    RUN_TEST(test_glitch_ticks_freeze_without_realigning);
    RUN_TEST(test_zero_or_negative_dt_never_advances_the_fault_hold_clock);
    RUN_TEST(test_long_gap_declares_no_sensor_immediately_then_realigns_once);
    RUN_TEST(test_b13_synthetic_2000dps_z_tick_latches_exactly_one_snapshot);
    return UNITY_END();
}
