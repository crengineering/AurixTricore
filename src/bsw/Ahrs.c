/**********************************************************************************************************************
 * \file Ahrs.c
 * \brief Quaternion Mahony attitude estimator — see Ahrs.h.
 *********************************************************************************************************************/
#include "Ahrs.h"
#include "Nvm.h"
#include "FusionCal.h"
#include "AhrsLatch.h"
#include "SharedRam.h"
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
 * the gyro. At rest this board measures |a| = 0.998 g. Still used verbatim
 * for the one-shot ahrs_align() gate (a clean, unambiguous reading to start
 * from); the RUNNING correction below no longer uses it as a hard cut --
 * see AHRS_ACC_TRUST_FULL_G. */
#define AHRS_ACC_MIN_G        (0.85f)
#define AHRS_ACC_MAX_G        (1.15f)

/* B3.4 (SYS1-001 strand B, SWE1-FW-006): a hard |a| window alone accepted a
 * lateral disturbance at full gain right up to its edge -- 0.15 g sideways
 * on a 1 g reading gives |a| = 1.011 g, comfortably INSIDE [0.85, 1.15],
 * while tilting the apparent vertical by 8.5 deg (B2(b)). Two continuous
 * weights replace the single hard cut, multiplied together into w_acc,
 * which scales twoKpAcc's P AND I contribution (both go through eAcc, so
 * scaling eAcc scales both integrator paths that read it):
 *   w_norm  ramps from 1 (|dev| <= AHRS_ACC_TRUST_FULL_G, +/-5%) down to 0
 *           at |dev| == AHRS_ACC_MAX_G - 1 (+/-15%, TODAY's outer edge --
 *           nothing previously rejected is newly accepted).
 *   w_rate  ramps from 1 (|gyro| <= AHRS_ACC_RATE_FULL_DPS) down to 0 at
 *           AHRS_ACC_RATE_FULL_DPS + AHRS_ACC_RATE_SPAN_DPS -- fast handling
 *           rides the gyro path already (no lag there); this is about not
 *           trusting the accel DURING it.
 * accTrusted (Ahrs_Values) becomes (w_acc > 0), the same outer edge as
 * before; accWeightPct (0..100) publishes w_acc itself.
 *
 * Task 11 (review round 1): the constants moved (30/90 -> 15/45, zero trust
 * at 60 deg/s instead of 120) and w_rate now reads a 50 ms low-passed |gyro|
 * (s_gyroLpDps below), not the instantaneous sample, for two reasons:
 *   - mean vs. peak vibration: an instantaneous |gyro| sample on a real
 *     airframe is dominated by vibration spikes riding on top of the real
 *     angular rate; gating on the INSTANTANEOUS value chatters the weight
 *     tick-to-tick on noise the accelerometer correction never actually
 *     needed protecting against -- the low-pass tracks the real motion,
 *     not the noise floor on top of it.
 *   - ~150 ms re-engagement hold-off: tau = 0.05 s means roughly 3 tau
 *     (~150 ms) after a fast rotation ends before the low-passed value
 *     decays back under AHRS_ACC_RATE_FULL_DPS and the accel correction
 *     re-engages at full weight -- deliberate: the vertical estimate right
 *     after a fast manoeuvre is exactly when the accelerometer is least
 *     trustworthy (settling structural vibration, residual specific force),
 *     so re-arming instantly would undo the point of gating on rate at all.
 */
#define AHRS_ACC_TRUST_FULL_G     (0.05f)
#define AHRS_ACC_TRUST_SPAN_G     ((AHRS_ACC_MAX_G - 1.0f) - AHRS_ACC_TRUST_FULL_G)
#define AHRS_ACC_RATE_FULL_DPS    (15.0f)
#define AHRS_ACC_RATE_SPAN_DPS    (45.0f)

/* Task 11: time constant of the |gyro| low-pass that feeds w_rate -- see the
 * block comment above for why 50 ms (not the instantaneous sample). */
#define AHRS_GYRO_LP_TAU_S        (0.05f)

/* Magnetometer trust window [gauss]. Earth's field is 0.25..0.65 G worldwide
 * (~0.48 G in Munich); the band is widened to tolerate a residual hard-iron
 * offset without accepting a value that is wrong by a clean factor — which is
 * what a bad scale constant or a mis-assembled 18-bit word produces. */
#define AHRS_MAG_MIN_G        (0.15f)
#define AHRS_MAG_MAX_G        (2.0f)

/* Gyro-bias calibration: a still-window DURATION to average over, and the
 * motion gate. T14 (docs/REFACTORING_PLAN.md §3.8): this used to be a sample
 * COUNT ("100 samples is ~2 s at the 50 Hz IMU task"), which silently became
 * ~0.1 s of averaging the moment the task moved to 1014 Hz -- 20x less noise
 * rejection on the one number the whole attitude solution rests on, with no
 * error anywhere. Accumulating `dt` instead of counting calls makes the
 * window mean the same thing at any rate, forever.
 *
 * The gate measures the SPREAD (max - min) across the window, NOT the absolute
 * rate — a large constant offset is precisely the thing being measured, so an
 * absolute threshold would read "moving" forever and calibration would never
 * finish. Motion restarts the window, because a biased bias is worse than none. */
#define AHRS_CAL_WINDOW_S     (2.0f)
#define AHRS_CAL_SPREAD_DPS   (3.0f)

/* ...but give up after this DURATION and go anyway (was "500 samples, 10 s at
 * 50 Hz" -- same T14 fix, same reason: a sample count silently changes
 * meaning with the task rate, a duration does not).
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
#define AHRS_CAL_DEADLINE_S   (10.0f)

/* Sanity bounds on dt [s]. The IMU task runs at 50 Hz; the upper bound matters
 * at boot, where the first measured interval is whatever the STM counter held. */
#define AHRS_DT_MIN_S         (0.0001f)
#define AHRS_DT_MAX_S         (0.2f)

/* SYS1-001: how long invalid input must PERSIST before the sensor is
 * presumed actually gone. Below this, one rejected tick (or a short run of
 * them) freezes the estimate and waits for the next good sample; only once
 * bad input has accumulated this much do we declare AHRS_NO_SENSOR and let
 * the next good tick re-align. The dispatched defect was a single one-tick
 * glitch (a duplicate DRDY edge, ~1 per 2400 edges) re-initialising the
 * whole attitude and zeroing the gyro-bias integral on the spot; the fix is
 * this debounce, not a faster sensor. Chosen well above one glitch (~1 ms)
 * and well below a flight-relevant reaction time (100 ms): a genuine outage
 * this long already means the last 50 ms of yaw is stale regardless of what
 * this estimator does about it. */
#define AHRS_FAULT_HOLD_S     (0.05f)

/* Upper bound for a dt COUNTED TOWARD THE FAULT-HOLD CLOCK -- deliberately
 * NOT AHRS_DT_MAX_S (flight-reviewer FAIL, SYS1-001 regression): that bound
 * exists to keep a nonsense interval out of the INTEGRATOR, and reusing it
 * here silently excluded the one dt that most needs to count -- a LONG
 * NavTask.c edge (a genuine gap the sensor took to answer again) reports the
 * real gap as dt, e.g. 0.5 s, and 0.5 s >= AHRS_DT_MAX_S (0.2 s) made the old
 * guard throw it away, so s_faultHoldS never moved and AHRS_NO_SENSOR never
 * fired -- a dead IMU / broken INT1 left the estimator reporting
 * AHRS_RUNNING on a frozen quaternion forever. This bound instead only
 * rejects what must never be trusted as a real duration: <= 0, NaN (compares
 * false against every relational operator here, same discipline as
 * navTask_dtValid()), or a corrupted/absurd value -- 60 s is generous
 * headroom over any interval NavTask.c can legitimately report (a uint32
 * STM0 tick delta at 100 MHz cannot even represent a gap past ~42.9 s). A
 * dt this large clears AHRS_FAULT_HOLD_S in one step, which is the point: a
 * gap that long already IS the outage. */
#define AHRS_FAULT_DT_MAX_S   (60.0f)

/* B3.3 (SYS1-001 strand B, SWE1-FW-005): a backstop, not the fix -- 1-2
 * remove the mechanism that let the integrals wind up in the first place,
 * this bounds what a REMAINING one can do (e.g. a calibration/alignment
 * residual too small to trip the plausibility checks). twoKi is already
 * slow (tau ~ 50 s) so an unbounded integral only ever loses slowly, but
 * "only ever loses slowly" is not the same guarantee as "cannot exceed a
 * known worst case" -- this makes it the latter. 2.0 deg/s per body axis
 * matches the worst standing roll/pitch error this defect produced (B1
 * evidence: 2.8 deg over 45 s -- one twoKi time constant's worth); 1.0 deg/s
 * on the heading integral is 4x the observed 0.5 deg/s boot-to-boot spread
 * (SWE1-FW-002's own status note). */
#define AHRS_FBI_MAX_DPS      (2.0f)
#define AHRS_FBI_YAW_MAX_DPS  (1.0f)

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

static float32 s_fbI[3];           /* Mahony integral feedback [rad/s], BODY --
                                     * B3.2 (SYS1-001 strand B): fed ONLY by
                                     * eAcc now, never by the magnetometer     */
static float32 s_fbIYaw;           /* Mahony integral feedback [rad/s], about
                                     * the estimated vertical (d_b) -- fed
                                     * ONLY by eMagD. Applied as s_fbIYaw*dB,
                                     * same "one degree of freedom" split as
                                     * the proportional term (B3.1)           */
static float32 s_gyroLpDps;        /* Task 11: 50 ms low-passed |gyro| [deg/s],
                                     * feeds w_rate -- see AHRS_GYRO_LP_TAU_S */
static float32 s_bias[3];          /* boot gyro bias, body frame [deg/s]      */
static float32 s_calSum[3];
static float32 s_calMin[3];
static float32 s_calMax[3];
static uint16  s_calCount;      /* samples in the CURRENT window -- only ever
                                  * used to divide s_calSum; the window's PASS
                                  * condition is s_calWindowS, not this */
static float32 s_calWindowS;    /* elapsed dt in the current window [s];
                                  * reset to 0 whenever motion restarts it */
static float32 s_calElapsedS;   /* elapsed dt since calibration STARTED [s];
                                  * NEVER reset by motion -- this is what the
                                  * AHRS_CAL_DEADLINE_S gate is measured against */
static boolean s_biasDegraded;  /* deadline hit before a clean window */

/* SYS1-001: accumulated dt of CONSECUTIVE invalid ticks [s], reset the
 * instant a good one arrives. Same "accumulate a duration, not a count"
 * idiom as s_calWindowS above -- see AHRS_FAULT_HOLD_S for what it gates. */
static float32 s_faultHoldS;

/* Named s_ahrsState rather than the obvious s_state: MISRA 5.9 wants
 * internal-linkage identifiers unique across the whole program, and
 * fusion.c and src/asw/CtrlReplay.c each had their own s_state. */
static Ahrs_State s_ahrsState;

/* CPU1-side cache of the last CONSUMED magnetometer sample, refreshed from
 * g_magLatch (AhrsLatch.h) at the top of every Ahrs_update() call -- see
 * ahrs_refreshMag() below. Ahrs_setMag() (CPU0) no longer writes these
 * directly, same T12 discipline as fusion.c's baro/GNSS latches. */
static float32 s_magB[3];          /* latched sample, BODY frame, corrected   */
static float32 s_magNorm;

/* cppcheck-suppress misra-c2012-8.7 ; deviation: read over XCP SHORT_UPLOAD
 * by raw address (tools/xcp_read.py), never referenced by C code outside
 * this file -- same class of deviation as g_imuDrdy* (ImuInt.c). SYS1-001
 * task 0 instrumentation: see Ahrs.h for what it counts. */
volatile uint32 g_dbgAhrsRealigns;

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

/* B3.3: clamp a scalar into [-limit, +limit]. A NaN fails both comparisons
 * below and falls through unclamped -- matching this file's own discipline
 * elsewhere (ahrs_invSqrt, navTask_dtValid): the caller (Ahrs_update) only
 * ever calls this with a value it just computed from finite inputs times a
 * bounded gain and a bounded dt, so a NaN here would already mean the
 * upstream AHRS_INPUT_MAX/dt-window checks were bypassed, not something
 * this clamp is the right place to paper over. */
static float32 ahrs_clamp(float32 v, float32 limit)
{
    float32 r = v;

    if (v > limit)
    {
        r = limit;
    }
    else if (v < -limit)
    {
        r = -limit;
    }
    else
    {
        /* already inside the band */
    }

    return r;
}

/* B3.4: clamp a scalar into [0, 1] -- the weight-ramp shape both w_norm and
 * w_rate share. A NaN input (e.g. an accNorm computed from a NaN sample --
 * already excluded upstream by AHRS_INPUT_MAX, same reasoning as
 * ahrs_clamp above) falls through both comparisons and returns v itself
 * unclamped, same discipline as the rest of this file. */
static float32 ahrs_clamp01(float32 v)
{
    float32 r = v;

    if (v > 1.0f)
    {
        r = 1.0f;
    }
    else if (v < 0.0f)
    {
        r = 0.0f;
    }
    else
    {
        /* already inside [0,1] */
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

    s_fbIYaw       = 0.0f;
    s_gyroLpDps    = 0.0f;
    s_calCount     = 0u;
    s_calWindowS   = 0.0f;
    s_calElapsedS  = 0.0f;
    s_biasDegraded = FALSE;
    s_ahrsState    = AHRS_CALIBRATING;
    s_magNorm  = 0.0f;
    s_faultHoldS = 0.0f;
    g_dbgAhrsRealigns = 0u;

    /* g_magLatch (AhrsLatch.h): the PRODUCER's state, zeroed here even though
     * Ahrs_init() runs on CPU1 (via NavTask_init, T12) -- safe by
     * construction, not synchronisation, for the same reason Fusion_init()
     * zeroing g_baroLatch/g_gnssLatch is safe: SensorTask_mag (the only
     * writer) does not run until CPU0's own Scheduler_run() loop starts,
     * strictly after CPU1 has already reached this point in its own, much
     * shorter boot sequence (see fusion.c Fusion_init() for the full
     * argument). */
    for (i = 0u; i < 3u; i++)
    {
        g_magLatch.magB[i] = 0.0f;
    }
    g_magLatch.magNorm  = 0.0f;
    g_magLatch.reserved = 0u;
    g_magLatch.gen      = 0u;
}

/* Average the gyro while the board is still; motion restarts the window.
 * T14: the window is now a DURATION (s_calWindowS/s_calElapsedS, accumulated
 * from dt) rather than a sample count -- see AHRS_CAL_WINDOW_S's comment.
 * s_calCount still counts samples, but only to divide s_calSum; it plays no
 * part in deciding when the window is done. */
static boolean ahrs_calibrate(const float32 gyroBody[3], float32 dt)
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

    s_calWindowS  += dt;
    s_calElapsedS += dt;

    if ((moving != FALSE) && (s_calElapsedS < AHRS_CAL_DEADLINE_S))
    {
        /* Throw the window away rather than bake the motion into the bias.
         * s_calElapsedS is NOT reset here -- the deadline is measured against
         * the whole calibration attempt, not against the current window, or a
         * board that never sits still for AHRS_CAL_WINDOW_S would restart the
         * deadline every time and never give up. */
        for (i = 0u; i < 3u; i++)
        {
            s_calSum[i] = 0.0f;
        }
        s_calCount   = 0u;
        s_calWindowS = 0.0f;
    }
    else if ((s_calWindowS >= AHRS_CAL_WINDOW_S)
             || (s_calElapsedS >= AHRS_CAL_DEADLINE_S))
    {
        /* Either a clean window completed, or the deadline expired and this is
         * the best that is going to be available. Divide by what was actually
         * accumulated, not by a nominal count -- at the deadline the window
         * is usually partial, and dividing by the nominal sample count
         * regardless would scale the bias down toward zero and look
         * deceptively small. */
        const uint16 n = (s_calCount > 0u) ? s_calCount : 1u;

        if (s_calWindowS < AHRS_CAL_WINDOW_S)
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

/* The Mahony correction. B3.2 (SYS1-001 strand B, SWE1-FW-004) splits what
 * used to be one combined e[3] into the two pieces the two integrators
 * (s_fbI, s_fbIYaw -- Ahrs_update) need kept apart:
 *   eAcc[3]  the accelerometer's full body-frame correction, unprojected --
 *            it is already perpendicular to d_b by construction (it IS the
 *            correction that keeps d_b aligned with gravity).
 *   eMagD    the magnetometer's correction, ALREADY projected onto d_b and
 *            ALREADY scaled by kp -- a single scalar along the one axis
 *            (heading, about d_b) the magnetometer is allowed to touch.
 *   dB       the current estimated vertical in body frame (nedToBody(0,0,1)),
 *            returned on EVERY call regardless of magUsed: Ahrs_update needs
 *            it to apply s_fbIYaw*dB even on a tick where the field itself
 *            is untrusted, same as s_fbI[i] keeps contributing through an
 *            accel dropout. */
static void ahrs_errorVector(const float32 accBody[3], float32 accNorm,
                             float32 wAcc, float32 eAcc[3], float32 dB[3],
                             float32 *eMagD, boolean *accUsed, boolean *magUsed)
{
    const float32 downNed[3] = { 0.0f, 0.0f, 1.0f };

    eAcc[0] = 0.0f;
    eAcc[1] = 0.0f;
    eAcc[2] = 0.0f;
    *eMagD  = 0.0f;
    *accUsed = FALSE;
    *magUsed = FALSE;

    ahrs_nedToBody(downNed, dB);

    if (wAcc > 0.0f)
    {
        /* Where the filter BELIEVES the specific force points: minus the NED
         * down axis, expressed in body. The cross product with what the
         * accelerometer actually measured is the rotation that reconciles the
         * two — small-angle, so no trigonometry is needed. B3.4: wAcc (the
         * continuous weight, Ahrs_update) scales the WHOLE correction here,
         * so both the proportional (this) and integral (s_fbI, fed by eAcc)
         * paths see it together. */
        const float32 up[3] = { 0.0f, 0.0f, -1.0f };
        const float32 recip = 1.0f / accNorm;
        const float32 ax = accBody[0] * recip;
        const float32 ay = accBody[1] * recip;
        const float32 az = accBody[2] * recip;
        float32 v[3];

        ahrs_nedToBody(up, v);

        const float32 kp = FusionCal_positive(g_fusionCal.twoKpAcc, 0.0f,
                                             AHRS_TWO_KP_ACC) * wAcc;

        eAcc[0] = kp * ((ay * v[2]) - (az * v[1]));
        eAcc[1] = kp * ((az * v[0]) - (ax * v[2]));
        eAcc[2] = kp * ((ax * v[1]) - (ay * v[0]));

        *accUsed = TRUE;
    }
    else
    {
        /* w_acc == 0: past the old hard edge (|a|-1| >= 15%) either way --
         * contaminated by real acceleration, coast on the gyro. */
    }

    if ((s_magNorm > AHRS_MAG_MIN_G) && (s_magNorm < AHRS_MAG_MAX_G))
    {
        /* Rotate the measured field into NED, then flatten it: whatever the
         * horizontal part turns out to be, DEFINE it as pointing north. */
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

        /* B3.1 (dispatch B2(a)): the raw cross product mn x w is NOT a
         * rotation about the vertical -- for a heading error psi its NED
         * components are (h_r*h_z*sinPsi, h_r*h_z*(1-cosPsi), -h_r^2*sinPsi),
         * dominant along NORTH (docs/FUSION.md documents the corrected pole
         * arithmetic; the false "rotation about the vertical only" claim
         * this comment used to make is deleted). Projecting onto d_b keeps
         * exactly the one degree of freedom (heading) the magnetometer is
         * allowed: *eMagD = raw . d_b is exactly -h_r^2*sinPsi, the same
         * kp_eff,mag = twoKpMag*h_r^2 the 7.1 s time-constant fit already
         * measures (docs/FUSION.md section 5), so yaw dynamics are
         * unchanged. At level d_b = [0,0,1] exactly, so this reduces to
         * raw[2] alone -- bit-identical to the pre-B3.1 e[2] term. */
        {
            float32 raw[3];

            raw[0] = (my * w[2]) - (mz * w[1]);
            raw[1] = (mz * w[0]) - (mx * w[2]);
            raw[2] = (mx * w[1]) - (my * w[0]);

            *eMagD = kp * ((raw[0] * dB[0]) + (raw[1] * dB[1]) + (raw[2] * dB[2]));
        }

        *magUsed = TRUE;
    }
    else
    {
        /* No usable field — yaw coasts on the gyro and will drift. */
    }
}

/* Read one consistent snapshot of g_magLatch into s_magB/s_magNorm. CPU1
 * (the consumer) ONLY -- same protocol as NavState_get()/
 * baroLatch_get()/gnssLatch_get() (fusion.c): read gen, copy the payload,
 * re-read gen; one retry on a mismatch, then give up and leave s_magB/
 * s_magNorm exactly as they were (the last consumed sample), rather than a
 * torn mix of old and new. Unlike the baro/GNSS latches there is no
 * "already consumed this gen" check: this latch is LEVEL-triggered, so every
 * Ahrs_update() tick re-applies whatever the latch currently holds, exactly
 * as the original single-core code did with no "new sample" concept at all. */
static void ahrs_refreshMag(void)
{
    uint8   attempt;
    boolean ok = FALSE;

    for (attempt = 0u; (attempt < 2u) && (ok == FALSE); attempt++)
    {
        uint32  genBefore = g_magLatch.gen;
        float32 magB[3];
        float32 magNorm;

        magB[0] = g_magLatch.magB[0];
        magB[1] = g_magLatch.magB[1];
        magB[2] = g_magLatch.magB[2];
        magNorm = g_magLatch.magNorm;

        if (g_magLatch.gen == genBefore)
        {
            s_magB[0] = magB[0];
            s_magB[1] = magB[1];
            s_magB[2] = magB[2];
            s_magNorm = magNorm;
            ok = TRUE;
        }
    }
}

void Ahrs_update(Ahrs_Values *out, const float32 acc[3], const float32 gyro[3],
                 float32 dt, boolean valid)
{
    boolean accUsed = FALSE;
    boolean magUsed = FALSE;
    uint8   i;

    ahrs_refreshMag();

    if (valid != FALSE)
    {
        /* SYS1-001 debounce: the caller says this tick's input is usable --
         * the hold clock resets regardless of what the dt/NaN checks below
         * make of it, same as a healthy sample answering a doubt. */
        s_faultHoldS = 0.0f;
    }
    else
    {
        /* accumulated below, in the valid==FALSE branch */
    }

    if ((valid == FALSE) || (dt <= AHRS_DT_MIN_S) || (dt >= AHRS_DT_MAX_S))
    {
        /* Freeze. A stale sample or a nonsense interval integrated as though
         * it were real is how an estimator ends up confidently wrong. */
        if (valid == FALSE)
        {
            /* SYS1-001 debounce: one rejected tick must freeze the estimate,
             * not re-initialise it -- re-aligning on every one-tick glitch is
             * what turned a duplicate DRDY edge into a yaw sawtooth (see
             * AHRS_FAULT_HOLD_S). Only once bad input has PERSISTED that long
             * is the sensor presumed actually gone. */
            if ((dt > 0.0f) && (dt < AHRS_FAULT_DT_MAX_S))
            {
                /* A genuinely measured, bounded interval -- count it toward
                 * the hold window, WHATEVER its size (see AHRS_FAULT_DT_MAX_S
                 * -- this is deliberately not AHRS_DT_MAX_S, the integration
                 * bound). A nonsense dt (<=0, NaN, or absurdly large; NaN
                 * compares false against every relational operator here,
                 * same discipline as navTask_dtValid()) contributes nothing,
                 * so a corrupt caller can neither race the debounce shut nor
                 * freeze it open forever. */
                s_faultHoldS += dt;
            }
            else
            {
                /* not a usable interval -- do not advance the hold clock */
            }

            if (s_faultHoldS >= AHRS_FAULT_HOLD_S)
            {
                s_ahrsState = AHRS_NO_SENSOR;
            }
            else
            {
                /* still within the hold window: frozen, not yet declared
                 * gone -- s_ahrsState (and s_fbI, untouched below) stay
                 * exactly as they were. */
            }

            out->accNed[0]     = 0.0f;
            out->accNed[1]     = 0.0f;
            out->accNed[2]     = 0.0f;
            out->rate[0]       = 0.0f;
            out->rate[1]       = 0.0f;
            out->rate[2]       = 0.0f;
            out->accMagG       = 0.0f;
            out->accWeightPct  = 0u;
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
            if (ahrs_calibrate(gyroBody, dt) != FALSE)
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
                s_fbIYaw = 0.0f;

                /* Task 11: seed the low-pass from the INSTANTANEOUS |gyro|
                 * on entry to RUNNING, not 0 -- a re-align after a genuine
                 * outage says nothing about the rate the board is moving at
                 * right now, and starting the filter at 0 would report full
                 * accel trust for one time constant regardless of reality. */
                s_gyroLpDps = sqrtf((gyroBody[0] * gyroBody[0])
                                   + (gyroBody[1] * gyroBody[1])
                                   + (gyroBody[2] * gyroBody[2]));

                s_ahrsState = AHRS_RUNNING;
                g_dbgAhrsRealigns++;
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
            float32 eAcc[3];
            float32 dB[3];
            float32 eMagD;
            float32 wAcc;
            float32 wx;
            float32 wy;
            float32 wz;
            float32 recipNorm;

            /* B3.4: the continuous accel weight, computed once here (both
             * accBody/accNorm and gyroBody are in scope) and threaded
             * through ahrs_errorVector rather than recomputed there. */
            {
                const float32 devNormG = fabsf(accNorm - 1.0f);
                const float32 gyroNormDps = sqrtf((gyroBody[0] * gyroBody[0])
                                                 + (gyroBody[1] * gyroBody[1])
                                                 + (gyroBody[2] * gyroBody[2]));
                const float32 wNorm = ahrs_clamp01(1.0f
                    - ((devNormG - AHRS_ACC_TRUST_FULL_G) / AHRS_ACC_TRUST_SPAN_G));
                float32 wRate;

                /* Task 11: w_rate reads the LOW-PASSED |gyro|, not the
                 * instantaneous sample -- see AHRS_GYRO_LP_TAU_S's block
                 * comment for the two reasons (mean-vs-peak vibration,
                 * ~150 ms re-engagement hold-off). One-pole update, same
                 * "duration, not a count" arithmetic as every other dt-scaled
                 * accumulator in this file. The coefficient is capped at 1.0:
                 * dt is bounded above by NAVTASK_DT_MAX_S (0.2 s), 4x
                 * AHRS_GYRO_LP_TAU_S, and an uncapped one-pole update
                 * overshoots (and can even go transiently negative -- an
                 * unphysical "rate" that would then read back as spurious
                 * full trust) once dt/tau exceeds the [0,1] BIBO-stable,
                 * non-overshooting range. Capping to 1.0 makes a rare
                 * near-boundary LONG dt behave as an instant re-seed
                 * (k=1 -> s_gyroLpDps = gyroNormDps exactly) instead --
                 * still correct, never an overshoot. */
                {
                    const float32 lpK = (dt < AHRS_GYRO_LP_TAU_S)
                                       ? (dt / AHRS_GYRO_LP_TAU_S) : 1.0f;
                    s_gyroLpDps += (gyroNormDps - s_gyroLpDps) * lpK;
                }

                wRate = ahrs_clamp01(1.0f
                    - ((s_gyroLpDps - AHRS_ACC_RATE_FULL_DPS) / AHRS_ACC_RATE_SPAN_DPS));

                wAcc = wNorm * wRate;
            }

            ahrs_errorVector(accBody, accNorm, wAcc, eAcc, dB, &eMagD, &accUsed, &magUsed);
            out->accWeightPct = (uint8)(wAcc * 100.0f);

            /* B3.2: two independent integrators, never mixed. Gains are read
             * every tick from the calibration block, so a tuning write takes
             * effect on the next update rather than the next flash. */
            {
                const float32 ki = FusionCal_positive(g_fusionCal.twoKi,
                                                      0.0f, AHRS_TWO_KI);
                /* B3.3: a backstop bound on each integral, applied right
                 * where it is produced -- see AHRS_FBI_MAX_DPS/
                 * AHRS_FBI_YAW_MAX_DPS above for why these numbers and why
                 * this is not the primary fix (1-2 are). */
                const float32 fbiMaxRad    = AHRS_FBI_MAX_DPS * AHRS_DEG_TO_RAD;
                const float32 fbiYawMaxRad = AHRS_FBI_YAW_MAX_DPS * AHRS_DEG_TO_RAD;

                for (i = 0u; i < 3u; i++)
                {
                    s_fbI[i] = ahrs_clamp(s_fbI[i] + (ki * eAcc[i] * dt), fbiMaxRad);
                }
                s_fbIYaw = ahrs_clamp(s_fbIYaw + (ki * eMagD * dt), fbiYawMaxRad);
            }

            /* The integral terms ARE the gyro-bias estimate: a rotation error
             * that keeps pointing the same way can only be a rate offset.
             * The proportional terms stay combined (eAcc + eMagD*dB spans
             * the same three axes eAcc alone used to, exactly as before
             * B3.1/B3.2 -- only which INTEGRATOR accumulates each piece
             * changed), and so does s_fbIYaw's contribution: applied along
             * dB on every axis, same as the integral always was. */

            wx = ((gyroBody[0] - s_bias[0]) * AHRS_DEG_TO_RAD)
               + eAcc[0] + (eMagD * dB[0]) + s_fbI[0] + (s_fbIYaw * dB[0]);
            wy = ((gyroBody[1] - s_bias[1]) * AHRS_DEG_TO_RAD)
               + eAcc[1] + (eMagD * dB[1]) + s_fbI[1] + (s_fbIYaw * dB[1]);
            wz = ((gyroBody[2] - s_bias[2]) * AHRS_DEG_TO_RAD)
               + eAcc[2] + (eMagD * dB[2]) + s_fbI[2] + (s_fbIYaw * dB[2]);

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
            out->rate[0]      = 0.0f;
            out->rate[1]      = 0.0f;
            out->rate[2]      = 0.0f;
            out->accWeightPct = 0u;
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

        /* B3.2: gyroBias keeps the SAME meaning and the SAME publish
         * formula shape -- s_fbI[i] + s_fbIYaw*dB[i] is simply the total
         * integral feedback now split across two accumulators instead of
         * one, so this is still "the constant found at boot plus whatever
         * the integral has tracked since". dB is recomputed fresh here
         * (cheap, one nedToBody call) rather than carried out of the
         * RUNNING block above, since it must reflect the ATTITUDE JUST
         * PUBLISHED (post quaternion update), not the one the correction
         * was computed against a moment earlier -- the two agree to within
         * one integration step regardless, but this keeps the invariant
         * exact rather than approximate. */
        {
            float32 dBPub[3];
            const float32 downNed[3] = { 0.0f, 0.0f, 1.0f };

            ahrs_nedToBody(downNed, dBPub);

            for (i = 0u; i < 3u; i++)
            {
                /* Report the total: the constant found at boot plus whatever
                 * the integral has tracked since. The sign is flipped
                 * because the integral is ADDED to the gyro, so it holds
                 * minus the bias. */
                out->gyroBias[i] = s_bias[i]
                                 - ((s_fbI[i] + (s_fbIYaw * dBPub[i])) * AHRS_RAD_TO_DEG);
            }
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

/* CPU0 ONLY (SensorTask_mag). Writes g_magLatch (AhrsLatch.h): payload ->
 * Ifx__dsync() -> gen++, same discipline as every other shared object
 * (SharedRam.h rule 3) -- applied on BOTH branches here, even though this
 * latch is level-triggered rather than one-shot, so the reader's torn-read
 * retry (ahrsMagLatch_get(), below) is meaningful for every write, not only
 * some of them. */
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
        float32 bodyMag[3];

        corrected[0] = (mag[0] - g_xcpNvm.magOffX) * sx;
        corrected[1] = (mag[1] - g_xcpNvm.magOffY) * sy;
        corrected[2] = (mag[2] - g_xcpNvm.magOffZ) * sz;

        ahrs_mountMag(corrected, bodyMag);

        g_magLatch.magB[0] = bodyMag[0];
        g_magLatch.magB[1] = bodyMag[1];
        g_magLatch.magB[2] = bodyMag[2];
        g_magLatch.magNorm = sqrtf((bodyMag[0] * bodyMag[0]) + (bodyMag[1] * bodyMag[1])
                                  + (bodyMag[2] * bodyMag[2]));
    }
    else
    {
        /* Stop trusting a field we are no longer receiving: leaving the last
         * sample latched would let a dead magnetometer keep steering yaw. */
        g_magLatch.magNorm = 0.0f;
    }

    /* cppcheck-suppress misra-c2012-17.3 ; deviation: Ifx__dsync() wraps
     * TASKING's __dsync() intrinsic, which has no declaration anywhere
     * cppcheck can see (SharedRam.h). */
    Ifx__dsync();
    g_magLatch.gen = g_magLatch.gen + 1u;
}
