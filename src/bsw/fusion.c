/**********************************************************************************************************************
 * \file fusion.c
 * \brief Vertical-channel Kalman filter — see fusion.h.
 *********************************************************************************************************************/
#include "fusion.h"

/* --- tuning -------------------------------------------------------------- */

/* Accelerometer uncertainty driving the prediction [m/s^2]. Deliberately ~30x
 * the measured noise floor (0.0094 m/s^2, from a 30 s bench record): the
 * dominant error is NOT the sensor but the level-desk scaffold in fusion.h,
 * which is worth 0.6 m/s^2 at 20 degrees of tilt. Raise this if the estimate
 * lags, lower it if the output is as noisy as the raw barometer. */
#define FUSION_SIGMA_A      (0.3f)

/* Barometer noise [m], MEASURED on this board: 30 s at rest gave a standard
 * deviation of 0.0197 m. R is a VARIANCE, so it is that value squared — the
 * covariance algebra only adds up in squared units. Note this is short-term
 * noise only; barometric drift with weather is far larger but is not white, so
 * inflating R is the wrong way to handle it (a baro-bias state is the right
 * one, if it ever matters). */
#define FUSION_SIGMA_BARO   (0.0197f)
#define FUSION_R_BARO       (FUSION_SIGMA_BARO * FUSION_SIGMA_BARO)

/* Outlier gate: reject a barometer sample further than 5 sigma from what the
 * filter expected, i.e. y^2 > 25 * S. One corrupt reading otherwise yanks the
 * estimate AND shrinks P as though it were good information, after which the
 * filter rejects the next genuine sample. */
#define FUSION_GATE_SIGMA_SQ (25.0f)

/* ESCAPE HATCH for the gate above, and it is not optional.
 *
 * Rejecting a sample also skips the covariance update, so a filter whose state
 * has been thrown far off rejects every subsequent measurement -- and because
 * the innovation grows with the diverging state faster than the gate threshold
 * grows with P, it never recovers. Observed on 2026-08-25: a knock against the
 * board (-8.2 g) left the estimate at 293 m and climbing, with 4282 consecutive
 * rejections at the full 50 Hz.
 *
 * Two layers. Every rejection INFLATES P, so an estimate the filter has reason
 * to doubt stops being defended and the gate reopens on its own. If that is
 * still not enough after FUSION_REJECT_MAX consecutive samples (0.5 s at
 * 50 Hz), the state is simply wrong: re-acquire from the barometer rather than
 * defend a fiction. */
#define FUSION_REJECT_INFLATE (2.0f)
#define FUSION_REJECT_MAX     (25u)

/* Sanity bound on the input [m/s^2]. NOT a manoeuvre limit -- real flight
 * accelerations must pass through untouched, that is what the accelerometer is
 * for. This catches only physically impossible values, i.e. a corrupted SPI
 * transfer, at roughly 10 g. */
#define FUSION_ACC_D_MAX      (100.0f)

/* Sanity bounds on dt [s]. The IMU task runs at 50 Hz. The lower bound keeps a
 * double call from dividing by nothing; the upper bound matters at boot, where
 * the first measured interval is whatever the STM counter happened to hold. */
#define FUSION_DT_MIN       (0.0001f)
#define FUSION_DT_MAX       (0.2f)

/* Initial covariance: position within ~1 m, velocity within ~1 m/s, no reason
 * yet to think those errors are correlated. */
#define FUSION_P00_INIT     (1.0f)
#define FUSION_P01_INIT     (0.0f)
#define FUSION_P11_INIT     (1.0f)

#define FUSION_GRAVITY      (9.80665f)

/* --- state --------------------------------------------------------------- */

static FusionValues s_state;

/* Covariance. Symmetric, so p10 is p01 and three floats describe all four
 * entries: p00 = var(d) [m^2], p11 = var(v_d) [(m/s)^2], p01 = cov(d,v_d)
 * [m^2/s]. p01 is what lets a barometer, which measures only d, correct v_d. */
static float32 s_p00;
static float32 s_p01;
static float32 s_p11;

static uint32  s_rejectRun;    /* consecutive rejections           */

static float32 s_baroAlt;      /* latched sample [m], positive UP  */
static boolean s_baroNew;      /* a sample is waiting              */
static float32 s_baroRef;      /* first valid sample = the origin  */
static boolean s_baroRefOk;

void Fusion_init(void)
{
    s_state.a_D     = 0.0f;
    s_state.a_d     = 0.0f;
    s_state.a_v_d   = 0.0f;
    s_state.innov   = 0.0f;
    s_state.p00     = FUSION_P00_INIT;
    s_state.rejects = 0u;
    s_state.resets  = 0u;

    s_p00 = FUSION_P00_INIT;
    s_p01 = FUSION_P01_INIT;
    s_p11 = FUSION_P11_INIT;

    s_rejectRun = 0u;

    s_baroAlt   = 0.0f;
    s_baroNew   = FALSE;
    s_baroRef   = 0.0f;
    s_baroRefOk = FALSE;
}

/* Predict: move the state forward, then grow the covariance.
 *
 * a_D is the down component of specific force with gravity removed. While the
 * board is level the projection collapses to this one line (fusion.h). */
static void Fusion_predict(const float32 acc[3], float32 dt)
{
    const float32 dt2 = dt * dt;
    const float32 qa  = FUSION_SIGMA_A * FUSION_SIGMA_A;

    s_state.a_D = FUSION_GRAVITY * (1.0f - acc[2]);

    /* Clamp only the impossible; real dynamics pass through untouched. */
    if (s_state.a_D > FUSION_ACC_D_MAX)
    {
        s_state.a_D = FUSION_ACC_D_MAX;
    }
    else if (s_state.a_D < -FUSION_ACC_D_MAX)
    {
        s_state.a_D = -FUSION_ACC_D_MAX;
    }
    else
    {
        /* in range */
    }

    /* Position FIRST, using the OLD velocity: d integrates the velocity that
     * was valid across this interval, plus half a*dt^2 because the velocity
     * ramps rather than jumping. Updating a_v_d first would double that term. */
    s_state.a_d   += (s_state.a_v_d * dt) + (s_state.a_D * dt2 * 0.5f);
    s_state.a_v_d += s_state.a_D * dt;

    /* P = F*P*F' + Q, multiplied out for F = [[1, dt], [0, 1]]. Q comes from a
     * single acceleration error feeding both states: sigma_a*dt into velocity
     * and sigma_a*dt^2/2 into position, and the entries are the products of
     * those two — hence dt^4/4, dt^3/2, dt^2.
     *
     * Order is load-bearing: every right-hand side must see the OLD values, so
     * p00 (which needs old p01 and p11) comes before p01 (which needs old p11)
     * which comes before p11. */
    s_p00 += (2.0f * dt * s_p01) + (dt2 * s_p11) + ((qa * dt2 * dt2) / 4.0f);
    s_p01 += (dt * s_p11) + ((qa * dt2 * dt) / 2.0f);
    s_p11 += qa * dt2;
}

/* Correct with the latched barometer sample. H = [1, 0], so S is a scalar and
 * the gain is a division — no matrix inverse anywhere in this filter. */
static void Fusion_correctBaro(void)
{
    const float32 z = -(s_baroAlt - s_baroRef);   /* NED: baro counts UP */
    const float32 y = z - s_state.a_d;            /* the innovation      */
    const float32 s = s_p00 + FUSION_R_BARO;

    s_state.innov = y;

    if ((y * y) > (FUSION_GATE_SIGMA_SQ * s))
    {
        /* Outlier. Counting these matters: a rising count is a sensor fault,
         * not a tuning knob. */
        s_state.rejects++;
        s_rejectRun++;

        if (s_rejectRun >= FUSION_REJECT_MAX)
        {
            /* Half a second of unbroken disagreement. The barometer is not the
             * one that is wrong: re-acquire from it and start over. */
            s_state.a_d   = z;
            s_state.a_v_d = 0.0f;
            s_p00         = FUSION_P00_INIT;
            s_p01         = FUSION_P01_INIT;
            s_p11         = FUSION_P11_INIT;
            s_rejectRun   = 0u;
            s_state.resets++;
        }
        else
        {
            /* Scaling the whole matrix by a positive constant keeps it a valid
             * covariance while widening the gate for the next sample. */
            s_p00 *= FUSION_REJECT_INFLATE;
            s_p01 *= FUSION_REJECT_INFLATE;
            s_p11 *= FUSION_REJECT_INFLATE;
        }
    }
    else
    {
        s_rejectRun = 0u;
        const float32 k0 = s_p00 / s;
        const float32 k1 = s_p01 / s;

        s_state.a_d   += k0 * y;
        s_state.a_v_d += k1 * y;

        /* P = (I - K*H)*P, multiplied out. p11 uses the OLD p01, so it is
         * computed before p01 is overwritten. */
        s_p11 -= k1 * s_p01;
        s_p00 -= k0 * s_p00;
        s_p01 -= k0 * s_p01;
    }
}

void Fusion_update(FusionValues *fusion, const float32 acc[3], float32 dt, boolean valid)
{
    if ((valid != FALSE) && (dt > FUSION_DT_MIN) && (dt < FUSION_DT_MAX))
    {
        Fusion_predict(acc, dt);

        if (s_baroNew != FALSE)
        {
            Fusion_correctBaro();
            s_baroNew = FALSE;
        }
    }
    else
    {
        /* Freeze rather than integrate a stale sample or a nonsense interval.
         * The first call after boot lands here: dt is then whatever the STM
         * counter held, up to 42.9 s, and integrating that would put the
         * estimate kilometres away before the barometer could pull it back. */
    }

    s_state.p00 = s_p00;

    fusion->a_D     = s_state.a_D;
    fusion->a_d     = s_state.a_d;
    fusion->a_v_d   = s_state.a_v_d;
    fusion->innov   = s_state.innov;
    fusion->p00     = s_state.p00;
    fusion->rejects = s_state.rejects;
    fusion->resets  = s_state.resets;
}

void Fusion_setBaroAlt(float32 altM, boolean valid)
{
    if (valid != FALSE)
    {
        s_baroAlt = altM;
        s_baroNew = TRUE;

        /* The first valid sample defines d = 0. Taking the reference here, not
         * at power-on, keeps the filter working in RELATIVE altitude: the
         * sea-level formula carries weather bias, and none of that belongs in
         * the state. */
        if (s_baroRefOk == FALSE)
        {
            s_baroRef   = altM;
            s_baroRefOk = TRUE;
        }
    }
}
