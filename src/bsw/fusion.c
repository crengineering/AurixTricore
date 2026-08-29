/**********************************************************************************************************************
 * \file fusion.c
 * \brief Navigation filter — see fusion.h.
 *********************************************************************************************************************/
#include "fusion.h"
#include "FusionCal.h"
#include "FusionLatch.h"
#include "SharedRam.h"
#include <math.h>

/* --- tuning: vertical channel -------------------------------------------- */

/* Accelerometer process noise driving the DOWN prediction, as a POWER
 * SPECTRAL DENSITY [m/s^2/sqrt(Hz)] -- NOT a per-tick sigma. This used to be
 * "the acceleration error held over one interval", which made the effective
 * noise inversely proportional to the update rate: raising the estimator
 * from 50 Hz to 1014 Hz (PR #15) silently divided it by 20.3 and stretched
 * the velocity correction time constant from 1.5 s to 6 s. See
 * docs/NAV_TUNING.md. The continuous white-noise-acceleration model used
 * below (fusion_chanPredict) integrates this PSD exactly, so the same value
 * now gives the same covariance growth at any tick rate.
 *
 * 0.0424 = 0.3 * sqrt(0.02): the old 50 Hz-tuned per-tick sigma (0.3 m/s^2 at
 * dt = 0.02 s) converted to the PSD it was equivalent to, which is well above
 * the measured noise floor (0.0144 m/s^2 over a 30 s bench record) because
 * the dominant error is not the sensor: it is attitude error feeding into the
 * projection, plus whatever the airframe does that a constant-acceleration
 * model does not describe. Raise it if the estimate lags, lower it if the
 * output is as noisy as the raw barometer. */
#define FUSION_SIGMA_A_D        (0.0424f)

/* Accelerometer bias random walk [m/s^2 per sqrt(s)]. Already rate-invariant
 * (applied as *dt below), unlike FUSION_SIGMA_A_D/A_H above -- untouched by
 * the PSD reparametrisation. Measured: the down-axis bias moved 0.0003 m/s^2
 * across 60 s at constant temperature, i.e. about 4e-5. Set higher so the
 * state can also follow thermal drift, which the bench record was too short
 * and too isothermal to show. Fallback for g_fusionCal.sigmaAccRw, which
 * makes this live-tunable (docs/NAV_TUNING.md section 4.3). */
#define FUSION_SIGMA_ACC_RW     (1.0e-4f)

/* Barometer noise [m], MEASURED on this board: 30 s at rest gave a standard
 * deviation of 0.0197 m, and the innovation of the running filter later
 * confirmed it at 0.0181 m. R is a VARIANCE, so it is that value squared — the
 * covariance algebra only adds up in squared units. */
#define FUSION_SIGMA_BARO       (0.0197f)

/* Barometer bias random walk [m per sqrt(s)]. This is the WEATHER, not the
 * sensor: the board sat still on the desk and the reported altitude walked
 * 0.19 m in 60 s. sqrt(60) * 0.025 reproduces that, so 0.025 it is.
 *
 * This is the reason the barometer gets a bias state at all. Its short-term
 * noise is 2 cm and its long-term wander is metres, and no single R describes
 * both — inflating R to cover the drift would throw away the 2 cm precision
 * that makes the barometer worth having. */
#define FUSION_SIGMA_BARO_RW    (0.025f)

/* ...and the time constant it reverts over [s].
 *
 * The barometer bias is NOT a free random walk, and modelling it as one has a
 * sting in the tail. Only the SUM (d + bias) is observable from a barometer;
 * split individually the two are free to wander in opposite directions
 * forever, so var(d) grows without bound even though the filter is tracking
 * altitude perfectly. Measured on the bench: var(d) reached 0.51 m^2 in 25 s
 * and was still climbing, which would eventually have hit FUSION_P_MAX and
 * tripped the health check for no real reason.
 *
 * Weather pressure does not actually wander to infinity, so the honest model
 * is mean-reverting: the bias decays toward zero over this time constant,
 * which gives var(bias) a steady state of sigma^2*tau/2 (about 0.43 m of
 * standard deviation here) and bounds var(d) with it. 600 s is the order of
 * time over which barometric pressure genuinely moves. */
#define FUSION_TAU_BARO_BIAS_S  (600.0f)

/* --- tuning: horizontal channels ----------------------------------------- */

/* Accelerometer process noise driving the NORTH/EAST prediction, as a PSD
 * [m/s^2/sqrt(Hz)] -- see FUSION_SIGMA_A_D above for why this is a PSD and
 * not a per-tick sigma. 0.0707 = 0.5 * sqrt(0.02), the same 50 Hz-equivalent
 * conversion. Larger than the vertical because the error source is
 * different: horizontal acceleration is contaminated by ATTITUDE error, and
 * one degree of roll or pitch error tips 0.17 m/s^2 of gravity straight into
 * the horizontal plane. This number is really a statement about how well the
 * AHRS is doing. */
#define FUSION_SIGMA_A_H        (0.0707f)

/* GNSS velocity noise [m/s]. UBX-NAV-PVT carries an sAcc field that this
 * driver does not decode; until it does, a fixed value covers a NEO-M9N with a
 * clear view. Pessimistic on purpose — velocity is the measurement that makes
 * the horizontal accelerometer bias observable, and over-trusting it would let
 * GNSS noise be absorbed as bias. */
#define FUSION_SIGMA_GNSS_VEL   (0.3f)

/* Floor on the reported horizontal accuracy [m]. The receiver is optimistic
 * about hAcc under a clear sky, and a too-small R makes the filter chase
 * multipath. */
#define FUSION_GNSS_HACC_MIN    (1.0f)

/* --- outlier gates -------------------------------------------------------- */

/* Reject a sample further than this many sigma from what the filter expected.
 * One corrupt reading otherwise yanks the estimate AND shrinks P as though it
 * were good information, after which the filter rejects the next genuine one. */
#define FUSION_GATE_SIGMA_SQ    (25.0f)

/* ...but never gate tighter than this, in metres.
 *
 * Measured 2026-08-26, and the reason this floor exists: with p00 converged to
 * 4.06e-05 the 5-sigma gate was only +/-0.104 m, and an ordinary 30 cm hand
 * lift already produced a 0.064 m innovation — 62 percent of the way to being
 * rejected. A gate that fires on real motion is worse than no gate: it drops
 * the barometer exactly when the filter needs it most.
 *
 * The gate is there to catch a CORRUPT reading, which is wrong by tens of
 * metres, not to police dynamics. A floor of 2 m separates the two cleanly. */
#define FUSION_GATE_MIN_M       (2.0f)

/* Same idea for GNSS, in units of the receiver-reported accuracy. */
#define FUSION_GNSS_GATE_K      (5.0f)
#define FUSION_GNSS_GATE_MIN_M  (10.0f)

/* ESCAPE HATCH for the barometer gate, and it is not optional.
 *
 * Rejecting a sample also skips the covariance update, so a filter whose state
 * has been thrown far off rejects every subsequent measurement — and because
 * the innovation grows with the diverging state faster than the gate threshold
 * grows with P, it never recovers on its own. Observed 2026-08-25: a knock
 * against the board (-8.2 g) left the estimate at 293 m and climbing, with
 * 4282 consecutive rejections at the full 50 Hz.
 *
 * Two layers. Every rejection INFLATES P, so an estimate the filter has reason
 * to doubt stops being defended and the gate reopens on its own. If that is
 * still not enough after FUSION_REJECT_MAX consecutive samples (0.5 s at
 * 50 Hz), the state is simply wrong: re-acquire from the barometer rather than
 * defend a fiction. */
#define FUSION_REJECT_INFLATE   (2.0f)
#define FUSION_REJECT_MAX       (25u)

/* Sanity bound on the acceleration input [m/s^2]. NOT a manoeuvre limit — real
 * flight accelerations must pass through untouched, that is what the
 * accelerometer is for. This catches only physically impossible values, i.e. a
 * corrupted SPI transfer, at roughly 10 g. */
#define FUSION_ACC_MAX          (100.0f)

/* Admissible bands for values arriving from outside, used by fusion_usable().
 * Deliberately generous -- these reject garbage, they do not police physics.
 * FUSION_ALT_MAX is well above any altitude this airframe reaches and well
 * below the point where float32 loses metre resolution. */
/* Bound on the relative sensor's offset [m]. METEOROLOGICAL, not numerical:
 * a barometer's zero drifts with the weather, and the weather does not move
 * the apparent altitude by more than a few tens of metres over a flight.
 *
 * This is the safety net for the one direction the filter cannot see. Only the
 * SUM (position + measBias) is observable from a barometer, so corrupt input
 * can drive the two apart without the innovation noticing -- measured at 438 m
 * after a 30 s corrupt burst, with the sum still correct to 0.2 mm and the
 * filter therefore looking perfectly healthy. Re-acquisition alone does not
 * fix it, because the split is driven again after the last reset, and mean
 * reversion at 600 s is far too slow to matter inside a flight. */
#define FUSION_MEASB_MAX        (50.0f)

#define FUSION_INPUT_MAX        (1.0e6f)
#define FUSION_ALT_MAX          (1.0e6f)

/* Sanity bounds on dt [s]. The IMU task measures ~1014 Hz on this board (was
 * 50 Hz before PR #15 -- this comment was stale). The lower bound keeps a
 * double call from dividing by nothing; the upper bound matters at boot, where
 * the first measured interval is whatever the STM counter happened to hold. */
#define FUSION_DT_MIN           (0.0001f)
#define FUSION_DT_MAX           (0.2f)

/* Initial covariance. Position and velocity within ~1 m and ~1 m/s; the
 * accelerometer bias within 0.1 m/s^2 (the measured value was 0.018). The
 * measurement-bias state starts at zero variance and gains it only when a
 * relative sensor is actually attached to the channel — see Fusion_init. */
#define FUSION_P_POS_INIT       (1.0f)
#define FUSION_P_VEL_INIT       (1.0f)
#define FUSION_P_ACCB_INIT      (0.01f)
#define FUSION_P_MEASB_INIT     (1.0f)

/* Ceiling on a variance before it is treated as divergence rather than
 * uncertainty. 1e6 m^2 is a kilometre of standard deviation — far past any
 * honest doubt about where this board is. */
#define FUSION_P_MAX            (1.0e6f)

/* Tangent plane. Metres per degree of latitude, and the equatorial value for
 * longitude which is then scaled by cos(lat). Good to a few parts in a
 * thousand over the few km this ever has to cover — the error is a slow scale
 * factor on the distance from the origin, not a position jump. */
#define FUSION_M_PER_DEG_LAT    (111132.0f)
#define FUSION_M_PER_DEG_LON    (111320.0f)
#define FUSION_DEG_TO_RAD       (0.017453293f)
#define FUSION_1E7_TO_DEG       (1.0e-7f)

/* State indices, shared by all three channels. */
#define FS_POS      (0u)
#define FS_VEL      (1u)
#define FS_ACCB     (2u)
#define FS_MEASB    (3u)
#define FS_N        (4u)

/* --- one channel ---------------------------------------------------------- */

typedef struct
{
    float32 x[FS_N];            /* pos [m], vel [m/s], accBias, measBias   */
    float32 p[FS_N][FS_N];      /* covariance, kept symmetric              */
    /* Pointers INTO the calibration block, not copies: the filter must see a
     * tuning write on the next tick, and a copy taken at init never would. */
    volatile const float32 *sigmaA;       /* process noise on acceleration    */
    volatile const float32 *sigmaMeasRw;  /* NULL when the channel has no
                                           * relative sensor (north/east)     */
    float32 sigmaADefault;      /* fallback if the calibration value is bad.
                                 * Per channel: the horizontal channels carry
                                 * more process noise than the vertical one,
                                 * so falling back to the vertical default
                                 * would quietly mistune them.               */
    float32 pMeasBInit;         /* initial variance of the measBias state  */
    uint32  rejectRun;
    boolean anchored;           /* an absolute or relative fix has arrived */
} Fusion_Chan;

/* Covariance re-initialisations forced by the health check in
 * fusion_chanPredict. Published: a non-zero value means the filter hit a
 * numerical problem, which is something to investigate rather than tune. */
static uint32 s_covResets;

static Fusion_Chan s_chD;       /* down  */
static Fusion_Chan s_chN;       /* north */
static Fusion_Chan s_chE;       /* east  */

/* s_navState, not s_state -- see the note in Ahrs.c (MISRA 5.9). */
static FusionValues s_navState;

/* CPU1-side cache of the last CONSUMED barometer/GNSS sample, refreshed from
 * g_baroLatch/g_gnssLatch (FusionLatch.h) inside Fusion_update(). The setters
 * (Fusion_setBaroAlt/Fusion_setGnss, called from CPU0) no longer write these
 * directly -- see baroLatch_get()/gnssLatch_get() below and
 * docs/REFACTORING_PLAN.md 2.4/T12. s_baroLastGen/s_gnssLastGen are the
 * consumer's OWN bookkeeping of "have I fused this generation yet"; they are
 * never shared and never written by the producer, which is what fixes the
 * old s_baroNew/s_gnssNew two-writer bug (the producer set it, the consumer
 * cleared it -- two cores writing the same word). */
static float32 s_baroAlt;
static float32 s_baroRef;
static uint32  s_baroLastGen;

static sint32  s_gnssLat;
static sint32  s_gnssLon;
static float32 s_gnssAlt;
static float32 s_gnssSpeed;
static float32 s_gnssHeading;
static float32 s_gnssHAcc;
static uint32  s_gnssLastGen;

/* GNSS duplicate-of-the-last-fix guard. Read and written ONLY by
 * Fusion_setGnss() (CPU0), never by the consumer, so -- unlike the fields
 * above -- these carry no cross-core visibility requirement and stay plain
 * statics rather than moving into g_gnssLatch (FusionLatch.h). */
static uint32  s_lastITow;
static boolean s_haveITow;

/* tangent-plane origin */
static sint32  s_originLat;
static sint32  s_originLon;
static float32 s_originAlt;
static float32 s_originAltOffset;   /* d at the instant the origin was set */
static float32 s_mPerDegLon;
static boolean s_originOk;

/* Put the covariance back to its initial value, off-diagonals included.
 *
 * The off-diagonals matter as much as the diagonal here. An earlier version of
 * the re-acquisition path reset only p00/p11/p22 and left the cross terms
 * holding whatever they had diverged to; the very next predict step mixed them
 * straight back into the diagonal and the filter was poisoned again inside one
 * tick. If the covariance is being thrown away, throw all of it away. */
static void fusion_chanResetCov(Fusion_Chan *ch)
{
    uint8 i;
    uint8 j;

    for (i = 0u; i < FS_N; i++)
    {
        for (j = 0u; j < FS_N; j++)
        {
            ch->p[i][j] = 0.0f;
        }
    }

    ch->p[FS_POS][FS_POS]     = FUSION_P_POS_INIT;
    ch->p[FS_VEL][FS_VEL]     = FUSION_P_VEL_INIT;
    ch->p[FS_ACCB][FS_ACCB]   = FUSION_P_ACCB_INIT;
    ch->p[FS_MEASB][FS_MEASB] = ch->pMeasBInit;
}

/* Hold the covariance at "maximally uncertain" without discarding it. Scaling
 * the whole matrix by one positive factor keeps it a valid covariance and
 * preserves every correlation the filter has learned. */
static void fusion_chanClampCov(Fusion_Chan *ch)
{
    float32 worst = ch->p[FS_POS][FS_POS];
    uint8   i;
    uint8   j;

    if (ch->p[FS_VEL][FS_VEL] > worst)
    {
        worst = ch->p[FS_VEL][FS_VEL];
    }
    else
    {
        /* position is the more uncertain of the two */
    }

    if (worst > FUSION_P_MAX)
    {
        const float32 scale = FUSION_P_MAX / worst;

        for (i = 0u; i < FS_N; i++)
        {
            for (j = 0u; j < FS_N; j++)
            {
                ch->p[i][j] *= scale;
            }
        }
    }
    else
    {
        /* nothing to do */
    }
}

static void fusion_chanInit(Fusion_Chan *ch, volatile const float32 *sigmaA,
                            float32 sigmaADefault,
                            volatile const float32 *sigmaMeasRw, float32 pMeasB)
{
    uint8 i;

    for (i = 0u; i < FS_N; i++)
    {
        ch->x[i] = 0.0f;
    }

    ch->sigmaA        = sigmaA;
    ch->sigmaADefault = sigmaADefault;
    ch->sigmaMeasRw   = sigmaMeasRw;
    ch->pMeasBInit  = pMeasB;
    ch->rejectRun   = 0u;
    ch->anchored    = FALSE;

    fusion_chanResetCov(ch);
}

/* Predict: move the state forward, then grow the covariance.
 *
 * The model is constant acceleration with an unknown constant offset:
 *   pos' = pos + vel*dt + (a - accBias)*dt^2/2
 *   vel' = vel        + (a - accBias)*dt
 * so the state-transition matrix carries the bias into both, with a minus
 * sign. That coupling is the whole point — it is what lets a position sensor
 * that never sees acceleration nevertheless work out the accelerometer offset. */
static void fusion_chanPredict(Fusion_Chan *ch, float32 a, float32 dt)
{
    const float32 dt2  = dt * dt;
    /* Process noise is read from the calibration block every tick. ch->sigmaA
     * selects WHICH parameter this channel uses; the value itself is live.
     * sa is a PSD [m/s^2/sqrt(Hz)], not a per-tick sigma -- see
     * FUSION_SIGMA_A_D/A_H. qa = sa^2 is therefore the PSD proper,
     * [m^2/s^3], and the Q terms below (qa*dt^3/3, qa*dt^2/2, qa*dt) are the
     * exact continuous-time white-noise-acceleration integral, rate-invariant
     * by construction. */
    const float32 sa   = FusionCal_positive(*ch->sigmaA, 0.0f, ch->sigmaADefault);
    const float32 sm   = (ch->sigmaMeasRw != NULL_PTR)
                       ? FusionCal_positive(*ch->sigmaMeasRw, 0.0f,
                                            FUSION_SIGMA_BARO_RW)
                       : 0.0f;
    /* Accelerometer bias random walk. One cal field shared by all three
     * channels (unlike sigmaA above) because it is a statement about the
     * SENSOR, not about which axis is being predicted. */
    const float32 rw   = FusionCal_positive(g_fusionCal.sigmaAccRw, 0.0f,
                                            FUSION_SIGMA_ACC_RW);
    const float32 qa   = sa * sa;
    const float32 qm   = sm * sm;
    const float32 f01  = dt;
    const float32 f02  = -0.5f * dt2;
    const float32 f12  = -dt;
    const float32 tau  = (ch->sigmaMeasRw != NULL_PTR)
                       ? FusionCal_positive(g_fusionCal.tauBaroBias, 1.0f,
                                            FUSION_TAU_BARO_BIAS_S)
                       : 0.0f;
    const float32 f33  = (tau > 0.0f) ? (1.0f - (dt / tau)) : 1.0f;
    const float32 aEff = a - ch->x[FS_ACCB];
    float32 t[FS_N][FS_N];
    uint8   i;
    uint8   j;

    ch->x[FS_POS] += (ch->x[FS_VEL] * dt) + (0.5f * aEff * dt2);
    ch->x[FS_VEL] += aEff * dt;
    ch->x[FS_MEASB] *= f33;
    /* accBias is a constant plus random walk; measBias reverts to zero */

    /* T = F * P. F is the identity plus three off-diagonal terms, so only the
     * first two rows change. */
    for (j = 0u; j < FS_N; j++)
    {
        t[FS_POS][j]   = ch->p[FS_POS][j] + (f01 * ch->p[FS_VEL][j])
                       + (f02 * ch->p[FS_ACCB][j]);
        t[FS_VEL][j]   = ch->p[FS_VEL][j] + (f12 * ch->p[FS_ACCB][j]);
        t[FS_ACCB][j]  = ch->p[FS_ACCB][j];
        t[FS_MEASB][j] = f33 * ch->p[FS_MEASB][j];
    }

    /* P = T * F'. Transposing F turns its ROWS into the combination applied to
     * each output COLUMN, so column POS collects the whole of F row 0 and
     * column VEL collects F row 1 — the same coefficients as above, but
     * gathered differently. Getting this the wrong way round is not a subtle
     * error: it drove p00 to -inf on the bench, after which S went negative,
     * the gate stopped meaning anything and the estimate free-integrated to
     * 166 m. */
    for (i = 0u; i < FS_N; i++)
    {
        const float32 tPos  = t[i][FS_POS];
        const float32 tVel  = t[i][FS_VEL];
        const float32 tAccb = t[i][FS_ACCB];

        ch->p[i][FS_POS]   = tPos + (f01 * tVel) + (f02 * tAccb);
        ch->p[i][FS_VEL]   = tVel + (f12 * tAccb);
        ch->p[i][FS_ACCB]  = tAccb;
        ch->p[i][FS_MEASB] = f33 * t[i][FS_MEASB];
    }

    /* Q, continuous white-noise-acceleration model: acceleration is white
     * noise with PSD qa, and pos/vel are its first two integrals, so their
     * covariance growth over one tick is the exact integral of that PSD
     * through the state-transition matrix --
     *   p[pos][pos] += qa*dt^3/3, p[pos][vel] += qa*dt^2/2, p[vel][vel] += qa*dt
     * -- rate-invariant by construction: half the tick at twice the rate
     * gives the same total over one second, unlike the old piecewise-
     * constant-acceleration form this replaces (qa*dt2*dt2/4 etc., which
     * silently divided the effective PSD by the tick rate -- see
     * docs/NAV_TUNING.md and FUSION_SIGMA_A_D/A_H above). The two bias states
     * still get random walks, which grow linearly in dt regardless of this
     * change. */
    ch->p[FS_POS][FS_POS]     += (qa * dt2 * dt) / 3.0f;
    ch->p[FS_POS][FS_VEL]     += (qa * dt2) / 2.0f;
    ch->p[FS_VEL][FS_POS]      = ch->p[FS_POS][FS_VEL];
    ch->p[FS_VEL][FS_VEL]     += qa * dt;
    ch->p[FS_ACCB][FS_ACCB]   += rw * rw * dt;
    ch->p[FS_MEASB][FS_MEASB] += qm * dt;

    /* Numerical health check, and it earns its keep: a variance that has gone
     * negative, infinite or NaN is not a filter that is merely mistuned, it is
     * one whose gate and gain have stopped meaning anything. Worse, NaN
     * poisons the escape hatch too — every comparison against NaN is false, so
     * the rejection counter never advances and the re-acquisition that would
     * have saved the filter never fires. Catch it here, at the one place every
     * channel passes through on every tick.
     *
     * The comparison is written as "not greater than zero" on purpose: that is
     * TRUE for NaN, where "less than or equal" would be false. */
    if (!((ch->p[FS_POS][FS_POS] > 0.0f) && (ch->p[FS_VEL][FS_VEL] > 0.0f)))
    {
        /* Negative or NaN. Not saturation — corruption. */
        fusion_chanResetCov(ch);
        ch->rejectRun = 0u;
        s_covResets++;
    }
    else if ((ch->p[FS_POS][FS_POS] > FUSION_P_MAX)
             || (ch->p[FS_VEL][FS_VEL] > FUSION_P_MAX))
    {
        /* Honest saturation, NOT a fault, so it deliberately does not count as
         * a health-check trip. A channel with no absolute reference — north
         * and east indoors, where GNSS never gets a fix — has a position
         * variance that grows without bound, because that is the truth: after
         * long enough the filter genuinely has no idea where it is. Clamping
         * says "maximally uncertain" and keeps the arithmetic finite; resetting
         * would falsely claim the estimate had improved. */
        fusion_chanClampCov(ch);
    }
    else
    {
        /* covariance is finite and positive */
    }
}

/* One scalar measurement update. h selects which linear combination of the
 * state the sensor sees, so the same routine serves an absolute position fix
 * (h = [1,0,0,0]), a relative one carrying a bias (h = [1,0,0,1]) and a
 * velocity fix (h = [0,1,0,0]).
 *
 * \return TRUE if the sample passed the gate and was fused. */
static boolean fusion_chanUpdate(Fusion_Chan *ch, const float32 h[FS_N],
                                 float32 z, float32 r, float32 gateSq,
                                 float32 gateMinSq, float32 *innovOut)
{
    float32 ph[FS_N];
    float32 hx = 0.0f;
    float32 s  = r;
    float32 y;
    float32 threshold;
    boolean accepted = FALSE;
    uint8   i;
    uint8   j;

    for (i = 0u; i < FS_N; i++)
    {
        float32 acc = 0.0f;

        for (j = 0u; j < FS_N; j++)
        {
            acc += ch->p[i][j] * h[j];
        }

        ph[i] = acc;
        hx   += h[i] * ch->x[i];
        s    += h[i] * acc;
    }

    y = z - hx;
    *innovOut = y;

    threshold = gateSq * s;

    if (threshold < gateMinSq)
    {
        /* The statistical gate has closed tighter than physical sense allows —
         * see FUSION_GATE_MIN_M. Use the floor instead. */
        threshold = gateMinSq;
    }
    else
    {
        /* the covariance-derived gate is the wider of the two */
    }

    if ((y * y) > threshold)
    {
        ch->rejectRun++;
    }
    else if (s > 0.0f)
    {
        const float32 recipS = 1.0f / s;

        ch->rejectRun = 0u;

        for (i = 0u; i < FS_N; i++)
        {
            ch->x[i] += (ph[i] * recipS) * y;
        }

        /* P -= K * (H*P). Written as an outer product of ph with itself over
         * s, which is symmetric by construction — so the covariance cannot
         * drift out of symmetry however long this runs. */
        for (i = 0u; i < FS_N; i++)
        {
            for (j = 0u; j < FS_N; j++)
            {
                ch->p[i][j] -= (ph[i] * ph[j]) * recipS;
            }
        }

        ch->anchored = TRUE;
        accepted     = TRUE;
    }
    else
    {
        /* S <= 0 is numerically impossible for a valid covariance; refuse to
         * divide by it rather than propagate a NaN through the whole filter. */
    }

    return accepted;
}

/* Hold the unobservable split inside physical bounds.
 *
 * The excess is moved INTO the position rather than discarded, so the
 * observable sum is untouched -- this constrains only the direction the
 * measurement cannot see, and never contradicts the measurement itself. */
static void fusion_boundMeasBias(Fusion_Chan *ch)
{
    if (ch->x[FS_MEASB] > FUSION_MEASB_MAX)
    {
        const float32 excess = ch->x[FS_MEASB] - FUSION_MEASB_MAX;
        ch->x[FS_MEASB] = FUSION_MEASB_MAX;
        ch->x[FS_POS]  += excess;
    }
    else if (ch->x[FS_MEASB] < -FUSION_MEASB_MAX)
    {
        const float32 excess = ch->x[FS_MEASB] + FUSION_MEASB_MAX;
        ch->x[FS_MEASB] = -FUSION_MEASB_MAX;
        ch->x[FS_POS]  += excess;
    }
    else
    {
        /* inside the physical range */
    }
}

/* Widen the covariance after a rejection so the gate reopens on its own. */
static void fusion_chanInflate(Fusion_Chan *ch)
{
    uint8 i;
    uint8 j;

    /* Scaling the whole matrix by a positive constant keeps it a valid
     * covariance while widening the gate for the next sample. */
    for (i = 0u; i < FS_N; i++)
    {
        for (j = 0u; j < FS_N; j++)
        {
            ch->p[i][j] *= FUSION_REJECT_INFLATE;
        }
    }
}

void Fusion_init(void)
{
    /* Only the DOWN channel has a relative sensor (the barometer) sitting
     * alongside an absolute one, so it is the only channel whose measurement
     * bias is observable. North and east get zero initial variance and zero
     * random walk, which pins that state at exactly zero for good. */
    fusion_chanInit(&s_chD, &g_fusionCal.sigmaAccD, FUSION_SIGMA_A_D,
                    &g_fusionCal.sigmaBaroRw, FUSION_P_MEASB_INIT);
    fusion_chanInit(&s_chN, &g_fusionCal.sigmaAccH, FUSION_SIGMA_A_H,
                    NULL_PTR, 0.0f);
    fusion_chanInit(&s_chE, &g_fusionCal.sigmaAccH, FUSION_SIGMA_A_H,
                    NULL_PTR, 0.0f);

    s_baroAlt     = 0.0f;
    s_baroRef     = 0.0f;
    s_baroLastGen = 0u;

    s_gnssLat     = 0;
    s_gnssLon     = 0;
    s_gnssAlt     = 0.0f;
    s_gnssSpeed   = 0.0f;
    s_gnssHeading = 0.0f;
    s_gnssHAcc    = 0.0f;
    s_gnssLastGen = 0u;

    /* g_baroLatch/g_gnssLatch (FusionLatch.h): the PRODUCER's state, zeroed
     * here even though Fusion_init() itself runs on CPU1 (via NavTask_init,
     * T12). Safe by construction, not by synchronisation: SensorTask_baro/
     * SensorTask_gnss (the only writers) do not run until CPU0's own
     * Scheduler_run() loop starts, which is the LAST thing core0_main() does
     * -- strictly after CPU1 has already reached this point in its own,
     * much shorter boot sequence. There is no concurrent access at boot,
     * just two sequential writes in real time, so no barrier is needed for
     * THIS store; Ifx__dsync() still guards every ONGOING publish below. */
    g_baroLatch.gen          = 0u;
    g_baroLatch.altM         = 0.0f;
    g_baroLatch.refM         = 0.0f;
    g_baroLatch.refOk        = 0u;
    g_baroLatch.droppedCount = 0u;
    g_baroLatch.reserved     = 0u;

    g_gnssLatch.gen          = 0u;
    g_gnssLatch.lat          = 0;
    g_gnssLatch.lon          = 0;
    g_gnssLatch.altM         = 0.0f;
    g_gnssLatch.speedMps     = 0.0f;
    g_gnssLatch.headingDeg   = 0.0f;
    g_gnssLatch.hAccM        = 0.0f;
    g_gnssLatch.iTOW         = 0u;
    g_gnssLatch.droppedCount = 0u;
    g_gnssLatch.dupesCount   = 0u;

    s_originLat       = 0;
    s_originLon       = 0;
    s_originAlt       = 0.0f;
    s_originAltOffset = 0.0f;
    s_mPerDegLon      = FUSION_M_PER_DEG_LON;
    s_originOk        = FALSE;
    s_lastITow        = 0u;
    s_haveITow        = FALSE;
    s_covResets       = 0u;

    s_navState.a_D          = 0.0f;
    s_navState.a_d          = 0.0f;
    s_navState.a_v_d        = 0.0f;
    s_navState.accBiasD     = 0.0f;
    s_navState.baroBias     = 0.0f;
    s_navState.innov        = 0.0f;
    s_navState.p00          = FUSION_P_POS_INIT;
    s_navState.a_N          = 0.0f;
    s_navState.a_E          = 0.0f;
    s_navState.posN         = 0.0f;
    s_navState.posE         = 0.0f;
    s_navState.velN         = 0.0f;
    s_navState.velE         = 0.0f;
    s_navState.accBiasN     = 0.0f;
    s_navState.accBiasE     = 0.0f;
    s_navState.innovN       = 0.0f;
    s_navState.innovE       = 0.0f;
    s_navState.innovVelN    = 0.0f;
    s_navState.innovVelE    = 0.0f;
    s_navState.pNN          = FUSION_P_POS_INIT;
    s_navState.originLatDeg = 0.0f;
    s_navState.originLonDeg = 0.0f;
    s_navState.originAltM   = 0.0f;
    s_navState.rejects      = 0u;
    s_navState.resets       = 0u;
    s_navState.gnssRejects  = 0u;
    s_navState.gnssUpdates  = 0u;
    s_navState.covResets    = 0u;
    s_navState.gnssITow     = 0u;
    s_navState.gnssDupes    = 0u;
    s_navState.dropped      = 0u;
    s_navState.verticalOk   = 0u;
    s_navState.horizontalOk = 0u;
    s_navState.originSet    = 0u;
    s_navState.reserved     = 0u;
}

/* Is this a number the filter can safely use?
 *
 * TRUE only for a finite value inside a physically sensible band. Written as a
 * positive test on both sides on purpose: every comparison against NaN is
 * FALSE, so NaN fails it, and the bounds reject both infinities.
 *
 * This exists because a NaN measurement is invisible to every other guard.
 * The outlier gate cannot catch it -- "is this innovation too large" is false
 * for NaN, so it is not an outlier. The covariance health check does not catch
 * it either, because P stays perfectly finite while the STATE goes NaN.
 * Measured: 2000 NaN barometer samples left every state NaN with zero
 * rejections, zero resets and zero health-check trips. Nothing noticed.
 *
 * So bad measurements have to be stopped at the door, before they reach any
 * arithmetic. */
static boolean fusion_usable(float32 v, float32 limit)
{
    return ((v > -limit) && (v < limit)) ? TRUE : FALSE;
}

/* Clamp an acceleration input to the physically possible. */
static float32 fusion_clampAcc(float32 a)
{
    float32 v = a;

    if (v > FUSION_ACC_MAX)
    {
        v = FUSION_ACC_MAX;
    }
    else if (v < -FUSION_ACC_MAX)
    {
        v = -FUSION_ACC_MAX;
    }
    else
    {
        /* in range */
    }

    return v;
}

/* Read one consistent snapshot of g_baroLatch/g_gnssLatch. CPU1 (the
 * consumer) ONLY -- mirrors NavState_get()'s protocol exactly (NavState.c):
 * read gen, copy the payload, re-read gen; one retry on a mismatch, then
 * give up and report FALSE so the caller keeps consuming its last-known-good
 * cache (s_baroAlt/s_baroRef, s_gnssLat/.../s_gnssHAcc) rather than a torn
 * mix of old and new fields. Never writes g_baroLatch/g_gnssLatch -- that is
 * what keeps this side of the crossing free of the old two-writer bug. */
static boolean baroLatch_get(BaroLatch_t *out)
{
    uint8   attempt;
    boolean ok = FALSE;

    for (attempt = 0u; (attempt < 2u) && (ok == FALSE); attempt++)
    {
        uint32 genBefore = g_baroLatch.gen;

        out->gen   = genBefore;
        out->altM  = g_baroLatch.altM;
        out->refM  = g_baroLatch.refM;
        out->refOk = g_baroLatch.refOk;

        if (g_baroLatch.gen == genBefore)
        {
            ok = TRUE;
        }
    }

    return ok;
}

static boolean gnssLatch_get(GnssLatch_t *out)
{
    uint8   attempt;
    boolean ok = FALSE;

    for (attempt = 0u; (attempt < 2u) && (ok == FALSE); attempt++)
    {
        uint32 genBefore = g_gnssLatch.gen;

        out->gen        = genBefore;
        out->lat        = g_gnssLatch.lat;
        out->lon        = g_gnssLatch.lon;
        out->altM       = g_gnssLatch.altM;
        out->speedMps   = g_gnssLatch.speedMps;
        out->headingDeg = g_gnssLatch.headingDeg;
        out->hAccM      = g_gnssLatch.hAccM;
        out->iTOW       = g_gnssLatch.iTOW;

        if (g_gnssLatch.gen == genBefore)
        {
            ok = TRUE;
        }
    }

    return ok;
}

/* Correct the DOWN channel with the latched barometer sample. */
static void fusion_correctBaro(void)
{
    /* The barometer is a RELATIVE sensor here: it measures d plus whatever the
     * weather has done to the reference since boot, which is the measBias
     * state. h therefore has a 1 in both slots. */
    static const float32 h[FS_N] = { 1.0f, 0.0f, 0.0f, 1.0f };
    const float32 z = -(s_baroAlt - s_baroRef);   /* NED: baro counts UP */
    const float32 sb = FusionCal_positive(g_fusionCal.sigmaBaro, 0.0f,
                                          FUSION_SIGMA_BARO);
    const float32 gm = FusionCal_positive(g_fusionCal.gateMinM, 0.0f,
                                          FUSION_GATE_MIN_M);
    const float32 gs = FusionCal_positive(g_fusionCal.gateSigmaSq, 0.0f,
                                          FUSION_GATE_SIGMA_SQ);
    float32 y = 0.0f;

    if (fusion_chanUpdate(&s_chD, h, z, sb * sb, gs, gm * gm, &y) != FALSE)
    {
        s_navState.innov = y;
    }
    else
    {
        s_navState.innov = y;
        s_navState.rejects++;

        if (s_chD.rejectRun >= FUSION_REJECT_MAX)
        {
            /* Half a second of unbroken disagreement. The barometer is not the
             * one that is wrong: re-acquire from it and start over. */
            /* Reset the WHOLE state, not part of it.
             *
             * fusion_chanResetCov() puts every variance back to its prior,
             * measBias included. Keeping the measBias MEAN while resetting its
             * VARIANCE leaves the state incoherent: the covariance says "I
             * know only the prior" while the mean still carries an accumulated
             * estimate. A Kalman state is (mean, covariance) together.
             *
             * Measured cost of getting this wrong: a corrupt burst drove the
             * barometer offset to +2125 m, and because every re-acquisition
             * faithfully preserved it, d settled at -2125 m against a true 0.
             * The observable SUM (d + measBias) stayed correct to a
             * millimetre, so the filter looked perfectly healthy while the
             * value everything actually reads was 2 km wrong.
             *
             * The cost of doing it this way is real but recoverable: measBias
             * is estimated from the barometer AND GNSS altitude, so this
             * discards GNSS-derived knowledge because the barometer
             * misbehaved. GNSS re-anchors it. An unbounded split does not
             * recover on its own. */
            s_chD.x[FS_POS]   = z;
            s_chD.x[FS_VEL]   = 0.0f;
            s_chD.x[FS_ACCB]  = 0.0f;
            s_chD.x[FS_MEASB] = 0.0f;
            fusion_chanResetCov(&s_chD);
            s_chD.rejectRun  = 0u;
            s_navState.resets++;
        }
        else
        {
            fusion_chanInflate(&s_chD);
        }
    }
}

/* Correct all three channels with the latched GNSS fix. */
static void fusion_correctGnss(void)
{
    static const float32 hPos[FS_N] = { 1.0f, 0.0f, 0.0f, 0.0f };
    static const float32 hVel[FS_N] = { 0.0f, 1.0f, 0.0f, 0.0f };
    float32 hAcc = s_gnssHAcc;
    float32 rPos;
    float32 gateMinSq;
    float32 zN;
    float32 zE;
    float32 headRad;
    float32 yN = 0.0f;
    float32 yE = 0.0f;
    boolean okN;
    boolean okE;

    if (hAcc < FUSION_GNSS_HACC_MIN)
    {
        hAcc = FUSION_GNSS_HACC_MIN;
    }
    else
    {
        /* the receiver is already being suitably modest */
    }

    /* Inflate the position R. The receiver's hAcc describes a single fix; it
     * does NOT describe ten per second, because consecutive NAV-PVT solutions
     * share most of their information. Counting them as independent is what
     * drove varN to claim 0.3 m against a measured 2.59 m. See FusionCal.h. */
    rPos      = hAcc * hAcc
              * FusionCal_positive(g_fusionCal.gnssPosRScale, 0.0f, 1.0f);
    gateMinSq = FUSION_GNSS_GATE_MIN_M * FUSION_GNSS_GATE_MIN_M;

    if (s_originOk == FALSE)
    {
        /* First usable fix defines the tangent plane. The longitude scale is
         * frozen at the origin latitude — recomputing it as the vehicle moves
         * would make the plane subtly non-Euclidean and the position estimate
         * would not close on itself. */
        const float32 latDeg = (float32)s_gnssLat * FUSION_1E7_TO_DEG;

        s_originLat  = s_gnssLat;
        s_originLon  = s_gnssLon;
        s_originAlt  = s_gnssAlt;
        s_mPerDegLon = FUSION_M_PER_DEG_LON * cosf(latDeg * FUSION_DEG_TO_RAD);

        /* Anchor GNSS altitude to whatever the vertical channel already
         * believes, so switching the barometer from "the only reference" to
         * "a relative sensor with a bias" does not step the altitude. From
         * here on, the DIFFERENCE between the two sensors is what the measBias
         * state tracks — which is the whole reason the barometer's weather
         * drift stops leaking into altitude. */
        s_originAltOffset = s_chD.x[FS_POS];

        s_originOk = TRUE;
        s_navState.originSet    = 1u;
        s_navState.originLatDeg = latDeg;
        s_navState.originLonDeg = (float32)s_gnssLon * FUSION_1E7_TO_DEG;
        s_navState.originAltM   = s_gnssAlt;
    }
    else
    {
        /* origin already established */
    }

    /* Position, as metres from the origin on the tangent plane. The difference
     * is taken in the receiver's native 1e-7 degree integers before anything
     * becomes a float: float32 holds only about seven digits, so converting
     * first would quantise the latitude to roughly 0.4 m and throw away most
     * of what the receiver is telling us. */
    {
        /* The differences are named rather than cast in place: MISRA 10.8
         * forbids casting a composite expression to a wider type, and naming
         * them also makes it obvious that the subtraction happens in the
         * receiver's integers, which is the whole point. */
        const sint32 dLat = s_gnssLat - s_originLat;
        const sint32 dLon = s_gnssLon - s_originLon;

        zN = (float32)dLat * FUSION_1E7_TO_DEG * FUSION_M_PER_DEG_LAT;
        zE = (float32)dLon * FUSION_1E7_TO_DEG * s_mPerDegLon;
    }

    okN = fusion_chanUpdate(&s_chN, hPos, zN, rPos, FUSION_GNSS_GATE_K * FUSION_GNSS_GATE_K,
                            gateMinSq, &yN);
    okE = fusion_chanUpdate(&s_chE, hPos, zE, rPos, FUSION_GNSS_GATE_K * FUSION_GNSS_GATE_K,
                            gateMinSq, &yE);

    s_navState.innovN = yN;
    s_navState.innovE = yE;

    if ((okN != FALSE) && (okE != FALSE))
    {
        s_navState.gnssUpdates++;
    }
    else
    {
        s_navState.gnssRejects++;

        if (s_chN.rejectRun >= FUSION_REJECT_MAX)
        {
            /* A run of rejected fixes means the horizontal state is wrong, not
             * the receiver. Re-acquire, exactly as the vertical channel does. */
            /* Same rule as the vertical channel: the covariance is going back
             * to its prior, so the mean must too -- all of it. These were
             * leaving accBias untouched. */
            s_chN.x[FS_POS]   = zN;
            s_chE.x[FS_POS]   = zE;
            s_chN.x[FS_VEL]   = 0.0f;
            s_chE.x[FS_VEL]   = 0.0f;
            s_chN.x[FS_ACCB]  = 0.0f;
            s_chE.x[FS_ACCB]  = 0.0f;
            s_chN.x[FS_MEASB] = 0.0f;
            s_chE.x[FS_MEASB] = 0.0f;
            fusion_chanResetCov(&s_chN);
            fusion_chanResetCov(&s_chE);
            s_chN.rejectRun = 0u;
            s_chE.rejectRun = 0u;
        }
        else
        {
            fusion_chanInflate(&s_chN);
            fusion_chanInflate(&s_chE);
        }
    }

    /* Velocity. The receiver reports it as a 2-D speed and a heading, so it
     * has to be resolved onto north and east before either channel can use it.
     * This is the measurement that makes the horizontal accelerometer bias
     * observable — position alone would leave bias and velocity entangled. */
    headRad = s_gnssHeading * FUSION_DEG_TO_RAD;

    {
        const float32 sv   = FusionCal_positive(g_fusionCal.sigmaGnssVel, 0.0f,
                                               FUSION_SIGMA_GNSS_VEL);
        const float32 rVel = sv * sv;
        const float32 vN   = s_gnssSpeed * cosf(headRad);
        const float32 vE   = s_gnssSpeed * sinf(headRad);
        const float32 gateMinVelSq = 25.0f;   /* 5 m/s: a corrupt fix, not a manoeuvre */
        float32 yvN = 0.0f;
        float32 yvE = 0.0f;

        /* Published rather than discarded: without these the velocity NIS
         * (var(y)/(P+R)) cannot be computed, and the velocity channel is
         * exactly the one the PSD reparametrisation above retunes. */
        (void)fusion_chanUpdate(&s_chN, hVel, vN, rVel, FUSION_GATE_SIGMA_SQ,
                                gateMinVelSq, &yvN);
        (void)fusion_chanUpdate(&s_chE, hVel, vE, rVel, FUSION_GATE_SIGMA_SQ,
                                gateMinVelSq, &yvE);

        s_navState.innovVelN = yvN;
        s_navState.innovVelE = yvE;
    }

    /* GNSS altitude, which is what makes the BAROMETER bias observable: the
     * barometer sees (d + bias) and this sees d, so the pair separates them.
     *
     * vAcc is typically 1.5 to 2 times hAcc because every satellite is above
     * the receiver and the vertical geometry is inherently poor, so this is
     * deliberately a weak measurement — it is here to pin the slow drift, not
     * to compete with the barometer for short-term altitude. */
    {
        const float32 zD    = -(s_gnssAlt - s_originAlt) + s_originAltOffset;
        const float32 sigma = hAcc * 2.0f;
        float32 yd = 0.0f;

        (void)fusion_chanUpdate(&s_chD, hPos, zD, sigma * sigma,
                                FUSION_GNSS_GATE_K * FUSION_GNSS_GATE_K,
                                gateMinSq, &yd);
    }

    s_navState.horizontalOk = (s_chN.anchored != FALSE) ? 1u : 0u;
}

void Fusion_update(FusionValues *fusion, const float32 accNed[3], float32 dt, boolean valid)
{
    if ((valid != FALSE) && (dt > FUSION_DT_MIN) && (dt < FUSION_DT_MAX)
        && (fusion_usable(accNed[0], FUSION_INPUT_MAX) != FALSE)
        && (fusion_usable(accNed[1], FUSION_INPUT_MAX) != FALSE)
        && (fusion_usable(accNed[2], FUSION_INPUT_MAX) != FALSE))
    {
        const float32 aN = fusion_clampAcc(accNed[0]);
        const float32 aE = fusion_clampAcc(accNed[1]);
        const float32 aD = fusion_clampAcc(accNed[2]);

        s_navState.a_N = aN;
        s_navState.a_E = aE;
        s_navState.a_D = aD;

        fusion_chanPredict(&s_chD, aD, dt);
        fusion_chanPredict(&s_chN, aN, dt);
        fusion_chanPredict(&s_chE, aE, dt);

        {
            BaroLatch_t baroSnap;

            /* gen differs from what was already fused -> a new sample is
             * waiting. A failed (torn) read is treated the same as "nothing
             * new": the cache from the last good consumption is left alone,
             * exactly as the original s_baroNew==FALSE path did. */
            if ((baroLatch_get(&baroSnap) != FALSE) && (baroSnap.gen != s_baroLastGen))
            {
                s_baroAlt     = baroSnap.altM;
                s_baroRef     = baroSnap.refM;
                s_baroLastGen = baroSnap.gen;

                fusion_correctBaro();
                fusion_boundMeasBias(&s_chD);
            }
            else
            {
                /* no new barometer sample this tick */
            }
        }

        {
            GnssLatch_t gnssSnap;

            if ((gnssLatch_get(&gnssSnap) != FALSE) && (gnssSnap.gen != s_gnssLastGen))
            {
                s_gnssLat     = gnssSnap.lat;
                s_gnssLon     = gnssSnap.lon;
                s_gnssAlt     = gnssSnap.altM;
                s_gnssSpeed   = gnssSnap.speedMps;
                s_gnssHeading = gnssSnap.headingDeg;
                s_gnssHAcc    = gnssSnap.hAccM;
                s_gnssLastGen = gnssSnap.gen;

                fusion_correctGnss();
                s_navState.gnssITow = gnssSnap.iTOW;
            }
            else
            {
                /* no new fix this tick */
            }
        }
    }
    else
    {
        /* Freeze rather than integrate a stale sample or a nonsense interval.
         * The first call after boot lands here: dt is then whatever the STM
         * counter held, up to 42.9 s, and integrating that would put the
         * estimate kilometres away before the barometer could pull it back. */
    }

    /* Producer-owned running counters (FusionLatch.h): read every tick
     * regardless of whether a new sample latched, exactly like the original
     * code, where the setters incremented s_navState.dropped/gnssDupes
     * directly and unconditionally. A single 32-bit field needs no gen
     * protection (SharedRam.h rule 3), so these are plain volatile reads. */
    s_navState.dropped   = g_baroLatch.droppedCount + g_gnssLatch.droppedCount;
    s_navState.gnssDupes = g_gnssLatch.dupesCount;

    s_navState.a_d      = s_chD.x[FS_POS];
    s_navState.a_v_d    = s_chD.x[FS_VEL];
    s_navState.accBiasD = s_chD.x[FS_ACCB];
    s_navState.baroBias = s_chD.x[FS_MEASB];
    s_navState.p00      = s_chD.p[FS_POS][FS_POS];

    s_navState.posN     = s_chN.x[FS_POS];
    s_navState.posE     = s_chE.x[FS_POS];
    s_navState.velN     = s_chN.x[FS_VEL];
    s_navState.velE     = s_chE.x[FS_VEL];
    s_navState.accBiasN = s_chN.x[FS_ACCB];
    s_navState.accBiasE = s_chE.x[FS_ACCB];
    s_navState.pNN      = s_chN.p[FS_POS][FS_POS];

    s_navState.verticalOk = (s_chD.anchored != FALSE) ? 1u : 0u;
    s_navState.covResets  = s_covResets;

    *fusion = s_navState;
}

/* CPU0 ONLY (SensorTask_baro). Writes g_baroLatch (FusionLatch.h): payload
 * fields -> Ifx__dsync() -> gen++, same discipline as NavState_publish() --
 * see SharedRam.h rule 3. droppedCount is a standalone counter and needs no
 * barrier of its own. */
void Fusion_setBaroAlt(float32 altM, boolean valid)
{
    /* A NaN or infinite altitude is dropped here rather than fused. It cannot
     * be caught later: see fusion_usable(). The barometric formula upstream is
     * a powf() of a pressure ratio, so a corrupt pressure of the wrong sign is
     * all it takes to produce one. */
    if ((valid != FALSE) && (fusion_usable(altM, FUSION_ALT_MAX) == FALSE))
    {
        /* Counted, not just dropped: a silently discarded sample makes a
         * failing sensor indistinguishable from a healthy one. */
        g_baroLatch.droppedCount = g_baroLatch.droppedCount + 1u;
    }
    else if (valid != FALSE)
    {
        g_baroLatch.altM = altM;

        /* The first valid sample defines d = 0. Taking the reference here, not
         * at power-on, keeps the filter working in RELATIVE altitude: the
         * sea-level formula carries weather bias, and none of that belongs in
         * the state. */
        if (g_baroLatch.refOk == 0u)
        {
            g_baroLatch.refM  = altM;
            g_baroLatch.refOk = 1u;
        }
        else
        {
            /* reference already taken */
        }

        /* cppcheck-suppress misra-c2012-17.3 ; deviation: Ifx__dsync() wraps
         * TASKING's __dsync() intrinsic, which has no declaration anywhere
         * cppcheck can see (SharedRam.h). */
        Ifx__dsync();
        g_baroLatch.gen = g_baroLatch.gen + 1u;
    }
    else
    {
        /* Read failed — drop the sample rather than re-fuse the last one. */
    }
}

/* CPU0 ONLY (SensorTask_gnss). Writes g_gnssLatch (FusionLatch.h) on a new
 * fix: payload -> Ifx__dsync() -> gen++ (SharedRam.h rule 3). s_lastITow/
 * s_haveITow are plain fusion.c statics, not part of the shared block --
 * only this function ever touches them (see their declaration above). */
void Fusion_setGnss(sint32 latDeg1e7, sint32 lonDeg1e7, float32 altM,
                    float32 speedMps, float32 headingDeg, float32 hAccM,
                    uint32 iTOW, boolean valid)
{
    if (valid != FALSE)
    {
        /* Solutions arrive at 10 Hz (CFG_RATE_MEAS = 100 ms, one NAV-PVT per
         * epoch) and this is polled at 10 Hz on an independent clock, so the
         * two beat against each other: most polls carry a new fix, some carry
         * the one already fused. Without the time-of-week check those repeats
         * would be counted as fresh evidence and the covariance would shrink
         * for information that was never there. */
        /* Assigned through an if rather than from the && chain directly: the
         * chain has essential type int, and storing that in a boolean (an
         * unsigned char here) is a narrowing across type categories that
         * MISRA 10.3 rejects. Same idiom as the rest of this file. */
        boolean sane = FALSE;

        if ((fusion_usable(altM, FUSION_ALT_MAX) != FALSE)
            && (fusion_usable(speedMps, FUSION_INPUT_MAX) != FALSE)
            && (fusion_usable(headingDeg, FUSION_INPUT_MAX) != FALSE)
            && (fusion_usable(hAccM, FUSION_ALT_MAX) != FALSE))
        {
            sane = TRUE;
        }
        else
        {
            /* at least one field is NaN, infinite or absurd */
        }

        if (sane == FALSE)
        {
            g_gnssLatch.droppedCount = g_gnssLatch.droppedCount + 1u;
        }
        else if ((s_haveITow == FALSE) || (iTOW != s_lastITow))
        {
            s_lastITow    = iTOW;
            s_haveITow    = TRUE;

            g_gnssLatch.lat        = latDeg1e7;
            g_gnssLatch.lon        = lonDeg1e7;
            g_gnssLatch.altM       = altM;
            g_gnssLatch.speedMps   = speedMps;
            g_gnssLatch.headingDeg = headingDeg;
            g_gnssLatch.hAccM      = hAccM;
            g_gnssLatch.iTOW       = iTOW;

            /* cppcheck-suppress misra-c2012-17.3 ; deviation: see
             * Fusion_setBaroAlt() above. */
            Ifx__dsync();
            g_gnssLatch.gen = g_gnssLatch.gen + 1u;
        }
        else
        {
            /* Same fix as last time. Counted rather than ignored silently: a
             * dupe count of exactly zero over a long run means iTOW is not
             * changing at all, and a count near the poll rate means the fix is
             * not being refreshed. Both are faults that leave the position
             * looking entirely plausible. */
            g_gnssLatch.dupesCount = g_gnssLatch.dupesCount + 1u;
        }
    }
    else
    {
        /* No usable solution. Deliberately NOT clearing s_haveITow: when the
         * fix comes back its iTOW will have moved on anyway, and forgetting it
         * would only risk re-fusing a stale fix. */
    }
}
