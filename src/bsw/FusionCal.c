/**********************************************************************************************************************
 * \file FusionCal.c
 * \brief Live-tunable estimator parameters — see FusionCal.h.
 *********************************************************************************************************************/
#include "FusionCal.h"

/* Fixed address so XCP masters reach it without the map file. Off __at()
 * onto `#pragma section` (docs/MEMORY_PLACEMENT.md T4) -- an absolute group
 * at the unchanged LCF_XCP_FUSIONCAL_START literal. The __at()-era
 * misra-c2012-8.2 false-positive deviation (cppcheck misreading "__at(ADDR)"
 * as an old-style function declarator) no longer applies -- #pragma section
 * is not parsed as a declarator at all. Guarded by `#if defined(__TASKING__)`
 * -- see SharedRam.c for why (GCC -Werror; this file is compiled for the
 * host as part of the `estimator` test library, and this is exactly where
 * that CI job actually caught it). */
#if defined(__TASKING__)
#pragma section farbss "xcp_fusioncal"
#endif
volatile Xcp_FusionCal g_fusionCal;
#if defined(__TASKING__)
#pragma section farbss restore
#endif

/* Compiled defaults. These are the values documented in docs/FUSION.md section
 * 2, every one of them derived from a measurement on this board rather than
 * picked. Changing a number here is a code change with a commit message; the
 * XCP block is for the experiment that decides what that number should be. */
#define FCAL_TWO_KP_ACC        (1.0f)
#define FCAL_TWO_KP_MAG        (0.5f)
#define FCAL_TWO_KI            (0.02f)

/* PSD, not a per-tick sigma -- see fusion.c FUSION_SIGMA_A_D/A_H for the
 * derivation (0.3 * sqrt(0.02), the 50 Hz-equivalent conversion) and
 * docs/NAV_TUNING.md for why the old per-tick parametrisation was rate-
 * dependent. Units m/s^2/sqrt(Hz). */
#define FCAL_SIGMA_ACC_D       (0.0424f)
#define FCAL_SIGMA_BARO        (0.0197f)
#define FCAL_SIGMA_BARO_RW     (0.025f)
#define FCAL_TAU_BARO_BIAS     (600.0f)

#define FCAL_SIGMA_ACC_H       (0.0707f)
#define FCAL_SIGMA_GNSS_VEL    (0.3f)

/* Accelerometer bias random walk, shared by all three channels -- see
 * fusion.c FUSION_SIGMA_ACC_RW. Already rate-invariant; wired to the cal
 * block only so it can be swept live (docs/NAV_TUNING.md section 4.3). */
#define FCAL_SIGMA_ACC_RW      (1.0e-4f)

/* GNSS position R multiplier. 1.0 means "trust hAcc as an independent
 * measurement" -- the outdoor run on 2026-08-26 found that over-confident
 * (varN claimed sigma 0.3 m while the loop closed at 2.59 m, because ten
 * correlated NAV-PVT solutions per second were counted as ten independent
 * draws), which is why this stood at 8.0.
 *
 * 1.0 (SWE1-FW-012, task 3/9, docs/NAV_STRAND_2026-09.md section 3.2): the
 * offline sweep over {8, 4, 2, 1} x sigmaGnssVel found no setting puts NIS in
 * 0.5-2.0 together with a fused-vs-raw ratio near 1x -- that is coloured
 * receiver noise, not mistuning, so NIS is a reported diagnostic here, not a
 * gate. The product bar is the 2-D scatter ratio against the raw fix, and 1.0
 * is the value that minimises it: t1 1.65x -> 1.27x (TASK-3 SWEEP FIGURE, LOCK
 * DISABLED BY CAL OVERRIDE -- SWE1-FW-014's stationary lock did not exist
 * when this was measured; at HEAD, with the lock active, t1 is a rest
 * recording and fuses ZERO GNSS fixes, so this number is not reproducible
 * there any more. Re-measured 2026-09-14, flight-reviewer fix pass: t2
 * 0.992x, t4 0.960x, t5 0.977x with the lock live, `tools/nav_replay.py`;
 * see the evidence index for hashes), every other recording <= 1.02x. varN
 * is correspondingly smaller and reads as more confident -- that is
 * relative to the (still metre-class) raw fix, not an absolute accuracy
 * claim; see docs/FUSION.md section 2. */
#define FCAL_GNSS_POS_R_SCALE  (1.0f)

#define FCAL_GATE_SIGMA_SQ     (25.0f)
#define FCAL_GATE_MIN_M        (2.0f)

/* SWE1-FW-010: max rate the GNSS-altitude update may move d [m/s]. 0.06 m/min
 * -- a third of the barometer's own measured 0.19 m/min wander -- while still
 * tracking the barometer bias's steady-state sigma (0.433 m) within one mean-
 * reversion time constant. ZERO IS A DEFINED VALUE ("update off"); see
 * FusionCal.h and fusion.c fusion_correctGnss(). */
#define FCAL_GNSS_ALT_SLEW_MPS (0.001f)

/* SWE1-FW-011: GNSS is trusted only at or below this reported hAcc [m].
 * Separates every recorded outdoor fix (<= 3.347 m) from every recorded
 * indoor one (>= 4.612 m) -- docs/NAV_STRAND_2026-09.md section 3.3. */
#define FCAL_GNSS_HACC_MAX     (4.0f)

/* SWE1-FW-014: stationary-lock thresholds, derived from the 1-second-window
 * maxima of the two detector inputs across every recording on file (clean
 * rest tops out at 1.242 deg/s / 0.0169 g; the quietest motion on file, a
 * 0.45 m hand lift, is 7x above lockAccG). Release is 1.5x the lock
 * threshold -- that ratio is the hysteresis. */
#define FCAL_LOCK_GYRO_DPS     (2.0f)
#define FCAL_LOCK_ACC_G        (0.03f)
#define FCAL_REL_GYRO_DPS      (3.0f)
#define FCAL_REL_ACC_G         (0.05f)
#define FCAL_LOCK_WINDOW_S     (1.0f)

/* SWE1-FW-015: the release mechanism. tauGnssBiasS = 60 s is the measured
 * 1/e autocorrelation of the raw GNSS horizontal error (60.3 s north /
 * 56.7 s east indoors, 32.4 s outdoor t1 -- docs/NAV_TUNING.md section 3). */
#define FCAL_SIGMA_ZUPT           (0.01f)
#define FCAL_TAU_GNSS_BIAS_S      (60.0f)
#define FCAL_GNSS_BIAS_RATE_MAX   (0.05f)

void FusionCal_init(void)
{
    g_fusionCal.twoKpAcc      = FCAL_TWO_KP_ACC;
    g_fusionCal.twoKpMag      = FCAL_TWO_KP_MAG;
    g_fusionCal.twoKi         = FCAL_TWO_KI;

    g_fusionCal.sigmaAccD     = FCAL_SIGMA_ACC_D;
    g_fusionCal.sigmaBaro     = FCAL_SIGMA_BARO;
    g_fusionCal.sigmaBaroRw   = FCAL_SIGMA_BARO_RW;
    g_fusionCal.tauBaroBias   = FCAL_TAU_BARO_BIAS;

    g_fusionCal.sigmaAccH     = FCAL_SIGMA_ACC_H;
    g_fusionCal.sigmaGnssVel  = FCAL_SIGMA_GNSS_VEL;
    g_fusionCal.gnssPosRScale = FCAL_GNSS_POS_R_SCALE;

    g_fusionCal.gateSigmaSq   = FCAL_GATE_SIGMA_SQ;
    g_fusionCal.gateMinM      = FCAL_GATE_MIN_M;

    g_fusionCal.sigmaAccRw    = FCAL_SIGMA_ACC_RW;

    g_fusionCal.gnssAltSlewMps = FCAL_GNSS_ALT_SLEW_MPS;
    g_fusionCal.gnssHAccMax    = FCAL_GNSS_HACC_MAX;

    g_fusionCal.lockGyroDps    = FCAL_LOCK_GYRO_DPS;
    g_fusionCal.lockAccG       = FCAL_LOCK_ACC_G;
    g_fusionCal.relGyroDps     = FCAL_REL_GYRO_DPS;
    g_fusionCal.relAccG        = FCAL_REL_ACC_G;
    g_fusionCal.lockWindowS    = FCAL_LOCK_WINDOW_S;

    g_fusionCal.sigmaZupt        = FCAL_SIGMA_ZUPT;
    g_fusionCal.tauGnssBiasS     = FCAL_TAU_GNSS_BIAS_S;
    g_fusionCal.gnssBiasRateMax  = FCAL_GNSS_BIAS_RATE_MAX;

    /* Magic last: a master polling for it sees a fully populated block or none
     * of it, never a half-written one. */
    g_fusionCal.magic = XCP_FUSIONCAL_MAGIC;
}

float32 FusionCal_positive(float32 v, float32 lo, float32 def)
{
    float32 r = def;

    /* Written as a POSITIVE test -- "is it inside the admissible band" --
     * rather than as a list of rejections. Both comparisons are FALSE for NaN,
     * so NaN takes the default; and the upper bound is what stops +INFINITY,
     * which an earlier version accepted because inf > lo is true. Bounding
     * both sides is the difference between rejecting the bad values you
     * thought of and accepting only the good ones. */
    if ((v > lo) && (v < FUSIONCAL_MAX))
    {
        r = v;
    }
    else
    {
        /* zero, negative, NaN, or either infinity — keep the compiled default */
    }

    return r;
}
