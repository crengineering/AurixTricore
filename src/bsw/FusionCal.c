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
 * measurement", which the outdoor run on 2026-08-26 showed is wrong: varN
 * claimed sigma 0.3 m while the loop closed at 2.59 m, because ten correlated
 * NAV-PVT solutions per second are counted as ten independent draws.
 *
 * 8.0 is the ratio between the claimed and the observed error, applied to the
 * variance so the reported uncertainty stops being a fiction. It is a stopgap
 * with an honest justification, not a derived constant: the correct fix is a
 * GNSS position-bias state, exactly as the barometer has. Until then this at
 * least makes varN safe to look at. */
#define FCAL_GNSS_POS_R_SCALE  (8.0f)

#define FCAL_GATE_SIGMA_SQ     (25.0f)
#define FCAL_GATE_MIN_M        (2.0f)

void FusionCal_init(void)
{
    uint8 i;

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

    for (i = 0u; i < 2u; i++)
    {
        g_fusionCal.reserved[i] = 0.0f;
    }

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
