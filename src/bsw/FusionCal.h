/**********************************************************************************************************************
 * \file FusionCal.h
 * \brief Live-tunable parameters for the attitude and navigation filters.
 *
 * Every constant the estimator is tuned by, in one XCP-writable block, so a
 * tuning pass does not need a rebuild and a reflash.
 *
 * WHY THIS EXISTS. The filters are tuned against measured sensor behaviour, and
 * the measurements that matter most cannot be taken on a bench: `sigmaAccD` was
 * set from a board sitting on a desk, and motors turning will change it by more
 * than any other single factor. Re-flashing between every attempt makes that a
 * bad afternoon; a calibration block makes it a slider. The same already
 * applies to the diagnostics thresholds in `Xcp_Cal` — this is that pattern,
 * applied to the estimator.
 *
 * ⚠️ **RAM ONLY.** Like `Xcp_Cal`, and deliberately: a tuning experiment must
 * never be able to leave the vehicle in a state it cannot be reset out of.
 * Power-cycle and you are back to the compiled defaults in `FusionCal_init()`,
 * which are the values documented in `docs/FUSION.md` §2. When a value has
 * earned its place, change the default in code — do not rely on the block.
 *
 * ⚠️ **Read every tick, not latched.** Writes take effect on the next filter
 * update, which is the point. It also means a nonsense value takes effect just
 * as fast, so the consumers clamp what would break the arithmetic (a variance
 * must stay positive) rather than trusting the master.
 *
 * A separate block rather than more of `Xcp_Cal` because ownership follows the
 * code: `Diagnostics.c` owns its thresholds, the estimator owns these. Mixing
 * them would mean two modules writing defaults into one struct.
 *********************************************************************************************************************/
#ifndef FUSIONCAL_H
#define FUSIONCAL_H

#include "Ifx_Types.h"

/* Next free 256-byte slot after Xcp_Fusion (0x70030500). */
/* XCP_FUSIONCAL_ADDR deleted, T5 (docs/MEMORY_PLACEMENT.md): Xcp.c's
 * whitelist now uses (uint32)&g_fusionCal, so Lcf_Tasking_Tricore_Tc.lsl
 * (LCF_XCP_FUSIONCAL_START) is the only place 0x70030600 is written down. */
#define XCP_FUSIONCAL_MAGIC  0x4C414346u        /* "FCAL" */
/* 64 -> 128, SWE1-FW-014 (docs/CODEMAP.md's "next free slot" rule does not
 * apply here: the block would still fit its existing 256-byte slot at
 * double this size, so appending inside it is cheaper). */
#define XCP_FUSIONCAL_SIZE   128u

/* Layout (little-endian, all 4-byte aligned; verify against
 * FusionCal.src after any edit — TASKING aligns uint32 to 2 bytes):
 *
 *   0x00  uint32  magic         0x4C414346, firmware-written, not writable
 *   --- attitude (Ahrs.c) ---
 *   0x04  float32 twoKpAcc      accelerometer correction gain [1/s]
 *   0x08  float32 twoKpMag      magnetometer correction gain [1/s]
 *   0x0C  float32 twoKi         gyro-bias integral gain [1/s^2]
 *   --- vertical channel (fusion.c) ---
 *   0x10  float32 sigmaAccD     accel process noise PSD, down [m/s^2/sqrt(Hz)]
 *                               ⚠️ was a per-tick sigma [m/s^2] before the
 *                               PSD reparametrisation (docs/NAV_TUNING.md);
 *                               A2L characteristic renamed so a stale saved
 *                               tuning file cannot write the old 32x value
 *   0x14  float32 sigmaBaro     barometer noise [m] (R is this SQUARED)
 *   0x18  float32 sigmaBaroRw   barometer bias random walk [m/sqrt(s)]
 *   0x1C  float32 tauBaroBias   barometer bias mean-reversion [s]
 *   --- horizontal channels (fusion.c) ---
 *   0x20  float32 sigmaAccH     accel process noise PSD, horizontal
 *                               [m/s^2/sqrt(Hz)] -- same ⚠️ as sigmaAccD
 *   0x24  float32 sigmaGnssVel  GNSS velocity noise [m/s]
 *   0x28  float32 gnssPosRScale multiplies the GNSS position R. >1 tells the
 *                               filter its position fixes are less independent
 *                               than hAcc implies — see docs/FUSION.md section 7
 *   --- gates (both) ---
 *   0x2C  float32 gateSigmaSq   outlier gate, in sigma squared
 *   0x30  float32 gateMinM      absolute floor on the barometer gate [m]
 *   --- accelerometer bias (fusion.c), taken from reserved[0] -- no offset
 *       change, no field movement ---
 *   0x34  float32 sigmaAccRw    accel bias random walk [m/s^2/sqrt(s)],
 *                               shared by all three channels. Already
 *                               rate-invariant, unlike sigmaAccD/sigmaAccH
 *                               above, so this is only a live-tuning wire-up
 *   --- SWE1-FW-010/-011 (taken from reserved[2], no offset change) ---
 *   0x38  float32 gnssAltSlewMps  max rate [m/s] the GNSS-altitude update
 *                               may move d (fusion.c fusion_correctGnss()).
 *                               0.001 default. ZERO IS A DEFINED VALUE:
 *                               "the GNSS-altitude update is off". Read
 *                               DIRECTLY, never through FusionCal_positive(),
 *                               which would substitute the default for 0
 *   0x3C  float32 gnssHAccMax    GNSS is trusted only while the reported
 *                               hAcc is at or below this [m]. 4.0 default,
 *                               separates every recorded outdoor fix from
 *                               every recorded indoor one (docs/NAV_STRAND_
 *                               2026-09.md section 3.3)
 *
 * Block was FULL at 64 bytes after SWE1-FW-010/-011 -- SWE1-FW-014 grows
 * XCP_FUSIONCAL_SIZE to 128 and appends its eight fields from 0x40, still
 * inside this block's own 256-byte slot (docs/CODEMAP.md's "next free slot"
 * rule applies when a block would overrun its slot; this one does not, so
 * appending here is cheaper). No existing offset moves.
 *   --- SWE1-FW-014 (the lock) ---
 *   0x40  float32 lockGyroDps   engage below this |omega| [deg/s], held for
 *                               lockWindowS. 2.0 default
 *   0x44  float32 lockAccG      engage below this abs(|a|-1g) [g]. 0.03 default
 *   0x48  float32 relGyroDps    release above this |omega|, on 2 consecutive
 *                               samples. 3.0 default
 *   0x4C  float32 relAccG       release above this abs(|a|-1g), same rule.
 *                               0.05 default
 *   0x50  float32 lockWindowS   how long BOTH lock thresholds must hold
 *                               before engaging [s]. 1.0 default
 *   --- SWE1-FW-015 (the release) ---
 *   0x54  float32 sigmaZupt     zero-velocity measurement noise [m/s].
 *                               0.01 default
 *   0x58  float32 tauGnssBiasS  GNSS position-bias decay time constant [s],
 *                               the measured 1/e autocorrelation of the raw
 *                               horizontal GNSS error. 60.0 default
 *   0x5C  float32 gnssBiasRateMax  max rate the GNSS position bias may decay
 *                               at [m/s]. 0.05 default. gnssBiasMaxM (the
 *                               clamp) stays a compiled #define -- see
 *                               fusion.c
 *
 * The struct itself ends at 0x60 (96 bytes) -- XCP_FUSIONCAL_SIZE is 128,
 * so 32 bytes (eight more float32 fields) of headroom remain inside this
 * block's own 256-byte slot before docs/CODEMAP.md's "take the next free
 * slot" rule would apply to a ninth field. Not literally full, despite the
 * design note's working assumption while the field count was still being
 * decided (docs/NAV_STRAND_2026-09.md section 10.5) -- corrected here,
 * 2026-09-14, against the compiled struct rather than carried forward.
 */
typedef struct
{
    uint32  magic;

    float32 twoKpAcc;
    float32 twoKpMag;
    float32 twoKi;

    float32 sigmaAccD;
    float32 sigmaBaro;
    float32 sigmaBaroRw;
    float32 tauBaroBias;

    float32 sigmaAccH;
    float32 sigmaGnssVel;
    float32 gnssPosRScale;

    float32 gateSigmaSq;
    float32 gateMinM;

    float32 sigmaAccRw;
    float32 gnssAltSlewMps;
    float32 gnssHAccMax;

    float32 lockGyroDps;
    float32 lockAccG;
    float32 relGyroDps;
    float32 relAccG;
    float32 lockWindowS;

    float32 sigmaZupt;
    float32 tauGnssBiasS;
    float32 gnssBiasRateMax;
} Xcp_FusionCal;

/* Compile-time invariant Xcp.c's write-whitelist depends on (flight-reviewer
 * MAJOR, 2026-09-14): the claimed slot must be at least as large as the
 * struct, so XCP_FUSIONCAL_SIZE can never silently fall below sizeof(Xcp_
 * FusionCal) -- the whitelist itself bounds on sizeof(g_fusionCal) directly
 * (Xcp.c), not on this macro, precisely so a future field can never write
 * past the real struct; this assert is the OTHER direction, that the macro
 * (used for nothing safety-critical any more, only as slot-claim
 * documentation) has not drifted below what it claims to describe. Not
 * generated: gen_a2l.py's LayoutAssert_gen.h asserts field offsets and the
 * struct's own sizeof, never a relationship to this slot macro. TASKING
 * does not accept _Static_assert under this project's build flags
 * (tools/gen_a2l.py's own comment) -- same negative-array-size idiom. */
/* cppcheck-suppress misra-c2012-2.3 ; deviation: the type itself is the
 * check -- a compile-time assertion via the negative-array-size idiom
 * (TASKING rejects _Static_assert, see the comment above) has no further
 * use for the typedef name by construction, the same as every one of
 * gen_a2l.py's ~185 generated LayoutAssert_gen.h assertions, which this
 * hand-written one extends to a relationship those generated ones do not
 * cover (a slot macro vs. the struct it is meant to contain). */
typedef char fusioncal_slot_covers_struct[
    (XCP_FUSIONCAL_SIZE >= sizeof(Xcp_FusionCal)) ? 1 : -1];

extern volatile Xcp_FusionCal g_fusionCal;

/** Load the compiled defaults. Call once at start-up, before Fusion_init(). */
void FusionCal_init(void);

/** Upper bound on any accepted tuning value.
 *
 *  Exists because "reject the bad values" is not the same as "accept only the
 *  good ones". An earlier version tested only the lower side and +INFINITY
 *  sailed straight through, because inf > lo is perfectly true — a master can
 *  write that from the GUI, and it reached the estimator and turned a
 *  covariance into NaN.
 *
 *  The bound only has to exclude infinity and the absurd -- it is not a
 *  plausibility check on the tuning, which is the operator's business. Every
 *  real value here is under 1e3, so 1e9 rejects what it must without
 *  second-guessing a deliberate experiment. */
#define FUSIONCAL_MAX   (1.0e9f)

/** \return \p v when it is a usable tuning value, else \p def.
 *
 *  Usable means strictly above \p lo AND below FUSIONCAL_MAX, which rejects
 *  zero, negatives, NaN and both infinities. Called on every read, because
 *  these values arrive from an XCP master and can be anything at all. */
float32 FusionCal_positive(float32 v, float32 lo, float32 def);

#endif /* FUSIONCAL_H */
