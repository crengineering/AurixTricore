/**********************************************************************************************************************
 * \file ImuInt.h
 * \brief ICM-42688-P INT1 data-ready timestamp ISR -- measurement only.
 *
 * docs/IMU_INTERRUPT.md is the design source: SS4 (ERU path), SS5.2/5.3
 * (these globals + the ISR), SS5.6 (decision tree this measurement settles).
 * This module owns P10.7's ERU wiring and nothing else about the pin --
 * BringUp_initImuIntPinSafe() (BringUp.c) already put P10.7 into its
 * permanent input/pulldown/TTL state and MUST run first; ImuInt_init() only
 * adds the ERU external-input mux selection on top of that, via
 * IfxScuEru_initReqPin(), which itself only ever sets a pin to INPUT. P10.7
 * is never configured as an output by this file, at init or in the ISR.
 *
 * The ISR times edges and bins the interval; it does not read the IMU over
 * SPI and does not decide anything about the flight chain. NavTask_step
 * keeps sampling on its own schedule -- see docs/IMU_INTERRUPT.md SS5.4 for
 * why this stays a measurement, not a clock, for now.
 *
 * All of the state below is a plain, non-static global on purpose: it is
 * read by tools/xcp_read.py (SHORT_UPLOAD works on any global in the map --
 * see that tool's header) with zero cost to Xcp_Data, the A2L or the GUI
 * (docs/IMU_INTERRUPT.md SS5.1). Do not fold these into a struct -- a struct
 * member does not get its own symbol in the map file.
 *********************************************************************************************************************/
#ifndef IMUINT_H
#define IMUINT_H

#include "Ifx_Types.h"

/** 1 us-wide bins (STM0 @ 100 MHz = 100 ticks/us); 32 bins = a 32 us window
 *  around the first post-warm-up sample -- docs/IMU_INTERRUPT.md SS5.2/SS5.6. */
#define IMUINT_HIST_BINS          (32u)
#define IMUINT_HIST_BIN_TICKS     (100u)
/** Intervals discarded before anything (min/max/sum/histogram) starts
 *  accumulating. First hardware run (docs/IMU_INTERRUPT.md SS5.6) found
 *  identical, boot-only outliers -- a short one (~741 us) and a long one
 *  (~20.4 ms) -- traced to the pin-arm/register-config boot sequence (ERU
 *  armed in core0_main() well before Icm42688_init() finally enables DRDY
 *  routing to INT1, itself after several other sensors' I2C init; see
 *  Cpu0_Main.c call order and docs/ICM42688P.md SS8's own "RESET_DONE_INT1_EN
 *  is on out of reset" trap). 100 intervals is ~100 ms at 1 kHz, generously
 *  past any boot transient, and cheap to discard. */
#define IMUINT_WARMUP_EDGES       (100u)
/** Bins below the anchor sample the centred base is set to. */
#define IMUINT_HIST_MARGIN_BINS   (16u)

/* --- edges since reset / most recent interval -------------------------- */
extern volatile uint32 g_imuDrdyCount;        /**< edges since reset                */
extern volatile uint32 g_imuDrdyLastTicks;    /**< STM0 tick of the last edge       */
extern volatile uint32 g_imuDrdyDtTicks;      /**< most recent interval, ticks      */

/* --- distribution ------------------------------------------------------ */
extern volatile uint32 g_imuDrdyDtMin;        /**< min interval since reset, ticks  */
extern volatile uint32 g_imuDrdyDtMax;        /**< max interval since reset, ticks  */
extern volatile uint64 g_imuDrdyDtSum;        /**< sum of intervals; mean computed
                                                *   on the host, never in the ISR   */
extern volatile uint32 g_imuDrdyHist[IMUINT_HIST_BINS];  /**< 1 us bins             */
extern volatile uint32 g_imuDrdyHistBase;     /**< ticks; bin i covers
                                                *   [base+100i, base+100(i+1)),
                                                *   0 = not yet centred             */
extern volatile uint32 g_imuDrdyUnder;        /**< dt below the histogram window    */
extern volatile uint32 g_imuDrdyOver;         /**< dt above the histogram window    */

/* --- I5: DRDY -> read staleness, written by NavTask.c, not by the ISR --- */
extern volatile uint32 g_imuDrdyStaleTicks;   /**< edge -> Task read latency, ticks */

/** ERU + interrupt setup for P10.7 (docs/IMU_INTERRUPT.md SS4), plus zeroing
 *  the accumulator state. Call once from core0_main, AFTER
 *  BringUp_initImuIntPinSafe() has put P10.7 into its permanent safe input
 *  state -- this function only adds the ERU mux selection on top of that
 *  (IfxScuEru_initReqPin sets a pin to input, never output). */
void ImuInt_init(void);

/** Non-volatile mirror of the running accumulator, used only so the min/max/
 *  sum/histogram-centring arithmetic can be exercised on the host --
 *  test/test_imuint.c -- with no iLLD include and no ISR. The real, live
 *  state lives in the volatile globals above; ImuInt.c copies to/from one of
 *  these once per interrupt, which is why the ISR touches at most a handful
 *  of scalars plus one histogram bin, never the whole array. */
typedef struct
{
    uint32  warmupRemaining; /**< counts down from IMUINT_WARMUP_EDGES; while
                               *   nonzero, every interval is discarded      */
    uint32  dtCount;         /**< intervals accumulated POST-warm-up (was
                               *   "edges - 1"; now "edges - 1 - warm-up")   */
    uint32  dtMin;
    uint32  dtMax;
    uint64  dtSum;
    uint32  histBase;
    boolean histCentered;
} ImuInt_State;

/** What ImuInt_accumulate() found for one interval, once state is updated. */
typedef enum
{
    IMUINT_BIN_NONE,     /**< still discarding warm-up intervals -- no bin touched */
    IMUINT_BIN_UNDER,    /**< dt fell below the centred window              */
    IMUINT_BIN_INDEX,    /**< dt fell in \p index                            */
    IMUINT_BIN_OVER      /**< dt fell above the centred window              */
} ImuInt_BinKind;

typedef struct
{
    ImuInt_BinKind kind;
    uint32         index;        /**< valid only when kind == IMUINT_BIN_INDEX */
    boolean        justCentered; /**< TRUE exactly once: the caller must zero
                                   *   the 32 histogram bins this one time     */
} ImuInt_BinResult;

/** Pure update: discards the first IMUINT_WARMUP_EDGES intervals outright
 *  (state->{dtMin,dtMax,dtSum,histBase} untouched -- see IMUINT_WARMUP_EDGES
 *  for why), then folds \p dt into state->{dtMin,dtMax,dtSum}, centres
 *  state->histBase on the FIRST post-warm-up interval's own \p dt, and
 *  classifies \p dt against the (by then always-centred) window.
 *
 * Anchoring on a single post-warm-up sample, not a running dtMin, is a
 * deliberate choice over "centre on the median/mode of the steady
 * population": a true median/mode needs either a sort (unbounded cost) or a
 * histogram that is itself already centred to bin into (the chicken-and-egg
 * this design avoids). A single sample, taken only after the boot transient
 * IMUINT_WARMUP_EDGES is sized to clear, is O(1) and bounded, and is a good
 * enough anchor for a distribution this tight (single-population, low-jitter
 * once past boot -- docs/IMU_INTERRUPT.md SS5.6) that IMUINT_HIST_MARGIN_BINS
 * of headroom below it comfortably covers the spread. A running dtMin was the
 * previous (wrong) choice: it let exactly one boot-transient outlier corrupt
 * the anchor for the whole run, which is the bug this replaces.
 *
 * `static inline`, defined here rather than in ImuInt.c, and on purpose: this
 * header includes nothing but Ifx_Types.h, so a host test can reach the real
 * arithmetic (test/test_imuint.c) without pulling in IfxScuEru.h/IfxSrc.h/
 * IfxPort.h -- exactly the "no iLLD include, a fake is not even needed" this
 * function is meant to satisfy (docs/IMU_INTERRUPT.md SS7 footer). The ISR in
 * ImuInt.c calls this same definition; there is only one copy of the logic. */
static inline ImuInt_BinResult ImuInt_accumulate(ImuInt_State *state, uint32 dt)
{
    ImuInt_BinResult result;

    result.kind         = IMUINT_BIN_NONE;
    result.index        = 0u;
    result.justCentered = FALSE;

    if (state->warmupRemaining > 0u)
    {
        state->warmupRemaining--;
        return result;   /* discarded: dtMin/dtMax/dtSum/histBase untouched */
    }

    state->dtCount++;
    if (dt < state->dtMin) { state->dtMin = dt; }
    if (dt > state->dtMax) { state->dtMax = dt; }
    state->dtSum += (uint64)dt;

    if (state->histCentered == FALSE)
    {
        /* This is necessarily the first post-warm-up interval (dtCount just
         * became 1): centre once, here, on dt itself -- see the function
         * comment for why a single sample, not a running dtMin. */
        const uint32 margin = IMUINT_HIST_MARGIN_BINS * IMUINT_HIST_BIN_TICKS;

        state->histBase     = (dt > margin) ? (dt - margin) : 0u;
        state->histCentered = TRUE;
        result.justCentered = TRUE;
    }

    if (state->histCentered != FALSE)
    {
        if (dt < state->histBase)
        {
            result.kind = IMUINT_BIN_UNDER;
        }
        else
        {
            const uint32 binWidth = IMUINT_HIST_BINS * IMUINT_HIST_BIN_TICKS;
            const uint32 offset   = dt - state->histBase;

            if (offset >= binWidth)
            {
                result.kind = IMUINT_BIN_OVER;
            }
            else
            {
                result.kind  = IMUINT_BIN_INDEX;
                result.index = offset / IMUINT_HIST_BIN_TICKS;
            }
        }
    }

    return result;
}

#endif /* IMUINT_H */
