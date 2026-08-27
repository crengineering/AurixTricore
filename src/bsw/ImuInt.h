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
/** Intervals per mean window (~1 s at ~1 kHz). Chosen so the running sum
 *  cannot overflow a uint32 even pathologically: at the nominal ~985 us
 *  (98 500 ticks) it is ~9.85e7 over a window, ~44x under 0xFFFFFFFF
 *  (~4.29e9); even 1000 back-to-back ~20 ms dropped-edge outliers
 *  (~2.04e6 ticks each, docs/IMU_INTERRUPT.md SS5.6) sum to ~2.04e9, still
 *  under the limit with margin. No uint64 anywhere in this module any more
 *  (user request, 2026-08-27) -- overflow is structurally impossible
 *  because the window resets long before it could approach one, not
 *  because the type was widened. See g_imuDrdyWindowSum's comment. */
#define IMUINT_MEAN_WINDOW_EDGES  (1000u)

/* --- edges since reset / most recent interval -------------------------- */
extern volatile uint32 g_imuDrdyCount;        /**< edges since reset                */
extern volatile uint32 g_imuDrdyLastTicks;    /**< STM0 tick of the last edge       */
extern volatile uint32 g_imuDrdyDtTicks;      /**< most recent interval, ticks      */

/* --- distribution ------------------------------------------------------ */
extern volatile uint32 g_imuDrdyDtMin;        /**< min interval since reset, ticks  */
extern volatile uint32 g_imuDrdyDtMax;        /**< max interval since reset, ticks  */
/** Sum of ticks over the most recently COMPLETED IMUINT_MEAN_WINDOW_EDGES-
 *  interval window (~1 s at ~1 kHz) -- was a running uint64 grand total
 *  since reset; replaced (2026-08-27) with a uint32 that resets every
 *  window instead, so overflow becomes structurally impossible rather than
 *  merely 43 s away. Together with g_imuDrdyWindowIntervals (always
 *  IMUINT_MEAN_WINDOW_EDGES once at least one window has closed, published
 *  alongside it for that reason) this gives an EXACT per-window mean,
 *  computed on the host -- never in the ISR, same rule as always. Updates
 *  once per window, not once per edge: between updates it holds the LAST
 *  completed window's sum, not a live-growing partial one, so a host read
 *  never sees a half-built window. A time series of these (poll roughly
 *  once a second) shows RC-oscillator drift better than one grand mean
 *  ever could. */
extern volatile uint32 g_imuDrdyWindowSum;
extern volatile uint32 g_imuDrdyWindowIntervals; /**< intervals in that window (see above) */
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
    uint32  windowSum;       /**< current (in-progress) window's running sum;
                               *   resets to 0 the moment it closes (below)  */
    uint32  windowCount;     /**< intervals folded into windowSum so far,
                               *   0 .. IMUINT_MEAN_WINDOW_EDGES-1           */
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
    uint32         index;         /**< valid only when kind == IMUINT_BIN_INDEX */
    boolean        justCentered;  /**< TRUE exactly once: the caller must zero
                                    *   the 32 histogram bins this one time     */
    boolean        windowComplete;/**< TRUE exactly on the interval that closes
                                    *   a mean window: windowSum/windowCount
                                    *   below are the values to publish       */
    uint32         windowSum;     /**< valid only when windowComplete==TRUE;
                                    *   captured BEFORE state resets for the
                                    *   next window, so the caller never has
                                    *   to race a read against that reset     */
    uint32         windowCount;   /**< valid only when windowComplete==TRUE;
                                    *   == IMUINT_MEAN_WINDOW_EDGES           */
} ImuInt_BinResult;

/** Pure update: discards the first IMUINT_WARMUP_EDGES intervals outright
 *  (state->{dtMin,dtMax,windowSum,windowCount,histBase} untouched -- see
 *  IMUINT_WARMUP_EDGES for why), then folds \p dt into
 *  state->{dtMin,dtMax,windowSum,windowCount}, centres state->histBase on
 *  the FIRST post-warm-up interval's own \p dt, and classifies \p dt
 *  against the (by then always-centred) window.
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

    result.kind           = IMUINT_BIN_NONE;
    result.index          = 0u;
    result.justCentered   = FALSE;
    result.windowComplete = FALSE;
    result.windowSum      = 0u;
    result.windowCount    = 0u;

    if (state->warmupRemaining > 0u)
    {
        /* discarded: dtMin/dtMax/windowSum/histBase untouched */
        state->warmupRemaining--;
    }
    else
    {
        state->dtCount++;
        if (dt < state->dtMin) { state->dtMin = dt; }
        if (dt > state->dtMax) { state->dtMax = dt; }

        /* Mean window (IMUINT_MEAN_WINDOW_EDGES, see its comment): a uint32
         * running sum that is read out and reset every window, instead of a
         * uint64 grand total that only ever grows -- overflow becomes
         * structurally impossible rather than merely 43 s away. */
        state->windowSum += dt;
        state->windowCount++;
        if (state->windowCount >= IMUINT_MEAN_WINDOW_EDGES)
        {
            /* Capture into the result FIRST -- the caller reads these from
             * `result`, never from `state`, so there is no window where the
             * completed values exist only in state right before being zeroed. */
            result.windowComplete = TRUE;
            result.windowSum      = state->windowSum;
            result.windowCount    = state->windowCount;
            state->windowSum      = 0u;
            state->windowCount    = 0u;
        }

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
    }

    return result;
}

#endif /* IMUINT_H */
