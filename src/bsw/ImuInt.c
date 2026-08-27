/**********************************************************************************************************************
 * \file ImuInt.c
 * \brief ICM-42688-P INT1 data-ready timestamp ISR -- see ImuInt.h.
 *
 * docs/IMU_INTERRUPT.md SS4 (ERU path) / SS5.2-5.3 (globals + ISR).
 *
 * SAFETY: P10.7 is a 5 V VEXT pad; the ICM-42688-P's INT1 absolute maximum is
 * VDDIO + 0.3 V = 3.6 V. BringUp_initImuIntPinSafe() (BringUp.c) already put
 * the pin into its permanent input/pulldown/TTL state, before anything else
 * on CPU0 touches a pin -- ImuInt_init() must run after that, and only adds
 * the ERU external-input mux selection on top of it. IfxScuEru_initReqPin()
 * itself only ever configures a pin as an INPUT (see IfxScuEru.h), so this
 * file never puts P10.7 into an output mode, in init or in the ISR: the ISR
 * touches only STM0 and this module's own globals, never a GPIO register.
 *********************************************************************************************************************/
#include "ImuInt.h"
#include "SysTime.h"
#include "ConfigurationIsr.h"
#include "IfxPort.h"
#include "IfxScuEru.h"
#include "IfxSrc.h"

#define IMU_ERU_IN    IfxScuEru_InputChannel_0     /* REQ0C = P10.7           */
#define IMU_ERU_OUT   IfxScuEru_OutputChannel_0    /* OGU0 -> SRC_SCUERU0     */

/* --- live state, non-static so tools/xcp_read.py finds them in the map --- */
/* cppcheck-suppress-begin misra-c2012-8.7 ; deviation: read over XCP
 * SHORT_UPLOAD by raw address (tools/xcp_read.py), never referenced by C
 * code outside this file -- same class of deviation as the Bmp388/Mpu6050
 * pool drivers (docs/ILLD_NOTES.md, PeriphDiag.md). g_imuDrdyStaleTicks is
 * the one exception (written from NavTask.c) but is included here too so
 * the whole block reads as one deliberate group. */
volatile uint32 g_imuDrdyCount;
volatile uint32 g_imuDrdyLastTicks;
volatile uint32 g_imuDrdyDtTicks;
volatile uint32 g_imuDrdyDtMin;
volatile uint32 g_imuDrdyDtMax;
volatile uint64 g_imuDrdyDtSum;
volatile uint32 g_imuDrdyHist[IMUINT_HIST_BINS];
volatile uint32 g_imuDrdyHistBase;
volatile uint32 g_imuDrdyUnder;
volatile uint32 g_imuDrdyOver;
volatile uint32 g_imuDrdyStaleTicks;   /* written by NavTask.c, not here */
/* cppcheck-suppress-end misra-c2012-8.7 */

/* Non-volatile shadow of the running accumulator (dtMin/dtMax/dtSum/
 * histBase/centred flag) -- the ISR is the only writer of both this and the
 * g_imuDrdy* globals above, so there is nothing to synchronise between them.
 * Keeping it out of the volatile globals is what lets ImuInt_accumulate()
 * stay a plain, host-testable function (test/test_imuint.c).
 * Named s_imuIntState rather than the obvious s_state -- MISRA 5.9 wants
 * every internal-linkage name unique project-wide, and s_state is already
 * taken (src/asw/CtrlReplay.c), same note as in Ahrs.c. */
static ImuInt_State s_imuIntState;

/* Declared before use (MISRA 8.4). Kept at external linkage because the
 * interrupt vector table references it, not C code -- which is also why
 * MISRA 8.7 is deviated below rather than fixed. */
void imuDrdyIsr(void);

/* cppcheck-suppress misra-c2012-17.3 ; deviation: IFX_INTERRUPT is a vendor
 * macro that emits the vector-table entry; the handler is prototyped
 * immediately above. */
IFX_INTERRUPT(imuDrdyIsr, 0, ISR_PRIORITY_IMU_DRDY);

/* cppcheck-suppress misra-c2012-8.7 ; deviation: referenced by the interrupt
 * vector table, not by C code; static would break the vector entry. */
/* Timestamp + bounded stats only -- no QSPI, no lwIP, no float, no unbounded
 * loop (the one 32-iteration histogram clear below runs exactly once, ever,
 * on the interval that centres the window -- see ImuInt_accumulate). The
 * actual IMU SPI burst stays in NavTask_step, in task context, as documented
 * in docs/IMU_INTERRUPT.md SS5.3. */
void imuDrdyIsr(void)
{
    const uint32 now = (uint32)SysTime_getTicks();
    const uint32 dt  = now - g_imuDrdyLastTicks;

    g_imuDrdyLastTicks = now;
    g_imuDrdyCount++;

    if (g_imuDrdyCount > 1u)   /* the first edge has no valid dt */
    {
        ImuInt_BinResult bin;

        g_imuDrdyDtTicks = dt;
        bin = ImuInt_accumulate(&s_imuIntState, dt);

        /* Only publish once warm-up (IMUINT_WARMUP_EDGES) is behind us --
         * before that, ImuInt_accumulate() left dtMin/dtMax/dtSum/histBase
         * untouched, and copying them out would publish stale sentinel/zero
         * values instead of leaving the previous (also stale, but at least
         * not misleadingly "final-looking") ones. g_imuDrdyDtTicks above is
         * still published every edge, warm-up included -- it is the one raw,
         * unfiltered number, on purpose. */
        if (s_imuIntState.warmupRemaining == 0u)
        {
            g_imuDrdyDtMin    = s_imuIntState.dtMin;
            g_imuDrdyDtMax    = s_imuIntState.dtMax;
            g_imuDrdyDtSum    = s_imuIntState.dtSum;
            g_imuDrdyHistBase = s_imuIntState.histBase;
        }

        if (bin.justCentered != FALSE)
        {
            uint8 i;
            for (i = 0u; i < IMUINT_HIST_BINS; i++)
            {
                g_imuDrdyHist[i] = 0u;
            }
        }

        switch (bin.kind)
        {
            case IMUINT_BIN_INDEX:
                g_imuDrdyHist[bin.index]++;
                break;
            case IMUINT_BIN_UNDER:
                g_imuDrdyUnder++;
                break;
            case IMUINT_BIN_OVER:
                g_imuDrdyOver++;
                break;
            case IMUINT_BIN_NONE:
            default:
                break;
        }
    }
}

void ImuInt_init(void)
{
    s_imuIntState.warmupRemaining = IMUINT_WARMUP_EDGES;
    s_imuIntState.dtCount         = 0u;
    s_imuIntState.dtMin           = 0xFFFFFFFFu;
    s_imuIntState.dtMax           = 0u;
    s_imuIntState.dtSum           = 0u;
    s_imuIntState.histBase        = 0u;
    s_imuIntState.histCentered    = FALSE;

    /* ERU wiring, docs/IMU_INTERRUPT.md SS4. IfxScuEru_initReqPin() only ever
     * sets a pin to INPUT (see IfxScuEru.h) -- additive to, never a
     * replacement of, BringUp_initImuIntPinSafe()'s pulldown/TTL state. The
     * pad-driver call is repeated here to match the documented recipe
     * exactly; it is idempotent with what BringUp.c already set. */
    IfxScuEru_initReqPin(&IfxScu_REQ0C_P10_7_IN, IfxPort_InputMode_pullDown);
    IfxPort_setPinPadDriver(&MODULE_P10, 7u, IfxPort_PadDriver_ttlSpeed1);

    IfxScuEru_disableFallingEdgeDetection(IMU_ERU_IN);
    IfxScuEru_enableRisingEdgeDetection(IMU_ERU_IN);      /* INT1 is active-high pulsed */
    /* Deliberately NOT IfxScuEru_enableAutoClear(), despite its name and
     * despite the vendor's own worked example (IfxScuEru.h:89-111) calling
     * it unconditionally in the same recipe this file otherwise follows.
     * The bit it writes is EICRn.LDENx = "Level Detection Enable"
     * (Ifx_SCU_EICR_Bits, Libraries/Infra/Sfr/TC39xB/IfxScu_regdef.h:368) --
     * the iLLD wrapper's name is a misnomer, not a description. With LDEN
     * enabled, INTFx tracks the pin LEVEL: set by the enabled edge (here,
     * rising) and cleared by the OPPOSITE edge (falling) or by software --
     * it does not self-clear after one edge the way the name suggests. The
     * OGU's pattern-detection stage below fires on every TRANSITION of
     * INTFx, so a genuine ~100 us pulse (push-pull, INT_TPULSE_DURATION=0,
     * docs/ICM42688P.md SS8) sets INTFx on the rising edge (the real
     * data-ready event) and clears it again ~100 us later on the falling
     * edge -- two transitions, two interrupts, for one data-ready pulse.
     * This is exactly the bimodal ~107 us / ~878 us distribution the first
     * hardware measurement found (SUM = the true ~985 us period,
     * docs/IMU_INTERRUPT.md SS5.6) before this fix. EIEN
     * (enableTriggerPulse, below) already generates a genuine one-shot
     * pulse per edge on its own -- LDEN is for a signal that is genuinely
     * HELD at a level (e.g. a button), not one that self-terminates.
     * See docs/ILLD_NOTES.md SS10 for the general finding. */
    IfxScuEru_enableTriggerPulse(IMU_ERU_IN);
    IfxScuEru_connectTrigger(IMU_ERU_IN, IfxScuEru_InputNodePointer_0);   /* -> OGU0 */

    IfxScuEru_setFlagPatternDetection(IMU_ERU_OUT, IMU_ERU_IN, TRUE);
    IfxScuEru_enablePatternDetectionTrigger(IMU_ERU_OUT);
    IfxScuEru_setInterruptGatingPattern(IMU_ERU_OUT,
                                        IfxScuEru_InterruptGatingPattern_alwaysActive);

    /* cppcheck-suppress-begin misra-c2012-19.2 ; deviation: SRC_SCUERU0 is a
     * vendor SFR macro backed by a union (Ifx_SRC_SRCR, IfxSrc_reg.h) -- this
     * fires on every iLLD register access of this shape project-wide
     * (docs/ILLD_NOTES.md), not something this file can fix. */
    IfxSrc_init(&SRC_SCUERU0, IfxSrc_Tos_cpu0, ISR_PRIORITY_IMU_DRDY);
    IfxSrc_enable(&SRC_SCUERU0);
    /* cppcheck-suppress-end misra-c2012-19.2 */
}
