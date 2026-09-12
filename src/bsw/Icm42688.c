/**********************************************************************************************************************
 * \file Icm42688.c
 * \brief TDK-InvenSense ICM-42688-P 6-axis IMU driver (QSPI0) — see Icm42688.h.
 *
 * ⚠️ Every register constant here is written from the ICM-42688-P register map
 * WITHOUT the device datasheet (only AN-000488, the EVB user guide, is
 * available and it contains no register information — docs/ICM42688P.md 4).
 * Icm42688_debugDump() is the instrument for confirming them against silicon;
 * WHO_AM_I is the gate that makes a wrong guess visible immediately.
 *
 * SPI framing: the first byte is the register address, and its MSB is the
 * direction — 1 for a read, 0 for a write. A burst read therefore costs
 * 1 + N bytes, and the first received byte is the turnaround and is discarded.
 *
 * Unlike the MPU-6050 this part outputs big-endian (high byte first) signed
 * 16-bit counts, and there is no compensation polynomial: the counts scale by
 * a fixed factor set by the configured full-scale range.
 *********************************************************************************************************************/
#include "Icm42688.h"
#include "Spi.h"
#include "IfxStm.h"
#include "SysTime.h"

/* --- Register map (bank 0) --- */
#define ICM42688_REG_DEVICE_CONFIG   (0x11u)
#define ICM42688_REG_INT_CONFIG      (0x14u)
#define ICM42688_REG_TEMP_DATA1      (0x1Du)   /* 0x1D..0x2A: temp(2) accel(6) gyro(6) */
#define ICM42688_REG_INT_STATUS      (0x2Du)
#define ICM42688_REG_PWR_MGMT0       (0x4Eu)
#define ICM42688_REG_GYRO_CONFIG0    (0x4Fu)
#define ICM42688_REG_ACCEL_CONFIG0   (0x50u)
#define ICM42688_REG_INT_CONFIG0     (0x63u)
#define ICM42688_REG_INT_CONFIG1     (0x64u)
#define ICM42688_REG_INT_SOURCE0     (0x65u)
#define ICM42688_REG_WHO_AM_I        (0x75u)

#define ICM42688_BURST_LEN           (14u)     /* temp(2) + accel(6) + gyro(6) */
#define ICM42688_SPI_READ            (0x80u)   /* MSB of the address byte */
#define ICM42688_ADDR_MASK           (0x7Fu)

/* DEVICE_CONFIG (0x11) bit 0: SOFT_RESET_CONFIG. */
#define ICM42688_SOFT_RESET          (0x01u)

/* PWR_MGMT0 (0x4E): GYRO_MODE (bits 3:2) = 0b11 low noise,
 *                   ACCEL_MODE (bits 1:0) = 0b11 low noise.
 * This is the write that wakes the part. Leave it at its reset value and every
 * sample reads zero while the bus looks perfectly healthy — the same class of
 * failure that hid on the MPU-6050 for days. */
#define ICM42688_PWR_MGMT0_VAL       (0x0Fu)

/* GYRO_CONFIG0 (0x4F): GYRO_FS_SEL (bits 7:5) = 0 -> +/-2000 dps,
 *                      GYRO_ODR (bits 3:0) = 0x06 -> 1 kHz. */
#define ICM42688_GYRO_CONFIG0_VAL    (0x06u)
#define ICM42688_GYRO_FULLSCALE_DPS  (2000.0f)

/* ACCEL_CONFIG0 (0x50): ACCEL_FS_SEL (bits 7:5) = 0 -> +/-16 g,
 *                       ACCEL_ODR (bits 3:0) = 0x06 -> 1 kHz.
 * The ODR is deliberately faster than the 50 Hz task: the data registers
 * always hold the newest completed sample, so a fast ODR costs nothing and
 * leaves headroom to raise the task rate later. */
#define ICM42688_ACCEL_CONFIG0_VAL   (0x06u)
#define ICM42688_ACCEL_FULLSCALE_G   (16.0f)

/* INT1 data-ready registers (docs/ICM42688P.md 8.2, verified against the
 * datasheet, DS-000347 rev 1.2). These are written as plain values, never as
 * a read-modify-write: INT_CONFIG1 resets to 0x10 and INT_SOURCE0 resets to
 * 0x10, not 0x00 (8.3), so preserving "defaults" would silently keep
 * INT_ASYNC_RESET at 1 -- the documented #1 cause of an interrupt that never
 * fires, looking exactly like a bad solder joint. Push-pull (not open-drain)
 * is a safety choice: open-drain needs a pull-up, and the only rail at the
 * P10.7 pad is 5 V, which would exceed INT1's VDDIO+0.3 V absolute maximum
 * (docs/IMU_INTERRUPT.md). SPI-side only -- no GPIO/ERU wiring here; that is
 * I4, out of scope for this change. */
#define ICM42688_INT_CONFIG_VAL      (0x03u)   /* INT1: active high, push-pull, pulsed */
#define ICM42688_INT_CONFIG0_VAL     (0x00u)   /* latched-mode clear opts; no-op here */
#define ICM42688_INT_CONFIG1_VAL     (0x00u)   /* INT_ASYNC_RESET -> 0 (was 1); 100us pulse */
#define ICM42688_INT_SOURCE0_VAL     (0x08u)   /* UI_DRDY_INT1_EN only */

#define ICM42688_COUNTS_FULLSCALE    (32768.0f)
#define ICM42688_GYRO_SCALE   (ICM42688_GYRO_FULLSCALE_DPS / ICM42688_COUNTS_FULLSCALE)
#define ICM42688_ACCEL_SCALE  (ICM42688_ACCEL_FULLSCALE_G  / ICM42688_COUNTS_FULLSCALE)

/* Die temperature: degC = raw/132.48 + 25. */
#define ICM42688_TEMP_SENS           (132.48f)
#define ICM42688_TEMP_OFFSET         (25.0f)

/* Timing. STM0 runs at ~100 MHz. The soft reset needs ~1 ms to complete, and
 * the configuration registers must not be written for a short settling period
 * after PWR_MGMT0 wakes the analog front end.
 *
 * SYS1-001 strand B task 17 (B6.4): these two waits used to be spent inside
 * one blocking Icm42688_delayMs() call each, "generous because they run
 * once, before the scheduler starts" -- false the moment the recovery path
 * in Icm42688_read() started calling Icm42688_init() again at runtime
 * (B4b, task 5), stealing 10-30 ms of NavTask_step dispatches whenever a
 * still-answering part needed re-initialising (reviewer major finding).
 * Both are now ACCUMULATED from the caller's dtS across many non-blocking
 * Icm42688_reinitStep() calls instead -- see ICM42688_RESET_S/WAKE_S below
 * -- so Icm42688_delayMs() itself is reachable only from the boot pump
 * (Icm42688_init()), truly one-time again. */
#define ICM42688_STM_TICKS_PER_MS    (100000u)
#define ICM42688_RESET_MS            (10u)
#define ICM42688_WAKE_MS             (10u)
#define ICM42688_RESET_S             ((float32)ICM42688_RESET_MS / 1000.0f)
#define ICM42688_WAKE_S              ((float32)ICM42688_WAKE_MS  / 1000.0f)

/* B4b (SYS1-001 strand B, evidence 952275AD99001303): "present" going stale
 * without ever going FALSE. Icm42688_read()'s own SPI-failure path (below)
 * only fires when the transfer itself errors; an unpowered/floating part
 * still clocks out a well-formed, frozen frame (constant 0x8000/axis), so
 * that path never trips. Two independent, time-gated triggers close the
 * gap, neither reachable while the sensor streams real data:
 *   trigger 1 -- Icm42688_verifyPresence(): NavTask calls this on its OWN
 *     no-edge path once DRDY has been silent for a while (longer than the
 *     dt-window fallback that already exists) -- one WHO_AM_I read, capped
 *     at this rate so a genuinely dead/unplugged part is not hammered.
 *   trigger 2 -- Icm42688_reportPlausibility(): NavTask forwards its own
 *     Icm42688_plausible() result (NavTask.c already computes it and used to
 *     discard it) every dispatch; a CONSECUTIVE run of implausible samples
 *     this long (INT1 still firing, payload stuck) also drops presence.
 * Both accumulate a DURATION from the caller's dt, not a sample/call count --
 * same idiom as Ahrs.c's AHRS_FAULT_HOLD_S/s_faultHoldS, for the same reason
 * T14 gives (docs/REFACTORING_PLAN.md §3.8): a count silently changes meaning
 * with the task rate, a duration does not. */
#define ICM42688_STUCK_HOLD_S     (0.1f)    /* trigger 2: 100 ms continuously implausible */
#define ICM42688_VERIFY_PERIOD_S  (0.2f)    /* trigger 1: <= 5 Hz repeat rate on the probe */

/* Upper bound on a dt accepted by either hold-clock -- same discipline as
 * Ahrs.c's AHRS_FAULT_DT_MAX_S: reject only what must never be trusted as a
 * real duration (<=0, NaN -- compares false against every relational
 * operator here -- or absurd), not a plausible NavTask dispatch interval.
 * 1 s is generous headroom over anything NavTask.c can pass (its own
 * NAVTASK_DT_MAX_S is 0.2 s). */
#define ICM42688_HOLD_DT_MAX_S    (1.0f)

static float32 s_icm42688StuckHoldS;      /* trigger 2: consecutive implausible dt [s] */
static float32 s_icm42688VerifySinceS;    /* trigger 1: dt since the last WHO_AM_I probe [s] */

/* cppcheck-suppress-begin misra-c2012-8.7 ; deviation: read over XCP
 * SHORT_UPLOAD by raw address (tools/xcp_read.py), never referenced by C
 * code outside this file -- same class of deviation as g_imuSpiBurst*
 * above and g_dbgAhrsRealigns (Ahrs.c). */
volatile uint32 g_dbgImuStuckDrops;      /**< trigger 2 fired: presence dropped */
volatile uint32 g_dbgImuWhoAmIFail;      /**< trigger 1's probe read a bad/no WHO_AM_I */
/* cppcheck-suppress-end misra-c2012-8.7 */

/* SYS1-001 strand B task 14 (SWE1-FW-009): the ICM-42688's documented
 * invalid-data pattern on any be16 word -- see Icm42688.h for what a hit
 * means and why it does NOT touch presence. */
#define ICM42688_SENTINEL_WORD    ((sint16)0x8000)

/* cppcheck-suppress-begin misra-c2012-8.7 ; deviation: read over XCP
 * SHORT_UPLOAD by raw address (tools/xcp_read.py), never referenced by C
 * code outside this file -- same class of deviation as g_dbgImuStuckDrops
 * above. */
volatile uint32 g_dbgImuSentinelWords;
/* cppcheck-suppress-end misra-c2012-8.7 */

/* Hot-plug recovery: retry this often, in CALLS, while the device is missing.
 * Was "50 per second, ~1 s" at the 20 ms IMU task (T13 and earlier); T15
 * (docs/REFACTORING_PLAN.md §3.6) gates NavTask_step's call to this function
 * on a pending DRDY edge, but a genuinely absent sensor produces none, so the
 * fallback there still calls this every ~500 us poll while absent -- 50 calls
 * is now ~25 ms, not ~1 s. Faster recovery, not a regression: this constant
 * only bounds how OFTEN the (cheap, single-byte) probe fires, never how much
 * it costs.
 *
 * Task 17 (B6.4): also the FAILED-state backoff in Icm42688_reinitStep()
 * below -- after the state machine exhausts its one-shot SPI-mode retry,
 * this many further (single-transaction, non-blocking) calls pass before a
 * fresh attempt re-arms, same reasoning: bound how often a genuinely dead
 * part is re-probed, never how much any one probe costs. */
#define ICM42688_RECOVERY_PERIOD     (50u)

/* Task 17: Icm42688_init()'s own step budget, at 1 ms/step -- the happy
 * path (no SPI-mode retry) reaches DONE in ~29 steps, a one-shot retry in
 * ~40; 64 leaves headroom for both without ever looping unbounded. */
#define ICM42688_REINIT_MAX_STEPS    (64u)

static boolean s_icm42688Present = FALSE;

/* I5, docs/IMU_INTERRUPT.md 5.5: duration of the TEMP_DATA1..GYRO burst read
 * below, the input to the 1 kHz feasibility question in that document's
 * SS5.6 decision tree. Non-static so tools/xcp_read.py can read them --
 * zero cost to Xcp_Data/A2L/GUI, same reasoning as ImuInt.c. Declared in
 * Icm42688.h (MISRA 8.4). */
/* cppcheck-suppress-begin misra-c2012-8.7 ; deviation: read over XCP
 * SHORT_UPLOAD by raw address (tools/xcp_read.py), never referenced by C
 * code outside this file -- same class of deviation as ImuInt.c's globals. */
volatile uint32 g_imuSpiBurstTicks;      /**< last burst duration, STM0 ticks */
volatile uint32 g_imuSpiBurstMaxTicks;   /**< running max since reset         */
/* cppcheck-suppress-end misra-c2012-8.7 */

/* MODULE_STM0, not MODULE_STM1, even though Icm42688_init() (the only
 * caller, since task 17 -- B6.4) runs on CPU1 since T12
 * (docs/REFACTORING_PLAN.md) -- same deliberate, read-only cross-core
 * exception as Spi.c/SysTime.c: a one-time boot delay has no
 * dt-consistency argument for its own STM, so it is simply left on the
 * module every other timeout in this tree already uses, rather than
 * introducing a third STM reference for one call site. True again since
 * task 17 moved the runtime recovery path off this function entirely (see
 * Icm42688_reinitStep()) -- between task 5 and task 17 it was reachable at
 * runtime too, and this comment's premise was flagged false in review. */
static void Icm42688_delayMs(uint32 ms)
{
    IfxStm_waitTicks(&MODULE_STM0, ms * ICM42688_STM_TICKS_PER_MS);
}

/* Read \p len bytes starting at \p reg. The transfer is 1 + len bytes and the
 * first received byte is the address turnaround, so the payload is shifted. */
static boolean Icm42688_readRegs(uint8 reg, uint8 *data, uint8 len)
{
    uint8   tx[1u + ICM42688_BURST_LEN];
    uint8   rx[1u + ICM42688_BURST_LEN];
    boolean ok = FALSE;

    if (len <= ICM42688_BURST_LEN)
    {
        uint8 i;

        for (i = 0u; i < (1u + len); i++)
        {
            tx[i] = 0u;
        }
        tx[0] = (uint8)((reg & ICM42688_ADDR_MASK) | ICM42688_SPI_READ);

        const uint16 frameLen = (uint16)len + 1u;   /* cast the object, not a
                                                     * composite (MISRA 10.8) */
        ok = Spi_transfer(tx, rx, frameLen);
        if (ok != FALSE)
        {
            for (i = 0u; i < len; i++)
            {
                data[i] = rx[i + 1u];
            }
        }
    }
    return ok;
}

static boolean Icm42688_writeReg(uint8 reg, uint8 value)
{
    uint8 tx[2];

    tx[0] = (uint8)(reg & ICM42688_ADDR_MASK);   /* MSB clear = write */
    tx[1] = value;
    return Spi_transfer(tx, NULL_PTR, 2u);
}

boolean Icm42688_readWhoAmI(uint8 *whoAmI)
{
    return Icm42688_readRegs(ICM42688_REG_WHO_AM_I, whoAmI, 1u);
}

/* SYS1-001 strand B task 17 (B6.4, SWE1-FW-008 clause g): the non-blocking
 * re-init state machine -- see Icm42688.h for the contract. Mirrors the
 * OLD blocking sequence's states exactly, one step (at most one SPI
 * transaction, never a wait) at a time:
 *   IDLE        issue the soft-reset write, then wait for it to settle
 *   RESET_WAIT  accumulate dtS toward ICM42688_RESET_S; no bus access
 *   ID_CHECK    read WHO_AM_I; match -> WAKE_WAIT; mismatch, first
 *               attempt -> switch to SPI_MODE_3 and retry via IDLE;
 *               mismatch, already retried -> FAILED
 *   WAKE_WAIT   first call: write PWR_MGMT0; later calls: accumulate dtS
 *               toward ICM42688_WAKE_S; no bus access once written
 *   CFG         walk s_icm42688CfgTable, one write per call
 *   DONE        terminal; presence TRUE; re-armed only by
 *               Icm42688_reinitStart() (a fresh presence drop)
 *   FAILED      terminal until ICM42688_RECOVERY_PERIOD further calls
 *               have passed, then re-arms (fresh SPI_MODE_0 attempt)
 * "The part accepts mode 0 and mode 3, and the device datasheet is not
 * available to say which this board powers up expecting" -- ID_CHECK's
 * retry is that same one-shot mode probe, preserved exactly, just spread
 * across calls instead of run inline. */
typedef enum
{
    ICM42688_REINIT_IDLE = 0,
    ICM42688_REINIT_RESET_WAIT,
    ICM42688_REINIT_ID_CHECK,
    ICM42688_REINIT_WAKE_WAIT,
    ICM42688_REINIT_CFG,
    ICM42688_REINIT_DONE,
    ICM42688_REINIT_FAILED
} Icm42688_ReinitState;

/** One entry of the six-register CFG table CFG walks. */
typedef struct
{
    uint8 reg;
    uint8 value;
} Icm42688_RegVal;

static Icm42688_ReinitState s_icm42688ReinitState       = ICM42688_REINIT_IDLE;
static float32              s_icm42688ReinitWaitS        = 0.0f;
static boolean              s_icm42688ReinitPwrWritten   = FALSE;
static uint8                s_icm42688ReinitCfgIndex     = 0u;
static boolean              s_icm42688ReinitModeRetried  = FALSE;
static uint16               s_icm42688ReinitFailedCalls  = 0u;

/* Task 17: this file's own "duration since the last poll" for the absent
 * branch of Icm42688_read(), which -- unlike Icm42688_verifyPresence()/
 * Icm42688_reportPlausibility() -- has no dtS parameter from its caller.
 * Same STM0-tick idiom NavTask.c uses for its own dt (NAVTASK_TICKS_TO_S). */
#define ICM42688_TICKS_TO_S   (1.0e-8f)
static uint32 s_icm42688LastAbsentPollTicks = 0u;

/* cppcheck-suppress-begin misra-c2012-8.7 ; deviation: read over XCP
 * SHORT_UPLOAD by raw address (tools/xcp_read.py), never referenced by C
 * code outside this file -- same class of deviation as g_dbgImuStuckDrops
 * above. */
volatile uint32 g_dbgImuReinits;
volatile uint32 g_dbgImuReinitFails;
/* cppcheck-suppress-end misra-c2012-8.7 */

/* (Re)arm the state machine for a fresh attempt -- called on every genuine
 * presence LOSS (never while already reiniting: all three call sites below
 * are reachable only while s_icm42688Present was TRUE) and once at boot.
 * Always restarts at SPI_MODE_0, same as the old Icm42688_init() always
 * did regardless of which mode last worked. */
static void Icm42688_reinitStart(void)
{
    Spi_setMode(SPI_MODE_0);
    s_icm42688ReinitState         = ICM42688_REINIT_IDLE;
    s_icm42688ReinitWaitS         = 0.0f;
    s_icm42688ReinitPwrWritten    = FALSE;
    s_icm42688ReinitCfgIndex      = 0u;
    s_icm42688ReinitModeRetried   = FALSE;
    s_icm42688ReinitFailedCalls   = 0u;
    s_icm42688LastAbsentPollTicks = (uint32)SysTime_getTicks();
    s_icm42688Present             = FALSE;
}

boolean Icm42688_reinitStep(float32 dtS)
{
    /* Block scope (MISRA 8.9): read only by the CFG case below. */
    static const Icm42688_RegVal s_icm42688CfgTable[6] =
    {
        { ICM42688_REG_GYRO_CONFIG0,  ICM42688_GYRO_CONFIG0_VAL  },
        { ICM42688_REG_ACCEL_CONFIG0, ICM42688_ACCEL_CONFIG0_VAL },
        { ICM42688_REG_INT_CONFIG,    ICM42688_INT_CONFIG_VAL    },
        { ICM42688_REG_INT_CONFIG0,   ICM42688_INT_CONFIG0_VAL   },
        { ICM42688_REG_INT_CONFIG1,   ICM42688_INT_CONFIG1_VAL   },
        { ICM42688_REG_INT_SOURCE0,   ICM42688_INT_SOURCE0_VAL   }
    };

    switch (s_icm42688ReinitState)
    {
        case ICM42688_REINIT_IDLE:
        {
            (void)Icm42688_writeReg(ICM42688_REG_DEVICE_CONFIG, ICM42688_SOFT_RESET);
            s_icm42688ReinitWaitS = 0.0f;
            s_icm42688ReinitState = ICM42688_REINIT_RESET_WAIT;
            break;
        }

        case ICM42688_REINIT_RESET_WAIT:
        {
            if ((dtS > 0.0f) && (dtS < ICM42688_HOLD_DT_MAX_S))
            {
                s_icm42688ReinitWaitS += dtS;
            }
            else
            {
                /* not a usable interval -- do not advance the wait clock */
            }

            if (s_icm42688ReinitWaitS >= ICM42688_RESET_S)
            {
                s_icm42688ReinitState = ICM42688_REINIT_ID_CHECK;
            }
            else
            {
                /* still settling */
            }
            break;
        }

        case ICM42688_REINIT_ID_CHECK:
        {
            uint8   whoAmI = 0u;
            boolean ok     = Icm42688_readWhoAmI(&whoAmI);

            if ((ok != FALSE) && (whoAmI == ICM42688_WHO_AM_I_VALUE))
            {
                s_icm42688ReinitPwrWritten = FALSE;
                s_icm42688ReinitWaitS      = 0.0f;
                s_icm42688ReinitState      = ICM42688_REINIT_WAKE_WAIT;
            }
            else if (s_icm42688ReinitModeRetried == FALSE)
            {
                /* Guessing the SPI mode wrong is invisible without a scope
                 * (docs/ICM42688P.md): every transfer reports success, MISO
                 * just reads a flat 0x00. Retry once, on the other mode --
                 * IDLE reissues the soft reset next call. */
                s_icm42688ReinitModeRetried = TRUE;
                Spi_setMode(SPI_MODE_3);
                s_icm42688ReinitState = ICM42688_REINIT_IDLE;
            }
            else
            {
                g_dbgImuReinitFails++;
                s_icm42688ReinitFailedCalls = 0u;
                s_icm42688ReinitState       = ICM42688_REINIT_FAILED;
            }
            break;
        }

        case ICM42688_REINIT_WAKE_WAIT:
        {
            boolean ok = TRUE;

            if (s_icm42688ReinitPwrWritten == FALSE)
            {
                /* Power up first, then configure: the ranges and ODR the
                 * CFG table writes apply to an already-running front end,
                 * and the part needs a moment after the mode change before
                 * it accepts them. */
                ok = Icm42688_writeReg(ICM42688_REG_PWR_MGMT0, ICM42688_PWR_MGMT0_VAL);
                s_icm42688ReinitPwrWritten = TRUE;
            }
            else
            {
                if ((dtS > 0.0f) && (dtS < ICM42688_HOLD_DT_MAX_S))
                {
                    s_icm42688ReinitWaitS += dtS;
                }
                else
                {
                    /* not a usable interval -- do not advance the wait clock */
                }
            }

            if (ok == FALSE)
            {
                g_dbgImuReinitFails++;
                s_icm42688ReinitFailedCalls = 0u;
                s_icm42688ReinitState       = ICM42688_REINIT_FAILED;
            }
            else if (s_icm42688ReinitWaitS >= ICM42688_WAKE_S)
            {
                s_icm42688ReinitCfgIndex = 0u;
                s_icm42688ReinitState    = ICM42688_REINIT_CFG;
            }
            else
            {
                /* still writing PWR_MGMT0's own call, or still settling */
            }
            break;
        }

        case ICM42688_REINIT_CFG:
        {
            /* INT1 data-ready configuration (docs/ICM42688P.md 8.2).
             * SPI-side only: no ERU/ISR wiring here, that is a separate
             * task (I4). Written as plain values, never read-modify-write
             * -- see the constants above for why. */
            const boolean ok = Icm42688_writeReg(s_icm42688CfgTable[s_icm42688ReinitCfgIndex].reg,
                                                  s_icm42688CfgTable[s_icm42688ReinitCfgIndex].value);

            if (ok == FALSE)
            {
                g_dbgImuReinitFails++;
                s_icm42688ReinitFailedCalls = 0u;
                s_icm42688ReinitState       = ICM42688_REINIT_FAILED;
            }
            else
            {
                s_icm42688ReinitCfgIndex++;
                if (s_icm42688ReinitCfgIndex >= 6u)
                {
                    /* B4b: a (re)init is exactly the point at which any
                     * stale hold state from before must not carry over --
                     * a freshly (re)configured part gets a full, immediate
                     * trigger-1 allowance (Icm42688_verifyPresence()) and a
                     * clean trigger-2 window. */
                    s_icm42688StuckHoldS   = 0.0f;
                    s_icm42688VerifySinceS = ICM42688_VERIFY_PERIOD_S;
                    s_icm42688Present      = TRUE;
                    g_dbgImuReinits++;
                    s_icm42688ReinitState  = ICM42688_REINIT_DONE;
                }
                else
                {
                    /* more entries to write */
                }
            }
            break;
        }

        case ICM42688_REINIT_DONE:
        {
            /* Terminal -- only Icm42688_reinitStart() (a fresh presence
             * loss) leaves this state. */
            break;
        }

        case ICM42688_REINIT_FAILED:
        {
            s_icm42688ReinitFailedCalls++;
            if (s_icm42688ReinitFailedCalls >= ICM42688_RECOVERY_PERIOD)
            {
                s_icm42688ReinitModeRetried = FALSE;
                s_icm42688ReinitFailedCalls = 0u;
                Spi_setMode(SPI_MODE_0);
                s_icm42688ReinitState = ICM42688_REINIT_IDLE;
            }
            else
            {
                /* still backing off -- no bus access this call */
            }
            break;
        }

        default:
        {
            /* Unreachable: every Icm42688_ReinitState value is handled
             * above. Fail safe rather than loop on an impossible value. */
            s_icm42688ReinitState = ICM42688_REINIT_FAILED;
            break;
        }
    }

    return s_icm42688Present;
}

boolean Icm42688_init(void)
{
    /* SYS1-001 strand B task 17 (B6.4): a bounded pump over the same
     * non-blocking state machine the runtime recovery path now uses --
     * see Icm42688.h. dtS = 1 ms per step, Icm42688_delayMs(1) between
     * them: the happy path (no SPI-mode retry) reaches DONE in ~29 steps,
     * a one-shot mode retry in ~40 -- both comfortably inside the budget. */
    uint16 step;

    Icm42688_reinitStart();

    for (step = 0u;
         (step < ICM42688_REINIT_MAX_STEPS)
             && (s_icm42688ReinitState != ICM42688_REINIT_DONE)
             && (s_icm42688ReinitState != ICM42688_REINIT_FAILED);
         step++)
    {
        (void)Icm42688_reinitStep(0.001f);
        Icm42688_delayMs(1u);
    }

    return (s_icm42688ReinitState == ICM42688_REINIT_DONE);
}

/* Assemble one big-endian signed 16-bit axis from the burst. */
static sint16 Icm42688_be16(const uint8 *p, uint8 msbIndex)
{
    /* Assemble first, then cast the finished object: casting the composite
     * expression to a different essential type breaks MISRA 10.8. */
    const uint16 raw = (uint16)(((uint16)p[msbIndex] << 8)
                                | (uint16)p[msbIndex + 1u]);
    return (sint16)raw;
}

boolean Icm42688_read(Icm42688_Sample *sample)
{
    uint8   raw[ICM42688_BURST_LEN];
    boolean ok = FALSE;

    if (s_icm42688Present == FALSE)
    {
        /* Task 17 (B6.4, SWE1-FW-008 clause g): advance the non-blocking
         * re-init state machine by at most one SPI transaction, never a
         * wait -- see Icm42688_reinitStep(). dtS is THIS call's own
         * measured interval since the last one (this function has no dtS
         * parameter, unlike Icm42688_verifyPresence()/
         * Icm42688_reportPlausibility() below, so it measures its own --
         * same STM0-tick idiom NavTask.c uses). Called every dispatch while
         * absent, same rate the old blocking recovery probe polled at. */
        const uint32  nowTicks = (uint32)SysTime_getTicks();
        const float32 dtS = (float32)(nowTicks - s_icm42688LastAbsentPollTicks)
                          * ICM42688_TICKS_TO_S;

        s_icm42688LastAbsentPollTicks = nowTicks;
        (void)Icm42688_reinitStep(dtS);
    }
    else
    {
        /* I5: bracket exactly the SPI transfer, not the recovery branch above
         * or the axis-scaling below -- see docs/IMU_INTERRUPT.md 5.5. */
        const uint32 burstStartTicks = (uint32)SysTime_getTicks();

        ok = Icm42688_readRegs(ICM42688_REG_TEMP_DATA1, raw, ICM42688_BURST_LEN);

        g_imuSpiBurstTicks = (uint32)SysTime_getTicks() - burstStartTicks;
        if (g_imuSpiBurstTicks > g_imuSpiBurstMaxTicks)
        {
            g_imuSpiBurstMaxTicks = g_imuSpiBurstTicks;
        }

        if (ok == FALSE)
        {
            /* Lost it. Re-arm the re-init state machine so the branch above
             * starts probing (via Icm42688_reinitStep(), never blocking)
             * instead of retrying a dead device forever on a stale state. */
            Icm42688_reinitStart();
        }
        else
        {
            /* Task 14: check the raw words for the sentinel BEFORE scaling
             * -- exact test, nothing to tune, one comparison per axis. Not
             * a bus error (see Icm42688.h): presence is left untouched here
             * on purpose, so a single corrupt word amid an otherwise
             * healthy stream does not force the recovery/re-init path --
             * only Icm42688_reportPlausibility()'s existing 100 ms hold
             * (task 5) does that, and only once the whole burst is
             * implausible (e.g. all six axes sentinel, evidence
             * 952275AD99001303). */
            static const uint8 s_icm42688AxisIdx[6] =
            {
                2u, 4u, 6u,     /* accel X/Y/Z */
                8u, 10u, 12u    /* gyro  X/Y/Z */
            };
            boolean sentinel = FALSE;
            uint8   axis;

            for (axis = 0u; axis < 6u; axis++)
            {
                if (Icm42688_be16(raw, s_icm42688AxisIdx[axis]) == ICM42688_SENTINEL_WORD)
                {
                    sentinel = TRUE;
                }
                else
                {
                    /* this axis is not the sentinel -- keep checking */
                }
            }

            if (sentinel != FALSE)
            {
                g_dbgImuSentinelWords++;
                ok = FALSE;
            }
            else
            {
                /* Layout: temp(0..1), accel X/Y/Z(2..7), gyro X/Y/Z(8..13). */
                sample->tempC = ((float32)Icm42688_be16(raw, 0u) / ICM42688_TEMP_SENS)
                                + ICM42688_TEMP_OFFSET;

                sample->acc[0] = (float32)Icm42688_be16(raw, 2u)  * ICM42688_ACCEL_SCALE;
                sample->acc[1] = (float32)Icm42688_be16(raw, 4u)  * ICM42688_ACCEL_SCALE;
                sample->acc[2] = (float32)Icm42688_be16(raw, 6u)  * ICM42688_ACCEL_SCALE;

                sample->gyro[0] = (float32)Icm42688_be16(raw, 8u)  * ICM42688_GYRO_SCALE;
                sample->gyro[1] = (float32)Icm42688_be16(raw, 10u) * ICM42688_GYRO_SCALE;
                sample->gyro[2] = (float32)Icm42688_be16(raw, 12u) * ICM42688_GYRO_SCALE;
            }
        }
    }
    return ok;
}

boolean Icm42688_verifyPresence(float32 dtS)
{
    if (s_icm42688Present != FALSE)
    {
        if ((dtS > 0.0f) && (dtS < ICM42688_HOLD_DT_MAX_S))
        {
            s_icm42688VerifySinceS += dtS;
        }
        else
        {
            /* not a usable interval -- do not advance the rate-cap clock */
        }

        if (s_icm42688VerifySinceS >= ICM42688_VERIFY_PERIOD_S)
        {
            uint8 whoAmI = 0u;
            boolean ok;

            s_icm42688VerifySinceS = 0.0f;
            ok = Icm42688_readWhoAmI(&whoAmI);
            if ((ok == FALSE) || (whoAmI != ICM42688_WHO_AM_I_VALUE))
            {
                /* Answers, but wrongly, or does not answer at all: either
                 * way the device this task has been trusting is gone. Drop
                 * presence and re-arm the re-init state machine -- the
                 * existing recovery probe in Icm42688_read() (gated on
                 * s_icm42688Present == FALSE) takes it from here via
                 * Icm42688_reinitStep(), unchanged in spirit, non-blocking
                 * since task 17. */
                g_dbgImuWhoAmIFail++;
                Icm42688_reinitStart();
            }
            else
            {
                /* Answered correctly: alive, just not producing DRDY edges
                 * right now (a legitimately quiet bus, or an INT1 problem
                 * that a data path is not the one to diagnose). Keep it. */
            }
        }
        else
        {
            /* below the rate cap -- no bus access this call */
        }
    }
    else
    {
        /* Already known absent -- Icm42688_read()'s own recovery probe owns
         * reconnection; nothing for this trigger to add, and it must not
         * keep accumulating a stale rate-cap window while absent. */
        s_icm42688VerifySinceS = 0.0f;
    }

    return s_icm42688Present;
}

boolean Icm42688_reportPlausibility(boolean plausible, float32 dtS)
{
    if (s_icm42688Present != FALSE)
    {
        if (plausible != FALSE)
        {
            s_icm42688StuckHoldS = 0.0f;
        }
        else
        {
            if ((dtS > 0.0f) && (dtS < ICM42688_HOLD_DT_MAX_S))
            {
                s_icm42688StuckHoldS += dtS;
            }
            else
            {
                /* not a usable interval -- do not advance the hold clock */
            }

            if (s_icm42688StuckHoldS >= ICM42688_STUCK_HOLD_S)
            {
                /* INT1 kept firing (this function is only ever reached
                 * because a sample was read) but the payload itself has
                 * stopped moving for the whole hold window -- a frozen,
                 * well-formed frame, exactly evidence 952275AD99001303. */
                g_dbgImuStuckDrops++;
                Icm42688_reinitStart();
            }
            else
            {
                /* still within the hold window -- one bad sample must not
                 * drop a live sensor */
            }
        }
    }
    else
    {
        s_icm42688StuckHoldS = 0.0f;
    }

    return s_icm42688Present;
}

boolean Icm42688_plausible(const Icm42688_Sample *sample, float32 *liveness)
{
    const float32 accMagSq = (sample->acc[0] * sample->acc[0])
                           + (sample->acc[1] * sample->acc[1])
                           + (sample->acc[2] * sample->acc[2]);
    boolean plausible = FALSE;

    if ((accMagSq > 0.0025f) && (accMagSq < 289.0f)
        && (sample->tempC > -40.0f) && (sample->tempC < 105.0f))
    {
        plausible = TRUE;
    }
    *liveness = sample->acc[0] + sample->acc[1] + sample->acc[2]
              + sample->gyro[0] + sample->gyro[1] + sample->gyro[2]
              + sample->tempC;
    return plausible;
}

boolean Icm42688_debugDump(uint8 cfg[ICM42688_DUMP_CFG_LEN], uint8 raw[14])
{
    /* Block scope (MISRA 8.9): the order here is the order documented for
     * cfg[] in Icm42688.h. */
    static const uint8 s_icm42688DumpRegs[ICM42688_DUMP_CFG_LEN] =
    {
        ICM42688_REG_WHO_AM_I,
        ICM42688_REG_PWR_MGMT0,
        ICM42688_REG_GYRO_CONFIG0,
        ICM42688_REG_ACCEL_CONFIG0,
        ICM42688_REG_INT_STATUS,
        ICM42688_REG_INT_CONFIG,
        ICM42688_REG_INT_CONFIG0,
        ICM42688_REG_INT_CONFIG1,
        ICM42688_REG_INT_SOURCE0
    };

    boolean ok = TRUE;
    uint8   i;

    for (i = 0u; (i < ICM42688_DUMP_CFG_LEN) && (ok != FALSE); i++)
    {
        ok = Icm42688_readRegs(s_icm42688DumpRegs[i], &cfg[i], 1u);
    }
    if (ok != FALSE)
    {
        ok = Icm42688_readRegs(ICM42688_REG_TEMP_DATA1, raw, ICM42688_BURST_LEN);
    }
    return ok;
}
