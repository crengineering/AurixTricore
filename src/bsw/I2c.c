/**********************************************************************************************************************
 * \file I2c.c
 * \brief Blocking I2C0 master bus service (shared sensor bus) — see I2c.h.
 *
 * Wraps the iLLD IfxI2c_I2c driver. Register access uses the classic
 * "write register pointer, then read" sequence. A STOP terminates both the
 * pointer write and the data read: the bus devices latch the register pointer
 * across the STOP, so a true repeated start is not required and the bus is
 * always released between phases.
 *
 * The STOP comes from the module-level SOPE bit set in I2c_init(), NOT from the
 * per-device enableRepeatedStart field — IfxI2c_I2c_read2()/write2() never read
 * that field. See the comment at the stopOnPacketEnd assignment; getting this
 * wrong silently freezes the MPU-6050's sensor registers.
 *
 * Hardware: I2C0, SCL = P13.1, SDA = P13.2, TTL pads.
 *********************************************************************************************************************/
#include "I2c.h"
#include "I2c/I2c/IfxI2c_I2c.h"   /* root-relative iLLD include (see gpio.h convention) */
#include "IfxPort.h"
#include "IfxStm.h"

/* Bring-up baud rate: 100 kHz standard mode. The BMP581 / MMC5983 / NEO-M9N bus
 * can move to 400 kHz Fast-mode once the wiring is proven (docs/PINNING.md 2.5). */
#define I2C_BAUDRATE_HZ   (100000.0f)

/* Retries on NAK only (slave busy or absent). Each attempt costs about one byte
 * time (~90 us at 100 kHz), so this stays far inside the 20 ms sensor task.
 * A hard failure (timeout / arbitration lost / FIFO error) is NOT retried — it
 * triggers bus recovery and fails the call immediately. */
#define I2C_MAX_ATTEMPTS  (8u)

/* Per-transfer deadline for every hardware wait, in STM0 ticks (100 MHz).
 * 1 000 000 ticks = 10 ms; the longest real transfer here is the 14-byte IMU
 * burst at ~1.4 ms, so this only ever fires on a genuinely stuck bus while
 * still leaving the 20 ms task period intact. */
#define I2C_XFER_TIMEOUT_TICKS  (1000000u)

/** Outcome of one bounded hardware transfer. */
typedef enum
{
    I2C_XFER_OK = 0,   /* all bytes moved, transfer terminated normally */
    I2C_XFER_NAK,      /* slave did not acknowledge — worth retrying     */
    I2C_XFER_FAIL      /* timeout / arbitration lost / FIFO error        */
} i2c_XferResult;

/* Largest register write payload ([reg] + data) this module supports. */
#define I2C_WRITE_MAX     (8u)

/* Bus-recovery bit-bang. SCL = P13.1, SDA = P13.2 (see I2c.h). */
#define I2C_SCL_PIN         (1u)
#define I2C_SDA_PIN         (2u)

/* One full byte plus its ACK: the most clocks any slave can still be waiting
 * for after being interrupted mid-transfer. */
#define I2C_RECOVERY_PULSES (9u)

/* Half a bit period at ~100 kHz. STM0 runs at 100 MHz (CLAUDE.md), so
 * 500 ticks = 5 us. I2c_init() runs once at startup, so blocking here is fine. */
#define I2C_RECOVERY_HALF_TICKS (500u)

static IfxI2c_I2c g_i2c;   /* I2C0 module handle */

/* Off __at() onto `#pragma section` (docs/MEMORY_PLACEMENT.md T4) -- an
 * absolute group at the unchanged LCF_XCP_I2CDBG_START literal. Guarded by
 * `#if defined(__TASKING__)` -- see SharedRam.c for why (GCC -Werror). */
#if defined(__TASKING__)
#pragma section farbss "xcp_i2cdbg"
#endif
volatile I2c_Debug g_i2cDebug;
#if defined(__TASKING__)
#pragma section farbss restore
#endif

/* Record the outcome of one transfer attempt for the XCP-readable counters. */
static void i2c_note(i2c_XferResult res, uint8 busStatus)
{
    g_i2cDebug.magic         = XCP_I2CDBG_MAGIC;
    g_i2cDebug.lastBusStatus = busStatus;
    g_i2cDebug.lastResult    = (uint8)res;
    if (res == I2C_XFER_OK)        { g_i2cDebug.okCount++;   }
    else if (res == I2C_XFER_NAK)  { g_i2cDebug.nakCount++;  }
    else                           { g_i2cDebug.failCount++; }
}

/* Bounded transfer engine, defined below the bus-recovery helpers it reports to. */
static i2c_XferResult i2c_xferWrite(uint16 devAddr8, const uint8 *data, uint16 len);
static i2c_XferResult i2c_xferRead(uint16 devAddr8, uint8 *data, uint16 len);

/* MODULE_STM0 note (T12, docs/REFACTORING_PLAN.md Risk 3): every I2c.c
 * timeout/delay below reads MODULE_STM0, same as Spi.c and SysTime.c. Unlike
 * those two, this one is NOT actually cross-core after T12: I2c_init() and
 * every caller (Bmp581/Mmc5983 via SensorTask_baro/SensorTask_mag,
 * PeriphDiag_update via Housekeeping_100ms) still run on CPU0, the same
 * core that owns MODULE_STM0. Documented here anyway, alongside the two
 * genuine exceptions, so nobody "fixes" this to MODULE_STM0 assuming it was
 * cross-core when the I2C bus itself never left CPU0. */
static void i2c_halfBitDelay(void)
{
    IfxStm_waitTicks(&MODULE_STM0, I2C_RECOVERY_HALF_TICKS);
}

/* Free a slave that is stuck holding SDA low, then hand the pins to the I2C
 * module.
 *
 * Why this is needed: the sensors are polled at 50 Hz, and a CPU-only reset
 * (debugger "start", the board RESET button, a watchdog) can land in the middle
 * of a transfer. The MCU restarts but the breakouts stay powered, so the slave
 * is still part-way through clocking out a byte and keeps SDA low, waiting for
 * clocks that never come. The I2C module then sees a permanently busy bus and
 * EVERY device fails to ACK — symptom: all sensors "not found" after a warm
 * reset, healthy again after a power cycle.
 *
 * Standard recovery (I2C-bus spec UM10204 §3.1.16): clock SCL until the slave
 * releases SDA, then issue a manual STOP. */
static void i2c_busRecovery(void)
{
    uint8 i;

    /* Both lines as open-drain GPIO and released: the breakout pull-ups drive
     * them high, and driving "high" on an open-drain pad just lets go. */
    IfxPort_setPinModeOutput(&MODULE_P13, I2C_SCL_PIN, IfxPort_OutputMode_openDrain,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(&MODULE_P13, I2C_SDA_PIN, IfxPort_OutputMode_openDrain,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinPadDriver(&MODULE_P13, I2C_SCL_PIN, IfxPort_PadDriver_ttlSpeed1);
    IfxPort_setPinPadDriver(&MODULE_P13, I2C_SDA_PIN, IfxPort_PadDriver_ttlSpeed1);
    IfxPort_setPinState(&MODULE_P13, I2C_SCL_PIN, IfxPort_State_high);
    IfxPort_setPinState(&MODULE_P13, I2C_SDA_PIN, IfxPort_State_high);
    i2c_halfBitDelay();

    for (i = 0u; i < I2C_RECOVERY_PULSES; i++)
    {
        if (IfxPort_getPinState(&MODULE_P13, I2C_SDA_PIN) != FALSE)
        {
            break;                  /* SDA released - bus already free */
        }
        IfxPort_setPinState(&MODULE_P13, I2C_SCL_PIN, IfxPort_State_low);
        i2c_halfBitDelay();
        IfxPort_setPinState(&MODULE_P13, I2C_SCL_PIN, IfxPort_State_high);
        i2c_halfBitDelay();
    }

    /* Manual STOP: pull SDA low with SCL high, then release SDA. Leaves every
     * slave's state machine at a clean transfer boundary. */
    IfxPort_setPinState(&MODULE_P13, I2C_SDA_PIN, IfxPort_State_low);
    i2c_halfBitDelay();
    IfxPort_setPinState(&MODULE_P13, I2C_SCL_PIN, IfxPort_State_high);
    i2c_halfBitDelay();
    IfxPort_setPinState(&MODULE_P13, I2C_SDA_PIN, IfxPort_State_high);
    i2c_halfBitDelay();
}

/* TRUE when both lines are released, i.e. the bus is idle. Pn_IN reflects the
 * pad level even while the pins are owned by the I2C module, so this is valid
 * at any time. */
static boolean i2c_busIdle(void)
{
    boolean idle = FALSE;

    if ((IfxPort_getPinState(&MODULE_P13, I2C_SCL_PIN) != FALSE)
        && (IfxPort_getPinState(&MODULE_P13, I2C_SDA_PIN) != FALSE))
    {
        idle = TRUE;
    }
    return idle;
}

/* Configure the pins and the I2C0 module. Split out of I2c_init() so the
 * fault path can rebuild the module after a bit-banged bus recovery. */
static void i2c_configureModule(void)
{
    IfxI2c_I2c_Config config;

    IfxI2c_I2c_initConfig(&config, &MODULE_I2C0);

    /* SCL = P13.1, SDA = P13.2; TTL pads so a 3.3 V bus reads high on 5 V VEXT. */
    static const IfxI2c_Pins pins = {
        .scl       = &IfxI2c0_SCL_P13_1_INOUT,
        .sda       = &IfxI2c0_SDA_P13_2_INOUT,
        .padDriver = IfxPort_PadDriver_ttlSpeed1
    };
    config.pins     = &pins;
    config.baudrate = I2C_BAUDRATE_HZ;

    /* Generate a real STOP at the end of every transfer.
     *
     * THIS IS NOT OPTIONAL. iLLD defaults SOPE to 0, which means "after transfer
     * go into master restart state" — the master keeps owning the bus and NEVER
     * emits a STOP condition. The device-level IfxI2c_I2c_deviceConfig field
     * `enableRepeatedStart` looks like it controls this, but IfxI2c_I2c_read2()
     * and write2() never read it; only this module-level bit matters.
     *
     * Without it the MPU-6050 sees a read burst that never ends. It holds its
     * sensor data registers static for the duration of a read (so a burst can't
     * tear), so the registers freeze at whatever was latched by the first read
     * and never update again — while WHO_AM_I, config readback and writes all
     * keep working normally. If the first read happens to land after a
     * DEVICE_RESET, the frozen value is 0x00 and the IMU appears permanently
     * dead. The BMP388 has no such hold behaviour, which is why it worked and
     * masked this for so long. Bench-bisected 2026-07-30, see docs/MPU6050.md §7. */
    config.addrFifoCfg.addressConfig.stopOnPacketEnd = 1;

    IfxI2c_I2c_initModule(&g_i2c, &config);
}

void I2c_init(void)
{
    i2c_busRecovery();      /* must run BEFORE the I2C module claims the pins */
    i2c_configureModule();
}

boolean I2c_busIsIdle(void)
{
    return i2c_busIdle();
}

void I2c_getLineState(boolean *sclReleased, boolean *sdaReleased)
{
    if (sclReleased != NULL_PTR)
    {
        *sclReleased = IfxPort_getPinState(&MODULE_P13, I2C_SCL_PIN);
    }
    if (sdaReleased != NULL_PTR)
    {
        *sdaReleased = IfxPort_getPinState(&MODULE_P13, I2C_SDA_PIN);
    }
}

/* Called after a hard transfer failure: the module may be mid-packet and a slave
 * may still be holding SDA. Free the lines and rebuild the module so the next
 * call starts from a defined state instead of inheriting the wreckage. */
static void i2c_recoverAfterFailure(void)
{
    g_i2cDebug.recoverCount++;

    /* Hard-reset the kernel and the FIFOs, not just re-run initModule().
     *
     * Measured 2026-07-30: after the IMU's supply was pulled and restored, the
     * register-pointer WRITE of every probe still completed while the following
     * READ failed - never a NAK, always a hardware error - and it stayed that
     * way indefinitely. So the slaves were answering and the master's receive
     * path was wedged. IfxI2c_I2c_initModule() reconfigures registers but does
     * not clear the kernel state machine or the FIFOs, so re-running it left
     * the module exactly as broken; the failure/recovery counters climbed in
     * lockstep forever. A kernel reset plus an explicit FIFO reset is what
     * actually clears it. */
    IfxI2c_resetModule(&MODULE_I2C0);
    IfxI2c_resetFifo(&MODULE_I2C0);

    i2c_busRecovery();
    i2c_configureModule();
}

/* NAK-retry wrapper around one bounded write. A hard failure is never retried:
 * it recovers the bus and gives up, so a stuck slave costs one timeout, not
 * I2C_MAX_ATTEMPTS of them. */
static boolean i2c_writePhase(uint8 addr7, const uint8 *data, uint16 len)
{
    i2c_XferResult res;
    uint32         attempts = 0u;
    boolean        ok       = FALSE;

    do
    {
        res = i2c_xferWrite((uint16)((uint16)addr7 << 1u), data, len);
        attempts++;
    } while ((res == I2C_XFER_NAK) && (attempts < I2C_MAX_ATTEMPTS));

    if (res == I2C_XFER_OK)
    {
        ok = TRUE;
    }
    else if (res == I2C_XFER_FAIL)
    {
        i2c_recoverAfterFailure();
    }
    else
    {
        /* persistent NAK: slave absent or busy — nothing to recover */
    }
    return ok;
}

/* NAK-retry wrapper around one bounded read. */
static boolean i2c_readPhase(uint8 addr7, uint8 *data, uint16 len)
{
    i2c_XferResult res;
    uint32         attempts = 0u;
    boolean        ok       = FALSE;

    do
    {
        res = i2c_xferRead((uint16)((uint16)addr7 << 1u), data, len);
        attempts++;
    } while ((res == I2C_XFER_NAK) && (attempts < I2C_MAX_ATTEMPTS));

    if (res == I2C_XFER_OK)
    {
        ok = TRUE;
    }
    else if (res == I2C_XFER_FAIL)
    {
        i2c_recoverAfterFailure();
    }
    else
    {
        /* persistent NAK */
    }
    return ok;
}

/* Gate every transfer on a released bus.
 *
 * IfxI2c_I2c_read2()/write2() contain UNBOUNDED busy-waits (`while (RIS == 0)`,
 * `while (TX_END == FALSE)`). If a slave is left holding SDA low — a jostled
 * jumper, a slave reset mid-byte, ESD on the wires — those spins never exit and
 * CPU0 hangs forever with the watchdogs disabled (observed 2026-07-30: the LED
 * froze solid after the sensor wiring was disturbed by hand). iLLD is not ours
 * to patch, so refuse to enter it while the bus is stuck: recover, rebuild the
 * module, and fail the transfer if the bus is still held.
 *
 * LIMITATION: this cannot save a transfer that is already in flight when the bus
 * breaks. It bounds the common case (bus found stuck before a transfer starts),
 * not a glitch landing mid-byte. Closing that gap needs bounded reimplementations
 * of read2/write2 against the low-level IfxI2c API. */
static boolean i2c_ensureBusFree(void)
{
    boolean ok = i2c_busIdle();

    if (ok == FALSE)
    {
        i2c_busRecovery();
        i2c_configureModule();
        ok = i2c_busIdle();
    }
    return ok;
}

/* --- bounded transfer engine ----------------------------------------------
 *
 * These replace IfxI2c_I2c_write2() / read2(). The iLLD versions are functionally
 * fine but every hardware wait in them is an UNBOUNDED spin:
 *     while ((i2c->RIS.U) == 0) {}
 *     while (IfxI2c_getProtocolInterruptSourceStatus(..., transmissionEnd) == FALSE) {}
 * If a slave stops clocking mid-byte — a jostled jumper, a slave reset, ESD —
 * no interrupt ever arrives and CPU0 spins forever. With both watchdogs disabled
 * on this project that is a permanent freeze (observed twice on 2026-07-30 while
 * the sensor was handled by hand: LED stuck solid, only a debugger reset revived
 * it). iLLD is not ours to patch, so the same state machine is reproduced here
 * with a deadline on every wait.
 *
 * Scope: 7-bit addressing, standard/fast mode. FIFO burst size and alignment are
 * read from FIFOCFG at run time, exactly as iLLD does, so a config change here
 * stays correct. --------------------------------------------------------- */

static uint32 i2c_ticksSince(uint32 start)
{
    /* Unsigned arithmetic, so the 32-bit STM0 wrap (~43 s) is handled. */
    return IfxStm_getLower(&MODULE_STM0) - start;
}

/* Bounded form of `while ((i2c->RIS.U) == 0) {}`. */
static boolean i2c_waitRis(Ifx_I2C *i2c, uint32 start)
{
    boolean timedOut = FALSE;
    boolean ok       = FALSE;

    while (((i2c->RIS.U) == 0u) && (timedOut == FALSE))
    {
        if (i2c_ticksSince(start) > I2C_XFER_TIMEOUT_TICKS)
        {
            timedOut = TRUE;
        }
    }
    if (timedOut == FALSE)
    {
        ok = TRUE;
    }
    return ok;
}

/* Bounded wait for TX_END, clearing it on success. */
static boolean i2c_waitTxEnd(Ifx_I2C *i2c, uint32 start)
{
    boolean timedOut = FALSE;
    boolean ok       = FALSE;

    while ((IfxI2c_getProtocolInterruptSourceStatus(i2c, IfxI2c_ProtocolInterruptSource_transmissionEnd)
            == FALSE) && (timedOut == FALSE))
    {
        if (i2c_ticksSince(start) > I2C_XFER_TIMEOUT_TICKS)
        {
            timedOut = TRUE;
        }
    }
    if (timedOut == FALSE)
    {
        IfxI2c_clearProtocolInterruptSource(i2c, IfxI2c_ProtocolInterruptSource_transmissionEnd);
        ok = TRUE;
    }
    return ok;
}

/* Map the post-transfer error flags to a result, consuming TX_END. */
static i2c_XferResult i2c_finishTransfer(Ifx_I2C *i2c, uint32 start)
{
    i2c_XferResult res = I2C_XFER_OK;

    if (IfxI2c_getProtocolInterruptSourceStatus(i2c, IfxI2c_ProtocolInterruptSource_arbitrationLost)
        != FALSE)
    {
        IfxI2c_clearProtocolInterruptSource(i2c, IfxI2c_ProtocolInterruptSource_arbitrationLost);
        res = I2C_XFER_FAIL;
    }
    else
    {
        if (IfxI2c_getErrorInterruptSourceStatus(i2c, IfxI2c_ErrorInterruptSource_txFifoOverflow)
            != FALSE)
        {
            IfxI2c_clearErrorInterruptSource(i2c, IfxI2c_ErrorInterruptSource_txFifoOverflow);
            res = I2C_XFER_FAIL;
        }
        if (IfxI2c_getErrorInterruptSourceStatus(i2c, IfxI2c_ErrorInterruptSource_rxFifoOverflow)
            != FALSE)
        {
            IfxI2c_clearErrorInterruptSource(i2c, IfxI2c_ErrorInterruptSource_rxFifoOverflow);
            res = I2C_XFER_FAIL;
        }
        if (IfxI2c_getProtocolInterruptSourceStatus(i2c,
                IfxI2c_ProtocolInterruptSource_notAcknowledgeReceived) != FALSE)
        {
            IfxI2c_clearProtocolInterruptSource(i2c,
                IfxI2c_ProtocolInterruptSource_notAcknowledgeReceived);
            res = I2C_XFER_NAK;
        }

        if (i2c_waitTxEnd(i2c, start) == FALSE)
        {
            res = I2C_XFER_FAIL;
        }
    }
    return res;
}

/* Bounded equivalent of IfxI2c_I2c_write2(): address byte + \p len payload. */
static i2c_XferResult i2c_xferWrite(uint16 devAddr8, const uint8 *data, uint16 len)
{
    Ifx_I2C         *i2c   = g_i2c.i2c;
    i2c_XferResult   res   = I2C_XFER_OK;
    uint32           start = IfxStm_getLower(&MODULE_STM0);
    uint32           words = (uint32)1u << i2c->FIFOCFG.B.TXBS;
    uint32           inc   = (uint32)1u << i2c->FIFOCFG.B.TXFA;
    uint16           left  = (uint16)(len + 1u);   /* payload + address byte */
    boolean          addrPending = TRUE;
    IfxI2c_BusStatus bus   = IfxI2c_getBusStatus(i2c);
    const uint8     *src   = data;      /* local cursor: MISRA 17.8 forbids
                                         * modifying the parameter itself */
    uint32           txPacket;

    if ((bus != IfxI2c_BusStatus_idle) && (bus != IfxI2c_BusStatus_busyMaster))
    {
        res = I2C_XFER_FAIL;
    }
    else
    {
        IfxI2c_setTransmitPacketSize(i2c, (Ifx_SizeT)left);

        while ((left > 0u) && (res == I2C_XFER_OK))
        {
            if (i2c_waitRis(i2c, start) == FALSE)
            {
                res = I2C_XFER_FAIL;
            }
            else if (IfxI2c_isFifoRequest(i2c) == FALSE)
            {
                res = I2C_XFER_FAIL;   /* an interrupt, but not a FIFO request */
            }
            else
            {
                uint32 burst = 1u;
                uint32 w;
                uint32 c;

                if ((IfxI2c_getDtrinterruptSourceStatus(i2c, IfxI2c_DtrInterruptSource_burstRequest)
                     != FALSE)
                    || (IfxI2c_getDtrinterruptSourceStatus(i2c,
                            IfxI2c_DtrInterruptSource_lastBurstRequest) != FALSE))
                {
                    burst = words;
                }

                for (w = 0u; w < burst; w++)
                {
                    txPacket = 0u;
                    for (c = 0u; c < 4u; c += inc)
                    {
                        uint8 byteVal = 0u;

                        if (addrPending != FALSE)
                        {
                            byteVal     = (uint8)(devAddr8 & 0xFEu);   /* R/W = 0 */
                            addrPending = FALSE;
                            left--;
                        }
                        else if (left > 0u)
                        {
                            byteVal = *src;
                            src++;
                            left--;
                        }
                        else
                        {
                            /* padding beyond the packet size — ignored by the FIFO */
                        }
                        /* Byte c of the FIFO word. Explicit shifts rather than a
                         * union: MISRA 19.2 forbids the union keyword, and the
                         * TriCore is little-endian so byte c is bits [8c+7:8c]. */
                        txPacket |= ((uint32)byteVal << (c * 8u));
                    }
                    IfxI2c_writeFifo(i2c, txPacket);
                }
                IfxI2c_clearAllDtrInterruptSources(i2c);
            }
        }

        if (res == I2C_XFER_OK)
        {
            if (i2c_waitRis(i2c, start) == FALSE)
            {
                res = I2C_XFER_FAIL;
            }
            else
            {
                res = i2c_finishTransfer(i2c, start);
            }
        }
    }
    i2c_note(res, (uint8)bus);
    return res;
}

/* Bounded equivalent of IfxI2c_I2c_read2(): address byte + \p len bytes in. */
static i2c_XferResult i2c_xferRead(uint16 devAddr8, uint8 *data, uint16 len)
{
    Ifx_I2C         *i2c   = g_i2c.i2c;
    i2c_XferResult   res   = I2C_XFER_OK;
    uint32           start = IfxStm_getLower(&MODULE_STM0);
    uint32           words = (uint32)1u << i2c->FIFOCFG.B.RXBS;
    uint32           inc   = (uint32)1u << i2c->FIFOCFG.B.RXFA;
    uint16           left  = len;
    IfxI2c_BusStatus bus   = IfxI2c_getBusStatus(i2c);
    uint8           *dst   = data;      /* local cursor, see MISRA 17.8 above */
    uint32           rxPacket;

    if ((bus != IfxI2c_BusStatus_idle) && (bus != IfxI2c_BusStatus_busyMaster))
    {
        res = I2C_XFER_FAIL;
    }
    else
    {
        IfxI2c_setTransmitPacketSize(i2c, 1);            /* the address byte */
        IfxI2c_setReceivePacketSize(i2c, (Ifx_SizeT)len);

        /* Send the address with R/W = 1. */
        if (i2c_waitRis(i2c, start) == FALSE)
        {
            res = I2C_XFER_FAIL;
        }
        else
        {
            if (IfxI2c_isFifoRequest(i2c) != FALSE)
            {
                uint32 addrRd = (uint32)devAddr8 | 0x1u;
                IfxI2c_writeFifo(i2c, addrRd);
                IfxI2c_clearAllDtrInterruptSources(i2c);
            }

            if (i2c_waitRis(i2c, start) == FALSE)
            {
                res = I2C_XFER_FAIL;
            }
            else if (IfxI2c_getProtocolInterruptSourceStatus(i2c,
                         IfxI2c_ProtocolInterruptSource_arbitrationLost) != FALSE)
            {
                IfxI2c_clearProtocolInterruptSource(i2c,
                    IfxI2c_ProtocolInterruptSource_arbitrationLost);
                res = I2C_XFER_FAIL;
            }
            else if (IfxI2c_getProtocolInterruptSourceStatus(i2c,
                         IfxI2c_ProtocolInterruptSource_notAcknowledgeReceived) != FALSE)
            {
                IfxI2c_clearProtocolInterruptSource(i2c,
                    IfxI2c_ProtocolInterruptSource_notAcknowledgeReceived);
                (void)i2c_waitTxEnd(i2c, start);
                res = I2C_XFER_NAK;        /* no such slave — caller may retry */
            }
            else if (IfxI2c_getProtocolInterruptSourceStatus(i2c,
                         IfxI2c_ProtocolInterruptSource_receiveMode) == FALSE)
            {
                IfxI2c_clearAllProtocolInterruptSources(i2c);
                res = I2C_XFER_FAIL;       /* never entered receive mode */
            }
            else
            {
                IfxI2c_clearProtocolInterruptSource(i2c,
                    IfxI2c_ProtocolInterruptSource_receiveMode);

                while ((left > 0u) && (res == I2C_XFER_OK))
                {
                    if (i2c_waitRis(i2c, start) == FALSE)
                    {
                        res = I2C_XFER_FAIL;
                    }
                    else if (IfxI2c_isFifoRequest(i2c) == FALSE)
                    {
                        res = I2C_XFER_FAIL;
                    }
                    else
                    {
                        uint32 burst = 1u;
                        uint32 w;
                        uint32 c;

                        if ((IfxI2c_getDtrinterruptSourceStatus(i2c,
                                 IfxI2c_DtrInterruptSource_burstRequest) != FALSE)
                            || (IfxI2c_getDtrinterruptSourceStatus(i2c,
                                    IfxI2c_DtrInterruptSource_lastBurstRequest) != FALSE))
                        {
                            burst = words;
                        }

                        for (w = 0u; w < burst; w++)
                        {
                            rxPacket = i2c->RXD.U;
                            for (c = 0u; c < 4u; c += inc)
                            {
                                if (left > 0u)
                                {
                                    *dst = (uint8)((rxPacket >> (c * 8u)) & 0xFFu);
                                    dst++;
                                    left--;
                                }
                                else
                                {
                                    /* trailing FIFO padding */
                                }
                            }
                        }
                        IfxI2c_clearAllDtrInterruptSources(i2c);
                    }
                }

                if (res == I2C_XFER_OK)
                {
                    if (i2c_waitRis(i2c, start) == FALSE)
                    {
                        res = I2C_XFER_FAIL;
                    }
                    else
                    {
                        res = i2c_finishTransfer(i2c, start);
                    }
                }
            }
        }
    }
    i2c_note(res, (uint8)bus);
    return res;
}

boolean I2c_readReg(uint8 addr7, uint8 reg, uint8 *data, uint16 len)
{
    uint8   regAddr = reg;
    boolean ok;

    ok = i2c_ensureBusFree();
    if (ok != FALSE)
    {
        ok = i2c_writePhase(addr7, &regAddr, 1u);   /* set the register pointer */
        if (ok != FALSE)
        {
            ok = i2c_readPhase(addr7, data, len);   /* read the data bytes      */
        }
    }
    return ok;
}

boolean I2c_writeReg(uint8 addr7, uint8 reg, const uint8 *data, uint16 len)
{
    uint8   buf[I2C_WRITE_MAX];      /* [reg][data...] */
    uint16  i;
    boolean ok = FALSE;

    if (len < I2C_WRITE_MAX)               /* [reg] + len data must fit the buffer */
    {
        ok = i2c_ensureBusFree();
        if (ok != FALSE)
        {
            buf[0] = reg;
            for (i = 0u; i < len; i++)
            {
                buf[i + 1u] = data[i];
            }
            ok = i2c_writePhase(addr7, buf, (uint16)(len + 1u));
        }
    }
    return ok;
}

boolean I2c_writeByte(uint8 addr7, uint8 reg, uint8 value)
{
    return I2c_writeReg(addr7, reg, &value, 1u);
}
