/**********************************************************************************************************************
 * \file Spi.c
 * \brief Blocking QSPI0 master bus service — see Spi.h.
 *
 * Wraps the iLLD IfxQspi_SpiMaster driver. That driver completes transfers
 * from its own interrupts, so this file owns the three service routines and
 * then polls the channel status against a deadline. The deadline is the point:
 * iLLD's own helpers spin unbounded, which hung CPU0 during the I2C bring-up
 * with the watchdogs disabled.
 *********************************************************************************************************************/
#include "Spi.h"
#include "Qspi/SpiMaster/IfxQspi_SpiMaster.h"   /* root-relative iLLD include (see I2c.c) */
#include "IfxQspi_PinMap.h"
#include "IfxPort.h"
#include "IfxStm.h"
#include "ConfigurationIsr.h"

/* ⚠️ SCLK is on P20.11, NOT P22.8. Proven by the pad self-test in Cpu0_Main.c
 * on 2026-08-01: P22.7 and P22.8 read stuck-high when driven as plain GPIO
 * with nothing attached, while three unwired neighbours on the same port drove
 * perfectly — so those two pads are committed on this board and cannot be
 * used. P20.13 is QSPI0's other usable SCLK output. It was D308, CPU2's
 * core-health LED, so Cpu2_Main.c no longer claims that pad — CPU0 keeps its
 * own D306 on P20.11.
 *
 * Bring-up baud rate. The ICM-42688-P accepts up to 24 MHz, but the MCU-driven
 * lines pass through 1 kOhm/2 kOhm dividers (docs/ICM42688P.md 2) whose source
 * impedance is ~0.67 kOhm, so the edges are the limit rather than the slave.
 * 1 MHz is far inside that and costs only ~112 us for the 14-byte burst at the
 * 50 Hz task rate. Raise it once the wiring is proven. */
#define SPI_BAUDRATE_HZ         (1000000.0f)

/* One data item = one byte. */
#define SPI_DATA_WIDTH_BITS     (8u)

/* Transfer deadline. STM0 runs at ~100 MHz, so 1 ms is 100 000 ticks —
 * still 8x the measured 124.4 us IMU burst (docs/IMU_INTERRUPT.md §5.6), so it
 * only ever fires on a genuine fault.
 *
 * T14 (docs/REFACTORING_PLAN.md §3.2/§3.8): was 10 ms, sized against the
 * 20 ms task period this bus is read from. At the 985 us period a wedged
 * transfer would cost TEN consecutive ticks instead of one — ten slots is
 * most of the control chain's budget for one fault. 1 ms bounds a wedge to a
 * single missed tick at any rate this task runs at. */
#define SPI_STM_TICKS_PER_MS    (100000u)
#define SPI_XFER_DEADLINE_MS    (1u)

static IfxQspi_SpiMaster         s_spiMaster;
static IfxQspi_SpiMaster_Channel s_spiImuChannel;
static boolean                   s_spiReady = FALSE;
static uint8                     s_spiMode = SPI_MODE_0;

static void spi_initChannel(uint8 spiMode);

/* The iLLD master finishes transfers in these three handlers; without them the
 * status never leaves "busy" and every transfer would hit the deadline.
 *
 * Declared before use (MISRA 8.4). They keep external linkage because the
 * interrupt vector table references them, not C code -- which is also why
 * MISRA 8.7 is deviated rather than fixed: making them static would leave the
 * vector entries unresolved. */
void spiIsrTransmit(void);
void spiIsrReceive(void);
void spiIsrError(void);

/* cppcheck-suppress-begin misra-c2012-17.3 ; deviation: IFX_INTERRUPT is an
 * iLLD macro that emits the vector-table entry; the handlers themselves are
 * prototyped immediately above.
 *
 * ⚠️ vectabNum (2nd argument) is NOT the same thing as isrProvider/`.TOS`
 * below, and the first T12 build shipped with them disagreeing -- CPU1 hung
 * forever, never reaching Scheduler_init, the moment Icm42688_init()
 * completed its first QSPI0 transfer. IFX_INTERRUPT expands to
 * `__vector_table(vectabNum)` (CompilerTasking.h): a LINK-TIME placement of
 * the handler into vector table `vectabNum` -- `int_tab_tc0`.."tc5" in
 * Lcf_Tasking_Tricore_Tc.lsl:807-833, one physical table per core
 * (`__INTTAB_CPU0`.."CPU1"... same file). `isrProvider` (IfxSrc's `.TOS`) is
 * a RUN-TIME SCU field selecting which core's interrupt controller the
 * request signals. Nothing checks the two agree: with
 * `isrProvider = IfxSrc_Tos_cpu1` and `vectabNum = 0`, CPU1 gets a real
 * interrupt request but its OWN table (int_tab_tc1) has no entry at that
 * SRPN -- CPU0's table does, uselessly -- so CPU1 traps into whatever the
 * linker left in that empty slot and never returns. The SPI_XFER_DEADLINE_MS
 * software timeout in Spi_transfer() never gets a chance to fire: a hardware
 * trap does not unwind through a C busy-wait loop. See docs/ILLD_NOTES.md
 * for the general form of this trap: vectabNum must equal the core number
 * named in isrProvider, always, for every ISR moved to a non-CPU0 core. */
IFX_INTERRUPT(spiIsrTransmit, 1, ISR_PRIORITY_QSPI0_TX);
IFX_INTERRUPT(spiIsrReceive,  1, ISR_PRIORITY_QSPI0_RX);
IFX_INTERRUPT(spiIsrError,    1, ISR_PRIORITY_QSPI0_ER);
/* cppcheck-suppress-end misra-c2012-17.3 */

/* cppcheck-suppress misra-c2012-8.7 ; deviation: referenced by the interrupt
 * vector table, not by C code; static would break the vector entry. */
void spiIsrTransmit(void)
{
    IfxQspi_SpiMaster_isrTransmit(&s_spiMaster);
}

/* cppcheck-suppress misra-c2012-8.7 ; deviation: interrupt vector entry. */
void spiIsrReceive(void)
{
    IfxQspi_SpiMaster_isrReceive(&s_spiMaster);
}

/* cppcheck-suppress misra-c2012-8.7 ; deviation: interrupt vector entry. */
void spiIsrError(void)
{
    IfxQspi_SpiMaster_isrError(&s_spiMaster);
}

void Spi_init(void)
{
    IfxQspi_SpiMaster_Config cfg;

    /* Pin set. The pad driver here is the whole reason the IMU can talk back:
     * a 5 V VEXT pad in its default mode wants ~3.5 V to read high and the IMU
     * only drives 3.3 V, so MRST would read a permanent low. ttlSpeed1 moves
     * VIH to 2.0 V. */
    const IfxQspi_SpiMaster_Pins pins =
    {
        &IfxQspi0_SCLK_P20_13_OUT, IfxPort_OutputMode_pushPull,
        &IfxQspi0_MTSR_P22_10_OUT, IfxPort_OutputMode_pushPull,
        &IfxQspi0_MRSTB_P22_9_IN,   IfxPort_InputMode_noPullDevice,
        IfxPort_PadDriver_ttlSpeed1
    };

    IfxQspi_SpiMaster_initModuleConfig(&cfg, &MODULE_QSPI0);
    cfg.txPriority      = ISR_PRIORITY_QSPI0_TX;
    cfg.rxPriority      = ISR_PRIORITY_QSPI0_RX;
    cfg.erPriority      = ISR_PRIORITY_QSPI0_ER;
    /* T12 (docs/REFACTORING_PLAN.md 2.4/Risk 3): Spi_init() itself now runs
     * on CPU1 (core1_main), so the three QSPI0 interrupts that complete a
     * transfer must be serviced there too. This alone is NOT "silent jitter
     * and nothing worse" the way Risk 3 originally framed it: it is a hard
     * hang unless the IFX_INTERRUPT vectabNum above is ALSO 1, not 0 -- see
     * the comment there, and docs/ILLD_NOTES.md, for the trap this shipped
     * as once already. */
    cfg.isrProvider     = IfxSrc_Tos_cpu1;
    cfg.maximumBaudrate = SPI_BAUDRATE_HZ;
    cfg.pins                 = &pins;
    /* No DMA: the transfers are a handful of bytes and the interrupt path is
     * simpler to reason about. The Dma tree is only linked because the iLLD
     * header pulls it in. */
    cfg.dma.useDma           = FALSE;
    IfxQspi_SpiMaster_initModule(&s_spiMaster, &cfg);

    spi_initChannel(SPI_MODE_0);
}

/* Re-configure the channel for SPI mode 0 or 3.
 *
 * Both are legal for the ICM-42688-P and the datasheet is not available to say
 * which the part powers up expecting, so Icm42688_init() tries one and falls
 * back to the other. Getting this wrong is invisible on a scope-less bench: the
 * master clocks happily, CS asserts, every transfer "completes", and MISO reads
 * a flat 0x00 because the slave never recognises the address byte and so never
 * drives SDO. */
void Spi_setMode(uint8 spiMode)
{
    spi_initChannel(spiMode);
}

static void spi_initChannel(uint8 spiMode)
{
    IfxQspi_SpiMaster_ChannelConfig chCfg;

    IfxQspi_SpiMaster_initChannelConfig(&chCfg, &s_spiMaster);
    chCfg.ch.baudrate         = SPI_BAUDRATE_HZ;
    chCfg.ch.mode.dataWidth   = SPI_DATA_WIDTH_BITS;
    chCfg.ch.mode.dataHeading = IfxQspi_DataHeading_msbFirst;

    if (spiMode == SPI_MODE_3)
    {
        chCfg.ch.mode.clockPolarity = IfxQspi_ClockPolarity_idleHigh;
        chCfg.ch.mode.shiftClock    = IfxQspi_ShiftClock_shiftTransmitDataOnLeadingEdge;
    }
    else
    {
        chCfg.ch.mode.clockPolarity = IfxQspi_ClockPolarity_idleLow;
        chCfg.ch.mode.shiftClock    = IfxQspi_ShiftClock_shiftTransmitDataOnTrailingEdge;
    }

    chCfg.ch.mode.csActiveLevel = Ifx_ActiveState_low;
    /* channelId is deliberately NOT set: iLLD overrides it from the SLSO pin's
     * slsoNr whenever sls.output.pin is non-NULL, so SLSO10 -> channel 10.
     * Setting it here would be silently ignored and misleading. */
    chCfg.sls.output.pin        = &IfxQspi0_SLSO10_P22_11_OUT;
    chCfg.sls.output.mode       = IfxPort_OutputMode_pushPull;
    chCfg.sls.output.driver     = IfxPort_PadDriver_ttlSpeed1;

    s_spiMode = spiMode;
    if (IfxQspi_SpiMaster_initChannel(&s_spiImuChannel, &chCfg) == IfxQspi_Status_ok)
    {
        s_spiReady = TRUE;
    }
}

uint8 Spi_getMode(void)
{
    return s_spiMode;
}

boolean Spi_transfer(const uint8 *tx, uint8 *rx, uint16 len)
{
    boolean ok = FALSE;

    if ((s_spiReady != FALSE) && (tx != NULL_PTR) && (len > 0u))
    {
        /* A discarded read still needs somewhere to land: the iLLD driver
         * always moves the received items, so give it a sink rather than a
         * NULL pointer. */
        /* Block scope (MISRA 8.9): only this function touches them. */
        static uint32 s_spiOkCount   = 0u;
        static uint32 s_spiFailCount = 0u;
        static uint8  s_spiSink[32];
        uint8       *dest = rx;

        if (dest == NULL_PTR)
        {
            dest = s_spiSink;
        }

        if (len <= (uint16)sizeof(s_spiSink))
        {
            if (IfxQspi_SpiMaster_exchange(&s_spiImuChannel, tx, dest, len)
                == IfxQspi_Status_ok)
            {
                /* ⚠️ Deliberate exception to "each core uses its own STM"
                 * (CLAUDE.md rule 2): since T12, Spi_transfer() runs on CPU1
                 * (called from NavTask_step) but this deadline still reads
                 * MODULE_STM0, CPU0's timer. That is intentional, not a
                 * leftover -- SysTime.c (the dt base every core shares) is
                 * likewise pinned to MODULE_STM0 on purpose, and a second
                 * time base here would let this deadline and NavTask_step's
                 * own dt drift apart. It is read-only and cross-core STM
                 * reads are safe (docs/REFACTORING_PLAN.md Risk 3); it is
                 * still a deliberate deviation and must not be "fixed" to
                 * MODULE_STM1 without re-deriving SPI_XFER_DEADLINE_MS
                 * against that module's own tick rate. */
                uint32 start = IfxStm_getLower(&MODULE_STM0);
                uint32 limit = (uint32)SPI_XFER_DEADLINE_MS * SPI_STM_TICKS_PER_MS;

                ok = TRUE;
                /* Bounded wait — see the file header. Unsigned subtraction is
                 * wrap-safe, so an STM rollover mid-transfer cannot make this
                 * spin forever. */
                while (IfxQspi_SpiMaster_getStatus(&s_spiImuChannel)
                       == IfxQspi_Status_busy)
                {
                    if ((IfxStm_getLower(&MODULE_STM0) - start) > limit)
                    {
                        ok = FALSE;
                        break;
                    }
                }
            }
        }

        /* Counters separate a silent slave from a wedged master -- the
         * distinction that cost two days on the I2C bus. */
        if (ok != FALSE) { s_spiOkCount++; } else { s_spiFailCount++; }
    }

    return ok;
}

