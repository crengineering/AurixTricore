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

/* Transfer deadline. STM0 runs at ~100 MHz, so 10 ms is 1 000 000 ticks —
 * three orders of magnitude more than the longest transfer this bus performs,
 * so it only ever fires on a genuine fault. */
#define SPI_STM_TICKS_PER_MS    (100000u)
#define SPI_XFER_DEADLINE_MS    (10u)

static IfxQspi_SpiMaster         s_spiMaster;
static IfxQspi_SpiMaster_Channel s_spiImuChannel;
static boolean                   s_spiReady = FALSE;
static uint8                     s_spiMode = SPI_MODE_0;

static void spi_initChannel(uint8 spiMode);
static uint32                    s_spiOkCount = 0u;
static uint32                    s_spiFailCount = 0u;

/* The iLLD master finishes transfers in these three handlers; without them the
 * status never leaves "busy" and every transfer would hit the deadline. */
IFX_INTERRUPT(spiIsrTransmit, 0, ISR_PRIORITY_QSPI0_TX);
IFX_INTERRUPT(spiIsrReceive,  0, ISR_PRIORITY_QSPI0_RX);
IFX_INTERRUPT(spiIsrError,    0, ISR_PRIORITY_QSPI0_ER);

void spiIsrTransmit(void)
{
    IfxQspi_SpiMaster_isrTransmit(&s_spiMaster);
}

void spiIsrReceive(void)
{
    IfxQspi_SpiMaster_isrReceive(&s_spiMaster);
}

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
    cfg.isrProvider     = IfxSrc_Tos_cpu0;
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
        static uint8 s_spiSink[32];
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
    }

    if (ok != FALSE)
    {
        s_spiOkCount++;
    }
    else
    {
        s_spiFailCount++;
    }
    return ok;
}

uint32 Spi_getOkCount(void)
{
    return s_spiOkCount;
}

uint32 Spi_getFailCount(void)
{
    return s_spiFailCount;
}
