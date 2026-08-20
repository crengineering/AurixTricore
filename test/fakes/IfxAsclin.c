/* Host fake for the ASCLIN receive path.
 *
 * Models the wire and the 16-entry hardware FIFO separately: FakeAsclin_pushRx
 * appends to an arbitrarily long "wire" queue, but getRxFifoFillLevel never
 * reports more than 16, exactly like the silicon. The driver's drain loop
 * re-reads the fill level after every byte, so it still empties the queue in
 * one poll -- the cap matters only if a test wants to reason about overflow.
 */

#include "IfxAsclin.h"
#include "IfxAsclin_Asc.h"

Ifx_ASCLIN MODULE_ASCLIN4;

const IfxAsclin_Rx_In  IfxAsclin4_RXC_P22_6_IN  = { 0 };
const IfxAsclin_Tx_Out IfxAsclin4_TX_P22_5_OUT  = { 0 };

#define FAKE_WIRE_CAPACITY 4096u
#define FAKE_HW_FIFO_DEPTH   16u

static uint8   s_wire[FAKE_WIRE_CAPACITY];
static uint32  s_writeIdx;
static uint32  s_readIdx;
static boolean s_frameError;
static uint8   s_tx[FAKE_WIRE_CAPACITY];
static uint32  s_txCount;
static boolean s_txBlocked;
static boolean s_rxOverflow;

static IfxAsclin_Asc_Config s_lastConfig;
static IfxAsclin_ClockSource s_clockSource = IfxAsclin_ClockSource_noClock;

/* Sticky override, see FakeAsclin_forceClockSource. Cleared only by
 * FakeAsclin_reset, so it survives the initModule call inside GnssM9N_init --
 * otherwise the "module never came up" branch would be unreachable. */
static boolean               s_clockForced;
static IfxAsclin_ClockSource s_forcedClockSource;

void FakeAsclin_reset(void)
{
    s_writeIdx   = 0u;
    s_readIdx    = 0u;
    s_frameError  = FALSE;
    s_rxOverflow  = FALSE;
    s_clockForced = FALSE;
    s_clockSource = IfxAsclin_ClockSource_noClock;
    s_txCount     = 0u;
    s_txBlocked   = FALSE;
}

void FakeAsclin_pushRx(const char *bytes)
{
    while ((*bytes != '\0') && (s_writeIdx < FAKE_WIRE_CAPACITY))
    {
        s_wire[s_writeIdx] = (uint8)*bytes;
        s_writeIdx++;
        bytes++;
    }
}

void FakeAsclin_setFrameError(void) { s_frameError = TRUE; }
void FakeAsclin_setRxOverflow(void) { s_rxOverflow = TRUE; }

uint32 FakeAsclin_pending(void)
{
    return s_writeIdx - s_readIdx;
}

uint32       FakeAsclin_txCount(void) { return s_txCount; }
const uint8 *FakeAsclin_txData(void)  { return s_tx; }

void FakeAsclin_setTxBlocked(boolean blocked) { s_txBlocked = blocked; }

/* ---- the iLLD surface the driver calls ------------------------------- */

uint8 IfxAsclin_getRxFifoFillLevel(Ifx_ASCLIN *asclin)
{
    uint32 pending = FakeAsclin_pending();
    (void)asclin;

    if (pending > FAKE_HW_FIFO_DEPTH)
    {
        pending = FAKE_HW_FIFO_DEPTH;
    }
    return (uint8)pending;
}

uint32 IfxAsclin_readRxData(Ifx_ASCLIN *asclin)
{
    uint32 value = 0u;
    (void)asclin;

    if (s_readIdx < s_writeIdx)
    {
        value = (uint32)s_wire[s_readIdx];
        s_readIdx++;
    }
    return value;
}

boolean IfxAsclin_getRxFifoOverflowFlagStatus(Ifx_ASCLIN *asclin)
{
    (void)asclin;
    return s_rxOverflow;
}

void IfxAsclin_clearRxFifoOverflowFlag(Ifx_ASCLIN *asclin)
{
    (void)asclin;
    s_rxOverflow = FALSE;
}

boolean IfxAsclin_getFrameErrorFlagStatus(Ifx_ASCLIN *asclin)
{
    (void)asclin;
    return s_frameError;
}

void IfxAsclin_clearFrameErrorFlag(Ifx_ASCLIN *asclin)
{
    (void)asclin;
    s_frameError = FALSE;
}

void IfxAsclin_flushRxFifo(Ifx_ASCLIN *asclin)
{
    (void)asclin;
    s_readIdx  = 0u;
    s_writeIdx = 0u;
}

uint8 IfxAsclin_getTxFifoFillLevel(Ifx_ASCLIN *asclin)
{
    uint8 level = 0u;
    (void)asclin;

    /* Either wedged full or drained instantly. The levels in between are not
     * interesting: the cases that matter are "the driver can send" and "the
     * driver never can". */
    if (s_txBlocked != FALSE)
    {
        level = (uint8)FAKE_HW_FIFO_DEPTH;
    }
    return level;
}

void IfxAsclin_writeTxData(Ifx_ASCLIN *asclin, uint32 data)
{
    (void)asclin;

    if (s_txCount < FAKE_WIRE_CAPACITY)
    {
        s_tx[s_txCount] = (uint8)(data & 0xFFu);
        s_txCount++;
    }
}

void IfxAsclin_clearAllFlags(Ifx_ASCLIN *asclin)
{
    (void)asclin;
    s_frameError = FALSE;
    s_rxOverflow = FALSE;
}

/* ---- the ASC layer --------------------------------------------------- */

void IfxAsclin_Asc_initModuleConfig(IfxAsclin_Asc_Config *config,
                                    Ifx_ASCLIN           *asclin)
{
    config->asclin            = asclin;
    config->baudrate.baudrate = 115200.0f;                  /* vendor defaults */
    config->pins              = NULL_PTR;
    config->clockSource       = IfxAsclin_ClockSource_ascFastClock;
    config->txBuffer          = NULL_PTR;
    config->txBufferSize      = 0u;
    config->rxBuffer          = NULL_PTR;
    config->rxBufferSize      = 0u;
    config->interrupt.txPriority    = 0u;
    config->interrupt.rxPriority    = 0u;
    config->interrupt.erPriority    = 0u;
    config->interrupt.typeOfService = IfxSrc_Tos_cpu0;
}

IfxAsclin_Status IfxAsclin_Asc_initModule(IfxAsclin_Asc              *asclin,
                                          const IfxAsclin_Asc_Config *config)
{
    (void)asclin;
    s_lastConfig  = *config;

    /* The register setup happens regardless of the status below -- that is why
     * the peripheral works on silicon even though init reports an error. */
    if (s_clockForced == FALSE)
    {
        s_clockSource = config->clockSource;
    }

    if ((config->txBuffer == NULL_PTR) || (config->rxBuffer == NULL_PTR))
        return IfxAsclin_Status_configurationError;      /* see the header */

    return IfxAsclin_Status_noError;
}

IfxAsclin_ClockSource IfxAsclin_getClockSource(Ifx_ASCLIN *asclin)
{
    (void)asclin;
    return (s_clockForced == TRUE) ? s_forcedClockSource : s_clockSource;
}

void FakeAsclin_forceClockSource(IfxAsclin_ClockSource src)
{
    s_clockForced       = TRUE;
    s_forcedClockSource = src;
    s_clockSource       = src;
}

const IfxAsclin_Asc_Config *FakeAsclin_Asc_lastConfig(void)
{
    return &s_lastConfig;
}
