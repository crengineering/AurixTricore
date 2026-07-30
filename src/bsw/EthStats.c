/**********************************************************************************************************************
 * \file EthStats.c
 * \brief Ethernet link utilisation from the GETH MMC counters — see EthStats.h.
 *********************************************************************************************************************/
#include "EthStats.h"
#include "SysTime.h"
#include "IfxGeth_reg.h"

/* STM0 ticks per second (100 MHz). */
#define ETH_TICKS_PER_SEC   (100000000u)

/* Line rate -> bytes/s. Ethernet is 8 bits per byte on the wire plus framing
 * overhead; utilisation here is payload octets against raw line rate, which is
 * the conventional and slightly conservative reading. */
#define ETH_BITS_PER_BYTE   (8u)

static uint32 s_lastTx;
static uint32 s_lastRx;
static uint32 s_lastTicks;
static uint32 s_bytesPerSec;
static uint16 s_utilPmil;

void EthStats_init(void)
{
    /* Free-running counters: no reset-on-read (we take deltas ourselves), not
     * frozen, and allowed to roll over — unsigned subtraction handles the wrap.
     * CNTRST self-clears once the counters are zeroed. */
    /* cppcheck-suppress misra-c2012-19.2 ; deviation: the union is
     * inside the iLLD SFR definition (Ifx_GETH_*), not this code. */
    GETH_MMC_CONTROL.B.RSTONRD   = 0u;
    /* cppcheck-suppress misra-c2012-19.2 ; deviation: the union is
     * inside the iLLD SFR definition (Ifx_GETH_*), not this code. */
    GETH_MMC_CONTROL.B.CNTFREEZ  = 0u;
    /* cppcheck-suppress misra-c2012-19.2 ; deviation: the union is
     * inside the iLLD SFR definition (Ifx_GETH_*), not this code. */
    GETH_MMC_CONTROL.B.CNTSTOPRO = 0u;
    /* cppcheck-suppress misra-c2012-19.2 ; deviation: the union is
     * inside the iLLD SFR definition (Ifx_GETH_*), not this code. */
    GETH_MMC_CONTROL.B.CNTRST    = 1u;

    s_lastTx      = 0u;
    s_lastRx      = 0u;
    s_lastTicks   = SysTime_getTicks();
    s_bytesPerSec = 0u;
    s_utilPmil    = 0u;
}

uint16 EthStats_getLinkMbits(void)
{
    uint16 mbits;

    /* DWMAC speed encoding: PS = 0 -> gigabit (GMII/RGMII), PS = 1 -> MII, and
     * then FES picks 100 over 10. */
    /* cppcheck-suppress misra-c2012-19.2 ; deviation: the union is
     * inside the iLLD SFR definition (Ifx_GETH_*), not this code. */
    if (GETH_MAC_CONFIGURATION.B.PS == 0u)
    {
        mbits = 1000u;
    }
    /* cppcheck-suppress misra-c2012-19.2 ; deviation: the union is
     * inside the iLLD SFR definition (Ifx_GETH_*), not this code. */
    else if (GETH_MAC_CONFIGURATION.B.FES != 0u)
    {
        mbits = 100u;
    }
    else
    {
        mbits = 10u;
    }
    return mbits;
}

void EthStats_update(void)
{
    uint32 now     = SysTime_getTicks();
    uint32 elapsed = now - s_lastTicks;          /* unsigned: wrap-safe */

    /* Average over a full second even though this is called at 100 Hz.
     *
     * Network traffic here is bursty: with a short window most windows contain
     * no packets at all and the rate reads a misleading 0, then spikes. One
     * second smooths that into a number worth putting on a bar graph, and it
     * also keeps the counter deltas large enough to be meaningful. */
    if (elapsed >= ETH_TICKS_PER_SEC)
    {
        /* cppcheck-suppress misra-c2012-19.2 ; deviation: the union is
         * inside the iLLD SFR definition (Ifx_GETH_*), not this code. */
        uint32 tx    = GETH_TX_OCTET_COUNT_GOOD_BAD.U;
        /* cppcheck-suppress misra-c2012-19.2 ; deviation: the union is
         * inside the iLLD SFR definition (Ifx_GETH_*), not this code. */
        uint32 rx    = GETH_RX_OCTET_COUNT_GOOD_BAD.U;
        uint32 delta = (uint32)(tx - s_lastTx) + (uint32)(rx - s_lastRx);
        uint32 capacity;

        /* bytes/s = delta * ticksPerSec / elapsed, in 64-bit.
         *
         * Do NOT reorder this into 32-bit divides. Dividing first truncates:
         * at 100 ms windows a light load of 64 bytes gives delta/100 = 0 and the
         * whole rate collapses to zero (observed 2026-07-30). The 64-bit product
         * cannot overflow — a full gigabit second is 1.25e8 bytes, times 1e8
         * ticks is 1.25e16, comfortably inside uint64. */
        s_bytesPerSec = (uint32)(((uint64)delta * (uint64)ETH_TICKS_PER_SEC)
                                 / (uint64)elapsed);

        /* Link capacity in bytes/s, then per mille of it — again 64-bit first. */
        capacity = ((uint32)EthStats_getLinkMbits() * 1000000u) / ETH_BITS_PER_BYTE;
        if (capacity > 0u)
        {
            uint32 pmil = (uint32)(((uint64)s_bytesPerSec * 1000u) / (uint64)capacity);
            if (pmil > 1000u)
            {
                pmil = 1000u;      /* clamp: framing overhead can nudge past 100% */
            }
            s_utilPmil = (uint16)pmil;
        }

        s_lastTx    = tx;
        s_lastRx    = rx;
        s_lastTicks = now;
    }
}

uint16 EthStats_getUtilPmil(void)
{
    return s_utilPmil;
}

uint32 EthStats_getBytesPerSec(void)
{
    return s_bytesPerSec;
}
