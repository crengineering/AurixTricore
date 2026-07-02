/**********************************************************************************************************************
 * \file Uart.c
 * \brief Blocking UART debug output via ASCLIN0 (X109 USB-to-UART).
 *
 * All six cores share this module. A spinlock (IfxCpu_acquireMutex) serialises
 * concurrent Uart_print / Uart_println calls so messages never interleave.
 *
 * Transmission bypasses the iLLD software FIFO: uart_putchar polls the 16-entry
 * hardware TX FIFO fill level (IfxAsclin_getTxFifoFillLevel) and writes directly
 * via IfxAsclin_writeTxData. No TX interrupt or software buffer needed.
 *
 * Hardware:  ASCLIN0, TX = P14.0, RX = P14.1
 * Baud rate: 115200 8N1 (iLLD default)
 *********************************************************************************************************************/
#include "Uart.h"
#include "IfxAsclin_Asc.h"
#include "IfxAsclin.h"
#include "IfxCpu.h"

static IfxAsclin_Asc             g_asclin;
static volatile IfxCpu_mutexLock g_uartMutex = 0u;

void Uart_init(void)
{
    IfxAsclin_Asc_Config config;
    IfxAsclin_Asc_initModuleConfig(&config, &MODULE_ASCLIN0);
    /* Default config: 115200 baud, 8 data bits, 1 stop bit, no parity */

    static const IfxAsclin_Asc_Pins pins = {
        .cts       = NULL_PTR,                        /* no hardware flow control */
        .rx        = &IfxAsclin0_RXA_P14_1_IN,
        .rxMode    = IfxPort_InputMode_pullUp,
        .rts       = NULL_PTR,                        /* no hardware flow control */
        .tx        = &IfxAsclin0_TX_P14_0_OUT,
        .txMode    = IfxPort_OutputMode_pushPull,
        .pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    config.pins = &pins;

    /* No software FIFO buffers — transmission goes directly to the HW FIFO */
    config.txBuffer     = NULL_PTR;
    config.txBufferSize = 0;
    config.rxBuffer     = NULL_PTR;
    config.rxBufferSize = 0;

    IfxAsclin_Asc_initModule(&g_asclin, &config);
}

/** Write one byte directly to the hardware TX FIFO (unlocked — callers must hold g_uartMutex). */
static void uart_putchar(uint8 c)
{
    while (IfxAsclin_getTxFifoFillLevel(&MODULE_ASCLIN0) >= 16u)
    {}
    IfxAsclin_writeTxData(&MODULE_ASCLIN0, (uint32)c);
}

void Uart_print(const char *str)
{
    while (!IfxCpu_acquireMutex((IfxCpu_mutexLock *)&g_uartMutex))
    {}
    while (*str)
    {
        uart_putchar((uint8)*str++);
    }
    IfxCpu_releaseMutex((IfxCpu_mutexLock *)&g_uartMutex);
}

boolean Uart_heartbeatReceived(void)
{
    boolean received = FALSE;

    /* Poll-only RX: no interrupt, no software buffer. The 16-entry hardware
     * FIFO may overflow between calls — irrelevant, only the heartbeat
     * counts. Everything else (especially 0x00 break garbage from an
     * unplugged, unpowered USB bridge) is discarded. */
    while (IfxAsclin_getRxFifoFillLevel(&MODULE_ASCLIN0) > 0u)
    {
        if ((IfxAsclin_readRxData(&MODULE_ASCLIN0) & 0xFFu) == UART_HEARTBEAT_BYTE)
        {
            received = TRUE;
        }
    }

    return received;
}

void Uart_println(const char *str)
{
    while (!IfxCpu_acquireMutex((IfxCpu_mutexLock *)&g_uartMutex))
    {}
    while (*str)
    {
        uart_putchar((uint8)*str++);
    }
    uart_putchar('\r');
    uart_putchar('\n');
    IfxCpu_releaseMutex((IfxCpu_mutexLock *)&g_uartMutex);
}
