/**********************************************************************************************************************
 * \file Uart.c
 * \brief Blocking UART debug output via ASCLIN0 (X109 USB-to-UART).
 *
 * Uses IfxAsclin_Asc_blockingWrite which polls the 16-byte hardware TX FIFO directly.
 * No interrupts or software FIFO buffers are needed for blocking-only operation.
 *
 * Hardware:  ASCLIN0, TX = P14.0, RX = P14.1
 * Baud rate: 115200 8N1 (iLLD default)
 *********************************************************************************************************************/
#include "Uart.h"
#include "IfxAsclin_Asc.h"

static IfxAsclin_Asc g_asclin;

void Uart_init(void)
{
    IfxAsclin_Asc_Config config;
    IfxAsclin_Asc_initModuleConfig(&config, &MODULE_ASCLIN0);
    /* Default config: 115200 baud, 8 data bits, 1 stop bit, no parity — no overrides needed */

    static const IfxAsclin_Asc_Pins pins = {
        .cts       = NULL_PTR,                            /* no hardware flow control */
        .rx        = &IfxAsclin0_RXA_P14_1_IN,
        .rxMode    = IfxPort_InputMode_pullUp,
        .rts       = NULL_PTR,                            /* no hardware flow control */
        .tx        = &IfxAsclin0_TX_P14_0_OUT,
        .txMode    = IfxPort_OutputMode_pushPull,
        .pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    config.pins = &pins;

    /* No software FIFO — blocking writes go straight to the hardware FIFO */
    config.txBuffer     = NULL_PTR;
    config.txBufferSize = 0;
    config.rxBuffer     = NULL_PTR;
    config.rxBufferSize = 0;

    IfxAsclin_Asc_initModule(&g_asclin, &config);
}

void Uart_print(const char *str)
{
    while (*str)
    {
        IfxAsclin_Asc_blockingWrite(&g_asclin, (uint8)*str++);
    }
}

void Uart_println(const char *str)
{
    Uart_print(str);
    IfxAsclin_Asc_blockingWrite(&g_asclin, '\r');
    IfxAsclin_Asc_blockingWrite(&g_asclin, '\n');
}
