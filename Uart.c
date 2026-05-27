/**********************************************************************************************************************
 * \file Uart.c
 * \brief Blocking UART debug output via ASCLIN0 (X109 USB-to-UART).
 *
 * IfxAsclin_Asc_initModule is used only for hardware setup (baud rate, frame format,
 * pin mux). Transmission bypasses the iLLD software FIFO entirely and writes directly
 * to the hardware TX FIFO using IfxAsclin_writeTxData / IfxAsclin_getTxFifoFillLevel.
 *
 * Why: IfxAsclin_Asc_blockingWrite -> IfxAsclin_Asc_write -> Ifx_Fifo_write with
 * TIME_INFINITE. Without a software TX buffer the Ifx_Fifo is never initialised, so
 * Ifx_Fifo_write blocks forever — hanging CPU0 before it reaches the LED blink loop.
 *
 * Hardware:  ASCLIN0, TX = P14.0, RX = P14.1
 * Baud rate: 115200 8N1 (iLLD default)
 *********************************************************************************************************************/
#include "Uart.h"
#include "IfxAsclin_Asc.h"
#include "IfxAsclin.h"

static IfxAsclin_Asc g_asclin;

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

/** Write one byte directly to the hardware TX FIFO (no software FIFO, no ISR). */
static void uart_putchar(uint8 c)
{
    /* Wait until the 16-entry hardware TX FIFO has at least one free slot */
    while (IfxAsclin_getTxFifoFillLevel(&MODULE_ASCLIN0) >= 16u)
    {}
    IfxAsclin_writeTxData(&MODULE_ASCLIN0, (uint32)c);
}

void Uart_print(const char *str)
{
    while (*str)
    {
        uart_putchar((uint8)*str++);
    }
}

void Uart_println(const char *str)
{
    Uart_print(str);
    uart_putchar('\r');
    uart_putchar('\n');
}
