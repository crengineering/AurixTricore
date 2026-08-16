#ifndef UART_H
#define UART_H

#include "Ifx_Types.h"

/* Baud-rate constants live in the real Uart.h too; GnssM9N.c uses this one. */
#define UART_SPEED_38400 38400.0f

void Uart_print(const char *str);
void Uart_println(const char *str);

/* ---- test control, host build only ---------------------------------- */

/** Forget every captured line. */
void        FakeUart_reset(void);
/** The most recent string passed to Uart_println (empty if none). */
const char *FakeUart_lastLine(void);
/** How many times Uart_println has been called since the last reset. */
uint32      FakeUart_lineCount(void);

#endif /* UART_H */
