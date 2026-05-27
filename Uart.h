/**********************************************************************************************************************
 * \file Uart.h
 * \brief Blocking UART debug output via ASCLIN0 (X109 USB-to-UART, 115200 8N1).
 *
 * Only CPU0 should call these functions — ASCLIN0 is owned exclusively by CPU0.
 *********************************************************************************************************************/
#ifndef UART_H
#define UART_H

/** Initialise ASCLIN0 at 115200 8N1 on P14.0 (TX) / P14.1 (RX). */
void Uart_init(void);

/** Transmit a null-terminated string (blocking). */
void Uart_print(const char *str);

/** Transmit a null-terminated string followed by \r\n (blocking). */
void Uart_println(const char *str);

#endif /* UART_H */
