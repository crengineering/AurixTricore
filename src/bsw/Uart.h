/**********************************************************************************************************************
 * \file Uart.h
 * \brief Blocking UART debug output via ASCLIN0 (X109 USB-to-UART, 115200 8N1).
 *
 * Only CPU0 should call these functions — ASCLIN0 is owned exclusively by CPU0.
 *********************************************************************************************************************/
#ifndef UART_H
#define UART_H

#include "Ifx_Types.h"

/** Initialise ASCLIN0 at 115200 8N1 on P14.0 (TX) / P14.1 (RX). */
void Uart_init(void);

/** Transmit a null-terminated string (blocking). */
void Uart_print(const char *str);

/** Transmit a null-terminated string followed by \r\n (blocking). */
void Uart_println(const char *str);

/** Transmit one byte as two upper-case hex digits (blocking). There is no
 *  formatted print available on this console, so every boot-time dump of a
 *  register or raw byte goes through this helper. */
void Uart_printHexByte(uint8 v);

/** Heartbeat byte the PC GUI sends every 500 ms ('H'). Only this value counts
 *  as link activity: an unplugged USB bridge pulls the RX line low (break),
 *  which floods the FIFO with 0x00 garbage frames. */
#define UART_HEARTBEAT_BYTE 0x48u

/* UART speed rates */
#define UART_SPEED_38400 38400.0f

/** Drain the hardware RX FIFO; TRUE if a heartbeat byte arrived since the
 *  last call. Used as link-alive detector for the UART-link diagnosis. */
boolean Uart_heartbeatReceived(void);

#endif /* UART_H */
