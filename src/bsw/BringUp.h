/**********************************************************************************************************************
 * \file BringUp.h
 * \brief Boot-time register dumps for the I2C0/QSPI0 sensors — bring-up
 *        diagnosis only, no run-time role.
 *
 * CPU0 only: it prints over the UART console, which CPU0 owns exclusively
 * (see Uart.h). Split out of Cpu0_Main.c so a driver-bring-up dump does not
 * have to live next to core entry / task registration.
 *********************************************************************************************************************/
#ifndef BRINGUP_H
#define BRINGUP_H

#include "Ifx_Types.h"

/** Dump the BMP581, MMC5983MA and ICM-42688-P configuration/status registers
 *  plus one raw measurement burst each, over Uart_print/Uart_println. Call
 *  once at boot, after each sensor's init has run — see the per-device
 *  comments in BringUp.c for how to read the output. */
void BringUp_dumpSensors(void);

/** Permanent, explicit safety configuration for P10.7 -- the ICM-42688-P
 *  INT1 candidate pin (docs/IMU_INTERRUPT.md). Configures it as an input
 *  only (pulldown, TTL pad mode) and nothing else may ever reconfigure it:
 *  P10.7 is a 5 V VEXT pad, and driving it as an output while the INT1 wire
 *  is attached would put 5 V into the part's 3.6 V absolute maximum. Also
 *  reads the P10 IOCR field for pin 7 back from the register into
 *  g_p10Pin7Iocr, so tools/xcp_read.py can confirm from silicon (not source)
 *  that the pin is really configured as an input.
 *  Supersedes the pad self-test that proved P10.7 free of the board
 *  (g_padProbeP10 == 0x7, docs/IMU_INTERRUPT.md 2.1) -- that test drove the
 *  pin as an output and has been removed rather than left dormant. Call
 *  once at boot, before anything else touches Port 10. */
void BringUp_initImuIntPinSafe(void);

/** Silicon read-back of P10.7's IOCR4.PC7 field, set by
 *  BringUp_initImuIntPinSafe() -- see its comment above. Read live with
 *  tools/xcp_read.py; expect 1 (input, pulldown). */
extern volatile uint32 g_p10Pin7Iocr;

#endif /* BRINGUP_H */
