#ifndef XCP_H_
#define XCP_H_

/* Minimal XCP-on-Ethernet (UDP) slave, lwIP raw API (NO_SYS=1).
 *
 * Supported commands: CONNECT, DISCONNECT, GET_STATUS, GET_COMM_MODE_INFO,
 * SYNCH, SET_MTA, UPLOAD, SHORT_UPLOAD — enough for pyXCP to connect and
 * read arbitrary memory (calibration/measurement reads).
 *
 * Transport framing per XCP on Ethernet: 2-byte length + 2-byte counter
 * (both little-endian) in front of each XCP packet.
 */
#include "Ifx_Types.h"

void xcpInit(void);

/* Transmit pending DAQ lists; call from the 100 ms task (event channel 0). */
void xcpDaqCycle(void);
/* 1 ms tick at which the last well-formed XCP command arrived (0 = never).
 * Motor.c compares it with the current tick for the link-loss failsafe. */
uint32 Xcp_getLastCommandMs(void);

#endif /* XCP_H_ */
