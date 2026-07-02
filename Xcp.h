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
void xcpInit(void);

#endif /* XCP_H_ */
