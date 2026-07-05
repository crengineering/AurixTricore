#ifndef UDPECHO_H_
#define UDPECHO_H_

/* UDP echo server on port 7: every received datagram is sent back
 * unchanged to the sender. Uses the lwIP raw API (NO_SYS=1). */
void udpEchoInit(void);

#endif /* UDPECHO_H_ */
