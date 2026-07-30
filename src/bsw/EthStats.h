/**********************************************************************************************************************
 * \file EthStats.h
 * \brief Ethernet interface throughput and link utilisation.
 *
 * Uses the GETH MAC Management Counters (MMC) — hardware octet counters in the
 * MAC itself. That keeps the measurement entirely inside BSW: no counting in
 * the lwIP data path and no edits to Libraries/Ethernet, and it counts every
 * frame the MAC actually moved, including ones lwIP dropped.
 *
 * Utilisation is bytes/s against the negotiated link rate, read back from
 * MAC_CONFIGURATION (PS/FES), so it stays correct whether the RTL8211F
 * negotiated 10, 100 or 1000 Mbit/s.
 *********************************************************************************************************************/
#ifndef ETHSTATS_H
#define ETHSTATS_H

#include "Ifx_Types.h"

/** Enable and zero the MMC counters. Call once, after Ifx_Lwip_init(). */
void EthStats_init(void);

/** Sample the counters and recompute the rates. Call at a fixed period. */
void EthStats_update(void);

/** \return link utilisation in per mille (0..1000), TX+RX against line rate. */
uint16 EthStats_getUtilPmil(void);

/** \return current throughput [bytes/s], transmit and receive summed. */
uint32 EthStats_getBytesPerSec(void);

/** \return negotiated link rate [Mbit/s]: 10, 100 or 1000. */
uint16 EthStats_getLinkMbits(void);

#endif /* ETHSTATS_H */
