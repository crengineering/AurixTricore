/**********************************************************************************************************************
 * \file GnssM9N.h
 * \brief Sparkfun GNSS NEO M9N Breakout board (UART).
 *

 *********************************************************************************************************************/
#ifndef GNSSM9N_H
#define GNSSM9N_H

#include "Ifx_Types.h"


/** Initialise ASCLIN4 at 38400 on P22.5 (TX) / P22.6 (RX). TX and RX based on TC399 view */
boolean GnssM9N_init(void);
void    GnssM9N_poll(void);

#endif /* GNSSM9N_H */
