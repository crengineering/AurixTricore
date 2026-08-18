/**********************************************************************************************************************
 * \file GnssM9N.h
 * \brief Sparkfun GNSS NEO M9N Breakout board (UART).
 *

 *********************************************************************************************************************/
#ifndef GNSSM9N_H
#define GNSSM9N_H

#include "Ifx_Types.h"

/* GnssM9N data structure */
typedef struct
{
  uint32 rxBytes;
  uint32 sentences;
  uint16 errors;
  uint8  fixType;
  uint8  numSats;
}GnssM9N_Sample;

typedef struct
{
  uint8 numSats;
}GnssM9N_Nav;

typedef enum
{
    NONE,
    GNGGA,
    GNGSA,
    GNRMC
}gnss_type_t;

/** Initialise ASCLIN4 at 38400 on P22.5 (TX) / P22.6 (RX). TX and RX based on TC399 view */
boolean GnssM9N_init(void);
boolean GnssM9N_read(GnssM9N_Sample *sample);

#endif /* GNSSM9N_H */
