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
  uint16 year;
  uint8  month;
  uint8  day;
  uint8  hour;
  uint8  min;
  uint8  sec;

  /* navigation solution, from UBX-NAV-PVT. Scaled to human units here so
   * every consumer sees the same definition (see docs/GNSS_UBX.md section 6). */
  uint8   fixOk;        /* flags.gnssFixOK AND NOT flags3.invalidLlh        */
  float32 latDeg;
  float32 lonDeg;
  float32 altM;         /* height above mean sea level                     */
  float32 speedMps;     /* ground speed, 2-D                               */
  float32 headingDeg;   /* heading of motion, 2-D                          */
  float32 hAccM;        /* horizontal accuracy estimate                    */
}GnssM9N_Sample;

typedef struct
{
  uint8  numSats;
  uint8  fixQuality;
  uint16 year;
  uint8  month;
  uint8  day;
  uint8  hour;
  uint8  min;
  uint8  sec;

  /* UBX-NAV-PVT, kept in the receiver's native integer units so nothing is
   * lost; scaling to float happens once, in GnssM9N_read. */
  uint8  fixOk;         /* 1 = gnssFixOK set and lon/lat/height valid      */
  sint32 lat;           /* 1e-7 deg                                        */
  sint32 lon;           /* 1e-7 deg                                        */
  sint32 hMSL;          /* mm above mean sea level                         */
  sint32 gSpeed;        /* mm/s, 2-D                                       */
  sint32 headMot;       /* 1e-5 deg                                        */
  uint32 hAcc;          /* mm                                              */
  uint32 vAcc;          /* mm                                              */
  uint32 iTOW;          /* ms into the GPS week                            */
  uint16 pDOP;          /* 0.01                                            */
}GnssM9N_Nav;

typedef enum
{
    NONE,
    GNGGA,
    GNGSA,
    GNRMC
}gnss_type_t;

typedef enum
{
    GNSS_IDLE,
    GNSS_UBX,
    GNSS_NMEA
}gnss_parse_state_t;

#define GNGGA_FIX_QUALITY     6u
#define GNGGA_SATELLITES_USED 7u
#define GNSS_MAX_SATELLITES  32u

/** Initialise ASCLIN4 at 38400 on P22.5 (TX) / P22.6 (RX). TX and RX based on TC399 view */
boolean GnssM9N_init(void);
boolean GnssM9N_read(GnssM9N_Sample *sample);

#endif /* GNSSM9N_H */
