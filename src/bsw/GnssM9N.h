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
  uint8   cfgOk;        /* 1 = every CFG-VALSET we sent was acknowledged    */
  uint8   fixOk;        /* flags.gnssFixOK AND NOT flags3.invalidLlh        */
  uint8   timeOk;       /* UTC date+time resolved -- see GnssM9N.c          */
  uint8   navOk;        /* THE flag to gate sensor fusion on -- see below   */
  float32 latDeg;
  float32 lonDeg;
  float32 altM;         /* height above mean sea level                     */
  float32 speedMps;     /* ground speed, 2-D                               */
  float32 headingDeg;   /* heading of motion, 2-D                          */
  float32 hAccM;        /* horizontal accuracy estimate                    */
  float32 vAccM;        /* vertical accuracy -- typically 1.5..2x hAccM,    */
                        /* every satellite is above you so the geometry for */
                        /* the vertical component is inherently poor        */

  /* Raw fields the navigation filter needs and the float ones above cannot
   * carry (see fusion.c):
   *   iTOW  marks a NEW fix. NAV-PVT arrives at 1 Hz but this sample is polled
   *         at 10 Hz, so without it the same fix would be fused ten times and
   *         the covariance would collapse as though ten independent
   *         measurements had arrived.
   *   lat/lon in the receiver's native 1e-7 deg integers. float32 holds about
   *         seven digits, so latDeg above quantises to roughly 0.4 m -- fine
   *         for display, useless as a filter input. */
  uint32  iTOW;         /* GPS time of week [ms]                           */
  sint32  latRaw;       /* 1e-7 deg                                        */
  sint32  lonRaw;       /* 1e-7 deg                                        */
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
  uint8  validDate;      /* valid bit 0                                    */
  uint8  validTime;      /* valid bit 1                                    */
  uint8  fullyResolved;  /* valid bit 2: no second-level uncertainty left  */
  uint8  timeOk;         /* all three of the above                         */
  uint8  navOk;          /* fixOk AND 3D AND hAcc within the usable gate   */

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
    GNSS_IDLE,
    GNSS_UBX,
    GNSS_NMEA
}gnss_parse_state_t;


/** Initialise ASCLIN4 at 38400 on P22.5 (TX) / P22.6 (RX). TX and RX based on TC399 view */
boolean GnssM9N_init(void);
boolean GnssM9N_read(GnssM9N_Sample *sample);

/* --- bring-up instrumentation (GitHub issue #16), non-static so
 * tools/xcp_read.py finds them in the map -- no Xcp_Data field, no A2L, no
 * GUI change (same pattern as ImuInt.c's g_imuDrdy* globals). Declared here
 * only to satisfy MISRA 8.4 (a compatible declaration must be visible when
 * an external-linkage object is defined); no other C code reads them --
 * see the definitions and detail comments in GnssM9N.c. */
extern volatile uint8  g_ring_buf_overflow_counter;

extern volatile uint8  g_gnssCfgSent;
extern volatile uint8  g_gnssCfgExpectedAcks;
extern volatile uint8  g_gnssCfgAcked;
extern volatile uint32 g_gnssCfgNaked;
extern volatile uint8  g_gnssCfgOk;
extern volatile uint8  g_gnssTxDiscards;

extern volatile uint32 g_gnssUbxNavPvt;
extern volatile uint32 g_gnssUbxSyncCount;

#endif /* GNSSM9N_H */
