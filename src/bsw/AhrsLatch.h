/**********************************************************************************************************************
 * \file AhrsLatch.h
 * \brief LMU-resident backing state for Ahrs_setMag() -- the third of the
 *        "three input latches" T12 moves off a plain static.
 *
 * Writer: CPU0 (SensorTask_mag, via Ahrs_setMag()).
 * Reader: CPU1 (Ahrs_update(), inside NavTask_step).
 *
 * Unlike the baro/GNSS latches this one is LEVEL-triggered, not edge/one-shot:
 * the original code let Ahrs_update() read whatever s_magB/s_magNorm currently
 * held, every tick, with no "new sample" flag to race over in the first
 * place. So there is no two-writer bug to fix here -- gen exists purely so
 * the reader can detect and retry a torn multi-field read (SharedRam.h),
 * exactly like NavState_get(), not to decide "is this new".
 *********************************************************************************************************************/
#ifndef AHRSLATCH_H
#define AHRSLATCH_H

#include "Ifx_Types.h"

typedef struct
{
    uint32  gen;       /**< +1 per Ahrs_setMag() call (producer only)        */
    float32 magB[3];   /**< latched sample, BODY frame, hard/soft-iron corrected */
    float32 magNorm;   /**< |magB|; 0 while valid==FALSE, same as the original */
    uint32  reserved;  /**< pads the object to a multiple of 8 bytes         */
} MagLatch_t;

/** Defined (with #pragma section) in SharedRam.c -- docs/MEMORY_PLACEMENT.md
 *  T3. Formerly its own file (AhrsLatchPlace.c, __at(0xB00F0400u)), required
 *  while __at() poisoned cppcheck's symbol table for the rest of any TU it
 *  appeared in; #pragma section has no such restriction. */
extern volatile MagLatch_t g_magLatch;

#endif /* AHRSLATCH_H */
