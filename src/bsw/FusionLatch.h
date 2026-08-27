/**********************************************************************************************************************
 * \file FusionLatch.h
 * \brief LMU-resident backing state for Fusion_setBaroAlt()/Fusion_setGnss() --
 *        the two of the "three input latches" T12 moves off a plain static.
 *
 * Writer: CPU0 (SensorTask_baro / SensorTask_gnss, via fusion.c's setters).
 * Reader: CPU1 (Fusion_update(), inside NavTask_step). Same rules as NavState
 * (SharedRam.h, docs/REFACTORING_PLAN.md 2.4): every field 32-bit, 8-byte
 * aligned, one writer, payload -> Ifx__dsync() -> gen++.
 *
 * FIXES the "consumer clears the flag" two-writer bug the plan calls out:
 * the old s_baroNew/s_gnssNew booleans were SET by the producer and CLEARED
 * by the consumer -- two cores writing the same word. Here only gen is the
 * publish marker (producer-only, monotonic) and the consumer keeps its OWN
 * "last consumed" generation locally (fusion.c statics, not shared) -- it
 * never writes back into this block.
 *
 * droppedCount/dupesCount are standalone 32-bit counters, not part of the
 * gen-guarded payload: SharedRam.h rule 3 says a single-field object needs no
 * barrier, and these are read every Fusion_update() tick regardless of
 * whether a new sample latched, exactly like the original code (dropped was
 * incremented straight into the published FusionValues from the setter, with
 * no gating on the consumer's gen at all).
 *
 * lastITow/haveITow (the GNSS dupe-of-the-last-fix guard) stay in fusion.c as
 * ordinary statics, NOT here: they are read and written exclusively by
 * Fusion_setGnss(), i.e. only ever touched by the one core that calls it, so
 * they carry no cross-core visibility requirement and do not belong in a
 * shared block.
 *********************************************************************************************************************/
#ifndef FUSIONLATCH_H
#define FUSIONLATCH_H

#include "Ifx_Types.h"

typedef struct
{
    uint32  gen;           /**< +1 per accepted valid sample (producer only) */
    float32 altM;          /**< latest valid altitude [m], positive UP       */
    float32 refM;          /**< first valid sample -- the d=0 reference      */
    uint32  refOk;         /**< 0/1; was a bare boolean, see SharedRam.h r.2 */
    uint32  droppedCount;  /**< cumulative NaN/absurd samples rejected       */
    uint32  reserved;      /**< pads the object to a multiple of 8 bytes     */
} BaroLatch_t;

typedef struct
{
    uint32  gen;           /**< +1 per accepted NEW fix (producer only)      */
    sint32  lat;           /**< [1e-7 deg]                                   */
    sint32  lon;           /**< [1e-7 deg]                                   */
    float32 altM;          /**< [m], positive UP                             */
    float32 speedMps;      /**< 2-D ground speed                             */
    float32 headingDeg;    /**< heading of motion                            */
    float32 hAccM;         /**< horizontal accuracy estimate                 */
    uint32  iTOW;          /**< GPS time of week of the fix just latched     */
    uint32  droppedCount;  /**< cumulative NaN/absurd fixes rejected         */
    uint32  dupesCount;    /**< cumulative fixes seen again (same iTOW)      */
} GnssLatch_t;

/** Defined (with __at()) in FusionLatchPlace.c -- see that file and
 *  SharedRam.h for why a second/third __at() object needs its own TU. */
extern volatile BaroLatch_t g_baroLatch;
extern volatile GnssLatch_t g_gnssLatch;

#endif /* FUSIONLATCH_H */
