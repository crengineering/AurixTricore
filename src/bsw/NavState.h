/**********************************************************************************************************************
 * \file NavState.h
 * \brief CPU1 -> CPU0 navigation-state snapshot -- the cross-core contract
 *        that NavTask (T11/T12) publishes and Housekeeping (T11) reads.
 *
 * Lives in the LMU shared block (SharedRam.h), non-cached alias, at
 * SHARED_LMU_ADDR + 0x60 -- right after the six g_coreStats slots
 * (0xB00F0000..0xB00F005F), 8-byte aligned. Single writer (the nav core),
 * any number of readers. See docs/REFACTORING_PLAN.md §2.4 for the full
 * argument; this header states the contract for this one object.
 *
 * Every field is 32-bit: LMU SRAM has no sub-word write (SharedRam.h rule 2).
 * `imuPresent` was a `uint8 imuPresent; uint8 reserved[3];` pair in the
 * original single-core code (Cpu0_Main.c's Task_Imu) -- that packs into one
 * word here instead, because a bare uint8 store would be a read-modify-write
 * of a doubleword nobody else needs to touch, but must not be trained to
 * think it is safe elsewhere.
 *
 * NOT yet wired to anything as of T10: NavTask/T11 is the first writer,
 * Housekeeping/T11 is the first reader. This file and its host test exist to
 * prove the publish/get protocol before flight code depends on it.
 *********************************************************************************************************************/
#ifndef NAVSTATE_H
#define NAVSTATE_H

#include "Ifx_Types.h"
#include "Ahrs.h"
#include "fusion.h"

typedef struct
{
    uint32       gen;          /**< monotonic, +1 per publish; no even/odd needed
                                 *   (the writer never holds an odd/half-written
                                 *   state visible -- payload is written
                                 *   completely, THEN this increments) */
    Ahrs_Values  ahrs;
    FusionValues fusion;
    float32      dtS;
    uint32       imuPresent;   /**< was uint8 + reserved[3]; see file header */
} NavState_t;

/** Defined (with __at()) in NavStatePlace.c, not here -- see SharedRam.h and
 *  that file's header comment for why a second LMU __at() object needs its
 *  own translation unit. */
extern volatile NavState_t g_navState;

/** Zero the block. CPU1 only, once at boot -- __at() storage is not
 *  guaranteed pre-zeroed the way ordinary .bss is (Measurements.c's
 *  g_xcpData needs the same explicit init, for the same reason). */
void NavState_init(void);

/** Publish one snapshot. CPU1 (the nav core) ONLY.
 *  Sequence: store payload (ahrs, fusion, dtS, imuPresent) -> Ifx__dsync() ->
 *  gen++. The barrier is what stops the TriCore store buffer from posting
 *  the gen store ahead of the payload stores even though the alias is
 *  non-cached; volatile alone only orders the compiler, not the store
 *  buffer. Never call this from any core but the nav core. */
void NavState_publish(const Ahrs_Values *ahrs, const FusionValues *fusion,
                       float32 dtS, boolean imuPresent);

/** Read one consistent snapshot into *out. Any core.
 *  Sequence: read gen -> copy payload -> re-read gen; if they match, the
 *  copy is consistent and this returns TRUE. One retry on a mismatch; a
 *  second mismatch returns FALSE and *out is left as whatever the last
 *  (possibly torn) attempt wrote -- the CALLER must not use *out on FALSE,
 *  and should keep its own last-good snapshot instead. No barrier needed on
 *  this side: TriCore is in-order and the alias is non-cached, so the
 *  compiler-ordered volatile reads execute exactly as written. Never writes
 *  shared state -- this is what makes it safe from any core. */
boolean NavState_get(NavState_t *out);

#endif /* NAVSTATE_H */
