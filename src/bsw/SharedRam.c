/**********************************************************************************************************************
 * \file SharedRam.c
 * \brief Placement for the whole LMU cross-core shared block -- see SharedRam.h.
 *
 * This file holds placement definitions and NOTHING else -- no logic. As of
 * T3 (docs/MEMORY_PLACEMENT.md) all six objects are `#pragma section farbss`
 * into the linker-managed `shared_lmu` group (Lcf_Tasking_Tricore_Tc.lsl);
 * `__at()` is gone from this file entirely, and with it the one-object-per-TU
 * rule that used to force NavStatePlace.c/FusionLatchPlace.c/GnssLatchPlace.c/
 * AhrsLatchPlace.c/ImuEdgePlace.c to exist. `#pragma section` parses cleanly
 * in cppcheck regardless of how many placed objects share a TU (measured,
 * docs/MEMORY_PLACEMENT.md §7) -- that is what makes collecting them here
 * safe, and per the T2 finding (same doc), it is also what FIXES cppcheck's
 * view of every object below: an `__at()` syntax error anywhere in a TU
 * discards the symbol table for the rest of that file, which is why
 * `g_imuEdge` stayed invisible through T2 despite already being
 * `#pragma section` -- `g_coreStats`'s `__at()` was still in this file. With
 * that gone too, all six are visible.
 *
 * Order below matches the physical layout before T3 (and the group's
 * `select` order in the .lsl): g_coreStats, g_navState, g_baroLatch,
 * g_gnssLatch, g_magLatch, g_imuEdge. The linker now owns the actual
 * addresses -- read them from the `.map`, never hardcode one.
 *********************************************************************************************************************/
#include "SharedRam.h"
#include "CoreStats.h"
#include "NavState.h"
#include "FusionLatch.h"
#include "AhrsLatch.h"
#include "ImuEdge.h"

/* First consumer of the block (T9), and its first live-on-hardware proof:
 * six independent 16-byte slots, one writer per core, all fields 32-bit
 * (CoreStats.h). T3: off __at(SHARED_LMU_ADDR) onto #pragma section, first
 * member of the `shared_lmu` group's select order. */
#pragma section farbss "shared_lmu.corestats"
volatile CoreStats_t g_coreStats[CORESTATS_NUM_CORES];
#pragma section farbss restore

/* T3: off __at(0xB00F0060u) (NavStatePlace.c, deleted) onto #pragma section.
 * CPU1 (NavTask_step) is the sole writer -- see NavState.c for the
 * publish/get protocol, unchanged by this move. */
#pragma section farbss "shared_lmu.navstate"
volatile NavState_t g_navState;
#pragma section farbss restore

/* T3: off __at(0xB00F0200u) (FusionLatchPlace.c, deleted) onto #pragma
 * section. CPU0 (SensorTask_baro) is the sole writer -- see fusion.c. */
#pragma section farbss "shared_lmu.barolatch"
volatile BaroLatch_t g_baroLatch;
#pragma section farbss restore

/* T3: off __at(0xB00F0300u) (GnssLatchPlace.c, deleted) onto #pragma
 * section. CPU0 (SensorTask_gnss) is the sole writer -- see fusion.c. */
#pragma section farbss "shared_lmu.gnsslatch"
volatile GnssLatch_t g_gnssLatch;
#pragma section farbss restore

/* T3: off __at(0xB00F0400u) (AhrsLatchPlace.c, deleted) onto #pragma
 * section. CPU0 (SensorTask_mag) is the sole writer -- see Ahrs.c. */
#pragma section farbss "shared_lmu.maglatch"
volatile MagLatch_t g_magLatch;
#pragma section farbss restore

/* T2 bridgehead, unchanged by T3: the first LMU object moved off __at() onto
 * the linker-managed `shared_lmu` group. Section name confirmed by build (T0
 * spike): `#pragma section farbss "X"` emits exactly `.bss.X`, matched by
 * the group's `select ".bss.shared_lmu.imuedge*"`. */
#pragma section farbss "shared_lmu.imuedge"
volatile ImuEdge_t g_imuEdge;
#pragma section farbss restore
