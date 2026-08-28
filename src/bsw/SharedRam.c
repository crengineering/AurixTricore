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
 *
 * Every `#pragma section farbss ...` below is guarded by `#if
 * defined(__TASKING__)`, the same idiom the vendor iLLD already uses for
 * compiler-specific pragmas (IfxCpu_Trap.c:316, Ifx_Lwip.c:96) -- found the
 * hard way (docs/MEMORY_PLACEMENT.md T4): GCC only *warns* about an unknown
 * `#pragma section` under a plain `-Wall`, which every local dev build and
 * `ctest` run uses, so this compiled clean everywhere it was checked by hand
 * from T2 onward. CI's `unit_tests.yml` "Warnings (-Og/-Os)" jobs add
 * `-Werror`, which promotes that warning to a build failure -- and nobody
 * had dispatched that workflow to notice, since T0-T3's own instructions
 * only named `misra.yml`. Guarding the pragma removes the warning outright
 * on any non-TASKING compiler instead of suppressing it after the fact. */
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
#if defined(__TASKING__)
#pragma section farbss "shared_lmu.corestats"
#endif
volatile CoreStats_t g_coreStats[CORESTATS_NUM_CORES];
#if defined(__TASKING__)
#pragma section farbss restore
#endif

/* T3: off __at(0xB00F0060u) (NavStatePlace.c, deleted) onto #pragma section.
 * CPU1 (NavTask_step) is the sole writer -- see NavState.c for the
 * publish/get protocol, unchanged by this move. */
#if defined(__TASKING__)
#pragma section farbss "shared_lmu.navstate"
#endif
volatile NavState_t g_navState;
#if defined(__TASKING__)
#pragma section farbss restore
#endif

/* T3: off __at(0xB00F0200u) (FusionLatchPlace.c, deleted) onto #pragma
 * section. CPU0 (SensorTask_baro) is the sole writer -- see fusion.c. */
#if defined(__TASKING__)
#pragma section farbss "shared_lmu.barolatch"
#endif
volatile BaroLatch_t g_baroLatch;
#if defined(__TASKING__)
#pragma section farbss restore
#endif

/* T3: off __at(0xB00F0300u) (GnssLatchPlace.c, deleted) onto #pragma
 * section. CPU0 (SensorTask_gnss) is the sole writer -- see fusion.c. */
#if defined(__TASKING__)
#pragma section farbss "shared_lmu.gnsslatch"
#endif
volatile GnssLatch_t g_gnssLatch;
#if defined(__TASKING__)
#pragma section farbss restore
#endif

/* T3: off __at(0xB00F0400u) (AhrsLatchPlace.c, deleted) onto #pragma
 * section. CPU0 (SensorTask_mag) is the sole writer -- see Ahrs.c. */
#if defined(__TASKING__)
#pragma section farbss "shared_lmu.maglatch"
#endif
volatile MagLatch_t g_magLatch;
#if defined(__TASKING__)
#pragma section farbss restore
#endif

/* T2 bridgehead, unchanged by T3: the first LMU object moved off __at() onto
 * the linker-managed `shared_lmu` group. Section name confirmed by build (T0
 * spike): `#pragma section farbss "X"` emits exactly `.bss.X`, matched by
 * the group's `select ".bss.shared_lmu.imuedge*"`. */
#if defined(__TASKING__)
#pragma section farbss "shared_lmu.imuedge"
#endif
volatile ImuEdge_t g_imuEdge;
#if defined(__TASKING__)
#pragma section farbss restore
#endif
