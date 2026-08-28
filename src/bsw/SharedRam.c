/**********************************************************************************************************************
 * \file SharedRam.c
 * \brief Placement ONLY for the LMU cross-core shared block -- see SharedRam.h.
 *
 * This file holds placement definitions and NOTHING else -- no logic. Two
 * mechanisms coexist here as of T2 (docs/MEMORY_PLACEMENT.md):
 *   - `__at()`: cppcheck cannot parse it at all; it tolerates the FIRST such
 *     definition in a translation unit and trips on a second one in the same
 *     file (see Measurements.c:28-36), so each `__at()` object still gets
 *     its own .c file. Do not add a second `__at()` object here.
 *   - `#pragma section farbss "..."`: parses cleanly in cppcheck regardless
 *     of how many placed objects share a TU (measured,
 *     docs/MEMORY_PLACEMENT.md §7) -- these can live together in this file.
 *********************************************************************************************************************/
#include "SharedRam.h"
#include "CoreStats.h"
#include "ImuEdge.h"

/* First consumer of the block (T9), and its first live-on-hardware proof:
 * six independent 16-byte slots, one writer per core, all fields 32-bit
 * (CoreStats.h). Placed at the very base of the block -- the next object
 * (NavState, T10) starts at SHARED_LMU_ADDR + 0x60 (96 bytes on, still
 * 8-byte aligned) in its own file. */
volatile CoreStats_t g_coreStats[CORESTATS_NUM_CORES] __at(SHARED_LMU_ADDR);

/* T2 bridgehead (docs/MEMORY_PLACEMENT.md): the first LMU object moved off
 * __at() onto the linker-managed `shared_lmu` group (Lcf_Tasking_Tricore_Tc.lsl).
 * Formerly its own file (ImuEdgePlace.c, __at(0xB00F0500u)) -- that file is
 * deleted; #pragma section needs no such isolation. Section name confirmed
 * by build (T0 spike): `#pragma section farbss "X"` emits exactly
 * `.bss.X`, matched by the group's `select ".bss.shared_lmu.imuedge*"`.
 * Address is now locator-assigned, not a literal -- read it from the .map,
 * never hardcode it (ImuEdge.h). */
#pragma section farbss "shared_lmu.imuedge"
volatile ImuEdge_t g_imuEdge;
#pragma section farbss restore
