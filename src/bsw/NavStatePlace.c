/**********************************************************************************************************************
 * \file NavStatePlace.c
 * \brief Placement ONLY for g_navState in the LMU shared block -- see
 *        NavState.h/SharedRam.h.
 *
 * Split out of NavState.c, which used to mix this __at() definition with the
 * publish/get logic in one file. cppcheck cannot parse `__at(ADDR)` at all
 * and merely tolerates the FIRST such definition in a translation unit as a
 * harmless parse error (SharedRam.h) -- but "tolerates" turned out to mean
 * only "does not also raise a second misra-c2012-8.2", not "keeps a working
 * symbol table for the rest of that file". Mixing the two in NavState.c left
 * every later g_navState.field access unresolved to cppcheck's misra addon
 * ("misra-config: Variable 'g_navState' is unknown", found only once this
 * branch was actually run through the MISRA gate). SharedRam.h already
 * stated the rule this file restores: "a later object that also needs
 * __at() here gets its own .c file". Do not add logic to this file -- see
 * SharedRam.c for the same discipline applied to g_coreStats.
 *********************************************************************************************************************/
#include "NavState.h"
#include "SharedRam.h"

/* Second object in the LMU shared block, right after the six g_coreStats
 * slots (0xB00F0000..0xB00F005F, SharedRam.c). 0x60 is 8-byte aligned, so
 * this object never shares a physical 64-bit line with g_coreStats -- a
 * different object written by different core(s) entirely. Bare literal, not
 * `SHARED_LMU_ADDR + 0x60u`: every other __at() site in this tree
 * (Measurements.c, SharedRam.c) uses a plain hex constant. */
#define NAVSTATE_LMU_ADDR   0xB00F0060u

volatile NavState_t g_navState __at(NAVSTATE_LMU_ADDR);
