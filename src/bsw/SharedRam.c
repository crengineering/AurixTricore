/**********************************************************************************************************************
 * \file SharedRam.c
 * \brief Placement ONLY for the LMU cross-core shared block -- see SharedRam.h.
 *
 * This file holds __at() definitions and NOTHING else. cppcheck cannot parse
 * the __at() storage-placement extension at all; it tolerates the FIRST such
 * definition in a translation unit and trips on a second one in the same
 * file (see Measurements.c:28-36 for the discovered detail, and SharedRam.h
 * for why that means one object per file here). Do not add logic to this
 * file, and do not add a second __at() object to it -- give the next one its
 * own .c file instead.
 *********************************************************************************************************************/
#include "SharedRam.h"
#include "CoreStats.h"

/* First consumer of the block (T9), and its first live-on-hardware proof:
 * six independent 16-byte slots, one writer per core, all fields 32-bit
 * (CoreStats.h). Placed at the very base of the block -- the next object
 * (NavState, T10) starts at SHARED_LMU_ADDR + 0x60 (96 bytes on, still
 * 8-byte aligned) in its own file. */
volatile CoreStats_t g_coreStats[CORESTATS_NUM_CORES] __at(SHARED_LMU_ADDR);
