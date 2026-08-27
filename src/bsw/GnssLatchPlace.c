/**********************************************************************************************************************
 * \file GnssLatchPlace.c
 * \brief Placement ONLY for g_gnssLatch in the LMU shared block -- see
 *        FusionLatch.h/SharedRam.h.
 *
 * Split out of FusionLatchPlace.c, which used to hold both g_baroLatch and
 * g_gnssLatch: cppcheck cannot parse `__at(ADDR)` at all and only tolerates
 * the FIRST such definition in a translation unit as a harmless parse error
 * -- a second one in the same file raises misra-c2012-8.5 (and would degrade
 * cppcheck's symbol table for the rest of the file the way NavStatePlace.c's
 * header describes for g_navState, if this file had any other code to
 * degrade). SharedRam.h already stated the rule this file restores: "a
 * later object that also needs __at() gets its own .c file". Do not add
 * logic here -- see fusion.c for the read/write protocol.
 *
 * 0xB00F0300: right after g_baroLatch (0xB00F0200, FusionLatchPlace.c), same
 * 256-byte spacing used for every block in this tree (docs/CODEMAP.md 3).
 * Confirmed non-overlapping via the .map file, same check T9 used for
 * g_navState. */
#include "FusionLatch.h"
#include "SharedRam.h"

volatile GnssLatch_t g_gnssLatch __at(0xB00F0300u);
