/**********************************************************************************************************************
 * \file FusionLatchPlace.c
 * \brief Placement ONLY for g_baroLatch in the LMU shared block -- see
 *        FusionLatch.h/SharedRam.h.
 *
 * Same discipline as NavStatePlace.c: cppcheck cannot parse `__at(ADDR)` at
 * all, and mixing it with fusion.c's own logic broke that file's symbol
 * table for g_navState the same way (NavStatePlace.c's header explains the
 * mechanism). This file used to also hold g_gnssLatch -- two `__at()`
 * objects in one TU trips cppcheck's misra-c2012-8.5 the same way a second
 * one always does (SharedRam.h) -- so g_gnssLatch now has its own TU,
 * GnssLatchPlace.c, right next to this one. Do not add logic here or a
 * second `__at()` object -- see fusion.c for the read/write protocol.
 *
 * 0xB00F0200: chosen with generous headroom past g_navState (0xB00F0060, and
 * however large NavState_t turns out to be -- Ahrs_Values + FusionValues is
 * comfortably under 256 bytes), so a NavState_t size change cannot silently
 * walk into this without also passing the 0xB00F0060+256 mark, matching the
 * 256-byte spacing already used for the XCP blocks (docs/CODEMAP.md 3).
 * Confirmed non-overlapping via the .map file, same check T9 used for
 * g_navState. */
#include "FusionLatch.h"
#include "SharedRam.h"

volatile BaroLatch_t g_baroLatch __at(0xB00F0200u);
