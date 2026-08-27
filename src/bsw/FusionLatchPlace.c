/**********************************************************************************************************************
 * \file FusionLatchPlace.c
 * \brief Placement ONLY for g_baroLatch/g_gnssLatch in the LMU shared block --
 *        see FusionLatch.h/SharedRam.h.
 *
 * Same discipline as NavStatePlace.c: cppcheck cannot parse `__at(ADDR)` at
 * all, and mixing it with fusion.c's own logic broke that file's symbol
 * table for g_navState the same way (NavStatePlace.c's header explains the
 * mechanism). Do not add logic here -- see fusion.c for the read/write
 * protocol.
 *
 * 0xB00F0200/0300: chosen with generous headroom past g_navState
 * (0xB00F0060, and however large NavState_t turns out to be -- Ahrs_Values +
 * FusionValues is comfortably under 256 bytes), so a NavState_t size change
 * cannot silently walk into these without also passing the 0xB00F0060+256
 * mark, matching the 256-byte spacing already used for the XCP blocks
 * (docs/CODEMAP.md 3). Confirmed non-overlapping via the .map file, same
 * check T9 used for g_navState. */
#include "FusionLatch.h"
#include "SharedRam.h"

volatile BaroLatch_t g_baroLatch __at(0xB00F0200u);
volatile GnssLatch_t g_gnssLatch __at(0xB00F0300u);
