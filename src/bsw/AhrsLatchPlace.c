/**********************************************************************************************************************
 * \file AhrsLatchPlace.c
 * \brief Placement ONLY for g_magLatch in the LMU shared block -- see
 *        AhrsLatch.h/SharedRam.h. Same discipline as NavStatePlace.c /
 *        FusionLatchPlace.c: cppcheck cannot parse `__at(ADDR)`, so this
 *        object gets its own translation unit and no logic. See Ahrs.c for
 *        the read/write protocol.
 *
 * 0xB00F0400: past g_baroLatch (0xB00F0200) and g_gnssLatch (0xB00F0300),
 * same 256-byte spacing. Confirmed non-overlapping via the .map file. */
#include "AhrsLatch.h"
#include "SharedRam.h"

volatile MagLatch_t g_magLatch __at(0xB00F0400u);
