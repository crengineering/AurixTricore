/**********************************************************************************************************************
 * \file ImuEdgePlace.c
 * \brief Placement ONLY for g_imuEdge in the LMU shared block -- see
 *        ImuEdge.h/SharedRam.h. Same discipline as NavStatePlace.c/
 *        AhrsLatchPlace.c/FusionLatchPlace.c: cppcheck cannot parse
 *        `__at(ADDR)`, so this object gets its own translation unit and no
 *        logic. See ImuInt.c (write) and ImuEdge.h (read) for the protocol.
 *
 * 0xB00F0500: the fourth crossing (docs/REFACTORING_PLAN.md §3.6), past
 * g_baroLatch (0xB00F0200), g_gnssLatch (0xB00F0300) and g_magLatch
 * (0xB00F0400), same 256-byte spacing. Confirmed non-overlapping via the
 * .map file, same check every other object in this block has had. */
#include "ImuEdge.h"
#include "SharedRam.h"

volatile ImuEdge_t g_imuEdge __at(0xB00F0500u);
