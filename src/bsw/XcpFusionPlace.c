/**********************************************************************************************************************
 * \file XcpFusionPlace.c
 * \brief Placement ONLY for g_xcpFusion -- see Measurements.h.
 *
 * Split out of Measurements.c, which used to hold this `__at()` definition
 * right after g_xcpData's. cppcheck cannot parse `__at(ADDR)` at all; it
 * tolerates the FIRST such definition in a translation unit (a harmless
 * parse error) and trips misra-c2012-8.2/8.5 on a second one in the same
 * file (Measurements.c used to carry a documented 8.2 deviation for exactly
 * this, and the file's growth then also triggered 8.5 -- the discovery that
 * prompted this split). Same discipline as NavStatePlace.c and
 * FusionLatchPlace.c/GnssLatchPlace.c: one `__at()` object per TU, no logic.
 *
 * This is a placement-only move: g_xcpFusion's type, address (XCP_FUSION_ADDR,
 * Measurements.h), and every field offset are unchanged, so the A2L is
 * unaffected. All of measurementsSetFusion()'s field writes stay in
 * Measurements.c, reaching g_xcpFusion through the same `extern` declaration
 * in Measurements.h as before. */
#include "Measurements.h"

volatile Xcp_Fusion g_xcpFusion __at(XCP_FUSION_ADDR);
