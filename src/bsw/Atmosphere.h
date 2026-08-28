/**********************************************************************************************************************
 * \file Atmosphere.h
 * \brief ISA barometric altitude formula. Pure, no iLLD — host-testable.
 *
 * Split out of Bmp581/Task_Baro (T4, docs/REFACTORING_PLAN.md) because the
 * formula is not a property of the BMP581, it is the standard atmosphere
 * model, and because a pure translation unit is worth more as a test seam
 * than as a saved file.
 *********************************************************************************************************************/
#ifndef ATMOSPHERE_H
#define ATMOSPHERE_H

#include "Ifx_Types.h"

/** International barometric formula: h = 44330 * (1 - (P/P0)^0.190295).
 *
 *  Applies the 80 000..120 000 Pa QNH clamp internally and falls back to the
 *  ISA standard 101 325 Pa when \p seaLevelPa is out of that band (zero, a
 *  stale/uninitialised NVM value, or non-finite) — the caller no longer owns
 *  that policy.
 *
 *  \p pressPa is not separately guarded: a non-finite pressure produces a
 *  non-finite result exactly like any other invalid altitude, and
 *  Fusion_setBaroAlt() already rejects it before it reaches the filter
 *  (fusion.c fusion_usable()) — a second guard here would be redundant.
 *
 *  \return the altitude in metres. */
float32 Atmosphere_altitudeM(float32 pressPa, float32 seaLevelPa);

#endif /* ATMOSPHERE_H */
