/**********************************************************************************************************************
 * \file NavState.c
 * \brief NavState publish/get protocol -- see NavState.h.
 *
 * Placement: g_navState is the second object in the LMU shared block
 * (SharedRam.h), immediately after g_coreStats (SharedRam.c, 96 bytes,
 * 0xB00F0000-0xB00F005F). 0x60 is 8-byte aligned, so this object never
 * shares a physical 64-bit line with g_coreStats -- a different object
 * written by different core(s) entirely.
 *
 * This file mixes the __at() placement with the publish/get logic, unlike
 * SharedRam.c (which is placement ONLY). That is the same pattern
 * Measurements.c already uses for the XCP blocks -- one __at() definition in
 * a file is what cppcheck tolerates; it only trips on a SECOND one in the
 * same translation unit (Measurements.c:28-36).
 *********************************************************************************************************************/
#include "NavState.h"
#include "SharedRam.h"

#define NAVSTATE_LMU_ADDR   (SHARED_LMU_ADDR + 0x60u)

volatile NavState_t g_navState __at(NAVSTATE_LMU_ADDR);

void NavState_init(void)
{
    static const Ahrs_Values  s_zeroAhrs;    /* static, no initialiser: the
                                               * language zero-initialises it */
    static const FusionValues s_zeroFusion;  /* same                          */

    /* Payload first, same discipline as NavState_publish, even though no
     * reader can race a boot-time init: keeps this function honest about the
     * order that matters once NavTask (T11) is running. */
    g_navState.ahrs       = s_zeroAhrs;
    g_navState.fusion     = s_zeroFusion;
    g_navState.dtS        = 0.0f;
    g_navState.imuPresent = 0u;
    Ifx__dsync();
    g_navState.gen        = 0u;
}

void NavState_publish(const Ahrs_Values *ahrs, const FusionValues *fusion,
                       float32 dtS, boolean imuPresent)
{
    /* Payload -> barrier -> gen++. This is the whole correctness argument
     * (docs/REFACTORING_PLAN.md §2.4): volatile only orders the compiler,
     * not the TriCore store buffer, so the barrier is what stops gen from
     * becoming visible to another core before the payload it describes. */
    g_navState.ahrs       = *ahrs;
    g_navState.fusion     = *fusion;
    g_navState.dtS        = dtS;
    g_navState.imuPresent = (imuPresent != FALSE) ? 1u : 0u;

    Ifx__dsync();

    g_navState.gen = g_navState.gen + 1u;
}

boolean NavState_get(NavState_t *out)
{
    uint8   attempt;
    boolean ok = FALSE;

    /* Bounded: at most one retry. A second mismatch means the writer is
     * publishing fast enough (or the reader stalled long enough) to overlap
     * every attempt -- give up and let the caller keep its last-good
     * snapshot rather than loop waiting on another core's progress. */
    for (attempt = 0u; (attempt < 2u) && (ok == FALSE); attempt++)
    {
        uint32 genBefore = g_navState.gen;

        out->ahrs       = g_navState.ahrs;
        out->fusion     = g_navState.fusion;
        out->dtS        = g_navState.dtS;
        out->imuPresent = g_navState.imuPresent;

        if (g_navState.gen == genBefore)
        {
            out->gen = genBefore;
            ok = TRUE;
        }
    }

    return ok;
}
