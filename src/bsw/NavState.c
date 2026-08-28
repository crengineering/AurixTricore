/**********************************************************************************************************************
 * \file NavState.c
 * \brief NavState publish/get protocol -- see NavState.h.
 *
 * g_navState itself is defined (with #pragma section) in SharedRam.c, not
 * here -- docs/MEMORY_PLACEMENT.md T3 (formerly NavStatePlace.c and __at(),
 * split out from this file after mixing the two broke cppcheck's misra
 * addon for every g_navState.field access below; #pragma section has no
 * such restriction, but the object still lives with its five LMU siblings
 * rather than back in this file). This file only ever sees g_navState
 * through NavState.h's plain `extern` declaration, so its symbol table is
 * exactly as ordinary as any other BSW file's.
 *********************************************************************************************************************/
#include "NavState.h"
#include "SharedRam.h"

void NavState_init(void)
{
    static const Ahrs_Values  s_zeroAhrs;    /* static, no initialiser: the
                                               * language zero-initialises it */
    static const FusionValues s_zeroFusion;  /* same                          */
    uint8 i;

    /* Payload first, same discipline as NavState_publish, even though no
     * reader can race a boot-time init: keeps this function honest about the
     * order that matters once NavTask (T11) is running. */
    g_navState.ahrs       = s_zeroAhrs;
    g_navState.fusion     = s_zeroFusion;
    g_navState.dtS        = 0.0f;
    g_navState.imuPresent = 0u;
    for (i = 0u; i < 3u; i++)
    {
        g_navState.imuAcc[i]  = 0.0f;
        g_navState.imuGyro[i] = 0.0f;
    }
    g_navState.imuTempC    = 0.0f;
    g_navState.imuLiveness = 0.0f;
    /* cppcheck-suppress misra-c2012-17.3 ; deviation: Ifx__dsync() wraps
     * TASKING's __dsync() intrinsic, which has no declaration anywhere
     * cppcheck can see (SharedRam.h) -- same root cause as the grandfathered
     * instance in Cpu0_Main.c. */
    Ifx__dsync();
    g_navState.gen        = 0u;
}

void NavState_publish(const Ahrs_Values *ahrs, const FusionValues *fusion,
                       float32 dtS, boolean imuPresent,
                       const float32 imuAcc[3], const float32 imuGyro[3],
                       float32 imuTempC, float32 liveness)
{
    uint8 i;

    /* Payload -> barrier -> gen++. This is the whole correctness argument
     * (docs/REFACTORING_PLAN.md §2.4): volatile only orders the compiler,
     * not the TriCore store buffer, so the barrier is what stops gen from
     * becoming visible to another core before the payload it describes. */
    g_navState.ahrs       = *ahrs;
    g_navState.fusion     = *fusion;
    g_navState.dtS        = dtS;
    g_navState.imuPresent = (imuPresent != FALSE) ? 1u : 0u;
    for (i = 0u; i < 3u; i++)
    {
        g_navState.imuAcc[i]  = imuAcc[i];
        g_navState.imuGyro[i] = imuGyro[i];
    }
    g_navState.imuTempC    = imuTempC;
    g_navState.imuLiveness = liveness;

    /* cppcheck-suppress misra-c2012-17.3 ; deviation: see NavState_init. */
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
        uint8  i;

        out->ahrs       = g_navState.ahrs;
        out->fusion     = g_navState.fusion;
        out->dtS        = g_navState.dtS;
        out->imuPresent = g_navState.imuPresent;
        for (i = 0u; i < 3u; i++)
        {
            out->imuAcc[i]  = g_navState.imuAcc[i];
            out->imuGyro[i] = g_navState.imuGyro[i];
        }
        out->imuTempC    = g_navState.imuTempC;
        out->imuLiveness = g_navState.imuLiveness;

        if (g_navState.gen == genBefore)
        {
            out->gen = genBefore;
            ok = TRUE;
        }
    }

    return ok;
}
