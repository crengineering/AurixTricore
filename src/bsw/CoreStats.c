/**********************************************************************************************************************
 * \file CoreStats.c
 * \brief Per-core execution-time accounting — see CoreStats.h.
 *
 * g_coreStats itself is defined in SharedRam.c (LMU shared block, `#pragma
 * section`, docs/MEMORY_PLACEMENT.md T3) -- not here. This file only owns
 * the logic that touches it.
 *********************************************************************************************************************/
#include "CoreStats.h"

void CoreStats_init(uint8 coreId)
{
    if (coreId < CORESTATS_NUM_CORES)
    {
        g_coreStats[coreId].execUs       = 0u;
        g_coreStats[coreId].execMaxUs    = 0u;
        g_coreStats[coreId].loadPmil     = 0u;
        g_coreStats[coreId].aliveCounter = 0u;
    }
}
