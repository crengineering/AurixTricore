/**********************************************************************************************************************
 * \file CoreStats.c
 * \brief Per-core execution-time accounting — see CoreStats.h.
 *********************************************************************************************************************/
#include "CoreStats.h"

volatile CoreStats_t g_coreStats[CORESTATS_NUM_CORES];

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
