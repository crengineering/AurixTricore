/**********************************************************************************************************************
 * \file GnssM9N.c
 * \brief
 *********************************************************************************************************************/
#include "fusion.h"


static FusionValues g_fusion;

void Fusion_update(FusionValues *fusion, const float32 acc[3], float32 dt, boolean valid){
    g_fusion.a_D = GRAVITY * (1.0f- acc[2]);



    /* write to points */
    fusion->a_D = g_fusion.a_D;
}
