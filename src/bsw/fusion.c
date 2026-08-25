/**********************************************************************************************************************
 * \file GnssM9N.c
 * \brief
 *********************************************************************************************************************/
#include "fusion.h"


static FusionValues g_fusion;


void Fusion_update(FusionValues *fusion, const float32 acc[3], float32 dt, boolean valid){
    g_fusion.a_D   = GRAVITY * (1.0f- acc[2]);
    g_fusion.a_v_d = g_fusion.a_v_d + g_fusion.a_D * dt;
    g_fusion.a_d   = g_fusion.a_d + g_fusion.a_v_d * dt + g_fusion.a_D * (dt / 2.0f);

    /* write to points */
    fusion->a_D   = g_fusion.a_D;
    fusion->a_v_d = g_fusion.a_v_d;
    fusion->a_d   = g_fusion.a_d;
}
