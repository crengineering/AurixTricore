/**********************************************************************************************************************
 * \file fusion.h
 *

 *********************************************************************************************************************/
#ifndef FUSION_H
#define FUSION_H

#include "Ifx_Types.h"


/* imu acc */
#define SIGMA_A 0.3f /* m/s2 */
#define GRAVITY 9.80665f

/* bmp */
#define SIGMA_BARO 0.0197 /* m */
#define R_BARO SIGMA_BARO * SIGMA_BARO


typedef struct
{
    float a_D;
    float a_d;
    float a_v_d;
} FusionValues;


/*
 * global function
 */
void Fusion_init(void);
void Fusion_update(FusionValues *fusion, const float32 acc[3], float32 dt, boolean valid);


#endif /* FUSION_H */
