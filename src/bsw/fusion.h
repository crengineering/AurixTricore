/**********************************************************************************************************************
 * \file fusion.h
 *

 *********************************************************************************************************************/
#ifndef FUSION_H
#define FUSION_H

#include "Ifx_Types.h"



#define SIGMA_A 0.3f /* m/s2 */
#define GRAVITY 9.80665f


typedef struct
{
    float a_D;
} FusionValues;


/*
 * global function
 */
void Fusion_init(void);
void Fusion_update(FusionValues *fusion, const float32 acc[3], float32 dt, boolean valid);


#endif /* FUSION_H */
