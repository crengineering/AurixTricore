/**********************************************************************************************************************
 * \file fusion.h
 *

 *********************************************************************************************************************/
#ifndef FUSION_H
#define FUSION_H

#include "Ifx_Types.h"



/*
 * global function
 */
void Fusion_init(void);
void Fusion_update(const float32 acc[3], float32 dt, boolean valid);


#endif /* FUSION_H */
