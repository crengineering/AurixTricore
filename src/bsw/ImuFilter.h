/*
 *  Code from crengineering
 *  Last modified: 09.11.2026
 *  Last Modified by: crengineering
 */

#ifndef IMUFILTER_H
#define IMUFILTER_H

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "Ifx_Types.h"
#include "Icm42688.h"

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/


/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/
void ImuFilter_step(Icm42688_Sample *imu_sample);
/* Call once before the first ImuFilter_step() after a gap (sensor dropout, no accepted
 * sample): that step passes the sample through untouched and restarts every stage from it. */
void ImuFilter_restart(void);

#endif /* IMUFILTER_H */
