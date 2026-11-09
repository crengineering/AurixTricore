/*
 *  Code from crengineering
 *  Last modified: 09.11.2026
 *  Last Modified by: crengineering
 */

#ifndef DSHOTLATCH_H
#define DSHOTLATCH_H

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "Ifx_Types.h"
/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/
typedef struct
{
    uint32 gen;               /**< +1 per accepted new dshot frame (producer only)                */
    uint32 rpm[4];            /**< mechanical rpm = eRPM / pole pairs, last valid, 0 until first  */
    uint32 dshot_ok_count[4]; /**< valid bidir replies since boot; unchanged = this motor stale   */
    uint32 pad[1];            /**< 40 bytes: SharedRam.h rule 2, own 64-bit lines                 */
} Dshot_bidir_rpm;

extern volatile Dshot_bidir_rpm g_dshot_bidir_rpm;

#endif /* DSHOTLATCH_H */
