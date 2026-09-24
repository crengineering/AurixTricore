/*
 *  Code from crengineering
 *  Last modified: 22.09.2026
 *  Last Modified by: crengineering
 */

#ifndef MOTOR_H
#define MOTOR_H

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "Ifx_Types.h"
#include "IfxPort.h"
#include "Dshot.h"
/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

typedef enum {
    MOTOR_INIT = 0,
    MOTOR_DISARMED,
    MOTOR_ARMED,
    MOTOR_FAILSAFE,
    MOTOR_END
} Motor_states_t;
/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

Motor_states_t Motor_task(const uint16 motor_speed_rqst[DSHOT_MEND], uint16 dshot_command_set[DSHOT_MEND]);

#endif /* MOTOR_H */
