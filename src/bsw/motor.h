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
#define MOTOR_POLES       14u                    /* X2216-III 12N14P */
#define MOTOR_POLE_PAIRS  (MOTOR_POLES / 2u)     /* 7, eRPM -> mechanical rpm */


typedef enum {
    MOTOR_DISARMED = 0,
    MOTOR_INIT,
    MOTOR_ARMED,
    MOTOR_FAILSAFE,
    MOTOR_END
} Motor_states_t;

typedef enum {
    MOTOR_SETPOINT_MODE_THROTTLE = 0,
    MOTOR_SETPOINT_MODE_SPEED,
    MOTOR_SETPOINT_MODE_END
} Motor_setpoint_mode_t;
/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

boolean Motor_task(const uint16 motor_speed_rqst[DSHOT_MEND], uint16 dshot_command_set[DSHOT_MEND], Motor_states_t *state);

#endif /* MOTOR_H */
