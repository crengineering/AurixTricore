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
/* X2216-III 12N14P: 14 poles = 7 pole pairs; eRPM / MOTOR_POLE_PAIRS = mechanical rpm.
 * A literal, not (14u / 2u): the float casts in Measurements.c hit MISRA 10.8 on a
 * composite expression, and a separate MOTOR_POLES would be unused (2.5). */
#define MOTOR_POLE_PAIRS  7u


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
