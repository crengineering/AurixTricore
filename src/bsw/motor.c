/*
 *  Code from crengineering
 *  Last modified: 22.09.2026
 *  Last Modified by: crengineering
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "motor.h"

/* Dshot protocol comments from betaflight */
#define DSHOT_CMD_MOTOR_STOP 0u
#define DSHOT_CMD_BEEP1      1u
#define DSHOT_CMD_BEEP2      2u
#define DSHOT_CMD_BEEP3      3u
#define DSHOT_CMD_BEEP4      4u
#define DSHOT_CMD_BEEP5      5u

#define DSHOT_BEEP_DELAY 1000u  /* 1s between beeps */


/******************************************************************************/
/*--------------------------Function Declaration------------------------------*/
/******************************************************************************/
static boolean Motor_beep(uint16 dshot_command_set[DSHOT_MEND]);

/******************************************************************************/
/*--------------------------Interrupts----------------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*--------------------------Function Implementations--------------------------*/
/******************************************************************************/
static boolean Motor_beep(uint16 dshot_command_set[DSHOT_MEND])
{
    static boolean motor_beep[DSHOT_MEND] = {FALSE};
    static uint16  motor_beep_counter     = 0u;
    static uint8   motor_id               = 0u;
           boolean motor_beep_finished    = FALSE;

    if ( (motor_beep[motor_id] == FALSE)                                 &&
         (motor_beep_counter   >= (DSHOT_BEEP_DELAY + ((uint8) motor_id ) * DSHOT_BEEP_DELAY ))  )
    {
        dshot_command_set[motor_id] = DSHOT_CMD_BEEP1;
        motor_beep[motor_id] = TRUE;
    }
    else
    {
        dshot_command_set[motor_id] = DSHOT_CMD_MOTOR_STOP;
        if (motor_beep[motor_id] != FALSE)
        {
            if (++motor_id >= DSHOT_MEND)
            {
                motor_id = 0u;
                motor_beep[DSHOT_M1] = motor_beep[DSHOT_M2] = motor_beep[DSHOT_M3] = motor_beep[DSHOT_M4] = FALSE;
                motor_beep_counter = 0u;
                motor_beep_finished = TRUE;
            }
        }
    }
    motor_beep_counter++;
    return motor_beep_finished;
}



Motor_states_t Motor_task(const uint16 motor_speed_rqst[DSHOT_MEND], uint16 dshot_command_set[DSHOT_MEND])
{
    static Motor_states_t state                         = MOTOR_INIT;

    // general transition for failsafe enter
    // if ethernet communication is lost then state = MOTOR_FAILSAFE;

    switch(state)
    {
        case MOTOR_INIT:
            // add a check for valid telemetrics packets received
            if (Motor_beep(dshot_command_set) != FALSE)
            {
              state = MOTOR_DISARMED;
            }
            break;
        case MOTOR_DISARMED:

            // leave state to ARMED if disarm via XCP is received to ARM

            break;
        case MOTOR_ARMED:
            for (Dshot_Motor_t motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
            {
                dshot_command_set[motor_id] = motor_speed_rqst[motor_id];
            }
            break;
        case MOTOR_FAILSAFE:
             // if communication is established back again, then leave to init
            break;
        default:
            break;
    }

    return state;
}
