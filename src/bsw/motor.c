/*
 *  Code from crengineering
 *  Last modified: 22.09.2026
 *  Last Modified by: crengineering
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "motor.h"
#include "Xcp.h"
#include "Diagnostics.h"

/* Dshot protocol comments from betaflight */
#define DSHOT_CMD_MOTOR_STOP      0u
#define DSHOT_CMD_BEEP1           1u
#define DSHOT_CMD_BEEP2           2u
#define DSHOT_CMD_BEEP3           3u
#define DSHOT_CMD_BEEP4           4u
#define DSHOT_CMD_BEEP5           5u
#define DSHOT_CMD_THROTTLE_MIN   48u
#define DSHOT_CMD_THROTTLE_MAX 2047u

#define DSHOT_BEEP_DELAY 1000u  /* 1s between beeps */


/******************************************************************************/
/*--------------------------Function Declaration------------------------------*/
/******************************************************************************/
static boolean Motor_beep(uint16 dshot_command_set[DSHOT_MEND]);
void           Motor_calc_clamp(const uint16 motor_speed_rqst[DSHOT_MEND], uint16 dshot_command_set[DSHOT_MEND]);
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

void Motor_calc_clamp(const uint16 motor_speed_rqst[DSHOT_MEND], uint16 dshot_command_set[DSHOT_MEND])
{
    for (Dshot_Motor_t motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
    {
        if (motor_speed_rqst[motor_id] > 0u)
        {
            if (motor_speed_rqst[motor_id] < DSHOT_CMD_THROTTLE_MIN)
            {
                dshot_command_set[motor_id] = DSHOT_CMD_THROTTLE_MIN;
            }
            else if (motor_speed_rqst[motor_id] > DSHOT_CMD_THROTTLE_MAX)
            {
                dshot_command_set[motor_id] = DSHOT_CMD_THROTTLE_MAX;
            }
            else
            {
                dshot_command_set[motor_id] = motor_speed_rqst[motor_id];
            }
        }
    }
}

Motor_states_t Motor_task(const uint16 motor_speed_rqst[DSHOT_MEND], uint16 dshot_command_set[DSHOT_MEND])
{
    static Motor_states_t state                         = MOTOR_INIT;
           boolean        eth_link_alive                = Xcp_linkAlive();
           uint16         setpoint[DSHOT_MEND]   = {0};
    // general transition for failsafe enter
    if (eth_link_alive == FALSE)
    {
        state = MOTOR_FAILSAFE;
    }

    switch(state)
    {
        case MOTOR_INIT:
            // add a check for valid telemetrics packets received
            // tbd do only when esc gives feedback
            if (Motor_beep(dshot_command_set) != FALSE)
            {
              state = MOTOR_DISARMED;
            }
            break;
        case MOTOR_DISARMED:
            if (g_xcpCal.motorCmd == MOTOR_CMD_ARM)
            {
                state = MOTOR_ARMED;
            }
            break;
        case MOTOR_ARMED:
            if (g_xcpCal.motorCmd != MOTOR_CMD_ARM)
            {
                state = MOTOR_DISARMED;
                g_xcpCal.motorManual = 0u;
                for (Dshot_Motor_t motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
                {
                    g_xcpCal.motorManualSp[motor_id] = 0u;
                }
            }

            if(g_xcpCal.motorManual == MOTOR_CMD_MANUAL)
            {
                for (Dshot_Motor_t motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
                {
                    setpoint[motor_id] = g_xcpCal.motorManualSp[motor_id];
                }
            }
            else
            {
                for (Dshot_Motor_t motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
                {
                    setpoint[motor_id] = motor_speed_rqst[motor_id];
                }
            }


            Motor_calc_clamp( setpoint, dshot_command_set);

            break;
        case MOTOR_FAILSAFE:
            g_xcpCal.motorCmd    = 0u;
            g_xcpCal.motorManual = 0u;
            for (Dshot_Motor_t motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
            {
                g_xcpCal.motorManualSp[motor_id] = 0u;
            }
            if (eth_link_alive != FALSE)
            {
                state = MOTOR_INIT;
            }
            break;
        default:
            break;
    }

    return state;
}
