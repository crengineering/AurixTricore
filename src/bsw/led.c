/*
 *  Code from VibeCoding96
 *  Last modified: 12.06.2026
 *  Last Modified by: VibeCoding96
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "led.h"

/******************************************************************************/
/*--------------------------Function Implementations--------------------------*/
/******************************************************************************/

void Led_init(Led_t *led, Ifx_P *port, uint8 pin)
{
    led->port  = port;
    led->pin   = pin;
    led->state = IfxPort_State_high;   /* OFF on active-low board */

    IfxPort_setPinMode(port, pin, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinState(port, pin, IfxPort_State_high);
}

void Led_toggle(Led_t *led)
{
    led->state = (led->state == IfxPort_State_high)
               ? IfxPort_State_low
               : IfxPort_State_high;

    IfxPort_setPinState(led->port, led->pin, led->state);
}
