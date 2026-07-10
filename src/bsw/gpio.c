/*
 *  Code from VibeCoding96
 *  Last modified: 10.07.2026
 *  Last Modified by: VibeCoding96
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "gpio.h"

/******************************************************************************/
/*--------------------------Function Implementations--------------------------*/
/******************************************************************************/

void gpio_init(gpio_t *gpio, Ifx_P *port, uint8 pin)
{
    gpio->port  = port;
    gpio->pin   = pin;
    gpio->state = IfxPort_State_high;   /* OFF on active-low board */

    IfxPort_setPinMode(port, pin, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinState(port, pin, IfxPort_State_high);
}

void gpio_toggle(gpio_t *gpio)
{
    gpio->state = IfxPort_State_high;

    IfxPort_setPinState(gpio->port, gpio->pin, gpio->state);
}
