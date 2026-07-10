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
/*----------------------------------Variables---------------------------------*/
/******************************************************************************/
static gpio_t      g_gpio_P00_0;
static gpio_t      g_gpio_P00_1;

/******************************************************************************/
/*--------------------------Local Function Implementations--------------------*/
/******************************************************************************/

void gpio_init(gpio_t *gpio, Ifx_P *port, uint8 pin)
{
    gpio->port  = port;
    gpio->pin   = pin;
    gpio->state = IfxPort_State_low;   /* OFF on active-low board */

    IfxPort_setPinMode(port, pin, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinState(port, pin, IfxPort_State_low);
}

void gpio_toggle(gpio_t *gpio, boolean condition)
{
    if (condition) {
        gpio->state = IfxPort_State_high;
    } else {
        gpio->state = IfxPort_State_low;
    }
    
    IfxPort_setPinState(gpio->port, gpio->pin, gpio->state);
}

/******************************************************************************/
/*-------------------------- Global Function Implementations------------------*/
/******************************************************************************/

/* init Pins*/
void init_gpio_pins(void){

    /* Module P00*/
    gpio_init(&g_gpio_P00_0, &MODULE_P00, 0u); /* Pin 0 */
    gpio_init(&g_gpio_P00_1, &MODULE_P00, 1u); /* Pin 1 */
}

void toggle_gpio_pins(boolean error_active){

    /* Module P00 */
    gpio_toggle(&g_gpio_P00_0, error_active); /* Pin 0 */
    gpio_toggle(&g_gpio_P00_1, TRUE);         /* Pin 1 */
}