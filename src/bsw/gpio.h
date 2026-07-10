/*
 *  Code from VibeCoding96
 *  Last modified: 10.07.2026
 *  Last Modified by: VibeCoding96
 */

#ifndef GPIO_H
#define GPIO_H

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "Ifx_Types.h"
#include "IfxPort.h"

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

typedef struct
{
    Ifx_P         *port;
    uint8          pin;
    IfxPort_State  state;   /**< Current output state (high = off, low = on) */
} gpio_t;

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/
/* Pin level */
void gpio_init(gpio_t *gpio, Ifx_P *port, uint8 pin);
void gpio_toggle(gpio_t *gpio, boolean condition);
void init_gpio_pins(void);
void toggle_gpio_pins(boolean error_active);

#endif /* LED_H */
