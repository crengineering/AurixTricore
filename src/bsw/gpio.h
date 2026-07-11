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
} gpio_cfg_t;

typedef enum {
    GPIO_P_00_0 = 0, 
    GPIO_P_00_1, 
    /* new pins here */
    GPIO_P_00_END
} gpio_P_00_t;

typedef enum {
    GPIO_STATE_OFF = 0,
    GPIO_STATE_ON  = 1
} gpio_state_t;

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/
/* Pin level */
//void gpio_init(gpio_t *gpio, Ifx_P *port, uint8 pin);
//void gpio_write(gpio_t *gpio, boolean condition);
void init_gpio_pins(void);
void gpio_write(gpio_P_00_t pin_id, gpio_state_t state);

#endif /* LED_H */
