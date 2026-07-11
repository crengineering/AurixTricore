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
    GPIO_P_00_2,
    GPIO_P_00_3,
    GPIO_P_00_4,
    GPIO_P_00_5,
    GPIO_P_00_6,
    GPIO_P_00_7,
    GPIO_P_00_8,
    GPIO_P_00_9,
    GPIO_P_00_10,
    GPIO_P_00_11,
    GPIO_P_00_12,
    GPIO_P_00_13,
    GPIO_P_00_14,
    GPIO_P_00_15,
    GPIO_P_00_END        /* == number of configured pins (16) */
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
