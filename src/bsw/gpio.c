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
static const gpio_cfg_t g_gpio_cfg[GPIO_P_00_END] =
{
    [GPIO_P_00_0]  = { &MODULE_P00,  0u, IfxPort_State_low },
    [GPIO_P_00_1]  = { &MODULE_P00,  1u, IfxPort_State_low },
    [GPIO_P_00_2]  = { &MODULE_P00,  2u, IfxPort_State_low },
    [GPIO_P_00_3]  = { &MODULE_P00,  3u, IfxPort_State_low },
    [GPIO_P_00_4]  = { &MODULE_P00,  4u, IfxPort_State_low },
    [GPIO_P_00_5]  = { &MODULE_P00,  5u, IfxPort_State_low },
    [GPIO_P_00_6]  = { &MODULE_P00,  6u, IfxPort_State_low },
    [GPIO_P_00_7]  = { &MODULE_P00,  7u, IfxPort_State_low },
    [GPIO_P_00_8]  = { &MODULE_P00,  8u, IfxPort_State_low },
    [GPIO_P_00_9]  = { &MODULE_P00,  9u, IfxPort_State_low },
    [GPIO_P_00_10] = { &MODULE_P00, 10u, IfxPort_State_low },
    [GPIO_P_00_11] = { &MODULE_P00, 11u, IfxPort_State_low },
    [GPIO_P_00_12] = { &MODULE_P00, 12u, IfxPort_State_low },
    [GPIO_P_00_13] = { &MODULE_P00, 13u, IfxPort_State_low },
    [GPIO_P_00_14] = { &MODULE_P00, 14u, IfxPort_State_low },
    [GPIO_P_00_15] = { &MODULE_P00, 15u, IfxPort_State_low }
};

static gpio_state_t g_gpio_state[GPIO_P_00_END];
/******************************************************************************/
/*--------------------------Local Function Implementations--------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------- Global Function Implementations------------------*/
/******************************************************************************/
void init_gpio_pins(void)
{
    uint32 pin_id;
    for (pin_id = 0u; pin_id < GPIO_P_00_END; pin_id++){
        IfxPort_setPinState(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin, g_gpio_cfg[pin_id].state);
        IfxPort_setPinMode(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin, IfxPort_Mode_outputPushPullGeneral);
    }
}

void gpio_write(gpio_P_00_t pin_id, gpio_state_t state)
{
    if (pin_id >= GPIO_P_00_END) {
        return;   /* out-of-range handle: ignore, never index past the table */
    }

    if (state == GPIO_STATE_ON) {
        IfxPort_setPinState(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin, IfxPort_State_high);
    } else {
        IfxPort_setPinState(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin, IfxPort_State_low);
    }
    g_gpio_state[pin_id] = state;
}