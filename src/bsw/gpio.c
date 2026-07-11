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
    [GPIO_P_00_0]  = { &MODULE_P00, 0u, IfxPort_State_low  },
    [GPIO_P_00_1]  = { &MODULE_P00, 1u, IfxPort_State_low  }
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
    if (state == GPIO_STATE_ON) {
        IfxPort_setPinState(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin, IfxPort_State_high);
    } else {
        IfxPort_setPinState(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin, IfxPort_State_low);
    }
    g_gpio_state[pin_id] = state;
}