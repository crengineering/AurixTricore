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
/* userControllable = FALSE reserves a pin for firmware and blocks XCP from
 * driving it. P00.0 is the Diagnostics error output, so it is reserved; add
 * any other in-use pin here the same way. */
static const gpio_cfg_t g_gpio_cfg[GPIO_P_00_END] =
{
    [GPIO_P_00_0]  = { &MODULE_P00,  0u, IfxPort_State_low, FALSE }, /* diagnostics */
    [GPIO_P_00_1]  = { &MODULE_P00,  1u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_2]  = { &MODULE_P00,  2u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_3]  = { &MODULE_P00,  3u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_4]  = { &MODULE_P00,  4u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_5]  = { &MODULE_P00,  5u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_6]  = { &MODULE_P00,  6u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_7]  = { &MODULE_P00,  7u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_8]  = { &MODULE_P00,  8u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_9]  = { &MODULE_P00,  9u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_10] = { &MODULE_P00, 10u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_11] = { &MODULE_P00, 11u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_12] = { &MODULE_P00, 12u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_13] = { &MODULE_P00, 13u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_14] = { &MODULE_P00, 14u, IfxPort_State_low, TRUE  },
    [GPIO_P_00_15] = { &MODULE_P00, 15u, IfxPort_State_low, TRUE  }
};

/* XCP control block at a fixed address (see gpio.h). RAM only: a reset
 * returns every pin to firmware control. The XCP slave only permits writes
 * inside this block, skipping the magic word. */
volatile Xcp_Gpio g_xcpGpio __at(XCP_GPIO_ADDR);
/******************************************************************************/
/*--------------------------Local Function Implementations--------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------- Global Function Implementations------------------*/
/******************************************************************************/
void init_gpio_pins(void)
{
    uint32 pin_id;
    for (pin_id = 0u; pin_id < (uint32)GPIO_P_00_END; pin_id++){
        IfxPort_setPinState(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin, g_gpio_cfg[pin_id].state);
        IfxPort_setPinMode(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin, IfxPort_Mode_outputPushPullGeneral);
    }
}

void gpio_write(gpio_P_00_t pin_id, gpio_state_t state)
{
    /* bounds check: never index past the config table */
    if (pin_id < GPIO_P_00_END) {
        if (state == GPIO_STATE_ON) {
            IfxPort_setPinState(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin, IfxPort_State_high);
        } else {
            IfxPort_setPinState(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin, IfxPort_State_low);
        }
    }
}

/* Load default XCP control state: valid magic, every pin state OFF. */
void gpio_calInit(void)
{
    uint32 i;

    g_xcpGpio.magic = XCP_GPIO_MAGIC;
    for (i = 0u; i < (uint32)GPIO_P_00_END; i++) {
        g_xcpGpio.state[i] = 0u;
    }
}

/* Apply the XCP control block to the pins. Call periodically. Only pins that
 * are hardcoded user-controllable are driven; reserved pins (e.g. the
 * Diagnostics output on P00.0) are never touched, so the user cannot override
 * them over XCP. A corrupted block reverts to defaults. */
void gpio_calApply(void)
{
    uint32 i;

    if (g_xcpGpio.magic != XCP_GPIO_MAGIC) {
        gpio_calInit();
    } else {
        for (i = 0u; i < (uint32)GPIO_P_00_END; i++) {
            if (g_gpio_cfg[i].userControllable != FALSE) {
                gpio_write((gpio_P_00_t)i,
                           (g_xcpGpio.state[i] != 0u) ? GPIO_STATE_ON : GPIO_STATE_OFF);
            }
        }
    }
}