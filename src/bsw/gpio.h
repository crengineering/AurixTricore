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
    IfxPort_State  state;            /**< initial/idle output level          */
    boolean        userControllable; /**< TRUE: XCP may drive it; FALSE:
                                       *   reserved for firmware (protected)  */
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
/*----------------------- XCP GPIO control block -----------------------------*/
/******************************************************************************/
/* Calibration block for driving the P00 outputs from an XCP master (GUI).
 * Fixed address so the master needs no map file; RAM only, so a reset returns
 * every pin to its idle level. Little-endian, 20 bytes:
 *
 *   0x00  uint32  magic          0x4F495047 ("GPIO")
 *   0x04  uint8   state[16]      per pin: 0 = OFF, 1 = ON
 *
 * A pin's state is applied only if that pin is hardcoded user-controllable
 * (gpio_cfg_t.userControllable); reserved pins ignore it, so the user can
 * never drive them from XCP. Which pins are reserved is fixed in firmware,
 * NOT selectable over the protocol. */
#define XCP_GPIO_ADDR   0x70030300u
#define XCP_GPIO_MAGIC  0x4F495047u
#define XCP_GPIO_SIZE   20u          /* 4 + 16; must equal sizeof(Xcp_Gpio) */

typedef struct
{
    uint32 magic;
    uint8  state[GPIO_P_00_END];    /* desired level per pin: 0 = OFF, 1 = ON */
} Xcp_Gpio;

extern volatile Xcp_Gpio g_xcpGpio;

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/
/* Pin level */
//void gpio_init(gpio_t *gpio, Ifx_P *port, uint8 pin);
//void gpio_write(gpio_t *gpio, boolean condition);
void init_gpio_pins(void);
void gpio_write(gpio_P_00_t pin_id, gpio_state_t state);
void gpio_calInit(void);    /* load defaults: magic + all states OFF                */
void gpio_calApply(void);   /* drive user-controllable pins from the block (periodic) */

#endif /* LED_H */
