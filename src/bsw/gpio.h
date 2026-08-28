/*
 *  Code from VibeCoding96
 *  Last modified: 14.07.2026
 *  Last Modified by: VibeCoding96
 */

#ifndef GPIO_H
#define GPIO_H

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "Ifx_Types.h"
#include "IfxPort.h"
#include "Gtm/Tom/Pwm/IfxGtm_Tom_Pwm.h"

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

/* Compile-time operating mode of a pin. Configured per pin in gpio_cfg.h.
 * GPIO_MODE_RESERVED is never set in the config table (reserved pins use
 * userControllable = FALSE); it only appears in the published mode[] array
 * of the XCP block so clients can tell reserved pins apart. */
typedef enum {
    GPIO_MODE_DIGITAL  = 0,   /* static level via gpio_write / XCP state[]     */
    GPIO_MODE_PWM      = 1,   /* GTM TOM PWM; duty via XCP duty[], 0..100 %    */
    GPIO_MODE_RESERVED = 2    /* published only: firmware-owned pin            */
} gpio_mode_t;

typedef struct
{
    Ifx_P              *port;
    uint8               pin;
    IfxPort_State       state;            /**< initial/idle output level (digital) */
    boolean             userControllable; /**< TRUE: XCP may drive it; FALSE:
                                           *   reserved for firmware (protected)   */
    gpio_mode_t         mode;             /**< GPIO_MODE_DIGITAL or GPIO_MODE_PWM  */
    IfxGtm_Tom_ToutMap *tomPin;           /**< TOM channel routed to this pin
                                           *   (used only in PWM mode)             */
    float32             pwmFreqHz;        /**< PWM frequency (PWM mode only)       */
    uint8               pwmDutyInit;      /**< default duty %, preloaded into the
                                           *   XCP duty[] array (PWM mode only)    */
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
 * every pin to its idle level. Little-endian, 52 bytes:
 *
 *   0x00  uint32  magic          0x4F495047 ("GPIO")
 *   0x04  uint8   state[16]      per pin: 0 = OFF, 1 = ON  (writable)
 *   0x14  uint8   mode[16]       per pin: gpio_mode_t      (READ-ONLY,
 *                                published by firmware from gpio_cfg.h)
 *   0x24  uint8   duty[16]       per pin: PWM duty 0..100 %(writable,
 *                                effective only for pins in PWM mode)
 *
 * state[] keeps its pre-PWM offset (0x04) so existing A2L addresses stay
 * valid. The XCP slave permits writes only to state[] and duty[]; magic and
 * mode[] are protected (see xcpWriteAllowed in Xcp.c).
 *
 * Semantics per pin mode:
 *   DIGITAL: state 0/1 drives the pin low/high (as before); duty is ignored.
 *   PWM:     state 0 forces 0 % duty (output low); state 1 applies duty[].
 *            Duty updates are glitch-free (TOM shadow registers, take effect
 *            at the next PWM period).
 * A pin is driven from this block only if it is hardcoded user-controllable
 * (gpio_cfg_t.userControllable); reserved pins ignore it, so the user can
 * never drive them from XCP. Which pins are reserved is fixed in firmware,
 * NOT selectable over the protocol. */
/* XCP_GPIO_ADDR deleted, T5 (docs/MEMORY_PLACEMENT.md): Xcp.c's whitelist
 * now uses (uint32)&g_xcpGpio, so Lcf_Tasking_Tricore_Tc.lsl
 * (LCF_XCP_GPIO_START) is the only place 0x70030300 is written down. */
#define XCP_GPIO_MAGIC          0x4F495047u
#define XCP_GPIO_SIZE           52u  /* 4 + 3*16; must equal sizeof(Xcp_Gpio) */
#define XCP_GPIO_STATE_OFFSET   4u   /* writable: state[16]                   */
#define XCP_GPIO_MODE_OFFSET    20u  /* read-only: mode[16]                   */
#define XCP_GPIO_DUTY_OFFSET    36u  /* writable: duty[16]                    */

#define GPIO_PWM_DUTY_MAX       100u /* duty is a percentage                  */

typedef struct
{
    uint32 magic;
    uint8  state[GPIO_P_00_END];    /* desired level per pin: 0 = OFF, 1 = ON */
    uint8  mode[GPIO_P_00_END];     /* published gpio_mode_t per pin          */
    uint8  duty[GPIO_P_00_END];     /* desired PWM duty per pin, 0..100 %     */
} Xcp_Gpio;

extern volatile Xcp_Gpio g_xcpGpio;

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/
void init_gpio_pins(void);
void gpio_write(gpio_P_00_t pin_id, gpio_state_t state);
void gpio_calInit(void);    /* load defaults: magic, states OFF, modes, duties */
void gpio_calApply(void);   /* drive user-controllable pins from the block (periodic) */

#endif /* GPIO_H */
