/*
 *  Code from VibeCoding96
 *  Last modified: 14.07.2026
 *  Last Modified by: VibeCoding96
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "gpio.h"
#include "gpio_cfg.h"
/******************************************************************************/
/*-----------------------------------Defines----------------------------------*/
/******************************************************************************/
/* TOM counters are 16 bit; a PWM period must fit and leave at least one
 * tick of resolution for the duty compare. */
#define GPIO_PWM_PERIOD_MIN     2.0f
#define GPIO_PWM_PERIOD_MAX     65535.0f
#define GPIO_PWM_CLOCK_COUNT    5u
/******************************************************************************/
/*----------------------------------Variables---------------------------------*/
/******************************************************************************/
/* Per-pin configuration — edit gpio_cfg.h, not this table. */
static const gpio_cfg_t g_gpio_cfg[GPIO_P_00_END] = GPIO_CFG_TABLE;

/* Effective mode after init, published in g_xcpGpio.mode[]. Differs from the
 * configured mode if a PWM setup failed (frequency not reachable) — the pin
 * then falls back to a digital output — or for reserved pins. */
static uint8 s_gpioEffMode[GPIO_P_00_END];

/* TOM driver handle and period (in TOM ticks) per PWM pin */
static IfxGtm_Tom_Pwm_Driver s_gpioPwmDrv[GPIO_P_00_END];
static uint16                s_gpioPwmPeriod[GPIO_P_00_END];

/* XCP control block at a fixed address (see gpio.h). RAM only: a reset
 * returns every pin to firmware control. The XCP slave only permits writes
 * to the state[] and duty[] arrays (magic and mode[] are protected). Off
 * __at() onto `#pragma section` (docs/MEMORY_PLACEMENT.md T4) -- an absolute
 * group at the unchanged LCF_XCP_GPIO_START literal. */
#pragma section farbss "xcp_gpio"
volatile Xcp_Gpio g_xcpGpio;
#pragma section farbss restore
/******************************************************************************/
/*--------------------------Local Function Implementations--------------------*/
/******************************************************************************/
/* Pick the fastest CMU fixed clock whose 16-bit period covers the requested
 * PWM frequency. FXCLK(n) = fGTM / 16^n, so each step trades resolution for
 * a 16x lower reachable frequency. */
static boolean gpioPwmSelectClock(float32 freqHz, IfxGtm_Tom_Ch_ClkSrc *clkSrc,
                                  uint16 *periodTicks)
{
    static const IfxGtm_Tom_Ch_ClkSrc clkSrcTab[GPIO_PWM_CLOCK_COUNT] = {
        IfxGtm_Tom_Ch_ClkSrc_cmuFxclk0, IfxGtm_Tom_Ch_ClkSrc_cmuFxclk1,
        IfxGtm_Tom_Ch_ClkSrc_cmuFxclk2, IfxGtm_Tom_Ch_ClkSrc_cmuFxclk3,
        IfxGtm_Tom_Ch_ClkSrc_cmuFxclk4
    };
    static const IfxGtm_Cmu_Fxclk fxClkTab[GPIO_PWM_CLOCK_COUNT] = {
        IfxGtm_Cmu_Fxclk_0, IfxGtm_Cmu_Fxclk_1, IfxGtm_Cmu_Fxclk_2,
        IfxGtm_Cmu_Fxclk_3, IfxGtm_Cmu_Fxclk_4
    };
    boolean found = FALSE;
    uint32  i;

    if (freqHz > 0.0f) {
        for (i = 0u; (i < GPIO_PWM_CLOCK_COUNT) && (found == FALSE); i++) {
            float32 fClk  = IfxGtm_Cmu_getFxClkFrequency(&MODULE_GTM, fxClkTab[i], TRUE);
            float32 ticks = fClk / freqHz;

            if ((ticks >= GPIO_PWM_PERIOD_MIN) && (ticks <= GPIO_PWM_PERIOD_MAX)) {
                *clkSrc      = clkSrcTab[i];
                *periodTicks = (uint16)ticks;
                found        = TRUE;
            }
        }
    }

    return found;
}

/* Configure one pin's TOM channel for PWM and route it to the pin. The
 * channel starts with 0 % duty (output low) until the XCP block enables it.
 * Returns FALSE if no TOM clock reaches the requested frequency. */
static boolean gpioPwmInitPin(uint32 pin_id)
{
    IfxGtm_Tom_Ch_ClkSrc clkSrc      = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk0;
    uint16               periodTicks = 0u;
    boolean              ok          = FALSE;

    if ((g_gpio_cfg[pin_id].tomPin != NULL_PTR)
        && (gpioPwmSelectClock(g_gpio_cfg[pin_id].pwmFreqHz, &clkSrc, &periodTicks) != FALSE)) {
        IfxGtm_Tom_Pwm_Config pwmCfg;

        IfxGtm_Tom_Pwm_initConfig(&pwmCfg, &MODULE_GTM);
        pwmCfg.tom                      = g_gpio_cfg[pin_id].tomPin->tom;
        pwmCfg.tomChannel               = g_gpio_cfg[pin_id].tomPin->channel;
        pwmCfg.clock                    = clkSrc;
        pwmCfg.period                   = periodTicks;
        pwmCfg.dutyCycle                = 0u;   /* off until enabled via XCP */
        pwmCfg.signalLevel              = Ifx_ActiveState_high;
        pwmCfg.synchronousUpdateEnabled = TRUE; /* duty via shadow registers */
        pwmCfg.immediateStartEnabled    = TRUE;
        pwmCfg.pin.outputPin            = g_gpio_cfg[pin_id].tomPin;
        pwmCfg.pin.outputMode           = IfxPort_OutputMode_pushPull;
        pwmCfg.pin.padDriver            = IfxPort_PadDriver_cmosAutomotiveSpeed1;

        (void)IfxGtm_Tom_Pwm_init(&s_gpioPwmDrv[pin_id], &pwmCfg);
        s_gpioPwmPeriod[pin_id] = periodTicks;
        ok = TRUE;
    }

    return ok;
}

/* Write the duty compare shadow register; the TOM transfers it into the
 * active compare register at the next period boundary (glitch-free).
 * 0 % keeps the output constantly low, 100 % constantly high. */
static void gpioPwmSetDuty(uint32 pin_id, uint8 dutyPercent)
{
    uint8  duty = dutyPercent;
    uint32 ticks;

    if (duty > (uint8)GPIO_PWM_DUTY_MAX) {
        duty = (uint8)GPIO_PWM_DUTY_MAX;
    }
    ticks = ((uint32)s_gpioPwmPeriod[pin_id] * (uint32)duty) / GPIO_PWM_DUTY_MAX;

    IfxGtm_Tom_Ch_setCompareOneShadow(s_gpioPwmDrv[pin_id].tom,
                                      s_gpioPwmDrv[pin_id].tomChannel,
                                      (uint16)ticks);
}
/******************************************************************************/
/*-------------------------- Global Function Implementations------------------*/
/******************************************************************************/
void init_gpio_pins(void)
{
    uint32  pin_id;
    boolean anyPwm = FALSE;

    for (pin_id = 0u; pin_id < (uint32)GPIO_P_00_END; pin_id++) {
        if ((g_gpio_cfg[pin_id].mode == GPIO_MODE_PWM)
            && (g_gpio_cfg[pin_id].userControllable != FALSE)) {
            anyPwm = TRUE;
        }
    }

    /* one-time GTM bring-up: module clock + CMU fixed clocks for the TOMs */
    if (anyPwm != FALSE) {
        Ifx_GTM *gtm = &MODULE_GTM;

        IfxGtm_enable(gtm);
        IfxGtm_Cmu_setGclkFrequency(gtm, IfxGtm_Cmu_getModuleFrequency(gtm));
        IfxGtm_Cmu_enableClocks(gtm, IFXGTM_CMU_CLKEN_FXCLK);
    }

    for (pin_id = 0u; pin_id < (uint32)GPIO_P_00_END; pin_id++) {
        boolean pwmActive = FALSE;

        /* reserved pins are always plain digital outputs */
        if ((g_gpio_cfg[pin_id].mode == GPIO_MODE_PWM)
            && (g_gpio_cfg[pin_id].userControllable != FALSE)) {
            pwmActive = gpioPwmInitPin(pin_id);
        }

        if (pwmActive != FALSE) {
            s_gpioEffMode[pin_id] = (uint8)GPIO_MODE_PWM;
        } else {
            s_gpioEffMode[pin_id] = (g_gpio_cfg[pin_id].userControllable != FALSE)
                                    ? (uint8)GPIO_MODE_DIGITAL
                                    : (uint8)GPIO_MODE_RESERVED;
            IfxPort_setPinState(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin,
                                g_gpio_cfg[pin_id].state);
            IfxPort_setPinMode(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin,
                               IfxPort_Mode_outputPushPullGeneral);
        }
    }
}

void gpio_write(gpio_P_00_t pin_id, gpio_state_t state)
{
    /* bounds check: never index past the config table; PWM pins are owned
     * by their TOM channel and cannot be driven via the port registers */
    if ((pin_id < GPIO_P_00_END) && (s_gpioEffMode[pin_id] != (uint8)GPIO_MODE_PWM)) {
        if (state == GPIO_STATE_ON) {
            IfxPort_setPinState(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin, IfxPort_State_high);
        } else {
            IfxPort_setPinState(g_gpio_cfg[pin_id].port, g_gpio_cfg[pin_id].pin, IfxPort_State_low);
        }
    }
}

/* Load default XCP control state: valid magic, every pin OFF, modes as
 * configured (effective after init), duties preloaded from the config. */
void gpio_calInit(void)
{
    uint32 i;

    g_xcpGpio.magic = XCP_GPIO_MAGIC;
    for (i = 0u; i < (uint32)GPIO_P_00_END; i++) {
        g_xcpGpio.state[i] = 0u;
        g_xcpGpio.mode[i]  = s_gpioEffMode[i];
        g_xcpGpio.duty[i]  = (s_gpioEffMode[i] == (uint8)GPIO_MODE_PWM)
                             ? g_gpio_cfg[i].pwmDutyInit : 0u;
    }
}

/* Apply the XCP control block to the pins. Call periodically. Only pins that
 * are hardcoded user-controllable are driven; reserved pins (e.g. the
 * Diagnostics output on P00.0) are never touched, so the user cannot override
 * them over XCP. Digital pins follow state[]; PWM pins run at duty[] percent
 * while state[] is ON and at 0 % (output low) while OFF. A corrupted block
 * reverts to defaults. */
void gpio_calApply(void)
{
    uint32 i;

    if (g_xcpGpio.magic != XCP_GPIO_MAGIC) {
        gpio_calInit();
    } else {
        for (i = 0u; i < (uint32)GPIO_P_00_END; i++) {
            /* mode[] is read-only for clients: re-publish every cycle */
            g_xcpGpio.mode[i] = s_gpioEffMode[i];

            if (g_gpio_cfg[i].userControllable != FALSE) {
                if (s_gpioEffMode[i] == (uint8)GPIO_MODE_PWM) {
                    uint8 duty = (g_xcpGpio.state[i] != 0u) ? g_xcpGpio.duty[i] : 0u;
                    gpioPwmSetDuty(i, duty);
                } else {
                    gpio_write((gpio_P_00_t)i,
                               (g_xcpGpio.state[i] != 0u) ? GPIO_STATE_ON : GPIO_STATE_OFF);
                }
            }
        }
    }
}
