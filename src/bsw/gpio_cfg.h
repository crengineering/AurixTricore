/*
 *  Code from VibeCoding96
 *  Last modified: 14.07.2026
 *  Last Modified by: VibeCoding96
 */

#ifndef GPIO_CFG_H
#define GPIO_CFG_H


/* ---------------------------------------------------------------------------
 * Per-pin GPIO configuration — EDIT HERE to reconfigure the P00 outputs.
 *
 * This header is included by gpio.c ONLY (it defines the config table).
 * Do not include it anywhere else.
 *
 * Each pin is either
 *   GPIO_MODE_DIGITAL: static high/low output, driven by gpio_write() or the
 *                      XCP state[] byte (5 V high side, 0 V low side), or
 *   GPIO_MODE_PWM:     hardware PWM from a GTM TOM channel; frequency is
 *                      fixed here at compile time, duty cycle is runtime-
 *                      adjustable via the XCP duty[] byte (0..100 %).
 *
 * userControllable = FALSE reserves a pin for firmware and blocks XCP from
 * driving it (P00.0 is the Diagnostics error output). Reserved pins are
 * always digital; a PWM entry on a reserved pin is ignored at init.
 *
 * TOM channel per pin: every P00 pin has exactly one entry here from the
 * TC39xB pin map, chosen so that all 16 assignments use distinct TOM
 * channels — any subset of pins may be switched to PWM without conflicts.
 * Do not change the tomPin column unless you know the alternate mapping
 * (see Libraries/.../IfxGtm_PinMap_TC39xB_516.h):
 *
 *   P00.0  TOM0 ch4    P00.4  TOM0 ch11   P00.8  TOM0 ch15   P00.12 TOM0 ch3
 *   P00.1  TOM0 ch9    P00.5  TOM0 ch12   P00.9  TOM0 ch0    P00.13 TOM3 ch12
 *   P00.2  TOM0 ch5    P00.6  TOM0 ch13   P00.10 TOM0 ch1    P00.14 TOM3 ch11
 *   P00.3  TOM0 ch10   P00.7  TOM0 ch14   P00.11 TOM0 ch2    P00.15 TOM3 ch13
 *
 * PWM frequency limits: 16-bit TOM counter on a CMU fixed clock
 * (fGTM / 16^n, n = 0..4). gpio.c picks the fastest clock whose period
 * fits, so roughly 0.1 Hz .. some MHz are reachable; resolution is best
 * at lower frequencies. pwmFreqHz/pwmDutyInit are ignored for digital pins.
 *
 * NOTE: the A2L (docs/AurixTricore.a2l) and the GUI read the resulting mode
 * over XCP, but the A2L duty entries are static — after changing modes here,
 * the duty rows for digital pins simply have no effect.
 * ------------------------------------------------------------------------ */

/*                     port         pin  idle state          user   mode               TOM output for PWM                 fPWM[Hz] duty%   */
#define GPIO_CFG_TABLE                                                                                                                      \
{                                                                                                                                           \
    /* cppcheck-suppress misra-c2012-11.4 ; deviation: iLLD MODULE_P00 SFR macro converts a fixed address to Ifx_P* */                      \
    [GPIO_P_00_0]  = { &MODULE_P00,  0u, IfxPort_State_low, FALSE, GPIO_MODE_DIGITAL, &IfxGtm_TOM0_4_TOUT9_P00_0_OUT,      0.0f,    0u },  \
    [GPIO_P_00_1]  = { &MODULE_P00,  1u, IfxPort_State_low, TRUE,  GPIO_MODE_PWM,     &IfxGtm_TOM0_9_TOUT10_P00_1_OUT,  1000.0f,   50u },  \
    [GPIO_P_00_2]  = { &MODULE_P00,  2u, IfxPort_State_low, TRUE,  GPIO_MODE_PWM,     &IfxGtm_TOM0_5_TOUT11_P00_2_OUT,     5.0f,   50u },  \
    [GPIO_P_00_3]  = { &MODULE_P00,  3u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM0_10_TOUT12_P00_3_OUT,    0.0f,    0u },  \
    [GPIO_P_00_4]  = { &MODULE_P00,  4u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM0_11_TOUT13_P00_4_OUT,    0.0f,    0u },  \
    [GPIO_P_00_5]  = { &MODULE_P00,  5u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM0_12_TOUT14_P00_5_OUT,    0.0f,    0u },  \
    [GPIO_P_00_6]  = { &MODULE_P00,  6u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM0_13_TOUT15_P00_6_OUT,    0.0f,    0u },  \
    [GPIO_P_00_7]  = { &MODULE_P00,  7u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM0_14_TOUT16_P00_7_OUT,    0.0f,    0u },  \
    [GPIO_P_00_8]  = { &MODULE_P00,  8u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM0_15_TOUT17_P00_8_OUT,    0.0f,    0u },  \
    [GPIO_P_00_9]  = { &MODULE_P00,  9u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM0_0_TOUT18_P00_9_OUT,     0.0f,    0u },  \
    [GPIO_P_00_10] = { &MODULE_P00, 10u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM0_1_TOUT19_P00_10_OUT,    0.0f,    0u },  \
    [GPIO_P_00_11] = { &MODULE_P00, 11u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM0_2_TOUT20_P00_11_OUT,    0.0f,    0u },  \
    [GPIO_P_00_12] = { &MODULE_P00, 12u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM0_3_TOUT21_P00_12_OUT,    0.0f,    0u },  \
    [GPIO_P_00_13] = { &MODULE_P00, 13u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM3_12_TOUT167_P00_13_OUT,  0.0f,    0u },  \
    [GPIO_P_00_14] = { &MODULE_P00, 14u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM3_11_TOUT166_P00_14_OUT,  0.0f,    0u },  \
    [GPIO_P_00_15] = { &MODULE_P00, 15u, IfxPort_State_low, TRUE,  GPIO_MODE_DIGITAL, &IfxGtm_TOM3_13_TOUT168_P00_15_OUT,  0.0f,    0u }   \
}

#endif /* GPIO_CFG_H */
