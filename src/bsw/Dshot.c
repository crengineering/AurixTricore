/*
 *  Code from crengineering
 *  Last modified: 16.09.2026
 *  Last Modified by: crengineering
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "Dshot.h"
#include "IfxGtm_Atom_Pwm.h"
#include "IfxGtm_PinMap.h"
#include "IfxCpu.h"

static IfxGtm_Atom_Pwm_Driver g_atomM1;
static uint32                 g_t0hTicks;
static uint32                 g_t1hTicks;
uint32                        g_dshotFrames;

float32                       g_dshotClk0Hz;
/******************************************************************************/
/*--------------------------Function Implementations--------------------------*/
/******************************************************************************/
static uint16 dshotBuild(uint16 value, boolean telem);


void Dshot_init(void)
{
    Ifx_GTM                 *gtm = &MODULE_GTM;

    /* CLK0 = GTM module clock, undivided. Additive: the 2-bit CLK_EN fields leave FXCLK alone. */
    IfxGtm_Cmu_setClkFrequency(gtm, IfxGtm_Cmu_Clk_0, IfxGtm_Cmu_getModuleFrequency(gtm));
    IfxGtm_Cmu_enableClocks(gtm, IFXGTM_CMU_CLKEN_CLK0);
    g_dshotClk0Hz = IfxGtm_Cmu_getClkFrequency(gtm, IfxGtm_Cmu_Clk_0, TRUE);

    uint32 ticksPerBit = (uint32)(g_dshotClk0Hz * 3.333e-6f + 0.5f);   /* DShot300 bit slot */
    g_t0hTicks = (3u * ticksPerBit) / 8u;    /* 1.25 us */
    g_t1hTicks = (3u * ticksPerBit) / 4u;    /* 2.50 us */

    IfxGtm_Atom_Pwm_Config cfg;
    IfxGtm_Atom_Pwm_initConfig(&cfg, gtm);

    cfg.atom                     = IfxGtm_Atom_0;
    cfg.atomChannel              = IfxGtm_Atom_Ch_0;
    cfg.clock                    = IfxGtm_Cmu_Clk_0;
    cfg.period                   = ticksPerBit;
    cfg.dutyCycle                = 0u;
    cfg.signalLevel              = Ifx_ActiveState_high;
    cfg.synchronousUpdateEnabled = TRUE;
    cfg.immediateStartEnabled    = TRUE;
    cfg.pin.outputPin            = &IfxGtm_ATOM0_0_TOUT48_P22_1_OUT;
    cfg.pin.outputMode           = IfxPort_OutputMode_openDrain;
    cfg.pin.padDriver            = IfxPort_PadDriver_ttlSpeed1;

    (void)IfxGtm_Atom_Pwm_init(&g_atomM1, &cfg);
    IfxGtm_Atom_Pwm_start(&g_atomM1, TRUE);
}

/* throttle 0..2047 (0 = stop, 1..47 = commands), telem = request bit */
static uint16 dshotBuild(uint16 value, boolean telem)
{
    uint16 v   = (uint16)(((value & 0x7FFu) << 1) | ((telem != FALSE) ? 1u : 0u));
    uint16 crc = (uint16)((v ^ (v >> 4) ^ (v >> 8)) & 0x0Fu);
    return (uint16)((v << 4) | crc);
}

static void dshotSendFrame(uint16 frame)
{
    Ifx_GTM_ATOM    *atom = g_atomM1.atom;
    IfxGtm_Atom_Ch   ch   = g_atomM1.atomChannel;
    Ifx_GTM_ATOM_CH *regs = IfxGtm_Atom_Ch_getChannelPointer(atom, ch);
    boolean          irq  = IfxCpu_disableInterrupts();
    sint8            i;

    IfxGtm_Atom_Ch_clearZeroNotification(atom, ch);            /* CCU0TC: period boundary */
    for (i = 15; i >= 0; i--)
    {
        uint32 hi = (((frame >> i) & 1u) != 0u) ? g_t1hTicks : g_t0hTicks;
        while (regs->IRQ.NOTIFY.B.CCU0TC == 0u) { }
        IfxGtm_Atom_Ch_clearZeroNotification(atom, ch);
        IfxGtm_Atom_Ch_setCompareOneShadow(atom, ch, hi);     /* lands at the next boundary */
    }

    while (regs->IRQ.NOTIFY.B.CCU0TC == 0u) { }
    IfxGtm_Atom_Ch_clearZeroNotification(atom, ch);
    IfxGtm_Atom_Ch_setCompareOneShadow(atom, ch, 0u);         /* back to idle low */

    IfxCpu_restoreInterrupts(irq);
    g_dshotFrames++;
}

/* 1 kHz task: zero throttle = the arming stream; every 256th frame asks for telemetry */
void Dshot_task(void)
{
    boolean telem = ((g_dshotFrames & 0xFFu) == 0u) ? TRUE : FALSE;
    dshotSendFrame(dshotBuild(0u, 1u));
}
