/*
 *  Code from crengineering
 *  Last modified: 16.09.2026
 *  Last Modified by: crengineering
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "Dshot.h"
#include "Uart.h"
#include "IfxGtm_Atom_Pwm.h"
#include "IfxGtm_PinMap.h"
#include "IfxCpu.h"
#include "IfxAsclin_Asc.h"
#include "IfxAsclin.h"
#include "ConfigurationIsr.h"

#define ESC_T_MESSAGE_LENGTH 10u

static IfxGtm_Atom_Pwm_Driver g_atomM1;
static uint32                 g_t0hTicks;
static uint32                 g_t1hTicks;
uint32                        g_dshotFrames;
uint32                        g_escTlmBytes;

float32                       g_dshotClk0Hz;

/* ESC Telemetrics */
volatile uint8                g_escTlmRaw[ESC_T_MESSAGE_LENGTH];
volatile uint8                g_escTlmIndex    = 0u;
volatile boolean              g_escTlmComplete = FALSE;
static   Esc_telemetry        g_esc_Tlm;
static   uint8                g_escTlmCrcOk    = 0u;
static   uint8                g_escTlmCrcFail  = 0u;

/******************************************************************************/
/*--------------------------Function Declaration------------------------------*/
/******************************************************************************/
static uint16 dshotBuild(uint16 value, boolean telem);

/******************************************************************************/
/*--------------------------Interrupts----------------------------------------*/
/******************************************************************************/
IFX_INTERRUPT(asclin6IsrReceive, 0, ISR_PRIORITY_ASCLIN6_RX);

void asclin6IsrReceive(void)
{

    uint8 fill_level = IfxAsclin_getRxFifoFillLevel(&MODULE_ASCLIN6);

    for (uint8 i=0u; i<fill_level; i++)
    {
        uint32 rx_word   = IfxAsclin_readRxData(&MODULE_ASCLIN6);
        uint8  fifo_byte = (uint8)(rx_word & 0xFFu);
        g_escTlmBytes++;

        if ((g_escTlmComplete == FALSE) && (g_escTlmIndex < ESC_T_MESSAGE_LENGTH))
        {
            g_escTlmRaw[g_escTlmIndex] = fifo_byte;
            g_escTlmIndex++;
            if (g_escTlmIndex == ESC_T_MESSAGE_LENGTH)
            {
                g_escTlmComplete = TRUE;
            }
        }
    }
}
/******************************************************************************/
/*--------------------------Function Implementations--------------------------*/
/******************************************************************************/
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

    /* 
    init UART Telemetry from ESC on P23.3
    */
    IfxAsclin_Status status = IfxAsclin_Status_configurationError;
    IfxAsclin_Asc_Config config;
    IfxAsclin_Asc_initModuleConfig(&config, &MODULE_ASCLIN6);

    /* set baudrate for ESC Telemetry*/
    config.baudrate.baudrate =  UART_SPEED_115200;

    static const IfxAsclin_Asc_Pins pins = {
        .cts       = NULL_PTR,                        /* no hardware flow control */
        .rx        = &IfxAsclin6_RXA_P23_3_IN,
        .rxMode    = IfxPort_InputMode_pullUp,
        .rts       = NULL_PTR,                        /* no hardware flow control */
        .pinDriver = IfxPort_PadDriver_ttlSpeed1

    };
    config.pins = &pins;

    /* interrupt config */
    config.interrupt.rxPriority    = ISR_PRIORITY_ASCLIN6_RX;
    config.interrupt.typeOfService = IfxSrc_Tos_cpu0;

    /* No software FIFO buffers — transmission goes directly to the HW FIFO */
    config.txBuffer     = NULL_PTR;
    config.txBufferSize = 0;
    config.rxBuffer     = NULL_PTR;
    config.rxBufferSize = 0;

    static IfxAsclin_Asc s_escAsclin;

    status = IfxAsclin_Asc_initModule(&s_escAsclin, &config);

    /* hardware reset on silicon */
    IfxAsclin_flushRxFifo(&MODULE_ASCLIN6);
    IfxAsclin_clearAllFlags(&MODULE_ASCLIN6);
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

static boolean Dshot_ESC_T_crc(void)
{
    uint8 crc = 0x00u;

    for(uint8 i = 0u; i < 9u; i++)
    {
        crc ^= g_escTlmRaw[i];
        for(uint8 j = 0u; j < 8u; j++)
        {
            if ( (crc & 0x80u) != 0u)
            {
                crc = (uint8)((crc << 1u) ^ 0x07u);
            }
            else
            {
                crc = (uint8)(crc << 1u);
            }
        }
    }

    return (boolean) (crc == g_escTlmRaw[9]);
}

static boolean Dshot_interpret_ESC_Tlm(Esc_telemetry *esc_Tlm){
    boolean Tlm_success = FALSE;
    if (Dshot_ESC_T_crc() != FALSE)
    {
        esc_Tlm->temperature = (sint8) g_escTlmRaw[0u];
        esc_Tlm->voltage     = (uint16) ( (g_escTlmRaw[1u] << 8) | (g_escTlmRaw[2u]) );
        esc_Tlm->current     = (uint16) ( (g_escTlmRaw[3u] << 8) | (g_escTlmRaw[4u]) );
        esc_Tlm->mAh         = (uint16) ( (g_escTlmRaw[5u] << 8) | (g_escTlmRaw[6u]) );
        esc_Tlm->eRPM        = (uint16) ( (g_escTlmRaw[7u] << 8) | (g_escTlmRaw[8u]) );
        g_escTlmCrcOk++;
        Tlm_success = TRUE;
    }
    else {
        g_escTlmCrcFail++;
    }

    return Tlm_success;
}

/* 1 kHz task: zero throttle = the arming stream; every 256th frame asks for telemetry */
void Dshot_task(void)
{
    static boolean telem_requested = FALSE;
    boolean telem = ((g_dshotFrames & 0xFFu) == 0u) ? TRUE : FALSE;
    if (telem != FALSE)
    {
        telem_requested = TRUE;
    }
    dshotSendFrame(dshotBuild(0u, telem));

    /* decode telemetrics*/
    if ( (telem_requested  != FALSE) &&
         (g_escTlmComplete != FALSE) )
    {
        (void)Dshot_interpret_ESC_Tlm(&g_esc_Tlm);
        g_escTlmIndex    = 0u;
        g_escTlmComplete = FALSE;
        telem_requested  = FALSE;

    }
}
