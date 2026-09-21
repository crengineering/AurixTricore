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
#include "Dshot_cfg.h"

#define ESC_T_MESSAGE_LENGTH 10u

static const Dshot_MotorCfg g_Dshot_MotorCfg[DSHOT_MEND] = DSHOT_MOTOR_CFG;
static IfxGtm_Atom_Pwm_Driver g_atomMX[DSHOT_MEND];

static uint32                 g_t0hTicks;
static uint32                 g_t1hTicks;
uint32                        g_dshotFrames;
uint32                        g_escTlmBytes;

float32                       g_dshotClk0Hz;

/* ESC Telemetrics */
volatile uint8                 g_escTlmRaw[ESC_T_MESSAGE_LENGTH];
volatile uint8                 g_escTlmIndex    = 0u;
volatile boolean               g_escTlmComplete = FALSE;
static   Dshot_TelemetryStatus g_esc_Tlm[DSHOT_MEND];
static   uint32                g_escTlmCrcOk    = 0u;
static   uint32                g_escTlmCrcFail  = 0u;

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
static void Dshot_init_motors(void)
{
    Ifx_GTM          *gtm  = &MODULE_GTM;
    Ifx_GTM_ATOM_AGC *agc;
    uint16            mask = 0u;
    uint32            ticksPerBit;
    Dshot_Motor_t     motor_id;

    /* CLK0 = GTM module clock, undivided. Additive: the 2-bit CLK_EN fields leave FXCLK alone.
     * Once, before any channel: CMU_CLK dividers may only be written while the clock is off. */
    IfxGtm_Cmu_setClkFrequency(gtm, IfxGtm_Cmu_Clk_0, IfxGtm_Cmu_getModuleFrequency(gtm));
    IfxGtm_Cmu_enableClocks(gtm, IFXGTM_CMU_CLKEN_CLK0);
    g_dshotClk0Hz = IfxGtm_Cmu_getClkFrequency(gtm, IfxGtm_Cmu_Clk_0, TRUE);

    ticksPerBit = (uint32)(g_dshotClk0Hz * 3.333e-6f + 0.5f);   /* DShot300 bit slot */
    g_t0hTicks  = (3u * ticksPerBit) / 8u;    /* 1.25 us */
    g_t1hTicks  = (3u * ticksPerBit) / 4u;    /* 2.50 us */

    for (motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
    {
        IfxGtm_Atom_Pwm_Config cfg;
        IfxGtm_Atom_Pwm_initConfig(&cfg, gtm);

        cfg.atom                     = IfxGtm_Atom_0;
        cfg.atomChannel              = g_Dshot_MotorCfg[motor_id].atomChannel;
        cfg.clock                    = IfxGtm_Cmu_Clk_0;
        cfg.period                   = ticksPerBit;
        cfg.dutyCycle                = 0u;
        cfg.signalLevel              = Ifx_ActiveState_high;
        cfg.synchronousUpdateEnabled = TRUE;
        cfg.immediateStartEnabled    = FALSE;      /* all channels start together below */
        cfg.pin.outputPin            = g_Dshot_MotorCfg[motor_id].pin;
        cfg.pin.outputMode           = IfxPort_OutputMode_openDrain;
        cfg.pin.padDriver            = IfxPort_PadDriver_ttlSpeed1;

        (void)IfxGtm_Atom_Pwm_init(&g_atomMX[motor_id], &cfg);
        mask |= (uint16)(1u << (uint16)g_atomMX[motor_id].atomChannel);
    }

    /* One host trigger starts every channel on the same GTM tick, so all four
     * share the period boundary the frame loop waits for on M1. Channels started
     * one after another (IfxGtm_Atom_Pwm_start) sit at fixed phase offsets and a
     * frame's first shadow write could be overwritten before it transferred
     * (bench 2026-09-19: 15 of 16 pulses on M3/M4, "sometimes"). */
    agc = g_atomMX[DSHOT_M1].agc;
    IfxGtm_Atom_Agc_enableChannels(agc, mask, 0u, FALSE);
    IfxGtm_Atom_Agc_enableChannelsOutput(agc, mask, 0u, FALSE);
    IfxGtm_Atom_Agc_trigger(agc);
}

static void Dshot_init_uart(void){
    /* 
    init UART Telemetry from ESC on P23.3
    */
    IfxAsclin_Status status = IfxAsclin_Status_configurationError;
    IfxAsclin_Asc_Config config;
    IfxAsclin_Asc_initModuleConfig(&config, &MODULE_ASCLIN6);

    /* set baudrate for ESC Telemetry*/
    config.baudrate.baudrate =  UART_SPEED_115200;
    /* 16 ticks per bit, sample mid-bit, majority of three: the iLLD default
     * (4x, one sample at the last quarter) framed-errored ~40 % of the ESC's
     * back-to-back bytes (bench 2026-09-17, ASCLIN6 FLAGS.FE set). */
    config.baudrate.oversampling         = IfxAsclin_OversamplingFactor_16;
    config.bitTiming.samplePointPosition = IfxAsclin_SamplePointPosition_8;
    config.bitTiming.medianFilter        = IfxAsclin_SamplesPerBit_three;

    static const IfxAsclin_Asc_Pins pins = {
        .cts       = NULL_PTR,                        /* no hardware flow control */
        .rx        = &IfxAsclin6_RXA_P23_3_IN,
        .rxMode    = IfxPort_InputMode_noPullDevice,
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

void Dshot_init(void)
{
    Dshot_init_motors();
    Dshot_init_uart();
}

/* throttle 0..2047 (0 = stop, 1..47 = commands), telem = request bit */
static uint16 dshotBuild(uint16 value, boolean telem)
{
    uint16 v   = (uint16)(((value & 0x7FFu) << 1) | ((telem != FALSE) ? 1u : 0u));
    uint16 crc = (uint16)((v ^ (v >> 4) ^ (v >> 8)) & 0x0Fu);
    return (uint16)((v << 4) | crc);
}

static void dshotSendFrame(uint16 frame[DSHOT_MEND])
{
    Ifx_GTM_ATOM    *atom  = g_atomMX[DSHOT_M1].atom;
    IfxGtm_Atom_Ch   refCh = g_atomMX[DSHOT_M1].atomChannel;
    Ifx_GTM_ATOM_CH *regs  = IfxGtm_Atom_Ch_getChannelPointer(atom, refCh);
    boolean          irq   = IfxCpu_disableInterrupts();
    sint8            i;
    Dshot_Motor_t    motor_id;

    IfxGtm_Atom_Ch_clearZeroNotification(atom, refCh);            /* CCU0TC: period boundary */
    for (i = 15; i >= 0; i--)
    {
        while (regs->IRQ.NOTIFY.B.CCU0TC == 0u) { }
        IfxGtm_Atom_Ch_clearZeroNotification(atom, refCh);
        for(motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
        {
            uint32 hi = (((frame[motor_id] >> i) & 1u) != 0u) ? g_t1hTicks : g_t0hTicks;
            IfxGtm_Atom_Ch_setCompareOneShadow(atom, g_atomMX[motor_id].atomChannel, hi);     /* lands at the next boundary */
        }
    }

    while (regs->IRQ.NOTIFY.B.CCU0TC == 0u) { }
    IfxGtm_Atom_Ch_clearZeroNotification(atom, refCh);
    for(motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
    {
        IfxGtm_Atom_Ch_setCompareOneShadow(atom, g_atomMX[motor_id].atomChannel, 0u);         /* back to idle low */
    }

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
    uint16 frames[DSHOT_MEND];
    static boolean telem_requested = FALSE;
    static Dshot_Motor_t dshot_motor = DSHOT_M1;
    /* telemetrics request */
    boolean telem = ((g_dshotFrames & 0x7u) == 0u) ? TRUE : FALSE;
    if (telem != FALSE)
    {
        if (telem_requested != FALSE)
        {
            g_esc_Tlm[dshot_motor].missed++;
        }

        if (dshot_motor < (DSHOT_MEND-1u))
        {
            dshot_motor++;
        }
        else {
            dshot_motor = DSHOT_M1;
        }


        telem_requested = TRUE;
    }

    /* building frames */
    for (uint8 motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
    {
        frames[motor_id] = dshotBuild(0u, (telem && (dshot_motor == motor_id)) ? TRUE : FALSE);
    }

    /* Dshot send Frames */
    dshotSendFrame(frames);


    /* decode telemetrics*/
    if ( (telem_requested != FALSE) &&
         (g_escTlmComplete != FALSE) )
    {
        if (Dshot_interpret_ESC_Tlm(&g_esc_Tlm[dshot_motor].packet) != FALSE)
        {
            g_esc_Tlm[dshot_motor].count++;
        }
        g_escTlmIndex    = 0u;
        g_escTlmComplete = FALSE;
        telem_requested  = FALSE;
    }
}

uint32 Dshot_getTelemetry(Dshot_TelemetryStatus *out)
{
    for (uint8 motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
    {
        out[motor_id].packet = g_esc_Tlm[motor_id].packet;
        out[motor_id].count  = g_esc_Tlm[motor_id].count;
        out[motor_id].missed = g_esc_Tlm[motor_id].missed;
    }

    return g_escTlmCrcFail;
}
