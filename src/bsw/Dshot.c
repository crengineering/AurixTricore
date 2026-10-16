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
#include "IfxStm.h"

#define ESC_T_MESSAGE_LENGTH 10u

static const Dshot_MotorCfg    g_Dshot_MotorCfg[DSHOT_MEND] = DSHOT_MOTOR_CFG;
static IfxGtm_Atom_Pwm_Driver  g_atomMX[DSHOT_MEND];

static uint32                  g_t0hTicks;
static uint32                  g_t1hTicks;
uint32                         g_dshotFrames;
uint32                         g_escTlmBytes;

float32                        g_dshotClk0Hz;

/* Bidirectional DShot reply: GCR burst at 5/4 x 300 kbit/s = 375 kbit/s, 2.667 us per
 * bit slot, 21 slots (1 leading pad bit + 20 GCR bits), NRZI: a transition at the start
 * of a slot means GCR bit 1. The ESC turns the line around ~30 us after our frame. */
#define DSHOT_REPLY_GCR_BITS    20u
#define DSHOT_REPLY_MAX_EDGES   24u            /* 21 slot edges + return-to-idle + slack */
#define DSHOT_REPLY_BIT_S       2.6667e-6f
#define DSHOT_TBU_MASK          0x00FFFFFFu    /* TBU_TS0 is a free-running 24-bit counter */

typedef struct
{
    uint32  interval_ticks[DSHOT_REPLY_MAX_EDGES]; /* edge-to-edge, TBU_TS0 ticks, 24 bit   */
    uint32  last_ts;                               /* GPR0[23:0] of the last consumed edge  */
    uint8   last_ecnt;                             /* GPR0[31:24] of the last consumed edge */
    uint8   edges;                                 /* edges consumed in this window         */
    boolean lost;                                  /* GPROFL or ECNT gap: do not decode     */
} Dshot_ReplyCapture;

static float32                 g_tbu_clock_frequency;
static uint32                  g_replyTicksPerBit;            /* TBU ticks per 2.667 us slot */
static sint32                  g_ticks_for_100us;

/* Reply diagnostics - deliberately not static so the debugger / A2L can see them */
uint8                          g_dshot_reply_counter[DSHOT_MEND];  /* edges per motor, last window        */
uint32                         g_dshot_reply_lostEcnt[DSHOT_MEND]; /* windows with an ECNT gap (edge lost)*/
uint32                         g_dshot_reply_crcFail[DSHOT_MEND];  /* complete captures, GCR/CRC rejected */
uint32                         g_dshot_reply_ok[DSHOT_MEND];
uint32                         g_dshot_reply_edt[DSHOT_MEND];      /* EDT frames (type-tagged, not eRPM)  */
uint32                         g_dshot_eRPM[DSHOT_MEND];           /* last valid eRPM per motor           */
uint8                          g_dshot_reply_edges[DSHOT_REPLY_MAX_EDGES]; /* M1 debug: slots per interval */
uint32                         g_dshot_reply_passes;               /* poll passes in the last window      */
uint32                         g_dshot_reply_minPasses = 0xFFFFFFFFu; /* slowest window since boot        */
static uint32                  g_dshot_reply_value = 0u;           /* M1 eRPM (legacy XCP hook)           */

/* ESC Telemetrics */
volatile uint8                 g_escTlmRaw[ESC_T_MESSAGE_LENGTH];
volatile uint8                 g_escTlmIndex    = 0u;
volatile boolean               g_escTlmComplete = FALSE;
static   Dshot_TelemetryStatus g_esc_Tlm[DSHOT_MEND];
static   uint32                g_escTlmCrcOk    = 0u;
static   uint32                g_escTlmCrcFail  = 0u;

static const uint8 GCR_DECODE_TABLE[32] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x00 - 0x07
    0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0xFF, 0x0E, 0x0F, // 0x08 - 0x0F
    0xFF, 0xFF, 0x02, 0x03, 0x05, 0x06, 0xFF, 0x07, // 0x10 - 0x17
    0xFF, 0x00, 0x08, 0x01, 0x0C, 0x0D, 0x04, 0xFF  // 0x18 - 0x1F (Index 30 ist 0x1E -> 0x04)
};
/******************************************************************************/
/*--------------------------Function Declaration------------------------------*/
/******************************************************************************/
static void    Dshot_init_motors      (void);
static void    Dshot_init_uart        (void);
static void    Dshot_init_motor_reply (void);

static uint16  dshotBuild             (uint16 value, boolean telem);
static void    dshotSendFrame         (uint16 frame[DSHOT_MEND], Dshot_ReplyCapture reply[DSHOT_MEND]);

static void    dshotReadReply         (Dshot_ReplyCapture reply[DSHOT_MEND]);
typedef enum
{
    DSHOT_REPLY_OK = 0,      /* *eRPM written                                        */
    DSHOT_REPLY_INVALID,     /* incomplete capture, bad GCR quintet or CRC mismatch  */
    DSHOT_REPLY_EDT          /* valid frame, but extended telemetry - not an eRPM    */
} Dshot_ReplyResult;

static Dshot_ReplyResult dshotDecodeReply(const Dshot_ReplyCapture *cap, uint32 *eRPM, boolean debugBits);

static boolean Dshot_ESC_T_crc        (void);
static boolean Dshot_interpret_ESC_Tlm(Esc_telemetry *esc_Tlm);

static void    Dshot_telem_request    (boolean *telem_requested, uint8 dshot_tlm_miss_streak[DSHOT_MEND], Dshot_Motor_t *dshot_motor, const boolean *telem);
static void    Dshot_sending          (uint16 frames[DSHOT_MEND], const uint16 dshot_command[DSHOT_MEND], const Dshot_Motor_t *dshot_motor, const boolean *telem, const boolean dshot_stream_allowed, Dshot_ReplyCapture reply[DSHOT_MEND]);
static void    Dshot_receive_decode   (const Dshot_ReplyCapture reply[DSHOT_MEND], uint32 dshot_eRPM[DSHOT_MEND]);
static void    Dshot_telem_decode     (boolean *telem_requested, uint8 dshot_tlm_miss_streak[DSHOT_MEND], Dshot_Motor_t *dshot_motor);

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
        cfg.signalLevel              = Ifx_ActiveState_low;
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

static void Dshot_init_uart(void)
{
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


static void Dshot_init_motor_reply (void)
{
    Ifx_GTM       *gtm      = &MODULE_GTM;
    Ifx_GTM_TIM   *gtm_tim  = &gtm->TIM[0];
    Dshot_Motor_t  motor_id = DSHOT_M1;

    g_ticks_for_100us = IfxStm_getTicksFromMicroseconds(&MODULE_STM0, 100u);

    IfxGtm_Tbu_setChannelClockSource(gtm, IfxGtm_Tbu_Ts_0, IfxGtm_Tbu_ClkSrc_0);
    IfxGtm_Tbu_enableChannel(gtm, IfxGtm_Tbu_Ts_0);
    g_tbu_clock_frequency = IfxGtm_Tbu_getClockFrequency(gtm, IfxGtm_Tbu_Ts_0);
    g_replyTicksPerBit    = (uint32)(g_tbu_clock_frequency * DSHOT_REPLY_BIT_S + 0.5f);
    for (motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
    {
        Ifx_GTM_TIM_CH *gtm_tim_ch = IfxGtm_Tim_getChannel(gtm_tim, g_Dshot_MotorCfg[motor_id].timChannel);

        gtm_tim_ch->CTRL.B.TIM_EN   = 0u;

        IfxGtm_PinMap_setTimTin(g_Dshot_MotorCfg[motor_id].pinIn, IfxPort_InputMode_undefined);
        gtm_tim_ch->CTRL.B.TIM_MODE = IfxGtm_Tim_Mode_inputEvent;
        gtm_tim_ch->CTRL.B.ISL      = 1u;
        gtm_tim_ch->CTRL.B.DSL      = 0u;
        gtm_tim_ch->CTRL.B.TBU0_SEL = 0u;
        gtm_tim_ch->CTRL.B.GPR0_SEL = IfxGtm_Tim_GprSel_tbuTs0;

        gtm_tim_ch->CTRL.B.TIM_EN   = 1u;

    }
}


/* throttle 0..2047 (0 = stop, 1..47 = commands), telem = request bit */
static uint16 dshotBuild(uint16 value, boolean telem)
{
    uint16 v   = (uint16)(((value & 0x7FFu) << 1) | ((telem != FALSE) ? 1u : 0u));
    uint16 crc = (uint16)(( ~(v ^ (v >> 4) ^ (v >> 8))) & 0x0Fu);
    return (uint16)((v << 4) | crc);
}

static void dshotSendFrame(uint16 frame[DSHOT_MEND], Dshot_ReplyCapture reply[DSHOT_MEND])
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
    while (regs->IRQ.NOTIFY.B.CCU0TC == 0u) { }
    IfxGtm_Atom_Ch_clearZeroNotification(atom, refCh);

    dshotReadReply(reply);

    IfxCpu_restoreInterrupts(irq);
    g_dshotFrames++;
}

/* Drain the four TIM channels for one reply window. The hardware does the timing:
 * in TIEM every edge captures TBU_TS0 into GPR0[23:0] and the edge count ECNT into
 * GPR0[31:24]. This loop only has to visit each channel once per 2.667 us slot; how
 * late it is inside that slot does not touch the timestamp.
 * Every GTM/STM register read is an SPB access (tens of CPU cycles), so the hot loop
 * is ONE read per channel per pass: GPR0 itself. A new edge shows as a changed ECNT,
 * so NOTIFY is neither read nor cleared in the loop. An ECNT step != 1 means an edge
 * came and went between two visits; that marks the capture lost instead of letting
 * it decode to garbage. The pass counter is the loop's own speedometer: passes per
 * 100 us window must stay well above 100us / 2.667us = 37. */
#pragma optimize 2
static void dshotReadReply(Dshot_ReplyCapture reply[DSHOT_MEND])
{
    Ifx_GTM                 *gtm     = &MODULE_GTM;
    Ifx_GTM_TIM             *gtm_tim = &gtm->TIM[0];
    volatile const uint32   *gpr0Reg[DSHOT_MEND];
    uint32                   start_count;
    uint32                   pass    = 0u;
    uint32                   stm_ticks = (uint32)g_ticks_for_100us;
    Dshot_Motor_t            motor_id;

    for (motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
    {
        Ifx_GTM_TIM_CH *ch = IfxGtm_Tim_getChannel(gtm_tim, g_Dshot_MotorCfg[motor_id].timChannel);

        gpr0Reg[motor_id] = (volatile const uint32 *)&ch->GPR0.U;
        /* our own frame and the pad release toggled this pin: take the current edge
         * count as the baseline, so the first reply edge is ECNT baseline + 1 */
        reply[motor_id].last_ecnt = (uint8)(*gpr0Reg[motor_id] >> 24u);
        reply[motor_id].edges     = 0u;
        reply[motor_id].lost      = FALSE;
    }

    start_count = IfxStm_getLower(&MODULE_STM0);
    for (;;)
    {
        for (motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
        {
            Dshot_ReplyCapture *cap  = &reply[motor_id];
            uint32              gpr0 = *gpr0Reg[motor_id];          /* the one SPB read per channel */
            uint8               ecnt = (uint8)(gpr0 >> 24u);

            if (ecnt != cap->last_ecnt)
            {
                uint32 ts = gpr0 & DSHOT_TBU_MASK;

                if ((uint8)(ecnt - cap->last_ecnt) != 1u)
                {
                    cap->lost = TRUE;                                /* an edge came and went between visits */
                }
                if ((cap->edges > 0u) && (cap->edges <= DSHOT_REPLY_MAX_EDGES))
                {
                    cap->interval_ticks[cap->edges - 1u] = (ts - cap->last_ts) & DSHOT_TBU_MASK;
                }
                cap->last_ts   = ts;
                cap->last_ecnt = ecnt;
                if (cap->edges < 255u)
                {
                    cap->edges++;
                }
            }
        }

        pass++;
        if ((pass & 7u) == 0u)                                       /* STM is on the SPB too: check rarely */
        {
            if ((IfxStm_getLower(&MODULE_STM0) - start_count) >= stm_ticks)
            {
                break;
            }
        }
    }

    g_dshot_reply_passes = pass;
    if (pass < g_dshot_reply_minPasses)
    {
        g_dshot_reply_minPasses = pass;
    }
    for (motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
    {
        if (reply[motor_id].lost != FALSE)
        {
            g_dshot_reply_lostEcnt[motor_id]++;
        }
        g_dshot_reply_counter[motor_id] = reply[motor_id].edges;
    }
}
#pragma endoptimize

/* One motor's edge intervals -> eRPM. *eRPM is written only for DSHOT_REPLY_OK.
 *
 * Edge k sits at the start of slot b(k). The slots between two edges are GCR 0, the
 * slot an edge starts is GCR 1. The first edge starts the leading pad slot and carries
 * no data. If the line's final level differs from idle the ESC's release produces one
 * more edge at slot 21 - that one and anything later is ignored. If the GCR word ends
 * in zeros there is no edge for them, so the word is padded to 20 bits. */
static Dshot_ReplyResult dshotDecodeReply(const Dshot_ReplyCapture *cap, uint32 *eRPM, boolean debugBits)
{
    uint32 gcr     = 0u;
    uint32 bits    = 0u;
    uint16 decoded = 0u;
    uint16 value;
    uint8  crc;
    uint8  exponent;
    uint16 mantissa;
    uint32 period_us;
    uint8  i;
    uint8  intervals;

    if ((cap->lost != FALSE) || (cap->edges < 2u) || (g_replyTicksPerBit == 0u))
    {
        return DSHOT_REPLY_INVALID;
    }

    intervals = (uint8)(cap->edges - 1u);
    if (intervals > DSHOT_REPLY_MAX_EDGES)
    {
        intervals = DSHOT_REPLY_MAX_EDGES;
    }

    for (i = 0u; i < intervals; i++)
    {
        uint32 slots = (cap->interval_ticks[i] + (g_replyTicksPerBit / 2u)) / g_replyTicksPerBit;

        if (debugBits != FALSE)
        {
            g_dshot_reply_edges[i] = (uint8)((slots > 255u) ? 255u : slots);
        }
        if (slots == 0u)
        {
            return DSHOT_REPLY_INVALID;                   /* two edges inside half a slot: glitch */
        }
        if ((bits + slots) > DSHOT_REPLY_GCR_BITS)
        {
            break;                                        /* return-to-idle edge (slot 21) or later */
        }
        gcr   = (gcr << slots) | 1u;
        bits += slots;
    }
    if (bits < DSHOT_REPLY_GCR_BITS)
    {
        gcr <<= (DSHOT_REPLY_GCR_BITS - bits);            /* trailing GCR zeros have no edge */
    }

    for (i = 4u; i > 0u; i--)
    {
        uint8 nibble = GCR_DECODE_TABLE[(gcr >> ((i - 1u) * 5u)) & 0x1Fu];

        if (nibble == 0xFFu)
        {
            return DSHOT_REPLY_INVALID;
        }
        decoded = (uint16)((decoded << 4u) | nibble);
    }

    value = (uint16)(decoded >> 4u);                      /* eee mmmmmmmmm */
    crc   = (uint8)(decoded & 0x0Fu);
    if (((uint8)(~(value ^ (value >> 4u) ^ (value >> 8u))) & 0x0Fu) != crc)
    {
        return DSHOT_REPLY_INVALID;
    }

    exponent  = (uint8)((value >> 9u) & 0x07u);
    mantissa  = (uint16)(value & 0x1FFu);

    /* AM32 normalises the period so the mantissa MSB is set whenever exponent > 0.
     * exponent != 0 with bit 8 clear is therefore an EDT frame: top nibble = type
     * tag (temperature, voltage, ...), low byte = payload (DSHOT.md D4). Only sent
     * after DShot command 13 enabled EDT, but never let one through as an eRPM. */
    if ((exponent != 0u) && ((value & 0x100u) == 0u))
    {
        return DSHOT_REPLY_EDT;
    }

    period_us = (uint32)mantissa << exponent;             /* one electrical revolution */
    *eRPM     = (period_us != 0u) ? (60000000u / period_us) : 0u;

    return DSHOT_REPLY_OK;
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

static void Dshot_telem_request(boolean *telem_requested, uint8 dshot_tlm_miss_streak[DSHOT_MEND], Dshot_Motor_t *dshot_motor, const boolean *telem)
{
    if (*telem != FALSE)
    {
        if (*telem_requested != FALSE)
        {
            g_esc_Tlm[*dshot_motor].missed++;
            g_escTlmIndex    = 0u;
            g_escTlmComplete = FALSE;
            (dshot_tlm_miss_streak[*dshot_motor] < 255u) ? (dshot_tlm_miss_streak[*dshot_motor]++) : (dshot_tlm_miss_streak[*dshot_motor] = 255u);
            (dshot_tlm_miss_streak[*dshot_motor] >= 3u ) ? (g_esc_Tlm[*dshot_motor].alive = FALSE) : (g_esc_Tlm[*dshot_motor].alive = TRUE);
        }

        if (*dshot_motor < (DSHOT_MEND-1u))
        {
            (*dshot_motor)++;
        }
        else {
            *dshot_motor = DSHOT_M1;
        }
        *telem_requested = TRUE;
    }
}

static void Dshot_telem_decode(boolean *telem_requested, uint8 dshot_tlm_miss_streak[DSHOT_MEND], Dshot_Motor_t *dshot_motor)
{
    if ( (*telem_requested  != FALSE) &&
         (g_escTlmComplete != FALSE)  )
    {
        if (Dshot_interpret_ESC_Tlm(&g_esc_Tlm[*dshot_motor].packet) != FALSE)
        {
            g_esc_Tlm[*dshot_motor].count++;
            g_esc_Tlm[*dshot_motor].alive = TRUE;
            dshot_tlm_miss_streak[*dshot_motor] = 0u;
        }
        g_escTlmIndex    = 0u;
        g_escTlmComplete = FALSE;
        *telem_requested  = FALSE;
    }
}

static void Dshot_sending(uint16 frames[DSHOT_MEND], const uint16 dshot_command[DSHOT_MEND], const Dshot_Motor_t *dshot_motor, const boolean *telem, const boolean dshot_stream_allowed, Dshot_ReplyCapture reply[DSHOT_MEND])
{
    /* building the frame */
    for (uint8 motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
    {
        frames[motor_id] = dshotBuild(dshot_command[motor_id], (*telem && (*dshot_motor == motor_id)) ? TRUE : FALSE);
    }

    /* Dshot send Frames */
    if (dshot_stream_allowed != FALSE)
    {
        dshotSendFrame(frames, reply);
    }
}



static void Dshot_receive_decode(const Dshot_ReplyCapture reply[DSHOT_MEND], uint32 dshot_eRPM[DSHOT_MEND])
{
    Dshot_Motor_t motor_id;

    for (motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
    {
        uint32 eRPM = 0u;

        if (reply[motor_id].edges != 0u)                  /* no reply window (stream off) is not a failure */
        {
            Dshot_ReplyResult result = dshotDecodeReply(&reply[motor_id], &eRPM, (motor_id == DSHOT_M1) ? TRUE : FALSE);

            if (result == DSHOT_REPLY_OK)
            {
                g_dshot_eRPM[motor_id] = eRPM;
                g_dshot_reply_ok[motor_id]++;
            }
            else if (result == DSHOT_REPLY_EDT)
            {
                g_dshot_reply_edt[motor_id]++;
            }
            else if (reply[motor_id].lost == FALSE)
            {
                g_dshot_reply_crcFail[motor_id]++;        /* lost captures are counted in dshotReadReply */
            }
        }
        dshot_eRPM[motor_id] = g_dshot_eRPM[motor_id];    /* last valid value, 0 until the first one */
    }

    g_dshot_reply_value = g_dshot_eRPM[DSHOT_M1];
}

/******************************************************************************/
/*--------------------------Global Functions----------------------------------*/
/******************************************************************************/

void Dshot_init(void)
{
    Dshot_init_motors();
    Dshot_init_uart();
    Dshot_init_motor_reply();
}

/* 1 kHz task: zero throttle = the arming stream; every 256th frame asks for telemetry */
void Dshot_task(const uint16 dshot_command[DSHOT_MEND], boolean dshot_stream_allowed, uint32 dshot_eRPM[DSHOT_MEND])
{
    static boolean       telem_requested    = FALSE;
    static Dshot_Motor_t dshot_motor        = DSHOT_M1;
    static uint8         dshot_tlm_miss_streak[DSHOT_MEND] = {0u};
           uint16        frames[DSHOT_MEND] = {0u};
           boolean       telem = ((g_dshotFrames & 0x7u) == 0u) ? TRUE : FALSE;
           Dshot_ReplyCapture reply[DSHOT_MEND] = {{{0u}, 0u, 0u, 0u, FALSE}};

    /* telemetrics request */
    Dshot_telem_request(&telem_requested, dshot_tlm_miss_streak, &dshot_motor, &telem);

    /* sending dshot */
    Dshot_sending(frames, dshot_command, &dshot_motor, &telem, dshot_stream_allowed, reply);

    /* receiving from bidirectional dshot */
    Dshot_receive_decode(reply, dshot_eRPM);

    /* decode telemetrics*/
    Dshot_telem_decode(&telem_requested, dshot_tlm_miss_streak, &dshot_motor);

}

uint32 Dshot_getTelemetry(Dshot_TelemetryStatus *out)
{
    for (uint8 motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
    {
        out[motor_id].packet = g_esc_Tlm[motor_id].packet;
        out[motor_id].count  = g_esc_Tlm[motor_id].count;
        //out[motor_id].missed = g_esc_Tlm[motor_id].missed;
        out[motor_id].missed = g_dshot_reply_value;
        out[motor_id].alive  = g_esc_Tlm[motor_id].alive;
    }

    return g_escTlmCrcFail;
}

void Dshot_getBidirErpm(uint32 dshot_eRPM[DSHOT_MEND], uint32 dshot_ok[DSHOT_MEND])
{
    for (uint8 motor_id = DSHOT_M1; motor_id < DSHOT_MEND; motor_id++)
    {
        dshot_eRPM[motor_id] = g_dshot_eRPM[motor_id];
        dshot_ok[motor_id]   = g_dshot_reply_ok[motor_id];
    }
}

void Dshot_getReplyEdges(uint8 dshot_edge_replys[DSHOT_MEND])
{
    dshot_edge_replys[DSHOT_M1] =  g_dshot_reply_counter[DSHOT_M1];
    dshot_edge_replys[DSHOT_M2] =  g_dshot_reply_counter[DSHOT_M2];
    dshot_edge_replys[DSHOT_M3] =  g_dshot_reply_counter[DSHOT_M3];
    dshot_edge_replys[DSHOT_M4] =  g_dshot_reply_counter[DSHOT_M4];
}
