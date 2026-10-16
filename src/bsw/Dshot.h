/*
 *  Code from crengineering
 *  Last modified: 16.09.2026
 *  Last Modified by: crengineering
 */

#ifndef DSHOT_H
#define DSHOT_H

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
  sint8  temperature;
  uint16 voltage;
  uint16 current;
  uint16 mAh;
  uint16 eRPM;
}Esc_telemetry;

typedef struct
{
    Esc_telemetry packet;    /* last CRC-valid packet from this motor                 */
    uint32        count;     /* CRC-valid packets from this motor since boot          */
    uint32        missed;    /* requests to this motor without a packet in their slot */
    boolean       alive;     /* fewer than 3 consecutive misses                       */
} Dshot_TelemetryStatus;

typedef enum {
    DSHOT_M1 = 0,
    DSHOT_M2,
    DSHOT_M3,
    DSHOT_M4,
    DSHOT_MEND
} Dshot_Motor_t;

#define DSHOT_REPLY_MAX_EDGES   24u   /* bidir reply: 21 slot edges + return-to-idle + slack */
#define DSHOT_ESC_TLM_LENGTH    10u   /* KISS telemetry frame, bytes                         */

/******************************************************************************/
/*-----------------------------Diagnostics------------------------------------*/
/******************************************************************************/
/* Deliberately global, not static: tools/xcp_read.py reads any of them live by
 * symbol name over XCP, no A2L entry needed. Nothing else references them. */
extern uint32          g_dshotFrames;                              /* frames sent since boot                */
extern uint32          g_escTlmBytes;                              /* KISS bytes received since boot        */
extern float32         g_dshotClk0Hz;                              /* GTM CLK0 actually configured          */
extern uint8           g_dshot_reply_counter[DSHOT_MEND];          /* edges per motor, last reply window    */
extern uint32          g_dshot_reply_lostEcnt[DSHOT_MEND];         /* windows with an ECNT gap (edge lost)  */
extern uint32          g_dshot_reply_crcFail[DSHOT_MEND];          /* complete captures, GCR/CRC rejected   */
extern uint32          g_dshot_reply_ok[DSHOT_MEND];               /* CRC-valid bidir replies since boot    */
extern uint32          g_dshot_reply_edt[DSHOT_MEND];              /* EDT frames (type-tagged, not eRPM)    */
extern uint32          g_dshot_eRPM[DSHOT_MEND];                   /* last valid eRPM per motor             */
extern uint8           g_dshot_reply_edges[DSHOT_REPLY_MAX_EDGES]; /* M1: bit slots per edge interval       */
extern uint32          g_dshot_reply_passes;                       /* poll passes in the last window        */
extern uint32          g_dshot_reply_minPasses;                    /* slowest window since boot             */
extern volatile uint8   g_escTlmRaw[DSHOT_ESC_TLM_LENGTH];         /* KISS frame being assembled (ISR)      */
extern volatile uint8   g_escTlmIndex;
extern volatile boolean g_escTlmComplete;

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

void Dshot_init(void);
void Dshot_task(const uint16 dshot_command[DSHOT_MEND], boolean dshot_stream_allowed, uint32 dshot_eRPM[DSHOT_MEND]);
/* Copies the last CRC-valid telemetry packet; returns the driver's CRC-ok packet
 * count so the caller can tell whether a new packet arrived since it last asked. */
uint32 Dshot_getTelemetry(Dshot_TelemetryStatus *out);
/* Bidir reply: edges captured per motor in the last reply window (diagnostic). */
void   Dshot_getReplyEdges(uint8 dshot_edge_replys[DSHOT_MEND]);
/* Bidirectional DShot: the ESC answers EVERY frame with its electrical period. Copies
 * the last CRC-valid eRPM per motor (0 until the first one) and the count of valid
 * replies since boot, so a consumer can tell a live value from a stale one. */
void   Dshot_getBidirErpm(uint32 dshot_eRPM[DSHOT_MEND], uint32 dshot_ok[DSHOT_MEND]);
/* The 11-bit DShot value that went onto each wire in the last frame (0 = stop,
 * 1..47 = commands, 48..2047 = throttle); 0 while the stream is not allowed. */
void   Dshot_getLastCommand(uint16 dshot_cmd[DSHOT_MEND]);

#endif /* DSHOT_H */
