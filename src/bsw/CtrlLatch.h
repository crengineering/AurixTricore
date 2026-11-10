/**********************************************************************************************************************
 * \file CtrlLatch.h
 * \brief CPU1 -> CPU0 flight-controller output: the four rotor-speed requests
 *        plus the intermediate values worth logging.
 *
 * Lives in the LMU shared block (SharedRam.h), placed in SharedRam.c. Single
 * writer: the flight core (FlightTask_step, CPU1), once per IMU sample. Reader:
 * Task_Dshot (CPU0, 1 ms).
 *
 * Unlike the older latches this one is a real seqlock: `gen` is odd while the
 * writer is inside the payload and even when it is stable. A reader that sees
 * an odd `gen`, or a different `gen` after its copy, has a torn copy and must
 * not use it. (The single-increment protocol of NavState cannot detect a copy
 * that overlaps the NEXT publish; four motor requests must not be mixed from
 * two controller ticks.)
 *********************************************************************************************************************/
#ifndef CTRLLATCH_H
#define CTRLLATCH_H

#include "Ifx_Types.h"

/* status bits */
#define CTRLLATCH_ST_DRIVE        0x00000001u  /* controller saw ctrlMode = drive: integrator live          */
#define CTRLLATCH_ST_NAV_INVALID  0x00000002u  /* IMU absent / AHRS not running / non-finite estimate       */
#define CTRLLATCH_ST_TILT         0x00000004u  /* |roll| or |pitch| beyond the tilt limit                   */
#define CTRLLATCH_ST_STALE        0x00000008u  /* set by the READER (CtrlLatch_request) in its snapshot:
                                                * no new publish for CTRLLATCH_STALE_CALLS calls -- the
                                                * flight core is silent. Never written into the latch.     */
#define CTRLLATCH_ST_FAULTS       (CTRLLATCH_ST_NAV_INVALID | CTRLLATCH_ST_TILT | CTRLLATCH_ST_STALE)

/* CtrlLatch_request(): calls without a new publish before the request counts
 * as dead. The controller publishes at the IMU rate (1014 Hz), the reader
 * runs at 1 kHz: 5 calls = 5 ms without the flight core. */
#define CTRLLATCH_STALE_CALLS     5u

typedef struct
{
    uint32  gen;        /**< seqlock counter: even = stable, odd = writer inside      */
    float32 rpm[4];     /**< rotor-speed request M1..M4 [rpm], 0 = no request         */
    float32 tau[3];     /**< torque demand roll/pitch/yaw [Nm]                        */
    float32 omSoll[3];  /**< rate setpoint p/q/r [rad/s]                              */
    float32 tauI[3];    /**< rate-loop integrator state [Nm]                          */
    float32 phiSoll[3]; /**< attitude setpoint in force roll/pitch/yaw [rad]          */
    uint32  status;     /**< CTRLLATCH_ST_* bits                                      */
    uint32  pad[2];     /**< 80 bytes: SharedRam.h rule 2, own 64-bit lines           */
} CtrlLatch_t;

/** Defined (with #pragma section) in SharedRam.c. */
extern volatile CtrlLatch_t g_ctrlLatch;

/** Publish one controller tick. CPU1 (the flight core) ONLY.
 *  gen -> odd, barrier, payload, barrier, gen -> even. */
void CtrlLatch_publish(const float32 rpm[4], const float32 tau[3],
                       const float32 omSoll[3], const float32 tauI[3],
                       const float32 phiSoll[3], uint32 status);

/** Read one consistent snapshot. Any core. One retry; FALSE = torn twice,
 *  *out must not be used then. */
boolean CtrlLatch_get(CtrlLatch_t *out);

/** CPU0, exactly once per DShot tick: the rotor-speed requests for Motor_task.
 *  TRUE  = rpm[] holds the controller's current request (rounded, 0..65535).
 *  FALSE = nothing usable, rpm[] is all zero: no new publish for
 *          CTRLLATCH_STALE_CALLS calls, or the controller flags NAV_INVALID /
 *          TILT. A torn read is not a request and not a failure by itself: it
 *          counts as "no new publish", the last consistent snapshot holds
 *          until the stale limit.
 *  *snap receives the last consistent snapshot either way (for logging), with
 *  CTRLLATCH_ST_STALE added to its status while the flight core is silent. */
boolean CtrlLatch_request(uint16 rpm[4], CtrlLatch_t *snap);

#endif /* CTRLLATCH_H */
