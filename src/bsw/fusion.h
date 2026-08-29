/**********************************************************************************************************************
 * \file fusion.h
 * \brief Navigation filter — position, velocity and sensor biases in NED.
 *
 * Combines five sensors that fail in different ways:
 *
 *   accelerometer  fast, but integrate it twice and a 0.018 m/s^2 offset
 *                  (measured on this board) becomes 32 m in 60 s
 *   barometer      never drifts fast, but WANDERS: 0.19 m/min measured while
 *                  the board sat still on the desk, and 0.020 m of scatter
 *   magnetometer   absolute heading, but only where the iron is not
 *   GNSS           absolute position, but 1 Hz, metres of noise, and gone
 *                  the moment there is a roof overhead
 *   gyro           smooth attitude, but drifts without the other two
 *
 * Attitude lives in Ahrs.h. This file takes its NED acceleration output and
 * runs three structurally identical Kalman channels — DOWN, NORTH, EAST.
 *
 * EACH CHANNEL HAS FOUR STATES: [position, velocity, accelBias, measBias].
 *
 *   accelBias  the accelerometer offset in that NED direction. Without it the
 *              filter fights the offset forever with the position correction
 *              and the velocity carries the error instead — which is exactly
 *              what the 2-state version did (+0.005 m/s at rest, creeping).
 *   measBias   the offset of the RELATIVE position sensor against the ABSOLUTE
 *              one. Only the DOWN channel uses it: the barometer measures
 *              (d + measBias) and GNSS altitude measures d, so the pair makes
 *              the barometer's weather drift observable and it stops leaking
 *              into altitude. On NORTH and EAST there is no relative sensor,
 *              so the state gets no process noise, is never updated, and
 *              stays exactly zero.
 *
 * Frame is NED throughout, so **d is POSITIVE DOWNWARD** and so is v_d.
 * Barometric and GNSS altitude both count upward; the sign flip happens once,
 * where each enters (Fusion_setBaroAlt, Fusion_setGnss).
 *
 * The horizontal origin is the first usable GNSS fix. Everything horizontal is
 * metres from there on a flat tangent plane, which is good to well under a
 * metre out to several km — far past anything this airframe will do.
 *********************************************************************************************************************/
#ifndef FUSION_H
#define FUSION_H

#include "Ifx_Types.h"

/** Filter outputs, copied out on every Fusion_update() for publishing. */
typedef struct
{
    /* --- vertical channel --------------------------------------------- */
    float32 a_D;        /**< accel along NED down, gravity removed [m/s^2]   */
    float32 a_d;        /**< position down, relative to the origin [m]       */
    float32 a_v_d;      /**< velocity down [m/s]                             */
    float32 accBiasD;   /**< estimated accelerometer bias, down [m/s^2]      */
    float32 baroBias;   /**< barometer offset vs GNSS altitude [m].
                         *   ⚠️ NOT zero without GNSS: it carries process
                         *   noise, so it random-walks (measured: 3 mm in
                         *   10 min). Only (a_d + baroBias) is observable from
                         *   a barometer alone; a GNSS fix is what pins the
                         *   split, not what starts the state moving.
                         *   Bounded to +/-FUSION_MEASB_MAX               */
    float32 innov;      /**< last barometer innovation [m] — the tuning signal */
    float32 p00;        /**< variance of a_d [m^2]; watch it converge        */

    /* --- horizontal channels ------------------------------------------ */
    float32 a_N;        /**< accel along NED north [m/s^2]                   */
    float32 a_E;        /**< accel along NED east  [m/s^2]                   */
    float32 posN;       /**< metres north of the tangent-plane origin        */
    float32 posE;       /**< metres east                                     */
    float32 velN;       /**< velocity north [m/s]                            */
    float32 velE;       /**< velocity east  [m/s]                            */
    float32 accBiasN;   /**< estimated accelerometer bias, north [m/s^2]     */
    float32 accBiasE;   /**< estimated accelerometer bias, east  [m/s^2]     */
    float32 innovN;     /**< last GNSS north innovation [m]                  */
    float32 innovE;     /**< last GNSS east innovation [m]                   */
    float32 innovVelN;  /**< last GNSS velocity innovation, north [m/s]      */
    float32 innovVelE;  /**< last GNSS velocity innovation, east  [m/s]      */
    float32 pNN;        /**< variance of posN [m^2]                          */

    /* --- tangent-plane origin ----------------------------------------- */
    float32 originLatDeg;
    float32 originLonDeg;
    float32 originAltM;

    /* --- counters and status ------------------------------------------ */
    uint32  rejects;      /**< barometer samples rejected by the outlier gate */
    uint32  resets;       /**< vertical re-acquisitions after a reject run.
                           *   Any non-zero value means the filter diverged
                           *   and recovered — worth knowing about            */
    uint32  gnssRejects;  /**< GNSS fixes rejected by the outlier gate        */
    uint32  gnssUpdates;  /**< GNSS fixes actually fused                      */
    uint32  covResets;    /**< covariance re-initialisations forced by the
                           *   numerical health check. Zero in normal
                           *   operation; a non-zero value with healthy
                           *   sensors is a bug, not a tuning problem.
                           *   ⚠️ NOT guaranteed zero under corrupt input —
                           *   measured 35 over 30 s of deliberately corrupt
                           *   barometer, which is the guard doing its job    */
    uint32  gnssITow;     /**< iTOW of the last fix actually fused [ms]. The
                           *   direct check on the new-fix guard: if this is
                           *   frozen while gnssUpdates also stops, iTOW is not
                           *   being decoded and every later fix is discarded
                           *   as a duplicate                                  */
    uint32  dropped;      /**< measurements refused at the input because they
                           *   were NaN, infinite or absurd. MUST stay 0 with
                           *   healthy sensors -- a rising count is a failing
                           *   sensor, and without it such a sample is dropped
                           *   silently and a dead barometer looks like a
                           *   healthy one                                     */
    uint32  gnssDupes;    /**< polls that carried a fix already fused. Expected
                           *   to be small but NON-ZERO: the 10 Hz poll and the
                           *   10 Hz solution are on independent clocks         */
    uint8   verticalOk;   /**< 1 once the barometer has anchored the channel  */
    uint8   horizontalOk; /**< 1 once the tangent-plane origin is set         */
    uint8   originSet;    /**< 1 once a usable fix defined the origin         */
    uint8   reserved;
} FusionValues;

/** Reset every channel, the covariances, and both position references. */
void Fusion_init(void);

/** Run one predict step on all three channels, then any corrections whose
 *  samples have arrived.
 *  \param fusion  receives the current estimate (never NULL)
 *  \param accNed  acceleration in NED with gravity removed [m/s^2], from
 *                 Ahrs_Values.accNed
 *  \param dt      measured time since the previous call [s]
 *  \param valid   FALSE when the IMU read failed or the attitude is not yet
 *                 usable — the estimate is frozen rather than integrating a
 *                 stale sample */
void Fusion_update(FusionValues *fusion, const float32 accNed[3], float32 dt, boolean valid);

/** Latch a barometric altitude for the next Fusion_update() to consume.
 *  \param altM   altitude above the NVM sea-level reference [m], positive UP
 *  \param valid  FALSE when the barometer read failed (sample ignored) */
void Fusion_setBaroAlt(float32 altM, boolean valid);

/** Latch a GNSS fix for the next Fusion_update() to consume. Ignored unless
 *  \p iTOW differs from the previous call.
 *
 *  The receiver is configured for a 100 ms measurement rate and one NAV-PVT per
 *  epoch (CFG_RATE_MEAS / CFG_RATE_NAV in GnssM9N.c), so solutions arrive at
 *  10 Hz — the same rate this is polled at, but on an independent clock. The
 *  guard therefore skips the occasional duplicate rather than nine out of ten
 *  of them, and gnssDupes counts those.
 *
 *  ⚠️ Successive 10 Hz solutions are NOT independent measurements: the
 *  receiver runs its own filter at the nav rate, so consecutive fixes share
 *  most of their information. Fusing every one at face value overstates the
 *  evidence and drives the covariance below the truth. See docs/FUSION.md.
 *  \param latDeg1e7  latitude  [1e-7 deg], the receiver's native integer
 *  \param lonDeg1e7  longitude [1e-7 deg] — integers on purpose, float32
 *                    degrees only resolve to about 0.4 m at this latitude
 *  \param altM       height above mean sea level [m], positive UP
 *  \param speedMps   2-D ground speed [m/s]
 *  \param headingDeg heading of motion [deg]
 *  \param hAccM      horizontal accuracy estimate [m]
 *  \param iTOW       GPS time of week [ms] — the new-fix marker
 *  \param valid      FALSE unless the receiver says the solution is usable */
void Fusion_setGnss(sint32 latDeg1e7, sint32 lonDeg1e7, float32 altM,
                    float32 speedMps, float32 headingDeg, float32 hAccM,
                    uint32 iTOW, boolean valid);

#endif /* FUSION_H */
