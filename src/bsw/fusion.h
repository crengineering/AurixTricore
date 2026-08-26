/**********************************************************************************************************************
 * \file fusion.h
 * \brief Vertical-channel Kalman filter — altitude and vertical speed.
 *
 * Estimates how high the vehicle is and how fast it is climbing or sinking, by
 * combining two sensors that fail in opposite ways:
 *
 *   the accelerometer is fast but drifts — integrate it twice and a 0.039 m/s²
 *     offset becomes 15 m of imaginary descent in 30 s (measured on this board)
 *   the barometer never drifts but is noisy — 0.0197 m of scatter, measured
 *
 * Frame is NED, so **d is POSITIVE DOWNWARD** and so is v_d. Barometric
 * altitude counts upward; the sign flip happens once, in Fusion_setBaroAlt().
 *
 * STAGE 3 OF 4. Two states, [d, v_d]. The accelerometer offset is NOT yet
 * estimated, so the innovation still carries a constant offset — that is
 * expected here and is what stage 4 (adding an accelerometer-bias state) fixes.
 *
 * ⚠️ LEVEL-DESK SCAFFOLD, KNOWN EXPIRY. a_D is computed assuming the board is
 * level, because there is no attitude estimator in the tree yet. At 20 degrees
 * of tilt the error is already 0.6 m/s². The moment roll and pitch exist, a_D
 * must come from the full projection (docs: sensor-fusion course, section 10)
 * and this comment must go.
 *********************************************************************************************************************/
#ifndef FUSION_H
#define FUSION_H

#include "Ifx_Types.h"

/** Filter outputs, copied out on every Fusion_update() for publishing. */
typedef struct
{
    float32 a_D;      /**< accel along NED down, gravity removed [m/s^2]      */
    float32 a_d;      /**< position down, relative to the first baro fix [m]  */
    float32 a_v_d;    /**< velocity down [m/s]                               */
    float32 innov;    /**< last barometer innovation [m] — THE tuning signal  */
    float32 p00;      /**< variance of a_d [m^2]; watch it converge           */
    uint32  rejects;  /**< barometer samples rejected by the outlier gate     */
    uint32  resets;   /**< re-acquisitions after a run of rejections. Any
                       *   non-zero value means the filter diverged and
                       *   recovered -- worth knowing about                   */
} FusionValues;

/** Reset the state, the covariance and the barometric reference. */
void Fusion_init(void);

/** Run one predict step, then a correction if a new barometer sample arrived.
 *  \param fusion  receives the current estimate (never NULL)
 *  \param acc     acceleration [g], SENSOR frame, X/Y/Z
 *  \param dt      measured time since the previous call [s]
 *  \param valid   FALSE when the IMU read failed — the estimate is frozen
 *                 rather than integrating a stale sample */
void Fusion_update(FusionValues *fusion, const float32 acc[3], float32 dt, boolean valid);

/** Latch a barometric altitude for the next Fusion_update() to consume.
 *  \param altM   altitude above the NVM sea-level reference [m], positive UP
 *  \param valid  FALSE when the barometer read failed (sample ignored) */
void Fusion_setBaroAlt(float32 altM, boolean valid);

#endif /* FUSION_H */
