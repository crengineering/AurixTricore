/**********************************************************************************************************************
 * \file Ahrs.h
 * \brief Attitude estimator — quaternion Mahony filter over accel + gyro + mag.
 *
 * Answers "which way is the board pointing", which everything else needs: the
 * vertical channel cannot separate gravity from acceleration without it, and
 * the horizontal channel cannot tell north from forward.
 *
 * Method: explicit complementary filter on SO(3) (Mahony). The gyro integrates
 * the quaternion — fast, smooth, and drifting. Two vectors that never drift
 * pull it back:
 *   the accelerometer knows where DOWN is, which fixes roll and pitch
 *   the magnetometer knows where NORTH is, which fixes yaw
 * Neither can fix the other's axis: gravity is invariant under rotation about
 * itself, which is exactly why the old accel+gyro AHRS had an unbounded yaw.
 *
 * The gyro bias falls out of the integral term for free, and is the reason
 * this is a Mahony filter rather than a plain complementary one.
 *
 * FRAME. Body is NED: x forward, y right, z DOWN. The quaternion rotates BODY
 * to NED. Euler angles follow the aerospace 3-2-1 (yaw, then pitch, then roll)
 * convention, so pitch is bounded at +/-90 deg and is clamped there.
 *
 * MOUNTING. Both breakouts sit in their own orientation and neither matches
 * the body frame. The transforms are AHRS_MOUNT_* (IMU) and AHRS_MAG_MOUNT_*
 * (magnetometer) in Ahrs.c — negate ZERO or TWO axes, never one, or the frame
 * turns left-handed and the gyro (a pseudovector) starts disagreeing with the
 * accelerometer about which way is up.
 *
 *   BENCH CHECK after any remount, board held still:
 *     level, chip up      -> roll ~0, pitch ~0
 *     nose up             -> pitch ~ +90 deg
 *     right side down     -> roll  ~ +90 deg
 *     rotate level by 90  -> yaw advances by 90, |B| unchanged
 *
 * HARD IRON. The magnetometer sees the board's own magnetics — the DC-DC, the
 * copper pour, anything ferrous — as a constant offset added to Earth's field.
 * Uncorrected it is a heading error that varies with heading, which no amount
 * of filtering removes. Offsets live in Xcp_Nvm (persistent, per board) and
 * default to zero; tools/mag_cal.py computes them from a rotation record.
 *********************************************************************************************************************/
#ifndef AHRS_H
#define AHRS_H

#include "Ifx_Types.h"

/** Estimator lifecycle. */
typedef enum
{
    AHRS_CALIBRATING = 0,   /**< averaging the gyro bias; hold the board still */
    AHRS_ALIGNING    = 1,   /**< bias done, waiting for a usable accel vector  */
    AHRS_RUNNING     = 2,   /**< tracking; roll/pitch valid                    */
    AHRS_NO_SENSOR   = 3    /**< IMU absent; outputs frozen                    */
} Ahrs_State;

/** Everything the estimator knows, copied out on every Ahrs_update(). */
typedef struct
{
    float32 q[4];          /**< body->NED quaternion, w/x/y/z, unit norm      */
    float32 rollRad;       /**< +right side down                              */
    float32 pitchRad;      /**< +nose up, clamped to +/-90 deg                */
    float32 yawRad;        /**< 0..2pi, TRUE north when declination is set    */
    float32 rate[3];       /**< bias-corrected body rates p/q/r [rad/s]       */
    float32 gyroBias[3];   /**< estimated gyro bias, body frame [deg/s]       */
    float32 accNed[3];     /**< acceleration in NED with gravity removed
                            *   [m/s^2] — THE input to the channel filters    */
    float32 accMagG;       /**< |a| [g]; 1.0 at rest, the health check        */
    float32 magFieldG;     /**< |B| after hard-iron correction [gauss]        */
    uint8   state;         /**< Ahrs_State                                    */
    uint8   accTrusted;    /**< 1 while |a| is close enough to 1 g to use     */
    uint8   magTrusted;    /**< 1 while |B| is plausible and being used       */
    uint8   biasDegraded;  /**< 1 if the boot gyro-bias calibration hit its
                            *   deadline instead of completing a still window,
                            *   i.e. the board was moving at power-on. The
                            *   estimate still runs; it is just less good      */
} Ahrs_Values;

/** Reset the estimator and start a fresh gyro-bias calibration. */
void Ahrs_init(void);

/** Feed one IMU sample and run the filter.
 *  \param out    receives the current estimate (never NULL)
 *  \param acc    acceleration [g],     SENSOR frame, X/Y/Z
 *  \param gyro   angular rate [deg/s], SENSOR frame, X/Y/Z
 *  \param dt     measured time since the previous call [s]
 *  \param valid  FALSE when the IMU read failed — the estimate is frozen
 *                rather than integrating a stale sample */
void Ahrs_update(Ahrs_Values *out, const float32 acc[3], const float32 gyro[3],
                 float32 dt, boolean valid);

/** Rotate a vector from NED into the BODY frame using the current attitude.
 *
 *  Exposed because the flight controller wants VELOCITY IN BODY AXES
 *  (`v_b_ist` in src/asw/flight_ctrl.h) while the navigation filter estimates
 *  it in NED. Somebody has to do that rotation, and doing it here reuses the
 *  same quaternion and the same tested code the filter runs on, rather than a
 *  second copy that can drift out of agreement with it.
 *
 *  \param vNed   input vector in NED
 *  \param vBody  receives the same vector in body axes (may not alias vNed) */
void Ahrs_nedToBody(const float32 vNed[3], float32 vBody[3]);

/** Latch a magnetometer sample for the next Ahrs_update() to consume.
 *  Hard-iron correction and the mounting transform happen here.
 *  \param mag    field [gauss], SENSOR frame, X/Y/Z
 *  \param valid  FALSE when the read failed (sample ignored) */
void Ahrs_setMag(const float32 mag[3], boolean valid);

#endif /* AHRS_H */
