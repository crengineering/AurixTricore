/**********************************************************************************************************************
 * \file Ahrs.h
 * \brief Attitude estimator (AHRS) — fuses the MPU-6050 accelerometer and gyro
 *        into roll/pitch/yaw angles for the flight controller.
 *
 * Produces exactly the two feedback vectors src/asw/flight_ctrl.h expects:
 *   phi_ist[3] = [roll, pitch, yaw] in RAD   -> att_ctrl_step()
 *   om_ist[3]  = [p, q, r]          in RAD/S -> rate_ctrl_step()
 * The driver reports deg and deg/s; the conversion happens here so the ASW never
 * has to care.
 *
 * Method: complementary filter. The gyro carries fast motion but drifts; the
 * accelerometer is noisy and corrupted by manoeuvres but never drifts, because
 * gravity is always exactly there. Blending them with a time constant
 * AHRS_TAU_S gives an estimate that is both responsive and drift-free.
 *
 *   attitude = alpha * (attitude + gyro * dt) + (1 - alpha) * attitude_from_accel
 *   alpha    = tau / (tau + dt)
 *
 * THREE THINGS TO KNOW BEFORE THIS FLIES
 *
 * 1. YAW DRIFTS AND CANNOT BE FIXED HERE. Rotating about the gravity vector does
 *    not change gravity's projection on any axis, so yaw is unobservable from
 *    accelerometer + gyro. Ahrs_getAttitude()[2] is a pure gyro integral: fine
 *    for a heading-hold rate loop, useless as an absolute heading. Absolute yaw
 *    needs the planned MMC5983MA magnetometer (PINNING.md) or GPS course.
 *
 * 2. THE MOUNTING TRANSFORM IS NOT SET. Angles are computed in the SENSOR frame
 *    (MPU axes as marked on the GY-521, Z up when the chip faces up), because
 *    the airframe mounting is not decided yet. flight_ctrl.h uses NED (z DOWN).
 *    Before flight, set the AHRS_MOUNT_* mapping below to rotate sensor axes
 *    into the body frame — otherwise the controller will drive the wrong way.
 *    This is deliberately one small, obvious place to change.
 *
 * 3. GYRO BIAS IS CALIBRATED AT EVERY BOOT, not stored. Bias moves with
 *    temperature and part-to-part, so a frozen constant goes stale; the observed
 *    +21.7 deg/s on this unit would integrate to a 21 deg error in one second.
 *    Calibration needs the board held STILL for ~2 s after reset and restarts
 *    itself if it detects motion.
 *********************************************************************************************************************/
#ifndef AHRS_H
#define AHRS_H

#include "Ifx_Types.h"

/** Estimator lifecycle. */
typedef enum
{
    AHRS_CALIBRATING = 0,   /**< averaging the gyro bias; hold the board still */
    AHRS_RUNNING     = 1,   /**< bias valid, attitude is being tracked         */
    AHRS_NO_SENSOR   = 2    /**< IMU absent; outputs held at zero              */
} Ahrs_State;

/** Reset the estimator and start a fresh gyro-bias calibration. */
void Ahrs_init(void);

/** Feed one IMU sample.
 *  \param acc  acceleration [g],     sensor frame, order X/Y/Z
 *  \param gyro   angular rate [deg/s], sensor frame, order X/Y/Z
 *  \param dt     elapsed time since the previous call [s]
 *  \param valid  FALSE when the IMU read failed — the estimate is frozen and
 *                the state falls back to AHRS_NO_SENSOR. */
void Ahrs_update(const float32 acc[3], const float32 gyro[3], float32 dt, boolean valid);

/** \return current estimator state. */
Ahrs_State Ahrs_getState(void);

/** Attitude [roll, pitch, yaw] in RAD — flight_ctrl.h phi_ist. Yaw drifts (§1). */
void Ahrs_getAttitude(float32 phi[3]);

/** Bias-corrected body rates [p, q, r] in RAD/S — flight_ctrl.h om_ist. */
void Ahrs_getRates(float32 om[3]);

/** Measured gyro bias [deg/s], sensor frame. Exposed for diagnosis: a healthy
 *  MPU-6050 sits within a few deg/s, and a large value that changes between
 *  boots means calibration ran while the board was moving. */
void Ahrs_getGyroBias(float32 bias[3]);

/** \return TRUE while the accelerometer is being trusted, i.e. |a| is close
 *  enough to 1 g for gravity to dominate. FALSE during brisk motion, when the
 *  filter coasts on the gyro alone. */
boolean Ahrs_isAccelTrusted(void);

#endif /* AHRS_H */
