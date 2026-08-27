/**********************************************************************************************************************
 * \file SensorTask.h
 * \brief Periodic barometer / magnetometer / GNSS tasks: read, convert,
 *        publish, diagnose. Registered on CPU0.
 *
 * One clear responsibility: everything about the off-chain sensors that used
 * to sit inline in Cpu0_Main.c (T6, docs/REFACTORING_PLAN.md). Each task
 * reads its device, publishes the result (measurements + the latch the
 * estimator consumes), and reports plausibility to PeriphDiag. The flight
 * chain itself (IMU -> AHRS -> fusion) is NavTask's, not this module's.
 *********************************************************************************************************************/
#ifndef SENSORTASK_H
#define SENSORTASK_H

/** Declare the baro/mag/GNSS slots fitted to PeriphDiag. Call once at boot,
 *  after Bmp581_init()/Mmc5983_init()/GnssM9N_init() have run. */
void SensorTask_init(void);

/** Read the BMP581, publish pressure/temperature/altitude, latch the
 *  altitude for the navigation filter, report plausibility. 20 ms (50 Hz). */
void SensorTask_baro(void);

/** Read the MMC5983MA, publish the field, latch it for the attitude filter,
 *  report plausibility. 20 ms (50 Hz). */
void SensorTask_mag(void);

/** Read the GNSS receiver, publish the fix, latch it for the navigation
 *  filter, report plausibility. 100 ms (10 Hz). */
void SensorTask_gnss(void);

#endif /* SENSORTASK_H */
