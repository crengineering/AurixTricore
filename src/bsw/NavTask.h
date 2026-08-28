/**********************************************************************************************************************
 * \file NavTask.h
 * \brief THE flight chain: IMU read -> AHRS -> fusion -> NavState.
 *
 * T11 (docs/REFACTORING_PLAN.md): moved out of Cpu0_Main.c's Task_Imu
 * verbatim, publishing via NavState_publish() instead of calling
 * measurementsSetFusion() directly -- Housekeeping_100ms is now the one that
 * reads NavState and publishes to XCP.
 *
 * T12: the actual migration. NavTask_init()/NavTask_step() now run on CPU1
 * (core1_main, after its sync barrier) -- the crossing NavState_publish()
 * exercises is a real cross-core one from here on, not the same-core dry run
 * T11 was. NavTask_step() also stopped calling measurementsSetImu()/
 * PeriphDiag_report() directly (both write CPU0-owned state -- g_xcpData is
 * CPU0 DSPR, PeriphDiag's s_periph[] is a plain non-volatile static); the raw
 * sample and an accumulated liveness now ride in the NavState payload and
 * Housekeeping_100ms makes those calls instead (NavState.h, Housekeeping.c).
 *********************************************************************************************************************/
#ifndef NAVTASK_H
#define NAVTASK_H

#include "Ifx_Types.h"

/** FusionCal_init, Ahrs_init, Fusion_init, NavState_init -- in that order,
 *  once at boot, before the scheduler starts. */
void NavTask_init(void);

/** The flight chain, one tick: IMU read, AHRS update, fusion update,
 *  NavState_publish. Registered at SCHED_US(500) (T15) -- a 2 kHz poll well
 *  above the sensor's measured ~1014.2 Hz DRDY rate; the body returns almost
 *  immediately unless a new edge (or a timeout, see NavTask.c) is pending, so
 *  the actual work rate tracks the sensor, not the poll period. */
void NavTask_step(void);

/** Whether this tick's inputs are trustworthy enough to feed the fusion
 *  filter: the dt window (bounded, NaN-safe -- an out-of-range or NaN dtS
 *  returns FALSE by construction, it is never accepted by omission the way
 *  "reject if dt < lo || dt > hi" would silently pass a NaN through),
 *  imuPresent, and the AHRS having left calibration (AHRS_RUNNING). Split out
 *  of NavTask_step for the host test -- this is where the NaN/validity logic
 *  lives (docs/REFACTORING_PLAN.md §4). */
boolean NavTask_inputValid(float32 dtS, boolean imuPresent, uint8 ahrsState);

#endif /* NAVTASK_H */
