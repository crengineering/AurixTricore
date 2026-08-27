/**********************************************************************************************************************
 * \file NavTask.h
 * \brief THE flight chain: IMU read -> AHRS -> fusion -> NavState.
 *
 * T11 (docs/REFACTORING_PLAN.md): moved out of Cpu0_Main.c's Task_Imu
 * verbatim, publishing via NavState_publish() instead of calling
 * measurementsSetFusion() directly -- Housekeeping_100ms is now the one that
 * reads NavState and publishes to XCP. Still registered on CPU0: T11 only
 * changes WHERE this code lives, not which core runs it, so a torn cross-core
 * read is structurally impossible here and a regression can only be a
 * refactoring bug, never a coherency bug. T12 is the actual migration.
 *********************************************************************************************************************/
#ifndef NAVTASK_H
#define NAVTASK_H

#include "Ifx_Types.h"

/** FusionCal_init, Ahrs_init, Fusion_init, NavState_init -- in that order,
 *  once at boot, before the scheduler starts. */
void NavTask_init(void);

/** The flight chain, one tick: IMU read, AHRS update, fusion update,
 *  NavState_publish. Registered at 20 ms. */
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
