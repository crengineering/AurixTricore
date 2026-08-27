/**********************************************************************************************************************
 * \file Housekeeping.c
 * \brief 100 Hz off-chain bookkeeping — see Housekeeping.h.
 *********************************************************************************************************************/
#include "Housekeeping.h"
#include "Measurements.h"
#include "Diagnostics.h"
#include "gpio.h"
#include "NavState.h"
#include "Icm42688.h"
#include "PeriphDiag.h"

/* T11: the last snapshot NavState_get() successfully returned. Kept here,
 * not in NavState.c, because "what to do when a read fails" is a policy
 * decision for the one reader, not part of the publish/get protocol itself
 * -- NavState_get() never writes shared state, but it is allowed to write
 * into a caller-owned *out even on a torn attempt, so this file is what
 * actually enforces "keep the previous snapshot on FALSE". */
static NavState_t s_lastNav;

/* T12 blocker fix (docs/REFACTORING_PLAN.md 3.7): the previous read of
 * NavState_t.imuLiveness, so this file can turn NavTask_step's running,
 * never-reset accumulator back into a per-window DELTA before handing it to
 * PeriphDiag_report() -- see that field's comment in NavState.h for why a
 * diff, not the raw accumulator, is what preserves the stuck-sensor
 * detector's per-tick sensitivity. */
static float32 s_lastImuLiveness;

void Housekeeping_init(void)
{
    NavState_t zero = { 0 };
    s_lastNav = zero;
    s_lastImuLiveness = 0.0f;
}

void Housekeeping_100ms(void)
{
    NavState_t candidate;

    if (NavState_get(&candidate) != FALSE)
    {
        s_lastNav = candidate;
    }
    /* else: torn read (or, before NavTask's first publish, none yet) --
     * keep s_lastNav exactly as it was; see Housekeeping.h. */

    /* T12 blocker fix (docs/REFACTORING_PLAN.md 3.7): raw IMU sample and
     * diagnostics, now published from the NavState snapshot instead of
     * directly by NavTask_step -- calling measurementsSetImu()/
     * PeriphDiag_report() from CPU1 would write g_xcpData (CPU0 DSPR) and
     * PeriphDiag's plain non-volatile s_periph[] from the wrong core (see
     * NavTask.c and NavState.h). Must run before diagnosticsUpdate() below,
     * same as SensorTask_baro/mag/gnss report before Housekeeping runs in
     * the scheduler's dispatch order (Cpu0_Main.c). */
    {
        boolean         imuPresentB = (s_lastNav.imuPresent != 0u) ? TRUE : FALSE;
        Icm42688_Sample imuSample;
        float32         instLiveness  = 0.0f;
        float32         livenessDelta;
        boolean         imuPlausible;

        imuSample.acc[0]  = s_lastNav.imuAcc[0];
        imuSample.acc[1]  = s_lastNav.imuAcc[1];
        imuSample.acc[2]  = s_lastNav.imuAcc[2];
        imuSample.gyro[0] = s_lastNav.imuGyro[0];
        imuSample.gyro[1] = s_lastNav.imuGyro[1];
        imuSample.gyro[2] = s_lastNav.imuGyro[2];
        imuSample.tempC   = s_lastNav.imuTempC;

        /* Same band check NavTask_step used to run itself (Icm42688.c) --
         * one formula, not two. Its own liveness output is discarded: the
         * accumulated DELTA below is what preserves per-tick sensitivity,
         * not this single instant's sum (NavState.h imuLiveness comment). */
        imuPlausible = Icm42688_plausible(&imuSample, &instLiveness);

        livenessDelta     = s_lastNav.imuLiveness - s_lastImuLiveness;
        s_lastImuLiveness = s_lastNav.imuLiveness;

        measurementsSetImu(imuPresentB, imuSample.acc, imuSample.gyro, imuSample.tempC);
        PeriphDiag_report(PERIPH_DIAG_IMU, imuPresentB, imuPlausible, livenessDelta);
    }

    /* imuPresent is uint32 in NavState_t (LMU no-sub-word rule, SharedRam.h)
     * but boolean in measurementsSetFusion(); compare explicitly rather than
     * pass the uint32 through an implicit narrowing conversion (MISRA
     * 10.3/10.8 -- same fix as the coreLoadPmil/coreAlive copy in
     * Measurements.c). */
    measurementsSetFusion(&s_lastNav.fusion, &s_lastNav.ahrs,
                           (s_lastNav.imuPresent != 0u) ? TRUE : FALSE,
                           s_lastNav.dtS);

    measurementsUpdate();
    if (diagnosticsUpdate() != FALSE)
    {
        gpio_write(GPIO_P_00_0, GPIO_STATE_ON);
    }
    else
    {
        gpio_write(GPIO_P_00_0, GPIO_STATE_OFF);
    }
    gpio_calApply();        /* XCP overrides win over the diagnostics write above */

    measurementsSetSystemLoad();   /* per-core exec time + Ethernet utilisation */
}
