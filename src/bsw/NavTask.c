/**********************************************************************************************************************
 * \file NavTask.c
 * \brief THE flight chain -- see NavTask.h.
 *********************************************************************************************************************/
#include "NavTask.h"
#include "Icm42688.h"
#include "Ahrs.h"
#include "fusion.h"
#include "FusionCal.h"
#include "NavState.h"
#include "Measurements.h"
#include "PeriphDiag.h"
#include "SysTime.h"

/* Same window as the original Task_Imu (Cpu0_Main.c, pre-T11): below
 * NAVTASK_DT_MIN_S the tick fired twice inside one STM period (nothing to
 * integrate), above NAVTASK_DT_MAX_S the previous tick was so late that
 * integrating the gap would put a real offset into attitude/position rather
 * than a measurement. */
#define NAVTASK_DT_MIN_S   (0.001f)
#define NAVTASK_DT_MAX_S   (0.2f)

/* NaN-safe by construction: written as "is dtS INSIDE the window", not "is
 * dtS outside the window". NaN compares false against every relational
 * operator, so the ORIGINAL Cpu0_Main.c form -- `(dt < lo) || (dt > hi)` to
 * mean "reject" -- let a NaN dt through as "not rejected", i.e. accepted.
 * This form rejects it: both comparisons below are false for NaN, so `valid`
 * never becomes TRUE. Same bug class as the GNSS input check fixed in
 * fusion.c (MISRA 10.3 commit) and the plausibility bands in Bmp581/Mmc5983/
 * Icm42688 -- NaN is a real value here, not an edge case. */
static boolean navTask_dtValid(float32 dtS)
{
    boolean valid = FALSE;

    if ((dtS >= NAVTASK_DT_MIN_S) && (dtS <= NAVTASK_DT_MAX_S))
    {
        valid = TRUE;
    }

    return valid;
}

boolean NavTask_inputValid(float32 dtS, boolean imuPresent, uint8 ahrsState)
{
    boolean valid = FALSE;

    if ((navTask_dtValid(dtS) != FALSE)
        && (imuPresent != FALSE)
        && (ahrsState == (uint8)AHRS_RUNNING))
    {
        valid = TRUE;
    }

    return valid;
}

void NavTask_init(void)
{
    NavState_init();        /* the publish target, before anything publishes */
    FusionCal_init();        /* estimator tuning defaults, BEFORE the filters */
    Ahrs_init();              /* start the gyro-bias calibration; hold still  */
    Fusion_init();            /* zero every channel state and covariance      */
}

void NavTask_step(void)
{
    Icm42688_Sample sample = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 0.0f };
    boolean         present;
    /* uint32_t, not uint32: SysTime.h is deliberately iLLD-free and takes the
     * stdint type, and on TASKING the two are distinct types of equal width. */
    static uint32_t s_lastTicks = 0u;
    boolean         ahrsInputOk;
    boolean         fusionInputOk;
    FusionValues    fusion;
    Ahrs_Values     ahrs;
    float32         elapsedTime;

    /* Called unconditionally, like the other sensor tasks: Icm42688_read()
     * owns the presence state and uses these calls to probe for a reconnected
     * sensor. */
    present     = Icm42688_read(&sample);
    elapsedTime = SysTime_getTimeElapsedS(&s_lastTicks);

    /* Attitude first, then navigation: the channel filters need acceleration
     * resolved into NED, and only the AHRS can do that. ahrs.state does not
     * exist yet at this point, so this first gate is dt+presence only --
     * NavTask_inputValid's AHRS_RUNNING check applies below, once ahrs.state
     * is an output rather than an unknown. */
    ahrsInputOk = (navTask_dtValid(elapsedTime) != FALSE) && (present != FALSE);
    Ahrs_update(&ahrs, sample.acc, sample.gyro, elapsedTime, ahrsInputOk);

    /* Gate the navigation filter on the attitude being usable, not merely on
     * the IMU answering. While the AHRS is still averaging the gyro bias or
     * waiting to align, its projection is meaningless and integrating it would
     * put a real offset into the velocity before the barometer ever sees it. */
    fusionInputOk = NavTask_inputValid(elapsedTime, present, ahrs.state);
    Fusion_update(&fusion, ahrs.accNed, elapsedTime, fusionInputOk);

    /* Publish for Housekeeping_100ms to pick up (NavState_get) and forward to
     * XCP -- replaces the direct measurementsSetFusion() call this task made
     * before T11. Still a same-core write in T11 (NavTask_step is CPU0 until
     * T12), so this is exercising the protocol, not yet the cross-core case. */
    NavState_publish(&ahrs, &fusion, elapsedTime, present);

    {
        /* Raw angular rate: there is no bias estimator in the tree, so the
         * published value still carries the sensor offset. Unlike the fusion
         * output above, this is the raw sample, which NavState does not
         * carry -- so it still goes straight to Xcp_Data here, same as
         * before T11 and safe for the same reason: still CPU0 writing to
         * CPU0's own DSPR. */
        measurementsSetImu(present, sample.acc, sample.gyro, sample.tempC);

        float32 liveness = 0.0f;
        boolean plausible = Icm42688_plausible(&sample, &liveness);
        PeriphDiag_report(PERIPH_DIAG_IMU, present, plausible, liveness);
    }
}
