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
#include "SysTime.h"
#include "ImuInt.h"

/* Below NAVTASK_DT_MIN_S the tick fired twice inside one STM period (nothing
 * to integrate), above NAVTASK_DT_MAX_S the previous tick was so late that
 * integrating the gap would put a real offset into attitude/position rather
 * than a measurement.
 *
 * T14 (docs/REFACTORING_PLAN.md §3.8, Risk 10): NAVTASK_DT_MIN_S was 0.001f,
 * sized against the 50 Hz task period -- and a SILENT KILLER at the measured
 * 985 us IMU interval, because 985 us < 1 ms rejects EVERY tick, permanently
 * FALSE-ing ahrsInputOk/fusionInputOk with no error anywhere: the estimator
 * just stops. 0.0002f (200 us) still catches a genuine double dispatch and
 * clears the measured 985 us period by 5x, at any rate this task ever runs
 * at. */
#define NAVTASK_DT_MIN_S   (0.0002f)
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

/* Running total of Icm42688_plausible()'s per-sample liveness, published
 * verbatim in NavState_t.imuLiveness (NEVER reset here after boot) -- see
 * that field's comment. Housekeeping_100ms does the resetting-by-diffing;
 * this side just keeps adding. CPU1-only, same as every other NavTask.c
 * static. */
static float32 s_imuLivenessAccum;

void NavTask_init(void)
{
    NavState_init();        /* the publish target, before anything publishes */
    FusionCal_init();        /* estimator tuning defaults, BEFORE the filters */
    Ahrs_init();              /* start the gyro-bias calibration; hold still  */
    Fusion_init();            /* zero every channel state and covariance      */
    s_imuLivenessAccum = 0.0f;
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

    /* I5, docs/IMU_INTERRUPT.md 5.5: how stale is the sample this task is
     * about to fetch, measured against the last INT1 edge the ISR saw. At
     * today's 50 Hz this settles at up to ~20 ms on its own -- that number,
     * not an opinion, is the evidence for D5. Read before the SPI burst so it
     * reflects the wait, not the transfer time. g_imuDrdyLastTicks starts at
     * 0 and only updates once ImuInt_init() has seen a real edge; a giant
     * value before the wire/ISR are alive is expected, not a fault. */
    g_imuDrdyStaleTicks = (uint32)SysTime_getTicks() - g_imuDrdyLastTicks;

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
    /* Written as an if/else into a boolean local, not a direct `&&`
     * assignment: cppcheck's MISRA 10.3 does not recognise `boolean` as an
     * essentially-Boolean type, so it flags a `&&`/comparison result stored
     * straight into one as a different essential type category. Same idiom
     * as navTask_dtValid()/NavTask_inputValid() just above and below. */
    ahrsInputOk = FALSE;
    if ((navTask_dtValid(elapsedTime) != FALSE) && (present != FALSE))
    {
        ahrsInputOk = TRUE;
    }
    Ahrs_update(&ahrs, sample.acc, sample.gyro, elapsedTime, ahrsInputOk);

    /* Gate the navigation filter on the attitude being usable, not merely on
     * the IMU answering. While the AHRS is still averaging the gyro bias or
     * waiting to align, its projection is meaningless and integrating it would
     * put a real offset into the velocity before the barometer ever sees it. */
    fusionInputOk = NavTask_inputValid(elapsedTime, present, ahrs.state);
    Fusion_update(&fusion, ahrs.accNed, elapsedTime, fusionInputOk);

    /* Raw sample + an accumulated liveness sum ride along in the SAME publish
     * as the fusion output (T12 blocker, docs/REFACTORING_PLAN.md 3.7):
     * measurementsSetImu() writes g_xcpData (CPU0 DSPR) and PeriphDiag_report()
     * writes PeriphDiag's plain non-volatile s_periph[] -- calling either one
     * from here, once NavTask_step runs on CPU1, would make both a two-writer
     * object racing against CPU0's Housekeeping_100ms/PeriphDiag_update. So
     * this task no longer calls them at all; it only accumulates and
     * publishes, and Housekeeping_100ms (CPU0) makes both calls from the
     * NavState snapshot. Icm42688_plausible()'s instantaneous liveness is
     * summed, never reset, so Housekeeping's diff of two reads sees every
     * tick's contribution instead of only the one tick nearest its 100 ms
     * boundary -- see NavState_t.imuLiveness. */
    {
        float32 sampleLiveness = 0.0f;

        (void)Icm42688_plausible(&sample, &sampleLiveness);
        s_imuLivenessAccum += sampleLiveness;
    }

    /* Publish for Housekeeping_100ms to pick up (NavState_get) and forward to
     * XCP -- replaces the direct measurementsSetFusion() call this task made
     * before T11. Still a same-core write in T11 (NavTask_step is CPU0 until
     * T12), so this is exercising the protocol, not yet the cross-core case. */
    NavState_publish(&ahrs, &fusion, elapsedTime, present,
                      sample.acc, sample.gyro, sample.tempC,
                      s_imuLivenessAccum);
}
