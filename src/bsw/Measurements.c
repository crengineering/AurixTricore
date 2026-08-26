#include "Measurements.h"
#include "Diagnostics.h"
#include "Nvm.h"
#include "Version.h"
#include "CoreStats.h"
#include "EthStats.h"
#include "Dts/Dts/IfxDts_Dts.h"
#include "Pms/Std/IfxPmsEvr.h"
#include "IfxScuWdt.h"
#include "Ifx_Lwip.h"
#include <math.h>


/* The full-scale voltages of the 8-bit PMS monitor ADCs live in the
 * XCP-calibratable block (g_xcpCal.fsVdd/fsVddp3/fsVext); defaults were
 * derived empirically on 2026-07-02 with the rails at nominal. */

/* Fixed address so XCP clients can read without the map file (TASKING __at).
 * 0x70030000 is high in CPU0 DSPR0 (240 KB), clear of linker-placed data —
 * the linker errors out on any overlap. */
#define MEAS_RAD_TO_DEG   (57.29578f)

volatile Xcp_Data g_xcpData __at(XCP_DATA_ADDR);

/* The navigation state lives in its own block at the next free 256-byte slot,
 * because Xcp_Data has 8 bytes left before Xcp_Cal and this needs 176. Putting
 * it here rather than growing Xcp_Data means no existing address moves. */
volatile Xcp_Fusion g_xcpFusion __at(XCP_FUSION_ADDR);

void measurementsInit(void)
{
    IfxDts_Dts_Config dtsConfig;

    IfxDts_Dts_initModuleConfig(&dtsConfig);    /* defaults, no DTS interrupt */
    IfxDts_Dts_initModule(&dtsConfig);

    /* enable the core-domain die temperature sensor (safety-ENDINIT protected) */
    {
        uint16 pw = IfxScuWdt_getSafetyWatchdogPassword();
        IfxScuWdt_clearSafetyEndinit(pw);
        SCU_DTSCLIM.B.EN = 1u;
        IfxScuWdt_setSafetyEndinit(pw);
    }

    g_xcpData.magic     = XCP_DATA_MAGIC;
    g_xcpData.verMajor  = SW_VERSION_MAJOR;
    g_xcpData.verMinor  = SW_VERSION_MINOR;
    g_xcpData.verStep   = SW_VERSION_STEP;
    g_xcpData.reserved  = 0u;
    g_xcpData.tickMs    = 0u;
    g_xcpData.dieTempC  = 0.0f;
    g_xcpData.dtscTempC = 0.0f;
    g_xcpData.vddCore   = 0.0f;
    g_xcpData.vddp3     = 0.0f;
    g_xcpData.vext      = 0.0f;
    g_xcpData.rawVdd      = 0u;
    g_xcpData.rawVddp3    = 0u;
    g_xcpData.rawVext     = 0u;
    g_xcpData.baroPresent = 0u;
    g_xcpData.diagStatus  = 0u;
    g_xcpData.baroPressPa = 0.0f;
    g_xcpData.baroTempC   = 0.0f;
    g_xcpData.baroAltM    = 0.0f;
    g_xcpData.imuPresent  = 0u;
    g_xcpData.imuReserved[0] = 0u;
    g_xcpData.imuReserved[1] = 0u;
    g_xcpData.imuReserved[2] = 0u;
    g_xcpData.accelX      = 0.0f;
    g_xcpData.accelY      = 0.0f;
    g_xcpData.accelZ      = 0.0f;
    g_xcpData.gyroX       = 0.0f;
    g_xcpData.gyroY       = 0.0f;
    g_xcpData.gyroZ       = 0.0f;
    g_xcpData.imuTempC    = 0.0f;
    g_xcpData.gnssrxBytes   = 0u;
    g_xcpData.gnsssentences = 0u;
    g_xcpData.gnsserrors    = 0u;
    g_xcpData.gnssfixType   = 0u;
    g_xcpData.gnssnumSats   = 0u;

    {
        uint8 i;
        for (i = 0u; i < CORESTATS_NUM_CORES; i++)
        {
            g_xcpData.coreExecUs[i]   = 0u;
            g_xcpData.coreLoadPmil[i] = 0u;
            g_xcpData.coreAlive[i]    = 0u;
        }
        g_xcpData.ethBytesPerSec = 0u;
        g_xcpData.ethUtilPmil    = 0u;
        g_xcpData.ethLinkMbits   = 0u;
    }
}

void measurementsSetSystemLoad(void)
{
    uint8 i;

    for (i = 0u; i < CORESTATS_NUM_CORES; i++)
    {
        g_xcpData.coreExecUs[i]   = g_coreStats[i].execUs;
        g_xcpData.coreLoadPmil[i] = g_coreStats[i].loadPmil;
        g_xcpData.coreAlive[i]    = g_coreStats[i].aliveCounter;
    }

    EthStats_update();
    g_xcpData.ethBytesPerSec = EthStats_getBytesPerSec();
    g_xcpData.ethUtilPmil    = EthStats_getUtilPmil();
    g_xcpData.ethLinkMbits   = EthStats_getLinkMbits();
}

void measurementsSetBaro(boolean present, float32 pressurePa, float32 temperatureC, float32 altM)
{
    if (present != FALSE)
    {

        g_xcpData.baroPresent = 1u;
        g_xcpData.baroPressPa = pressurePa;
        g_xcpData.baroTempC   = temperatureC;
        g_xcpData.baroAltM    = altM;
    }
    else
    {
        g_xcpData.baroPresent = 0u;
        g_xcpData.baroPressPa = 0.0f;
        g_xcpData.baroTempC   = 0.0f;
        g_xcpData.baroAltM    = 0.0f;
    }
}

void measurementsSetMag(boolean present, const float32 mag[3], float32 headingDeg)
{
    if (present != FALSE)
    {
        g_xcpData.magPresent = 1u;
        g_xcpData.magX       = mag[0];
        g_xcpData.magY       = mag[1];
        g_xcpData.magZ       = mag[2];
        /* |B| is computed here rather than in the driver so that every
         * consumer — diagnostics, GUI, later the fusion — uses one definition.
         * It is the orientation-independent check that validates scaling. */
        g_xcpData.magFieldG  = sqrtf((mag[0] * mag[0]) + (mag[1] * mag[1])
                                     + (mag[2] * mag[2]));
        g_xcpData.magHeadingDeg = headingDeg;
    }
    else
    {
        g_xcpData.magPresent    = 0u;
        g_xcpData.magX          = 0.0f;
        g_xcpData.magY          = 0.0f;
        g_xcpData.magZ          = 0.0f;
        g_xcpData.magFieldG     = 0.0f;
        g_xcpData.magHeadingDeg = 0.0f;
    }
}

void measurementsSetImu(boolean present, const float32 acc[3], const float32 gyro[3],
                        float32 temperatureC)
{
    if (present != FALSE)
    {
        g_xcpData.imuPresent = 1u;
        g_xcpData.accelX     = acc[0];
        g_xcpData.accelY     = acc[1];
        g_xcpData.accelZ     = acc[2];
        g_xcpData.gyroX      = gyro[0];
        g_xcpData.gyroY      = gyro[1];
        g_xcpData.gyroZ      = gyro[2];
        g_xcpData.imuTempC   = temperatureC;
    }
    else
    {
        g_xcpData.imuPresent = 0u;
        g_xcpData.accelX     = 0.0f;
        g_xcpData.accelY     = 0.0f;
        g_xcpData.accelZ     = 0.0f;
        g_xcpData.gyroX      = 0.0f;
        g_xcpData.gyroY      = 0.0f;
        g_xcpData.gyroZ      = 0.0f;
        g_xcpData.imuTempC   = 0.0f;
    }
}

void measurementsSetGnss(boolean present, GnssM9N_Sample sample_info)
{
    if (present != FALSE)
    {
        g_xcpData.gnssPresent   = 1u;
        g_xcpData.gnssrxBytes   = sample_info.rxBytes;
        g_xcpData.gnsssentences = sample_info.sentences;
        g_xcpData.gnsserrors    = sample_info.errors;
        g_xcpData.gnssfixType   = sample_info.fixType;
        g_xcpData.gnssnumSats   = sample_info.numSats;
        g_xcpData.gnssfixOk      = sample_info.fixOk;
        g_xcpData.gnsstimeOk     = sample_info.timeOk;
        g_xcpData.gnssnavOk      = sample_info.navOk;
        g_xcpData.gnsslatDeg     = sample_info.latDeg;
        g_xcpData.gnsslonDeg     = sample_info.lonDeg;
        g_xcpData.gnssaltM       = sample_info.altM;
        g_xcpData.gnssspeedMps   = sample_info.speedMps;
        g_xcpData.gnssheadingDeg = sample_info.headingDeg;
        g_xcpData.gnsshAccM      = sample_info.hAccM;
        g_xcpData.gnssvAccM      = sample_info.vAccM;
        g_xcpData.gnssyear      = sample_info.year;
        g_xcpData.gnssmonth     = sample_info.month;
        g_xcpData.gnssday       = sample_info.day;
        g_xcpData.gnsshour      = sample_info.hour;
        g_xcpData.gnssmin       = sample_info.min;    
        g_xcpData.gnsssec       = sample_info.sec;
    }
    else
    {
        g_xcpData.gnssPresent   = 0u;
        g_xcpData.gnssrxBytes   = 0u;
        g_xcpData.gnsssentences = 0u;
        g_xcpData.gnsserrors    = 0u;
        g_xcpData.gnssfixType   = 0u;
        g_xcpData.gnssnumSats   = 0u;
        g_xcpData.gnssyear      = 0u;
        g_xcpData.gnssmonth     = 0u;
        g_xcpData.gnssday       = 0u;
        g_xcpData.gnsshour      = 0u;
        g_xcpData.gnssmin       = 0u;
        g_xcpData.gnsssec       = 0u;
        g_xcpData.gnssfixOk      = 0u;
        g_xcpData.gnsstimeOk     = 0u;
        g_xcpData.gnssnavOk      = 0u;
        g_xcpData.gnsslatDeg     = 0.0f;
        g_xcpData.gnsslonDeg     = 0.0f;
        g_xcpData.gnssaltM       = 0.0f;
        g_xcpData.gnssspeedMps   = 0.0f;
        g_xcpData.gnssheadingDeg = 0.0f;
        g_xcpData.gnsshAccM      = 0.0f;
        g_xcpData.gnssvAccM      = 0.0f;
    }
}

void measurementsSetFusion(const FusionValues *fusion, const Ahrs_Values *ahrs,
                           boolean present, float32 elapsed)
{
    uint8 i;

    /* --- legacy fields in Xcp_Data ------------------------------------
     * Kept and still written so the A2L entries, the GUI plots and every
     * hardcoded address in tools/ survive the move to the block below. */
    if (present != FALSE)
    {
        g_xcpData.fusionElapsed = elapsed;
        g_xcpData.a_D           = fusion->a_D;
        g_xcpData.a_v_d         = fusion->a_v_d;
        g_xcpData.a_d           = fusion->a_d;
        g_xcpData.fusionInnov   = fusion->innov;
        g_xcpData.fusionP00     = fusion->p00;
        g_xcpData.fusionRejects = fusion->rejects;
        g_xcpData.fusionResets  = fusion->resets;
    }
    else
    {
        g_xcpData.fusionElapsed = 0.0f;
        g_xcpData.a_D           = 0.0f;
    }

    /* --- the full state ------------------------------------------------ */
    g_xcpFusion.magic  = XCP_FUSION_MAGIC;
    g_xcpFusion.tickMs = g_TickCount_1ms;

    /* Angles are published in DEGREES. The filter works in radians and always
     * will, but every consumer of this block is a human or a plot. */
    g_xcpFusion.rollDeg  = ahrs->rollRad  * MEAS_RAD_TO_DEG;
    g_xcpFusion.pitchDeg = ahrs->pitchRad * MEAS_RAD_TO_DEG;
    g_xcpFusion.yawDeg   = ahrs->yawRad   * MEAS_RAD_TO_DEG;

    for (i = 0u; i < 4u; i++)
    {
        g_xcpFusion.q[i] = ahrs->q[i];
    }

    for (i = 0u; i < 3u; i++)
    {
        g_xcpFusion.rateDps[i]  = ahrs->rate[i] * MEAS_RAD_TO_DEG;
        g_xcpFusion.gyroBias[i] = ahrs->gyroBias[i];
        g_xcpFusion.accNed[i]   = ahrs->accNed[i];
    }

    g_xcpFusion.accMagG    = ahrs->accMagG;
    g_xcpFusion.magFieldG  = ahrs->magFieldG;
    g_xcpFusion.ahrsState  = ahrs->state;
    g_xcpFusion.accTrusted = ahrs->accTrusted;
    g_xcpFusion.magTrusted = ahrs->magTrusted;
    g_xcpFusion.reserved   = 0u;

    g_xcpFusion.posD     = fusion->a_d;
    g_xcpFusion.velD     = fusion->a_v_d;
    g_xcpFusion.accBiasD = fusion->accBiasD;
    g_xcpFusion.baroBias = fusion->baroBias;
    g_xcpFusion.innovD   = fusion->innov;
    g_xcpFusion.varD     = fusion->p00;

    g_xcpFusion.posN     = fusion->posN;
    g_xcpFusion.posE     = fusion->posE;
    g_xcpFusion.velN     = fusion->velN;
    g_xcpFusion.velE     = fusion->velE;
    g_xcpFusion.accBiasN = fusion->accBiasN;
    g_xcpFusion.accBiasE = fusion->accBiasE;
    g_xcpFusion.innovN   = fusion->innovN;
    g_xcpFusion.innovE   = fusion->innovE;
    g_xcpFusion.varN     = fusion->pNN;

    g_xcpFusion.originLatDeg = fusion->originLatDeg;
    g_xcpFusion.originLonDeg = fusion->originLonDeg;
    g_xcpFusion.originAltM   = fusion->originAltM;

    g_xcpFusion.baroRejects = fusion->rejects;
    g_xcpFusion.baroResets  = fusion->resets;
    g_xcpFusion.gnssRejects = fusion->gnssRejects;
    g_xcpFusion.gnssUpdates = fusion->gnssUpdates;

    g_xcpFusion.verticalOk   = fusion->verticalOk;
    g_xcpFusion.horizontalOk = fusion->horizontalOk;
    g_xcpFusion.originSet    = fusion->originSet;
    g_xcpFusion.reserved2    = 0u;
    g_xcpFusion.covResets    = fusion->covResets;
}

void measurementsUpdate(void)
{
    uint8 rawVdd   = IfxPmsEvr_getSecondaryAdcResult(&MODULE_PMS, IfxPmsEvr_SupplyMode_evrc);
    uint8 rawVddp3 = IfxPmsEvr_getSecondaryAdcResult(&MODULE_PMS, IfxPmsEvr_SupplyMode_evr33);
    uint8 rawVext  = IfxPmsEvr_getSecondaryAdcResult(&MODULE_PMS, IfxPmsEvr_SupplyMode_swd);

    g_xcpData.tickMs    = g_TickCount_1ms;
    g_xcpData.dieTempC  = IfxDts_Dts_getTemperatureCelsius();
    g_xcpData.dtscTempC = IfxDts_Dts_convertToCelsius((uint16)SCU_DTSCSTAT.B.RESULT);
    g_xcpData.rawVdd    = rawVdd;
    g_xcpData.rawVddp3  = rawVddp3;
    g_xcpData.rawVext   = rawVext;
    g_xcpData.vddCore   = ((float32)rawVdd   * g_xcpCal.fsVdd)   / 255.0f;
    g_xcpData.vddp3     = ((float32)rawVddp3 * g_xcpCal.fsVddp3) / 255.0f;
    g_xcpData.vext      = ((float32)rawVext  * g_xcpCal.fsVext)  / 255.0f;
}
