#include "Measurements.h"
#include "Diagnostics.h"
#include "Nvm.h"
#include "Version.h"
#include "Ahrs.h"
#include "CoreStats.h"
#include "EthStats.h"
#include "Dts/Dts/IfxDts_Dts.h"
#include "Pms/Std/IfxPmsEvr.h"
#include "IfxScuWdt.h"
#include "Ifx_Lwip.h"
#include <math.h>

/* Pressure-altitude reference: the sea-level reference pressure (QNH) is a
 * persistent NVM parameter (g_xcpNvm.seaLevelPa, DFLASH). Set it to local QNH
 * for true elevation; the default is the ISA standard atmosphere. Values
 * outside a sane band fall back to the standard so a bad write can't blow up
 * the altitude. */
#define MEAS_SEA_LEVEL_PA   (101325.0f)
#define MEAS_SEA_LEVEL_MIN  (80000.0f)       /* ~2 km above sea level QNH floor  */
#define MEAS_SEA_LEVEL_MAX  (120000.0f)      /* well above any real QNH          */
#define MEAS_ALT_EXPONENT   (0.190294957f)   /* 1 / 5.25588 (barometric formula) */
#define MEAS_ALT_SCALE      (44330.0f)       /* [m] */

/* The full-scale voltages of the 8-bit PMS monitor ADCs live in the
 * XCP-calibratable block (g_xcpCal.fsVdd/fsVddp3/fsVext); defaults were
 * derived empirically on 2026-07-02 with the rails at nominal. */

/* Fixed address so XCP clients can read without the map file (TASKING __at).
 * 0x70030000 is high in CPU0 DSPR0 (240 KB), clear of linker-placed data —
 * the linker errors out on any overlap. */
volatile Xcp_Data g_xcpData __at(XCP_DATA_ADDR);

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
        g_xcpData.ahrsState      = 0u;
        g_xcpData.ahrsAccOk      = 0u;
        g_xcpData.ahrsReserved[0] = 0u;
        g_xcpData.ahrsReserved[1] = 0u;
        g_xcpData.roll  = 0.0f;
        g_xcpData.pitch = 0.0f;
        g_xcpData.yaw   = 0.0f;
        g_xcpData.rateP = 0.0f;
        g_xcpData.rateQ = 0.0f;
        g_xcpData.rateR = 0.0f;
        g_xcpData.biasX = 0.0f;
        g_xcpData.biasY = 0.0f;
        g_xcpData.biasZ = 0.0f;

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

void measurementsSetAttitude(void)
{
    float32 phi[3];
    float32 om[3];
    float32 bias[3];

    Ahrs_getAttitude(phi);
    Ahrs_getRates(om);
    Ahrs_getGyroBias(bias);

    g_xcpData.ahrsState = (uint8)Ahrs_getState();
    g_xcpData.ahrsAccOk = (Ahrs_isAccelTrusted() != FALSE) ? 1u : 0u;
    g_xcpData.roll      = phi[0];
    g_xcpData.pitch     = phi[1];
    g_xcpData.yaw       = phi[2];
    g_xcpData.rateP     = om[0];
    g_xcpData.rateQ     = om[1];
    g_xcpData.rateR     = om[2];
    g_xcpData.biasX     = bias[0];
    g_xcpData.biasY     = bias[1];
    g_xcpData.biasZ     = bias[2];
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

void measurementsSetBaro(boolean present, float32 pressurePa, float32 temperatureC)
{
    if ((present != FALSE) && (pressurePa > 0.0f))
    {
        float32 p0 = (float32)g_xcpNvm.seaLevelPa;

        if ((p0 < MEAS_SEA_LEVEL_MIN) || (p0 > MEAS_SEA_LEVEL_MAX))
        {
            p0 = MEAS_SEA_LEVEL_PA;         /* guard against a bad/zero NVM value */
        }

        g_xcpData.baroPresent = 1u;
        g_xcpData.baroPressPa = pressurePa;
        g_xcpData.baroTempC   = temperatureC;
        /* International barometric formula: h = 44330 * (1 - (P/P0)^0.190295). */
        g_xcpData.baroAltM    = MEAS_ALT_SCALE *
            (1.0f - powf(pressurePa / p0, MEAS_ALT_EXPONENT));
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
         * consumer — diagnostics, GUI, later the AHRS — uses one definition.
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
