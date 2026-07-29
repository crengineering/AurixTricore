#include "Measurements.h"
#include "Diagnostics.h"
#include "Nvm.h"
#include "Version.h"
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
