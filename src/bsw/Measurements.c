#include "Measurements.h"
#include "Diagnostics.h"
#include "Version.h"
#include "Dts/Dts/IfxDts_Dts.h"
#include "Pms/Std/IfxPmsEvr.h"
#include "IfxScuWdt.h"
#include "Ifx_Lwip.h"

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
    g_xcpData.rawVdd     = 0u;
    g_xcpData.rawVddp3   = 0u;
    g_xcpData.rawVext    = 0u;
    g_xcpData.reserved2  = 0u;
    g_xcpData.diagStatus = 0u;
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
