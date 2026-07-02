#include "Measurements.h"
#include "Version.h"
#include "Dts/Dts/IfxDts_Dts.h"
#include "Pms/Std/IfxPmsEvr.h"
#include "IfxScuWdt.h"
#include "Ifx_Lwip.h"

/* Full-scale voltages of the 8-bit PMS monitor ADCs (code 255).
 * Derived empirically on 2026-07-02 from the raw codes with the rails at
 * nominal (VDD 219@1.25V, VDDP3 220@3.30V, VEXT 216@5.00V — the SWD channel
 * must cover >5.5V, so raw 216 implies a 5V VEXT on the TriBoard). These
 * become XCP-calibratable values in the diagnostics module. */
#define MEAS_VDD_FULLSCALE      1.455f
#define MEAS_VDDP3_FULLSCALE    3.825f
#define MEAS_VEXT_FULLSCALE     5.903f

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
    g_xcpData.rawVdd    = 0u;
    g_xcpData.rawVddp3  = 0u;
    g_xcpData.rawVext   = 0u;
    g_xcpData.reserved2 = 0u;
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
    g_xcpData.vddCore   = ((float32)rawVdd   * MEAS_VDD_FULLSCALE)   / 255.0f;
    g_xcpData.vddp3     = ((float32)rawVddp3 * MEAS_VDDP3_FULLSCALE) / 255.0f;
    g_xcpData.vext      = ((float32)rawVext  * MEAS_VEXT_FULLSCALE)  / 255.0f;
}
