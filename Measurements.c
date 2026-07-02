#include "Measurements.h"
#include "Version.h"
#include "Dts/Dts/IfxDts_Dts.h"
#include "Ifx_Lwip.h"

/* Fixed address so XCP clients can read without the map file (TASKING __at).
 * 0x70030000 is high in CPU0 DSPR0 (240 KB), clear of linker-placed data —
 * the linker errors out on any overlap. */
volatile Xcp_Data g_xcpData __at(XCP_DATA_ADDR);

void measurementsInit(void)
{
    IfxDts_Dts_Config dtsConfig;

    IfxDts_Dts_initModuleConfig(&dtsConfig);    /* defaults, no DTS interrupt */
    IfxDts_Dts_initModule(&dtsConfig);

    g_xcpData.magic    = XCP_DATA_MAGIC;
    g_xcpData.verMajor = SW_VERSION_MAJOR;
    g_xcpData.verMinor = SW_VERSION_MINOR;
    g_xcpData.verStep  = SW_VERSION_STEP;
    g_xcpData.reserved = 0u;
    g_xcpData.tickMs   = 0u;
    g_xcpData.dieTempC = 0.0f;
}

void measurementsUpdate(void)
{
    g_xcpData.tickMs   = g_TickCount_1ms;
    g_xcpData.dieTempC = IfxDts_Dts_getTemperatureCelsius();
}
