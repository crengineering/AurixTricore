#include "Diagnostics.h"
#include "Measurements.h"
#include "Nvm.h"
#include "PeriphDiag.h"
#include "Uart.h"

/* Calibration block at a fixed address so XCP masters can read/write it
 * without the map file. The XCP slave only permits writes inside this block. */
volatile Xcp_Cal g_xcpCal __at(XCP_CAL_ADDR);

extern volatile Xcp_Data g_xcpData;

#define DIAG_NUM_CHECKS     12u
#define DIAG_TICK_MS        100u    /* diagnosticsUpdate() call period       */
#define DIAG_MAX_DEBOUNCE_S 60.0f

/* The PC GUI sends a heartbeat byte every 500 ms over UART. No byte for
 * this many ticks means the UART link (USB cable / GUI) is gone. */
#define DIAG_UART_TIMEOUT_TICKS 20u     /* 2 s in 100 ms ticks */

/* one debounce counter per status bit (in 100 ms ticks) */
static uint16 s_violationTicks[DIAG_NUM_CHECKS];

/* 100 ms ticks since the last UART RX byte (saturating) */
static uint16 s_uartSilenceTicks;

static void diag_loadDefaults(void)
{
    g_xcpCal.dtsMin       = -40.0f;     /* TC39x junction temperature range  */
    g_xcpCal.dtsMax       = 105.0f;     /* warn well below Tj,max = 150 degC */
    g_xcpCal.dtscMin      = -40.0f;
    g_xcpCal.dtscMax      = 105.0f;
    g_xcpCal.vddMin       = 1.17f;      /* datasheet operating ranges        */
    g_xcpCal.vddMax       = 1.32f;
    g_xcpCal.vddp3Min     = 3.13f;
    g_xcpCal.vddp3Max     = 3.47f;
    g_xcpCal.vextMin      = 4.50f;      /* 5 V system                        */
    g_xcpCal.vextMax      = 5.50f;
    g_xcpCal.tempDeltaMax = 10.0f;      /* DTS vs DTSC plausibility          */
    g_xcpCal.debounceSec  = 1.0f;
    g_xcpCal.fsVdd        = 1.455f;     /* monitor-ADC full scales, derived  */
    g_xcpCal.fsVddp3      = 3.825f;     /* empirically 2026-07-02            */
    g_xcpCal.fsVext       = 5.903f;
    g_xcpCal.magic        = XCP_CAL_MAGIC;
}

void diagnosticsInit(void)
{
    uint32 i;

    diag_loadDefaults();
    PeriphDiag_init();

    for (i = 0u; i < DIAG_NUM_CHECKS; i++)
    {
        s_violationTicks[i] = 0u;
    }

    /* start disconnected: the bit appears unless a heartbeat arrives */
    s_uartSilenceTicks = DIAG_UART_TIMEOUT_TICKS;

    g_xcpData.diagStatus = 0u;
}

/* Debounce one check: the violation must persist debounceSec (0.1 s
 * resolution) before its bit is set; it clears immediately when back in
 * range. */
static uint32 diag_debounce(uint32 checkIndex, boolean violated, uint32 bitMask)
{
    float32 debounce = g_xcpCal.debounceSec;
    uint16  limitTicks;

    if (debounce < 0.0f)
    {
        debounce = 0.0f;
    }
    else if (debounce > DIAG_MAX_DEBOUNCE_S)
    {
        debounce = DIAG_MAX_DEBOUNCE_S;
    }

    limitTicks = (uint16)((debounce * 1000.0f / (float32)DIAG_TICK_MS) + 0.5f);

    if (violated)
    {
        if (s_violationTicks[checkIndex] < 0xFFFFu)
        {
            s_violationTicks[checkIndex]++;
        }

        if (s_violationTicks[checkIndex] > limitTicks)
        {
            return bitMask;
        }
    }
    else
    {
        s_violationTicks[checkIndex] = 0u;
    }

    return 0u;
}

boolean diagnosticsUpdate(void)
{
    uint32  status = 0u;
    float32 tempDelta;
    boolean uartLost;
    boolean anyFault;

    /* UART link heartbeat: a received 'H' resets the silence counter */
    if (Uart_heartbeatReceived() != FALSE)
    {
        s_uartSilenceTicks = 0u;
    }
    else if (s_uartSilenceTicks < 0xFFFFu)
    {
        s_uartSilenceTicks++;
    }
    else
    {
        /* counter saturated */
    }

    /* self-check: a garbled calibration block falls back to defaults */
    if (g_xcpCal.magic != XCP_CAL_MAGIC)
    {
        diag_loadDefaults();
        status |= DIAG_CAL_INVALID;
    }

    /* persistent-block health, maintained by the Nvm module */
    if (Nvm_hasFault() != FALSE)
    {
        status |= DIAG_NVM_FAULT;
    }

    /* peripheral wiring / communication / liveness faults (PeriphDiag.c).
     * Already debounced there, so no diag_debounce() around it. */
    status |= PeriphDiag_update();

    tempDelta = g_xcpData.dieTempC - g_xcpData.dtscTempC;
    if (tempDelta < 0.0f)
    {
        tempDelta = -tempDelta;
    }

    status |= diag_debounce(0u,  (boolean)(g_xcpData.dieTempC  < g_xcpCal.dtsMin),   DIAG_DTS_UNDERTEMP);
    status |= diag_debounce(1u,  (boolean)(g_xcpData.dieTempC  > g_xcpCal.dtsMax),   DIAG_DTS_OVERTEMP);
    status |= diag_debounce(2u,  (boolean)(g_xcpData.dtscTempC < g_xcpCal.dtscMin),  DIAG_DTSC_UNDERTEMP);
    status |= diag_debounce(3u,  (boolean)(g_xcpData.dtscTempC > g_xcpCal.dtscMax),  DIAG_DTSC_OVERTEMP);
    status |= diag_debounce(4u,  (boolean)(g_xcpData.vddCore   < g_xcpCal.vddMin),   DIAG_VDD_UNDERVOLT);
    status |= diag_debounce(5u,  (boolean)(g_xcpData.vddCore   > g_xcpCal.vddMax),   DIAG_VDD_OVERVOLT);
    status |= diag_debounce(6u,  (boolean)(g_xcpData.vddp3     < g_xcpCal.vddp3Min), DIAG_VDDP3_UNDERVOLT);
    status |= diag_debounce(7u,  (boolean)(g_xcpData.vddp3     > g_xcpCal.vddp3Max), DIAG_VDDP3_OVERVOLT);
    status |= diag_debounce(8u,  (boolean)(g_xcpData.vext      < g_xcpCal.vextMin),  DIAG_VEXT_UNDERVOLT);
    status |= diag_debounce(9u,  (boolean)(g_xcpData.vext      > g_xcpCal.vextMax),  DIAG_VEXT_OVERVOLT);
    status |= diag_debounce(10u, (boolean)(tempDelta           > g_xcpCal.tempDeltaMax), DIAG_TEMP_IMPLAUSIBLE);

    if (s_uartSilenceTicks >= DIAG_UART_TIMEOUT_TICKS)
    {
        uartLost = TRUE;
    }
    else
    {
        uartLost = FALSE;
    }
    status |= diag_debounce(11u, uartLost, DIAG_UART_DISCONNECTED);

    g_xcpData.diagStatus = status;

    if (status != 0u) {
        anyFault = TRUE;
    } else {
        anyFault = FALSE;
    }
    return anyFault;
}
