#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "Ifx_Cfg_Ssw.h"
#include "IfxPort.h"
#include "IfxStm.h"
#include "Uart.h"

IFX_ALIGN(4) IfxCpu_syncEvent cpuSyncEvent = 0;

/* LED time-division phase flag (shared with all cores via extern).
 * 0 = CPU0–3 have the LEDs   (500 ms ON + 500 ms OFF per cycle)
 * 1 = CPU4–5 have the LEDs   (500 ms ON window; borrow D306/D307) */
volatile uint32 g_ledPhase = 0u;

/* LED for CPU0 — D306, Pin P20.11, active-low */
#define BLINKY_LED_PORT     &MODULE_P20
#define BLINKY_LED_PIN      11u
#define BLINKY_LED_ON       IfxPort_State_low
#define BLINKY_LED_OFF      IfxPort_State_high

int core0_main(void)
{
    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    Uart_init();
    Uart_println("CPU0 started");

    IfxPort_setPinMode(BLINKY_LED_PORT, BLINKY_LED_PIN, IfxPort_Mode_outputPushPullGeneral);

    while (TRUE)
    {
        /* --- Phase 0: CPU0–3 get one 500 ms ON / 500 ms OFF blink --- */
        g_ledPhase = 0u;
        /* 300 ms grace: visible dark gap between phase 1 (CPU4/5 window) and
         * phase 0 ON half.  Also ensures CPU4/5 finish their LED_OFF handover
         * write (takes at most 10 ms) well before we assert ON. */
        IfxStm_waitTicks(&MODULE_STM0, 30000000u);
        IfxPort_setPinState(BLINKY_LED_PORT, BLINKY_LED_PIN, BLINKY_LED_ON);
        IfxStm_waitTicks(&MODULE_STM0, 50000000u);          /* 500 ms */
        IfxPort_setPinState(BLINKY_LED_PORT, BLINKY_LED_PIN, BLINKY_LED_OFF);
        IfxStm_waitTicks(&MODULE_STM0, 50000000u);          /* 500 ms */

        /* --- Phase 1: CPU4–5 get a 500 ms window; CPU0 LED stays off --- */
        g_ledPhase = 1u;
        IfxStm_waitTicks(&MODULE_STM0, 50000000u);          /* 500 ms */
    }

    return 0;
}
