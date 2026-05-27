#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "Ifx_Cfg_Ssw.h"
#include "IfxPort.h"
#include "IfxStm.h"
#include "Uart.h"

IFX_ALIGN(4) IfxCpu_syncEvent cpuSyncEvent = 0;

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
        IfxPort_setPinState(BLINKY_LED_PORT, BLINKY_LED_PIN, BLINKY_LED_ON);
        IfxStm_waitTicks(&MODULE_STM0, 50000000u);
        IfxPort_setPinState(BLINKY_LED_PORT, BLINKY_LED_PIN, BLINKY_LED_OFF);
        IfxStm_waitTicks(&MODULE_STM0, 50000000u);
    }

    return 0;
}
