#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "Ifx_Cfg_Ssw.h"
#include "Uart.h"
#include "scheduler.h"
#include "led.h"

IFX_ALIGN(4) IfxCpu_syncEvent cpuSyncEvent = 0;

static Scheduler_t g_sched;
static Led_t       g_led;

static void Task_LedToggle(void)
{
    Led_toggle(&g_led);
}

static void Task_App10ms(void)
{
    /* TODO: add CPU0 application logic here */
}

int core0_main(void)
{
    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    Uart_init();
    Uart_println("CPU0 started");

    Led_init(&g_led, &MODULE_P20, 11u);

    Scheduler_init(&g_sched, &MODULE_STM0);
    Scheduler_addTask(&g_sched, Task_LedToggle, SCHED_MS(500u));
    Scheduler_addTask(&g_sched, Task_App10ms,   SCHED_MS(10u));

    while (TRUE)
    {
        Scheduler_run(&g_sched);
    }

    return 0;
}
