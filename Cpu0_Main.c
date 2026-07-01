#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxStm.h"
#include "Uart.h"
#include "scheduler.h"
#include "led.h"
#include "IfxGeth_Eth.h"
#include "Ifx_Console.h"
#include "Configuration.h"
#include "ConfigurationIsr.h"
#include "Ifx_Lwip.h"
#include "Echo.h"

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

    // Ethernet init
    /* 2. Lokale Variablen statt globaler */
    Uart_println("Ethernet starting");
    eth_addr_t myMacAddr = {{0x00, 0x03, 0x19, 0x12, 0x34, 0x56}};
    IfxStm_CompareConfig stmCompareConfig;

    /* Hardware Init */
    IfxStm_initCompareConfig(&stmCompareConfig);
    IfxStm_initCompare(&MODULE_STM0, &stmCompareConfig);
    IfxGeth_enableModule(&MODULE_GETH);

    /* LwIP und Echo Initialisierung mit lokaler MAC */
    Ifx_Lwip_init(myMacAddr);
    echoInit();
    Uart_println("Ethernet started");

    while (TRUE)
    {
        Scheduler_run(&g_sched);

        // Ethernet loop
        Ifx_Lwip_pollTimerFlags();
        Ifx_Lwip_pollReceiveFlags();
    }

    return 0;
}
