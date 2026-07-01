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

    /* Ethernet & Timer Initialisierung */
    IfxGeth_enableModule(&MODULE_GETH);

    IfxStm_CompareConfig stmCompareConfig;
    IfxStm_initCompareConfig(&stmCompareConfig);
    stmCompareConfig.triggerPriority    = ISR_PRIORITY_OS_TICK;
    stmCompareConfig.comparatorInterrupt = IfxStm_ComparatorInterrupt_ir0;
    stmCompareConfig.ticks              = IFX_CFG_STM_TICKS_PER_MS * 10;
    stmCompareConfig.typeOfService      = IfxSrc_Tos_cpu0;
    IfxStm_initCompare(&MODULE_STM0, &stmCompareConfig);

    eth_addr_t myMacAddr = {{0x00, 0x03, 0x19, 0x12, 0x34, 0x56}};
    Ifx_Lwip_init(myMacAddr);
    echoInit();
    Uart_println("Ethernet started");
    while (TRUE)
    {
        Scheduler_run(&g_sched);

        /* Ethernet polling - ohne störende UART prints */
        Ifx_Lwip_pollTimerFlags();
        Ifx_Lwip_pollReceiveFlags();
    }

    return 0;
}

/* 4. Timer Interrupt für den Stack-Takt */
IFX_INTERRUPT(updateLwIPStackISR, 0, ISR_PRIORITY_OS_TICK);

void updateLwIPStackISR(void)
{
    IfxStm_increaseCompare(&MODULE_STM0, IfxStm_Comparator_0, IFX_CFG_STM_TICKS_PER_MS);
    g_TickCount_1ms++;
    Ifx_Lwip_onTimerTick();
}
