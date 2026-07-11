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
#include "UdpEcho.h"
#include "Xcp.h"
#include "Measurements.h"
#include "Diagnostics.h"
#include "Nvm.h"
#include "Version.h"
#include "gpio.h"

IFX_ALIGN(4) IfxCpu_syncEvent cpuSyncEvent = 0;

static Scheduler_t g_sched;
static Led_t       g_led;
static boolean error_active = FALSE;

static void Task_LedToggle(void)
{
    Led_toggle(&g_led);
}

static void Task_App10ms(void)
{
    /* TODO: add CPU0 application logic here */
}

static void Task_Measure100ms(void)
{
    Nvm_task100ms();        /* before diagnostics: fresh NVM fault state */
    measurementsUpdate();
    if (diagnosticsUpdate()) {
        gpio_write(GPIO_P_00_0, GPIO_STATE_ON);
    } else {
        gpio_write(GPIO_P_00_0, GPIO_STATE_OFF);
    }
    xcpDaqCycle();
}

int core0_main(void)
{
    IfxCpu_enableInterrupts();

    /*disable Watchdogs*/
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    /* init UART communication*/
    Uart_init();
    Uart_println("CPU0 started, SW v" SW_VERSION_STRING);

    /* init LED toggle for task*/
    Led_init(&g_led, &MODULE_P20, 11u);

    /* init GPIO */
    init_gpio_pins();

    /* init scheduler */
    Scheduler_init(&g_sched, &MODULE_STM0);
    Scheduler_addTask(&g_sched, Task_LedToggle, SCHED_MS(500u));
    Scheduler_addTask(&g_sched, Task_App10ms,   SCHED_MS(10u));
    Scheduler_addTask(&g_sched, Task_Measure100ms, SCHED_MS(100u));

    /* init persistent memory*/
    Nvm_bootInit();         /* load persistent parameters from DFLASH       */
    diagnosticsInit();      /* before measurementsInit: provides ADC scales */
    measurementsInit();

    /* STM0 Comparator 0 als 1-ms-Tick für den lwIP-Stack scharf schalten
     * (ohne initCompare feuert updateLwIPStackISR nie -> keine TCP/ARP-Timer) */
    IfxStm_CompareConfig stmCompareConfig;
    IfxStm_initCompareConfig(&stmCompareConfig);
    stmCompareConfig.triggerPriority     = ISR_PRIORITY_OS_TICK;
    stmCompareConfig.comparatorInterrupt = IfxStm_ComparatorInterrupt_ir0;
    stmCompareConfig.ticks               = IFX_CFG_STM_TICKS_PER_MS * 10;   /* erster Interrupt nach 10 ms */
    stmCompareConfig.typeOfService       = IfxSrc_Tos_cpu0;
    IfxStm_initCompare(&MODULE_STM0, &stmCompareConfig);

    /* Ethernet & Timer Initialisierung */
    /* LwIP Stack initialisieren */
    eth_addr_t myMacAddr = {{0x00, 0x03, 0x19, 0x12, 0x34, 0x56}};
    Ifx_Lwip_init(myMacAddr);

    /* Statische IP anstatt DHCP */
    struct netif *netif = Ifx_Lwip_getNetIf();
    ip_addr_t ip, netmask, gw;

    IP4_ADDR(&ip,      192, 168, 0, 10);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw,      0,   0,   0,   0);

    netif_set_addr(netif, &ip, &netmask, &gw);
    netif_set_up(netif);

    echoInit();
    udpEchoInit();
    xcpInit();
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
