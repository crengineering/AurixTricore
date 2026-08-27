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
#include "I2c.h"
#include "Bmp581.h"
#include "Mmc5983.h"
#include "Spi.h"
#include "Icm42688.h"
#include "GnssM9N.h"
#include "SharedRam.h"
#include "CoreStats.h"
#include "BringUp.h"
#include "SensorTask.h"
#include "NavTask.h"
#include "Housekeeping.h"
#include "PeriphDiag.h"
#include "EthStats.h"
#include "CtrlReplay.h"     /* ASW replay harness; single BSW->ASW init
                             * callout — documented deviation, see
                             * docs/CTRL_REPLAY.md                      */

IFX_ALIGN(4) IfxCpu_syncEvent cpuSyncEvent = 0;

static Scheduler_t g_sched;
static Led_t       g_led;

static void Task_LedToggle(void)
{
    Led_toggle(&g_led);

}

/* Was a free-running poll in the while(TRUE) body; scheduled here (T8,
 * docs/REFACTORING_PLAN.md) so its cost is accounted like every other task
 * instead of being invisible in the load figures. lwIP's own tick is already
 * 1 ms (see updateLwIPStackISR), so this matches it. */
static void Task_Lwip(void)
{
    Ifx_Lwip_pollTimerFlags();
    Ifx_Lwip_pollReceiveFlags();
}

/* Split into its own task (T7, docs/REFACTORING_PLAN.md) so a DFLASH save
 * (Nvm_task100ms(), tens of ms, see Nvm.c) can no longer delay DAQ,
 * diagnostics or the GNSS feed by sharing a dispatch with them. */
static void Task_Nvm(void)
{
    Nvm_task100ms();
}

static void Task_XcpDaq(void)
{
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

    /* T9 boot self-check (docs/REFACTORING_PLAN.md §2.4, Risk 2): confirm the
     * LMU shared block really landed in a non-cacheable segment. A cachable
     * address here means the __at(SHARED_LMU_ADDR) alias is wrong and every
     * cross-core read of this block -- g_coreStats today, NavState from T10
     * on -- could see stale data. g_coreStats is the first object in the
     * block and its own address is what we are actually placing. */
    /* cppcheck-suppress misra-c2012-11.8 ; deviation: IfxCpu_isAddressCachable
     * takes a plain void* (vendor header, not editable); g_coreStats is
     * volatile only because it is cross-core shared state, not because this
     * read-only address check could ever see it change mid-call. */
    if (IfxCpu_isAddressCachable((void *)&g_coreStats[0]) == FALSE)
    {
        Uart_println("SharedRam: LMU block @0xB00F0000 is non-cacheable (OK)");
    }
    else
    {
        Uart_println("SharedRam: LMU block @0xB00F0000 IS CACHEABLE -- alias WRONG");
    }

    /* Prove Ifx__dsync() itself compiles to a real DSYNC instruction on
     * TASKING before NavState (T10) makes its publish ordering depend on it
     * -- CoreStats_t's own fields tolerate a torn read and do not need this
     * barrier (D6). A missing/no-op barrier is silent -- it would not fail
     * this build, only a later cross-core snapshot under load -- so it is
     * exercised here and confirmed in Cpu0_Main.src rather than trusted
     * because the build succeeded. See docs/REFACTORING_PLAN.md §2.4/Risk 2. */
    Ifx__dsync();

    /* P10.7 (ICM-42688-P INT1 candidate, docs/IMU_INTERRUPT.md): permanent
     * input-only configuration, run before anything else on this core
     * touches a pin. The pad self-test that proved this pin free of the
     * board (g_padProbeP10 == 0x7) drove it as an output and has been
     * removed now that the INT1 wire is the intended use -- see BringUp.h. */
    BringUp_initImuIntPinSafe();

    /* init LED toggle for task*/
    Led_init(&g_led, &MODULE_P20, 11u);

    /* init GPIO */
    init_gpio_pins();
    gpio_calInit();         /* XCP GPIO control block: all pins firmware-owned */

    /* init scheduler */
    Scheduler_init(&g_sched, &MODULE_STM0, 0u);
    Scheduler_addTask(&g_sched, Task_LedToggle, SCHED_MS(500u));
    (void)Scheduler_addTask(&g_sched, Task_Lwip,       SCHED_MS(1u));    /* 1 kHz lwIP poll */
    (void)Scheduler_addTask(&g_sched, SensorTask_baro, SCHED_MS(20u));   /* 50 Hz barometer */
    (void)Scheduler_addTask(&g_sched, SensorTask_mag,  SCHED_MS(20u));   /* 50 Hz magnetometer */
    (void)Scheduler_addTask(&g_sched, NavTask_step,    SCHED_MS(20u));  /* 50 Hz IMU (QSPI0) */
    /* Registration order preserves the original dispatch order within a
     * shared 100 ms dispatch (Scheduler_run walks tasks array-order): NVM
     * before GNSS before housekeeping before DAQ, exactly as the single
     * Task_Measure100ms body ran them. Nvm_task100ms() in particular must
     * still run before Housekeeping_100ms's diagnosticsUpdate() -- fresh NVM
     * fault state for that cycle's diagnostics word. */
    (void)Scheduler_addTask(&g_sched, Task_Nvm,            SCHED_MS(100u));
    (void)Scheduler_addTask(&g_sched, SensorTask_gnss,     SCHED_MS(100u));  /* 10 Hz GNSS */
    (void)Scheduler_addTask(&g_sched, Housekeeping_100ms,  SCHED_MS(100u));
    (void)Scheduler_addTask(&g_sched, Task_XcpDaq,         SCHED_MS(100u));

    /* init persistent memory*/
    Nvm_bootInit();         /* load persistent parameters from DFLASH       */
    diagnosticsInit();      /* before measurementsInit: provides ADC scales */
    measurementsInit();
    Housekeeping_init();

    /* init shared I2C0 sensor bus + the barometer (Adafruit BMP581 STEMMA QT).
     * Since the BMP388 and MPU-6050 came off on 2026-07-31 this breakout is the
     * only device on the bus, so its own 10 kOhm pull-ups are the only ones
     * holding the lines up — the idle check below is what proves that. */
    I2c_init();
    if (I2c_busIsIdle() != FALSE)
    {
        Uart_println("I2C0 bus idle (SCL+SDA released)");
    }
    else
    {
        Uart_println("I2C0 bus HELD - a slave is pulling SCL/SDA low");
    }
    if (Bmp581_init() != FALSE)
    {
        Uart_println("BMP581 detected (CHIP_ID 0x50/0x51)");
    }
    else
    {
        Uart_println("BMP581 not found - check wiring/address (0x47 default, 0x46 via addr jumper)");
    }

    /* Second device on the shared I2C0 bus: MMC5983MA magnetometer at 0x30.
     * Brought up after the barometer so the UART order matches the debug order
     * in docs/MMC5983MA.md 8.2 — if adding this device broke the BMP581, that
     * is visible above before the magnetometer is even mentioned. */
    if (Mmc5983_init() != FALSE)
    {
        Uart_println("MMC5983 detected (PRODUCT_ID 0x30)");
    }
    else
    {
        Uart_println("MMC5983 not found - check wiring/CS strap (CS must be tied to +3V3)");
    }

    /* Tell the peripheral diagnostics what this build actually expects to find.
     * The IMU slot must be declared unfitted or it would assert NO_RESPONSE
     * forever and leave the diagnostics permanently red over hardware that is
     * deliberately absent. Baro/mag/GNSS are SensorTask's to declare — see
     * SensorTask_init() below, called once GnssM9N_init() has run. */
    PeriphDiag_setFitted(PERIPH_DIAG_IMU, TRUE);

    /* Flight IMU on its own bus: QSPI0, nothing shared with the I2C sensors.
     * Brought up after them so a failure here cannot be confused with a
     * problem on the shared bus. */
    Spi_init();
    if (Icm42688_init() != FALSE)
    {
        Uart_print("ICM-42688-P detected (WHO_AM_I 0x47) in SPI mode ");
        Uart_printHexByte(Spi_getMode());
        Uart_println("");
    }
    else
    {
        uint8 who = 0u;

        Uart_println("ICM-42688-P not found");
        Spi_setMode(SPI_MODE_0);
        (void)Icm42688_readWhoAmI(&who);
        Uart_print("  WHO_AM_I mode0=");
        Uart_printHexByte(who);
        who = 0u;
        Spi_setMode(SPI_MODE_3);
        (void)Icm42688_readWhoAmI(&who);
        Uart_print("  mode3=");
        Uart_printHexByte(who);
        Uart_println("");
    }
    BringUp_dumpSensors();  /* one-time register dump, all three sensors, for bring-up diagnosis */
    NavTask_init();         /* NavState_init, FusionCal_init, Ahrs_init, Fusion_init (T11) */

    /* init of the GNSS*/
    if (GnssM9N_init() != FALSE)
    {
        Uart_println("Gnss-NEO-M9N init successfully");
    }
    else
    {
        Uart_println("Gnss-NEO-M9N init NOT successfully");
    }
    SensorTask_init();      /* declare baro/mag/GNSS fitted, now that all three have been brought up */

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
    CtrlReplay_init();
    EthStats_init();        /* MMC octet counters; after the MAC is up */
    Uart_println("Ethernet started");

    while (TRUE)
    {
        Scheduler_run(&g_sched);   /* lwIP polling is Task_Lwip, SCHED_MS(1) above */
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
