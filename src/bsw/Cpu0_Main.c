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
#include "Ahrs.h"
#include "PeriphDiag.h"
#include "EthStats.h"
#include "CtrlReplay.h"     /* ASW replay harness; single BSW->ASW init
                             * callout — documented deviation, see
                             * docs/CTRL_REPLAY.md                      */

IFX_ALIGN(4) IfxCpu_syncEvent cpuSyncEvent = 0;

static Scheduler_t g_sched;
static Led_t       g_led;

/* print one byte as two hex digits (no UART formatted-print available) */
static void uartHexByte(uint8 v)
{
    static const char hexDigits[] = "0123456789ABCDEF";
    char buf[3];
    buf[0] = hexDigits[(v >> 4) & 0x0Fu];
    buf[1] = hexDigits[v & 0x0Fu];
    buf[2] = '\0';
    Uart_print(buf);
}

/* One-time raw BMP581 register dump at boot for bring-up diagnosis.
 *
 * Every register value in Bmp581.c was written from the BMP5 register map
 * without the datasheet in hand (docs/BMP581.md section 10), so this dump is how
 * those assumptions get checked against silicon. Read it in the order the
 * bring-up fails — see the Bmp581_debugDump() comment in Bmp581.h for what each
 * field should read; the short version is INT_STATUS bit 4 set (reset landed),
 * STATUS = nvm_rdy without nvm_err, OSR bit 6 set (pressure enabled) and ODR
 * bits 1:0 = 1 (converting). A burst that is all-zero or identical across two
 * resets means the device is not converting, not that the scaling is wrong. */
static void bmp581DebugDump(void)
{
    uint8 cfg[BMP581_DUMP_CFG_LEN];
    uint8 osrEff = 0u;
    uint8 raw[6];

    if (Bmp581_debugDump(cfg, &osrEff, raw) != FALSE)
    {
        uint8 i;
        Uart_print("BMP581 CHIP/REV/INT_STAT/STAT/IIR/OSR/ODR=");
        for (i = 0u; i < BMP581_DUMP_CFG_LEN; i++)
        {
            uartHexByte(cfg[i]);
            Uart_print(" ");
        }
        Uart_print("OSR_EFF=");
        uartHexByte(osrEff);
        Uart_print(" burst(T,P)=");
        for (i = 0u; i < 6u; i++)
        {
            uartHexByte(raw[i]);
            Uart_print(" ");
        }
        Uart_println("");
    }
    else
    {
        Uart_println("BMP581 dump failed - no ACK on the bus");
    }
}

/* One-time raw MMC5983MA register dump at boot, for the same reason as the
 * BMP581's: no device datasheet exists (docs/MMC5983MA.md section 6 — the PDF
 * is the prototyping board's guide and carries no register information), so
 * every constant in Mmc5983.c is unverified until silicon says otherwise.
 *
 * PRODUCT_ID must read 0x30; that one value proves the whole I2C path including
 * the CS strap. CTRL0/1/2 reading 0x00 is EXPECTED and not a failed write —
 * the control registers are write-only on this part. The data block is the real
 * evidence, and |B| in the measurement block is the decisive test. */
static void mmc5983DebugDump(void)
{
    uint8 cfg[MMC5983_DUMP_CFG_LEN];
    uint8 raw[7];

    if (Mmc5983_debugDump(cfg, raw) != FALSE)
    {
        uint8 i;
        Uart_print("MMC5983 PID/STATUS/CTRL0/CTRL1/CTRL2=");
        for (i = 0u; i < MMC5983_DUMP_CFG_LEN; i++)
        {
            uartHexByte(cfg[i]);
            Uart_print(" ");
        }
        Uart_print(" burst=");
        for (i = 0u; i < 7u; i++)
        {
            uartHexByte(raw[i]);
            Uart_print(" ");
        }
        Uart_println("");
    }
    else
    {
        Uart_println("MMC5983 dump failed - no ACK on the bus");
    }
}

static void Task_LedToggle(void)
{
    Led_toggle(&g_led);
}

static void Task_App10ms(void)
{
    /* TODO: add CPU0 application logic here */
}

static void Task_Baro(void)
{
    float32 pressPa = 0.0f;
    float32 tempC   = 0.0f;
    boolean present;

    /* Called unconditionally: Bmp581_read() owns the presence state and uses
     * these calls to probe for a reconnected sensor. Gating on a presence flag
     * here would make that recovery unreachable — which is exactly why a
     * replugged barometer never came back, and why no such accessor exists. */
    present = Bmp581_read(&pressPa, &tempC);
    measurementsSetBaro(present, pressPa, tempC);

    /* Plausibility bands are the sensor's physical envelope, not tuning:
     * 300..1200 hPa spans sea level to well above any altitude this airframe
     * reaches, and the BMP581 is specified from -40 to +85 degC. The liveness
     * sum moves with sensor noise on every sample, so a frozen value is a real
     * fault rather than a quiet signal. */
    {
        boolean plausible = FALSE;
        if ((pressPa > 30000.0f) && (pressPa < 120000.0f)
            && (tempC > -40.0f) && (tempC < 85.0f))
        {
            plausible = TRUE;
        }
        PeriphDiag_report(PERIPH_DIAG_BARO, present, plausible, pressPa + tempC);
    }
}

static void Task_Mag(void)
{
    Mmc5983_Sample sample = { { 0.0f, 0.0f, 0.0f }, 0.0f };
    boolean        present;

    /* Called unconditionally, like Task_Baro: Mmc5983_read() owns the presence
     * state and uses these calls to probe for a reconnected sensor. */
    present = Mmc5983_read(&sample);
    measurementsSetMag(present, sample.mag, sample.headingDeg);

    {
        /* The plausibility test here is stronger than the barometer's, because
         * |B| is a property of the LOCATION and not of the orientation: it must
         * stay put however the board is turned. Earth's field spans ~0.25..0.65
         * G worldwide (~0.48 G in Munich), and the band below is widened to
         * 0.15..2.0 G to tolerate the hard-iron offset of the board's own
         * magnetics without accepting a value that is wrong by a clean factor —
         * which is exactly what a bad scaling constant or a mis-assembled
         * 18-bit word would produce. */
        const float32 fieldSq = (sample.mag[0] * sample.mag[0])
                              + (sample.mag[1] * sample.mag[1])
                              + (sample.mag[2] * sample.mag[2]);
        boolean plausible = FALSE;
        if ((fieldSq > 0.0225f) && (fieldSq < 4.0f))
        {
            plausible = TRUE;
        }
        /* Liveness spans all three axes, so it only freezes if the whole sample
         * block stops updating. Heading is deliberately excluded: it is derived
         * from X and Y, so it would add no independent information. */
        PeriphDiag_report(PERIPH_DIAG_MAG, present, plausible,
                          sample.mag[0] + sample.mag[1] + sample.mag[2]);
    }
}

/* Publish "no IMU fitted" once at start-up.
 *
 * The MPU-6050 came off the bus on 2026-07-31 and its replacement (ICM-42688-P
 * on QSPI0) is not here yet, so there is no Task_Imu to run. Without this the
 * IMU and attitude fields of the XCP block would simply never be written and
 * the GUI would show whatever the RAM powered up with. Writing them once says
 * the true thing instead: absent, zeroed, AHRS in AHRS_NO_SENSOR. */
static void publishNoImu(void)
{
    static const float32 zero3[3] = { 0.0f, 0.0f, 0.0f };

    Ahrs_init();
    Ahrs_update(zero3, zero3, 0.0f, FALSE);   /* valid = FALSE -> AHRS_NO_SENSOR */
    measurementsSetAttitude();
    measurementsSetImu(FALSE, zero3, zero3, 0.0f);
}

static void Task_Measure100ms(void)
{
    Nvm_task100ms();        /* before diagnostics: fresh NVM fault state */
    measurementsUpdate();
    if (diagnosticsUpdate() != FALSE) {
        gpio_write(GPIO_P_00_0, GPIO_STATE_ON);
    } else {
        gpio_write(GPIO_P_00_0, GPIO_STATE_OFF);
    }
    gpio_calApply();        /* XCP overrides win over the diagnostics write above */
    measurementsSetSystemLoad();   /* per-core exec time + Ethernet utilisation */
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
    gpio_calInit();         /* XCP GPIO control block: all pins firmware-owned */

    /* init scheduler */
    Scheduler_init(&g_sched, &MODULE_STM0, 0u);
    Scheduler_addTask(&g_sched, Task_LedToggle, SCHED_MS(500u));
    Scheduler_addTask(&g_sched, Task_App10ms,   SCHED_MS(10u));
    (void)Scheduler_addTask(&g_sched, Task_Baro, SCHED_MS(20u));  /* 50 Hz barometer */
    (void)Scheduler_addTask(&g_sched, Task_Mag,  SCHED_MS(20u));  /* 50 Hz magnetometer */
    Scheduler_addTask(&g_sched, Task_Measure100ms, SCHED_MS(100u));

    /* init persistent memory*/
    Nvm_bootInit();         /* load persistent parameters from DFLASH       */
    diagnosticsInit();      /* before measurementsInit: provides ADC scales */
    measurementsInit();

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
    bmp581DebugDump();  /* one-time register dump for bring-up diagnosis */

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
    mmc5983DebugDump();

    /* Tell the peripheral diagnostics what this build actually expects to find.
     * The IMU slot must be declared unfitted or it would assert NO_RESPONSE
     * forever and leave the diagnostics permanently red over hardware that is
     * deliberately absent. */
    PeriphDiag_setFitted(PERIPH_DIAG_BARO, TRUE);
    PeriphDiag_setFitted(PERIPH_DIAG_IMU,  FALSE);
    PeriphDiag_setFitted(PERIPH_DIAG_MAG,  TRUE);
    publishNoImu();

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
    Uart_println("No IMU fitted - attitude held at zero (AHRS_NO_SENSOR)");

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
