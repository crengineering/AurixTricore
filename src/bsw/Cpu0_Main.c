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
#include "Bmp388.h"
#include "Mpu6050.h"
#include "Ahrs.h"
#include "PeriphDiag.h"
#include "EthStats.h"
#include "SysTime.h"
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

/* One-time raw MPU-6050 register dump at boot for bring-up diagnosis.
 * If the burst ever reads all-zero or never changes between resets, read
 * docs/MPU6050.md section 7 before suspecting the sensor hardware. */
static void mpuDebugDump(void)
{
    uint8 cfg[4] = { 0u, 0u, 0u, 0u };
    uint8 pwr[3] = { 0u, 0u, 0u };
    uint8 raw[14];

    if (Mpu6050_debugDump(cfg, pwr, raw) != FALSE)
    {
        uint8 i;
        Uart_print("MPU cfg(SMPLRT/CFG/GYRO/ACCEL)=");
        for (i = 0u; i < 4u; i++)
        {
            uartHexByte(cfg[i]);
            Uart_print(" ");
        }
        Uart_print("USER_CTRL/PWR1/PWR2=");
        for (i = 0u; i < 3u; i++)
        {
            uartHexByte(pwr[i]);
            Uart_print(" ");
        }
        Uart_print("burst=");
        for (i = 0u; i < 14u; i++)
        {
            uartHexByte(raw[i]);
            Uart_print(" ");
        }
        Uart_println("");
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

    /* Called unconditionally, like Task_Imu: Bmp388_read() owns the presence
     * state and uses these calls to probe for a reconnected sensor. Gating on
     * Bmp388_isPresent() here would make that recovery unreachable, which is
     * exactly why a replugged barometer never came back. */
    present = Bmp388_read(&pressPa, &tempC);
    measurementsSetBaro(present, pressPa, tempC);

    /* Plausibility bands are the sensor's physical envelope, not tuning:
     * 300..1200 hPa spans sea level to well above any altitude this airframe
     * reaches, and the BMP388 is specified from -40 to +85 degC. The liveness
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

static void Task_Imu(void)
{
    Mpu6050_Sample sample = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 0.0f };
    boolean        present;
    static uint32  lastTicks = 0u;
    uint32         nowTicks;
    uint32         elapsed;
    float32        dt;

    /* Called unconditionally: Mpu6050_read() owns the presence state and uses
     * these calls to re-run the bring-up periodically after the device drops
     * off the bus. Gating on a presence flag here would make that recovery
     * unreachable, which is why no such accessor exists any more. */
    present = Mpu6050_read(&sample);

    /* Measured dt, not the nominal 20 ms: the scheduler dispatches on a "period
     * elapsed" test, so the real interval jitters and integrating the nominal
     * value would bias the attitude. STM0 ticks are 10 ns. */
    nowTicks = SysTime_getTicks();
    elapsed  = nowTicks - lastTicks;
    dt       = (float32)elapsed * 1e-8f;
    lastTicks = nowTicks;

    /* AHRS first: it owns the gyro bias, and the published rates below are
     * bias-corrected using the value it measured at start-up. */
    Ahrs_update(sample.acc, sample.gyro, dt, present);
    measurementsSetAttitude();

    {
        float32 bias[3];
        float32 gyrCorr[3];
        uint8   i;

        /* Publish the CORRECTED rate, not the raw one. This unit has a ~21.8
         * deg/s offset on Y; showing it raw makes a stationary board look like
         * it is yawing. The bias itself stays visible separately (biasX/Y/Z),
         * so the raw value is still recoverable as corrected + bias.
         * Zero until calibration finishes, so this is a no-op until then. */
        Ahrs_getGyroBias(bias);
        for (i = 0u; i < 3u; i++)
        {
            gyrCorr[i] = sample.gyro[i] - bias[i];
        }
        measurementsSetImu(present, sample.acc, gyrCorr, sample.tempC);

        /* |a| must stay inside the +/-8 g full scale; a sustained 0 g means a
         * dead element rather than free fall, which never lasts seconds on the
         * bench. Temperature bounds are the MPU-6050's specified range. The
         * liveness sum spans every axis, so it only freezes if the whole
         * sample block stops updating -- exactly the failure that hid for days
         * on 2026-07-30 while the bus and presence flag looked healthy. */
        const float32 accMagSq = (sample.acc[0] * sample.acc[0])
                               + (sample.acc[1] * sample.acc[1])
                               + (sample.acc[2] * sample.acc[2]);
        boolean plausible = FALSE;
        if ((accMagSq > 0.0025f) && (accMagSq < 169.0f)
            && (sample.tempC > -40.0f) && (sample.tempC < 85.0f))
        {
            plausible = TRUE;
        }
        const float32 liveness = sample.acc[0] + sample.acc[1] + sample.acc[2]
                               + sample.gyro[0] + sample.gyro[1] + sample.gyro[2]
                               + sample.tempC;
        PeriphDiag_report(PERIPH_DIAG_IMU, present, plausible, liveness);
    }
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
    (void)Scheduler_addTask(&g_sched, Task_Imu,  SCHED_MS(20u));  /* 50 Hz IMU */
    Scheduler_addTask(&g_sched, Task_Measure100ms, SCHED_MS(100u));

    /* init persistent memory*/
    Nvm_bootInit();         /* load persistent parameters from DFLASH       */
    diagnosticsInit();      /* before measurementsInit: provides ADC scales */
    measurementsInit();

    /* init shared I2C0 sensor bus + first sensor (BMP388 barometer, CJMCU-388) */
    I2c_init();
    if (I2c_busIsIdle() != FALSE)
    {
        Uart_println("I2C0 bus idle (SCL+SDA released)");
    }
    else
    {
        Uart_println("I2C0 bus HELD - a slave is pulling SCL/SDA low");
    }
    if (Bmp388_init() != FALSE)
    {
        Uart_println("BMP388 detected (CHIP_ID 0x50)");
    }
    else
    {
        Uart_println("BMP388 not found - check wiring/address (0x77 vs 0x76)");
    }

    /* second sensor on the shared I2C0 bus: MPU-6050 IMU (GY-521), address 0x68 */
    if (Mpu6050_init() != FALSE)
    {
        Uart_println("MPU-6050 detected (WHO_AM_I 0x68)");
    }
    else
    {
        Uart_println("MPU-6050 not found - check wiring/address (0x68 vs 0x69)");
    }
    mpuDebugDump();     /* one-time raw register dump for bring-up diagnosis */

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
    Ahrs_init();            /* starts the gyro-bias calibration - hold still */
    Uart_println("Ethernet started");
    Uart_println("AHRS calibrating gyro bias - keep the board still ~2 s");

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
