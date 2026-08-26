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
#include "fusion.h"
#include "Ahrs.h"
#include "FusionCal.h"
#include "SysTime.h"
#include "Nvm.h"
#include <math.h>

/* Temporary bring-up switch: 1 = drive P22.8/P22.7 as GPIO instead of starting
 * the QSPI, to prove whether those pads can drive at all. Set back to 0. */
#define SPI_PAD_TEST  (0)
#include "PeriphDiag.h"
#include "EthStats.h"
#include "CtrlReplay.h"     /* ASW replay harness; single BSW->ASW init
                             * callout — documented deviation, see
                             * docs/CTRL_REPLAY.md                      */
#define MEAS_SEA_LEVEL_PA   (101325.0f)
#define MEAS_SEA_LEVEL_MIN  (80000.0f)       /* ~2 km above sea level QNH floor  */
#define MEAS_SEA_LEVEL_MAX  (120000.0f)      /* well above any real QNH          */
#define MEAS_ALT_SCALE      (44330.0f)       /* [m] */
#define MEAS_ALT_EXPONENT   (0.190294957f)   /* 1 / 5.25588 (barometric formula) */


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

/* One-time raw ICM-42688-P register dump at boot. Third sensor in a row with
 * no device datasheet available (docs/ICM42688P.md 4 — the PDF is the EVB
 * user guide), so this is again how the register assumptions meet silicon.
 *
 * WHO_AM_I must read 0x47. A 0x00 or 0xFF means nothing is coming back at all,
 * and on this bus that points at the MRST pad mode or the level shifters
 * before it points at the part. PWR_MGMT0 = 0x0F is the other one worth
 * checking: leave it at its reset value and every axis reads zero forever
 * while the bus looks perfectly healthy. */
static void icm42688DebugDump(void)
{
    uint8 cfg[ICM42688_DUMP_CFG_LEN];
    uint8 raw[14];

    if (Icm42688_debugDump(cfg, raw) != FALSE)
    {
        uint8 i;
        Uart_print("ICM42688 WHO/PWR/GYRO/ACCEL/INT=");
        for (i = 0u; i < ICM42688_DUMP_CFG_LEN; i++)
        {
            uartHexByte(cfg[i]);
            Uart_print(" ");
        }
        Uart_print(" burst(T,A,G)=");
        for (i = 0u; i < 14u; i++)
        {
            uartHexByte(raw[i]);
            Uart_print(" ");
        }
        Uart_println("");
    }
    else
    {
        Uart_println("ICM42688 dump failed - no response on QSPI0");
    }
}

#if SPI_PAD_TEST
/* Drive each Port 22 pin high then low as plain GPIO and read the pad back.
 * A pad that drives reports hi=1/lo=0. A pad that cannot drive (owned by
 * another function, or not actually this pin) reports whatever the external
 * network imposes — for the SCLK/MOSI/CS lines that is the 1k/2k divider with
 * the EVB's 10k pull-up, which sits at 0.5 V and reads back as 0. */
static void padSelfTestPort(Ifx_P *port, const char *name,
                            const uint8 *padPins, uint8 count)
{
    uint8 i;

    Uart_print("PAD SELF-TEST ");
    Uart_println(name);
    for (i = 0u; i < count; i++)
    {
        const uint8 pin = padPins[i];
        uint8       hi;
        uint8       lo;

        IfxPort_setPinModeOutput(port, pin, IfxPort_OutputMode_pushPull,
                                 IfxPort_OutputIdx_general);
        IfxPort_setPinPadDriver(port, pin, IfxPort_PadDriver_ttlSpeed1);

        IfxPort_setPinHigh(port, pin);
        IfxStm_waitTicks(&MODULE_STM0, 100000u);      /* 1 ms settle */
        hi = (uint8)IfxPort_getPinState(port, pin);

        IfxPort_setPinLow(port, pin);
        IfxStm_waitTicks(&MODULE_STM0, 100000u);
        lo = (uint8)IfxPort_getPinState(port, pin);

        Uart_print("  pin ");
        uartHexByte(pin);
        Uart_print(" hi=");
        uartHexByte(hi);
        Uart_print(" lo=");
        uartHexByte(lo);
        Uart_println((hi == 1u) && (lo == 0u) ? "  DRIVES" : "  NOT DRIVING");

        /* hand the pin back as an input so Spi_init() can claim it cleanly */
        IfxPort_setPinMode(port, pin, IfxPort_Mode_inputNoPullDevice);
    }
}

static void padSelfTest(void)
{
    /* Port 22: 4,5,6 are unwired and act as the control group. */
    static const uint8 p22[] = { 4u, 5u, 6u, 7u, 8u, 9u, 10u, 11u };
    /* Port 15: the candidate QSPI2 set (SCLK 3/6/8, MOSI 5/6, MISO 7, CS 2). */
    static const uint8 p15[] = { 1u, 2u, 3u, 5u, 6u, 7u, 8u };

    /* Port 20: QSPI0's other SCLK pins are P20.11 and P20.13 (the core-health
     * LEDs D306/D308). The user has cleared using them, and keeping QSPI0
     * turns a four-wire relocation into moving SCLK alone. */
    static const uint8 p20[] = { 11u, 13u };

    padSelfTestPort(&MODULE_P22, "P22.x", p22, (uint8)(sizeof(p22)));
    padSelfTestPort(&MODULE_P15, "P15.x", p15, (uint8)(sizeof(p15)));
    padSelfTestPort(&MODULE_P20, "P20.x (QSPI0 SCLK alternates)", p20, (uint8)(sizeof(p20)));
}
#endif  /* SPI_PAD_TEST */

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
    float32 baroAlt = 0.0f;

    /* Called unconditionally: Bmp581_read() owns the presence state and uses
     * these calls to probe for a reconnected sensor. Gating on a presence flag
     * here would make that recovery unreachable — which is exactly why a
     * replugged barometer never came back, and why no such accessor exists. */
    present = Bmp581_read(&pressPa, &tempC);


    /* alt calculation */
    float32 p0 = (float32)g_xcpNvm.seaLevelPa;

    if ((p0 < MEAS_SEA_LEVEL_MIN) || (p0 > MEAS_SEA_LEVEL_MAX))
    {
        p0 = MEAS_SEA_LEVEL_PA;         /* guard against a bad/zero NVM value */
    }

    /* International barometric formula: h = 44330 * (1 - (P/P0)^0.190295). */
    baroAlt    = MEAS_ALT_SCALE * (1.0f - powf(pressPa / p0, MEAS_ALT_EXPONENT));
    Fusion_setBaroAlt(baroAlt, present);

    measurementsSetBaro(present, pressPa, tempC, baroAlt);
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

    /* Latch it for the attitude filter, which consumes it on the next IMU
     * tick. Hard-iron correction and the mounting transform happen in there,
     * not here -- sample.mag stays the RAW field so the published values and
     * tools/mag_cal.py keep seeing what the sensor actually reported. */
    Ahrs_setMag(sample.mag, present);

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

static void Task_Imu(void)
{
    Icm42688_Sample sample = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 0.0f };
    boolean         present;
    /* uint32_t, not uint32: SysTime.h is deliberately iLLD-free and takes the
     * stdint type, and on TASKING the two are distinct types of equal width. */
    static uint32_t last_ticks = 0u;
    boolean         fusion_valid = TRUE;
    FusionValues    fusion;
    Ahrs_Values     ahrs;

    /* Called unconditionally, like the other sensor tasks: Icm42688_read()
     * owns the presence state and uses these calls to probe for a reconnected
     * sensor. */
    present = Icm42688_read(&sample);

    /* Attitude first, then navigation: the channel filters need acceleration
     * resolved into NED, and only the AHRS can do that. */
    float32 elapsedTime = SysTime_getTimeElapsedS(&last_ticks);
    if ( (elapsedTime < 0.001f) ||
         (elapsedTime > 0.2f))
    {
        fusion_valid = FALSE;
    }

    Ahrs_update(&ahrs, sample.acc, sample.gyro, elapsedTime,
                (fusion_valid != FALSE) && (present != FALSE));

    /* Gate the navigation filter on the attitude being usable, not merely on
     * the IMU answering. While the AHRS is still averaging the gyro bias or
     * waiting to align, its projection is meaningless and integrating it would
     * put a real offset into the velocity before the barometer ever sees it. */
    if (ahrs.state != (uint8)AHRS_RUNNING)
    {
        fusion_valid = FALSE;
    }

    Fusion_update(&fusion, ahrs.accNed, elapsedTime,
                  (fusion_valid != FALSE) && (present != FALSE));
    measurementsSetFusion(&fusion, &ahrs, present, elapsedTime);

    {
        /* Raw angular rate: there is no bias estimator in the tree, so the
         * published value still carries the sensor offset. */
        measurementsSetImu(present, sample.acc, sample.gyro, sample.tempC);

        /* |a| must stay inside the configured +/-16 g full scale; a sustained
         * 0 g means a dead element rather than free fall, which never lasts
         * seconds on the bench. The liveness sum spans every axis, so it only
         * freezes if the whole sample block stops updating -- exactly the
         * failure that hid for days on the MPU-6050 while the bus and the
         * presence flag both looked healthy. */
        const float32 accMagSq = (sample.acc[0] * sample.acc[0])
                               + (sample.acc[1] * sample.acc[1])
                               + (sample.acc[2] * sample.acc[2]);
        boolean plausible = FALSE;
        if ((accMagSq > 0.0025f) && (accMagSq < 289.0f)
            && (sample.tempC > -40.0f) && (sample.tempC < 105.0f))
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
    /* GNSS: publish + report */
    GnssM9N_Sample gnss_sample = {0};
    boolean        gnssPresent;
    boolean        gnssPlausible;

    gnssPresent = GnssM9N_read(&gnss_sample);

    /* Plausible = the receiver accepted every configuration command we sent.
     *
     * Without this the config path is fire-and-forget: a rejected key changes
     * nothing observable on the wire, so a silently unconfigured receiver
     * would look perfectly healthy. Raising DIAG_GNSS_IMPLAUSIBLE puts it on
     * the diagnostics LED and in the GUI instead.
     *
     * NOT gated on navOk -- "no satellites" is the normal indoor state, not a
     * fault, and a diagnostics word that is permanently red is one nobody
     * reads. PeriphDiag debounces this for PD_IMPLAUSIBLE_TICKS (1 s) and only
     * evaluates it when the read succeeded, so the boot window is covered. */
    gnssPlausible = (gnss_sample.cfgOk != 0u) ? TRUE : FALSE;
    measurementsSetGnss(gnssPresent, gnss_sample);

    /* Feed the navigation filter. navOk -- not fixOk -- is the gate: it also
     * requires a 3-D fix and an hAcc inside the usable band, which is what
     * keeps a barely-acquired solution from dragging the origin around.
     * Fusion_setGnss ignores a repeat of the same iTOW, so calling it at 10 Hz
     * on a 1 Hz solution is safe. */
    Fusion_setGnss(gnss_sample.latRaw, gnss_sample.lonRaw, gnss_sample.altM,
                   gnss_sample.speedMps, gnss_sample.headingDeg,
                   gnss_sample.hAccM, gnss_sample.iTOW,
                   (gnssPresent != FALSE) && (gnss_sample.navOk != 0u));
    PeriphDiag_report(PERIPH_DIAG_GNSS, gnssPresent, gnssPlausible, (float)gnss_sample.rxBytes);

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
    (void)Scheduler_addTask(&g_sched, Task_Imu,  SCHED_MS(20u));  /* 50 Hz IMU (QSPI0) */
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
    PeriphDiag_setFitted(PERIPH_DIAG_IMU,  TRUE);
    PeriphDiag_setFitted(PERIPH_DIAG_MAG,  TRUE);
    PeriphDiag_setFitted(PERIPH_DIAG_GNSS, TRUE);

    /* Flight IMU on its own bus: QSPI0, nothing shared with the I2C sensors.
     * Brought up after them so a failure here cannot be confused with a
     * problem on the shared bus. */
#if SPI_PAD_TEST
    padSelfTest();
#endif

    Spi_init();
    if (Icm42688_init() != FALSE)
    {
        Uart_print("ICM-42688-P detected (WHO_AM_I 0x47) in SPI mode ");
        uartHexByte(Spi_getMode());
        Uart_println("");
    }
    else
    {
        uint8 who = 0u;

        Uart_println("ICM-42688-P not found");
        Spi_setMode(SPI_MODE_0);
        (void)Icm42688_readWhoAmI(&who);
        Uart_print("  WHO_AM_I mode0=");
        uartHexByte(who);
        who = 0u;
        Spi_setMode(SPI_MODE_3);
        (void)Icm42688_readWhoAmI(&who);
        Uart_print("  mode3=");
        uartHexByte(who);
        Uart_println("");
    }
    icm42688DebugDump();
    FusionCal_init();       /* estimator tuning defaults, BEFORE the filters */
    Ahrs_init();            /* start the gyro-bias calibration; hold still  */
    Fusion_init();          /* zero every channel state and covariance      */

    /* init of the GNSS*/
    if (GnssM9N_init() != FALSE)
    {
        Uart_println("Gnss-NEO-M9N init successfully");
    }
    else
    {
        Uart_println("Gnss-NEO-M9N init NOT successfully");
    }

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
