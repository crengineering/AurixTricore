/**********************************************************************************************************************
 * \file Mpu6050.c
 * \brief InvenSense MPU-6050 6-axis IMU driver (I2C0) — see Mpu6050.h.
 *
 * Register map and scaling follow the MPU-6000/MPU-6050 Register Map and
 * Descriptions (InvenSense RM-MPU-6000A, Rev 4.2) and the Product Specification
 * (PS-MPU-6000A). Unlike the BMP388 (little-endian, factory-compensated), the
 * MPU-6050 returns raw signed 16-bit counts, big-endian (high byte first), that
 * a fixed full-scale sensitivity converts to g / deg/s.
 *
 * Bring-up history: early versions read all-zero acc, gyro AND temperature
 * while WHO_AM_I and every config readback were correct, and the part was twice
 * misdiagnosed as a dead/counterfeit chip. The fault was NOT in this driver: the
 * I2C master never generated a STOP condition (iLLD defaults SOPE = 0), so the
 * MPU-6050 saw one endless read burst and held its sensor registers frozen at
 * whatever the first read latched — zeros, if that read followed a DEVICE_RESET.
 * Fixed in I2c.c (stopOnPacketEnd). See docs/MPU6050.md §7 before changing any
 * of the init steps below; several of them were wrongly blamed along the way.
 *********************************************************************************************************************/
#include "Mpu6050.h"
#include "I2c.h"
#include "IfxStm.h"

/* --- Register map (RM-MPU-6000A Table, §3) --- */
#define MPU6050_REG_SMPLRT_DIV        (0x19u)
#define MPU6050_REG_CONFIG            (0x1Au)
#define MPU6050_REG_GYRO_CONFIG       (0x1Bu)
#define MPU6050_REG_ACCEL_CONFIG      (0x1Cu)
#define MPU6050_REG_ACCEL_XOUT_H      (0x3Bu)   /* 14-byte burst: acc(6) temp(2) gyro(6) */
#define MPU6050_REG_USER_CTRL         (0x6Au)
#define MPU6050_REG_PWR_MGMT_1        (0x6Bu)
#define MPU6050_REG_PWR_MGMT_2        (0x6Cu)
#define MPU6050_REG_WHO_AM_I          (0x75u)

#define MPU6050_BURST_LEN         (14u)

/* PWR_MGMT_1 (RM §4.28): DEVICE_RESET (bit 7) restores the power-on defaults —
 * used to kick a device whose sampling never started. */
#define MPU6050_DEVICE_RESET      (0x80u)

/* PWR_MGMT_1 (RM §4.28): clear SLEEP (reset default = 1), CLKSEL = 0 (internal
 * 8 MHz oscillator).
 *
 * CLKSEL = 0 is EMPIRICALLY PROVEN on this exact GY-521: a minimal Arduino
 * sketch whose only setup write is PWR_MGMT_1 = 0x00 streams valid acc / gyro
 * / temperature from it. RM §4.28 recommends a gyro-PLL reference (CLKSEL = 1)
 * for stability; that is a legitimate future change, but re-validate on hardware
 * rather than assuming, and keep it a separate step from anything else.
 *
 * This write MUST be ACK-checked: if it is dropped, the device stays asleep and
 * the data registers all read 0. */
#define MPU6050_PWR_WAKE          (0x00u)

/* PWR_MGMT_1 bits that must read back after the wake: SLEEP (6) and the CLKSEL
 * field (2:0) — i.e. awake, on the selected clock. Verified explicitly so a
 * silently-still-asleep device fails init instead of streaming zeros. */
#define MPU6050_PWR1_CHECK_MASK   (0x4Fu)

/* PWR_MGMT_2 (RM §4.29): STBY bits for all six acc/gyro axes = 0 -> every
 * axis active. Written explicitly so a stray standby state can't silently zero
 * the acc/gyro outputs while temperature still reads. */
#define MPU6050_PWR2_ALL_ACTIVE   (0x00u)

/* USER_CTRL (RM §4.27) = 0: FIFO_EN, I2C_MST_EN and the aux-bus/FIFO resets all
 * off. Written explicitly because I2C_MST_EN (bit 5) points the internal master
 * at the auxiliary bus, and on the GY-521 the XDA/XCL pads are left FLOATING
 * (see Mpu6050.h pad map). Defensive rather than corrective — a power-on reset
 * already clears it, and it was measured at 0x00 throughout bring-up — but it is
 * the one register that can silently stall the sample path, so it is now both
 * written and included in Mpu6050_debugDump(). */
#define MPU6050_USER_CTRL_VAL     (0x00u)

/* CONFIG (RM §4.3): DLPF_CFG = 3 -> ~44 Hz acc / 42 Hz gyro bandwidth and a
 * 1 kHz internal gyro output rate. Filters out frame/motor vibration well above
 * the attitude bandwidth of interest. */
#define MPU6050_CONFIG_VAL        (0x03u)

/* SMPLRT_DIV (RM §4.2): sample rate = 1 kHz / (1 + div). div = 9 -> 100 Hz
 * register-update rate (the driver polls at 50 Hz; a faster internal rate keeps
 * the polled sample fresh). */
#define MPU6050_SMPLRT_DIV_VAL    (0x09u)

/* GYRO_CONFIG (RM §4.4): FS_SEL = 3 -> +/-2000 deg/s. Wide range for the fast
 * body rates of a quadcopter. Reg value = FS_SEL << 3. */
#define MPU6050_GYRO_CONFIG_VAL   (0x18u)
#define MPU6050_GYRO_FULLSCALE    (2000.0f)

/* ACCEL_CONFIG (RM §4.5): AFS_SEL = 2 -> +/-8 g. Captures flight manoeuvres
 * without clipping while keeping resolution. Reg value = AFS_SEL << 3. */
#define MPU6050_ACCEL_CONFIG_VAL  (0x10u)
#define MPU6050_ACCEL_FULLSCALE   (8.0f)

/* Signed 16-bit full scale: raw / 2^15 * fullscale. */
#define MPU6050_COUNTS_FULLSCALE  (32768.0f)
#define MPU6050_GYRO_SCALE        (MPU6050_GYRO_FULLSCALE  / MPU6050_COUNTS_FULLSCALE)  /* deg/s per LSB */
#define MPU6050_ACCEL_SCALE       (MPU6050_ACCEL_FULLSCALE / MPU6050_COUNTS_FULLSCALE)  /* g per LSB */

/* Temperature (RM §4.18): degC = raw / 340 + 36.53. */
#define MPU6050_TEMP_SENS         (340.0f)
#define MPU6050_TEMP_OFFSET       (36.53f)

/* Startup delays use the running STM0 (100 MHz, see Configuration.h
 * IFX_CFG_STM_TICKS_PER_MS). init runs once before the scheduler, so blocking
 * here is fine. Reads via IfxStm_waitTicks are non-destructive to STM0. */
#define MPU6050_STM_TICKS_PER_MS  (100000u)
#define MPU6050_RESET_MS          (100u)     /* post DEVICE_RESET settle (DS)   */
#define MPU6050_SETTLE_MS         (100u)     /* post-config sample-path settle  */
#define MPU6050_WAKE_MS           (50u)      /* clock/oscillator start-up        */

/* Auto-recovery: Mpu6050_read() is called at 50 Hz, so this retries the full
 * bring-up about once a second while the device is missing. Needed because a
 * supply glitch (a nudged jumper) drops the MPU back into SLEEP: the bus stays
 * healthy, but the part answers nothing until it is configured again, and
 * without this it would stay dead until the next power cycle. */
#define MPU6050_RECOVERY_PERIOD  (50u)

static boolean s_mpuPresent  = FALSE;

static void Mpu6050_delayMs(uint32 ms)
{
    IfxStm_waitTicks(&MODULE_STM0, ms * MPU6050_STM_TICKS_PER_MS);
}

boolean Mpu6050_readWhoAmI(uint8 *whoAmI)
{
    return I2c_readReg(MPU6050_I2C_ADDR, MPU6050_REG_WHO_AM_I, whoAmI, 1u);
}

/* cppcheck-suppress misra-c2012-8.7 ; deviation: retained in the peripheral
 * driver pool. The hardware was removed on 2026-07-31, so this function has
 * no caller today, but external linkage is intentional — the driver is kept
 * so the device can be re-fitted by configuration rather than by rewriting
 * it. See docs/PINNING.md 2.2/2.3. Remove the deviation once the driver
 * registration table gives every pool driver a caller. */
boolean Mpu6050_debugDump(uint8 *cfg4, uint8 *pwr3, uint8 *raw14)
{
    boolean ok;

    /* cfg4 = SMPLRT_DIV, CONFIG, GYRO_CONFIG, ACCEL_CONFIG (0x19..0x1C) — reads
     * back what init wrote, to prove config actually stuck in silicon. */
    ok = I2c_readReg(MPU6050_I2C_ADDR, MPU6050_REG_SMPLRT_DIV, cfg4, 4u);
    if (ok != FALSE)
    {
        /* pwr3 = USER_CTRL, PWR_MGMT_1, PWR_MGMT_2 (0x6A..0x6C) — the three
         * registers that decide whether the sample path runs at all. */
        ok = I2c_readReg(MPU6050_I2C_ADDR, MPU6050_REG_USER_CTRL, pwr3, 3u);
    }
    if (ok != FALSE)
    {
        ok = I2c_readReg(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, raw14, MPU6050_BURST_LEN);
    }
    return ok;
}

/* Assemble a big-endian signed 16-bit sample: raw[msbIndex] is the high byte. */
static sint16 Mpu6050_be16(const uint8 *p, uint8 msbIndex)
{
    uint16 raw = (uint16)(((uint16)p[msbIndex] << 8) | (uint16)p[msbIndex + 1u]);
    return (sint16)raw;
}

boolean Mpu6050_init(void)
{
    uint8   whoAmI = 0u;
    boolean ok;

    s_mpuPresent = FALSE;

    /* Full power-on reset, then wake. Safe now that the I2C master emits a real
     * STOP (I2c.c, stopOnPacketEnd) — before that fix DEVICE_RESET looked fatal,
     * because it zeroes the sensor registers and the very first burst read then
     * latched those zeros permanently. Reset writes ignore ACK (a fresh device
     * may hold the bus briefly on the first transaction). */
    (void)I2c_writeByte(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, MPU6050_DEVICE_RESET);
    Mpu6050_delayMs(MPU6050_RESET_MS);

    /* WHO_AM_I gate (reads work even while the device sleeps). */
    ok = Mpu6050_readWhoAmI(&whoAmI);
    if ((ok != FALSE) && (whoAmI == MPU6050_WHO_AM_I_VALUE))
    {
        /* Checked wake: clear SLEEP, internal-oscillator clock (see PWR_WAKE). */
        ok = I2c_writeByte(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_WAKE);
        Mpu6050_delayMs(MPU6050_WAKE_MS);

        /* No SIGNAL_PATH_RESET: DEVICE_RESET above already resets the signal
         * paths, and the known-good Arduino reference does not issue one either.
         * (It was briefly blamed for the all-zero reads; it was not the cause.) */
        if (ok != FALSE)
        {
            ok = I2c_writeByte(MPU6050_I2C_ADDR, MPU6050_REG_USER_CTRL, MPU6050_USER_CTRL_VAL);
        }
        if (ok != FALSE)
        {
            ok = I2c_writeByte(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_2, MPU6050_PWR2_ALL_ACTIVE);
        }
        if (ok != FALSE)
        {
            ok = I2c_writeByte(MPU6050_I2C_ADDR, MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_DIV_VAL);
        }
        if (ok != FALSE)
        {
            ok = I2c_writeByte(MPU6050_I2C_ADDR, MPU6050_REG_CONFIG, MPU6050_CONFIG_VAL);
        }
        if (ok != FALSE)
        {
            ok = I2c_writeByte(MPU6050_I2C_ADDR, MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_CONFIG_VAL);
        }
        if (ok != FALSE)
        {
            ok = I2c_writeByte(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_VAL);
        }

        /* Prove the device really left SLEEP and kept the selected clock source.
         * A dropped or ignored wake is otherwise invisible: every register read
         * still succeeds, the data registers simply stay at 0. */
        if (ok != FALSE)
        {
            uint8 pwr1 = 0u;
            ok = I2c_readReg(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, &pwr1, 1u);
            if ((ok != FALSE) && ((pwr1 & MPU6050_PWR1_CHECK_MASK) != MPU6050_PWR_WAKE))
            {
                ok = FALSE;
            }
        }

        /* Let the sample path settle before the first read. Straight after
         * DEVICE_RESET + configuration the first burst still carries a partial
         * sample (acc Z ~0.1 g instead of 1 g). */
        if (ok != FALSE)
        {
            Mpu6050_delayMs(MPU6050_SETTLE_MS);
        }
    }
    else
    {
        ok = FALSE;      /* chip absent or wrong WHO_AM_I */
    }

    s_mpuPresent = ok;
    return ok;
}

/* cppcheck-suppress misra-c2012-8.7 ; deviation: retained in the peripheral
 * driver pool. The hardware was removed on 2026-07-31, so this function has
 * no caller today, but external linkage is intentional — the driver is kept
 * so the device can be re-fitted by configuration rather than by rewriting
 * it. See docs/PINNING.md 2.2/2.3. Remove the deviation once the driver
 * registration table gives every pool driver a caller. */
boolean Mpu6050_read(Mpu6050_Sample *sample)
{
    static uint16 s_recovery = 0u;   /* block scope: used only here (MISRA 8.9) */
    uint8   raw[MPU6050_BURST_LEN];
    boolean ok = FALSE;

    if (s_mpuPresent == FALSE)
    {
        /* Device missing: probe periodically so it comes back on its own once
         * the wiring/supply is restored.
         *
         * The probe is a single WHO_AM_I read (~1 ms) and the full bring-up
         * runs ONLY once that answers. Mpu6050_init() blocks for ~250 ms of
         * reset and settle delays, so running it blindly every second would
         * stall the scheduler - and therefore the Ethernet polling and the
         * control loop - the entire time a wire is off. Re-init is required
         * rather than optional: unplugging the sensor power-cycles it back to
         * SLEEP with its configuration lost. */
        s_recovery++;
        if (s_recovery >= MPU6050_RECOVERY_PERIOD)
        {
            uint8 whoAmI = 0u;

            s_recovery = 0u;
            if ((Mpu6050_readWhoAmI(&whoAmI) != FALSE)
                && (whoAmI == MPU6050_WHO_AM_I_VALUE))
            {
                (void)Mpu6050_init();
            }
        }
    }
    else
    {
        ok = I2c_readReg(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, raw, MPU6050_BURST_LEN);
        if (ok == FALSE)
        {
            /* Lost it. Drop presence so the branch above re-runs the bring-up
             * instead of streaming stale values forever. */
            s_mpuPresent  = FALSE;
            s_recovery = 0u;
        }
        else
        {
            /* Burst layout (big-endian, RM §4.17-4.19):
             *   acc X/Y/Z @ 0,2,4 | temp @ 6 | gyro X/Y/Z @ 8,10,12 */
            sample->acc[0] = (float32)Mpu6050_be16(raw, 0u)  * MPU6050_ACCEL_SCALE;
            sample->acc[1] = (float32)Mpu6050_be16(raw, 2u)  * MPU6050_ACCEL_SCALE;
            sample->acc[2] = (float32)Mpu6050_be16(raw, 4u)  * MPU6050_ACCEL_SCALE;

            sample->tempC    = ((float32)Mpu6050_be16(raw, 6u) / MPU6050_TEMP_SENS)
                               + MPU6050_TEMP_OFFSET;

            sample->gyro[0]  = (float32)Mpu6050_be16(raw, 8u)  * MPU6050_GYRO_SCALE;
            sample->gyro[1]  = (float32)Mpu6050_be16(raw, 10u) * MPU6050_GYRO_SCALE;
            sample->gyro[2]  = (float32)Mpu6050_be16(raw, 12u) * MPU6050_GYRO_SCALE;
        }
    }
    return ok;
}
