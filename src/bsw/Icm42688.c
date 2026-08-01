/**********************************************************************************************************************
 * \file Icm42688.c
 * \brief TDK-InvenSense ICM-42688-P 6-axis IMU driver (QSPI0) — see Icm42688.h.
 *
 * ⚠️ Every register constant here is written from the ICM-42688-P register map
 * WITHOUT the device datasheet (only AN-000488, the EVB user guide, is
 * available and it contains no register information — docs/ICM42688P.md 4).
 * Icm42688_debugDump() is the instrument for confirming them against silicon;
 * WHO_AM_I is the gate that makes a wrong guess visible immediately.
 *
 * SPI framing: the first byte is the register address, and its MSB is the
 * direction — 1 for a read, 0 for a write. A burst read therefore costs
 * 1 + N bytes, and the first received byte is the turnaround and is discarded.
 *
 * Unlike the MPU-6050 this part outputs big-endian (high byte first) signed
 * 16-bit counts, and there is no compensation polynomial: the counts scale by
 * a fixed factor set by the configured full-scale range.
 *********************************************************************************************************************/
#include "Icm42688.h"
#include "Spi.h"
#include "IfxStm.h"

/* --- Register map (bank 0) --- */
#define ICM42688_REG_DEVICE_CONFIG   (0x11u)
#define ICM42688_REG_TEMP_DATA1      (0x1Du)   /* 0x1D..0x2A: temp(2) accel(6) gyro(6) */
#define ICM42688_REG_INT_STATUS      (0x2Du)
#define ICM42688_REG_PWR_MGMT0       (0x4Eu)
#define ICM42688_REG_GYRO_CONFIG0    (0x4Fu)
#define ICM42688_REG_ACCEL_CONFIG0   (0x50u)
#define ICM42688_REG_WHO_AM_I        (0x75u)

#define ICM42688_BURST_LEN           (14u)     /* temp(2) + accel(6) + gyro(6) */
#define ICM42688_SPI_READ            (0x80u)   /* MSB of the address byte */
#define ICM42688_ADDR_MASK           (0x7Fu)

/* DEVICE_CONFIG (0x11) bit 0: SOFT_RESET_CONFIG. */
#define ICM42688_SOFT_RESET          (0x01u)

/* PWR_MGMT0 (0x4E): GYRO_MODE (bits 3:2) = 0b11 low noise,
 *                   ACCEL_MODE (bits 1:0) = 0b11 low noise.
 * This is the write that wakes the part. Leave it at its reset value and every
 * sample reads zero while the bus looks perfectly healthy — the same class of
 * failure that hid on the MPU-6050 for days. */
#define ICM42688_PWR_MGMT0_VAL       (0x0Fu)

/* GYRO_CONFIG0 (0x4F): GYRO_FS_SEL (bits 7:5) = 0 -> +/-2000 dps,
 *                      GYRO_ODR (bits 3:0) = 0x06 -> 1 kHz. */
#define ICM42688_GYRO_CONFIG0_VAL    (0x06u)
#define ICM42688_GYRO_FULLSCALE_DPS  (2000.0f)

/* ACCEL_CONFIG0 (0x50): ACCEL_FS_SEL (bits 7:5) = 0 -> +/-16 g,
 *                       ACCEL_ODR (bits 3:0) = 0x06 -> 1 kHz.
 * The ODR is deliberately faster than the 50 Hz task: the data registers
 * always hold the newest completed sample, so a fast ODR costs nothing and
 * leaves headroom to raise the task rate later. */
#define ICM42688_ACCEL_CONFIG0_VAL   (0x06u)
#define ICM42688_ACCEL_FULLSCALE_G   (16.0f)

#define ICM42688_COUNTS_FULLSCALE    (32768.0f)
#define ICM42688_GYRO_SCALE   (ICM42688_GYRO_FULLSCALE_DPS / ICM42688_COUNTS_FULLSCALE)
#define ICM42688_ACCEL_SCALE  (ICM42688_ACCEL_FULLSCALE_G  / ICM42688_COUNTS_FULLSCALE)

/* Die temperature: degC = raw/132.48 + 25. */
#define ICM42688_TEMP_SENS           (132.48f)
#define ICM42688_TEMP_OFFSET         (25.0f)

/* Timing. STM0 runs at ~100 MHz. The soft reset needs ~1 ms to complete, and
 * the configuration registers must not be written for a short settling period
 * after PWR_MGMT0 wakes the analog front end — both are generous here because
 * they run once, before the scheduler starts. */
#define ICM42688_STM_TICKS_PER_MS    (100000u)
#define ICM42688_RESET_MS            (10u)
#define ICM42688_WAKE_MS             (10u)

/* Hot-plug recovery: retry this often (calls, i.e. 50 per second from the
 * 20 ms IMU task) while the device is missing. */
#define ICM42688_RECOVERY_PERIOD     (50u)     /* ~1 s */

static boolean s_icm42688Present = FALSE;

static void Icm42688_delayMs(uint32 ms)
{
    IfxStm_waitTicks(&MODULE_STM0, ms * ICM42688_STM_TICKS_PER_MS);
}

/* Read \p len bytes starting at \p reg. The transfer is 1 + len bytes and the
 * first received byte is the address turnaround, so the payload is shifted. */
static boolean Icm42688_readRegs(uint8 reg, uint8 *data, uint8 len)
{
    uint8   tx[1u + ICM42688_BURST_LEN];
    uint8   rx[1u + ICM42688_BURST_LEN];
    boolean ok = FALSE;

    if (len <= ICM42688_BURST_LEN)
    {
        uint8 i;

        for (i = 0u; i < (1u + len); i++)
        {
            tx[i] = 0u;
        }
        tx[0] = (uint8)((reg & ICM42688_ADDR_MASK) | ICM42688_SPI_READ);

        const uint16 frameLen = (uint16)len + 1u;   /* cast the object, not a
                                                     * composite (MISRA 10.8) */
        ok = Spi_transfer(tx, rx, frameLen);
        if (ok != FALSE)
        {
            for (i = 0u; i < len; i++)
            {
                data[i] = rx[i + 1u];
            }
        }
    }
    return ok;
}

static boolean Icm42688_writeReg(uint8 reg, uint8 value)
{
    uint8 tx[2];

    tx[0] = (uint8)(reg & ICM42688_ADDR_MASK);   /* MSB clear = write */
    tx[1] = value;
    return Spi_transfer(tx, NULL_PTR, 2u);
}

boolean Icm42688_readWhoAmI(uint8 *whoAmI)
{
    return Icm42688_readRegs(ICM42688_REG_WHO_AM_I, whoAmI, 1u);
}

/* Reset the part and check WHO_AM_I on whichever SPI mode is configured. */
static boolean Icm42688_probe(void)
{
    uint8   whoAmI = 0u;
    boolean ok;

    (void)Icm42688_writeReg(ICM42688_REG_DEVICE_CONFIG, ICM42688_SOFT_RESET);
    Icm42688_delayMs(ICM42688_RESET_MS);

    ok = Icm42688_readWhoAmI(&whoAmI);
    if (ok != FALSE)
    {
        if (whoAmI != ICM42688_WHO_AM_I_VALUE)
        {
            ok = FALSE;
        }
    }
    return ok;
}

boolean Icm42688_init(void)
{
    boolean ok;

    s_icm42688Present = FALSE;

    /* Probe both SPI modes rather than trusting one.
     *
     * The part accepts mode 0 and mode 3, and the device datasheet is not
     * available to say which this board powers up expecting. Guessing wrong is
     * invisible without a scope: the master clocks, CS asserts, every transfer
     * reports success, and MISO reads a flat 0x00 — because the slave never
     * recognises the address byte and so never drives SDO, leaving the EVB's
     * 100 kOhm pulldown to hold the line low. That is exactly what the first
     * bring-up produced. Trying both costs two transactions, once, at boot. */
    Spi_setMode(SPI_MODE_0);
    ok = Icm42688_probe();
    if (ok == FALSE)
    {
        Spi_setMode(SPI_MODE_3);
        ok = Icm42688_probe();
    }

    /* Power up first, then configure: the ranges and ODR below apply to an
     * already-running front end, and the part needs a moment after the mode
     * change before it accepts them. */
    if (ok != FALSE)
    {
        ok = Icm42688_writeReg(ICM42688_REG_PWR_MGMT0, ICM42688_PWR_MGMT0_VAL);
        Icm42688_delayMs(ICM42688_WAKE_MS);
    }
    if (ok != FALSE)
    {
        ok = Icm42688_writeReg(ICM42688_REG_GYRO_CONFIG0, ICM42688_GYRO_CONFIG0_VAL);
    }
    if (ok != FALSE)
    {
        ok = Icm42688_writeReg(ICM42688_REG_ACCEL_CONFIG0, ICM42688_ACCEL_CONFIG0_VAL);
    }

    s_icm42688Present = ok;
    return ok;
}

/* Assemble one big-endian signed 16-bit axis from the burst. */
static sint16 Icm42688_be16(const uint8 *p, uint8 msbIndex)
{
    /* Assemble first, then cast the finished object: casting the composite
     * expression to a different essential type breaks MISRA 10.8. */
    const uint16 raw = (uint16)(((uint16)p[msbIndex] << 8)
                                | (uint16)p[msbIndex + 1u]);
    return (sint16)raw;
}

boolean Icm42688_read(Icm42688_Sample *sample)
{
    /* Block scope (MISRA 8.9): only this function drives the retry. */
    static uint16 s_icm42688Recovery = 0u;

    uint8   raw[ICM42688_BURST_LEN];
    boolean ok = FALSE;

    if (s_icm42688Present == FALSE)
    {
        /* Device absent: probe periodically so it comes back on its own when
         * the wiring is restored. The probe is a single WHO_AM_I read and the
         * full bring-up runs only once that answers — a power-cycled part
         * comes back asleep with its configuration lost, so simply resuming
         * reads would return zeros forever. */
        s_icm42688Recovery++;
        if (s_icm42688Recovery >= ICM42688_RECOVERY_PERIOD)
        {
            uint8 whoAmI = 0u;

            s_icm42688Recovery = 0u;
            if ((Icm42688_readWhoAmI(&whoAmI) != FALSE)
                && (whoAmI == ICM42688_WHO_AM_I_VALUE))
            {
                (void)Icm42688_init();
            }
        }
    }
    else
    {
        ok = Icm42688_readRegs(ICM42688_REG_TEMP_DATA1, raw, ICM42688_BURST_LEN);
        if (ok == FALSE)
        {
            /* Lost it. Drop presence so the branch above starts probing
             * instead of retrying a dead device at 50 Hz forever. */
            s_icm42688Present  = FALSE;
            s_icm42688Recovery = 0u;
        }
        else
        {
            /* Layout: temp(0..1), accel X/Y/Z(2..7), gyro X/Y/Z(8..13). */
            sample->tempC = ((float32)Icm42688_be16(raw, 0u) / ICM42688_TEMP_SENS)
                            + ICM42688_TEMP_OFFSET;

            sample->acc[0] = (float32)Icm42688_be16(raw, 2u)  * ICM42688_ACCEL_SCALE;
            sample->acc[1] = (float32)Icm42688_be16(raw, 4u)  * ICM42688_ACCEL_SCALE;
            sample->acc[2] = (float32)Icm42688_be16(raw, 6u)  * ICM42688_ACCEL_SCALE;

            sample->gyro[0] = (float32)Icm42688_be16(raw, 8u)  * ICM42688_GYRO_SCALE;
            sample->gyro[1] = (float32)Icm42688_be16(raw, 10u) * ICM42688_GYRO_SCALE;
            sample->gyro[2] = (float32)Icm42688_be16(raw, 12u) * ICM42688_GYRO_SCALE;
        }
    }
    return ok;
}

boolean Icm42688_debugDump(uint8 cfg[ICM42688_DUMP_CFG_LEN], uint8 raw[14])
{
    /* Block scope (MISRA 8.9): the order here is the order documented for
     * cfg[] in Icm42688.h. */
    static const uint8 s_icm42688DumpRegs[ICM42688_DUMP_CFG_LEN] =
    {
        ICM42688_REG_WHO_AM_I,
        ICM42688_REG_PWR_MGMT0,
        ICM42688_REG_GYRO_CONFIG0,
        ICM42688_REG_ACCEL_CONFIG0,
        ICM42688_REG_INT_STATUS
    };

    boolean ok = TRUE;
    uint8   i;

    for (i = 0u; (i < ICM42688_DUMP_CFG_LEN) && (ok != FALSE); i++)
    {
        ok = Icm42688_readRegs(s_icm42688DumpRegs[i], &cfg[i], 1u);
    }
    if (ok != FALSE)
    {
        ok = Icm42688_readRegs(ICM42688_REG_TEMP_DATA1, raw, ICM42688_BURST_LEN);
    }
    return ok;
}
