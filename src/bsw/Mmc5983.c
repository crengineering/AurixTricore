/**********************************************************************************************************************
 * \file Mmc5983.c
 * \brief MEMSIC MMC5983MA 3-axis magnetometer driver (I2C0) — see Mmc5983.h.
 *
 * ⚠️ Every register constant here is written from the MMC5983MA register map
 * WITHOUT the device datasheet (the PDF available is the prototyping board's
 * user guide, which has no register information — docs/MMC5983MA.md section 9).
 * Mmc5983_debugDump() is the instrument for confirming them against silicon;
 * PRODUCT_ID is the gate that makes a wrong guess visible immediately.
 *
 * Two behaviours that are easy to get wrong and expensive to debug:
 *   1. The internal control registers (0x09..0x0C) do NOT read back what was
 *      written. Measured 2026-08-01: CTRL0/1/2 all returned 0x61 — identical to
 *      one another, matching neither the written values (0x20, 0x00, 0x0C) nor
 *      zero. Never verify configuration by reading them; only the data block
 *      tells you a write took effect.
 *   2. An AMR bridge like this one drifts its own magnetisation, so a raw
 *      reading carries an offset that wanders with temperature and with any
 *      strong field it has seen. Automatic set/reset (CTRL0 bit 5) makes the
 *      device perform the set/reset pair internally around every measurement
 *      and output the compensated value. Without it the output looks perfectly
 *      alive and is quietly wrong — the worst failure mode there is.
 *********************************************************************************************************************/
#include "Mmc5983.h"
#include "I2c.h"
#include "IfxStm.h"
#include <math.h>

/* --- Register map --- */
#define MMC5983_REG_XOUT0        (0x00u)   /* 0x00..0x06: X, Y, Z, then XYZout2 */
#define MMC5983_REG_STATUS       (0x08u)
#define MMC5983_REG_CTRL0        (0x09u)
#define MMC5983_REG_CTRL1        (0x0Au)
#define MMC5983_REG_CTRL2        (0x0Bu)
#define MMC5983_REG_PRODUCT_ID   (0x2Fu)

#define MMC5983_DATA_LEN         (7u)      /* X(2) Y(2) Z(2) + XYZout2(1) */

/* STATUS (0x08) bit 0 is Meas_M_Done. Not polled before each read: in
 * continuous mode the data registers always hold the most recent completed
 * sample, so a status read would cost an extra transaction at 50 Hz to learn
 * nothing. It is in the boot dump instead (see Mmc5983.h). */

/* CTRL0 (0x09) bit 5: Auto_SR_en — automatic set/reset around each
 * measurement. See the file header: this is what makes the output an absolute
 * field reading instead of a drifting one. */
#define MMC5983_CTRL0_AUTO_SR_EN (0x20u)

/* CTRL1 (0x0A) bit 7: SW_RST. Bits 1:0 select the measurement bandwidth;
 * 0b00 = 100 Hz (8 ms per measurement), which is the quietest setting and is
 * comfortably faster than the 50 Hz output rate configured below. */
#define MMC5983_CTRL1_SW_RST     (0x80u)
#define MMC5983_CTRL1_BW_100HZ   (0x00u)

/* CTRL2 (0x0B): bits 2:0 CM_freq, bit 3 Cmm_en (continuous mode).
 * CM_freq = 0b100 -> 50 Hz, matching the 20 ms magnetometer task so each poll
 * gets a fresh sample. Cmm_en only takes effect when CM_freq is non-zero;
 * setting the enable bit alone leaves the device idle. */
#define MMC5983_CTRL2_CM_FREQ_50HZ (0x04u)
#define MMC5983_CTRL2_CMM_EN       (0x08u)
#define MMC5983_CTRL2_VAL          (MMC5983_CTRL2_CMM_EN | MMC5983_CTRL2_CM_FREQ_50HZ)

/* Output scaling. The device returns unsigned 18-bit counts per axis with
 * zero field at mid-scale (2^17), and 16384 counts per gauss. */
#define MMC5983_ZERO_COUNTS      (131072.0f)   /* 2^17 */
#define MMC5983_COUNTS_PER_GAUSS (16384.0f)

/* Post-reset settle before the device answers again. */
#define MMC5983_STM_TICKS_PER_MS (100000u)     /* STM0 at ~100 MHz */
#define MMC5983_RESET_MS         (20u)

/* Hot-plug recovery: retry this often (calls, i.e. 50 per second from the
 * 20 ms magnetometer task) while the device is missing. */
#define MMC5983_RECOVERY_PERIOD  (50u)         /* ~1 s */

#define MMC5983_RAD_TO_DEG       (57.295779513f)
#define MMC5983_DEG_FULL_TURN    (360.0f)

static boolean s_mmc5983Present = FALSE;

static void Mmc5983_delayMs(uint32 ms)
{
    IfxStm_waitTicks(&MODULE_STM0, ms * MMC5983_STM_TICKS_PER_MS);
}

boolean Mmc5983_readProductId(uint8 *productId)
{
    return I2c_readReg(MMC5983_I2C_ADDR, MMC5983_REG_PRODUCT_ID, productId, 1u);
}

boolean Mmc5983_init(void)
{
    uint8   productId = 0u;
    boolean ok;

    s_mmc5983Present = FALSE;

    /* Software reset (ignore ACK: a fresh device may hold the bus briefly). */
    (void)I2c_writeByte(MMC5983_I2C_ADDR, MMC5983_REG_CTRL1, MMC5983_CTRL1_SW_RST);
    Mmc5983_delayMs(MMC5983_RESET_MS);

    ok = Mmc5983_readProductId(&productId);
    if ((ok == FALSE) || (productId != MMC5983_PRODUCT_ID))
    {
        return FALSE;
    }

    /* Automatic set/reset first: it must be armed before the device starts
     * producing samples, otherwise the first samples carry the uncorrected
     * bridge offset. */
    ok = I2c_writeByte(MMC5983_I2C_ADDR, MMC5983_REG_CTRL0, MMC5983_CTRL0_AUTO_SR_EN);
    if (ok != FALSE)
    {
        ok = I2c_writeByte(MMC5983_I2C_ADDR, MMC5983_REG_CTRL1, MMC5983_CTRL1_BW_100HZ);
    }
    if (ok != FALSE)
    {
        /* Written last — this is the write that starts continuous conversion. */
        ok = I2c_writeByte(MMC5983_I2C_ADDR, MMC5983_REG_CTRL2, MMC5983_CTRL2_VAL);
    }

    s_mmc5983Present = ok;
    return ok;
}

boolean Mmc5983_isPresent(void)
{
    return s_mmc5983Present;
}

/* Reassemble one 18-bit axis: [17:10] in msb, [9:2] in lsb, [1:0] in the
 * shared XYZout2 byte at the given bit offset. */
static uint32 Mmc5983_axis18(uint8 msb, uint8 lsb, uint8 xyz2, uint8 shift)
{
    return ((uint32)msb << 10) | ((uint32)lsb << 2)
           | (((uint32)xyz2 >> shift) & 0x03u);
}

boolean Mmc5983_read(Mmc5983_Sample *sample)
{
    /* Block scope (MISRA 8.9): only this function drives the retry. */
    static uint16 s_mmc5983Recovery = 0u;

    uint8   raw[MMC5983_DATA_LEN];
    boolean ok = FALSE;

    if (s_mmc5983Present == FALSE)
    {
        /* Device absent: probe periodically so it comes back on its own when
         * the wiring is restored. The probe is a single PRODUCT_ID read and
         * the full bring-up runs only once that answers — unplugging the
         * sensor power-cycles it back to its reset state with continuous mode
         * off, so simply resuming reads would return nothing. */
        s_mmc5983Recovery++;
        if (s_mmc5983Recovery >= MMC5983_RECOVERY_PERIOD)
        {
            uint8 productId = 0u;

            s_mmc5983Recovery = 0u;
            if ((Mmc5983_readProductId(&productId) != FALSE)
                && (productId == MMC5983_PRODUCT_ID))
            {
                (void)Mmc5983_init();
            }
        }
    }
    else
    {
        ok = I2c_readReg(MMC5983_I2C_ADDR, MMC5983_REG_XOUT0, raw, MMC5983_DATA_LEN);
        if (ok == FALSE)
        {
            /* Lost it. Drop presence so the branch above starts probing
             * instead of retrying a dead device at 50 Hz forever. */
            s_mmc5983Present  = FALSE;
            s_mmc5983Recovery = 0u;
        }
        else
        {
            uint32 xRaw = Mmc5983_axis18(raw[0], raw[1], raw[6], 6u);
            uint32 yRaw = Mmc5983_axis18(raw[2], raw[3], raw[6], 4u);
            uint32 zRaw = Mmc5983_axis18(raw[4], raw[5], raw[6], 2u);
            float32 heading;

            sample->mag[0] = ((float32)xRaw - MMC5983_ZERO_COUNTS) / MMC5983_COUNTS_PER_GAUSS;
            sample->mag[1] = ((float32)yRaw - MMC5983_ZERO_COUNTS) / MMC5983_COUNTS_PER_GAUSS;
            sample->mag[2] = ((float32)zRaw - MMC5983_ZERO_COUNTS) / MMC5983_COUNTS_PER_GAUSS;

            /* Level-only, uncalibrated bring-up heading — see Mmc5983.h. */
            heading = atan2f(sample->mag[1], sample->mag[0]) * MMC5983_RAD_TO_DEG;
            if (heading < 0.0f)
            {
                heading += MMC5983_DEG_FULL_TURN;
            }
            sample->headingDeg = heading;
        }
    }
    return ok;
}

boolean Mmc5983_debugDump(uint8 cfg[MMC5983_DUMP_CFG_LEN], uint8 raw[7])
{
    /* Block scope (MISRA 8.9): the order here is the order documented for
     * cfg[] in Mmc5983.h. */
    static const uint8 s_mmc5983DumpRegs[MMC5983_DUMP_CFG_LEN] =
    {
        MMC5983_REG_PRODUCT_ID,
        MMC5983_REG_STATUS,
        MMC5983_REG_CTRL0,
        MMC5983_REG_CTRL1,
        MMC5983_REG_CTRL2
    };

    boolean ok = TRUE;
    uint8   i;

    for (i = 0u; (i < MMC5983_DUMP_CFG_LEN) && (ok != FALSE); i++)
    {
        ok = I2c_readReg(MMC5983_I2C_ADDR, s_mmc5983DumpRegs[i], &cfg[i], 1u);
    }
    if (ok != FALSE)
    {
        ok = I2c_readReg(MMC5983_I2C_ADDR, MMC5983_REG_XOUT0, raw, MMC5983_DATA_LEN);
    }
    return ok;
}
