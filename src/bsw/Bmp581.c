/**********************************************************************************************************************
 * \file Bmp581.c
 * \brief Bosch BMP581 barometric pressure + temperature driver (I2C0) — see Bmp581.h.
 *
 * Register map, field layouts and output scaling follow the BMP581 datasheet as
 * implemented by Bosch's BMP5 sensor API. The datasheet is not in the repo, so
 * every constant here is stated with its field decomposition rather than as a
 * bare magic number, and Bmp581_debugDump() reads the configuration back so the
 * whole set can be confirmed against silicon in a single boot.
 *
 * Two ordering constraints are not obvious from the register map and are the
 * usual reason a BMP581 reads zeros or an unfiltered value:
 *   1. The DSP (IIR) registers only accept writes while the device is in
 *      standby. Soft reset leaves it in standby, so all configuration happens
 *      before ODR_CONFIG switches it to normal mode — never after.
 *   2. The IIR output only reaches the data registers when the shadow-select
 *      bits in DSP_CONFIG are set. Without them the filter runs but the burst
 *      read still returns the raw, unfiltered sample.
 *********************************************************************************************************************/
#include "Bmp581.h"
#include "I2c.h"

/* --- Register map (BMP5 register memory map) --- */
#define BMP581_REG_CHIP_ID      (0x01u)
#define BMP581_REG_REV_ID       (0x02u)
#define BMP581_REG_TEMP_XLSB    (0x1Du)   /* T = 0x1D..0x1F, P = 0x20..0x22 (6-byte burst) */
#define BMP581_REG_INT_STATUS   (0x27u)
#define BMP581_REG_STATUS       (0x28u)
#define BMP581_REG_DSP_CONFIG   (0x30u)
#define BMP581_REG_DSP_IIR      (0x31u)
#define BMP581_REG_OSR_CONFIG   (0x36u)
#define BMP581_REG_ODR_CONFIG   (0x37u)
#define BMP581_REG_OSR_EFF      (0x38u)
#define BMP581_REG_CMD          (0x7Eu)

#define BMP581_CMD_SOFT_RESET   (0xB6u)

#define BMP581_DATA_LEN         (6u)      /* temperature (3) + pressure (3) */

/* STATUS (0x28): bit 1 nvm_rdy (trim loaded), bit 2 nvm_err. Both must be
 * right before the outputs mean anything — a part that answers with nvm_err
 * set is reporting numbers derived from calibration it failed to load. */
#define BMP581_STATUS_NVM_RDY   (0x02u)
#define BMP581_STATUS_NVM_ERR   (0x04u)

/* DSP_CONFIG (0x30): bit 1 shdw_sel_iir_t, bit 3 shdw_sel_iir_p — route the
 * IIR-filtered temperature and pressure into the data registers. Without these
 * the burst read below returns the unfiltered sample and the filter set in
 * DSP_IIR has no visible effect at all. */
#define BMP581_DSP_CONFIG_VAL   (0x0Au)

/* DSP_IIR (0x31): set_iir_t (bits 2:0) = coeff 3 (0b010),
 *                 set_iir_p (bits 5:3) = coeff 15 (0b100) -> 0x20.
 * Same intent as the BMP388's indoor-navigation preset: a strong low-pass on
 * pressure to keep the at-rest altitude quiet, trading a little step-response
 * lag, and a light one on the inherently slow temperature channel. */
#define BMP581_DSP_IIR_VAL      (0x22u)

/* OSR_CONFIG (0x36): osr_t (bits 2:0) = x2 (0b001),
 *                    osr_p (bits 5:3) = x16 (0b100) -> 0x20,
 *                    press_en (bit 6) = 1 -> 0x40.
 * press_en is the one that matters most: with it clear the pressure registers
 * stay at their reset value while temperature updates normally, which looks
 * like a broken pressure element rather than a configuration mistake. */
#define BMP581_OSR_CONFIG_VAL   (0x61u)

/* ODR_CONFIG (0x37): pwr_mode (bits 1:0) = 1 (normal / continuous),
 *                    odr (bits 6:2) = 0x0F = 50 Hz -> 0x3C,
 *                    deep_dis (bit 7) = 1 -> 0x80.
 * 50 Hz matches the 20 ms baro task, so each poll gets a fresh conversion
 * without the task racing the sensor. deep_dis stops the part from dropping
 * into deep standby, where it stops converting and answers stale data. */
#define BMP581_ODR_CONFIG_VAL   (0xBDu)

/* Output scaling: temperature = raw24(signed) / 2^16 degC,
 *                 pressure    = raw24(unsigned) / 2^6 Pa.
 * The BMP581 applies its own calibration on-chip, which is why there is no
 * trim block and no compensation polynomial here (contrast Bmp388.c). */
#define BMP581_TEMP_DIV         (65536.0f)
#define BMP581_PRESS_DIV        (64.0f)

#define BMP581_SIGN_BIT_24      (0x00800000u)
#define BMP581_SIGN_EXT_24      (0xFF000000u)

/* Bounded post-reset settle (~2 ms start-up) before the NVM trim is reloaded.
 * The bus helpers are themselves slow, so a coarse busy-wait is adequate here
 * and runs once, before the scheduler starts. volatile prevents elision. */
#define BMP581_RESET_SPIN       (2000000u)

/* Hot-plug recovery: retry this often (calls, i.e. 50 per second from the
 * 20 ms baro task) while the device is missing. */
#define BMP581_RECOVERY_PERIOD  (50u)      /* ~1 s */

/* Module-scoped statics carry the module name: several sensor drivers coexist
 * in src/bsw and MISRA 5.9 requires internal-linkage identifiers to be unique
 * across the whole project, not just per file. */
static boolean s_bmp581Present = FALSE;

/* TRUE if this ID belongs to a BMP581 (either family variant). */
static boolean bmp581_idIsValid(uint8 chipId)
{
    boolean valid = FALSE;

    if ((chipId == BMP581_CHIP_ID_PRIMARY) || (chipId == BMP581_CHIP_ID_SECONDARY))
    {
        valid = TRUE;
    }
    return valid;
}

boolean Bmp581_readChipId(uint8 *chipId)
{
    return I2c_readReg(BMP581_I2C_ADDR, BMP581_REG_CHIP_ID, chipId, 1u);
}

boolean Bmp581_init(void)
{
    uint8            chipId = 0u;
    uint8            status = 0u;
    volatile uint32  spin;
    boolean          ok;

    s_bmp581Present = FALSE;

    /* Soft reset (ignore ACK: a fresh device may hold the bus briefly). */
    (void)I2c_writeByte(BMP581_I2C_ADDR, BMP581_REG_CMD, BMP581_CMD_SOFT_RESET);
    for (spin = 0u; spin < BMP581_RESET_SPIN; spin++)
    {
        /* settle */
    }

    /* INT_STATUS is deliberately NOT read here: its por_softreset_complete bit
     * (bit 4) is clear-on-read, and reading it now would consume the one piece
     * of positive evidence that the reset above actually took effect rather
     * than merely being ACKed. Bmp581_debugDump() reads it instead, so the
     * boot dump still shows it. */

    ok = Bmp581_readChipId(&chipId);
    if ((ok == FALSE) || (bmp581_idIsValid(chipId) == FALSE))
    {
        return FALSE;
    }

    /* Trim must be loaded and error-free before any output is trustworthy. */
    ok = I2c_readReg(BMP581_I2C_ADDR, BMP581_REG_STATUS, &status, 1u);
    if (ok != FALSE)
    {
        if (((status & BMP581_STATUS_NVM_RDY) == 0u)
            || ((status & BMP581_STATUS_NVM_ERR) != 0u))
        {
            ok = FALSE;
        }
    }

    /* Configuration order is deliberate: the DSP registers are writable only in
     * standby, which is where the soft reset left the device, and ODR_CONFIG is
     * written last because it is the write that starts conversions. */
    if (ok != FALSE)
    {
        ok = I2c_writeByte(BMP581_I2C_ADDR, BMP581_REG_DSP_CONFIG, BMP581_DSP_CONFIG_VAL);
    }
    if (ok != FALSE)
    {
        ok = I2c_writeByte(BMP581_I2C_ADDR, BMP581_REG_DSP_IIR, BMP581_DSP_IIR_VAL);
    }
    if (ok != FALSE)
    {
        ok = I2c_writeByte(BMP581_I2C_ADDR, BMP581_REG_OSR_CONFIG, BMP581_OSR_CONFIG_VAL);
    }
    if (ok != FALSE)
    {
        ok = I2c_writeByte(BMP581_I2C_ADDR, BMP581_REG_ODR_CONFIG, BMP581_ODR_CONFIG_VAL);
    }

    s_bmp581Present = ok;
    return ok;
}

boolean Bmp581_isPresent(void)
{
    return s_bmp581Present;
}

boolean Bmp581_read(float32 *pressurePa, float32 *temperatureC)
{
    /* Block scope (MISRA 8.9): only this function drives the retry. */
    static uint16 s_bmp581Recovery = 0u;

    uint8   raw[BMP581_DATA_LEN];
    boolean ok = FALSE;

    if (s_bmp581Present == FALSE)
    {
        /* Device absent: probe periodically so it comes back on its own when
         * the wiring is restored.
         *
         * The probe is a single CHIP_ID read (~1 ms) and the full bring-up runs
         * ONLY once that answers, so a disconnected sensor costs one short
         * transaction a second instead of a reset-plus-reconfigure sequence at
         * 50 Hz. Re-init is required rather than optional: unplugging the
         * sensor power-cycles it back to standby with its configuration lost,
         * so simply resuming reads would return nothing. */
        s_bmp581Recovery++;
        if (s_bmp581Recovery >= BMP581_RECOVERY_PERIOD)
        {
            uint8 chipId = 0u;

            s_bmp581Recovery = 0u;
            if ((Bmp581_readChipId(&chipId) != FALSE) && (bmp581_idIsValid(chipId) != FALSE))
            {
                (void)Bmp581_init();
            }
        }
    }
    else
    {
        ok = I2c_readReg(BMP581_I2C_ADDR, BMP581_REG_TEMP_XLSB, raw, BMP581_DATA_LEN);
        if (ok == FALSE)
        {
            /* Lost it. Drop presence so the branch above starts probing
             * instead of retrying a dead device at 50 Hz forever. */
            s_bmp581Present  = FALSE;
            s_bmp581Recovery = 0u;
        }
        else
        {
            /* 24-bit little-endian: temperature = [0..2], pressure = [3..5]. */
            uint32 tRaw = ((uint32)raw[2] << 16) | ((uint32)raw[1] << 8) | (uint32)raw[0];
            uint32 pRaw = ((uint32)raw[5] << 16) | ((uint32)raw[4] << 8) | (uint32)raw[3];
            sint32 tSigned = (sint32)tRaw;

            /* temperature is signed 24-bit two's complement — sign-extend to 32 */
            if ((tRaw & BMP581_SIGN_BIT_24) != 0u)
            {
                tSigned = (sint32)(tRaw | BMP581_SIGN_EXT_24);
            }

            *temperatureC = (float32)tSigned / BMP581_TEMP_DIV;
            *pressurePa   = (float32)pRaw    / BMP581_PRESS_DIV;
        }
    }
    return ok;
}

boolean Bmp581_debugDump(uint8 cfg[BMP581_DUMP_CFG_LEN], uint8 *osrEff, uint8 raw[6])
{
    /* Block scope (MISRA 8.9): only this function walks the dump list, and the
     * order here is the order documented for cfg[] in Bmp581.h. */
    static const uint8 s_bmp581DumpRegs[BMP581_DUMP_CFG_LEN] =
    {
        BMP581_REG_CHIP_ID,
        BMP581_REG_REV_ID,
        BMP581_REG_INT_STATUS,
        BMP581_REG_STATUS,
        BMP581_REG_DSP_IIR,
        BMP581_REG_OSR_CONFIG,
        BMP581_REG_ODR_CONFIG
    };

    boolean ok = TRUE;
    uint8   i;

    for (i = 0u; (i < BMP581_DUMP_CFG_LEN) && (ok != FALSE); i++)
    {
        ok = I2c_readReg(BMP581_I2C_ADDR, s_bmp581DumpRegs[i], &cfg[i], 1u);
    }
    if (ok != FALSE)
    {
        ok = I2c_readReg(BMP581_I2C_ADDR, BMP581_REG_OSR_EFF, osrEff, 1u);
    }
    if (ok != FALSE)
    {
        ok = I2c_readReg(BMP581_I2C_ADDR, BMP581_REG_TEMP_XLSB, raw, BMP581_DATA_LEN);
    }
    return ok;
}
