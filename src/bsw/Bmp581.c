/**********************************************************************************************************************
 * \file Bmp581.c
 * \brief Bosch BMP581 barometric pressure + temperature driver (I2C0) — see Bmp581.h.
 *
 * Register map, config values and output scaling follow the BMP581 datasheet.
 * TODO(hw): confirm every constant marked "DS" against the BMP581 datasheet on
 * first bring-up (sensors arrive 2026-07-29) — the chip-ID check gates the rest,
 * so a wrong data-register address surfaces immediately as an implausible value.
 *********************************************************************************************************************/
#include "Bmp581.h"
#include "I2c.h"

/* --- Register map (BMP581 DS, register memory map) --- */
#define BMP581_REG_CHIP_ID      (0x01u)
#define BMP581_REG_TEMP_XLSB    (0x1Du)   /* T = 0x1D..0x1F, P = 0x20..0x22 (6-byte burst) */
#define BMP581_REG_OSR_CONFIG   (0x36u)
#define BMP581_REG_ODR_CONFIG   (0x37u)
#define BMP581_REG_CMD          (0x7Eu)

#define BMP581_CMD_SOFT_RESET   (0xB6u)

/* OSR_CONFIG (DS): bit6 press_en = 1, pressure/temperature oversampling 1x for
 * bring-up. Raise osr_p (bits 5:3) later for lower pressure noise. */
#define BMP581_OSR_CONFIG_VAL   (0x40u)

/* ODR_CONFIG (DS): bits 1:0 pwr_mode = 1 (normal / continuous). The odr field
 * (bits 6:2) is left at its reset default; set it per the BMP581 ODR table to
 * trim the sample rate — the data path works regardless of the exact ODR. */
#define BMP581_ODR_CONFIG_VAL   (0x01u)

/* Output scaling (DS): temperature = raw24(signed) / 2^16 degC,
 *                      pressure    = raw24(unsigned) / 2^6 Pa. */
#define BMP581_TEMP_DIV         (65536.0f)
#define BMP581_PRESS_DIV        (64.0f)

#define BMP581_SIGN_BIT_24      (0x00800000u)
#define BMP581_SIGN_EXT_24      (0xFF000000u)

/* Bounded post-reset settle (~2 ms, DS) before the NVM trim is reloaded. The
 * bus helpers are themselves slow, so a coarse busy-wait is adequate here and
 * runs once, before the scheduler starts. volatile prevents elision. */
#define BMP581_RESET_SPIN       (2000000u)

static boolean s_present = FALSE;

boolean Bmp581_readChipId(uint8 *chipId)
{
    return I2c_readReg(BMP581_I2C_ADDR, BMP581_REG_CHIP_ID, chipId, 1u);
}

boolean Bmp581_init(void)
{
    uint8            chipId = 0u;
    volatile uint32  spin;
    boolean          ok;

    s_present = FALSE;

    /* Soft reset (ignore ACK: a fresh device may hold the bus briefly). */
    (void)I2c_writeByte(BMP581_I2C_ADDR, BMP581_REG_CMD, BMP581_CMD_SOFT_RESET);
    for (spin = 0u; spin < BMP581_RESET_SPIN; spin++)
    {
        /* settle */
    }

    ok = Bmp581_readChipId(&chipId);
    if ((ok == FALSE) || (chipId != BMP581_CHIP_ID_VALUE))
    {
        return FALSE;
    }

    ok = I2c_writeByte(BMP581_I2C_ADDR, BMP581_REG_OSR_CONFIG, BMP581_OSR_CONFIG_VAL);
    if (ok != FALSE)
    {
        ok = I2c_writeByte(BMP581_I2C_ADDR, BMP581_REG_ODR_CONFIG, BMP581_ODR_CONFIG_VAL);
    }

    s_present = ok;
    return ok;
}

boolean Bmp581_isPresent(void)
{
    return s_present;
}

boolean Bmp581_read(float32 *pressurePa, float32 *temperatureC)
{
    uint8   raw[6];
    boolean ok;

    ok = I2c_readReg(BMP581_I2C_ADDR, BMP581_REG_TEMP_XLSB, raw, 6u);
    if (ok != FALSE)
    {
        /* 24-bit little-endian: [xlsb, lsb, msb] */
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
    return ok;
}
