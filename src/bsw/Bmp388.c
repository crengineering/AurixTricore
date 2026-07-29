/**********************************************************************************************************************
 * \file Bmp388.c
 * \brief Bosch BMP388 barometric pressure + temperature driver (I2C0) — see Bmp388.h.
 *
 * Register map, calibration layout and compensation follow the BMP388 datasheet
 * BST-BMP388-DS001 (Rev 1.7): register map Table 24, trim coefficients Table 23,
 * compensation reference code §9.
 *
 * TODO(hw): on first bring-up confirm the CJMCU-388 wiring (see Bmp388.h pad
 * map): I2C address (0x77 vs 0x76, set by the SDO pad), CS tied to 3V3 (I2C
 * mode), and VIN on +3V3 (not 5V). The chip-ID check gates the rest, so a wrong
 * address surfaces immediately as Bmp388_init() returning FALSE.
 *********************************************************************************************************************/
#include "Bmp388.h"
#include "I2c.h"

/* --- Register map (DS Table 24) --- */
#define BMP388_REG_CHIP_ID      (0x00u)
#define BMP388_REG_DATA_0       (0x04u)   /* press[0..2] @ 0x04..0x06, temp[0..2] @ 0x07..0x09 */
#define BMP388_REG_PWR_CTRL     (0x1Bu)
#define BMP388_REG_OSR          (0x1Cu)
#define BMP388_REG_ODR          (0x1Du)
#define BMP388_REG_CONFIG       (0x1Fu)
#define BMP388_REG_CALIB_0      (0x31u)   /* NVM_PAR trim block, 21 bytes @ 0x31..0x45 */
#define BMP388_REG_CMD          (0x7Eu)

#define BMP388_CMD_SOFT_RESET   (0xB6u)

#define BMP388_DATA_LEN         (6u)      /* pressure (3) + temperature (3) */
#define BMP388_CALIB_LEN        (21u)     /* 0x31..0x45 inclusive */

/* PWR_CTRL (DS 4.3.15): press_en (bit0) | temp_en (bit1) | mode = normal (bits 5:4 = 0b11). */
#define BMP388_PWR_CTRL_VAL     (0x33u)

/* OSR (DS 4.3.16): osr_p (bits 2:0) = x16 (0b100), osr_t (bits 5:3) = x2 (0b001).
 * Bosch "indoor navigation" low-noise preset — high pressure oversampling to
 * push the ADC noise floor well below a 0.5 m step. */
#define BMP388_OSR_VAL          (0x0Cu)

/* ODR (DS 4.3.17): odr_sel = 0x03 -> 25 Hz. Must be slow enough for the x16
 * conversion time (a too-fast ODR sets conf_err in ERR_REG). */
#define BMP388_ODR_VAL          (0x03u)

/* CONFIG (DS 4.3.18): iir_filter (bits 3:1) = coeff 15 (0b100) -> value 0x08.
 * Strong on-chip low-pass; trades a little step-response lag for a much
 * quieter at-rest signal (matches the indoor-navigation preset). */
#define BMP388_CONFIG_VAL       (0x08u)

/* Bounded post-reset settle (~2 ms start-up, DS Table 3). The bus helpers are
 * themselves slow, so a coarse busy-wait is adequate here and runs once, before
 * the scheduler starts. volatile prevents elision. */
#define BMP388_RESET_SPIN       (2000000u)

/* --- Calibration coefficient scaling (DS §9.1). Each raw NVM value is converted
 * to floating point by dividing by 2^n; here we store the reciprocal as a
 * multiplier so compensation is a plain multiply. --- */
#define BMP388_SCALE_T1         (256.0f)                     /* / 2^-8  */
#define BMP388_SCALE_T2         (1073741824.0f)              /* / 2^30  */
#define BMP388_SCALE_T3         (281474976710656.0f)         /* / 2^48  */
#define BMP388_SCALE_P1_OFFSET  (16384.0f)                   /* 2^14    */
#define BMP388_SCALE_P1         (1048576.0f)                 /* / 2^20  */
#define BMP388_SCALE_P2         (536870912.0f)               /* / 2^29  */
#define BMP388_SCALE_P3         (4294967296.0f)              /* / 2^32  */
#define BMP388_SCALE_P4         (137438953472.0f)            /* / 2^37  */
#define BMP388_SCALE_P5         (0.125f)                     /* / 2^-3  */
#define BMP388_SCALE_P6         (64.0f)                      /* / 2^6   */
#define BMP388_SCALE_P7         (256.0f)                     /* / 2^8   */
#define BMP388_SCALE_P8         (32768.0f)                   /* / 2^15  */
#define BMP388_SCALE_P9         (281474976710656.0f)         /* / 2^48  */
#define BMP388_SCALE_P10        (281474976710656.0f)         /* / 2^48  */
#define BMP388_SCALE_P11        (36893488147419103232.0f)   /* / 2^65  */

/* Calibration coefficients in floating point (DS §9.1), plus t_lin carried from
 * the temperature compensation into the pressure compensation (DS §9.2/9.3). */
typedef struct
{
    float32 par_t1;
    float32 par_t2;
    float32 par_t3;
    float32 par_p1;
    float32 par_p2;
    float32 par_p3;
    float32 par_p4;
    float32 par_p5;
    float32 par_p6;
    float32 par_p7;
    float32 par_p8;
    float32 par_p9;
    float32 par_p10;
    float32 par_p11;
    float32 t_lin;
} Bmp388_Calib;

static boolean      s_present = FALSE;
static Bmp388_Calib s_calib;

boolean Bmp388_readChipId(uint8 *chipId)
{
    return I2c_readReg(BMP388_I2C_ADDR, BMP388_REG_CHIP_ID, chipId, 1u);
}

/* Assemble a 16-bit little-endian field from the calibration burst. */
static uint16 Bmp388_u16(const uint8 *p, uint8 lsbIndex)
{
    return (uint16)(((uint16)p[lsbIndex + 1u] << 8) | (uint16)p[lsbIndex]);
}

/* Parse the 21-byte NVM trim block (DS Table 23) into the float coefficients
 * (DS §9.1). Signedness per Table 23: T1/T2/P5/P6 unsigned, the rest signed. */
static void Bmp388_parseCalib(const uint8 *raw)
{
    s_calib.par_t1  = (float32)Bmp388_u16(raw, 0u)          * BMP388_SCALE_T1;
    s_calib.par_t2  = (float32)Bmp388_u16(raw, 2u)          / BMP388_SCALE_T2;
    s_calib.par_t3  = (float32)(sint8)raw[4]                / BMP388_SCALE_T3;
    s_calib.par_p1  = ((float32)(sint16)Bmp388_u16(raw, 5u) - BMP388_SCALE_P1_OFFSET) / BMP388_SCALE_P1;
    s_calib.par_p2  = ((float32)(sint16)Bmp388_u16(raw, 7u) - BMP388_SCALE_P1_OFFSET) / BMP388_SCALE_P2;
    s_calib.par_p3  = (float32)(sint8)raw[9]                / BMP388_SCALE_P3;
    s_calib.par_p4  = (float32)(sint8)raw[10]               / BMP388_SCALE_P4;
    s_calib.par_p5  = (float32)Bmp388_u16(raw, 11u)         / BMP388_SCALE_P5;
    s_calib.par_p6  = (float32)Bmp388_u16(raw, 13u)         / BMP388_SCALE_P6;
    s_calib.par_p7  = (float32)(sint8)raw[15]               / BMP388_SCALE_P7;
    s_calib.par_p8  = (float32)(sint8)raw[16]               / BMP388_SCALE_P8;
    s_calib.par_p9  = (float32)(sint16)Bmp388_u16(raw, 17u) / BMP388_SCALE_P9;
    s_calib.par_p10 = (float32)(sint8)raw[19]               / BMP388_SCALE_P10;
    s_calib.par_p11 = (float32)(sint8)raw[20]               / BMP388_SCALE_P11;
    s_calib.t_lin   = 0.0f;
}

/* Temperature compensation (DS §9.2). Stores t_lin for the pressure step. */
static float32 Bmp388_compensateTemp(uint32 uncompTemp)
{
    float32 d1 = (float32)uncompTemp - s_calib.par_t1;
    float32 d2 = d1 * s_calib.par_t2;

    s_calib.t_lin = d2 + (d1 * d1) * s_calib.par_t3;
    return s_calib.t_lin;
}

/* Pressure compensation (DS §9.3). Requires t_lin from Bmp388_compensateTemp. */
static float32 Bmp388_compensatePress(uint32 uncompPress)
{
    float32 up   = (float32)uncompPress;
    float32 tl   = s_calib.t_lin;
    float32 out1;
    float32 out2;
    float32 d1;
    float32 d2;
    float32 d3;
    float32 d4;

    d1   = s_calib.par_p6 * tl;
    d2   = s_calib.par_p7 * (tl * tl);
    d3   = s_calib.par_p8 * (tl * tl * tl);
    out1 = s_calib.par_p5 + d1 + d2 + d3;

    d1   = s_calib.par_p2 * tl;
    d2   = s_calib.par_p3 * (tl * tl);
    d3   = s_calib.par_p4 * (tl * tl * tl);
    out2 = up * (s_calib.par_p1 + d1 + d2 + d3);

    d1   = up * up;
    d2   = s_calib.par_p9 + (s_calib.par_p10 * tl);
    d3   = d1 * d2;
    d4   = d3 + ((up * up * up) * s_calib.par_p11);

    return out1 + out2 + d4;
}

boolean Bmp388_init(void)
{
    uint8            chipId = 0u;
    uint8            calibRaw[BMP388_CALIB_LEN];
    volatile uint32  spin;
    boolean          ok;

    s_present = FALSE;

    /* Soft reset (ignore ACK: a fresh device may hold the bus briefly). */
    (void)I2c_writeByte(BMP388_I2C_ADDR, BMP388_REG_CMD, BMP388_CMD_SOFT_RESET);
    for (spin = 0u; spin < BMP388_RESET_SPIN; spin++)
    {
        /* settle */
    }

    ok = Bmp388_readChipId(&chipId);
    if ((ok != FALSE) && (chipId == BMP388_CHIP_ID_VALUE))
    {
        /* Read factory calibration trim (DS Table 23) before configuring. */
        ok = I2c_readReg(BMP388_I2C_ADDR, BMP388_REG_CALIB_0, calibRaw, BMP388_CALIB_LEN);
        if (ok != FALSE)
        {
            Bmp388_parseCalib(calibRaw);
        }

        /* Configure oversampling, ODR and IIR before enabling normal mode. */
        if (ok != FALSE)
        {
            ok = I2c_writeByte(BMP388_I2C_ADDR, BMP388_REG_OSR, BMP388_OSR_VAL);
        }
        if (ok != FALSE)
        {
            ok = I2c_writeByte(BMP388_I2C_ADDR, BMP388_REG_ODR, BMP388_ODR_VAL);
        }
        if (ok != FALSE)
        {
            ok = I2c_writeByte(BMP388_I2C_ADDR, BMP388_REG_CONFIG, BMP388_CONFIG_VAL);
        }
        if (ok != FALSE)
        {
            ok = I2c_writeByte(BMP388_I2C_ADDR, BMP388_REG_PWR_CTRL, BMP388_PWR_CTRL_VAL);
        }
    }
    else
    {
        ok = FALSE;      /* chip absent or wrong ID */
    }

    s_present = ok;
    return ok;
}

boolean Bmp388_isPresent(void)
{
    return s_present;
}

boolean Bmp388_read(float32 *pressurePa, float32 *temperatureC)
{
    uint8   raw[BMP388_DATA_LEN];
    boolean ok = FALSE;

    if (s_present != FALSE)
    {
        ok = I2c_readReg(BMP388_I2C_ADDR, BMP388_REG_DATA_0, raw, BMP388_DATA_LEN);
        if (ok != FALSE)
        {
            /* 24-bit little-endian: pressure = [0..2], temperature = [3..5]. */
            uint32 pRaw = ((uint32)raw[2] << 16) | ((uint32)raw[1] << 8) | (uint32)raw[0];
            uint32 tRaw = ((uint32)raw[5] << 16) | ((uint32)raw[4] << 8) | (uint32)raw[3];

            /* Temperature first — it produces t_lin, which pressure depends on. */
            *temperatureC = Bmp388_compensateTemp(tRaw);
            *pressurePa   = Bmp388_compensatePress(pRaw);
        }
    }
    return ok;
}
