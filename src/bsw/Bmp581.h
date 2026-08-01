/**********************************************************************************************************************
 * \file Bmp581.h
 * \brief Bosch BMP581 barometric pressure + temperature sensor driver (I2C0).
 *
 * Sole device on the shared I2C0 bus (docs/PINNING.md 2.3/2.5) since the BMP388
 * and the MPU-6050 were removed on 2026-07-31. Uses the blocking I2c bus
 * service. Wiring, pull-ups and the bring-up order are in docs/BMP581.md.
 *
 * Board: Adafruit BMP581 STEMMA QT breakout. It brings its own 10 kOhm SDA/SCL
 * pull-ups (it is now the only source of them on this bus) and straps CS high
 * for I2C, so neither of the two classic BMP wiring traps applies here.
 *
 * The bring-up milestone is Bmp581_readChipId() returning 0x50 (or 0x51) — that
 * one transaction proves the whole I2C path (pins, TTL pads, pull-ups, address,
 * CS strap, driver). Bmp581_debugDump() then shows the configuration and the
 * raw measurement block in one go, which is what separates "wrong register
 * address" from "sensor not converting".
 *
 * Unlike the BMP388 the BMP581 outputs ready-scaled values: there is no NVM
 * trim block to read and no compensation polynomial, only a fixed divide.
 *
 * Register addresses, field layouts and the ODR/OSR encodings below follow the
 * BMP581 datasheet as implemented by Bosch's BMP5 sensor API. The datasheet is
 * not in the repo (docs/BMP581.md section 10); Bmp581_debugDump() exists so the
 * assumptions can be checked against real silicon in one boot.
 *********************************************************************************************************************/
#ifndef BMP581_H
#define BMP581_H

#include "Ifx_Types.h"

/* Adafruit BMP581 STEMMA QT (PID 5620): the silkscreen reads "Default I2C addr
 * 0x47" — the board straps SDO high. Cutting/bridging the "addr" jumper on the
 * back pulls SDO low and moves it to 0x46. The bare-die default in the
 * datasheet is the other way round, which is why this is not 0x46. */
#define BMP581_I2C_ADDR        (0x47u)  /* SDO = high (board default); 0x46 if the addr jumper is moved */

/* CHIP_ID (0x01). The BMP5 family ships two IDs — 0x50 on the primary variant
 * and 0x51 on the secondary — and both are BMP581 parts, so accept either.
 * Note 0x50 is *also* the BMP388's chip ID; the two are told apart by their
 * address (0x46/0x47 vs 0x76/0x77), not by this register. */
#define BMP581_CHIP_ID_PRIMARY   (0x50u)
#define BMP581_CHIP_ID_SECONDARY (0x51u)

/** Soft-reset, verify the chip ID and the NVM status, configure oversampling
 *  and the IIR filter, then start continuous (normal-mode) measurement.
 *  \return TRUE if the chip ID matched and every configuration write was ACKed.
 *          Safe to call with no sensor attached — it returns FALSE without
 *          hanging. */
boolean Bmp581_init(void);

/* There is deliberately no Bmp581_isPresent(). Presence is owned inside
 * Bmp581_read(), which uses its own calls to probe for a reconnected sensor.
 * An accessor would invite callers to gate the read on it, which is exactly
 * what made hot-plug recovery unreachable on the BMP388. Presence reaches the
 * outside world through measurementsSetBaro() and PeriphDiag instead. */

/** Read the CHIP_ID register (0x01). Bring-up helper; a BMP581 returns
 *  BMP581_CHIP_ID_PRIMARY or BMP581_CHIP_ID_SECONDARY. */
boolean Bmp581_readChipId(uint8 *chipId);

/** Read the latest pressure [Pa] and temperature [degC].
 *  Also re-probes for the device while it is absent, so a replugged sensor
 *  comes back on its own.
 *  \return FALSE on a bus error or while absent (outputs left unchanged). */
boolean Bmp581_read(float32 *pressurePa, float32 *temperatureC);

/** Number of configuration/status registers Bmp581_debugDump() returns. */
#define BMP581_DUMP_CFG_LEN    (7u)

/** One-shot bring-up dump. Fills \p cfg with the configuration and status
 *  registers in this order:
 *      [0] CHIP_ID 0x01   [1] REV_ID  0x02   [2] INT_STATUS 0x27
 *      [3] STATUS  0x28   [4] DSP_IIR 0x31   [5] OSR_CONFIG 0x36
 *      [6] ODR_CONFIG 0x37
 *  What to look for, in the order the bring-up fails:
 *      INT_STATUS bit 4 (por_softreset_complete) = 1 -> the soft reset landed.
 *          Clear-on-read, so only the first dump after init shows it.
 *      STATUS bit 0 (core_rdy) = 1. Do NOT expect bit 1 (nvm_rdy) here: it
 *          reads 1 in standby right after reset — which is where Bmp581_init()
 *          checks it, the only place it means anything — and 0 once the device
 *          is in normal mode, which is where this dump runs. Measured 0x01 on
 *          hardware 2026-08-01 while init had passed the same check moments
 *          earlier. A genuine trim failure shows up as Bmp581_init() returning
 *          FALSE, not as a bit in this dump.
 *      OSR_CONFIG bit 6 (press_en) = 1 -> pressure is actually being converted.
 *          Clear here is why a device reads a sane temperature next to a
 *          pressure register that never moves.
 *      ODR_CONFIG bits 1:0 (pwr_mode) = 1 -> normal mode, i.e. converting.
 *  \p osrEff receives OSR_EFF (0x38) — its bit 7 (odr_is_valid) is 1 only if
 *  the configured ODR is achievable at the configured oversampling; a 0 there
 *  means the device silently reduced the effective OSR, so the signal is
 *  noisier than configured but otherwise fine.
 *  \p raw receives the 6-byte measurement block at 0x1D (T then P).
 *      An all-0x7F burst here is NORMAL and not a fault: the data registers
 *      hold 0x7F7F7F until the first conversion completes, and this dump runs
 *      within a few ms of ODR_CONFIG starting them. Measured on hardware
 *      2026-08-01, with the 50 Hz task reading valid data immediately after.
 *      A burst that is still 0x7F once the task is running is the real fault.
 *  \return FALSE on a bus error (outputs undefined). */
boolean Bmp581_debugDump(uint8 cfg[BMP581_DUMP_CFG_LEN], uint8 *osrEff, uint8 raw[6]);

#endif /* BMP581_H */
