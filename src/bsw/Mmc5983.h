/**********************************************************************************************************************
 * \file Mmc5983.h
 * \brief MEMSIC MMC5983MA 3-axis magnetometer driver (I2C0).
 *
 * Second device on the shared I2C0 bus (docs/PINNING.md 2.3/2.5), alongside the
 * BMP581 barometer. Uses the blocking I2c bus service. Wiring, the CS trap and
 * the bring-up order are in docs/MMC5983MA.md.
 *
 * Board: MEMSIC MMC5983-B prototyping board. Two things about it drive this
 * driver's assumptions, both read off the board's schematic:
 *   - it carries its own 2.7 kOhm SDA/SCL pull-ups to VDD, which now sit in
 *     parallel with the BMP581's 10 kOhm (combined ~2.1 kOhm — still fine);
 *   - ⚠️ CS is brought straight out to the header with NO pull-up. It must be
 *     tied to VDD externally or the part answers on SPI and never ACKs on I2C.
 *
 * The address is fixed at 0x30 — the MMC5983MA has no address select pin, so
 * there is no 0x30/0x31 variant to get wrong (unlike the BMP581's 0x46/0x47).
 *
 * ⚠️ The register map below is NOT from the datasheet. The PDF in the
 * Quadrocopter repo (docs/MMC5983MA.md section 9) is the *prototyping board*
 * user's guide and contains no register information at all. Everything here is
 * written from the MMC5983MA register map and must be confirmed against
 * silicon — Mmc5983_debugDump() exists for exactly that, the same way
 * Bmp581_debugDump() did.
 *********************************************************************************************************************/
#ifndef MMC5983_H
#define MMC5983_H

#include "Ifx_Types.h"

#define MMC5983_I2C_ADDR        (0x30u)  /* fixed, no address select pin */
#define MMC5983_PRODUCT_ID      (0x30u)  /* expected value of PRODUCT_ID (0x2F) */

/** One magnetometer sample. Field is in gauss (1 G = 100 uT); Earth's field is
 *  roughly 0.25..0.65 G depending on location, ~0.48 G in Munich. */
typedef struct
{
    float32 mag[3];      /**< X, Y, Z field [gauss], sensor frame */
    float32 headingDeg;  /**< see Mmc5983_read() — level-only, uncalibrated */
} Mmc5983_Sample;

/** Soft-reset, verify the product ID, enable automatic set/reset, and start
 *  continuous measurement.
 *  \return TRUE if the product ID matched and every configuration write was
 *          ACKed. Safe to call with no sensor attached — returns FALSE without
 *          hanging. */
boolean Mmc5983_init(void);

/** \return TRUE while the device is answering. Diagnostic only — do NOT gate
 *  Mmc5983_read() on this, that would make the hot-plug recovery unreachable. */
boolean Mmc5983_isPresent(void);

/** Read the PRODUCT_ID register (0x2F). Bring-up helper; returns 0x30. */
boolean Mmc5983_readProductId(uint8 *productId);

/** Read the latest field sample. Also re-probes for the device while it is
 *  absent, so a replugged sensor comes back on its own.
 *
 *  \p sample->headingDeg is a bring-up aid, not a navigation output: it is
 *  atan2 of the two horizontal axes in degrees (0..360), which is only correct
 *  with the board LEVEL, and it carries no hard/soft-iron calibration and no
 *  magnetic declination. Rotating a level board must sweep it through a full
 *  360 deg — that is what it is for. Tilt compensation needs the IMU, which is
 *  not fitted (see docs/PINNING.md 2.2).
 *
 *  \return FALSE on a bus error or while absent (sample left unchanged). */
boolean Mmc5983_read(Mmc5983_Sample *sample);

/** Number of registers Mmc5983_debugDump() returns in \p cfg. */
#define MMC5983_DUMP_CFG_LEN    (5u)

/** One-shot bring-up dump. Fills \p cfg in this order:
 *      [0] PRODUCT_ID 0x2F   [1] STATUS 0x08   [2] CTRL0 0x09
 *      [3] CTRL1 0x0A        [4] CTRL2 0x0B
 *  What to look for:
 *      PRODUCT_ID = 0x30 -> the whole I2C path works. Anything else and
 *          nothing below is meaningful; check the CS strap first.
 *      STATUS -> observed 0x10 at boot (bit 4), i.e. no measurement completed
 *          yet, which is expected this close to init.
 *      CTRL0/1/2 -> ⚠️ DO NOT read these back to verify configuration. Measured
 *          on hardware 2026-08-01: all three returned 0x61 — identical to each
 *          other, and matching neither the written values (0x20, 0x00, 0x0C)
 *          nor zero. Three distinct registers reporting one value means the
 *          readback path simply does not return configuration. That the writes
 *          DO land is proven elsewhere: continuous mode runs and the field data
 *          fits a sphere to 0.9%, neither of which could happen if CTRL2 had
 *          been ignored. The data block is the only real test.
 *  \p raw receives the 7-byte measurement block at 0x00 (X, Y, Z, XYZout2).
 *      An all-zero burst here is NORMAL: continuous mode has only just been
 *      started and no conversion has completed yet (the BMP581 shows 0x7F in
 *      the same situation). All-zero once the 50 Hz task is running is the
 *      real fault.
 *  \return FALSE on a bus error (outputs undefined). */
boolean Mmc5983_debugDump(uint8 cfg[MMC5983_DUMP_CFG_LEN], uint8 raw[7]);

#endif /* MMC5983_H */
