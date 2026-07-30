/**********************************************************************************************************************
 * \file Bmp388.h
 * \brief Bosch BMP388 barometric pressure + temperature sensor driver (I2C0).
 *
 * Learning-project sensor (CJMCU-388 breakout) on the shared I2C0 bus
 * (docs/PINNING.md 2.3/2.5), standing in for the delayed BMP581. Uses the
 * blocking I2c bus service.
 *
 * The bring-up milestone is Bmp388_readChipId() returning 0x50 — that one
 * transaction proves the whole I2C path (pins, TTL pads, breakout pull-ups,
 * address, driver).
 *
 * Unlike the BMP581 (which outputs ready-scaled values), the BMP388 returns raw
 * 24-bit ADC counts plus a block of factory calibration coefficients; the driver
 * runs Bosch's floating-point compensation (datasheet BST-BMP388-DS001 §9) to
 * produce Pa and degC.
 *
 * CJMCU-388 breakout pad map (8 pads) and wiring for I2C0:
 *   VIN -> board +3V3 rail (NOT 5V: the SDA/SCL pull-ups reference VIN, so 5V
 *          here would over-volt the 3.3V bus and the sensor's abs-max)
 *   3Vo -> leave open (regulated 3.3V *output* of the onboard regulator)
 *   GND -> header GND
 *   SCK -> P13.1 SCL (X702-29)          SDI -> P13.2 SDA (X702-35)
 *   CS  -> tie to 3V3 (selects I2C; datasheet §5.1 — a low latches SPI)
 *   SDO -> address LSB, must not float: GND = 0x76, 3V3 = 0x77
 *   INT -> data-ready, left open for now
 *********************************************************************************************************************/
#ifndef BMP388_H
#define BMP388_H

#include "Ifx_Types.h"

#define BMP388_I2C_ADDR       (0x77u)   /* SDO = high; use 0x76 if SDO = low */
#define BMP388_CHIP_ID_VALUE  (0x50u)   /* expected value of the CHIP_ID register */

/** Soft-reset, verify the chip ID, read the calibration trim, and start
 *  continuous (normal-mode) measurement.
 *  \return TRUE if the chip ID matched and configuration was ACKed. Safe to
 *          call with no sensor attached — it returns FALSE without hanging. */
boolean Bmp388_init(void);

/** Read the CHIP_ID register (0x00). Bring-up helper; BMP388 returns 0x50. */
boolean Bmp388_readChipId(uint8 *chipId);

/** Read the latest pressure [Pa] and temperature [degC], applying the Bosch
 *  compensation formula against the calibration read at init.
 *  \return FALSE on a bus error or before init (outputs left unchanged). */
boolean Bmp388_read(float32 *pressurePa, float32 *temperatureC);

#endif /* BMP388_H */
