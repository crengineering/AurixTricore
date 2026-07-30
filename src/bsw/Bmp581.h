/**********************************************************************************************************************
 * \file Bmp581.h
 * \brief Bosch BMP581 barometric pressure + temperature sensor driver (I2C0).
 *
 * First sensor on the shared I2C0 bus (docs/PINNING.md 2.3/2.5). Uses the
 * blocking I2c bus service. The bring-up milestone is Bmp581_readChipId()
 * returning 0x50 — that one transaction proves the whole I2C path (pins, TTL
 * pads, breakout pull-ups, address, driver).
 *
 * Register addresses / scaling below follow the BMP581 datasheet; they are
 * flagged for confirmation against the datasheet once hardware is in hand.
 *********************************************************************************************************************/
#ifndef BMP581_H
#define BMP581_H

#include "Ifx_Types.h"

#define BMP581_I2C_ADDR       (0x46u)   /* SDO = low; use 0x47 if SDO = high */
#define BMP581_CHIP_ID_VALUE  (0x50u)   /* expected value of the CHIP_ID register */

/** Soft-reset, verify the chip ID, and start continuous measurement.
 *  \return TRUE if the chip ID matched and configuration was ACKed. Safe to
 *          call with no sensor attached — it returns FALSE without hanging. */
boolean Bmp581_init(void);

/** \return TRUE once Bmp581_init() has succeeded. */
boolean Bmp581_isPresent(void);

/** Read the CHIP_ID register (0x01). Bring-up helper; BMP581 returns 0x50. */
boolean Bmp581_readChipId(uint8 *chipId);

/** Read the latest pressure [Pa] and temperature [degC].
 *  \return FALSE on a bus error (outputs left unchanged). */
boolean Bmp581_read(float32 *pressurePa, float32 *temperatureC);

#endif /* BMP581_H */
