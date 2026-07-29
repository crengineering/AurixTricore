/**********************************************************************************************************************
 * \file I2c.h
 * \brief Blocking I2C0 master bus service (shared sensor bus).
 *
 * I2C0 on P13.1 (SCL, X702.29) / P13.2 (SDA, X702.35). The bus is 3.3 V
 * open-drain with the sensor breakouts' own pull-ups; the pads run in a TTL
 * driver mode (VIH = 2.0 V) so the 5 V VEXT port domain still reads a 3.3 V
 * bus. See docs/PINNING.md sections 2.3 and 2.5.
 *
 * All accessors are blocking with a bounded internal retry, so an absent or
 * stuck slave fails fast (returns FALSE) instead of hanging the caller.
 *
 * Device addresses are passed as the 7-bit slave address (e.g. BMP581 = 0x46);
 * the 8-bit left-shift the iLLD driver expects is applied internally.
 *********************************************************************************************************************/
#ifndef I2C_H
#define I2C_H

#include "Ifx_Types.h"

/** Bring up the I2C0 master (pins, TTL pads, baud rate). Call once at startup. */
void I2c_init(void);

/** Read \p len bytes starting at register \p reg of the 7-bit-addressed slave.
 *  \return TRUE only if the whole transfer was ACKed. */
boolean I2c_readReg(uint8 addr7, uint8 reg, uint8 *data, uint16 len);

/** Write \p len bytes to register \p reg of the 7-bit-addressed slave.
 *  \return TRUE only if the whole transfer was ACKed. */
boolean I2c_writeReg(uint8 addr7, uint8 reg, const uint8 *data, uint16 len);

/** Convenience: write a single register byte. */
boolean I2c_writeByte(uint8 addr7, uint8 reg, uint8 value);

#endif /* I2C_H */
