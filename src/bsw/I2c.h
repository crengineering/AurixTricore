/**********************************************************************************************************************
 * \file I2c.h
 * \brief Blocking I2C0 master bus service (shared sensor bus).
 *
 * I2C0 on P13.1 (SCL, X702.29) / P13.2 (SDA, X702.35). The bus is 3.3 V
 * open-drain with the sensor breakouts' own pull-ups; the pads run in a TTL
 * driver mode (VIH = 2.0 V) so the 5 V VEXT port domain still reads a 3.3 V
 * bus. See docs/PINNING.md sections 2.3 and 2.5.
 *
 * All accessors are blocking. An absent slave (NAK) or a bus found stuck before
 * the transfer starts fails fast and returns FALSE: the retry loops are bounded,
 * and each call first checks that SCL/SDA are released, bit-banging a recovery
 * sequence if they are not.
 *
 * NOT fully hang-proof: the underlying iLLD IfxI2c_I2c_read2()/write2() contain
 * unbounded busy-waits, so a bus that breaks *during* a transfer can still spin
 * CPU0 forever (the watchdogs are disabled on this project). See i2c_ensureBusFree()
 * in I2c.c for what is and is not covered.
 *
 * Device addresses are passed as the 7-bit slave address (e.g. BMP581 = 0x46);
 * the 8-bit left-shift the iLLD driver expects is applied internally.
 *********************************************************************************************************************/
#ifndef I2C_H
#define I2C_H

#include "Ifx_Types.h"

/** Bring up the I2C0 master (pins, TTL pads, baud rate). Call once at startup. */
void I2c_init(void);

/** Diagnostic: TRUE when both SCL and SDA read released (bus idle).
 *  Should be TRUE whenever no transfer is in progress; a persistent FALSE means
 *  a slave is holding the bus (or the pad level cannot be read back). */
boolean I2c_busIsIdle(void);

/** Read \p len bytes starting at register \p reg of the 7-bit-addressed slave.
 *  \return TRUE only if the whole transfer was ACKed. */
boolean I2c_readReg(uint8 addr7, uint8 reg, uint8 *data, uint16 len);

/** Write \p len bytes to register \p reg of the 7-bit-addressed slave.
 *  \return TRUE only if the whole transfer was ACKed. */
boolean I2c_writeReg(uint8 addr7, uint8 reg, const uint8 *data, uint16 len);

/** Convenience: write a single register byte. */
boolean I2c_writeByte(uint8 addr7, uint8 reg, uint8 value);

#endif /* I2C_H */
