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

/* Bus-level counters at a fixed address, readable over XCP without the map
 * file. Exists because a sensor that fails to reconnect at run time looks
 * identical from the outside whether the device is silent or the master is
 * wedged -- these separate the two. Sits clear of the other pinned blocks
 * (cal 0x...100, nvm 0x...200, gpio 0x...300). */
#define XCP_I2CDBG_ADDR   0x70030400u
#define XCP_I2CDBG_MAGIC  0x49324344u   /* "I2CD" */

typedef struct
{
    uint32 magic;
    uint8  lastBusStatus;   /* IfxI2c_BusStatus of the last transfer attempt */
    uint8  lastResult;      /* 0 = ok, 1 = nak, 2 = fail                     */
    uint8  reserved[2];
    uint32 okCount;
    uint32 nakCount;
    uint32 failCount;
    uint32 recoverCount;    /* bus recovery + module re-init invocations     */
} I2c_Debug;

extern volatile I2c_Debug g_i2cDebug;

/** Bring up the I2C0 master (pins, TTL pads, baud rate). Call once at startup. */
void I2c_init(void);

/** Diagnostic: TRUE when both SCL and SDA read released (bus idle).
 *  Should be TRUE whenever no transfer is in progress; a persistent FALSE means
 *  a slave is holding the bus (or the pad level cannot be read back). */
boolean I2c_busIsIdle(void);

/** Diagnostic: the two bus lines separately, TRUE = released (pulled high).
 *  Which line is stuck says what broke: with the bus idle both must float
 *  high, so SCL low is a short to ground or a slave stretching the clock
 *  forever, and SDA low is a short or a slave jammed mid-byte. An OPEN circuit
 *  looks the opposite -- both lines perfectly idle, nothing ever ACKs.
 *  Call only between transfers (see PeriphDiag.c). */
void I2c_getLineState(boolean *sclReleased, boolean *sdaReleased);

/** Read \p len bytes starting at register \p reg of the 7-bit-addressed slave.
 *  \return TRUE only if the whole transfer was ACKed. */
boolean I2c_readReg(uint8 addr7, uint8 reg, uint8 *data, uint16 len);

/** Write \p len bytes to register \p reg of the 7-bit-addressed slave.
 *  \return TRUE only if the whole transfer was ACKed. */
boolean I2c_writeReg(uint8 addr7, uint8 reg, const uint8 *data, uint16 len);

/** Convenience: write a single register byte. */
boolean I2c_writeByte(uint8 addr7, uint8 reg, uint8 value);

#endif /* I2C_H */
