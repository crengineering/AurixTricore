/* Test control surface for fakes/Spi.c (SYS1-001 strand B task 5). Not part
 * of the real Spi.h contract -- included directly by test_icm42688.c only. */
#ifndef FAKESPI_H
#define FAKESPI_H

#include "Ifx_Types.h"

/** Bus mode, WHO_AM_I answer and burst payload back to their defaults
 *  (mode 0, bus healthy, WHO_AM_I = 0x47, burst all zero). */
void FakeSpi_reset(void);

/** Make every subsequent Spi_transfer() fail (or succeed again). */
void FakeSpi_setBusOk(boolean ok);

/** WHO_AM_I register (0x75) value returned on the next read. */
void FakeSpi_setWhoAmI(uint8 whoAmI);

/** 14-byte TEMP_DATA1..GYRO burst returned by every subsequent read
 *  (layout: temp(0..1), accel X/Y/Z(2..7), gyro X/Y/Z(8..13), big-endian --
 *  same as Icm42688.c's own ICM42688_BURST_LEN). */
void FakeSpi_setBurst(const uint8 burst[14]);

/** Number of DEVICE_CONFIG (0x11) soft-reset writes attempted since the last
 *  FakeSpi_reset() -- since task 17, Icm42688_reinitStep()'s IDLE state
 *  issues exactly one per state-machine (re)start, same "how many times has
 *  a (re)init attempt begun" proxy the pre-task-17 blocking Icm42688_probe()
 *  gave, without touching production code. */
uint32 FakeSpi_softResetWriteCount(void);

/* --- task 17 (SYS1-001 strand B, B6.4) additions ------------------------- */

/** Total Spi_transfer() calls since the last FakeSpi_reset() -- the
 *  acceptance for Icm42688_reinitStep() is "at most one SPI transaction per
 *  call", checked by snapshotting this before/after a single step. */
uint32 FakeSpi_transferCallCount(void);

/** Total Spi_setMode() calls since the last FakeSpi_reset(). */
uint32 FakeSpi_setModeCallCount(void);

/** Number of entries logged in the write trace (register writes only, i.e.
 *  every 2-byte Spi_transfer with rx == NULL_PTR) -- capped at
 *  FAKE_SPI_WRITE_LOG_LEN; excess writes are still counted by
 *  FakeSpi_transferCallCount() but not logged. */
uint32 FakeSpi_writeLogCount(void);

/** The register (MSB already stripped) and value of write \p i (0-based, in
 *  the order they were issued). */
uint8 FakeSpi_writeLogReg(uint32 i);
uint8 FakeSpi_writeLogValue(uint32 i);

#endif /* FAKESPI_H */
