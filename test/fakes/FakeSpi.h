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
 *  FakeSpi_reset() -- Icm42688_probe() (the only caller) issues exactly one
 *  per Icm42688_init() call as long as WHO_AM_I answers correctly on the
 *  first (SPI_MODE_0) attempt, making this a faithful "how many times was
 *  Icm42688_init() entered" proxy without touching production code. */
uint32 FakeSpi_softResetWriteCount(void);

#endif /* FAKESPI_H */
