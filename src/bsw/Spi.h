/**********************************************************************************************************************
 * \file Spi.h
 * \brief Blocking QSPI0 master bus service (flight IMU).
 *
 * QSPI0 on P22.8 SCLK / P22.9 MRST (MISO) / P22.10 MTSR (MOSI) / P22.11 SLSO10
 * (CS). One slave today: the ICM-42688-P evaluation board. See
 * docs/PINNING.md 2.2/2.5 and docs/ICM42688P.md.
 *
 * ⚠️ Electrically this bus is NOT symmetric, and the driver depends on it:
 *   - Port 22 is the 5 V VEXT domain and the IMU's absolute maximum is
 *     VDDIO + 0.3 V, so the three MCU-driven lines (SCLK, MTSR, CS) are level
 *     shifted OUTSIDE the chip by 1 kOhm/2 kOhm dividers. Nothing in software
 *     can substitute for those resistors.
 *   - The IMU answers at 3.3 V into a 5 V pad, whose default threshold is
 *     ~3.5 V and would never read high. Spi_init() therefore puts the pins in
 *     TTL pad mode (VIH = 2.0 V) — the same trick I2c.c uses for the sensor
 *     bus. Forgetting it looks exactly like a dead slave: clocks come out,
 *     nothing comes back.
 *
 * All accessors are blocking, and every wait is bounded. That is deliberate:
 * the I2C bring-up on 2026-07-30 hung CPU0 forever on an iLLD busy-wait with
 * no deadline, and the watchdogs are disabled on this project. A transfer that
 * does not complete inside the deadline fails and returns rather than spinning.
 *********************************************************************************************************************/
#ifndef SPI_H
#define SPI_H

#include "Ifx_Types.h"

/** SPI clock polarity/phase. The ICM-42688-P accepts either. */
#define SPI_MODE_0   (0u)
#define SPI_MODE_3   (3u)

/** Re-configure the bus for SPI mode 0 or 3 without re-initialising the module.
 *  Exists because the device datasheet is unavailable, so the driver probes
 *  both rather than guessing — see Icm42688_init(). */
void Spi_setMode(uint8 spiMode);

/** eturn the SPI mode currently configured (SPI_MODE_0 or SPI_MODE_3). */
uint8 Spi_getMode(void);

/** Bring up the QSPI0 master: pins, TTL pads, baud rate and the IMU channel.
 *  Call once at startup, before the scheduler. */
void Spi_init(void);

/** Full-duplex transfer of \p len bytes. \p rx may be NULL to discard the
 *  incoming bytes; \p tx must always be valid (send dummy bytes to clock a
 *  read out). CS is asserted for the whole transfer and released after it.
 *  \return TRUE if the transfer completed inside the deadline. */
boolean Spi_transfer(const uint8 *tx, uint8 *rx, uint16 len);

/** Diagnostic counters — separate a silent slave from a wedged master, the
 *  distinction that cost two days on the I2C bus. */
uint32 Spi_getOkCount(void);
uint32 Spi_getFailCount(void);

#endif /* SPI_H */
