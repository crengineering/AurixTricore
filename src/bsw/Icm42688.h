/**********************************************************************************************************************
 * \file Icm42688.h
 * \brief TDK-InvenSense ICM-42688-P 6-axis IMU driver (QSPI0).
 *
 * The flight IMU: 3-axis gyro + 3-axis accelerometer on the QSPI0 master
 * (src/bsw/Spi.c), replacing the interim MPU-6050 that sat on I2C0 until
 * 2026-07-31. Wiring, jumper settings, level shifting and bring-up order are
 * in docs/ICM42688P.md; MCU pins are in docs/PINNING.md 2.2/2.5.
 *
 * Board: TDK-InvenSense EV_ICM-42688-P evaluation board. Two things about that
 * board are load-bearing and are handled OUTSIDE this driver:
 *   - it runs at 3.3 V injected past its on-board LDOs (JP1/JP2 pin 2, jumpers
 *     open), and
 *   - the three MCU-driven lines are level shifted by 1 kOhm/2 kOhm dividers,
 *     because Port 22 is the 5 V VEXT domain and this part's absolute maximum
 *     is VDDIO + 0.3 V.
 * No amount of driver code substitutes for either. See docs/ICM42688P.md 1-2.
 *
 * ⚠️ The register map below is NOT from the datasheet. The PDF available
 * (AN-000488, in the Quadrocopter repo) is the *evaluation board* user guide
 * and says outright that registers are in the product specification, which we
 * do not have. Everything here is written from the ICM-42688-P register map
 * and must be confirmed against silicon — Icm42688_debugDump() exists for
 * exactly that, the same way it did for the BMP581 and MMC5983MA.
 *********************************************************************************************************************/
#ifndef ICM42688_H
#define ICM42688_H

#include "Ifx_Types.h"

/** Expected value of the WHO_AM_I register (0x75). */
#define ICM42688_WHO_AM_I_VALUE   (0x47u)

/** One IMU sample. Deliberately laid out like Mpu6050_Sample so the existing
 *  AHRS and measurement path take it unchanged. */
typedef struct
{
    float32 acc[3];    /**< acceleration [g],     sensor frame, X/Y/Z */
    float32 gyro[3];   /**< angular rate [deg/s], sensor frame, X/Y/Z */
    float32 tempC;     /**< die temperature [degC] */
} Icm42688_Sample;

/** Soft-reset, verify WHO_AM_I, and start the gyro and accelerometer in
 *  low-noise mode at 1 kHz.
 *  \return TRUE if WHO_AM_I matched and every configuration write went out.
 *          Safe to call with no sensor attached — returns FALSE without
 *          hanging (every SPI wait is bounded, see Spi.h). */
boolean Icm42688_init(void);

/* There is deliberately no Icm42688_isPresent() — see the note in Bmp581.h:
 * presence is owned inside Icm42688_read() so hot-plug recovery stays
 * reachable, and it reaches the outside world via measurementsSetImu(). */

/** Read the WHO_AM_I register (0x75). Bring-up helper; returns 0x47.
 *  This single transfer proves pins, pad modes, level shifting, CS, SPI mode
 *  and driver together — it is the whole bring-up gate. */
boolean Icm42688_readWhoAmI(uint8 *whoAmI);

/** Read the latest sample. Also re-probes for the device while it is absent,
 *  so a replugged sensor comes back on its own.
 *  \return FALSE on a bus error or while absent (sample left unchanged). */
boolean Icm42688_read(Icm42688_Sample *sample);

/** Number of registers Icm42688_debugDump() returns in \p cfg. */
#define ICM42688_DUMP_CFG_LEN   (5u)

/** One-shot bring-up dump. Fills \p cfg in this order:
 *      [0] WHO_AM_I 0x75   [1] PWR_MGMT0 0x4E   [2] GYRO_CONFIG0 0x4F
 *      [3] ACCEL_CONFIG0 0x50   [4] INT_STATUS 0x2D
 *  What to look for:
 *      WHO_AM_I = 0x47 -> everything below is meaningful. Anything else and
 *          the problem is wiring, CS or SPI mode, not configuration.
 *          0x00 or 0xFF specifically means "no data coming back at all":
 *          check the TTL pad mode on MRST before suspecting the part.
 *      PWR_MGMT0 = 0x0F -> gyro and accel both in low-noise mode. 0x00 means
 *          the part is still asleep and every sample will read zero.
 *      GYRO_CONFIG0 / ACCEL_CONFIG0 = 0x06 -> full scale and 1 kHz ODR.
 *  \p raw receives the 14-byte measurement block at 0x1D (temp, accel, gyro).
 *  \return FALSE on a bus error (outputs undefined). */
boolean Icm42688_debugDump(uint8 cfg[ICM42688_DUMP_CFG_LEN], uint8 raw[14]);

#endif /* ICM42688_H */
