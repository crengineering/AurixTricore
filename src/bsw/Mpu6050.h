/**********************************************************************************************************************
 * \file Mpu6050.h
 * \brief InvenSense MPU-6050 6-axis IMU driver (3-axis gyro + 3-axis acc), I2C0.
 *
 * GY-521 breakout on the shared I2C0 bus (docs/PINNING.md 2.2/2.3/2.5), sharing
 * SCL/SDA with the BMP388 barometer. Uses the blocking I2c bus service. This is
 * a learning-project stand-in for the delayed ICM-42688-P (which was planned on
 * QSPI0); the MPU-6050 lives on I2C instead — see docs/MPU6050.md.
 *
 * The bring-up milestone is Mpu6050_readWhoAmI() returning 0x68 — that one
 * transaction proves the whole I2C path (pins, TTL pads, breakout pull-ups,
 * address, driver). No address collision with the BMP388 (0x77): AD0 = low
 * gives 0x68.
 *
 * GY-521 pad map (8 pads) and wiring for I2C0:
 *   VCC -> board +3V3 rail (the breakout LDO + bus pull-ups reference this;
 *          keeps the shared bus at 3.3 V — see docs/MPU6050.md 3)
 *   GND -> header GND
 *   SCL -> P13.1 SCL (X702-29)          SDA -> P13.2 SDA (X702-35)
 *   AD0 -> GND (or left open; on-board pull-down) -> address 0x68
 *   INT -> data-ready, left open for now (driver polls at 50 Hz)
 *   XDA/XCL -> auxiliary I2C master (external mag), left open
 *********************************************************************************************************************/
#ifndef MPU6050_H
#define MPU6050_H

#include "Ifx_Types.h"

#define MPU6050_I2C_ADDR       (0x68u)   /* AD0 = low; use 0x69 if AD0 = high */
#define MPU6050_WHO_AM_I_VALUE (0x68u)   /* expected value of the WHO_AM_I register */

/** One IMU sample in physical units. */
typedef struct
{
    float32 acc[3];   /* acceleration  [g]     order X, Y, Z */
    float32 gyro[3];    /* angular rate  [deg/s] order X, Y, Z */
    float32 tempC;      /* on-die temperature [degC] */
} Mpu6050_Sample;

/** Wake the device, verify WHO_AM_I, and configure ranges / filter / rate.
 *  \return TRUE if WHO_AM_I matched and configuration was ACKed. Safe to call
 *          with no sensor attached — it returns FALSE without hanging. */
boolean Mpu6050_init(void);

/** Read the WHO_AM_I register (0x75). Bring-up helper; MPU-6050 returns 0x68.
 *  (Some GY-521 clones carry an MPU-6500/9250 die returning 0x70/0x71 — see
 *  docs/MPU6050.md.) */
boolean Mpu6050_readWhoAmI(uint8 *whoAmI);

/** Read the latest 14-byte sensor burst and scale it to physical units.
 *  \return FALSE on a bus error or before init (output left unchanged). */
boolean Mpu6050_read(Mpu6050_Sample *sample);

/** Bring-up diagnostic for a one-time boot dump over UART:
 *  \p cfg4  = SMPLRT_DIV, CONFIG, GYRO_CONFIG, ACCEL_CONFIG (0x19..0x1C)
 *  \p pwr3  = USER_CTRL, PWR_MGMT_1, PWR_MGMT_2 (0x6A..0x6C)
 *  \p raw14 = the raw 14-byte sample burst (0x3B..0x48).
 *  If \p raw14 is all zero, read \p pwr3 first — see docs/MPU6050.md §7. */
boolean Mpu6050_debugDump(uint8 *cfg4, uint8 *pwr3, uint8 *raw14);

#endif /* MPU6050_H */
