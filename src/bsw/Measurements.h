#ifndef MEASUREMENTS_H_
#define MEASUREMENTS_H_

#include "Ifx_Types.h"

/* Live measurement block read by XCP masters (pyXCP, AurixGUI) via
 * SHORT_UPLOAD. Pinned to a fixed address so clients do not need the map
 * file. Little-endian, 84 bytes:
 *
 *   0x00  uint32   magic       0x41555258
 *   0x04  uint8    verMajor
 *   0x05  uint8    verMinor
 *   0x06  uint8    verStep
 *   0x07  uint8    reserved
 *   0x08  uint32   tickMs      uptime in ms (lwIP tick)
 *   0x0C  float32  dieTempC    die temperature, PMS DTS (standby domain)
 *   0x10  float32  dtscTempC   die temperature, SCU DTSC (core domain)
 *   0x14  float32  vddCore     VDD 1.25 V core rail [V]
 *   0x18  float32  vddp3       VDDP3 3.3 V I/O rail [V]
 *   0x1C  float32  vext        VEXT board supply [V]
 *   0x20  uint8    rawVdd      raw 8-bit monitor-ADC codes, for verifying
 *   0x21  uint8    rawVddp3    the voltage scale factors against nominals
 *   0x22  uint8    rawVext
 *   0x23  uint8    baroPresent 1 if the BMP388 answered at init, else 0
 *   0x24  uint32   diagStatus  diagnostics bitmask, see DIAGNOSTICS.md
 *   0x28  float32  baroPressPa BMP388 pressure [Pa]    (0 when not present)
 *   0x2C  float32  baroTempC   BMP388 temperature [degC] (0 when not present)
 *   0x30  float32  baroAltM    pressure altitude [m] vs standard 1013.25 hPa
 *                              (0 when not present; see measurementsSetBaro)
 *   0x34  uint8    imuPresent  1 if the MPU-6050 answered at init, else 0
 *   0x35  uint8    imuReserved[3]  padding to 4-byte float alignment
 *   0x38  float32  accelX      MPU-6050 acceleration X [g]     (0 when absent)
 *   0x3C  float32  accelY      MPU-6050 acceleration Y [g]
 *   0x40  float32  accelZ      MPU-6050 acceleration Z [g]
 *   0x44  float32  gyroX       MPU-6050 angular rate X [deg/s], BIAS-CORRECTED
 *                              (raw = this + biasX; 0 when absent)
 *   0x48  float32  gyroY       MPU-6050 angular rate Y [deg/s]
 *   0x4C  float32  gyroZ       MPU-6050 angular rate Z [deg/s]
 *   0x50  float32  imuTempC    MPU-6050 die temperature [degC] (0 when absent)
 *
 *   --- attitude estimate (Ahrs.c), matches flight_ctrl.h conventions ---
 *   0x54  uint8    ahrsState   0 = calibrating, 1 = running, 2 = no sensor
 *   0x55  uint8    ahrsAccOk   1 while |a| ~= 1 g and the acc is trusted
 *   0x56  uint8    ahrsReserved[2]
 *   0x58  float32  roll        phi   [rad]   -> flight_ctrl phi_ist[0]
 *   0x5C  float32  pitch       theta [rad]   -> flight_ctrl phi_ist[1]
 *   0x60  float32  yaw         psi   [rad]   -> flight_ctrl phi_ist[2] (drifts)
 *   0x64  float32  rateP       p [rad/s]     -> flight_ctrl om_ist[0]
 *   0x68  float32  rateQ       q [rad/s]     -> flight_ctrl om_ist[1]
 *   0x6C  float32  rateR       r [rad/s]     -> flight_ctrl om_ist[2]
 *   0x70  float32  biasX       measured gyro bias X [deg/s]
 *   0x74  float32  biasY       measured gyro bias Y [deg/s]
 *   0x78  float32  biasZ       measured gyro bias Z [deg/s]
 *
 *   --- per-core execution time (CoreStats.c) ---
 *   0x7C  uint32   coreExecUs[6]    busy time per 100 ms window, per core [us]
 *   0x94  uint16   coreLoadPmil[6]  that time as per mille of the window
 *   0xA0  uint16   coreAlive[6]     +1 per window; frozen => that core hung
 *
 *   --- Ethernet (EthStats.c) ---
 *   0xAC  uint32   ethBytesPerSec   TX+RX throughput [bytes/s]
 *   0xB0  uint16   ethUtilPmil      link utilisation [per mille, 0..1000]
 *   0xB2  uint16   ethLinkMbits     negotiated link rate [Mbit/s]
 *
 * Total size 0xB4 = 180 bytes. Exceeds XCP MAX_CTO (64), so clients read it in
 * several SHORT_UPLOADs — AurixGUI already does this for the IMU sub-block.
 */
#define XCP_DATA_ADDR   0x70030000u
#define XCP_DATA_MAGIC  0x41555258u

typedef struct
{
    uint32  magic;
    uint8   verMajor;
    uint8   verMinor;
    uint8   verStep;
    uint8   reserved;
    uint32  tickMs;
    float32 dieTempC;
    float32 dtscTempC;
    float32 vddCore;
    float32 vddp3;
    float32 vext;
    uint8   rawVdd;
    uint8   rawVddp3;
    uint8   rawVext;
    uint8   baroPresent;
    uint32  diagStatus;
    float32 baroPressPa;
    float32 baroTempC;
    float32 baroAltM;
    uint8   imuPresent;
    uint8   imuReserved[3];
    float32 accelX;
    float32 accelY;
    float32 accelZ;
    float32 gyroX;
    float32 gyroY;
    float32 gyroZ;
    float32 imuTempC;

    /* attitude estimate — see Ahrs.h */
    uint8   ahrsState;
    uint8   ahrsAccOk;
    uint8   ahrsReserved[2];
    float32 roll;
    float32 pitch;
    float32 yaw;
    float32 rateP;
    float32 rateQ;
    float32 rateR;
    float32 biasX;
    float32 biasY;
    float32 biasZ;

    /* per-core execution time — see CoreStats.h */
    uint32  coreExecUs[6];
    uint16  coreLoadPmil[6];
    uint16  coreAlive[6];

    /* Ethernet link — see EthStats.h */
    uint32  ethBytesPerSec;
    uint16  ethUtilPmil;
    uint16  ethLinkMbits;
} Xcp_Data;

void measurementsInit(void);    /* DTS + DTSC + EVR monitor init (CPU0)  */
void measurementsUpdate(void);  /* call cyclically, e.g. every 100 ms    */

/* Publish the latest barometer sample into the XCP block. Called by the baro
 * task; pass present = FALSE to show "no sensor" (pressure/temp forced to 0). */
void measurementsSetBaro(boolean present, float32 pressurePa, float32 temperatureC);

/* Publish the latest IMU sample into the XCP block. Called by the IMU task;
 * pass present = FALSE to show "no sensor" (all axes + temp forced to 0).
 * acc is [g] and gyro is [deg/s] BIAS-CORRECTED, each in X,Y,Z order. */
void measurementsSetImu(boolean present, const float32 acc[3], const float32 gyro[3],
                        float32 temperatureC);

/* Publish the attitude estimate (Ahrs.c) into the XCP block. */
void measurementsSetAttitude(void);

/* Publish per-core execution time and Ethernet utilisation. Called from the
 * 100 ms task on CPU0; reads the other cores' slots in g_coreStats. */
void measurementsSetSystemLoad(void);

#endif /* MEASUREMENTS_H_ */
