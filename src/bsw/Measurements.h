#ifndef MEASUREMENTS_H_
#define MEASUREMENTS_H_

#include "Ifx_Types.h"
#include "GnssM9N.h"
#include "fusion.h"

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
 *   0x23  uint8    baroPresent 1 if the BMP581 answered at init, else 0
 *   0x24  uint32   diagStatus  diagnostics bitmask, see DIAGNOSTICS.md
 *   0x28  float32  baroPressPa BMP581 pressure [Pa]    (0 when not present)
 *   0x2C  float32  baroTempC   BMP581 temperature [degC] (0 when not present)
 *   0x30  float32  baroAltM    pressure altitude [m] vs standard 1013.25 hPa
 *                              (0 when not present; see measurementsSetBaro)
 *
 *   --- IMU block. No IMU has been fitted since 2026-07-31 (the MPU-6050 came
 *       off with the BMP388); every field below is published as zero once at
 *       start-up and never updated, until the ICM-42688-P arrives on QSPI0.
 *       The layout is deliberately unchanged so the A2L and the GUI keep
 *       working across the swap. ---
 *   0x34  uint8    imuPresent  1 if an IMU answered at init, else 0
 *   0x35  uint8    imuReserved[3]  padding to 4-byte float alignment
 *   0x38  float32  accelX      acceleration X [g]              (0 when absent)
 *   0x3C  float32  accelY      acceleration Y [g]
 *   0x40  float32  accelZ      acceleration Z [g]
 *   0x44  float32  gyroX       angular rate X [deg/s], RAW -- there is no
 *                              bias estimator in the tree (0 when absent)
 *   0x48  float32  gyroY       angular rate Y [deg/s]
 *   0x4C  float32  gyroZ       angular rate Z [deg/s]
 *   0x50  float32  imuTempC    IMU die temperature [degC]      (0 when absent)
 *
 *   --- per-core execution time (CoreStats.c) ---
 *   0x54  uint32   coreExecUs[6]    busy time per 100 ms window, per core [us]
 *   0x6C  uint16   coreLoadPmil[6]  that time as per mille of the window
 *   0x78  uint16   coreAlive[6]     +1 per window; frozen => that core hung
 *
 *   --- Ethernet (EthStats.c) ---
 *   0x84  uint32   ethBytesPerSec   TX+RX throughput [bytes/s]
 *   0x88  uint16   ethUtilPmil      link utilisation [per mille, 0..1000]
 *   0x8A  uint16   ethLinkMbits     negotiated link rate [Mbit/s]
 *
 *   --- magnetometer (Mmc5983.c), appended in v1.15.0 ---
 *   Appended at the END on purpose: every address above keeps its value, so
 *   the existing A2L entries and the GUI need no rework to stay correct.
 *   0x8C  uint8    magPresent  1 if the MMC5983MA answered at init, else 0
 *   0x8D  uint8    magReserved[3]  padding to 4-byte float alignment
 *   0x90  float32  magX        field X [gauss] (1 G = 100 uT; 0 when absent)
 *   0x94  float32  magY        field Y [gauss]
 *   0x98  float32  magZ        field Z [gauss]
 *   0x9C  float32  magFieldG   |B| [gauss] — the orientation-INDEPENDENT
 *                              magnitude, ~0.48 G in Munich. The single most
 *                              useful number here: it must not change as the
 *                              board is rotated, so it validates scaling and
 *                              axis assembly at a glance (docs/MMC5983MA.md 8.5)
 *   0xA0  float32  magHeadingDeg  0..360, LEVEL-ONLY and uncalibrated, no
 *                              declination — a bring-up aid, not navigation
 *
 *   --- GNSS (GnssM9N.c), appended in v1.18.0; ends at 0xD8 ---
 *
 * Total size 0xD8 = 216 bytes, leaving 40 bytes before Xcp_Cal at
 * 0x70030100. That headroom exists only because the attitude block was
 * removed on 2026-08-22 -- before that the block ended exactly at 0x100 and
 * had nothing left. Exceeds XCP MAX_CTO (64), so clients read it in
 * several SHORT_UPLOADs — AurixGUI already does this for the IMU sub-block.
 * DAQ has room: 8 ODTs x 63 B = 504 B (see XCP_DAQ_MAX_ODTS in Xcp.c).
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

    /* per-core execution time — see CoreStats.h */
    uint32  coreExecUs[6];
    uint16  coreLoadPmil[6];
    uint16  coreAlive[6];

    /* Ethernet link — see EthStats.h */
    uint32  ethBytesPerSec;
    uint16  ethUtilPmil;
    uint16  ethLinkMbits;

    /* magnetometer — see Mmc5983.h. Appended last to keep every address above
     * unchanged; do the same for the next sensor. */
    uint8   magPresent;
    uint8   magReserved[3];
    float32 magX;
    float32 magY;
    float32 magZ;
    float32 magFieldG;
    float32 magHeadingDeg;

    /* gnss- see GnssM9N.h */
    uint8   gnssPresent;
    uint16  gnssyear;
    uint8   gnssmonth;
    uint8   gnssday;
    uint8   gnsshour;
    uint8   gnssmin;
    uint8   gnsssec;
    uint8   gnssfixOk;          /* 1 = receiver says the solution is valid */
    uint8   gnsstimeOk;         /* 1 = UTC date+time fully resolved        */
    uint8   gnssnavOk;          /* 1 = safe to fuse; see GnssM9N.c         */
                                /* these three also pad gnssrxBytes to a   */
                                /* 4-byte boundary: TASKING 2-aligns       */
                                /* uint32, gen_a2l.py assumes 4            */
    uint32  gnssrxBytes;
    uint32  gnsssentences;
    uint16  gnsserrors;
    uint8   gnssfixType;
    uint8   gnssnumSats;

    /* navigation solution from UBX-NAV-PVT. gnssnumSats above ends on a
     * 4-byte boundary, so no padding is needed here -- but check
     * Measurements.src if you add anything, TASKING aligns uint32 to 2. */
    float32 gnsslatDeg;
    float32 gnsslonDeg;
    float32 gnssaltM;
    float32 gnssspeedMps;
    float32 gnssheadingDeg;
    float32 gnsshAccM;
    float32 gnssvAccM;

    float32 fusionElapsed;
    float32 a_D;

} Xcp_Data;

void measurementsInit(void);    /* DTS + DTSC + EVR monitor init (CPU0)  */
void measurementsUpdate(void);  /* call cyclically, e.g. every 100 ms    */

/* Publish the latest barometer sample into the XCP block. Called by the baro
 * task; pass present = FALSE to show "no sensor" (pressure/temp forced to 0). */
void measurementsSetBaro(boolean present, float32 pressurePa, float32 temperatureC);

/* Publish the latest magnetometer sample into the XCP block. Called by the mag
 * task; pass present = FALSE to show "no sensor" (all fields forced to 0).
 * mag is [gauss] in X,Y,Z order; the magnitude is computed here so every
 * consumer sees the same definition. */
void measurementsSetMag(boolean present, const float32 mag[3], float32 headingDeg);

/* Publish the latest IMU sample into the XCP block. Called by the IMU task;
 * pass present = FALSE to show "no sensor" (all axes + temp forced to 0).
 * acc is [g] and gyro is [deg/s] BIAS-CORRECTED, each in X,Y,Z order. */
void measurementsSetImu(boolean present, const float32 acc[3], const float32 gyro[3],
                        float32 temperatureC);

/* Publish per-core execution time and Ethernet utilisation. Called from the
 * 100 ms task on CPU0; reads the other cores' slots in g_coreStats. */
void measurementsSetSystemLoad(void);

/*
 * Publish the latest gnss sample into the XCP block. Called by the measurement task
 */
void measurementsSetGnss(boolean present, GnssM9N_Sample sample_info);

/*
 * Publish the fusion results
 */
void measurementsSetFusion(FusionValues *fusion, boolean present, float32 elapsed);

#endif /* MEASUREMENTS_H_ */
