#ifndef MEASUREMENTS_H_
#define MEASUREMENTS_H_

#include "Ifx_Types.h"

/* Live measurement block read by XCP masters (pyXCP, AurixGUI) via
 * SHORT_UPLOAD. Pinned to a fixed address so clients do not need the map
 * file. Little-endian, 36 bytes:
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
 *   0x23  uint8    reserved2
 *   0x24  uint32   diagStatus  diagnostics bitmask, see DIAGNOSTICS.md
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
    uint8   reserved2;
    uint32  diagStatus;
} Xcp_Data;

void measurementsInit(void);    /* DTS + DTSC + EVR monitor init (CPU0)  */
void measurementsUpdate(void);  /* call cyclically, e.g. every 100 ms    */

#endif /* MEASUREMENTS_H_ */
