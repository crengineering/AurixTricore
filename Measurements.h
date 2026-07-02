#ifndef MEASUREMENTS_H_
#define MEASUREMENTS_H_

#include "Ifx_Types.h"

/* Live measurement block read by XCP masters (pyXCP, AurixGUI) via
 * SHORT_UPLOAD. Pinned to a fixed address so clients do not need the map
 * file. Little-endian, 16 bytes:
 *
 *   0x00  uint32   magic       0x41555258 ("XRUA" byte order on the wire)
 *   0x04  uint8    verMajor
 *   0x05  uint8    verMinor
 *   0x06  uint8    verStep
 *   0x07  uint8    reserved
 *   0x08  uint32   tickMs      uptime in ms (lwIP tick)
 *   0x0C  float32  dieTempC    die temperature from the PMS DTS
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
} Xcp_Data;

void measurementsInit(void);    /* DTS init + struct init (CPU0, once)   */
void measurementsUpdate(void);  /* call cyclically, e.g. every 100 ms    */

#endif /* MEASUREMENTS_H_ */
