#ifndef DIAGNOSTICS_H_
#define DIAGNOSTICS_H_

#include "Ifx_Types.h"

/* Board diagnostics: compares the live measurements against calibratable
 * limits and reports violations as a 32-bit status bitmask in Xcp_Data
 * (see DIAGNOSTICS.md for the user-facing documentation).
 *
 * Calibration block, pinned at a fixed address, writable via XCP
 * DOWNLOAD/SHORT_DOWNLOAD (the only writable region). Little-endian,
 * 64 bytes, all values float32:
 *
 *   0x00  uint32   magic          0x4C414358 ("XCAL")
 *   0x04  float32  dtsMin         [degC]  PMS DTS lower limit
 *   0x08  float32  dtsMax         [degC]  PMS DTS upper limit
 *   0x0C  float32  dtscMin        [degC]  SCU DTSC lower limit
 *   0x10  float32  dtscMax        [degC]  SCU DTSC upper limit
 *   0x14  float32  vddMin         [V]     VDD 1.25 V rail
 *   0x18  float32  vddMax         [V]
 *   0x1C  float32  vddp3Min       [V]     VDDP3 3.3 V rail
 *   0x20  float32  vddp3Max       [V]
 *   0x24  float32  vextMin        [V]     VEXT 5 V board supply
 *   0x28  float32  vextMax        [V]
 *   0x2C  float32  tempDeltaMax   [K]     max |DTS - DTSC| plausibility
 *   0x30  float32  debounceSec    [s]     violation must persist this long
 *                                         (0.1 s resolution, e.g. 1.5)
 *   0x34  float32  fsVdd          [V]     monitor-ADC full scale VDD
 *   0x38  float32  fsVddp3        [V]     monitor-ADC full scale VDDP3
 *   0x3C  float32  fsVext         [V]     monitor-ADC full scale VEXT
 */
#define XCP_CAL_ADDR    0x70030100u
#define XCP_CAL_MAGIC   0x4C414358u
#define XCP_CAL_SIZE    64u

typedef struct
{
    uint32  magic;
    float32 dtsMin;
    float32 dtsMax;
    float32 dtscMin;
    float32 dtscMax;
    float32 vddMin;
    float32 vddMax;
    float32 vddp3Min;
    float32 vddp3Max;
    float32 vextMin;
    float32 vextMax;
    float32 tempDeltaMax;
    float32 debounceSec;
    float32 fsVdd;
    float32 fsVddp3;
    float32 fsVext;
} Xcp_Cal;

/* diagStatus bits in Xcp_Data (see DIAGNOSTICS.md) */
#define DIAG_DTS_UNDERTEMP      (1u << 0)
#define DIAG_DTS_OVERTEMP       (1u << 1)
#define DIAG_DTSC_UNDERTEMP     (1u << 2)
#define DIAG_DTSC_OVERTEMP      (1u << 3)
#define DIAG_VDD_UNDERVOLT      (1u << 4)
#define DIAG_VDD_OVERVOLT       (1u << 5)
#define DIAG_VDDP3_UNDERVOLT    (1u << 6)
#define DIAG_VDDP3_OVERVOLT     (1u << 7)
#define DIAG_VEXT_UNDERVOLT     (1u << 8)
#define DIAG_VEXT_OVERVOLT      (1u << 9)
#define DIAG_TEMP_IMPLAUSIBLE   (1u << 10)
#define DIAG_CAL_INVALID        (1u << 31)

extern volatile Xcp_Cal g_xcpCal;

void diagnosticsInit(void);     /* load default calibration (CPU0, once)     */
void diagnosticsUpdate(void);   /* call every 100 ms, after measurements     */

#endif /* DIAGNOSTICS_H_ */
