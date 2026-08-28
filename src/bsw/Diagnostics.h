#ifndef DIAGNOSTICS_H_
#define DIAGNOSTICS_H_

#include "Ifx_Types.h"

/* Board diagnostics: compares the live measurements against calibratable
 * limits and reports violations as a 32-bit status bitmask in Xcp_Data
 * (see DIAGNOSTICS.md for the user-facing documentation).
 *
 * Calibration block, pinned at a fixed address, writable via XCP
 * DOWNLOAD/SHORT_DOWNLOAD. RAM only — after a reset the defaults apply
 * (persistent parameters live in the separate Xcp_Nvm block, see Nvm.h).
 * Little-endian, 64 bytes, all values float32:
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
/* XCP_CAL_ADDR deleted, T5 (docs/MEMORY_PLACEMENT.md): Xcp.c's whitelist
 * now uses (uint32)&g_xcpCal, so Lcf_Tasking_Tricore_Tc.lsl
 * (LCF_XCP_CAL_START) is the only place 0x70030100 is written down. */
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

/* diagStatus bits in Xcp_Data (see DIAGNOSTICS.md). Hex literals instead
 * of (1u << n): MISRA 12.2 sees 1u as essentially unsigned char, making
 * every shift beyond bit 7 a violation. */
#define DIAG_DTS_UNDERTEMP      0x00000001u     /* bit 0  */
#define DIAG_DTS_OVERTEMP       0x00000002u     /* bit 1  */
#define DIAG_DTSC_UNDERTEMP     0x00000004u     /* bit 2  */
#define DIAG_DTSC_OVERTEMP      0x00000008u     /* bit 3  */
#define DIAG_VDD_UNDERVOLT      0x00000010u     /* bit 4  */
#define DIAG_VDD_OVERVOLT       0x00000020u     /* bit 5  */
#define DIAG_VDDP3_UNDERVOLT    0x00000040u     /* bit 6  */
#define DIAG_VDDP3_OVERVOLT     0x00000080u     /* bit 7  */
#define DIAG_VEXT_UNDERVOLT     0x00000100u     /* bit 8  */
#define DIAG_VEXT_OVERVOLT      0x00000200u     /* bit 9  */
#define DIAG_TEMP_IMPLAUSIBLE   0x00000400u     /* bit 10 */
#define DIAG_UART_DISCONNECTED  0x00000800u     /* bit 11 */
#define DIAG_NVM_FAULT          0x00001000u     /* bit 12 */

/* Peripheral faults, maintained by PeriphDiag.c. The bus bits are shared by
 * every device on I2C0; the per-device bits say which sensor is affected.
 * Reading them together tells you where to look: a bus bit means the wiring
 * common to both sensors, a single device's bits mean that device's own
 * wiring, and STUCK_DATA means the wiring is fine and the part is not. */
#define DIAG_I2C_SCL_STUCK      0x00002000u     /* bit 13 SCL held low       */
#define DIAG_I2C_SDA_STUCK      0x00004000u     /* bit 14 SDA held low       */
#define DIAG_BARO_NO_RESPONSE   0x00008000u     /* bit 15 never answered     */
#define DIAG_BARO_TIMEOUT       0x00010000u     /* bit 16 answered, now gone */
#define DIAG_BARO_STUCK_DATA    0x00020000u     /* bit 17 value frozen       */
#define DIAG_BARO_IMPLAUSIBLE   0x00040000u     /* bit 18 out of range       */
#define DIAG_IMU_NO_RESPONSE    0x00080000u     /* bit 19                    */
#define DIAG_IMU_TIMEOUT        0x00100000u     /* bit 20                    */
#define DIAG_IMU_STUCK_DATA     0x00200000u     /* bit 21                    */
#define DIAG_IMU_IMPLAUSIBLE    0x00400000u     /* bit 22                    */
#define DIAG_MAG_NO_RESPONSE    0x00800000u     /* bit 23 MMC5983MA          */
#define DIAG_MAG_TIMEOUT        0x01000000u     /* bit 24                    */
#define DIAG_MAG_STUCK_DATA     0x02000000u     /* bit 25                    */
#define DIAG_MAG_IMPLAUSIBLE    0x04000000u     /* bit 26                    */
#define DIAG_GNSS_NO_RESPONSE   0x08000000u     /* bit 27                    */
#define DIAG_GNSS_TIMEOUT       0x10000000u     /* bit 28                    */
#define DIAG_GNSS_STUCK_DATA    0x20000000u     /* bit 29                    */
#define DIAG_GNSS_IMPLAUSIBLE   0x40000000u     /* bit 30                    */

/* ⚠️ Bits 27-30 are the LAST four free bits: room for exactly ONE more
 * peripheral in this scheme. Still to come are the GNSS (NEO-M9N), the flight
 * IMU (ICM-42688-P) and 4x ESC telemetry — six devices needing 24 bits into 4.
 * Before adding the device after next, move the per-device faults OUT of this
 * shared word into a per-peripheral status array indexed by PeriphDiag_Id, and
 * leave diagStatus for board-level faults. That change touches Xcp_Data, the
 * A2L and the GUI's BIT_MASK rows together, so it wants doing deliberately
 * rather than under pressure. See docs/DIAGNOSTICS.md. */

#define DIAG_CAL_INVALID        0x80000000u     /* bit 31 */

extern volatile Xcp_Cal g_xcpCal;

void diagnosticsInit(void);     /* load default calibration (CPU0, once)     */
boolean diagnosticsUpdate(void);   /* call every 100 ms, after measurements     */

#endif /* DIAGNOSTICS_H_ */
