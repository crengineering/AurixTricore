#ifndef NVM_H_
#define NVM_H_

#include "Ifx_Types.h"

/* Persistent parameter block, strictly separated from the RAM-only
 * calibration block (Diagnostics.h): only the parameters in this block are
 * stored in the on-chip DFLASH0. Pinned at a fixed address so XCP masters
 * can access it without the map file. Little-endian:
 *
 *   0x00  uint32  magic       0x4D564E58 ("XNVM"), set only by the firmware
 *   0x04  uint32  command     NVM command (NVM_CMD_*): the master writes
 *                             SAVE/DFLT, the firmware executes it in the
 *                             100 ms task and clears the word (handshake)
 *   0x08  uint32  userValue   first persistent parameter (free scratch
 *                             word, also used by tools/nvm_test.py)
 *   0x0C  uint32  seaLevelPa  sea-level reference pressure [Pa] for the baro
 *                             altitude (QNH); default 101325 (standard atm)
 *
 *   --- magnetometer calibration, appended in v1.19.0 (NVM layout 4) ---
 *   Per-BOARD, not per-design: the offsets are the board's own magnetics as
 *   the MMC5983MA sees them, so they belong in flash rather than in a header.
 *   All default to a no-op (zero offset, unit scale), which is the same
 *   uncalibrated behaviour as before this block existed.
 *   0x10  float32 magOffX     hard-iron offset [gauss], SENSOR frame
 *   0x14  float32 magOffY
 *   0x18  float32 magOffZ
 *   0x1C  float32 magScaleX   soft-iron scale, dimensionless, default 1.0
 *   0x20  float32 magScaleY
 *   0x24  float32 magScaleZ
 *   0x28  float32 magDeclDeg  magnetic declination [deg], east positive.
 *                             +3.9 in Munich. Turns magnetic north into TRUE
 *                             north; applied at the AHRS output, not inside
 *                             the filter (see Ahrs.c).
 *
 * New persistent parameters are appended here (bump NVM_LAYOUT_VERSION in
 * Nvm.c; stored records of an older layout are ignored, not a fault).
 *
 * Storage: two 4 KB DFLASH sectors as a ping-pong pair; every save erases
 * the inactive sector and programs a CRC-32-protected record there, so a
 * power loss during a save can never destroy the last good record. At
 * boot the newest valid record wins, otherwise defaults apply.
 *
 * All functions must be called from CPU0 only (no cross-core locking); a
 * save blocks the caller for the erase + program time (a few ms).
 */
#define XCP_NVM_ADDR    0x70030200u
#define XCP_NVM_MAGIC   0x4D564E58u
#define XCP_NVM_SIZE    44u

/* default sea-level reference pressure [Pa] (ISA standard atmosphere) */
#define NVM_SEA_LEVEL_PA_DEFAULT    101325u

/* Default magnetic declination [deg east]. Munich, 2026: +3.9 deg. Unlike the
 * hard-iron offsets this one is safe to default to a real value — it depends
 * on WHERE the board is, not on the board, and being 4 degrees out is strictly
 * better than being 0 degrees out by pretending declination does not exist. */
#define NVM_MAG_DECL_DEG_DEFAULT    (3.9f)

/* values for the 'command' word */
#define NVM_CMD_NONE        0x00000000u
#define NVM_CMD_SAVE        0x45564153u     /* "SAVE" (ASCII, little-endian) */
#define NVM_CMD_DEFAULTS    0x544C4644u     /* "DFLT" reload default values  */

typedef struct
{
    uint32 magic;
    uint32 command;
    uint32 userValue;
    uint32 seaLevelPa;

    /* magnetometer calibration — see Ahrs.c. Appended at the END so every
     * address above keeps its value; do the same for the next parameter. */
    float32 magOffX;
    float32 magOffY;
    float32 magOffZ;
    float32 magScaleX;
    float32 magScaleY;
    float32 magScaleZ;
    float32 magDeclDeg;
} Xcp_Nvm;

extern volatile Xcp_Nvm g_xcpNvm;

void    Nvm_bootInit(void);     /* load defaults, then the DFLASH record     */
void    Nvm_task100ms(void);    /* execute a pending SAVE/DFLT command       */
boolean Nvm_hasFault(void);     /* corrupt boot record or last save failed   */

#endif /* NVM_H_ */
