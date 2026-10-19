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
 *   The magnetometer calibration lived here from v1.19.0 (layout 4) to
 *   v1.19.27. Since v1.19.28 (layout 5) it is in Xcp_FusionCal: compiled
 *   defaults carry the board's measured set (calibration/board.json), so a
 *   reflash -- which erases DFLASH -- comes up calibrated, and the RAM block
 *   is where tools/mag_cal.py tries a new fit before it goes into code
 *   (Chris, 2026-10-19: "no NVM, flash defaults with a calibration
 *   possibility"). Nothing here is board-specific any more.
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
/* XCP_NVM_ADDR deleted, T5 (docs/MEMORY_PLACEMENT.md): Xcp.c's whitelist
 * now uses (uint32)&g_xcpNvm, so Lcf_Tasking_Tricore_Tc.lsl
 * (LCF_XCP_NVM_START) is the only place 0x70030200 is written down. */
#define XCP_NVM_MAGIC   0x4D564E58u
#define XCP_NVM_SIZE    16u

/* default sea-level reference pressure [Pa] (ISA standard atmosphere) */
#define NVM_SEA_LEVEL_PA_DEFAULT    101325u

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
    /* New parameters are appended at the END so every address above keeps
     * its value (and NVM_LAYOUT_VERSION is bumped). */
} Xcp_Nvm;

extern volatile Xcp_Nvm g_xcpNvm;

void    Nvm_bootInit(void);     /* load defaults, then the DFLASH record     */
void    Nvm_task100ms(void);    /* execute a pending SAVE/DFLT command       */
boolean Nvm_hasFault(void);     /* corrupt boot record or last save failed   */

#endif /* NVM_H_ */
