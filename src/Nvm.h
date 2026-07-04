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
#define XCP_NVM_SIZE    12u

/* values for the 'command' word */
#define NVM_CMD_NONE        0x00000000u
#define NVM_CMD_SAVE        0x45564153u     /* "SAVE" (ASCII, little-endian) */
#define NVM_CMD_DEFAULTS    0x544C4644u     /* "DFLT" reload default values  */

typedef struct
{
    uint32 magic;
    uint32 command;
    uint32 userValue;
} Xcp_Nvm;

extern volatile Xcp_Nvm g_xcpNvm;

void    Nvm_bootInit(void);     /* load defaults, then the DFLASH record     */
void    Nvm_task100ms(void);    /* execute a pending SAVE/DFLT command       */
boolean Nvm_hasFault(void);     /* corrupt boot record or last save failed   */

#endif /* NVM_H_ */
