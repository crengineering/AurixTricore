#include <string.h>
#include "Nvm.h"
#include "IfxFlash.h"
#include "IfxScuWdt.h"

/* The persistent parameter block; XCP write access is permitted from
 * offset 0x04 (Xcp.c), the magic word is set only by the firmware. Off
 * __at() onto `#pragma section` (docs/MEMORY_PLACEMENT.md T4) -- an absolute
 * group at the unchanged LCF_XCP_NVM_START literal. */
#pragma section farbss "xcp_nvm"
volatile Xcp_Nvm g_xcpNvm;
#pragma section farbss restore

/* Local u-suffixed copies of the iLLD flash constants: the iLLD defines
 * them without suffix, which leaks essentially-signed operands into every
 * expression (MISRA 7.2/10.4). Guarded against drift below. */
#define NVM_DFLASH_START    0xAF000000u     /* = IFXFLASH_DFLASH_START       */
#define NVM_PAGE_SIZE       8u              /* = IFXFLASH_DFLASH_PAGE_LENGTH */

#if (IFXFLASH_DFLASH_START != 0xAF000000u) || (IFXFLASH_DFLASH_PAGE_LENGTH != 8u)
#error "NVM flash constants out of sync with the iLLD device configuration"
#endif

/* Two logical 4 KB sectors at the start of the DFLASH0 EEPROM area.
 * Nothing else in this project uses the DFLASH. */
#define NVM_SECTOR_COUNT    2u
#define NVM_SECTOR_SIZE     0x1000u
#define NVM_SECTOR_ADDR(i)  (NVM_DFLASH_START + ((i) * NVM_SECTOR_SIZE))

#define NVM_REC_MAGIC       0x4C41434Eu     /* "NCAL" (ASCII, little-endian) */
#define NVM_LAYOUT_VERSION  4u              /* bump on Xcp_Nvm layout change */

typedef struct
{
    uint32 magic;       /* NVM_REC_MAGIC                                     */
    uint16 layout;      /* NVM_LAYOUT_VERSION                                */
    uint16 length;      /* payload length in bytes (= sizeof(Xcp_Nvm))       */
    uint32 sequence;    /* increments on every save; newest record wins      */
    uint32 crc;         /* CRC-32 (IEEE 802.3) over the payload              */
} Nvm_Header;

typedef struct
{
    Nvm_Header hdr;
    Xcp_Nvm    payload;
} Nvm_Record;

/* record buffer rounded up to whole 8-byte DFLASH pages, in 32-bit words */
#define NVM_RECORD_PAGES    ((sizeof(Nvm_Record) + (NVM_PAGE_SIZE - 1u)) / NVM_PAGE_SIZE)
#define NVM_RECORD_WORDS    (NVM_RECORD_PAGES * 2u)

/* per-sector record classification */
typedef enum
{
    Nvm_RecState_valid = 0,     /* current layout, CRC ok                    */
    Nvm_RecState_blank = 1,     /* no record magic (erased flash reads 0)    */
    Nvm_RecState_stale = 2,     /* record of an older layout - ignored       */
    Nvm_RecState_corrupt = 3    /* current layout but damaged                */
} Nvm_RecState;

/* ping-pong state, established by Nvm_bootInit() */
static boolean s_haveActive;
static uint32  s_activeSector;      /* index 0/1 of the sector holding the
                                       newest valid record                   */
static uint32  s_activeSequence;

/* TRUE after a corrupt boot record or a failed save; cleared by the next
 * successful save */
static boolean s_fault;

static uint32 nvm_crc32(const uint8 *data, uint32 len)
{
    uint32 crc = 0xFFFFFFFFu;
    uint32 i;

    for (i = 0u; i < len; i++)
    {
        uint32 bit;

        crc ^= (uint32)data[i];

        for (bit = 0u; bit < 8u; bit++)
        {
            if ((crc & 1u) != 0u)
            {
                crc = (crc >> 1) ^ 0xEDB88320u;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFFu;
}

/* CRC over the payload as raw bytes; the copy avoids a pointer-type cast
 * on the struct (MISRA 11.3) */
static uint32 nvm_crcPayload(const Xcp_Nvm *payload)
{
    uint8 bytes[sizeof(Xcp_Nvm)];

    /* cppcheck-suppress misra-c2012-21.15 ; deviation: intentional
     * serialization of the struct into raw bytes for the CRC */
    (void)memcpy(bytes, payload, sizeof(bytes));

    return nvm_crc32(bytes, (uint32)sizeof(bytes));
}

static uint32 nvm_readWord(uint32 addr)
{
    /* cppcheck-suppress misra-c2012-11.4 ; deviation: memory-mapped DFLASH
     * can only be read through an integer-to-pointer conversion */
    return *(const volatile uint32 *)addr;
}

/* Read a whole record image from flash into words[NVM_RECORD_WORDS]. */
static void nvm_readRecord(uint32 sectorAddr, uint32 *words)
{
    uint32 i;

    for (i = 0u; i < NVM_RECORD_WORDS; i++)
    {
        words[i] = nvm_readWord(sectorAddr + (i * 4u));
    }
}

/* Classify a record image; for a valid record copy the payload to *dst. */
static Nvm_RecState nvm_classify(const uint32 *words, Xcp_Nvm *dst, uint32 *sequence)
{
    Nvm_Record   rec;
    Nvm_RecState state;

    /* cppcheck-suppress misra-c2012-21.15 ; deviation: intentional
     * deserialization of the raw flash words into the record struct */
    (void)memcpy(&rec, words, sizeof(rec));

    if (rec.hdr.magic != NVM_REC_MAGIC)
    {
        state = Nvm_RecState_blank;
    }
    else if (rec.hdr.layout != (uint16)NVM_LAYOUT_VERSION)
    {
        state = Nvm_RecState_stale;     /* pre-update record: expected      */
    }
    else if ((rec.hdr.length == (uint16)sizeof(Xcp_Nvm))
             && (rec.hdr.crc == nvm_crcPayload(&rec.payload)))
    {
        *dst      = rec.payload;
        *sequence = rec.hdr.sequence;
        state     = Nvm_RecState_valid;
    }
    else
    {
        state = Nvm_RecState_corrupt;
    }

    return state;
}

static void nvm_loadDefaults(void)
{
    g_xcpNvm.userValue  = 0u;
    g_xcpNvm.seaLevelPa = NVM_SEA_LEVEL_PA_DEFAULT;

    /* Magnetometer calibration defaults to a no-op: zero hard iron, unit
     * scale. That is deliberately the SAME behaviour as before this block
     * existed, so a board with no stored record behaves exactly as it used
     * to rather than applying a plausible-looking wrong correction. */
    g_xcpNvm.magOffX    = 0.0f;
    g_xcpNvm.magOffY    = 0.0f;
    g_xcpNvm.magOffZ    = 0.0f;
    g_xcpNvm.magScaleX  = 1.0f;
    g_xcpNvm.magScaleY  = 1.0f;
    g_xcpNvm.magScaleZ  = 1.0f;
    g_xcpNvm.magDeclDeg = NVM_MAG_DECL_DEG_DEFAULT;

    g_xcpNvm.command    = NVM_CMD_NONE;
    g_xcpNvm.magic      = XCP_NVM_MAGIC;
}

static boolean nvm_saveBlock(const Xcp_Nvm *src)
{
    uint32     words[NVM_RECORD_WORDS] = {0};
    Nvm_Record rec;
    uint32     target;
    uint32     targetAddr;
    uint32     page;
    uint16     pw;
    boolean    ok = TRUE;

    rec.hdr.magic    = NVM_REC_MAGIC;
    rec.hdr.layout   = (uint16)NVM_LAYOUT_VERSION;
    rec.hdr.length   = (uint16)sizeof(Xcp_Nvm);
    rec.hdr.sequence = s_activeSequence + 1u;
    rec.payload      = *src;
    rec.hdr.crc      = nvm_crcPayload(&rec.payload);
    /* cppcheck-suppress misra-c2012-21.15 ; deviation: intentional
     * serialization of the record struct into the flash word buffer */
    (void)memcpy(words, &rec, sizeof(rec));

    /* ping-pong: never touch the sector holding the last good record */
    target     = (s_haveActive != FALSE) ? (1u - s_activeSector) : 0u;
    targetAddr = NVM_SECTOR_ADDR(target);
    pw         = IfxScuWdt_getSafetyWatchdogPassword();

    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_eraseSector(targetAddr);
    IfxScuWdt_setSafetyEndinit(pw);

    if (IfxFlash_waitUnbusy(0u, IfxFlash_FlashType_D0) != 0u)
    {
        ok = FALSE;
    }

    for (page = 0u; (page < NVM_RECORD_PAGES) && (ok != FALSE); page++)
    {
        uint32 pageAddr = targetAddr + (page * NVM_PAGE_SIZE);

        if (IfxFlash_enterPageMode(pageAddr) != 0u)
        {
            ok = FALSE;
        }
        else
        {
            (void)IfxFlash_waitUnbusy(0u, IfxFlash_FlashType_D0);
            IfxFlash_loadPage2X32(pageAddr, words[page * 2u], words[(page * 2u) + 1u]);

            IfxScuWdt_clearSafetyEndinit(pw);
            IfxFlash_writePage(pageAddr);
            IfxScuWdt_setSafetyEndinit(pw);

            if (IfxFlash_waitUnbusy(0u, IfxFlash_FlashType_D0) != 0u)
            {
                ok = FALSE;
            }
        }
    }

    if (ok != FALSE)
    {
        uint32 i;

        for (i = 0u; i < NVM_RECORD_WORDS; i++)
        {
            if (nvm_readWord(targetAddr + (i * 4u)) != words[i])
            {
                ok = FALSE;
            }
        }
    }

    if (ok != FALSE)
    {
        s_haveActive     = TRUE;
        s_activeSector   = target;
        s_activeSequence = rec.hdr.sequence;
    }

    return ok;
}

void Nvm_bootInit(void)
{
    uint32  sector;
    boolean anyCorrupt = FALSE;

    s_haveActive     = FALSE;
    s_activeSector   = 0u;
    s_activeSequence = 0u;

    nvm_loadDefaults();

    for (sector = 0u; sector < NVM_SECTOR_COUNT; sector++)
    {
        uint32       words[NVM_RECORD_WORDS];
        Xcp_Nvm      stored;
        uint32       sequence;
        Nvm_RecState state;

        nvm_readRecord(NVM_SECTOR_ADDR(sector), words);
        state = nvm_classify(words, &stored, &sequence);

        if (state == Nvm_RecState_valid)
        {
            if ((s_haveActive == FALSE) || ((uint32)(sequence - s_activeSequence) < 0x80000000u))
            {
                s_haveActive     = TRUE;
                s_activeSector   = sector;
                s_activeSequence = sequence;

                stored.command = NVM_CMD_NONE;  /* never resurrect a command */
                stored.magic   = XCP_NVM_MAGIC;
                g_xcpNvm       = stored;
            }
        }
        else if (state == Nvm_RecState_corrupt)
        {
            anyCorrupt = TRUE;
        }
        else
        {
            /* blank or stale layout: expected, not a fault */
        }
    }

    /* blank/stale flash silently falls back to the defaults */
    if ((s_haveActive == FALSE) && (anyCorrupt != FALSE))
    {
        s_fault = TRUE;
    }
    else
    {
        s_fault = FALSE;
    }
}

void Nvm_task100ms(void)
{
    uint32 cmd = g_xcpNvm.command;

    if (cmd == NVM_CMD_SAVE)
    {
        Xcp_Nvm snapshot = g_xcpNvm;

        snapshot.command = NVM_CMD_NONE;    /* never persist a command       */
        snapshot.magic   = XCP_NVM_MAGIC;

        if (nvm_saveBlock(&snapshot) != FALSE)
        {
            s_fault = FALSE;
        }
        else
        {
            s_fault = TRUE;
        }

        g_xcpNvm.command = NVM_CMD_NONE;    /* handshake for the master      */
    }
    else if (cmd == NVM_CMD_DEFAULTS)
    {
        nvm_loadDefaults();                 /* also clears the command word  */
    }
    else
    {
        /* no (known) command pending */
    }

    /* self-check: a garbled block falls back to defaults */
    if (g_xcpNvm.magic != XCP_NVM_MAGIC)
    {
        nvm_loadDefaults();
    }
}

boolean Nvm_hasFault(void)
{
    return s_fault;
}
