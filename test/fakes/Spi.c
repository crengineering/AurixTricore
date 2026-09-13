/* Stub QSPI0 bus service for host tests -- see fakes/I2c.c for why.
 *
 * Controllable as of SYS1-001 strand B task 5: test_icm42688.c needs
 * Icm42688_init()/Icm42688_verifyPresence() to see a WHO_AM_I answer (right
 * or wrong), and Icm42688_read() to see a chosen 14-byte burst (including a
 * constant, "stuck" frame -- evidence 952275AD99001303) -- a fake that
 * always fails (the pre-task-5 version) cannot drive either. Every existing
 * caller of this file (only test_icm42688's plausibility tests, which never
 * touch the bus at all) is unaffected by the new default. */
#include "Spi.h"

#define FAKE_SPI_BURST_LEN     (14u)
#define FAKE_SPI_WRITE_LOG_LEN (32u)

static uint8   s_mode = SPI_MODE_0;
static boolean s_busOk = TRUE;
static uint8   s_whoAmI = 0x47u;                  /* ICM42688_WHO_AM_I_VALUE */
static uint8   s_burst[FAKE_SPI_BURST_LEN];       /* TEMP_DATA1..GYRO payload */
static uint32  s_resetWrites;                     /* DEVICE_CONFIG soft-reset attempts */

/* Task 17 (SYS1-001 strand B, B6.4) additions. */
static uint32  s_transferCalls;
static uint32  s_setModeCalls;
static uint8   s_writeLogReg[FAKE_SPI_WRITE_LOG_LEN];
static uint8   s_writeLogVal[FAKE_SPI_WRITE_LOG_LEN];
static uint32  s_writeLogCount;

void Spi_setMode(uint8 spiMode)
{
    s_mode = spiMode;
    s_setModeCalls++;
}

uint8 Spi_getMode(void)
{
    return s_mode;
}

void Spi_init(void) {}

boolean Spi_transfer(const uint8 *tx, uint8 *rx, uint16 len)
{
    boolean ok = s_busOk;

    s_transferCalls++;

    /* DEVICE_CONFIG (0x11) soft-reset write, MSB clear = write. Since task 17,
     * Icm42688_reinitStep()'s IDLE state issues exactly this write,
     * ATTEMPTED unconditionally (its own return value is discarded) --
     * counting the attempt, not a success, is what makes this a faithful
     * "how many times has a (re)init attempt begun" proxy regardless of
     * s_busOk at that instant. */
    if ((len == 2u) && (tx[0] == 0x11u) && (tx[1] == 0x01u))
    {
        s_resetWrites++;
    }

    /* Task 17: log every 2-byte WRITE (rx == NULL_PTR distinguishes a write
     * from a 2-byte WHO_AM_I-style read) for the golden-trace comparison. */
    if ((len == 2u) && (rx == NULL_PTR) && (s_writeLogCount < FAKE_SPI_WRITE_LOG_LEN))
    {
        s_writeLogReg[s_writeLogCount] = tx[0] & 0x7Fu;
        s_writeLogVal[s_writeLogCount] = tx[1];
        s_writeLogCount++;
    }
    else
    {
        /* not a loggable write, or the log is full */
    }

    if ((ok != FALSE) && (rx != NULL_PTR) && (len >= 1u))
    {
        const uint8 reg = tx[0] & 0x7Fu;
        uint16      i;

        rx[0] = 0u;   /* turnaround byte -- the real driver discards it too */

        if (reg == 0x75u)              /* WHO_AM_I */
        {
            if (len >= 2u)
            {
                rx[1] = s_whoAmI;
            }
        }
        else if (reg == 0x1Du)         /* TEMP_DATA1 -- the 14-byte burst start */
        {
            for (i = 1u; (i < len) && ((i - 1u) < FAKE_SPI_BURST_LEN); i++)
            {
                rx[i] = s_burst[i - 1u];
            }
        }
        else
        {
            for (i = 1u; i < len; i++)
            {
                rx[i] = 0u;
            }
        }
    }
    else
    {
        /* bus failure, or a write (rx == NULL_PTR): nothing to fill in */
    }

    return ok;
}

/* --- test control surface: FakeSpi_* is used ONLY from test_icm42688.c --- */

void FakeSpi_reset(void)
{
    uint8 i;

    s_mode          = SPI_MODE_0;
    s_busOk         = TRUE;
    s_whoAmI        = 0x47u;
    s_resetWrites   = 0u;
    s_transferCalls = 0u;
    s_setModeCalls  = 0u;
    s_writeLogCount = 0u;
    for (i = 0u; i < FAKE_SPI_BURST_LEN; i++)
    {
        s_burst[i] = 0u;
    }
    for (i = 0u; i < FAKE_SPI_WRITE_LOG_LEN; i++)
    {
        s_writeLogReg[i] = 0u;
        s_writeLogVal[i] = 0u;
    }
}

uint32 FakeSpi_softResetWriteCount(void)
{
    return s_resetWrites;
}

uint32 FakeSpi_transferCallCount(void)
{
    return s_transferCalls;
}

uint32 FakeSpi_setModeCallCount(void)
{
    return s_setModeCalls;
}

uint32 FakeSpi_writeLogCount(void)
{
    return s_writeLogCount;
}

uint8 FakeSpi_writeLogReg(uint32 i)
{
    uint8 reg = 0u;

    if (i < s_writeLogCount)
    {
        reg = s_writeLogReg[i];
    }
    else
    {
        /* out of range -- 0 is not a valid register in this map anyway */
    }
    return reg;
}

uint8 FakeSpi_writeLogValue(uint32 i)
{
    uint8 value = 0u;

    if (i < s_writeLogCount)
    {
        value = s_writeLogVal[i];
    }
    else
    {
        /* out of range */
    }
    return value;
}

void FakeSpi_setBusOk(boolean ok)
{
    s_busOk = ok;
}

void FakeSpi_setWhoAmI(uint8 whoAmI)
{
    s_whoAmI = whoAmI;
}

/** Program the 14-byte TEMP_DATA1..GYRO burst returned by every subsequent
 *  read. Passing the same constant frame on every call is exactly evidence
 *  952275AD99001303's "well-formed but frozen" sample. */
void FakeSpi_setBurst(const uint8 burst[14])
{
    uint8 i;

    for (i = 0u; i < FAKE_SPI_BURST_LEN; i++)
    {
        s_burst[i] = burst[i];
    }
}
