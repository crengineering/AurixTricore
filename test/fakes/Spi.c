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

#define FAKE_SPI_BURST_LEN   (14u)

static uint8   s_mode = SPI_MODE_0;
static boolean s_busOk = TRUE;
static uint8   s_whoAmI = 0x47u;                  /* ICM42688_WHO_AM_I_VALUE */
static uint8   s_burst[FAKE_SPI_BURST_LEN];       /* TEMP_DATA1..GYRO payload */
static uint32  s_resetWrites;                     /* DEVICE_CONFIG soft-reset attempts */

void Spi_setMode(uint8 spiMode)
{
    s_mode = spiMode;
}

uint8 Spi_getMode(void)
{
    return s_mode;
}

void Spi_init(void) {}

boolean Spi_transfer(const uint8 *tx, uint8 *rx, uint16 len)
{
    boolean ok = s_busOk;

    /* DEVICE_CONFIG (0x11) soft-reset write, MSB clear = write. Icm42688_init()
     * -> Icm42688_probe() issues exactly this write, ATTEMPTED unconditionally
     * (its own return value is discarded) -- counting the attempt, not a
     * success, is what makes this a faithful "how many times was
     * Icm42688_init() entered" proxy regardless of s_busOk at that instant. */
    if ((len == 2u) && (tx[0] == 0x11u) && (tx[1] == 0x01u))
    {
        s_resetWrites++;
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

    s_mode        = SPI_MODE_0;
    s_busOk       = TRUE;
    s_whoAmI      = 0x47u;
    s_resetWrites = 0u;
    for (i = 0u; i < FAKE_SPI_BURST_LEN; i++)
    {
        s_burst[i] = 0u;
    }
}

uint32 FakeSpi_softResetWriteCount(void)
{
    return s_resetWrites;
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
