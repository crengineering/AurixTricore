/**********************************************************************************************************************
 * \file PeriphDiag.c
 * \brief Generic peripheral fault detection — see PeriphDiag.h.
 *********************************************************************************************************************/
#include "PeriphDiag.h"
#include "Diagnostics.h"
#include "I2c.h"

/* A device is "gone" after this long without a successful read. Generous
 * against the 50 Hz poll rate: a handful of NAKs during a bus recovery is
 * normal and must not raise a fault. */
#define PD_TIMEOUT_TICKS      (10u)     /* 1.0 s */

/* An unchanging value only means something over a long window. Sensor noise
 * moves the liveness sum every sample on a healthy device, so 5 s of a
 * bit-identical value is decisive rather than merely quiet. */
#define PD_STUCK_TICKS        (50u)     /* 5.0 s */

/* Implausible readings must persist; a single corrupted transfer should not
 * raise a fault the operator has to interpret. */
#define PD_IMPLAUSIBLE_TICKS  (10u)     /* 1.0 s */

/* A bus line held low this long is a short or a jammed slave rather than
 * normal traffic. Sampling happens between transfers (see PeriphDiag_update),
 * so even one tick would be suspicious; three makes it certain. */
#define PD_BUS_STUCK_TICKS    (3u)      /* 0.3 s */

/* diagStatus bits per peripheral, in PeriphDiag_Id order. */
typedef struct
{
    uint32 noResponse;
    uint32 timeout;
    uint32 stuck;
    uint32 implausible;
} PeriphDiag_Bits;

typedef struct
{
    boolean fitted;          /* this build expects the device to be present   */
    boolean everOk;          /* a read has succeeded at least once since boot */
    boolean sawOk;           /* a read succeeded since the last update()      */
    boolean sawImplausible;  /* a successful read carried a bad value         */
    boolean haveLast;        /* lastLive is valid                             */
    float32 lastLive;        /* previous liveness value                       */
    uint16  silentTicks;     /* update() ticks with no successful read        */
    uint16  frozenTicks;     /* update() ticks with an unchanged liveness     */
    uint16  implausibleTicks;
} PeriphDiag_State;

static PeriphDiag_State s_periph[PERIPH_DIAG_COUNT];
static uint16           s_sclLowTicks;
static uint16           s_sdaLowTicks;

void PeriphDiag_init(void)
{
    uint8 i;

    for (i = 0u; i < (uint8)PERIPH_DIAG_COUNT; i++)
    {
        /* Unfitted by default: the caller declares what this build expects, so
         * adding an enum entry for a device that is not wired up yet cannot
         * light up the diagnostics on its own. */
        s_periph[i].fitted           = FALSE;
        s_periph[i].everOk           = FALSE;
        s_periph[i].sawOk            = FALSE;
        s_periph[i].sawImplausible   = FALSE;
        s_periph[i].haveLast         = FALSE;
        s_periph[i].lastLive         = 0.0f;
        s_periph[i].silentTicks      = 0u;
        s_periph[i].frozenTicks      = 0u;
        s_periph[i].implausibleTicks = 0u;
    }
    s_sclLowTicks = 0u;
    s_sdaLowTicks = 0u;
}

void PeriphDiag_setFitted(PeriphDiag_Id id, boolean fitted)
{
    if ((uint8)id < (uint8)PERIPH_DIAG_COUNT)
    {
        s_periph[id].fitted = fitted;
    }
}

void PeriphDiag_report(PeriphDiag_Id id, boolean readOk, boolean plausible, float32 liveness)
{
    if ((uint8)id < (uint8)PERIPH_DIAG_COUNT)
    {
        PeriphDiag_State *st = &s_periph[id];

        if (readOk != FALSE)
        {
            st->sawOk  = TRUE;
            st->everOk = TRUE;

            if (plausible == FALSE)
            {
                st->sawImplausible = TRUE;
            }

            /* Exact float comparison is deliberate. On a live sensor the low
             * bits of the liveness sum move with noise every single sample, so
             * bit-equality over seconds means the device has genuinely stopped
             * updating -- not that the signal happens to be quiet. */
            if ((st->haveLast != FALSE) && (liveness == st->lastLive))
            {
                if (st->frozenTicks < 0xFFFFu)
                {
                    st->frozenTicks++;
                }
            }
            else
            {
                st->frozenTicks = 0u;
            }
            st->lastLive = liveness;
            st->haveLast = TRUE;
        }
    }
}

/* Count a condition up to a saturating limit, or reset it. */
static void pd_track(uint16 *ticks, boolean active)
{
    if (active != FALSE)
    {
        if (*ticks < 0xFFFFu)
        {
            (*ticks)++;
        }
    }
    else
    {
        *ticks = 0u;
    }
}

uint32 PeriphDiag_update(void)
{
    /* Block scope (MISRA 8.9): only this function maps peripherals to bits. */
    static const PeriphDiag_Bits s_bits[PERIPH_DIAG_COUNT] =
    {
        { DIAG_BARO_NO_RESPONSE, DIAG_BARO_TIMEOUT, DIAG_BARO_STUCK_DATA, DIAG_BARO_IMPLAUSIBLE },
        { DIAG_IMU_NO_RESPONSE,  DIAG_IMU_TIMEOUT,  DIAG_IMU_STUCK_DATA,  DIAG_IMU_IMPLAUSIBLE  },
        { DIAG_MAG_NO_RESPONSE,  DIAG_MAG_TIMEOUT,  DIAG_MAG_STUCK_DATA,  DIAG_MAG_IMPLAUSIBLE  },
    };

    uint32  status = 0u;
    boolean sclReleased = TRUE;
    boolean sdaReleased = TRUE;
    boolean cond;
    uint8   i;

    /* Bus lines. Safe to sample here without disturbing anything: the sensor
     * tasks and this diagnostics tick run in the same single-threaded
     * scheduler loop, so no transfer can be in progress. With the bus idle
     * the pull-ups must hold both lines high; a line low is a short to ground
     * or a slave jamming the bus. */
    I2c_getLineState(&sclReleased, &sdaReleased);
    cond = FALSE;
    if (sclReleased == FALSE) { cond = TRUE; }
    pd_track(&s_sclLowTicks, cond);
    cond = FALSE;
    if (sdaReleased == FALSE) { cond = TRUE; }
    pd_track(&s_sdaLowTicks, cond);

    if (s_sclLowTicks >= PD_BUS_STUCK_TICKS)
    {
        status |= DIAG_I2C_SCL_STUCK;
    }
    if (s_sdaLowTicks >= PD_BUS_STUCK_TICKS)
    {
        status |= DIAG_I2C_SDA_STUCK;
    }

    for (i = 0u; i < (uint8)PERIPH_DIAG_COUNT; i++)
    {
        PeriphDiag_State      *st = &s_periph[i];
        const PeriphDiag_Bits *bt = &s_bits[i];

        /* An unfitted peripheral contributes no bits. Its counters are left
         * alone rather than reset, so a device declared unfitted after it has
         * been running keeps its history for inspection. */
        if (st->fitted != FALSE)
        {
            cond = FALSE;
            if (st->sawOk == FALSE) { cond = TRUE; }
            pd_track(&st->silentTicks, cond);
            pd_track(&st->implausibleTicks, st->sawImplausible);

            if (st->everOk == FALSE)
            {
                /* Never answered since boot. Reported on its own rather than as
                 * a timeout: this is a wiring or addressing problem, not a
                 * device that failed in service. */
                status |= bt->noResponse;
            }
            else if (st->silentTicks >= PD_TIMEOUT_TICKS)
            {
                status |= bt->timeout;
            }
            else
            {
                /* communicating */
            }

            if (st->frozenTicks >= PD_STUCK_TICKS)
            {
                status |= bt->stuck;
            }
            if (st->implausibleTicks >= PD_IMPLAUSIBLE_TICKS)
            {
                status |= bt->implausible;
            }

            /* Consume the per-window flags; the counters carry the history. */
            st->sawOk          = FALSE;
            st->sawImplausible = FALSE;
        }
    }

    return status;
}
