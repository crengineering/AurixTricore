/**********************************************************************************************************************
 * \file CtrlLatch.c
 * \brief Controller-output handoff CPU1 -> CPU0 -- see CtrlLatch.h.
 *
 * g_ctrlLatch itself is defined in SharedRam.c.
 *********************************************************************************************************************/
#include "CtrlLatch.h"
#include "SharedRam.h"

#define CTRLLATCH_RPM_MAX   (65535.0f)

void CtrlLatch_publish(const float32 rpm[4], const float32 tau[3],
                       const float32 omSoll[3], const float32 tauI[3],
                       const float32 phiSoll[3], uint32 status)
{
    uint8 i;

    g_ctrlLatch.gen = g_ctrlLatch.gen + 1u;          /* odd: writer inside */
    /* cppcheck-suppress misra-c2012-17.3 ; deviation: Ifx__dsync() wraps
     * TASKING's __dsync() intrinsic (see SharedRam.h), same as NavState.c */
    Ifx__dsync();

    for (i = 0u; i < 4u; i++)
    {
        g_ctrlLatch.rpm[i] = rpm[i];
    }
    for (i = 0u; i < 3u; i++)
    {
        g_ctrlLatch.tau[i]    = tau[i];
        g_ctrlLatch.omSoll[i] = omSoll[i];
        g_ctrlLatch.tauI[i]    = tauI[i];
        g_ctrlLatch.phiSoll[i] = phiSoll[i];
    }
    g_ctrlLatch.status = status;

    /* cppcheck-suppress misra-c2012-17.3 ; deviation: see above. */
    Ifx__dsync();
    g_ctrlLatch.gen = g_ctrlLatch.gen + 1u;          /* even: stable */
}

boolean CtrlLatch_get(CtrlLatch_t *out)
{
    uint8   attempt;
    boolean ok = FALSE;

    for (attempt = 0u; (attempt < 2u) && (ok == FALSE); attempt++)
    {
        const uint32 genBefore = g_ctrlLatch.gen;
        uint8        i;

        for (i = 0u; i < 4u; i++)
        {
            out->rpm[i] = g_ctrlLatch.rpm[i];
        }
        for (i = 0u; i < 3u; i++)
        {
            out->tau[i]    = g_ctrlLatch.tau[i];
            out->omSoll[i] = g_ctrlLatch.omSoll[i];
            out->tauI[i]    = g_ctrlLatch.tauI[i];
            out->phiSoll[i] = g_ctrlLatch.phiSoll[i];
        }
        out->status = g_ctrlLatch.status;
        out->pad[0] = 0u;
        out->pad[1] = 0u;

        if (((genBefore & 1u) == 0u) && (g_ctrlLatch.gen == genBefore))
        {
            out->gen = genBefore;
            ok = TRUE;
        }
    }
    return ok;
}

boolean CtrlLatch_request(uint16 rpm[4], CtrlLatch_t *snap)
{
    static CtrlLatch_t s_last;                              /* last consistent snapshot      */
    static uint32      s_staleCalls = CTRLLATCH_STALE_CALLS; /* boot: nothing published yet   */
    CtrlLatch_t        now;
    boolean            usable = FALSE;
    uint8              i;

    if ((CtrlLatch_get(&now) != FALSE) && (now.gen != s_last.gen))
    {
        s_last       = now;
        s_staleCalls = 0u;
    }
    else if (s_staleCalls < CTRLLATCH_STALE_CALLS)
    {
        s_staleCalls++;
    }
    else
    {
        /* stays at the limit */
    }

    if ((s_staleCalls < CTRLLATCH_STALE_CALLS)
        && ((s_last.status & (CTRLLATCH_ST_NAV_INVALID | CTRLLATCH_ST_TILT)) == 0u))
    {
        usable = TRUE;
    }

    for (i = 0u; i < 4u; i++)
    {
        const float32 r = s_last.rpm[i];

        /* written so that a NaN fails both comparisons and ends up 0 */
        if ((usable != FALSE) && (r >= 0.5f) && (r <= CTRLLATCH_RPM_MAX))
        {
            const float32 rounded = r + 0.5f;

            rpm[i] = (uint16)rounded;
        }
        else if ((usable != FALSE) && (r > CTRLLATCH_RPM_MAX))
        {
            rpm[i] = 65535u;
        }
        else
        {
            rpm[i] = 0u;
        }
    }

    *snap = s_last;
    if (s_staleCalls >= CTRLLATCH_STALE_CALLS)
    {
        snap->status |= CTRLLATCH_ST_STALE;
    }
    return usable;
}
