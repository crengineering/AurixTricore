/**********************************************************************************************************************
 * \file Housekeeping.c
 * \brief 100 Hz off-chain bookkeeping — see Housekeeping.h.
 *********************************************************************************************************************/
#include "Housekeeping.h"
#include "Measurements.h"
#include "Diagnostics.h"
#include "gpio.h"
#include "NavState.h"

/* T11: the last snapshot NavState_get() successfully returned. Kept here,
 * not in NavState.c, because "what to do when a read fails" is a policy
 * decision for the one reader, not part of the publish/get protocol itself
 * -- NavState_get() never writes shared state, but it is allowed to write
 * into a caller-owned *out even on a torn attempt, so this file is what
 * actually enforces "keep the previous snapshot on FALSE". */
static NavState_t s_lastNav;

void Housekeeping_init(void)
{
    NavState_t zero = { 0 };
    s_lastNav = zero;
}

void Housekeeping_100ms(void)
{
    NavState_t candidate;

    if (NavState_get(&candidate) != FALSE)
    {
        s_lastNav = candidate;
    }
    /* else: torn read (or, before NavTask's first publish, none yet) --
     * keep s_lastNav exactly as it was; see Housekeeping.h. */

    /* imuPresent is uint32 in NavState_t (LMU no-sub-word rule, SharedRam.h)
     * but boolean in measurementsSetFusion(); compare explicitly rather than
     * pass the uint32 through an implicit narrowing conversion (MISRA
     * 10.3/10.8 -- same fix as the coreLoadPmil/coreAlive copy in
     * Measurements.c). */
    measurementsSetFusion(&s_lastNav.fusion, &s_lastNav.ahrs,
                           (s_lastNav.imuPresent != 0u) ? TRUE : FALSE,
                           s_lastNav.dtS);

    measurementsUpdate();
    if (diagnosticsUpdate() != FALSE)
    {
        gpio_write(GPIO_P_00_0, GPIO_STATE_ON);
    }
    else
    {
        gpio_write(GPIO_P_00_0, GPIO_STATE_OFF);
    }
    gpio_calApply();        /* XCP overrides win over the diagnostics write above */

    measurementsSetSystemLoad();   /* per-core exec time + Ethernet utilisation */
}
