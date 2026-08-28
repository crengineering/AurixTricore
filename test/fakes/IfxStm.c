#include "IfxStm.h"

Ifx_STM MODULE_STM0;

static uint32 s_ticks;
static uint32 s_autoAdvance;

uint32 IfxStm_getLower(Ifx_STM *stm)
{
    uint32 now = s_ticks;
    (void)stm;          /* Parameter ungenutzt - beruhigt -Wextra */

    /* Nach dem Lesen weiterstellen, damit eine Warteschleife Fortschritt
     * sieht. Der Aufrufer bekommt noch den alten Wert - so liefert der erste
     * Aufruf (der Startzeitpunkt) immer genau s_ticks. */
    s_ticks += s_autoAdvance;
    return now;
}

void IfxStm_waitTicks(Ifx_STM *stm, uint32 ticks)
{
    (void)stm;
    (void)ticks;    /* no-op: nothing under host test exercises the delay path */
}

void FakeStm_setTicks(uint32 ticks)       { s_ticks = ticks; }
void FakeStm_advance(uint32 delta)        { s_ticks += delta; }
void FakeStm_setAutoAdvance(uint32 delta) { s_autoAdvance = delta; }

void FakeStm_reset(void)
{
    s_ticks       = 0u;
    s_autoAdvance = 0u;
}
