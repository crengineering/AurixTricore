#include "IfxStm.h"

static uint32 s_ticks;

uint32 IfxStm_getLower(Ifx_STM *stm)
{
    (void)stm;          /* Parameter ungenutzt - beruhigt -Wextra */
    return s_ticks;
}

void FakeStm_setTicks(uint32 ticks) { s_ticks = ticks; }
void FakeStm_advance(uint32 delta)  { s_ticks += delta; }