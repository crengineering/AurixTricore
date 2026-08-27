/* SysTime.c — see SysTime.h */

#include "SysTime.h"
#include "IfxStm.h"

uint32_t SysTime_getTicks(void)
{
    /* MODULE_STM0, not the caller's own STM -- see the file header for why
     * this is a deliberate exception, not a leftover from before NavTask_step
     * moved to CPU1 (T12). */
    return (uint32_t)IfxStm_getLower(&MODULE_STM0);
}

float SysTime_getTimeElapsedS(uint32_t *lastTicks){
    uint32_t now = SysTime_getTicks();
    uint32_t elapsed = now - *lastTicks;
    *lastTicks = now;

    return (float) (elapsed * 1e-08f);
}
