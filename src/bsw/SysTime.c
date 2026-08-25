/* SysTime.c — see SysTime.h */

#include "SysTime.h"
#include "IfxStm.h"

uint32_t SysTime_getTicks(void)
{
    return (uint32_t)IfxStm_getLower(&MODULE_STM0);
}

float32 SysTime_getTimeElapsedS(uint32 *lastTicks){
    uint32 now = SysTime_getTicks();
    uint32 elapsed = now - *lastTicks;
    *lastTicks = now;

    return (float) (elapsed * 1e-08f);
}
