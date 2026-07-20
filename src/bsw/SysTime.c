/* SysTime.c — see SysTime.h */

#include "SysTime.h"
#include "IfxStm.h"

uint32_t SysTime_getTicks(void)
{
    return (uint32_t)IfxStm_getLower(&MODULE_STM0);
}
