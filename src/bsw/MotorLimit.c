#include "MotorLimit.h"

uint16 MotorLimit_ceiling(uint16 calSpMax, uint16 fallback)
{
    uint16 result = fallback;

    if ((calSpMax >= MOTORLIMIT_THROTTLE_MIN) && (calSpMax <= MOTORLIMIT_THROTTLE_MAX))
    {
        result = calSpMax;
    }
    return result;
}

uint16 MotorLimit_setpoint(uint16 sp, uint16 ceiling)
{
    uint16 v = 0u;

    if (sp > 0u)
    {
        v = (sp < ceiling) ? sp : ceiling;
        if (v < MOTORLIMIT_THROTTLE_MIN)
        {
            v = MOTORLIMIT_THROTTLE_MIN;
        }
        if (v > MOTORLIMIT_THROTTLE_MAX)
        {
            v = MOTORLIMIT_THROTTLE_MAX;
        }
    }
    return v;
}
