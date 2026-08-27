/**********************************************************************************************************************
 * \file Atmosphere.c
 * \brief ISA barometric altitude formula — see Atmosphere.h.
 *********************************************************************************************************************/
#include "Atmosphere.h"
#include <math.h>

#define ATMOSPHERE_SEA_LEVEL_PA   (101325.0f)
#define ATMOSPHERE_SEA_LEVEL_MIN  (80000.0f)        /* ~2 km above sea level QNH floor  */
#define ATMOSPHERE_SEA_LEVEL_MAX  (120000.0f)       /* well above any real QNH          */
#define ATMOSPHERE_ALT_SCALE      (44330.0f)        /* [m] */
#define ATMOSPHERE_ALT_EXPONENT   (0.190294957f)    /* 1 / 5.25588 (barometric formula) */

float32 Atmosphere_altitudeM(float32 pressPa, float32 seaLevelPa)
{
    float32 p0 = seaLevelPa;
    boolean p0InBand;

    /* Positive test on both sides, project idiom (see fusion.c
     * fusion_usable()): every comparison against NaN is FALSE, so this
     * rejects a non-finite seaLevelPa the same way it rejects one that is
     * merely out of range — the original `< MIN || > MAX` form let a NaN
     * NVM value slip through unclamped. */
    p0InBand = ((p0 > ATMOSPHERE_SEA_LEVEL_MIN) && (p0 < ATMOSPHERE_SEA_LEVEL_MAX)) ? TRUE : FALSE;
    if (p0InBand == FALSE)
    {
        p0 = ATMOSPHERE_SEA_LEVEL_PA;
    }

    return ATMOSPHERE_ALT_SCALE * (1.0f - powf(pressPa / p0, ATMOSPHERE_ALT_EXPONENT));
}
