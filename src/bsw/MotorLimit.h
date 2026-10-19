#ifndef MOTORLIMIT_H_
#define MOTORLIMIT_H_

#include "Ifx_Types.h"

/* DShot throttle limiting, pure functions (no hardware, no globals) so the host
 * tests can reach them. Only the protocol range lives here; the ceiling value
 * itself is an aircraft calibration (Xcp_Cal.motorSpMax, default in
 * Diagnostics.c). SWE1-FW-022, SWE1-FW-025. */
#define MOTORLIMIT_THROTTLE_MIN    48u      /* DShot 0 % throttle            */
#define MOTORLIMIT_THROTTLE_MAX  2047u      /* DShot 11-bit top              */

/* calSpMax if 48 <= calSpMax <= 2047, else fallback. */
uint16 MotorLimit_ceiling(uint16 calSpMax, uint16 fallback);

/* sp == 0 -> 0 (stop). Else v = min(sp, ceiling), then max(v, 48), then
 * min(v, 2047): the protocol clamp is last, so the result is 0 or 48..2047
 * for ANY ceiling (no ESC command 1..47 and no out-of-range value on the wire). */
uint16 MotorLimit_setpoint(uint16 sp, uint16 ceiling);

#endif /* MOTORLIMIT_H_ */
