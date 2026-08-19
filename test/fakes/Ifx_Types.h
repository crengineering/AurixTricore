#ifndef IFX_TYPES_H
#define IFX_TYPES_H

/* Host-Ersatz fuer die iLLD-Typen. Nur was der Testcode braucht.
 * Exakte Breiten aus stdint.h statt "unsigned long" wie im Original:
 * das ist auf Linux 64 Bit und wuerde die Tests verfaelschen. */

#include <stdint.h>

typedef uint8_t   uint8;
typedef uint16_t  uint16;
typedef uint32_t  uint32;
typedef int32_t   sint32;
typedef float     float32;

typedef unsigned char boolean;
#define TRUE  (1u)
#define FALSE (0u)

/* On target this comes from Compilers.h (via Platform_Types.h): it decorates
 * the following function as an interrupt handler and plants the vector-table
 * entry. On the host there is no vector table, so it degrades to a plain
 * prototype -- which is exactly what the GCC variant of the vendor macro does
 * when IFX_USE_SW_MANAGED_INT is off. The handler then stays an ordinary
 * function that a test can call directly to simulate "a byte arrived". */
#define IFX_INTERRUPT(isr, vectabNum, prio) void isr(void)

#endif /* IFX_TYPES_H */