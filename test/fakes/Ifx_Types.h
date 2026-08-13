#ifndef IFX_TYPES_H
#define IFX_TYPES_H

/* Host-Ersatz fuer die iLLD-Typen. Nur was der Testcode braucht.
 * Exakte Breiten aus stdint.h statt "unsigned long" wie im Original:
 * das ist auf Linux 64 Bit und wuerde die Tests verfaelschen. */

#include <stdint.h>

typedef uint8_t   uint8;
typedef uint16_t  uint16;
typedef uint32_t  uint32;

#endif /* IFX_TYPES_H */