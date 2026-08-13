#ifndef IFXSTM_H
#define IFXSTM_H

#include "Ifx_Types.h"

/* scheduler.c reicht den Zeiger nur durch und schaut nie hinein,
 * also genuegt ein Platzhalter statt der echten Registerstruktur. */
typedef struct { int dummy; } Ifx_STM;

uint32 IfxStm_getLower(Ifx_STM *stm);

/* Steuerung der Fake-Uhr - existiert nur im Host-Build. */
void   FakeStm_setTicks(uint32 ticks);
void   FakeStm_advance(uint32 delta);

#endif /* IFXSTM_H */