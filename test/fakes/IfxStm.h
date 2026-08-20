#ifndef IFXSTM_H
#define IFXSTM_H

#include "Ifx_Types.h"

/* scheduler.c reicht den Zeiger nur durch und schaut nie hinein,
 * also genuegt ein Platzhalter statt der echten Registerstruktur. */
typedef struct { int dummy; } Ifx_STM;

/* SysTime.c liest fest STM0, also muss das Modul hier existieren. */
extern Ifx_STM MODULE_STM0;

uint32 IfxStm_getLower(Ifx_STM *stm);

/* Steuerung der Fake-Uhr - existiert nur im Host-Build. */
void   FakeStm_setTicks(uint32 ticks);
void   FakeStm_advance(uint32 delta);

/* Laesst die Uhr bei JEDEM Lesen um delta weiterlaufen.
 *
 * Noetig fuer Code, der in einer Schleife auf eine Deadline wartet: mit einer
 * stehenden Uhr wird die Deadline nie erreicht und der Test haengt, statt zu
 * scheitern. 0 schaltet ab und ist der Normalfall - der Scheduler-Test steuert
 * die Zeit weiter selbst. FakeStm_reset setzt es zurueck. */
void   FakeStm_setAutoAdvance(uint32 delta);

/* Uhr und Auto-Advance in den Ausgangszustand. */
void   FakeStm_reset(void);

#endif /* IFXSTM_H */