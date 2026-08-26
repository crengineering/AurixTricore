/* TASKING-Spracherweiterungen, die GCC nicht kennt.
 *
 * Wird per -include vor jede Estimator-Quelldatei gezogen (siehe
 * CMakeLists.txt). Nur was zum Uebersetzen noetig ist -- kein Verhalten.
 *
 *   __at(addr)  pins an object at an absolute address. On the host there is no
 *               fixed memory map, so the object simply lands in .bss; every
 *               test reaches it through its extern declaration anyway.
 */
#ifndef TASKING_SHIM_H
#define TASKING_SHIM_H

#define __at(a)

#endif /* TASKING_SHIM_H */
