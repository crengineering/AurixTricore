/* TASKING-Spracherweiterungen, die GCC nicht kennt.
 *
 * Wird per -include vor jede Estimator-Quelldatei gezogen (siehe
 * CMakeLists.txt). Nur was zum Uebersetzen noetig ist -- kein Verhalten.
 *
 *   __at(addr)     pins an object at an absolute address. On the host there
 *                  is no fixed memory map, so the object simply lands in
 *                  .bss; every test reaches it through its extern
 *                  declaration anyway.
 *   Ifx__dsync()   TriCore data-synchronisation barrier (SharedRam.h). The
 *                  host has one core and no store buffer to order, so this
 *                  is a no-op -- NavState's host test covers the
 *                  publish/get PROTOCOL (ordering of the writes, the
 *                  torn-read retry, the FALSE path), never the barrier
 *                  itself. That is a hardware/.src check
 *                  (docs/REFACTORING_PLAN.md §4, Risk 2), done on target.
 */
#ifndef TASKING_SHIM_H
#define TASKING_SHIM_H

#define __at(a)
#define Ifx__dsync()

#endif /* TASKING_SHIM_H */
