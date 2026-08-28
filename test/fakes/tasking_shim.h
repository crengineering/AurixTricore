/* TASKING-Spracherweiterungen, die GCC nicht kennt.
 *
 * Wird per -include vor jede Estimator-Quelldatei gezogen (siehe
 * CMakeLists.txt). Nur was zum Uebersetzen noetig ist -- kein Verhalten.
 *
 *   Ifx__dsync()   TriCore data-synchronisation barrier (SharedRam.h). The
 *                  host has one core and no store buffer to order, so this
 *                  is a no-op -- NavState's host test covers the
 *                  publish/get PROTOCOL (ordering of the writes, the
 *                  torn-read retry, the FALSE path), never the barrier
 *                  itself. That is a hardware/.src check
 *                  (docs/REFACTORING_PLAN.md §4, Risk 2), done on target.
 *
 * __at(addr) used to need the same treatment (pinned an object at an
 * absolute address the host has no equivalent for). docs/MEMORY_PLACEMENT.md
 * T5 deleted the last __at() anywhere in this tree -- every placed object is
 * `#pragma section` now, which GCC just warns about and ignores
 * (-Wunknown-pragmas), no macro needed. Removed from here rather than left
 * as a dead no-op define.
 */
#ifndef TASKING_SHIM_H
#define TASKING_SHIM_H

#define Ifx__dsync()

#endif /* TASKING_SHIM_H */
