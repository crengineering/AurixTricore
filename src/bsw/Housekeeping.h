/**********************************************************************************************************************
 * \file Housekeeping.h
 * \brief 100 Hz off-chain bookkeeping: measurements, diagnostics, GPIO,
 *        system load. Registered on CPU0.
 *
 * Everything here is allowed to be a cycle late without a flight
 * consequence — the opposite of NavTask. Split out of Cpu0_Main.c's
 * Task_Measure100ms (T7, docs/REFACTORING_PLAN.md); the GNSS read moved to
 * SensorTask in T6, and xcpDaqCycle()/Nvm_task100ms() became their own
 * scheduler tasks in the same step so a DFLASH save can no longer delay this.
 *********************************************************************************************************************/
#ifndef HOUSEKEEPING_H
#define HOUSEKEEPING_H

/** Zeroes the last-good NavState snapshot this file keeps between calls
 *  (T11, docs/REFACTORING_PLAN.md): NavState_get() can return FALSE on a
 *  torn read, and the contract is that the caller then keeps its own
 *  previous snapshot rather than publish a mix of two ticks -- this is
 *  where that snapshot lives. */
void Housekeeping_init(void);

/** NavState_get() + measurementsSetFusion() (T11 -- this task is now
 *  NavState's one reader) + measurementsUpdate() + diagnosticsUpdate()
 *  (+ the diagnostics GPIO) + gpio_calApply() + measurementsSetSystemLoad().
 *  100 ms (10 Hz). */
void Housekeeping_100ms(void);

#endif /* HOUSEKEEPING_H */
