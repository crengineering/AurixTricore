/*
 *  Code from VibeCoding96
 *  Last modified: 12.06.2026
 *  Last Modified by: VibeCoding96
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "scheduler.h"

/******************************************************************************/
/*--------------------------Function Implementations--------------------------*/
/******************************************************************************/

void Scheduler_init(Scheduler_t *sched, Ifx_STM *stm)
{
    sched->stm       = stm;
    sched->taskCount = 0u;
}

boolean Scheduler_addTask(Scheduler_t *sched, Scheduler_TaskFn_t fn, uint32 period)
{
    uint8 idx;

    if (sched->taskCount >= SCHEDULER_MAX_TASKS)
        return FALSE;

    idx = sched->taskCount;
    sched->tasks[idx].fn      = fn;
    sched->tasks[idx].period  = period;
    /* Seed lastRun to now so first dispatch fires exactly one period later */
    sched->tasks[idx].lastRun = IfxStm_getLower(sched->stm);
    sched->taskCount++;

    return TRUE;
}

void Scheduler_run(Scheduler_t *sched)
{
    uint8  i;
    uint32 now = IfxStm_getLower(sched->stm);

    for (i = 0u; i < sched->taskCount; i++)
    {
        /* Unsigned subtraction handles 32-bit counter wraparound correctly */
        if ((uint32)(now - sched->tasks[i].lastRun) >= sched->tasks[i].period)
        {
            sched->tasks[i].lastRun = now;
            sched->tasks[i].fn();
        }
    }
}
