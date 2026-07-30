/*
 *  Code from VibeCoding96
 *  Last modified: 12.06.2026
 *  Last Modified by: VibeCoding96
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "scheduler.h"
#include "CoreStats.h"

/* STM runs at 100 MHz, so 100 ticks = 1 us. */
#define SCHED_TICKS_PER_US   (100u)

/******************************************************************************/
/*--------------------------Function Implementations--------------------------*/
/******************************************************************************/

void Scheduler_init(Scheduler_t *sched, Ifx_STM *stm, uint8 coreId)
{
    sched->stm         = stm;
    sched->taskCount   = 0u;
    sched->coreId      = coreId;
    sched->busyTicks   = 0u;
    sched->maxTicks    = 0u;
    sched->windowStart = IfxStm_getLower(stm);
    CoreStats_init(coreId);
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
    uint32 elapsed;

    for (i = 0u; i < sched->taskCount; i++)
    {
        /* Unsigned subtraction handles 32-bit counter wraparound correctly */
        if ((uint32)(now - sched->tasks[i].lastRun) >= sched->tasks[i].period)
        {
            uint32 taskStart;
            uint32 taskTicks;

            sched->tasks[i].lastRun = now;

            taskStart = IfxStm_getLower(sched->stm);
            sched->tasks[i].fn();
            taskTicks = IfxStm_getLower(sched->stm) - taskStart;

            sched->busyTicks += taskTicks;
            if (taskTicks > sched->maxTicks)
            {
                sched->maxTicks = taskTicks;   /* worst-case dispatch, for headroom */
            }
        }
    }

    /* Close the accounting window and publish. */
    elapsed = IfxStm_getLower(sched->stm) - sched->windowStart;
    if (elapsed >= SCHED_STATS_WINDOW)
    {
        uint8 id = sched->coreId;

        if (id < CORESTATS_NUM_CORES)
        {
            g_coreStats[id].execUs    = sched->busyTicks / SCHED_TICKS_PER_US;
            g_coreStats[id].execMaxUs = sched->maxTicks  / SCHED_TICKS_PER_US;
            /* Per mille of the window actually spent in tasks. Scale first in
             * 64-bit-free form: busyTicks stays far below 2^32/1000 here. */
            g_coreStats[id].loadPmil  = (uint16)((sched->busyTicks / (elapsed / 1000u)));
            g_coreStats[id].aliveCounter++;
        }
        sched->busyTicks   = 0u;
        sched->windowStart = IfxStm_getLower(sched->stm);
    }
}
