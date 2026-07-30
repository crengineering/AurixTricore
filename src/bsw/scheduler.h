/*
 *  Code from VibeCoding96
 *  Last modified: 12.06.2026
 *  Last Modified by: VibeCoding96
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "Ifx_Types.h"
#include "IfxStm.h"

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

/* STM runs at 100 MHz after PLL init: 1 ms = 100 000 ticks */
#define SCHED_MS(ms)   ((uint32)((ms) * 100000UL))
#define SCHED_US(us)   ((uint32)((us) * 100UL))

#ifndef SCHEDULER_MAX_TASKS
#define SCHEDULER_MAX_TASKS  8u
#endif

typedef void (*Scheduler_TaskFn_t)(void);

typedef struct
{
    Scheduler_TaskFn_t  fn;
    uint32              period;    /**< Period in STM ticks            */
    uint32              lastRun;   /**< STM tick value at last dispatch */
} Scheduler_Task_t;

/* Execution-time accounting window. 100 ms is long enough to average out the
 * 500 ms LED task appearing in only some windows, short enough to react. */
#define SCHED_STATS_WINDOW   SCHED_MS(100u)

typedef struct
{
    Ifx_STM          *stm;
    uint8             taskCount;
    uint8             coreId;      /**< index into g_coreStats                */
    uint32            busyTicks;   /**< accumulated task time this window     */
    uint32            maxTicks;    /**< longest single dispatch since boot    */
    uint32            windowStart; /**< STM tick at the start of the window   */
    Scheduler_Task_t  tasks[SCHEDULER_MAX_TASKS];
} Scheduler_t;

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

/** \param coreId which core this scheduler runs on (0..5); its execution time
 *                is published to g_coreStats[coreId]. */
void    Scheduler_init(Scheduler_t *sched, Ifx_STM *stm, uint8 coreId);
boolean Scheduler_addTask(Scheduler_t *sched, Scheduler_TaskFn_t fn, uint32 period);
void    Scheduler_run(Scheduler_t *sched);

#endif /* SCHEDULER_H */
