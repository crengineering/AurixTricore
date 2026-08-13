#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "fakes/IfxStm.h"
#include "../src/bsw/scheduler.h"

static Scheduler_t g_sched;

/*
dummy tasks A, B, C, D, E, F, G, H & I
*/
static uint32 s_taskACalls;
static uint32 s_taskBCalls;
static uint32 s_taskCCalls;
static uint32 s_taskDCalls;
static uint32 s_taskECalls;
static uint32 s_taskFCalls;
static uint32 s_taskGCalls;
static uint32 s_taskHCalls;
static uint32 s_taskICalls;

static void taskA(void) { s_taskACalls++; }
static void taskB(void) { s_taskBCalls++; }
static void taskC(void) { s_taskCCalls++; }
static void taskD(void) { s_taskDCalls++; }
static void taskE(void) { s_taskECalls++; }
static void taskF(void) { s_taskFCalls++; }
static void taskG(void) { s_taskGCalls++; }
static void taskH(void) { s_taskHCalls++; }
static void taskI(void) { s_taskICalls++; }

void setUp(void)
{
  /* This is run before EACH TEST */
  //Scheduler_init(&g_sched, &MODULE_STM0, 0u);
}

void tearDown(void)
{
    /* This is run after test*/
}


/* 
adding more than 8 Tasks
*/
void test_add_tasks(void)
{
    /* first 8 tasks*/
    TEST_ASSERT_EQUAL(TRUE,  Scheduler_addTask(&g_sched, taskA, SCHED_MS(500u)));
    TEST_ASSERT_EQUAL(TRUE,  Scheduler_addTask(&g_sched, taskB, SCHED_MS(500u)));
    TEST_ASSERT_EQUAL(TRUE,  Scheduler_addTask(&g_sched, taskC, SCHED_MS(500u)));
    TEST_ASSERT_EQUAL(TRUE,  Scheduler_addTask(&g_sched, taskD, SCHED_MS(500u)));
    TEST_ASSERT_EQUAL(TRUE,  Scheduler_addTask(&g_sched, taskE, SCHED_MS(500u)));
    TEST_ASSERT_EQUAL(TRUE,  Scheduler_addTask(&g_sched, taskF, SCHED_MS(500u)));
    TEST_ASSERT_EQUAL(TRUE,  Scheduler_addTask(&g_sched, taskG, SCHED_MS(500u)));
    TEST_ASSERT_EQUAL(TRUE,  Scheduler_addTask(&g_sched, taskH, SCHED_MS(500u)));

    /* task 9 should not be allowed*/
    TEST_ASSERT_EQUAL(FALSE, Scheduler_addTask(&g_sched, taskI, SCHED_MS(500u)));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_add_tasks);
    return UNITY_END();
}