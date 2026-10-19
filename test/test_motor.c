#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/motor.c"      /* #included: Motor_task is public, but the TU
                                    * needs the Xcp/Diagnostics symbols stubbed below */

/* SWE1-FW-025 (ceiling) and SWE1-FW-019 (arming state machine) through the
 * public Motor_task(). Diagnostics.c / Xcp.c are target code; only what
 * motor.c touches is stubbed here. */
volatile Xcp_Cal g_xcpCal;
static boolean s_linkAlive = TRUE;

boolean Xcp_linkAlive(void)
{
    return s_linkAlive;
}

static Motor_states_t s_state;
static const uint16   s_zero[DSHOT_MEND] = {0u, 0u, 0u, 0u};

void setUp(void)
{
    s_linkAlive = TRUE;
    s_state     = MOTOR_DISARMED;
    g_xcpCal.motorCmd       = 0u;
    g_xcpCal.motorManual    = 0u;
    g_xcpCal.motorSpMax     = 1600u;
    for (uint32 i = 0u; i < DSHOT_MEND; i++) { g_xcpCal.motorManualSp[i] = 0u; }
}

void tearDown(void)
{
}

/* One Motor_task call with a zeroed command array (the caller zero-inits it). */
static void step(uint16 cmd[DSHOT_MEND])
{
    for (uint32 i = 0u; i < DSHOT_MEND; i++) { cmd[i] = 0u; }
    (void)Motor_task(s_zero, cmd, &s_state);
}

/* DISARMED -> INIT (beacons) -> ARMED, bounded. */
static void arm(void)
{
    uint16 cmd[DSHOT_MEND];

    g_xcpCal.motorCmd = MOTOR_CMD_ARM;
    for (uint32 n = 0u; (n < 10000u) && (s_state != MOTOR_ARMED); n++)
    {
        step(cmd);
    }
    TEST_ASSERT_EQUAL_INT(MOTOR_ARMED, s_state);
}

static uint16 clampM1(uint16 sp)
{
    uint16 cmd[DSHOT_MEND];

    g_xcpCal.motorManualSp[0] = sp;
    step(cmd);
    return cmd[0];
}

void test_arming_sequence_with_beacons(void)
{
    uint16 cmd[DSHOT_MEND];
    uint32 beacons[DSHOT_MEND] = {0u, 0u, 0u, 0u};

    step(cmd);
    TEST_ASSERT_EQUAL_INT(MOTOR_DISARMED, s_state);
    g_xcpCal.motorCmd = MOTOR_CMD_ARM;
    step(cmd);
    TEST_ASSERT_EQUAL_INT(MOTOR_INIT, s_state);
    for (uint32 n = 0u; (n < 10000u) && (s_state != MOTOR_ARMED); n++)
    {
        step(cmd);
        for (uint32 i = 0u; i < DSHOT_MEND; i++)
        {
            if (cmd[i] == 1u) { beacons[i]++; }
        }
    }
    TEST_ASSERT_EQUAL_INT(MOTOR_ARMED, s_state);
    for (uint32 i = 0u; i < DSHOT_MEND; i++) { TEST_ASSERT_EQUAL_UINT32(1u, beacons[i]); }
}

void test_ceiling_sweep(void)
{
    static const uint16 ceilings[] = {0u, 47u, 48u, 1600u, 2047u, 2048u, 65535u};
    uint16 eff = 1600u;                 /* last valid ceiling */

    arm();
    g_xcpCal.motorManual = MOTOR_CMD_MANUAL;
    (void)clampM1(1u);                  /* adopt 1600 from setUp */
    for (uint32 c = 0u; c < 7u; c++)
    {
        if ((ceilings[c] >= 48u) && (ceilings[c] <= 2047u)) { eff = ceilings[c]; }
        uint32 hi;
        uint16 prev = 0u;

        hi = (eff > 48u) ? eff : 48u;
        g_xcpCal.motorSpMax = ceilings[c];
        for (uint32 sp = 0u; sp <= 65535u; sp++)
        {
            const uint16 r = clampM1((uint16)sp);

            if (sp == 0u)
            {
                TEST_ASSERT_EQUAL_UINT16(0u, r);
            }
            else
            {
                TEST_ASSERT_TRUE(r >= 48u);
                TEST_ASSERT_TRUE(r <= hi);
            }
            TEST_ASSERT_TRUE(r >= prev);
            prev = r;
        }
        TEST_ASSERT_EQUAL_UINT16(eff, g_xcpCal.motorSpMax);   /* read-back = value in force */
    }
}

void test_spot_values(void)
{
    arm();
    g_xcpCal.motorManual = MOTOR_CMD_MANUAL;
    TEST_ASSERT_EQUAL_UINT16(1599u, clampM1(1599u));
    TEST_ASSERT_EQUAL_UINT16(1600u, clampM1(1600u));
    TEST_ASSERT_EQUAL_UINT16(1600u, clampM1(1601u));
    TEST_ASSERT_EQUAL_UINT16(1600u, clampM1(2047u));
    TEST_ASSERT_EQUAL_UINT16(48u,   clampM1(47u));
    g_xcpCal.motorSpMax = 2047u;
    TEST_ASSERT_EQUAL_UINT16(2047u, clampM1(2047u));
    TEST_ASSERT_EQUAL_UINT16(2047u, clampM1(2048u));
}

void test_ceiling_validation_reverts_to_last_valid(void)
{
    static const uint16 bad[]  = {0u, 47u, 2048u, 65535u};
    static const uint16 good[] = {48u, 1600u, 2047u};
    static const uint16 base[] = {1600u, 2047u};
    uint16 cmd[DSHOT_MEND];

    for (uint32 b = 0u; b < 2u; b++)
    {
        g_xcpCal.motorSpMax = base[b];
        step(cmd);                                           /* adopt the valid value */
        for (uint32 i = 0u; i < 4u; i++)
        {
            g_xcpCal.motorSpMax = bad[i];
            step(cmd);
            TEST_ASSERT_EQUAL_UINT16(base[b], g_xcpCal.motorSpMax);
        }
    }
    for (uint32 i = 0u; i < 3u; i++)
    {
        g_xcpCal.motorSpMax = good[i];
        step(cmd);
        TEST_ASSERT_EQUAL_UINT16(good[i], g_xcpCal.motorSpMax);
    }
}

/* Must run first: needs the file-static s_ceiling of motor.c untouched, i.e. no valid
 * motorSpMax adopted yet (setUp's 1600 is overridden before the first Motor_task call). */
void test_fresh_ceiling_is_protocol_floor(void)
{
    g_xcpCal.motorSpMax = 0u;
    arm();
    g_xcpCal.motorManual = MOTOR_CMD_MANUAL;
    TEST_ASSERT_EQUAL_UINT16(48u, clampM1(2047u));
    TEST_ASSERT_EQUAL_UINT16(48u, g_xcpCal.motorSpMax);      /* read-back = value in force */
}

void test_manual_sp_not_rewritten_and_all_motors_same_ceiling(void)
{
    uint16 cmd[DSHOT_MEND];

    arm();
    g_xcpCal.motorManual = MOTOR_CMD_MANUAL;
    for (uint32 i = 0u; i < DSHOT_MEND; i++) { g_xcpCal.motorManualSp[i] = 2047u; }
    step(cmd);
    for (uint32 i = 0u; i < DSHOT_MEND; i++)
    {
        TEST_ASSERT_EQUAL_UINT16(1600u, cmd[i]);
        TEST_ASSERT_EQUAL_UINT16(2047u, g_xcpCal.motorManualSp[i]);
    }
}

void test_disarm_zeroes_everything(void)
{
    uint16 cmd[DSHOT_MEND];

    arm();
    g_xcpCal.motorManual = MOTOR_CMD_MANUAL;
    for (uint32 i = 0u; i < DSHOT_MEND; i++) { g_xcpCal.motorManualSp[i] = 1000u; }
    step(cmd);
    TEST_ASSERT_EQUAL_UINT16(1000u, cmd[0]);
    g_xcpCal.motorCmd = 0u;
    step(cmd);
    TEST_ASSERT_EQUAL_INT(MOTOR_DISARMED, s_state);
    TEST_ASSERT_EQUAL_UINT32(0u, g_xcpCal.motorManual);
    for (uint32 i = 0u; i < DSHOT_MEND; i++)
    {
        TEST_ASSERT_EQUAL_UINT16(0u, cmd[i]);
        TEST_ASSERT_EQUAL_UINT16(0u, g_xcpCal.motorManualSp[i]);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_fresh_ceiling_is_protocol_floor);      /* first: needs the untouched static */
    RUN_TEST(test_arming_sequence_with_beacons);
    RUN_TEST(test_ceiling_sweep);
    RUN_TEST(test_spot_values);
    RUN_TEST(test_ceiling_validation_reverts_to_last_valid);
    RUN_TEST(test_manual_sp_not_rewritten_and_all_motors_same_ceiling);
    RUN_TEST(test_disarm_zeroes_everything);
    return UNITY_END();
}
