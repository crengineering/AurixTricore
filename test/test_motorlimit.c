#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/MotorLimit.h"

void setUp(void)
{
}

void tearDown(void)
{
}

/* The fallback is the caller's aircraft default, not a module constant. */
void test_ceiling_out_of_range_gives_fallback(void)
{
    static const uint16 bad[] = {0u, 1u, 47u, 2048u, 65535u};
    static const uint16 fb[]  = {1600u, 1000u};

    for (uint32 f = 0u; f < 2u; f++)
    {
        for (uint32 i = 0u; i < 5u; i++)
        {
            TEST_ASSERT_EQUAL_UINT16(fb[f], MotorLimit_ceiling(bad[i], fb[f]));
        }
    }
}

void test_ceiling_in_range_passes_through(void)
{
    static const uint16 good[] = {48u, 1000u, 1600u, 2047u};
    static const uint16 fb[]   = {1600u, 1000u};

    for (uint32 f = 0u; f < 2u; f++)
    {
        for (uint32 i = 0u; i < 4u; i++)
        {
            TEST_ASSERT_EQUAL_UINT16(good[i], MotorLimit_ceiling(good[i], fb[f]));
        }
    }
}

void test_setpoint_exhaustive(void)
{
    static const uint16 ceilings[] = {0u, 1u, 47u, 48u, 1600u, 2047u, 2048u, 65535u};

    for (uint32 c = 0u; c < 8u; c++)
    {
        uint32 hi = (ceilings[c] > MOTORLIMIT_THROTTLE_MIN) ? ceilings[c] : MOTORLIMIT_THROTTLE_MIN;
        uint16 prev = 0u;

        if (hi > MOTORLIMIT_THROTTLE_MAX)
        {
            hi = MOTORLIMIT_THROTTLE_MAX;
        }
        for (uint32 sp = 0u; sp <= 65535u; sp++)
        {
            const uint16 r = MotorLimit_setpoint((uint16)sp, ceilings[c]);

            if (sp == 0u)
            {
                TEST_ASSERT_EQUAL_UINT16(0u, r);
            }
            else
            {
                TEST_ASSERT_TRUE(r >= MOTORLIMIT_THROTTLE_MIN);
                TEST_ASSERT_TRUE(r <= hi);
            }
            TEST_ASSERT_TRUE(r >= prev);        /* monotonic in sp */
            prev = r;
        }
    }
}

void test_setpoint_spot_ceiling_1600(void)
{
    TEST_ASSERT_EQUAL_UINT16(48u,   MotorLimit_setpoint(1u, 1600u));
    TEST_ASSERT_EQUAL_UINT16(48u,   MotorLimit_setpoint(47u, 1600u));
    TEST_ASSERT_EQUAL_UINT16(48u,   MotorLimit_setpoint(48u, 1600u));
    TEST_ASSERT_EQUAL_UINT16(1599u, MotorLimit_setpoint(1599u, 1600u));
    TEST_ASSERT_EQUAL_UINT16(1600u, MotorLimit_setpoint(1600u, 1600u));
    TEST_ASSERT_EQUAL_UINT16(1600u, MotorLimit_setpoint(1601u, 1600u));
    TEST_ASSERT_EQUAL_UINT16(1600u, MotorLimit_setpoint(2047u, 1600u));
    TEST_ASSERT_EQUAL_UINT16(1600u, MotorLimit_setpoint(65535u, 1600u));
}

void test_setpoint_spot_ceiling_2047(void)
{
    TEST_ASSERT_EQUAL_UINT16(1601u, MotorLimit_setpoint(1601u, 2047u));
    TEST_ASSERT_EQUAL_UINT16(2047u, MotorLimit_setpoint(2047u, 2047u));
    TEST_ASSERT_EQUAL_UINT16(2047u, MotorLimit_setpoint(2048u, 2047u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ceiling_out_of_range_gives_fallback);
    RUN_TEST(test_ceiling_in_range_passes_through);
    RUN_TEST(test_setpoint_exhaustive);
    RUN_TEST(test_setpoint_spot_ceiling_1600);
    RUN_TEST(test_setpoint_spot_ceiling_2047);
    return UNITY_END();
}
