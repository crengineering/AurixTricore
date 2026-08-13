#include "unity.h"

void setUp(void)
{
  /* This is run before EACH TEST */
}

void tearDown(void)
{
    /* This is run after test*/
}

void test_check_harness(void)
{
  /* All of these should pass */
  TEST_ASSERT_EQUAL(0, 0);
  TEST_ASSERT_EQUAL(1, 1);
  TEST_ASSERT_EQUAL(2, 2);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_check_harness);
    return UNITY_END();
}