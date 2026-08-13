#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/CoreStats.h"

void setUp(void)
{
  /* This is run before EACH TEST */
}

void tearDown(void)
{
    /* This is run after test*/
}

/* 
testing if all cores are init correctful 
*/
void test_check_corestats_init(void)
{
  for(uint8 coreId=0; coreId < CORESTATS_NUM_CORES; coreId++)
  {
    CoreStats_init(coreId);
    TEST_ASSERT_EQUAL(0, g_coreStats[coreId].aliveCounter);
    TEST_ASSERT_EQUAL(0, g_coreStats[coreId].execMaxUs);
    TEST_ASSERT_EQUAL(0, g_coreStats[coreId].execUs);
    TEST_ASSERT_EQUAL(0, g_coreStats[coreId].loadPmil);
  }
}

/* 
testing if all cores reset init correctful 
*/
void test_check_corestats_reset_init(void)
{
  /* adding new values to core stats*/
  for (uint8 coreId = 0; coreId < CORESTATS_NUM_CORES; coreId++)
  {
    g_coreStats[coreId].execUs       = 0xAAAAAAAAu;
    g_coreStats[coreId].execMaxUs    = 0xBBBBBBBBu;
    g_coreStats[coreId].loadPmil     = 0xCCCCu;
    g_coreStats[coreId].aliveCounter = 0xDDDDu;
  }

  for(uint8 coreId=0; coreId < CORESTATS_NUM_CORES; coreId++)
  {
    CoreStats_init(coreId);
    TEST_ASSERT_EQUAL(0, g_coreStats[coreId].aliveCounter);
    TEST_ASSERT_EQUAL(0, g_coreStats[coreId].execMaxUs);
    TEST_ASSERT_EQUAL(0, g_coreStats[coreId].execUs);
    TEST_ASSERT_EQUAL(0, g_coreStats[coreId].loadPmil);
  }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_check_corestats_init);
    RUN_TEST(test_check_corestats_reset_init);
    return UNITY_END();
}