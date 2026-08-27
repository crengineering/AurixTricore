#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/Bmp581.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

void test_nominal_plausible(void)
{
    float32 liveness = 0.0f;
    TEST_ASSERT_TRUE(Bmp581_plausible(95000.0f, 20.0f, &liveness));
    TEST_ASSERT_EQUAL_FLOAT(95020.0f, liveness);
}

void test_pressure_band_edges(void)
{
    float32 liveness = 0.0f;
    TEST_ASSERT_FALSE(Bmp581_plausible(30000.0f, 20.0f, &liveness));   /* == lo, excluded */
    TEST_ASSERT_TRUE(Bmp581_plausible(30001.0f, 20.0f, &liveness));
    TEST_ASSERT_FALSE(Bmp581_plausible(120000.0f, 20.0f, &liveness));  /* == hi, excluded */
    TEST_ASSERT_TRUE(Bmp581_plausible(119999.0f, 20.0f, &liveness));
}

void test_temperature_band_edges(void)
{
    float32 liveness = 0.0f;
    TEST_ASSERT_FALSE(Bmp581_plausible(95000.0f, -40.0f, &liveness));
    TEST_ASSERT_TRUE(Bmp581_plausible(95000.0f, -39.9f, &liveness));
    TEST_ASSERT_FALSE(Bmp581_plausible(95000.0f, 85.0f, &liveness));
    TEST_ASSERT_TRUE(Bmp581_plausible(95000.0f, 84.9f, &liveness));
}

/* NaN fails every comparison -- both bands are written as a positive
 * `> lo && < hi` test, so a NaN reading is rejected rather than slipping
 * through, and liveness still comes out NaN (visibly stuck, not silently
 * plausible). */
void test_nan_is_not_plausible(void)
{
    float32 liveness = 0.0f;
    TEST_ASSERT_FALSE(Bmp581_plausible(NAN, 20.0f, &liveness));
    TEST_ASSERT_TRUE(isnan(liveness));

    liveness = 0.0f;
    TEST_ASSERT_FALSE(Bmp581_plausible(95000.0f, NAN, &liveness));
    TEST_ASSERT_TRUE(isnan(liveness));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nominal_plausible);
    RUN_TEST(test_pressure_band_edges);
    RUN_TEST(test_temperature_band_edges);
    RUN_TEST(test_nan_is_not_plausible);
    return UNITY_END();
}
