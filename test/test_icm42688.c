#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/Icm42688.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

void test_nominal_plausible(void)
{
    /* ~1 g on Z, at rest. */
    Icm42688_Sample s = { { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, 20.0f };
    float32 liveness = 0.0f;
    TEST_ASSERT_TRUE(Icm42688_plausible(&s, &liveness));
    TEST_ASSERT_EQUAL_FLOAT(21.0f, liveness);
}

void test_accel_band_edges(void)
{
    float32 liveness = 0.0f;
    /* |a|^2 comfortably below 0.0025 (0.03 g) -> excluded. Not tested bit-exact
     * on the boundary: 0.05f*0.05f is not guaranteed to round to exactly
     * 0.0025f, which would make the test assert on float rounding rather than
     * on the band logic. */
    Icm42688_Sample lo = { { 0.03f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 20.0f };
    TEST_ASSERT_FALSE(Icm42688_plausible(&lo, &liveness));

    Icm42688_Sample loIn = { { 0.1f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 20.0f };
    TEST_ASSERT_TRUE(Icm42688_plausible(&loIn, &liveness));

    /* |a|^2 comfortably above 289 (17.5 g) -> excluded; comfortably below (16 g)
     * -> included. */
    Icm42688_Sample hi = { { 17.5f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 20.0f };
    TEST_ASSERT_FALSE(Icm42688_plausible(&hi, &liveness));

    Icm42688_Sample hiIn = { { 16.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 20.0f };
    TEST_ASSERT_TRUE(Icm42688_plausible(&hiIn, &liveness));
}

void test_temperature_band_edges(void)
{
    float32 liveness = 0.0f;
    Icm42688_Sample cold = { { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, -40.0f };
    TEST_ASSERT_FALSE(Icm42688_plausible(&cold, &liveness));

    Icm42688_Sample hot = { { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, 105.0f };
    TEST_ASSERT_FALSE(Icm42688_plausible(&hot, &liveness));
}

/* NaN on any axis, in either sensor, must fail the band -- not pass it. */
void test_nan_is_not_plausible(void)
{
    Icm42688_Sample s = { { NAN, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, 20.0f };
    float32 liveness = 0.0f;
    TEST_ASSERT_FALSE(Icm42688_plausible(&s, &liveness));
    TEST_ASSERT_TRUE(isnan(liveness));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nominal_plausible);
    RUN_TEST(test_accel_band_edges);
    RUN_TEST(test_temperature_band_edges);
    RUN_TEST(test_nan_is_not_plausible);
    return UNITY_END();
}
