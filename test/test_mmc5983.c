#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/Mmc5983.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

void test_nominal_plausible(void)
{
    /* Munich-ish field, ~0.48 G on one axis. */
    Mmc5983_Sample s = { { 0.48f, 0.0f, 0.0f }, 0.0f };
    float32 liveness = 0.0f;
    TEST_ASSERT_TRUE(Mmc5983_plausible(&s, &liveness));
    TEST_ASSERT_EQUAL_FLOAT(0.48f, liveness);
}

void test_band_edges(void)
{
    float32 liveness = 0.0f;
    /* |B|^2 = 0.0225 exactly on the low edge -> excluded (positive test). */
    Mmc5983_Sample lo = { { 0.15f, 0.0f, 0.0f }, 0.0f };
    TEST_ASSERT_FALSE(Mmc5983_plausible(&lo, &liveness));

    Mmc5983_Sample loIn = { { 0.151f, 0.0f, 0.0f }, 0.0f };
    TEST_ASSERT_TRUE(Mmc5983_plausible(&loIn, &liveness));

    /* |B|^2 = 4.0 exactly on the high edge -> excluded. */
    Mmc5983_Sample hi = { { 2.0f, 0.0f, 0.0f }, 0.0f };
    TEST_ASSERT_FALSE(Mmc5983_plausible(&hi, &liveness));

    Mmc5983_Sample hiIn = { { 1.999f, 0.0f, 0.0f }, 0.0f };
    TEST_ASSERT_TRUE(Mmc5983_plausible(&hiIn, &liveness));
}

/* NaN on any axis makes fieldSq NaN, which fails both comparisons -- rejected
 * rather than accepted, and it shows up in liveness rather than vanishing. */
void test_nan_is_not_plausible(void)
{
    Mmc5983_Sample s = { { NAN, 0.2f, 0.1f }, 0.0f };
    float32 liveness = 0.0f;
    TEST_ASSERT_FALSE(Mmc5983_plausible(&s, &liveness));
    TEST_ASSERT_TRUE(isnan(liveness));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nominal_plausible);
    RUN_TEST(test_band_edges);
    RUN_TEST(test_nan_is_not_plausible);
    return UNITY_END();
}
