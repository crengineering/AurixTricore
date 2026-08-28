#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/Atmosphere.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

/* Nominal: sea level pressure at a plausible QNH gives ~0 m. */
void test_nominal(void)
{
    float32 alt = Atmosphere_altitudeM(101325.0f, 101325.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, alt);
}

/* A QNH of zero is out of the 80 000..120 000 Pa band and must fall back to
 * the ISA standard 101 325 Pa rather than dividing by (or propagating) zero. */
void test_qnh_zero_falls_back_to_isa(void)
{
    float32 withDefault = Atmosphere_altitudeM(90000.0f, 101325.0f);
    float32 withZero     = Atmosphere_altitudeM(90000.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, withDefault, withZero);
}

/* Out-of-band QNH (both directions) also falls back. */
void test_qnh_out_of_band_falls_back(void)
{
    float32 def   = Atmosphere_altitudeM(90000.0f, 101325.0f);
    float32 tooLo = Atmosphere_altitudeM(90000.0f, 79999.0f);
    float32 tooHi = Atmosphere_altitudeM(90000.0f, 120001.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, def, tooLo);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, def, tooHi);
}

/* A NaN QNH must also fall back -- NaN fails both `< MIN` and `> MAX`, so a
 * naive out-of-band guard written that way would let it through unclamped.
 * The positive-form guard in Atmosphere.c is what catches it. */
void test_qnh_nan_falls_back(void)
{
    float32 def = Atmosphere_altitudeM(90000.0f, 101325.0f);
    float32 nan = Atmosphere_altitudeM(90000.0f, NAN);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, def, nan);
}

/* pressPa = 0 is the formula's edge: (0/p0)^exp = 0, so altitude = the full
 * scale height. Not a physically reachable sample, but the formula must not
 * do anything undefined at the edge. */
void test_press_zero(void)
{
    float32 alt = Atmosphere_altitudeM(0.0f, 101325.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 44330.0f, alt);
}

/* A NaN pressure is not separately guarded here (see Atmosphere.h): it must
 * propagate as NaN so the existing downstream guard in Fusion_setBaroAlt
 * (fusion.c fusion_usable()) is what catches it, exactly like any other
 * invalid altitude -- a second guard here would be redundant and would hide
 * a regression in that guard instead of failing loudly. */
void test_press_nan_propagates(void)
{
    float32 alt = Atmosphere_altitudeM(NAN, 101325.0f);
    TEST_ASSERT_TRUE(isnan(alt));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nominal);
    RUN_TEST(test_qnh_zero_falls_back_to_isa);
    RUN_TEST(test_qnh_out_of_band_falls_back);
    RUN_TEST(test_qnh_nan_falls_back);
    RUN_TEST(test_press_zero);
    RUN_TEST(test_press_nan_propagates);
    return UNITY_END();
}
