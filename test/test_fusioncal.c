/* FusionCal — der Sanitizer, durch den JEDER Tuning-Parameter geht.
 *
 * FusionCal.h documents the contract of FusionCal_positive() in one sentence:
 *   "return a positive value: v if it is finite and above lo, else def".
 *
 * Spec section 2.3 and defect 7 ("a calibration value of zero or NaN reaching a
 * divisor") say why this matters: an XCP master can write any 32-bit pattern
 * into g_fusionCal, and the value is read every tick, not latched. The trap
 * named in the spec is that every comparison against NaN is false, so a guard
 * written as "if (v <= lo) return def" lets NaN straight through.
 */
#include "unity.h"
#include <math.h>
#include <stdio.h>
#include "Ifx_Types.h"
#include "FusionCal.h"
#include "test_util_math.h"

void setUp(void) { }
void tearDown(void) { }

/* --- 1. the plain contract: a good value passes through unchanged -------- */

void test_positive_passes_good_values(void)
{
    TEST_ASSERT_EQUAL_FLOAT(0.5f,   FusionCal_positive(0.5f,   1e-6f, 9.0f));
    TEST_ASSERT_EQUAL_FLOAT(1e-3f,  FusionCal_positive(1e-3f,  1e-6f, 9.0f));
    TEST_ASSERT_EQUAL_FLOAT(1e6f,   FusionCal_positive(1e6f,   1e-6f, 9.0f));
}

/* --- 2. at or below the floor falls back to the default ------------------ */

void test_positive_rejects_zero_and_negative(void)
{
    TEST_ASSERT_EQUAL_FLOAT(9.0f, FusionCal_positive(0.0f,   1e-6f, 9.0f));
    TEST_ASSERT_EQUAL_FLOAT(9.0f, FusionCal_positive(-1.0f,  1e-6f, 9.0f));
    TEST_ASSERT_EQUAL_FLOAT(9.0f, FusionCal_positive(-0.0f,  1e-6f, 9.0f));
    TEST_ASSERT_EQUAL_FLOAT(9.0f, FusionCal_positive(1e-9f,  1e-6f, 9.0f));
}

/* --- 3. the NaN trap (spec 2.3) ----------------------------------------- */

void test_positive_rejects_nan(void)
{
    const float32 out = FusionCal_positive(NAN, 1e-6f, 9.0f);
    TEST_ASSERT_TRUE_MESSAGE(isFiniteF(out), "NaN calibration value escaped the sanitizer");
    TEST_ASSERT_EQUAL_FLOAT(9.0f, out);
}

void test_positive_rejects_inf(void)
{
    float32 out = FusionCal_positive(INFINITY, 1e-6f, 9.0f);
    TEST_ASSERT_TRUE_MESSAGE(isFiniteF(out), "+inf calibration value escaped the sanitizer");
    TEST_ASSERT_EQUAL_FLOAT(9.0f, out);

    out = FusionCal_positive(-INFINITY, 1e-6f, 9.0f);
    TEST_ASSERT_TRUE_MESSAGE(isFiniteF(out), "-inf calibration value escaped the sanitizer");
    TEST_ASSERT_EQUAL_FLOAT(9.0f, out);
}

/* --- 4. property: the result is ALWAYS finite and > lo ------------------- */

void test_positive_is_always_admissible(void)
{
    Rng r; rngSeed(&r, 0x51A7Eu);
    int i;
    for (i = 0; i < 20000; ++i)
    {
        /* every bit pattern a master could write, including the NaN space */
        union { unsigned int u; float f; } bits;
        bits.u = rngNext(&r);
        const float32 lo  = 1e-6f;
        const float32 def = 0.25f;
        const float32 out = FusionCal_positive(bits.f, lo, def);

        char msg[128];
        (void)snprintf(msg, sizeof msg, "input bits 0x%08X (%g) -> %g", bits.u, (double)bits.f, (double)out);
        TEST_ASSERT_TRUE_MESSAGE(isFiniteF(out), msg);
        TEST_ASSERT_TRUE_MESSAGE(out > lo, msg);
    }
}

/* --- 5. the compiled defaults are themselves admissible ------------------ */

void test_defaults_are_sane(void)
{
    FusionCal_init();
    TEST_ASSERT_EQUAL_HEX32(XCP_FUSIONCAL_MAGIC, g_fusionCal.magic);

    const float32 v[] = {
        g_fusionCal.twoKpAcc, g_fusionCal.twoKpMag, g_fusionCal.twoKi,
        g_fusionCal.sigmaAccD, g_fusionCal.sigmaBaro, g_fusionCal.sigmaBaroRw,
        g_fusionCal.tauBaroBias, g_fusionCal.sigmaAccH, g_fusionCal.sigmaGnssVel,
        g_fusionCal.gnssPosRScale, g_fusionCal.gateSigmaSq, g_fusionCal.gateMinM,
        g_fusionCal.sigmaAccRw
    };
    unsigned i;
    for (i = 0u; i < sizeof v / sizeof v[0]; ++i)
    {
        TEST_ASSERT_TRUE(isFiniteF(v[i]));
        TEST_ASSERT_TRUE(v[i] > 0.0f);
    }

    /* FusionCal.h: sigmaBaro is the barometer noise in metres and R is its
     * square. Spec section 5 measured 0.0197 m on this board; the default must
     * be in the same decade or the tuning is not the one that was measured. */
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0197f, g_fusionCal.sigmaBaro);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_positive_passes_good_values);
    RUN_TEST(test_positive_rejects_zero_and_negative);
    RUN_TEST(test_positive_rejects_nan);
    RUN_TEST(test_positive_rejects_inf);
    RUN_TEST(test_positive_is_always_admissible);
    RUN_TEST(test_defaults_are_sane);
    return UNITY_END();
}
