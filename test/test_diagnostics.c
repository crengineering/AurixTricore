#include "unity.h"
#include "fakes/Ifx_Types.h"
#include "../src/bsw/Diagnostics.h"

/* SWE1-FW-011: docs/DIAGNOSTICS.md / Diagnostics.h name this file as the
 * host test for diagnosticsNavGnssUntrusted(), the pure decision behind
 * NAVDIAG_GNSS_UNTRUSTED. `static inline`, no iLLD include (only
 * Ifx_Types.h) -- same idiom as ImuInt_accumulate() (test_imuint.c), so no
 * fake is needed here either. diagnosticsUpdate() itself (Diagnostics.c,
 * Uart/Nvm/PeriphDiag-dependent) is not exercised by this file; only the
 * decision this task actually asked for is.
 *
 * Truth table (task): (gnssTrusted, gnssNavOk) ->
 *   (0, 1) -> set    "fix present, refused by the trust gate"
 *   (1, 1) -> clear  trusted
 *   (0, 0) -> clear  no fix at all (not this bit's job -- see Diagnostics.h)
 *   (1, 0) -> clear  trusted (vacuously; no fix either way) */

void setUp(void)
{
}

void tearDown(void)
{
}

void test_untrusted_with_fix_sets_the_bit(void)
{
    TEST_ASSERT_EQUAL_UINT32(NAVDIAG_GNSS_UNTRUSTED,
                              diagnosticsNavGnssUntrusted(FALSE, TRUE));
}

void test_trusted_with_fix_clears_the_bit(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, diagnosticsNavGnssUntrusted(TRUE, TRUE));
}

void test_untrusted_without_a_fix_clears_the_bit(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, diagnosticsNavGnssUntrusted(FALSE, FALSE));
}

void test_trusted_without_a_fix_clears_the_bit(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, diagnosticsNavGnssUntrusted(TRUE, FALSE));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_untrusted_with_fix_sets_the_bit);
    RUN_TEST(test_trusted_with_fix_clears_the_bit);
    RUN_TEST(test_untrusted_without_a_fix_clears_the_bit);
    RUN_TEST(test_trusted_without_a_fix_clears_the_bit);
    return UNITY_END();
}
