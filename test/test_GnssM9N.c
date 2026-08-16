#include "unity.h"
#include "fakes/Uart.h"
#include "fakes/IfxAsclin.h"
#include "../src/bsw/GnssM9N.h"

void setUp(void)
{
  /* This is run before EACH TEST */
  FakeAsclin_reset();
  (void)GnssM9N_init();
}

void tearDown(void)
{
    /* This is run after test*/
}


/*
checking the gnss read function
*/
void test_gnss_read(void)
{
    GnssM9N_Sample gnss_sample;

    /* GNSS not detected yet */
    TEST_ASSERT_EQUAL(FALSE, GnssM9N_read(&gnss_sample));

    /* fake byte to the Fifo Buffer, so Gnss module can detect it */
    FakeAsclin_pushRx("$");
    GnssM9N_poll();

    TEST_ASSERT_EQUAL(TRUE, GnssM9N_read(&gnss_sample));
    TEST_ASSERT_EQUAL_UINT32(1u, gnss_sample.rxBytes); 
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_gnss_read);
    return UNITY_END();
}
