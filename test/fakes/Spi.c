/* Stub QSPI0 bus service for host tests -- see fakes/I2c.c for why. */
#include "Spi.h"

static uint8 s_mode = SPI_MODE_0;

void Spi_setMode(uint8 spiMode)
{
    s_mode = spiMode;
}

uint8 Spi_getMode(void)
{
    return s_mode;
}

void Spi_init(void) {}

boolean Spi_transfer(const uint8 *tx, uint8 *rx, uint16 len)
{
    (void)tx;
    (void)rx;
    (void)len;
    return FALSE;
}
