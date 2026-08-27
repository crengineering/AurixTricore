/* Stub I2c bus service for host tests. The real I2c.c pulls the iLLD IfxI2c
 * driver; a plausibility-band test never calls any of these (the sensor
 * drivers only reach the bus from init/read, neither of which the tests
 * exercise), so every function here exists purely to satisfy the linker. */
#include "I2c.h"

volatile I2c_Debug g_i2cDebug;

void I2c_init(void) {}

boolean I2c_busIsIdle(void)
{
    return TRUE;
}

void I2c_getLineState(boolean *sclReleased, boolean *sdaReleased)
{
    *sclReleased = TRUE;
    *sdaReleased = TRUE;
}

boolean I2c_readReg(uint8 addr7, uint8 reg, uint8 *data, uint16 len)
{
    (void)addr7;
    (void)reg;
    (void)data;
    (void)len;
    return FALSE;
}

boolean I2c_writeReg(uint8 addr7, uint8 reg, const uint8 *data, uint16 len)
{
    (void)addr7;
    (void)reg;
    (void)data;
    (void)len;
    return FALSE;
}

boolean I2c_writeByte(uint8 addr7, uint8 reg, uint8 value)
{
    (void)addr7;
    (void)reg;
    (void)value;
    return FALSE;
}
