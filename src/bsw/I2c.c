/**********************************************************************************************************************
 * \file I2c.c
 * \brief Blocking I2C0 master bus service (shared sensor bus) — see I2c.h.
 *
 * Wraps the iLLD IfxI2c_I2c driver. Register access uses the classic
 * "write register pointer, then read" sequence. The per-transfer device is
 * opened with enableRepeatedStart = FALSE, i.e. a STOP terminates both the
 * pointer write and the data read: the BMP581 (and the other bus devices)
 * latch the register pointer across the STOP, so a true repeated start is not
 * required and the bus is always released between phases.
 *
 * Hardware: I2C0, SCL = P13.1, SDA = P13.2, TTL pads.
 *********************************************************************************************************************/
#include "I2c.h"
#include "I2c/I2c/IfxI2c_I2c.h"   /* root-relative iLLD include (see gpio.h convention) */

/* Bring-up baud rate: 100 kHz standard mode. The BMP581 / MMC5983 / NEO-M9N bus
 * can move to 400 kHz Fast-mode once the wiring is proven (docs/PINNING.md 2.5). */
#define I2C_BAUDRATE_HZ   (100000.0f)

/* Bounded polling. write2/read2 return NAK while a slave is busy or absent;
 * retry a finite number of times so a missing sensor can never hang the
 * scheduler. Far beyond any real ACK-polling window at 100 kHz. */
#define I2C_MAX_ATTEMPTS  (2000u)

/* Largest register write payload ([reg] + data) this module supports. */
#define I2C_WRITE_MAX     (8u)

static IfxI2c_I2c g_i2c;   /* I2C0 module handle */

void I2c_init(void)
{
    IfxI2c_I2c_Config config;
    IfxI2c_I2c_initConfig(&config, &MODULE_I2C0);

    /* SCL = P13.1, SDA = P13.2; TTL pads so a 3.3 V bus reads high on 5 V VEXT. */
    static const IfxI2c_Pins pins = {
        .scl       = &IfxI2c0_SCL_P13_1_INOUT,
        .sda       = &IfxI2c0_SDA_P13_2_INOUT,
        .padDriver = IfxPort_PadDriver_ttlSpeed1
    };
    config.pins     = &pins;
    config.baudrate = I2C_BAUDRATE_HZ;

    IfxI2c_I2c_initModule(&g_i2c, &config);
}

/* Build a per-transfer device descriptor for the given 7-bit address. */
static void i2c_openDevice(IfxI2c_I2c_Device *dev, uint8 addr7)
{
    IfxI2c_I2c_deviceConfig cfg;
    IfxI2c_I2c_initDeviceConfig(&cfg, &g_i2c);
    cfg.deviceAddress       = (uint16)((uint16)addr7 << 1u);  /* iLLD wants the 8-bit form */
    cfg.enableRepeatedStart = FALSE;
    IfxI2c_I2c_initDevice(dev, &cfg);
}

/* One bounded write2 attempt loop. \return TRUE on Status_ok. */
static boolean i2c_writePhase(IfxI2c_I2c_Device *dev, volatile uint8 *data, uint16 len)
{
    IfxI2c_I2c_Status st;
    uint32            attempts = 0u;
    boolean           ok       = FALSE;

    do
    {
        st = IfxI2c_I2c_write2(dev, data, (Ifx_SizeT)len);
        attempts++;
    } while (((st == IfxI2c_I2c_Status_nak) || (st == IfxI2c_I2c_Status_busNotFree))
             && (attempts < I2C_MAX_ATTEMPTS));

    if (st == IfxI2c_I2c_Status_ok)
    {
        ok = TRUE;
    }
    return ok;
}

/* One bounded read2 attempt loop. \return TRUE on Status_ok. */
static boolean i2c_readPhase(IfxI2c_I2c_Device *dev, volatile uint8 *data, uint16 len)
{
    IfxI2c_I2c_Status st;
    uint32            attempts = 0u;
    boolean           ok       = FALSE;

    do
    {
        st = IfxI2c_I2c_read2(dev, data, (Ifx_SizeT)len);
        attempts++;
    } while (((st == IfxI2c_I2c_Status_nak) || (st == IfxI2c_I2c_Status_busNotFree))
             && (attempts < I2C_MAX_ATTEMPTS));

    if (st == IfxI2c_I2c_Status_ok)
    {
        ok = TRUE;
    }
    return ok;
}

boolean I2c_readReg(uint8 addr7, uint8 reg, uint8 *data, uint16 len)
{
    IfxI2c_I2c_Device dev;
    uint8             regAddr = reg;
    boolean           ok;

    i2c_openDevice(&dev, addr7);
    ok = i2c_writePhase(&dev, &regAddr, 1u);   /* set the register pointer */
    if (ok != FALSE)
    {
        ok = i2c_readPhase(&dev, data, len);   /* read the data bytes       */
    }
    return ok;
}

boolean I2c_writeReg(uint8 addr7, uint8 reg, const uint8 *data, uint16 len)
{
    IfxI2c_I2c_Device dev;
    uint8             buf[I2C_WRITE_MAX];      /* [reg][data...] */
    uint16            i;
    boolean           ok = FALSE;

    if (len < I2C_WRITE_MAX)               /* [reg] + len data must fit the buffer */
    {
        buf[0] = reg;
        for (i = 0u; i < len; i++)
        {
            buf[i + 1u] = data[i];
        }
        i2c_openDevice(&dev, addr7);
        ok = i2c_writePhase(&dev, buf, (uint16)(len + 1u));
    }
    return ok;
}

boolean I2c_writeByte(uint8 addr7, uint8 reg, uint8 value)
{
    return I2c_writeReg(addr7, reg, &value, 1u);
}
