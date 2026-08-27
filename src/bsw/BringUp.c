/**********************************************************************************************************************
 * \file BringUp.c
 * \brief Boot-time register dumps for the I2C0/QSPI0 sensors — see BringUp.h.
 *
 * Moved out of Cpu0_Main.c unchanged (T3, docs/REFACTORING_PLAN.md): each
 * dump was written for a device with no datasheet in the repo, so the boot
 * dump is how the register assumptions in Bmp581.c/Mmc5983.c/Icm42688.c get
 * checked against real silicon. See those files' headers for what each field
 * should read.
 *********************************************************************************************************************/
#include "BringUp.h"
#include "Uart.h"
#include "Bmp581.h"
#include "Mmc5983.h"
#include "Icm42688.h"
#include "IfxPort.h"
#include "IfxStm.h"

/* One-time raw BMP581 register dump at boot for bring-up diagnosis.
 *
 * Every register value in Bmp581.c was written from the BMP5 register map
 * without the datasheet in hand (docs/BMP581.md section 10), so this dump is how
 * those assumptions get checked against silicon. Read it in the order the
 * bring-up fails — see the Bmp581_debugDump() comment in Bmp581.h for what each
 * field should read; the short version is INT_STATUS bit 4 set (reset landed),
 * STATUS = nvm_rdy without nvm_err, OSR bit 6 set (pressure enabled) and ODR
 * bits 1:0 = 1 (converting). A burst that is all-zero or identical across two
 * resets means the device is not converting, not that the scaling is wrong. */
static void bmp581DebugDump(void)
{
    uint8 cfg[BMP581_DUMP_CFG_LEN];
    uint8 osrEff = 0u;
    uint8 raw[6];

    if (Bmp581_debugDump(cfg, &osrEff, raw) != FALSE)
    {
        uint8 i;
        Uart_print("BMP581 CHIP/REV/INT_STAT/STAT/IIR/OSR/ODR=");
        for (i = 0u; i < BMP581_DUMP_CFG_LEN; i++)
        {
            Uart_printHexByte(cfg[i]);
            Uart_print(" ");
        }
        Uart_print("OSR_EFF=");
        Uart_printHexByte(osrEff);
        Uart_print(" burst(T,P)=");
        for (i = 0u; i < 6u; i++)
        {
            Uart_printHexByte(raw[i]);
            Uart_print(" ");
        }
        Uart_println("");
    }
    else
    {
        Uart_println("BMP581 dump failed - no ACK on the bus");
    }
}

/* One-time raw MMC5983MA register dump at boot, for the same reason as the
 * BMP581's: no device datasheet exists (docs/MMC5983MA.md section 6 — the PDF
 * is the prototyping board's guide and carries no register information), so
 * every constant in Mmc5983.c is unverified until silicon says otherwise.
 *
 * PRODUCT_ID must read 0x30; that one value proves the whole I2C path including
 * the CS strap. CTRL0/1/2 reading 0x00 is EXPECTED and not a failed write —
 * the control registers are write-only on this part. The data block is the real
 * evidence, and |B| in the measurement block is the decisive test. */
static void mmc5983DebugDump(void)
{
    uint8 cfg[MMC5983_DUMP_CFG_LEN];
    uint8 raw[7];

    if (Mmc5983_debugDump(cfg, raw) != FALSE)
    {
        uint8 i;
        Uart_print("MMC5983 PID/STATUS/CTRL0/CTRL1/CTRL2=");
        for (i = 0u; i < MMC5983_DUMP_CFG_LEN; i++)
        {
            Uart_printHexByte(cfg[i]);
            Uart_print(" ");
        }
        Uart_print(" burst=");
        for (i = 0u; i < 7u; i++)
        {
            Uart_printHexByte(raw[i]);
            Uart_print(" ");
        }
        Uart_println("");
    }
    else
    {
        Uart_println("MMC5983 dump failed - no ACK on the bus");
    }
}

/* One-time raw ICM-42688-P register dump at boot. Third sensor in a row with
 * no device datasheet available (docs/ICM42688P.md 4 — the PDF is the EVB
 * user guide), so this is again how the register assumptions meet silicon.
 *
 * WHO_AM_I must read 0x47. A 0x00 or 0xFF means nothing is coming back at all,
 * and on this bus that points at the MRST pad mode or the level shifters
 * before it points at the part. PWR_MGMT0 = 0x0F is the other one worth
 * checking: leave it at its reset value and every axis reads zero forever
 * while the bus looks perfectly healthy. */
static void icm42688DebugDump(void)
{
    uint8 cfg[ICM42688_DUMP_CFG_LEN];
    uint8 raw[14];

    if (Icm42688_debugDump(cfg, raw) != FALSE)
    {
        uint8 i;
        Uart_print("ICM42688 WHO/PWR/GYRO/ACCEL/INT_STAT/INT_CFG/INT_CFG0/INT_CFG1/INT_SRC0=");
        for (i = 0u; i < ICM42688_DUMP_CFG_LEN; i++)
        {
            Uart_printHexByte(cfg[i]);
            Uart_print(" ");
        }
        Uart_print(" burst(T,A,G)=");
        for (i = 0u; i < 14u; i++)
        {
            Uart_printHexByte(raw[i]);
            Uart_print(" ");
        }
        Uart_println("");
    }
    else
    {
        Uart_println("ICM42688 dump failed - no response on QSPI0");
    }
}

void BringUp_dumpSensors(void)
{
    bmp581DebugDump();
    mmc5983DebugDump();
    icm42688DebugDump();
}

/* The P10 pad self-test (docs/IMU_INTERRUPT.md 2.1) ran with nothing wired to
 * Port 10 and returned g_padProbeP10 == 0x7 -- P10.7 is free of the board,
 * same as its P10.3/P10.8 control group. That test drove all three pins as
 * outputs, which must not happen again now that P10.7 is the IMU INT1
 * candidate, so it has been removed rather than left dormant. What replaces
 * it is the permanent configuration below: P10.7 as an input, nothing else,
 * ever.
 *
 * P10.7 is a 5 V VEXT pad; the ICM-42688-P's INT1 absolute maximum is
 * VDDIO + 0.3 V = 3.6 V. Configuring this pin as an output at any point while
 * the INT1 wire is attached puts 5 V on the part and destroys it. Pulldown
 * (never pull-up -- the internal pull-up returns to the 5 V rail and would
 * fight the IMU's 3.3 V push-pull driver) plus TTL pad mode (the pad's
 * default CMOS threshold is ~3.5 V and would never register the IMU's 3.3 V
 * high) -- docs/IMU_INTERRUPT.md 1/3.3. Declared in BringUp.h (MISRA 8.4). */
/* cppcheck-suppress misra-c2012-8.7 ; deviation: read over XCP SHORT_UPLOAD
 * by raw address (tools/xcp_read.py), never referenced by C code outside
 * this file -- same class of deviation as ImuInt.c's globals. */
volatile uint32 g_p10Pin7Iocr;   /* silicon read-back proof, see below */

void BringUp_initImuIntPinSafe(void)
{
    IfxPort_setPinModeInput(&MODULE_P10, 7u, IfxPort_InputMode_pullDown);
    IfxPort_setPinPadDriver(&MODULE_P10, 7u, IfxPort_PadDriver_ttlSpeed1);

    /* Read the fact back from the register, not from the source: P10.7 is
     * pin 7 of Port 10, so its IOCR field is IOCR4.PC7 (Ifx_P_IOCR4_Bits,
     * IfxPort_reg.h; IfxPort.h IfxPort_Mode documents the PC encoding).
     * Expect 1 (input, pulldown). 0 would be input/no-pull, 2 input/pull-up;
     * any value >= 16 is an OUTPUT mode and must never be seen here. */
    g_p10Pin7Iocr = (uint32)MODULE_P10.IOCR4.B.PC7;

    Uart_print("P10.7 (IMU INT1 candidate) IOCR4.PC7 = 0x");
    Uart_printHexByte((uint8)g_p10Pin7Iocr);
    Uart_println(" (expect 1 = input, pulldown)");
}
