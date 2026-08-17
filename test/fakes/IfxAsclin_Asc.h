#ifndef IFXASCLIN_ASC_H
#define IFXASCLIN_ASC_H

/* Host replacement for the iLLD ASC (UART mode) layer.
 *
 * Again: only what GnssM9N.c uses. The real IfxAsclin_Asc handle carries
 * software-FIFO pointers, ISR state and timestamps -- the driver touches none
 * of it, it just hands &g_asclin to initModule. So a placeholder is enough,
 * the same trick the IfxStm fake uses for Ifx_STM.
 */

#include "Ifx_Types.h"
#include "IfxAsclin.h"

#ifndef NULL_PTR
#define NULL_PTR ((void *)0)
#endif

/* Only the enumerators the driver names are needed. */
typedef enum { IfxPort_InputMode_pullUp    = 0 } IfxPort_InputMode;
typedef enum { IfxPort_OutputMode_pushPull = 0 } IfxPort_OutputMode;
typedef enum { IfxPort_PadDriver_ttlSpeed1 = 0 } IfxPort_PadDriver;

/* Pin descriptor objects. The driver only takes their addresses. */
typedef struct { int dummy; } IfxAsclin_Cts_In;
typedef struct { int dummy; } IfxAsclin_Rts_Out;
typedef struct { int dummy; } IfxAsclin_Rx_In;
typedef struct { int dummy; } IfxAsclin_Tx_Out;

extern const IfxAsclin_Rx_In  IfxAsclin4_RXC_P22_6_IN;
extern const IfxAsclin_Tx_Out IfxAsclin4_TX_P22_5_OUT;

typedef struct
{
    const IfxAsclin_Cts_In  *cts;
    IfxPort_InputMode        ctsMode;
    const IfxAsclin_Rx_In   *rx;
    IfxPort_InputMode        rxMode;
    const IfxAsclin_Rts_Out *rts;
    IfxPort_OutputMode       rtsMode;
    const IfxAsclin_Tx_Out  *tx;
    IfxPort_OutputMode       txMode;
    IfxPort_PadDriver        pinDriver;
} IfxAsclin_Asc_Pins;

typedef struct
{
    float32 baudrate;
} IfxAsclin_Asc_BaudRate;

typedef struct
{
    Ifx_ASCLIN               *asclin;
    IfxAsclin_Asc_BaudRate    baudrate;
    const IfxAsclin_Asc_Pins *pins;
    IfxAsclin_ClockSource     clockSource;
    void                     *txBuffer;
    uint32                    txBufferSize;
    void                     *rxBuffer;
    uint32                    rxBufferSize;
} IfxAsclin_Asc_Config;

typedef struct { int dummy; } IfxAsclin_Asc;

/** Fills the config with the vendor defaults (115200 baud, no pins, no
 *  software FIFOs) so a driver that forgets to override something behaves
 *  in the test exactly as it would on silicon. */
void IfxAsclin_Asc_initModuleConfig(IfxAsclin_Asc_Config *config,
                                    Ifx_ASCLIN           *asclin);

/** Applies the config and returns the vendor's status.
 *
 *  IMPORTANT, and faithful to the real iLLD: the status is
 *  configurationError whenever txBuffer or rxBuffer is NULL_PTR. The vendor
 *  sets it in the *else* branch of "did the application provide a buffer",
 *  so a deliberately polled driver -- which must pass NULL_PTR -- can never
 *  get noError. All the register setup still completes either side of it.
 *
 *  The fake reproduces that on purpose. Returning noError here would let the
 *  host test exercise a path that does not exist on silicon. */
IfxAsclin_Status IfxAsclin_Asc_initModule(IfxAsclin_Asc             *asclin,
                                          const IfxAsclin_Asc_Config *config);

/** The config passed to the most recent initModule call. */
const IfxAsclin_Asc_Config *FakeAsclin_Asc_lastConfig(void);

#endif /* IFXASCLIN_ASC_H */
