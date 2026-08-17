#ifndef IFXASCLIN_H
#define IFXASCLIN_H

/* Host replacement for the iLLD raw ASCLIN layer.
 *
 * This is NOT a copy of the vendor header. It declares only the surface that
 * GnssM9N.c actually calls, plus a FakeAsclin_* API that exists only in the
 * host build and lets a test drive the receive path.
 *
 * The register block is a placeholder: the driver only ever passes the pointer
 * through to the accessors below and never looks inside it.
 */

#include "Ifx_Types.h"

typedef struct { int dummy; } Ifx_ASCLIN;

extern Ifx_ASCLIN MODULE_ASCLIN4;

/* Values match the vendor enum: noError is 1, NOT 0 (see docs/ILLD_NOTES).
 * Getting this backwards is the whole reason the driver compares against the
 * named enumerator instead of testing truthiness. */
typedef enum
{
    IfxAsclin_Status_configurationError = 0,
    IfxAsclin_Status_noError            = 1
} IfxAsclin_Status;

/* Values are the vendor's, not 0/1/2: the driver compares against the
 * enumerator, so a test that hardcoded 1 would pass for the wrong reason. */
typedef enum
{
    IfxAsclin_ClockSource_noClock      = 0,
    IfxAsclin_ClockSource_ascFastClock = 2,
    IfxAsclin_ClockSource_ascSlowClock = 4
} IfxAsclin_ClockSource;

uint8   IfxAsclin_getRxFifoFillLevel(Ifx_ASCLIN *asclin);
uint32  IfxAsclin_readRxData(Ifx_ASCLIN *asclin);
boolean IfxAsclin_getRxFifoOverflowFlagStatus(Ifx_ASCLIN *asclin);
void    IfxAsclin_clearRxFifoOverflowFlag(Ifx_ASCLIN *asclin);
boolean IfxAsclin_getFrameErrorFlagStatus(Ifx_ASCLIN *asclin);
void    IfxAsclin_clearFrameErrorFlag(Ifx_ASCLIN *asclin);
void    IfxAsclin_flushRxFifo(Ifx_ASCLIN *asclin);
void    IfxAsclin_clearAllFlags(Ifx_ASCLIN *asclin);

/* Reads back what initModule applied. GnssM9N_init() uses this instead of the
 * initModule status, because that status is a constant for a polled driver
 * (see the note in IfxAsclin_Asc_initModule below). */
IfxAsclin_ClockSource IfxAsclin_getClockSource(Ifx_ASCLIN *asclin);

/* ---- test control, host build only ---------------------------------- */

/** Drop every pending byte and clear both error flags. */
void FakeAsclin_reset(void);

/** Append bytes to the simulated wire. They become visible to
 *  IfxAsclin_getRxFifoFillLevel exactly as if the GNSS had sent them. */
void FakeAsclin_pushRx(const char *bytes);

/** Raise the sticky error flags, as the hardware would. */
void FakeAsclin_setFrameError(void);
void FakeAsclin_setRxOverflow(void);

/** Force the clock source the next getClockSource() reports, to simulate a
 *  module that did not come up. Reset by the next initModule(). */
void FakeAsclin_forceClockSource(IfxAsclin_ClockSource src);

/** Bytes still waiting on the simulated wire (may exceed the 16-byte FIFO). */
uint32 FakeAsclin_pending(void);

#endif /* IFXASCLIN_H */
