/**********************************************************************************************************************
 * \file PeriphDiag.h
 * \brief Generic fault detection for peripherals (sensors, and later ESCs, GNSS, ...).
 *
 * The board-level checks in Diagnostics.c watch rails and die temperature.
 * This module watches the things that go wrong *outside* the chip: a jumper
 * pulled off, a chafed wire shorting a bus line, a device that answers but has
 * silently stopped converting.
 *
 * Four independent faults per peripheral, because they need different repairs:
 *
 *   NO_RESPONSE  never answered since boot -> not wired, wrong address, or dead.
 *                Distinct from TIMEOUT so "never worked" is not confused with
 *                "worked, then broke", which point at different causes.
 *   TIMEOUT      answered before, now silent for longer than the timeout ->
 *                wire came off, connector vibrated loose, device browned out.
 *   STUCK_DATA   transactions succeed but the value never changes. Worth its
 *                own bit: on 2026-07-30 the MPU-6050 froze its sensor
 *                registers while every read still ACKed, so both the bus and
 *                the presence flag looked perfectly healthy for days. Nothing
 *                except a liveness check catches that class of fault.
 *   IMPLAUSIBLE  answering and changing, but outside the physically possible
 *                range -> wrong scaling, corrupt calibration, damaged element.
 *
 * Bus-level shorts are detected separately, see PeriphDiag_update(): with I2C
 * idle both lines must float high, so a line held low means a short to ground
 * or a slave jamming the bus. That is what separates a SHORT from an OPEN
 * circuit -- an open bus reads perfectly idle and simply never ACKs.
 *
 * Sensor-specific knowledge (what counts as a plausible pressure, which signal
 * proves liveness) stays with the caller; this module only does the timing,
 * debouncing and bit assignment, so adding a peripheral needs no changes here
 * beyond one enum entry and its bit masks.
 *********************************************************************************************************************/
#ifndef PERIPHDIAG_H
#define PERIPHDIAG_H

#include "Ifx_Types.h"

/** Monitored peripherals. Add new devices here and in s_bits[] in PeriphDiag.c. */
typedef enum
{
    PERIPH_DIAG_BARO = 0,   /**< BMP581 barometer,     I2C0 0x47 */
    PERIPH_DIAG_IMU  = 1,   /**< IMU slot — nothing fitted since 2026-07-31 */
    PERIPH_DIAG_MAG  = 2,   /**< MMC5983MA magnetometer, I2C0 0x30 */
    PERIPH_DIAG_COUNT
} PeriphDiag_Id;

/** Reset all peripheral state. Call once at start-up, before the scheduler.
 *  Leaves every peripheral UNFITTED — the build declares what it expects with
 *  PeriphDiag_setFitted() rather than this module assuming the full list. */
void PeriphDiag_init(void);

/** Declare whether \p id is expected to be present in this build.
 *
 *  An unfitted peripheral contributes no diagStatus bits at all. Without this
 *  a slot whose hardware has been removed reports NO_RESPONSE forever, and a
 *  diagnostics word that is permanently red is a diagnostics word nobody
 *  reads. It is also the hook the planned driver pool needs: which devices are
 *  connected is configuration, not a compile-time fact about the code.
 *
 *  Call after PeriphDiag_init() and before the scheduler starts. */
void PeriphDiag_setFitted(PeriphDiag_Id id, boolean fitted);

/** Report the outcome of one peripheral access. Call at the sensor's poll rate.
 *
 *  \param id         which peripheral
 *  \param readOk     TRUE if the transaction completed and returned data
 *  \param plausible  TRUE if that data is inside the physically possible range.
 *                    Only evaluated when \p readOk is TRUE; the caller owns
 *                    this because only it knows what the numbers mean.
 *  \param liveness   a value that must keep changing on a healthy device --
 *                    typically a sum of the live signals, so sensor noise moves
 *                    it every sample. Held bit-identical for the stuck timeout
 *                    sets STUCK_DATA. */
void PeriphDiag_report(PeriphDiag_Id id, boolean readOk, boolean plausible, float32 liveness);

/** Evaluate all peripherals and the bus, and return the diagStatus bits to set.
 *  Call once per 100 ms diagnostics tick, after the sensor tasks have run. */
uint32 PeriphDiag_update(void);

#endif /* PERIPHDIAG_H */
