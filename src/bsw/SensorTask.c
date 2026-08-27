/**********************************************************************************************************************
 * \file SensorTask.c
 * \brief Periodic barometer / magnetometer / GNSS tasks — see SensorTask.h.
 *
 * Moved out of Cpu0_Main.c unchanged (T6, docs/REFACTORING_PLAN.md): the
 * plausibility bands themselves already moved into the drivers in T5, and
 * the ISA altitude formula into Atmosphere.c in T4 — this module is just the
 * read/publish/latch/report glue, one task per sensor.
 *********************************************************************************************************************/
#include "SensorTask.h"
#include "Bmp581.h"
#include "Mmc5983.h"
#include "GnssM9N.h"
#include "Atmosphere.h"
#include "fusion.h"
#include "Ahrs.h"
#include "Measurements.h"
#include "Nvm.h"
#include "PeriphDiag.h"

void SensorTask_init(void)
{
    /* The IMU slot is NavTask's to declare (still Cpu0_Main.c until T11) --
     * this module only owns the three sensors above. */
    PeriphDiag_setFitted(PERIPH_DIAG_BARO, TRUE);
    PeriphDiag_setFitted(PERIPH_DIAG_MAG,  TRUE);
    PeriphDiag_setFitted(PERIPH_DIAG_GNSS, TRUE);
}

void SensorTask_baro(void)
{
    float32 pressPa = 0.0f;
    float32 tempC   = 0.0f;
    boolean present;
    float32 baroAlt;
    float32 liveness = 0.0f;
    boolean plausible;

    /* Called unconditionally: Bmp581_read() owns the presence state and uses
     * these calls to probe for a reconnected sensor. Gating on a presence flag
     * here would make that recovery unreachable — which is exactly why a
     * replugged barometer never came back, and why no such accessor exists. */
    present = Bmp581_read(&pressPa, &tempC);

    baroAlt = Atmosphere_altitudeM(pressPa, (float32)g_xcpNvm.seaLevelPa);
    Fusion_setBaroAlt(baroAlt, present);

    measurementsSetBaro(present, pressPa, tempC, baroAlt);

    plausible = Bmp581_plausible(pressPa, tempC, &liveness);
    PeriphDiag_report(PERIPH_DIAG_BARO, present, plausible, liveness);
}

void SensorTask_mag(void)
{
    Mmc5983_Sample sample = { { 0.0f, 0.0f, 0.0f }, 0.0f };
    boolean        present;
    float32        liveness = 0.0f;
    boolean        plausible;

    /* Called unconditionally, like SensorTask_baro: Mmc5983_read() owns the
     * presence state and uses these calls to probe for a reconnected sensor. */
    present = Mmc5983_read(&sample);
    measurementsSetMag(present, sample.mag, sample.headingDeg);

    /* Latch it for the attitude filter, which consumes it on the next IMU
     * tick. Hard-iron correction and the mounting transform happen in there,
     * not here -- sample.mag stays the RAW field so the published values and
     * tools/mag_cal.py keep seeing what the sensor actually reported. */
    Ahrs_setMag(sample.mag, present);

    plausible = Mmc5983_plausible(&sample, &liveness);
    PeriphDiag_report(PERIPH_DIAG_MAG, present, plausible, liveness);
}

void SensorTask_gnss(void)
{
    GnssM9N_Sample gnss_sample = {0};
    boolean        gnssPresent;
    boolean        gnssPlausible;

    gnssPresent = GnssM9N_read(&gnss_sample);

    /* Plausible = the receiver accepted every configuration command we sent.
     *
     * Without this the config path is fire-and-forget: a rejected key changes
     * nothing observable on the wire, so a silently unconfigured receiver
     * would look perfectly healthy. Raising DIAG_GNSS_IMPLAUSIBLE puts it on
     * the diagnostics LED and in the GUI instead.
     *
     * NOT gated on navOk -- "no satellites" is the normal indoor state, not a
     * fault, and a diagnostics word that is permanently red is one nobody
     * reads. PeriphDiag debounces this for PD_IMPLAUSIBLE_TICKS (1 s) and only
     * evaluates it when the read succeeded, so the boot window is covered. */
    gnssPlausible = (gnss_sample.cfgOk != 0u) ? TRUE : FALSE;
    measurementsSetGnss(gnssPresent, gnss_sample);

    /* Feed the navigation filter. navOk -- not fixOk -- is the gate: it also
     * requires a 3-D fix and an hAcc inside the usable band, which is what
     * keeps a barely-acquired solution from dragging the origin around.
     * Fusion_setGnss ignores a repeat of the same iTOW, so calling it at 10 Hz
     * on a 1 Hz solution is safe. */
    Fusion_setGnss(gnss_sample.latRaw, gnss_sample.lonRaw, gnss_sample.altM,
                   gnss_sample.speedMps, gnss_sample.headingDeg,
                   gnss_sample.hAccM, gnss_sample.iTOW,
                   (gnssPresent != FALSE) && (gnss_sample.navOk != 0u));
    PeriphDiag_report(PERIPH_DIAG_GNSS, gnssPresent, gnssPlausible, (float)gnss_sample.rxBytes);
}
