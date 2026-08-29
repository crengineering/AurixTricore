/* gen_fusion_trace -- treibt den ECHTEN fusion.c mit einer Kommandofolge von
 * stdin und schreibt den Zustand nach jedem Schritt als CSV nach stdout.
 *
 * Exists so the differential test (spec section 4) and the hardware replay
 * (spec section 5) can drive the C filter from Python without either side
 * knowing anything about the other's internals. The command language is the
 * public interface of fusion.h, one call per line:
 *
 *   INIT                                     Fusion_init()
 *   CAL <field> <value>                      write one g_fusionCal field
 *   BARO <altM>                              Fusion_setBaroAlt(altM, TRUE)
 *   BAROBAD <altM>                           Fusion_setBaroAlt(altM, FALSE)
 *   GNSS <lat1e7> <lon1e7> <alt> <spd> <hdg> <hacc> <itow>
 *   STEP <aN> <aE> <aD> <dt>                 Fusion_update(), then print a row
 *   STEPBAD <aN> <aE> <aD> <dt>              same with valid = FALSE
 *
 * Values print with %.9g: float32 round-trips exactly at 9 significant digits,
 * so the comparison in Python sees the same numbers the C saw.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Ifx_Types.h"
#include "fusion.h"
#include "FusionCal.h"

static FusionValues f;
static unsigned long step;

static void header(void)
{
    printf("step,d,vd,accBiasD,baroBias,innov,p00,aD,"
           "posN,posE,velN,velE,accBiasN,accBiasE,innovN,innovE,pNN,aN,aE,"
           "rejects,resets,gnssRejects,gnssUpdates,covResets,"
           "verticalOk,horizontalOk,originSet\n");
}

static void row(void)
{
    printf("%lu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
           "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
           "%u,%u,%u,%u,%u,%u,%u,%u\n",
           step,
           (double)f.a_d, (double)f.a_v_d, (double)f.accBiasD, (double)f.baroBias,
           (double)f.innov, (double)f.p00, (double)f.a_D,
           (double)f.posN, (double)f.posE, (double)f.velN, (double)f.velE,
           (double)f.accBiasN, (double)f.accBiasE, (double)f.innovN, (double)f.innovE,
           (double)f.pNN, (double)f.a_N, (double)f.a_E,
           (unsigned)f.rejects, (unsigned)f.resets, (unsigned)f.gnssRejects,
           (unsigned)f.gnssUpdates, (unsigned)f.covResets,
           (unsigned)f.verticalOk, (unsigned)f.horizontalOk, (unsigned)f.originSet);
}

static void setCal(const char *name, double v)
{
    const float32 x = (float32)v;
    if      (!strcmp(name, "twoKpAcc"))      { g_fusionCal.twoKpAcc = x; }
    else if (!strcmp(name, "twoKpMag"))      { g_fusionCal.twoKpMag = x; }
    else if (!strcmp(name, "twoKi"))         { g_fusionCal.twoKi = x; }
    else if (!strcmp(name, "sigmaAccD"))     { g_fusionCal.sigmaAccD = x; }
    else if (!strcmp(name, "sigmaBaro"))     { g_fusionCal.sigmaBaro = x; }
    else if (!strcmp(name, "sigmaBaroRw"))   { g_fusionCal.sigmaBaroRw = x; }
    else if (!strcmp(name, "tauBaroBias"))   { g_fusionCal.tauBaroBias = x; }
    else if (!strcmp(name, "sigmaAccH"))     { g_fusionCal.sigmaAccH = x; }
    else if (!strcmp(name, "sigmaGnssVel"))  { g_fusionCal.sigmaGnssVel = x; }
    else if (!strcmp(name, "gnssPosRScale")) { g_fusionCal.gnssPosRScale = x; }
    else if (!strcmp(name, "gateSigmaSq"))   { g_fusionCal.gateSigmaSq = x; }
    else if (!strcmp(name, "gateMinM"))      { g_fusionCal.gateMinM = x; }
    else if (!strcmp(name, "sigmaAccRw"))    { g_fusionCal.sigmaAccRw = x; }
    else { fprintf(stderr, "unknown cal field '%s'\n", name); exit(2); }
}

int main(void)
{
    char line[512];

    FusionCal_init();
    Fusion_init();
    memset(&f, 0, sizeof f);
    header();

    while (fgets(line, (int)sizeof line, stdin) != NULL)
    {
        char cmd[32];
        if (sscanf(line, "%31s", cmd) != 1) { continue; }

        if (!strcmp(cmd, "INIT"))
        {
            Fusion_init();
            memset(&f, 0, sizeof f);
            step = 0u;
        }
        else if (!strcmp(cmd, "CAL"))
        {
            char name[32]; double v;
            if (sscanf(line, "%*s %31s %lf", name, &v) == 2) { setCal(name, v); }
        }
        else if (!strcmp(cmd, "BARO") || !strcmp(cmd, "BAROBAD"))
        {
            double a;
            if (sscanf(line, "%*s %lf", &a) == 1)
            {
                Fusion_setBaroAlt((float32)a, (boolean)(cmd[4] == '\0' ? TRUE : FALSE));
            }
        }
        else if (!strcmp(cmd, "GNSS"))
        {
            long lat, lon; double alt, spd, hdg, hacc; unsigned long itow;
            if (sscanf(line, "%*s %ld %ld %lf %lf %lf %lf %lu",
                       &lat, &lon, &alt, &spd, &hdg, &hacc, &itow) == 7)
            {
                Fusion_setGnss((sint32)lat, (sint32)lon, (float32)alt, (float32)spd,
                               (float32)hdg, (float32)hacc, (uint32)itow, TRUE);
            }
        }
        else if (!strcmp(cmd, "STEP") || !strcmp(cmd, "STEPBAD"))
        {
            double an, ae, ad, dt;
            if (sscanf(line, "%*s %lf %lf %lf %lf", &an, &ae, &ad, &dt) == 4)
            {
                const float32 acc[3] = { (float32)an, (float32)ae, (float32)ad };
                Fusion_update(&f, acc, (float32)dt, (boolean)(cmd[4] == '\0' ? TRUE : FALSE));
                ++step;
                row();
            }
        }
        else
        {
            fprintf(stderr, "unknown command '%s'\n", cmd);
            return 2;
        }
    }
    return 0;
}
