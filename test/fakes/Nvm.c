/* Host-Fake fuer Nvm.c.
 *
 * Ahrs.c liest g_xcpNvm (Hard-Iron-Offsets, Soft-Iron-Skalen, Deklination).
 * Die echte Definition benutzt die TASKING-Erweiterung __at() und laesst sich
 * auf dem Host nicht uebersetzen; hier steht deshalb nur das Objekt selbst,
 * ohne DFLASH und ohne Kommandoverarbeitung.
 *
 * The identity calibration (zero offset, unit scale, zero declination) is what
 * a freshly provisioned board has, so it is the default a test sees unless it
 * writes something else.
 */
#include "Nvm.h"

volatile Xcp_Nvm g_xcpNvm;

void NvmFake_identity(void)
{
    g_xcpNvm.magic      = 0x4D564E58u;
    g_xcpNvm.command    = NVM_CMD_NONE;
    g_xcpNvm.userValue  = 0u;
    g_xcpNvm.seaLevelPa = 101325u;
    g_xcpNvm.magOffX    = 0.0f;
    g_xcpNvm.magOffY    = 0.0f;
    g_xcpNvm.magOffZ    = 0.0f;
    g_xcpNvm.magScaleX  = 1.0f;
    g_xcpNvm.magScaleY  = 1.0f;
    g_xcpNvm.magScaleZ  = 1.0f;
    g_xcpNvm.magDeclDeg = 0.0f;
}

void    Nvm_bootInit(void)  { NvmFake_identity(); }
void    Nvm_task100ms(void) { }
boolean Nvm_hasFault(void)  { return FALSE; }
