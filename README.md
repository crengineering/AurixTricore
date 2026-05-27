# AURIX TC399 Blinky – Getting Started

First embedded software project on the Infineon AURIX TC399 TriBoard (2nd Generation).  
Implements a simple LED blinking program (Blinky) using the iLLD Low-Level Driver library and the STM hardware timer.

---

## Hardware

| Component | Details |
|---|---|
| Board | AURIX TC3xx Starter Kit 2nd Gen (TriBoard TC3X9 V2.0) |
| Microcontroller | Infineon TC399XX-256, Silicon Revision B (Package Code: BD) |
| Package | LFBGA-516 |
| Cores | 6x TriCore (CPU0–CPU5), 300 MHz |
| Flash | 16 MB |
| User LEDs | D306–D309 (blue), connected to P20.11–P20.14 (active-low) |
| Debug Interface | Onboard DAP (USB, no external debugger required) |

---

## Toolchain

| Component | Version |
|---|---|
| IDE | AURIX Development Studio (ADS) 1.10.32 |
| Compiler | TASKING VX-toolset for TriCore (non-commercial), v1.1r8 |
| Low-Level Driver | iLLD (Infineon Low-Level Driver), iLLD_1_20_0 |
| Debugger | TASKING winIDEA (integrated in ADS) |
| Flasher | AURIX Flasher Software Tool 3.0.16.0 |
| OS | Windows |

### Toolchain Notes

- ADS ships the TASKING toolchain in a **non-commercial** variant. There is no code-size restriction for debugging, but it is not licensed for commercial products.
- The toolchain is **configured automatically** by ADS when creating a new project.
- The project template used is **Empty Project with iLLD**.

---

## Project Configuration

### Device Define

For the iLLD to select the correct code path for the TC399, the following preprocessor define must be set:

```
DEVICE_TC39XB
```

Location in ADS:  
`Project → Properties → C/C++ Build → Settings → TASKING C/C++ Compiler → Preprocessing → Defined Symbols`

Without this define, the iLLD selects a wrong `#else` branch in `IfxPort.c` and GPIO configuration will not work correctly.

### Compiler Flags (set automatically)

```
-D__CPU__=tc39xb
-DDEVICE_TC39XB
-O0  (no optimization, debug build)
-Ctc39xb
```

---

## Project Structure

```
Blinky_TC300/
├── Configurations/
│   ├── Ifx_Cfg_Ssw.c       # Startup software configuration (PLL init, PMS init)
│   ├── Ifx_Cfg_Ssw.h       # Startup software defines
│   ├── Ifx_Cfg_SswBmhd.c   # Boot mode header
│   └── Debug/
│       └── sync_on_halt.c  # Debugger sync configuration
├── Libraries/
│   ├── iLLD/               # Infineon Low-Level Driver (auto-generated)
│   └── Infra/              # Startup software (SSW)
├── Cpu0_Main.c             # Core 0 main program (Blinky)
├── Cpu1_Main.c             # Cores 1–5 (watchdog disable, sync)
├── Cpu2_Main.c
├── Cpu3_Main.c
├── Cpu4_Main.c
└── Cpu5_Main.c
```

---

## Implementation

### LED Mapping

| LED | Pin | Active |
|---|---|---|
| D306 | P20.11 | LOW (active-low) |
| D307 | P20.12 | LOW (active-low) |
| D308 | P20.13 | LOW (active-low) |
| D309 | P20.14 | LOW (active-low) |

This Blinky program uses **D306 (P20.11)**.

### Key Findings

**1. Use `IfxPort_State` enum correctly**

`IfxPort_setPinState` writes directly into the **OMR (Output Modification Register)** of the TC399. This register has separate bit fields for setting (bits 0–15) and clearing (bits 16–31) pins. The `IfxPort_State` enum values must therefore be used explicitly:

```c
/* WRONG – does not work */
#define BLINKY_LED_ON   FALSE   /* = 0 → IfxPort_State_notChanged → pin stays unchanged */
#define BLINKY_LED_OFF  TRUE    /* = 1 → IfxPort_State_high → works by coincidence */

/* CORRECT */
#define BLINKY_LED_ON   IfxPort_State_low   /* active-low: LOW = LED on */
#define BLINKY_LED_OFF  IfxPort_State_high  /* active-low: HIGH = LED off */
```

**2. STM clock after PLL initialization**

The startup code (`Ifx_Cfg_Ssw.h`) initializes the PLL automatically (`IFX_CFG_SSW_ENABLE_PLL_INIT = 1`). The STM (System Timer) then runs at ~100 MHz (measured: 100,000,000 ticks ≈ 1 second).

For a 500 ms delay:
```c
IfxStm_waitTicks(&MODULE_STM0, 50000000u);
```

**3. Multi-core sync**

The TC399 has 6 cores. All cores must call `IfxCpu_emitEvent` and `IfxCpu_waitEvent`, otherwise the sync mechanism stalls. If the sync is removed in `core0_main` but kept in the other cores, the controller resets after the sync timeout.

**4. Watchdog**

Both watchdogs must be disabled:
```c
IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
```

---

## Full Source Code (Cpu0_Main.c)

```c
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "Ifx_Cfg_Ssw.h"
#include "IfxPort.h"
#include "IfxStm.h"

IFX_ALIGN(4) IfxCpu_syncEvent cpuSyncEvent = 0;

#define BLINKY_LED_PORT     &MODULE_P20
#define BLINKY_LED_PIN      11u                  /* P20.11 = D306 (blue) */
#define BLINKY_LED_ON       IfxPort_State_low    /* active-low: LOW = on */
#define BLINKY_LED_OFF      IfxPort_State_high   /* active-low: HIGH = off */

#define STM_TICKS_500MS     50000000u            /* ~500ms at ~100MHz STM clock */

int core0_main(void)
{
    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    IfxPort_setPinMode(BLINKY_LED_PORT, BLINKY_LED_PIN, IfxPort_Mode_outputPushPullGeneral);

    while (TRUE)
    {
        IfxPort_setPinState(BLINKY_LED_PORT, BLINKY_LED_PIN, BLINKY_LED_ON);
        IfxStm_waitTicks(&MODULE_STM0, STM_TICKS_500MS);
        IfxPort_setPinState(BLINKY_LED_PORT, BLINKY_LED_PIN, BLINKY_LED_OFF);
        IfxStm_waitTicks(&MODULE_STM0, STM_TICKS_500MS);
    }

    return 0;
}
```

---

## Build & Flash

### Build

```
Project → Clean
Ctrl+B
```

Expected build output:
```
Build Finished. 0 errors, 0 warnings.
```

### Flash

1. Connect the board to the PC via Micro-USB (DAP connector)
2. Flash using the AURIX Flasher button in the ADS toolbar
3. Success message: `AURIXFlasher Exit Status: Pass – Flashing was successfull`

### Debug

1. `Debug Configurations → TASKING winIDEA → Debug`
2. `F8` (Run) to the next breakpoint
3. `F6` (Step Over), `F5` (Step Into)

---

## Known Issues & Solutions

| Problem | Root Cause | Solution |
|---|---|---|
| `Cpu0_Main.c` not recompiled after changes | Incremental build does not detect the change | Run `Project → Clean` before building |
| `IfxPort_setPinMode` selects wrong code path | `DEVICE_TC39XB` not defined | Add define in Compiler Preprocessing settings |
| LED does not respond to `setPinState` | `FALSE`/`TRUE` used instead of `IfxPort_State_*` | Use `IfxPort_State_low` / `IfxPort_State_high` |
| Controller resets unexpectedly | Watchdog not disabled | Call `IfxScuWdt_disableCpuWatchdog` + `disableSafetyWatchdog` |
| `IfxCpu_waitEvent` stalls | Sync timeout expired | Increase timeout value or keep sync consistent across all cores |

---

## Resources

- [AURIX Development Studio Download](https://www.infineon.com/cms/en/tools/aurix-development-studio/)
- [TriBoard TC3X9 User Manual (ManualsLib)](https://www.manualslib.com/manual/1496611/Infineon-Triboard-Tc3x9-Th.html)
- [Infineon AURIX Community](https://community.infineon.com/t5/AURIX/bd-p/AURIX)
- [iLLD Documentation](https://www.infineon.com/cms/en/product/promopages/AURIX-microcontroller-boards/)
