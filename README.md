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
AurixTricore/
├── Configurations/
│   ├── Ifx_Cfg_Ssw.c       # Startup software configuration (PLL init, PMS init)
│   ├── Ifx_Cfg_Ssw.h       # Startup software defines
│   ├── Ifx_Cfg_SswBmhd.c   # Boot mode header
│   └── Debug/
│       └── sync_on_halt.c  # Debugger sync configuration
├── Libraries/
│   ├── iLLD/               # Infineon Low-Level Driver (auto-generated)
│   └── Infra/              # Startup software (SSW)
├── Cpu0_Main.c             # Core 0 — phase master, blinks D306 (P20.11)
├── Cpu1_Main.c             # Core 1 — blinks D307 (P20.12), shared with CPU5
├── Cpu2_Main.c             # Core 2 — blinks D308 (P20.13)
├── Cpu3_Main.c             # Core 3 — blinks D309 (P20.14)
├── Cpu4_Main.c             # Core 4 — borrows D306 (P20.11) during phase 1
├── Cpu5_Main.c             # Core 5 — borrows D307 (P20.12) during phase 1
├── Uart.h                  # UART helper API (ASCLIN0 / X109)
└── Uart.c                  # Blocking UART init + print via ASCLIN0
```

---

## Implementation

### LED Mapping

The TC399 TriBoard has four user LEDs (D306–D309). All six cores are given LED
health indication via a **time-division multiplexing** scheme: CPU0–3 blink their
own dedicated (or borrowed) LED during phase 0; CPU4–5 borrow two of the same
LEDs during phase 1.

| LED | Pin | Phase 0 owner | Phase 1 owner |
|---|---|---|---|
| D306 | P20.11 | CPU0 (dedicated) | CPU4 (borrowed) |
| D307 | P20.12 | CPU1 (dedicated) | CPU5 (borrowed) |
| D308 | P20.13 | CPU2 (dedicated) | — (off) |
| D309 | P20.14 | CPU3 (dedicated) | — (off) |

All LEDs are **active-low** (pin LOW = LED on).

### Blink Pattern

One full cycle lasts **1 800 ms** and repeats continuously:

```
|← 500 ms →|←── 300 ms ──→|← 500 ms →|← 500 ms →|
  CPU4+5 ON    all dark      CPU0–3 ON   all dark
  (D306+D307)  (pause)       (D306–D309)
```

| Window | Duration | Visible |
|---|---|---|
| Phase 1 — CPU4+5 ON | 500 ms | D306 + D307 lit |
| Grace pause (all dark) | 300 ms | All 4 LEDs off (visible gap between groups) |
| Phase 0 ON — CPU0–3 | 500 ms | D306 + D307 + D308 + D309 lit |
| Phase 0 OFF — CPU0–3 | 500 ms | All 4 LEDs off |

**Pin-sharing rules** — only the active owner writes to a shared pin; the idle
core polls `g_ledPhase` without touching GPIO. CPU0 is the phase timing master
and writes `g_ledPhase` (declared `volatile uint32`). The 300 ms grace period
ensures the phase-1 cores finish their LED_OFF handover write before the
phase-0 cores assert LED_ON on the shared pins.

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

### UART Debug Output

A blocking UART wrapper (`Uart.h` / `Uart.c`) is provided for runtime debug output
over **ASCLIN0**, routed to the onboard **X109 USB-to-UART** connector.

| Setting | Value |
|---|---|
| Peripheral | ASCLIN0 |
| TX pin | P14.0 (`IfxAsclin0_TX_P14_0_OUT`) |
| RX pin | P14.1 (`IfxAsclin0_RXA_P14_1_IN`) |
| Connector | X109 (appears as a COM port on the PC) |
| Baud rate | 115 200, 8N1, no flow control |

On reset, each core prints its startup message (e.g. `CPU0 started`) once it passes
the multi-core sync barrier. Connect any serial terminal (PuTTY, Tera Term, …) to the
X109 COM port at 115 200 baud to observe the output.

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
