# CLAUDE.md — AurixTricore

Bare-metal multi-core blinky on the Infineon AURIX TC399 (6x TriCore, 300 MHz).
Uses iLLD for GPIO and STM timer. No RTOS, no HAL beyond iLLD.

---

## Build & Flash

| Action | How |
|---|---|
| Build | `Project → Clean`, then `Ctrl+B` in AURIX Development Studio (ADS) |
| Expected output | `Build Finished. 0 errors, 0 warnings.` |
| Flash | AURIX Flasher button in ADS toolbar (board connected via Micro-USB DAP) |
| Debug | `Debug Configurations → TASKING winIDEA → Debug`, then `F8` to run |

Build config: **TriCore Debug (TASKING)** — outputs `AurixTricore.elf`.

---

## Project Layout

```
Cpu0_Main.c          Core 0 — blinks D306 (P20.11) via STM0
Cpu1_Main.c          Core 1 — blinks D307 (P20.12) via STM1
Cpu2_Main.c          Core 2 — blinks D308 (P20.13) via STM2
Cpu3_Main.c          Core 3 — blinks D309 (P20.14) via STM3
Cpu4_Main.c          Core 4 — sync + idle (no spare LED)
Cpu5_Main.c          Core 5 — sync + idle (no spare LED)
Configurations/      PLL init, boot mode header, startup software
Libraries/iLLD/      Infineon Low-Level Driver (do not edit)
Libraries/Infra/     SSW / SFR infrastructure (do not edit)
```

---

## Key Technical Rules

**1. Always use `IfxPort_State` enum — never raw booleans**
`IfxPort_setPinState` writes directly to the OMR register (separate SET/CLEAR bit fields).
```c
// WRONG — FALSE=0 means "no change", TRUE=1 means "set high"
#define LED_ON  FALSE
// CORRECT
#define LED_ON  IfxPort_State_low   // active-low board: LOW = LED on
#define LED_OFF IfxPort_State_high
```

**2. Each core uses its own STM module**
`MODULE_STM0` → CPU0, `MODULE_STM1` → CPU1, … `MODULE_STM5` → CPU5.
Cross-core STM access works but is unnecessary and adds bus dependency.

**3. STM runs at ~100 MHz after PLL init**
`50 000 000 ticks ≈ 500 ms`. PLL init is automatic via `IFX_CFG_SSW_ENABLE_PLL_INIT = 1`.

**4. Multi-core sync must involve all six cores**
Every `coreN_main` must call both `IfxCpu_emitEvent` and `IfxCpu_waitEvent` before any
application code. Skipping either on any core causes a watchdog reset after the timeout.

**5. Both watchdogs must be disabled**
CPU watchdog: `IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword())`
Safety watchdog (CPU0 only): `IfxScuWdt_disableSafetyWatchdog(...)`

**6. `DEVICE_TC39XB` must be defined**
Set in the TASKING compiler preprocessing symbols. Without it, `IfxPort.c` takes the wrong
`#else` branch and GPIO does not work.

---

## Git Workflow Rules

**Never run `git commit` or `git push` unless the user explicitly asks.**
Make code changes, describe what was done, and wait. Only commit/push when
the user gives a direct instruction such as "commit", "push", or "commit and push".

---

## Commit Message Style Guide

This project follows **Conventional Commits** (`type(scope): description`).

### Types

| Type | Use for |
|---|---|
| `feat` | New application behaviour (new LED pattern, new peripheral) |
| `fix` | Correcting a bug (wrong pin, wrong timing, wrong state enum) |
| `refactor` | Code restructuring with no behaviour change |
| `perf` | Timing or efficiency improvement |
| `chore` | Build system, toolchain config, linker script changes |
| `docs` | README, CLAUDE.md, code comments only |
| `style` | Formatting, naming — no logic change |
| `test` | Adding or updating debug/test code |

### Scopes

| Scope | Covers |
|---|---|
| `cpu0` … `cpu5` | Changes to a specific core's main file |
| `port` | GPIO / IfxPort driver usage |
| `stm` | Timer / IfxStm usage |
| `sync` | Multi-core synchronisation |
| `wdt` | Watchdog configuration |
| `build` | `.cproject`, linker scripts, compiler flags |
| `config` | `Configurations/` files (PLL, SSW, BMHD) |
| `docs` | Documentation files |

Scope is optional for changes that span multiple areas.

### Rules

- **Subject line**: imperative mood, lowercase after the colon, no trailing period, ≤ 72 chars
  `feat(cpu1): add blinking LED on P20.12 via STM1`
- **Body** (optional): wrap at 72 chars; explain *why*, not just *what*
- **Breaking changes**: add `!` after the type/scope and a `BREAKING CHANGE:` footer
  `refactor(sync)!: remove cpuSyncEvent — all cores must be updated together`
- **Co-author line**: always include when Claude Code writes the commit
  `Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>`

### Examples

```
feat(cpu2): add blinking LED on P20.13 via STM2

Assigns D308 to CPU2 so core health is visible on the board.
Uses MODULE_STM2 to keep timing independent of other cores.

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
```

```
fix(port): use IfxPort_State enum instead of boolean for LED state

FALSE (= 0) maps to IfxPort_State_notChanged, leaving the pin
unchanged rather than turning the LED on. Use IfxPort_State_low
and IfxPort_State_high explicitly.
```

```
chore(build): enable -O1 optimisation for release config
```

```
docs: update README LED mapping table for CPU1–CPU3
```
