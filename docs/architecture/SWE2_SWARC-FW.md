# Firmware Software Architecture — SWARC-FW (ASPICE SWE.2)

**ASPICE:** SWE.2 — software architecture, AurixTricore firmware · realizes
SYS-TIM-001/002, SYS-NAV-001/002, SYS-IF-001, SYS-COM-001 (SAF/ACT/CTRL
items pending design) · process: QuadSE/requirements/README.md

Owned by `flight-architect`. This file allocates modules and budgets; the
per-module truth lives in the linked SWE.3 docs and the headers themselves.

## Change log

- 2026-08-29 — created as-built (post PR #15 core partition); collects what
  `REFACTORING_PLAN.md` (now archive) established.

## Layering (load-bearing, CLAUDE.md rule)

```
src/asw/   application (flight_ctrl, CtrlReplay, estimator-to-come)
   ↓ calls BSW only — never iLLD
src/bsw/   base software — owns hardware + generic services
   ↓
Libraries/iLLD, Infra   vendor (frozen, never edited)
```

BSW never includes an ASW header. Testable logic sits behind seams reached
by host tests (`test/`, fakes = minimal surfaces, `unit_tests.yml`).

## Core & task allocation

| Core | Role | Modules (entry: `src/bsw/CpuN_Main.c`) |
|---|---|---|
| CPU0 | comms & services | Ethernet/lwIP, `Echo.c`/`UdpEcho.c`, `Xcp.c` (XCP-on-UDP 5555, DAQ), `Measurements.c`, `Diagnostics.c` (100 ms), `Nvm.c` (DFLASH ping-pong), I²C sensors (blocking, ~9.4 % = wire time), `Uart.c`, GNSS decode consume |
| CPU1 | flight core | IMU DRDY ISR (ERU ch0, SRPN 106, P10.7) → SPI burst (124.4 µs) → `NavTask_step`: `Ahrs.c` (Mahony) + `fusion.c` (3× 4-state KF) at **1014.2 Hz**, budget **300 µs**, measured worst case 170 µs |
| CPU2–5 | sync + reserve | emit/wait sync only (all six cores, or watchdog reset) |

Scheduler: cooperative, `SCHEDULER_MAX_TASKS 8` per core, own STM per core
(`scheduler.c`). An overrunning task steals from everything after it — new
periodic work must state its budget.

## Cross-core communication (the one pattern)

LMU `0xB00F0000`, non-cached, single writer per block, 32-bit fields,
payload → `dsync` → generation counter increment (`src/bsw/SharedRam.h`).
`CoreStats.h` is diagnostics-only. Do not invent a second pattern.
One `__at()` per TU (cppcheck loses the symbol table otherwise).

## Module → detailed design (SWE.3)

| Module | Files | SWE.3 doc |
|---|---|---|
| Estimator (attitude + nav) | `Ahrs.c/.h`, `fusion.c/.h`, `FusionCal.h` | `docs/FUSION.md` (READ FIRST), spec `docs/FUSION_SPEC.md`, tuning `docs/NAV_TUNING.md` |
| IMU driver + DRDY chain | QSPI0 driver, ERU/ISR | `docs/ICM42688P.md`, `docs/IMU_INTERRUPT.md` |
| Baro / Mag drivers, I²C engine | in-house bounded I²C transfer engine | `docs/BMP581.md`, `docs/MMC5983MA.md` |
| GNSS | ASCLIN4 ISR → 512 B ring → `GnssM9N_decode` | `docs/GNSS_UBX.md` |
| XCP / measurements / cal / NVM | `Xcp.c`, `Measurements.c`, `Diagnostics.c`, `Nvm.c` | `docs/DIAGNOSTICS.md`; A2L contract `docs/CODEMAP.md` + `tools/gen_a2l.py` |
| Replay harness | `src/asw/CtrlReplay.c` (UDP 5556) | `docs/CTRL_REPLAY.md` (SWE.5/6 harness) |
| Memory placement | `Lcf_Tasking_Tricore_Tc.lsl` | `docs/MEMORY_PLACEMENT.md` |
| iLLD usage rules | — | `docs/ILLD_NOTES.md` (trap list, READ FIRST for iLLD work) |

## Standing constraints

MISRA C:2012 gate (new code clean); TASKING packs `uint32` at 2-byte
alignment — struct layout changes ripple into A2L + GUI (`CODEMAP.md`);
NaN compares false — `isfinite()` guards on every non-finite-capable path;
no unbounded wait (iLLD spins hung CPU0 before — bounded in-house engines).

## Open architectural work (waiting on requirements)

Disarm/failsafe path (SYS-SAF-001/002 — mechanism is a user decision),
motor interface (SYS-ACT-001 — ESC/protocol open), altitude/position hold
loop closure on target (SYS-CTRL-001/002).
