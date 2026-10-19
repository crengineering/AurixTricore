# DShot setpoint ceiling — design (SWE1-FW-025)

Chain: `SYS1-012 → SYS2-PWR-001 → SWE1-FW-025` and `SYS1-012 / SYS1-005 → SYS2-ACT-001 → SWE1-FW-025`.
Decision: dispatch SYS1-012 §12 (Chris, 2026-10-18) — ceiling sp **1600**, calibration, rig-only raise to 2047.
Second task (CtrlReplay mirror): `SYS1-012 → SYS2-PWR-001 → SWE1-MDL-001` (layer 1, flight-control's item).
Status: **draft for gate 2** — nothing implemented. Base: AurixTricore `main` @ `e19a7e3`.

## 1. Verdict

The only throttle bound on the wire today is the DShot protocol top: `Motor_calc_clamp`
(`src/bsw/motor.c:75-95`) clamps to 48..2047 and nothing else. The manual path copies
`g_xcpCal.motorManualSp[]` straight into the clamp (`motor.c:132`, `:138`), so any Motors-tab
slider value up to 2047 reaches the ESC; rig run 5 puts 2047 at 17,6–19,7 A per motor =
57–79 A for four, against the 50 A criterion of SYS2-PWR-001. The future flight-controller
path (`motor_speed_rqst[]`, `motor.c:136`, fed with zeros today from `Cpu0_Main.c:76`) goes
through the same clamp, so one change there covers both sources. Separately, the PIL mirror
of the mixer limit is the sweep top of run 3, not a design value
(`src/asw/CtrlReplay.c:84`, `782.0f`).

## 2. Target architecture

| Element | Owner / core | Change | Why the boundary |
|---|---|---|---|
| `src/bsw/motor.c` | BSW, CPU0, `Task_Dshot` 1 ms | `Motor_task` validates the cal + writes back; `Motor_calc_clamp` gets one more branch (`> ceiling`) | the state machine is the single point every throttle source passes; host-tested through `Motor_task` (`test/test_motor.c`, fakes) |
| `Xcp_Cal.motorSpMax` | `Diagnostics.h/.c` | new `uint16` at `0x50`, default 1600 | RAM-only cal block = reset restores the default by construction |
| A2L `CAL_motorSpMax` | `tools/a2l_meta.json` → `gen_a2l.py` | new CHARACTERISTIC 48..2047 | GUI is A2L-driven |

No new task, no new core, no inter-core data. Beacons (INIT, values 1..5, `motor.c:41-73`,
`:109`) stay outside the ceiling, as SWE1-FW-022 requires.

### 2.1 Field

```
Xcp_Cal (0x70030100, RAM only)
  0x48  uint16  motorManualSp[4]   (unchanged)
  0x50  uint16  motorSpMax         NEW  DShot throttle ceiling, 48..2047, default 1600
  0x52  uint16  motorSpReserved    NEW  explicit pad -> sizeof 0x54 on TASKING and host GCC alike
XCP_CAL_SIZE 80u -> 84u
```

Collision check: `Xcp_Cal` ends at `0x50` today (`LayoutAssert_gen.h:124`, `sizeof == 0x50`);
the slot runs to `0x70030200` (`LCF_XCP_NVM_START`, `Lcf_Tasking_Tricore_Tc.lsl:170`), so
84 of 256 bytes are used after the change. Existing offsets do not move — appending only.
The pad is load-bearing: without it TASKING (uint32 2-aligned) gives `sizeof` 0x52, GCC 0x54.

### 2.2 Clamp — order of operations (per motor, per 1 ms frame)

```c
/* motor.c, Motor_calc_clamp: the existing chain of main, one branch added.
 * sp == 0      -> command untouched (caller zero-inits, as before)
 * sp <  48     -> 48          protocol floor
 * sp > ceiling -> ceiling     NEW (ceiling is validated 48..2047, so 48/2047 still hold)
 * sp > 2047    -> 2047        protocol top (kept)
 */
```

> Amended at gate 2 (Chris, 2026-10-19): default is a calibration default in Diagnostics
> (`DIAG_DEFAULT_MOTOR_SP_MAX 1600u`, `Diagnostics.h`). Amended at gate 2 (Chris,
> 2026-10-19): no separate module, clamp stays in `motor.c`, host test through `Motor_task`
> with fakes.

Validation is the first thing in `Motor_task` and keeps the ceiling in 48..2047, so the
existing 48 floor / 2047 top cannot be undone by the ceiling:

```c
if ((g_xcpCal.motorSpMax < DSHOT_CMD_THROTTLE_MIN) || (g_xcpCal.motorSpMax > DSHOT_CMD_THROTTLE_MAX))
{
    g_xcpCal.motorSpMax = DIAG_DEFAULT_MOTOR_SP_MAX;   /* read-back = value in force */
}
... Motor_state_armed(..., g_xcpCal.motorSpMax);       /* one read, all four motors */
```

- One read of the cal per `Motor_task` call → all four motors of a frame use the same ceiling.
- `motorManualSp[]` is never rewritten by the clamp — the Motors tab's read-back
  confirmation (SWE1-GUI-011) stays valid.
- The write-back is safe without a lock: XCP DOWNLOAD is served by `Task_Lwip` and
  `Task_Dshot` runs in the same cooperative scheduler on CPU0 (`Cpu0_Main.c:164`, `:180`).
- `motor.c`'s private `DSHOT_CMD_THROTTLE_MIN/MAX` (`motor.c:18-19`) stay as on main.

### 2.3 Default and reset

`diag_loadDefaults()` (`Diagnostics.c:34-58`) gains `g_xcpCal.motorSpMax = DIAG_DEFAULT_MOTOR_SP_MAX;`
and `motorSpReserved = 0u`. It runs at boot (`diagnosticsInit`, `Cpu0_Main.c:184`) and on a
garbled cal magic (`Diagnostics.c:139-143`). The block is `.bss` in RAM (`Diagnostics.c:12-18`),
never in `Xcp_Nvm`/DFLASH — **a 2047 cannot survive a reset**. Even before `diagnosticsInit`
(`.bss` = 0), the validation in `Motor_task` yields 1600 on the first call.

Not chosen: reverting the ceiling to 1600 on every exit from ARMED (like `motorManualSp[]`).
Defensive, but a link blip on the rig would silently drop the ceiling mid-session; the
decided rule is "reset restores". Open at gate (§7).

## 3. Timing

| Task | Rate | Core | Added WCET | Overrun |
|---|---|---|---|---|
| `Task_Dshot` (`Motor_task` + `Dshot_task`) | 1 ms | CPU0 | ~10 compares + 1 load/store, < 0,2 µs | none new — bounded, no loop beyond the existing 4-motor loop |

The ceiling takes effect in the next frame (≤ 1 ms). Lowering it while armed at a higher
setpoint is a thrust step in one frame — rig only, intended.

## 4. Interfaces

| Header | Change |
|---|---|
| `src/bsw/Diagnostics.h` | `motorSpMax`, `motorSpReserved` appended; `XCP_CAL_SIZE` 84u; layout comment `0x50`/`0x52`, "84 bytes" |
| `src/bsw/motor.h` | unchanged (`Motor_task` signature unchanged) |

Host-testable: `test/test_motor.c` #includes `motor.c` and drives the public `Motor_task()`
(needs the `Ifx_Types`/`IfxPort` fakes; `g_xcpCal` and `Xcp_linkAlive()` are stubbed in the
test): arming sequence with beacons (first host test of SWE1-FW-019), the whole 16-bit
setpoint range per ceiling, validation read-back, `motorManualSp[]` not rewritten, DISARM.

## 5. Risks

- **A2L/GUI contract:** append-only, no existing address moves. New A2L on old firmware:
  `CAL_motorSpMax` reads slot garbage and writes return `ERR_WRITE_PROTECTED` (old
  `XCP_CAL_SIZE`) — GUI must treat a missing characteristic as "no ceiling known".
- **Forgotten `XCP_CAL_SIZE` bump** = every GUI write of the ceiling rejected (`Xcp.c:101`).
  Nothing asserts `XCP_CAL_SIZE == sizeof(Xcp_Cal)`; bench step B3 catches it.
- **Behaviour change for the rig:** the Motors tab tops out at 1600 by default. The rig
  procedure (QuadSE `evidence/thrust_rig_runs.md` protocol) must gain "write
  `CAL_motorSpMax` = 2047 first" — orchestrator's note, not firmware.
- **Board rule conflict:** `motor.c` arms/drives motors; QuadSE CLAUDE.md says such code is
  never flashed by an agent. Verification below is therefore ESC-unpowered (logic level)
  with flashing after Chris's explicit go (§7).
- **Ownership:** `motor.c` is Chris's file (crengineering header). §7.
- Fully reversible; no NVM layout change, no DFLASH write.

## 6. Ordered tasks (flight-dev)

Branches: `feat/dshot-ceiling` (T1–T6), `feat/w-lim-865` (T7, pairs with the Quadrocopter
branch of the same name). Each step builds; firmware behaviour is unchanged until T3.

| # | Files | Change | Acceptance |
|---|---|---|---|
| T1 | `test/test_motor.c` (new), `test/fakes/IfxPort.h` (new, empty), `test/fakes/Ifx_Types.h` (`sint8`), `test/CMakeLists.txt` | host test through `Motor_task` (no new module, amended at gate 2): DISARMED → INIT (beacons) → ARMED; ceilings {0, 47, 48, 1600, 2047, 2048, 65535} × sp 0..65535 on M1: 0 ⇔ sp 0, else 48 ≤ r ≤ min(max(ceiling,48),2047), monotonic; spot values (1600: 1599, 1600, 1601→1600, 2047→1600; 2047: 2047, 2048→2047); validation read-back 1600 for 0/47/2048/65535, itself for 48/1600/2047; `motorManualSp[]` not rewritten; DISARM zeroes | `python tools/utest.py` green incl. sanitize variant |
| T2 | `src/bsw/Diagnostics.h`, `Diagnostics.c`, `tools/a2l_meta.json`, regenerate `docs/AurixTricore.a2l` + `src/bsw/LayoutAssert_gen.h` | append `uint16 motorSpMax; uint16 motorSpReserved;`, `XCP_CAL_SIZE 84u`, header layout comment; default `DIAG_DEFAULT_MOTOR_SP_MAX` (1600u, `Diagnostics.h`) + reserved 0 in `diag_loadDefaults`; meta: `"motorSpMax": {"name": "CAL_motorSpMax", "desc": "DShot throttle ceiling for every setpoint source, 48..2047; default 1600 after every reset (50 A criterion, SYS2-PWR-001); raise to 2047 for thrust-rig runs only; out-of-range writes read back as 1600", "maxdiff": "2047", "lo": "48", "hi": "2047"}`, `"motorSpReserved": {"skip": true}` | `python tools/gen_a2l.py` then `--check` exit 0; build (layout asserts `0x50`, `0x52`, `sizeof 0x54`); `check_memmap` OK; MISRA clean |
| T3 | `src/bsw/motor.c` | minimal (amended at gate 2): `ceiling` parameter through `Motor_state_armed` → `Motor_calc_clamp`, one `else if (> ceiling)` branch in the existing chain, range check + write-back at the top of `Motor_task`; `DSHOT_CMD_THROTTLE_MIN/MAX` stay | builds; MISRA: `motor.c` stays at zero baseline entries; host tests green |
| T4 | `docs/DIAGNOSTICS.md` (table after `:200`, German like its rows), `docs/DSHOT.md` (new `## 8. FC-side setpoint chain` before `## Traps`: source → ceiling → 48/2047 → driver validation, beacons bypass), `docs/CODEMAP.md` (*Add a calibration parameter*, `:99-102`: add `tools/a2l_meta.json`, `gen_a2l.py` regenerates A2L + `LayoutAssert_gen.h`, bump `XCP_CAL_SIZE`), `src/bsw/Version.h` step bump | rows: `0x50 motorSpMax DShot 1600 Gas-Obergrenze für jede Sollwertquelle (48..2047), nach jedem Reset 1600, 2047 nur am Schubprüfstand; Werte außerhalb werden als 1600 zurückgeschrieben` and `0x52 motorSpReserved – 0 Füllwort` | `python tools/check_docs.py` exit 0 |
| T5 | — (bench, §6.1) | B0–B9 | evidence row proposed to the orchestrator (INDEX), results reported, not entered |
| T6 | — | report: chain, build/test/MISRA exit codes, bench table, `EscMnDshotCmd` values | flight-reviewer verdict |
| T7 | `src/asw/CtrlReplay.c:84` (branch `feat/w-lim-865`) | `782.0f` → `865.0f`, comment: `/* w_lim: rotor-speed ceiling from the 50 A criterion (SYS2-PWR-001), sp 1600 on a full pack = 8257 rpm, thrust rig run 5 2026-10-18, evidence 4829A25237BFB7C8; NOT the motor top (w_motor_max 1080 rad/s) */`. Struct field stays `w_max` (`flight_ctrl.h:36` mirrors the Quadrocopter C file — rename only if flight-control renames it there) | builds; `tools/ctrl_replay_test.py` equivalence against reference vectors regenerated by flight-control on Quadrocopter `feat/w-lim-865` (until those exist, saturating vectors are expected to differ — do not merge before) |

### 6.1 Bench verification (T5)

Board rules (QuadSE `plans/PLAN-001-agents.md` §4): pin map untouched → hardware-safe build,
but `motor.c` is motor-driving code → **flash only after Chris's explicit go** (from the main
session per standing practice). GUI disconnected, one XCP master. **ESC power off / no
battery for B1–B8** — verification is on the DShot logic level (`EscMnDshotCmd` + Nano 33 IoT
capture on COM3; check it runs the analyzer sketch, not the AM32 writer). No propeller ever.
After the reflash Chris restores the mag calibration (`python tools/mag_cal.py
--restore-from calibration/board.json`, NVM write = Chris).

| Step | Action | Pass |
|---|---|---|
| B0 | flash, `tools/xcp_verify_version.py` | new version reported |
| B1 | read `CAL_motorSpMax` after boot | 1600 |
| B2 | ARM, `CAL_motorManual` = 1, `CAL_motorManualSp_M1..M4` = 2047; also 1599, 1601, raw 47, 0 on M1 | `EscMnDshotCmd` = 1600 (all four); 1599→1599, 1601→1600, 47→48, 0→0; capture throttle field = 1600 at sp 2047; `CAL_motorManualSp_Mn` reads back 2047 (not rewritten) |
| B3 | write `CAL_motorSpMax` = 2047 (XCP accepted, read-back 2047), sp 2047 | `EscMnDshotCmd` = 2047 |
| B4 | raw writes 0, 47, 2048, 65535 to `CAL_motorSpMax` | each reads back 1600 within one poll; `EscMnDshotCmd` = 1600 at sp 2047 |
| B5 | ceiling 1000 while armed at sp 1500 | `EscMnDshotCmd` = 1000 in the next DAQ sample |
| B6 | DISARM | frames 0, `motorManualSp` zeroed (SWE1-FW-019), `motorSpMax` unchanged |
| B7 | set 2047, reset the board | `CAL_motorSpMax` = 1600 |
| B8 | `python tools/killchain_test.py` unchanged | passes as recorded for SWE1-FW-024 |
| B9 (Chris, optional) | ESC powered, motor on channel 3, **no prop**, existing manual path only: sp 2047 with ceiling 1600, then ceiling 2047 | `EscM3BidirRpm` at the sp-1600 level, rises only after the raise |

## 7. Open for gate 2

1. **Who edits `motor.c`** — Chris's own file. Recommendation: flight-dev, change confined
   to `Motor_task`'s first two lines and `Motor_calc_clamp`'s body; Chris reviews the diff.
2. **Flash permission** — the dispatch lets flight-dev flash; QuadSE CLAUDE.md forbids agents
   flashing motor-driving code. Recommendation: flash after Chris's go, ESC unpowered for
   B1–B8; B9 is Chris's.
3. **Revert on exit from ARMED?** Recommendation: no — reset-only as decided (§2.3).
4. **Out-of-range write → 1600 (default), not saturation into 48..2047.** A raw 3000
   meaning "full" yields 1600 (safe side); a raw 10 yields 1600, not idle (no thrust loss in
   flight). Recommended as specified.

## 8. Hand-off to aurix-gui (SWE1-GUI-011, after T2 lands an A2L with `CAL_motorSpMax`)

The Motors tab's slider and spinbox upper bound shall follow the read-back of
`CAL_motorSpMax` instead of the fixed 2047 of SWE1-GUI-011 §2 (STOP position 47 → 0 and
48 floor unchanged); a ceiling read-back is part of the tab's existing fresh-read rule. The
tab shows the ceiling in force next to the sliders and highlights it when it is not 1600.
Raising it is an explicit operator action — a field in the Motors tab with a confirmation
("thrust rig only — exceeds the 50 A pack criterion") when > 1600, written through the
normal calibration path and read-back-confirmed like the other `CAL_motor*` words; lowering
needs no confirmation. With an A2L that lacks `CAL_motorSpMax` (older firmware) the tab keeps
today's 2047 range and shows "ceiling: not reported". The firmware clamps regardless; the GUI
range only keeps the operator from asking for values that will not reach the wire.
SWE1-GUI-011 §2's text changes accordingly (aurix-gui's item).
