# CODEMAP — where things live, and what a change touches

Navigation aid, not an authority. It exists for the one thing `grep` cannot do:
follow a chain **whose name changes at each hop** (a pin → a driver → an
`Xcp_Data` field → an A2L entry → a GUI plot).

For anything lexical — "where is `X` defined", "what calls `Y`" — just grep.
`src/` is 56 files / ~8k lines; search is faster and always correct.

**Staleness policy:** these are pointers, so a wrong one costs a redundant grep.
Verify before relying on a claim here. If you find drift, fix the line.

---

## 1. Where things live

| Subsystem | Files | Notes |
|---|---|---|
| Core entry | `Cpu0_Main.c` … `Cpu5_Main.c` | CPU0 does everything; **CPU1–5 idle at 0.00%** |
| Scheduling | `scheduler.c/.h` | cooperative, `SCHEDULER_MAX_TASKS 8` per core |
| Timing | `SysTime.c` | BSW time service (ASW must use this, not iLLD) |
| XCP slave | `Xcp.c` | UDP 5555: poll, cal writes, DAQ |
| Measurements | `Measurements.c/.h` | **`Xcp_Data` @ `0x70030000`** — offset map in the header |
| Diagnostics | `Diagnostics.c/.h` | `DIAG_*` bits + **`Xcp_Cal` @ `0x70030100`** (RAM only) |
| Peripheral health | `PeriphDiag.c/.h` | per-device fault classification → diag bits |
| Persistence | `Nvm.c/.h` | **`Xcp_Nvm` @ `0x70030200`**, DFLASH0 ping-pong + CRC |
| I2C | `I2c.c` (697 lines) | in-house bounded engine — **not** iLLD `read2`/`write2` |
| SPI | `Spi.c` | QSPI0, ICM-42688-P |
| Sensors | `Bmp581.c`, `Mmc5983.c`, `Icm42688.c` | fitted |
| Sensor pool | `Bmp388.c`, `Mpu6050.c` | compiled, **never called** — MISRA 8.7 fires, held with deviations |
| Fusion | *(none — removed 2026-08-22)* | to be rewritten; see docs/SENSOR_FUSION.md |
| GPIO / PWM | `gpio.c`, `gpio_cfg.h`, `led.c` | per-pin config; GTM TOM PWM |
| Networking | `Echo.c`, `UdpEcho.c`, `EthStats.c` | TCP+UDP echo port 7 |
| Per-core stats | `CoreStats.c/.h` | **the inter-core pattern to copy** — single writer/slot, no locks |
| ASW | `src/asw/flight_ctrl.c`, `CtrlReplay.c` | calls BSW only, never iLLD |

Reference docs: `ILLD_NOTES.md` (vendor driver traps), `PINNING.md` (pin SSoT),
`DIAGNOSTICS.md` (bit meanings), `GNSS_UBX.md` (READ FIRST for any UBX work —
frame format, config keys, NAV-PVT offsets), per-peripheral notes, `readpdf`
skill for PDFs.

---

## 2. Task → touch points

### Add a field to `Xcp_Data` (a new measurement)

1. `Measurements.h` — **add to the offset comment block AND the struct.** The
   comment block is the layout SSoT; keep 4-byte alignment (pad with
   `uint8 xReserved[3]` after a lone `uint8`).
2. `Measurements.c` — populate it.
3. `docs/AurixTricore.a2l` — add a `MEASUREMENT` entry at the new offset.
4. **AurixGUI (separate repo)** — memory records **3 edits** needed there.
5. Bump `Version.h`, then **verify over XCP** — `amk` may not rebuild every
   includer (see `amk-stale-intermediates`: delete `.src` + `.o` + `.d`).

Addresses are fixed via TASKING `__at`, so the A2L does not depend on the link.
Appending is safe; **inserting shifts every later offset** and silently breaks
every A2L entry and GUI plot below it. Append.

### Add a sensor / peripheral

1. Driver `src/bsw/<Chip>.c/.h`, modelled on `Bmp581.c` (I2C) or `Icm42688.c` (SPI).
2. `PeriphDiag.h` — new `PERIPH_DIAG_*` enum member before `PERIPH_DIAG_COUNT`.
3. `Diagnostics.h` — **4 diag bits** (`NO_RESPONSE` / `TIMEOUT` / `STUCK_DATA` /
   `IMPLAUSIBLE`). ⚠️ **The word is full** — bits 27–30 went to the GNSS, 31 is
   `DIAG_CAL_INVALID`. A fifth peripheral needs a second bitmask word.
   A fifth device needs a second bitmask word — see `Diagnostics.h:99`.
4. `Cpu0_Main.c` — `<Chip>_init()` in the init block, `PeriphDiag_setFitted()`,
   and `Scheduler_addTask(&g_sched, Task_<Chip>, SCHED_MS(20u))` (~line 411).
   Budget: 8 tasks max per core.
5. Measurement fields → follow the chain above.
6. `docs/PINNING.md` — claim the pins **before** wiring. Read the whole section
   first; it is the SSoT and contradicting it has cost time before.
7. `docs/<CHIP>.md` — the peripheral note.
8. `.cproject` — un-exclude the iLLD dir if it's a new bus (`SCR|MCS|HSM` are
   the only current exclusions), and use **root-relative** iLLD includes.

### Add a diagnostic bit

`Diagnostics.h` (`#define DIAG_*`) → the check in `Diagnostics.c` →
`docs/DIAGNOSTICS.md` → the GUI's `BIT_MASK` lines (`DIAGNOSTICS.md:55` says
these move together). Bit budget as above.

### Add a calibration parameter (RAM-only, lost on reset)

`Diagnostics.h` `Xcp_Cal` struct → default in `Diagnostics.c` → A2L
`CHARACTERISTIC` → GUI *Ethernet → Kalibrierung* tab. Never lands in flash.

### Add a persistent parameter (survives reset)

`Nvm.h` `Xcp_Nvm` struct → `Nvm.c` (CRC covers the block, so a size change
invalidates stored data — handle the migration) → A2L. **Only `Xcp_Nvm` lives in
DFLASH; the cal block is deliberately RAM-only.** Validate with
`tools/nvm_test.py`.

### Add a periodic task

`Scheduler_addTask(&g_sched, fn, SCHED_MS(n))` in the owning `CpuN_Main.c`.
Max 8/core. Execution time is measured per core into `coreExecUs` /
`coreLoadPmil` — a long task shows up there, and `coreAlive` freezing means that
core hung.

### Put work on another core

`CoreStats.h` documents the working pattern — single writer per slot, no locks,
cross-core DSPR reads bypass the cache. **Reuse it, don't reinvent.** Every
`coreN_main` must still call `IfxCpu_emitEvent` + `IfxCpu_waitEvent` before
application code, or the watchdog resets the board.

### Release

`Version.h` (`SW_VERSION_MAJOR/MINOR/STEP`) → rebuild → **confirm the version
string over XCP**, because a stale `.src` will happily ship the old one.

---

## 3. Fixed memory map

| Block | Address | Backing | Defined in |
|---|---|---|---|
| `Xcp_Data` | `0x70030000` | RAM | `Measurements.h` |
| `Xcp_Cal` | `0x70030100` | RAM only | `Diagnostics.h` |
| `Xcp_Nvm` | `0x70030200` | DFLASH0 | `Nvm.h` |
| `Xcp_Gpio` | `0x70030300` | RAM only | `gpio.h` |

Blocks are 256 bytes apart. `Xcp_Data` is **full**: `GnssVAccuracy` sits at `0xFC`
and ends on the 256-byte boundary, so the next appended field collides with
`Xcp_Cal`. Formerly reached `0xCC` (204 B) —
**~52 bytes of headroom before it collides with `Xcp_Cal`.**

---

## 4. Known drift ⚠️

**Resolved.** `docs/AurixTricore.a2l` used to sit at v1.3.0 for fourteen releases
while the structs moved on, and the failure was silent — the GUI computed
`ECU_ADDRESS - block base` and read the wrong bytes, which is how the magnetometer
read a constant 0 for a while.

It is now generated from the C structs by `tools/gen_a2l.py`, and
`.github/workflows/a2l.yml` runs `gen_a2l.py --check` on every push, so a stale
file fails CI instead of silently misreading. The committed file is at v1.18.0 and
matches the structs.

⚠️ On Windows, `gen_a2l.py --check` reports a false positive locally: it compares
the committed file with universal-newline translation against generated CRLF text,
so every line looks changed. CI checks out LF and passes. Trust the CI result.

---

## 5. Not covered here

Call graphs, symbol locations, file inventories — **grep**. This file is only for
chains that cross artifact boundaries (code ↔ A2L ↔ GUI ↔ docs ↔ hardware). If
you find yourself wanting to add a "what calls what" section, don't; it will rot
and grep already answers it correctly.
