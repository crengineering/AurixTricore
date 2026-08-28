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
| Core entry | `Cpu0_Main.c` … `Cpu5_Main.c` | **CPU1 is the flight core** (`NavTask_step`, T12), clocked at the IMU's own **1014.2 Hz DRDY edge** (T15, `docs/REFACTORING_PLAN.md` §3.6/§9) — never 1 kHz fixed, `dt` per tick; CPU0 does comms + sensors + housekeeping; **CPU2–5 idle** |
| Scheduling | `scheduler.c/.h` | cooperative, `SCHEDULER_MAX_TASKS 12` per core |
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
| Attitude | `Ahrs.c/.h` | quaternion Mahony over accel+gyro+mag — **`docs/FUSION.md`** |
| Navigation | `fusion.c/.h` | three 4-state KF channels (down/north/east) + **`Xcp_Fusion` @ `0x70030500`** |
| GPIO / PWM | `gpio.c`, `gpio_cfg.h`, `led.c` | per-pin config; GTM TOM PWM |
| Networking | `Echo.c`, `UdpEcho.c`, `EthStats.c` | TCP+UDP echo port 7 |
| Per-core stats | `CoreStats.c/.h` | diagnostic counters, tolerant of a torn read — **not** the cross-core pattern; see `SharedRam.h` |
| Cross-core shared state | `SharedRam.c/.h`, `NavState.c/.h`, `*Latch*.c/.h` | **the inter-core pattern to copy** — non-cached LMU alias, single writer, generation counter |
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

### IMU data-ready interrupt

`ImuInt.c`, `ConfigurationIsr.h` (new SRPN), `PINNING.md` §2.2/§5,
`ICM42688P.md` §3. **No A2L, no GUI, no `Xcp_Data`, no diag bit** — the
measurement is read by name with `tools/xcp_read.py`. See
`docs/IMU_INTERRUPT.md` for the full design; a rare case of a hardware change
that deliberately skips the "Add a field to `Xcp_Data`" chain above.

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

`SharedRam.h` documents the sanctioned pattern (`docs/REFACTORING_PLAN.md`
§2.4): shared objects live in the LMU at the non-cached alias
(`0xB00F0000` upward, see §3 below), every field `volatile` and 32-bit,
exactly one writer per object, publish ordered as *payload → `Ifx__dsync()` →
`gen++`*, reader does *read `gen` → copy → re-read `gen`*, one retry, then
keep the previous snapshot. **Reuse it, don't reinvent.** `CoreStats.h` is
diagnostics only now — tolerant of a torn read, not a template for a new
crossing (`docs/REFACTORING_PLAN.md` D6). Every `coreN_main` must still call
`IfxCpu_emitEvent` + `IfxCpu_waitEvent` before application code, or the
watchdog resets the board.

### Release

`Version.h` (`SW_VERSION_MAJOR/MINOR/STEP`) → rebuild → **confirm the version
string over XCP**, because a stale `.src` will happily ship the old one.

---

### Adding a fusion signal

`fusion.h` `FusionValues` (or `Ahrs.h` `Ahrs_Values`) → `Measurements.h`
`Xcp_Fusion` struct **and** its offset comment → `Measurements.c`
`measurementsSetFusion()` → `tools/a2l_meta.json` (name, description, unit,
limits) → `python tools/gen_a2l.py` → GUI.

⚠️ `gen_a2l.py` **warns but does not fail** on a struct field missing from the
sidecar — it emits a placeholder description so a new signal can never be
silently dropped. CI runs `gen_a2l.py --check`, which fails only if the
committed A2L does not match what the structs generate, so a bumped
`Version.h` alone will turn it red.

⚠️ Verify offsets against `TriCore Debug (TASKING)/src/bsw/Measurements.src`
after any struct edit, never against host `offsetof` — TASKING aligns `uint32`
to 2 bytes, so a `uint8` before a `uint32` shifts every later field.

### Changing a sensor's filter configuration

Register value in the driver (e.g. `BMP581_DSP_IIR_VAL`) → the measured noise
constant it invalidates (`FUSION_SIGMA_BARO`) → `docs/<PERIPHERAL>.md` register
table → `docs/FUSION.md` §2 bench numbers.

**A filter setting and a filter constant are one change, not two.** The Kalman
`R` values are *measured*, so altering what the sensor sends silently makes them
wrong. See `docs/FUSION.md` §8 for the case where this bit.

---

## 3. Fixed memory map

| Block | Address | Backing | Defined in |
|---|---|---|---|
| `Xcp_Data` | `0x70030000` | RAM | `Measurements.h` |
| `Xcp_Cal` | `0x70030100` | RAM only | `Diagnostics.h` |
| `Xcp_Nvm` | `0x70030200` | DFLASH0 | `Nvm.h` |
| `Xcp_Gpio` | `0x70030300` | RAM only | `gpio.h` |
| `I2c_Debug` | `0x70030400` | RAM only | `I2c.h` |
| `Xcp_Fusion` | `0x70030500` | RAM only | `Measurements.h` |
| `SharedRam` (LMU, non-XCP) | `0xB00F0000` | LMU, non-cached alias | `SharedRam.c`/`.h` |

Blocks are 256 bytes apart. `Xcp_Data` is **full** — its last field ends within
8 bytes of the 256-byte boundary, so the next appended measurement collides
with `Xcp_Cal`.

**`SharedRam` is not an XCP block** — it is the cross-core (CPU-to-CPU) shared
block introduced in T9 (`docs/REFACTORING_PLAN.md` §2.4), separate from the
`0x700300xx` XCP series above and not reachable by SHORT_UPLOAD. It sits in the
top 64 K of `lmuram` (768 K total) at the **non-cached** alias so no coherency
handling is needed on either side of a crossing; the cached alias of the same
physical RAM is `0x90040000`-`0x900FFFFF`. Every object placed there gets its
own `.c` file with a single `__at()` definition (cppcheck cannot parse a
second one in the same translation unit — see `SharedRam.h`). Occupants so far
(writer, then address):

| object | writer | address | defined in |
|---|---|---|---|
| `g_coreStats` | each core, own slot | `0xB00F0000`, 96 bytes | `CoreStats.h` / `SharedRam.c` |
| `g_navState` | CPU1 (`NavTask_step`) | `0xB00F0060` | `NavState.h` / `NavStatePlace.c` |
| `g_baroLatch` | CPU0 (`SensorTask_baro`) | `0xB00F0200`, 24 bytes | `FusionLatch.h` / `FusionLatchPlace.c` |
| `g_gnssLatch` | CPU0 (`SensorTask_gnss`) | `0xB00F0300`, 40 bytes | `FusionLatch.h` / `FusionLatchPlace.c` |
| `g_magLatch` | CPU0 (`SensorTask_mag`) | `0xB00F0400`, 24 bytes | `AhrsLatch.h` / `AhrsLatchPlace.c` |
| `g_imuEdge` | CPU1 (`imuDrdyIsr`) | `0xB00F0500`, 8 bytes | `ImuEdge.h` / `ImuEdgePlace.c` |

The three latches (T12) are the "three input latches" of §2.4/§2.3 — moved off
plain statics that a CPU0 writer and a CPU1 reader shared with no protocol.
Each is 256 bytes clear of its neighbour, generously past `g_navState`'s own
size, so a `NavState_t` growth cannot silently reach one without first
crossing `0xB00F0060 + 256` — the same spacing convention as the XCP blocks
above. Confirmed non-overlapping via the `.map` file, the same check T9/T10
used for `g_navState`.

`g_imuEdge` (T15, §3.6) is the fourth crossing and the odd one out: writer
(`imuDrdyIsr`) and reader (`NavTask_step`) are on the SAME core, both CPU1,
since the DRDY ISR retargeted there in T15 — so it is not a cache-coherency
crossing at all — it is placed here
anyway because it is the same "must never see a torn combination of two
fields written together" problem the ISR and the task race on, and reusing
the one audited protocol beats inventing a second for an ISR/task boundary.

**When a block fills, take the next free slot rather than re-spacing the map.**
That is what `Xcp_Fusion` did on 2026-08-26: the navigation state needed 188
bytes and `Xcp_Data` had 8, so it went to `0x70030500`. No existing address
moved, so every A2L entry, GUI plot and hard-coded tool address stayed valid —
and the seven original fusion fields are *still written* inside `Xcp_Data` for
exactly that reason. Re-spacing is the expensive option and touches firmware,
A2L and GUI together.

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
