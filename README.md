# AurixTricore

Bare-metal firmware for the Infineon **AURIX TC399** (6× TriCore, 300 MHz) on the
TriBoard TC3X9 V2.0. No RTOS — a cooperative scheduler per core, iLLD for the
hardware, lwIP for Ethernet.

A flight-controller platform: an Ethernet-attached measurement and calibration
target that reads inertial, barometric, magnetic and GNSS sensors, monitors its
own health, and exposes all of it live over **XCP-on-UDP** to a PC GUI.

**Firmware version: v1.18.0** (`src/bsw/Version.h`) · Hardware-validated on the
bench.

### Companion repositories

This is the firmware half of a three-repository project. Each side documents
itself; nothing is described twice.

| Repository | Owns |
|---|---|
| **AurixTricore** (here) | Firmware, drivers, XCP memory map, A2L generation, replay protocol |
| [**AurixGUI**](https://github.com/crengineering/AurixGUI) | Qt6 measurement and calibration GUI — XCP client, A2L-driven views, MF4 logging |
| [**Quadrocopter**](https://github.com/crengineering/Quadrocopter) | Simulink 6-DoF model, controller design, MiL/SiL/PiL verification — the origin of `src/asw/flight_ctrl.c` |

---

## What runs today

| Area | State |
|---|---|
| 6-core boot + sync barrier, both watchdogs disabled | ✅ |
| Cooperative scheduler (`SCHED_MS(n)`, 8 tasks/core) | ✅ |
| Ethernet: RGMII + RTL8211F PHY + lwIP, static `192.168.0.10` | ✅ |
| TCP + UDP echo (port 7), XCP-on-UDP slave (port 5555) | ✅ pyXCP-validated |
| XCP: SHORT_UPLOAD polling, calibration writes, DAQ | ✅ |
| Persistent parameters in DFLASH0 (2-sector ping-pong + CRC) | ✅ |
| BMP581 barometer (I2C0 `0x47`) | ✅ HW-validated |
| MMC5983MA magnetometer (I2C0 `0x30`) | ✅ HW-validated |
| ICM-42688-P IMU (QSPI0) | ✅ HW-validated |
| u-blox NEO-M9N GNSS (ASCLIN4, interrupt-driven UBX) | ✅ HW-validated |
| AHRS complementary filter (NED, nose-up = +90°) | 🚧 provisional — to be replaced by the full sensor fusion |
| Diagnostics: 28 debounced status bits, per-device peripheral health | ✅ |
| GPIO / PWM feature on Port 00 (GTM TOM), XCP-controlled duty | ✅ |
| Flight-control replay harness (UDP 5556) | ✅ |
| Host unit tests + MISRA gate in CI | ✅ |
| DShot ESC output, CAN, EVADC | 📋 planned — `docs/PINNING.md`, `docs/CAN_ADC_PLAN.md` |

⚠️ **CPU1–CPU5 are essentially idle (0.00 % load); CPU0 does everything.**
Distributing work is the main open architectural item — see [Status](#status).

---

## Hardware & toolchain

| Component | Details |
|---|---|
| Board | AURIX TC3xx Starter Kit 2nd Gen (TriBoard TC3X9 V2.0) |
| MCU | TC399XX-256, step BD, LFBGA-516, 6× TriCore @ 300 MHz, 16 MB PFLASH |
| PHY | RTL8211F (RGMII on Port 11) |
| Debug / flash | On-board DAP over Micro-USB (no external probe) |
| Console | X109 USB-to-UART, ASCLIN0, 115 200 8N1 |
| IDE | AURIX Development Studio 1.10.32 |
| Compiler | TASKING VX-toolset for TriCore (non-commercial), v1.1r8 |
| Drivers | iLLD 1.20.0 (`Libraries/iLLD/` — do not edit) |
| Flasher | AURIX Flasher Software Tool 3.0.16.0 |
| Host OS | Windows |

Build config: **TriCore Debug (TASKING)** → `AurixTricore.elf` / `.hex`.
`DEVICE_TC39XB` **must** be defined in the compiler's preprocessing symbols —
without it `IfxPort.c` takes the wrong `#else` branch and GPIO silently fails.

---

## Quick start

### Build

```bat
build.bat            :: incremental, headless (works while ADS is open)
build.bat clean      :: after any header change — see the gotcha below
```

Or in the IDE: `Project → Clean`, then `Ctrl+B`. Expected:
`Build Finished. 0 errors, 0 warnings.`

> ⚠️ **`amk` does not reliably rebuild on header changes.** A stale `.o` (or even
> a stale `.src`) will happily link the old constant into the ELF. After editing
> a `.h`, use `build.bat clean` — and verify the version string over XCP before
> believing a release.

### Flash

```bat
flash.bat            :: AURIXFlasher, erase + program + verify
erase_flash.bat      :: full chip erase
```

### Watch it run

```bat
python tools\uart_cap.py                 :: console — start BEFORE flashing
python tools\xcp_read.py g_TickCount_1ms :: read ANY global off the live board
python tools\xcp_test.py                 :: XCP slave validation (pyXCP)
python tools\nvm_test.py                 :: DFLASH persistence + RAM/NVM split
python tools\gpio_pwm_test.py            :: GPIO/PWM feature validation
```

`xcp_read.py` is the fastest debug loop in the repo: SHORT_UPLOAD accepts any
address, so any global in the map file can be watched live — no `Xcp_Data`
field, no A2L entry, no GUI change needed.

The `/bringup` skill wraps the whole build → flash → observe cycle.

---

## Architecture

### BSW / ASW split

```
src/bsw/   Base software — owns all hardware and generic services.
           Never includes an ASW header.
src/asw/   Application software — calls BSW only, never iLLD/SFR directly.
           Runs on CPU1…CPU5; publishes via its own XCP block.
```

The rule exists so the compute half stays host-testable (see `docs/TESTING_PLAN.md`
§A1). `flight_ctrl.c` is byte-identical to its original in the
[Quadrocopter](https://github.com/crengineering/Quadrocopter) repository for
exactly this reason — the same translation unit runs as a Simulink S-function, as
a PC reference and here on the target, which is what makes the replay comparison
meaningful. Controller design, tuning and the MiL/SiL verification live there.

### Core allocation

| Core | Role | LED |
|---|---|---|
| CPU0 | Ethernet/lwIP, XCP, all sensors, AHRS, diagnostics, NVM | D306 (P20.11) |
| CPU1 | sync + 10 ms idle task | D307 (P20.12) |
| CPU2 | sync + 10 ms idle task | — (D308's pin is now QSPI0 SCLK) |
| CPU3 | sync + 10 ms idle task | D309 (P20.14) |
| CPU4/5 | sync + 10 ms idle task | — |

Every `coreN_main` **must** call `IfxCpu_emitEvent` *and* `IfxCpu_waitEvent`
before any application code — skipping either on any core causes a watchdog
reset. Each core uses its own STM module (`MODULE_STM0` → CPU0, …).

Cross-core data follows one pattern, documented in `CoreStats.h`: **single writer
per slot, no locks, cross-core DSPR reads bypass the cache.** Reuse it rather
than inventing a second scheme.

### Fixed XCP memory map

Blocks are pinned with TASKING `__at`, 256 bytes apart, so clients need no map
file:

| Block | Address | Backing | Defined in |
|---|---|---|---|
| `Xcp_Data` — live measurements | `0x70030000` | RAM | `Measurements.h` |
| `Xcp_Cal` — calibration | `0x70030100` | **RAM only**, lost on reset | `Diagnostics.h` |
| `Xcp_Nvm` — persistent params | `0x70030200` | DFLASH0 | `Nvm.h` |
| `Xcp_Gpio` — pin modes / duty | `0x70030300` | RAM only | `gpio.h` |

`Xcp_Data` is **full**: the last field, `GnssVAccuracy`, sits at offset `0xFC` and
ends exactly on the 256-byte boundary. **The next appended field collides with
`Xcp_Cal`** — the block has to be enlarged, or the map re-spaced, before another
measurement is added. See *Status → Open*.

The offset comment block in `Measurements.h` is the layout SSoT. **Append new
fields; never insert** — an insert silently shifts every A2L entry and GUI plot
below it.

### Network endpoints

| Port | Protocol | Service |
|---|---|---|
| 7 | TCP + UDP | echo (`Echo.c`, `UdpEcho.c`) |
| 5555 | UDP | XCP slave — poll, calibration writes, DAQ (`Xcp.c`) |
| 5556 | UDP | flight-control replay: 16 floats in → 17 floats + ticks out (`CtrlReplay.c`) |

Static IP `192.168.0.10/24`, set in `Cpu0_Main.c`.

### Sensors

| Part | Bus | Address | Rate | Doc |
|---|---|---|---|---|
| ICM-42688-P IMU | QSPI0 | SLSO10 | 50 Hz | `docs/ICM42688P.md` |
| BMP581 barometer | I2C0 | `0x47` | 50 Hz | `docs/BMP581.md` |
| MMC5983MA magnetometer | I2C0 | `0x30` | 50 Hz | `docs/MMC5983MA.md` |
| u-blox NEO-M9N GNSS | ASCLIN4 | UART 38 400 | 1 Hz | `docs/GNSS_UBX.md` |

`Bmp388.c` and `Mpu6050.c` remain in-tree as a validated driver pool — compiled,
never called, held with justified MISRA 8.7 deviations.

**`docs/PINNING.md` is the single source of truth for every pin.** Read the whole
relevant section before wiring; contradicting it has cost days.

---

## Repository layout

```
src/bsw/              Base software (see Architecture above)
src/asw/              Application software — flight_ctrl.c, CtrlReplay.c
test/                 Host unit tests: vendored Unity 2.7.0, CMake, fakes
tools/                Python: on-target integration tests + host test/MISRA runners
docs/                 Pin SSoT, per-peripheral notes, plans, A2L, vendor PDFs
configs/              GUI plot configurations
Configurations/       PLL init, boot mode header, startup software, lwipopts
Libraries/Ethernet/   lwIP + Infineon port + RTL8211F PHY driver
Libraries/iLLD/       Infineon Low-Level Driver — do not edit
Libraries/Infra/      SSW / SFR infrastructure — do not edit
.github/workflows/    misra.yml, unit_tests.yml
build.bat flash.bat erase_flash.bat test_replay.bat
```

A companion **AurixGUI** (separate repo, Qt) is the XCP master: live plots,
calibration tab, diagnostics table, MF4 logging.

---

## Tests & CI

Two independent layers:

**Host unit tests** — `src/` compiled with GCC, no hardware, milliseconds.
This is where the failure paths live that the bench cannot produce: counter
wraparound, CRC mismatch, both NVM sectors claiming to be newest.

```bat
python tools\utest.py                    :: build + run (CTest)
python tools\utest.py --variant coverage :: gcovr HTML report
python tools\utest.py --variant sanitize :: ASan + UBSan (CI only — no libasan on MinGW)
```

**On-target integration tests** — `tools/xcp_test.py`, `nvm_test.py`,
`ctrl_replay_test.py`, `gpio_pwm_test.py`: real board, real peripherals, pyXCP
over Ethernet. Complementary, not a substitute.

**CI** (GitHub Actions, `ubuntu-latest`):

| Workflow | Checks |
|---|---|
| `unit_tests.yml` | unit tests · ASan+UBSan · coverage (artifact + summary) · warnings at `-Og` and `-Os` |
| `misra.yml` | cppcheck MISRA addon (pinned 2.21.0) over `src/` |

The MISRA gate uses a **baseline ratchet**: legacy findings are grandfathered in
`tools/misra_baseline.txt`, only *new* violations fail. New code should be
MISRA-clean — fix, don't baseline. Justified deviations:
`/* cppcheck-suppress misra-c2012-X.Y ; deviation: reason */`.

Both workflows keep the YAML thin and the work in `tools/*.py`, so every check
reproduces locally with one command.

---

## Documentation index

| File | What it answers |
|---|---|
| `CLAUDE.md` | Build/flash commands, hard rules, commit conventions |
| `docs/CODEMAP.md` | "What does this change touch?" — chains crossing code ↔ A2L ↔ GUI ↔ docs |
| `docs/PINNING.md` | **Pin SSoT** — every allocated, planned, board-fixed and free pin |
| `docs/DIAGNOSTICS.md` | Diagnostic bit meanings, calibration block |
| `docs/ILLD_NOTES.md` | Vendor-driver traps — read before grepping 801 iLLD files |
| `docs/TESTING_PLAN.md` | Test/CI strategy and the remaining stages |
| `docs/BMP581.md` etc. | Per-peripheral wiring and bring-up notes |
| `docs/CTRL_REPLAY.md` | Replay harness protocol |
| `docs/CAN_ADC_PLAN.md` | MCMCAN + EVADC plan (blocked on hardware) |
| `docs/AurixTricore.a2l` | ASAP2 description for XCP masters ⚠️ **stale — see below** |

---

## Status

### Done

**Platform**

- [x] 6-core boot with a full sync barrier; both watchdogs handled correctly
- [x] Cooperative scheduler per core (`SCHED_MS(n)`, 8 tasks/core) — no RTOS
- [x] PLL init via SSW, STM at ~100 MHz, one STM module per core
- [x] Lock-free cross-core data pattern (single writer per slot, cache-bypassing
      DSPR reads), documented once in `CoreStats.h` and reused

**Communication**

- [x] RGMII + RTL8211F PHY brought up, lwIP integrated, static `192.168.0.10`
- [x] TCP and UDP echo servers (port 7)
- [x] XCP-on-UDP slave (port 5555): SHORT_UPLOAD polling, calibration writes, DAQ
      — validated against pyXCP
- [x] Flight-control replay endpoint (port 5556) with on-target cycle-time
      measurement — see `docs/CTRL_REPLAY.md`
- [x] A2L generated from the C structs (`tools/gen_a2l.py`), so the GUI's view of
      memory cannot silently drift from the firmware's

**Sensors and fusion**

- [x] ICM-42688-P IMU over QSPI0 — HW-validated, |a| = 0.998 g at rest
- [x] BMP581 barometer over I2C0 — HW-validated, 954.6 hPa / 27.8 °C
- [x] MMC5983MA magnetometer over I2C0 — HW-validated, sphere-fit residual 0.9 %
- [x] u-blox NEO-M9N GNSS over ASCLIN4 — HW-validated, 12 satellites, interrupt-
      driven UBX framing with a 512-byte ring buffer

**Robustness**

- [x] In-house bounded I2C transfer engine replacing the iLLD calls, which spin
      unbounded and could hang CPU0 permanently — verified hang-immune under
      60 s of deliberate wire-wiggling
- [x] I2C bus recovery (9 SCL pulses + kernel and FIFO reset) so a slave
      brown-out no longer takes every device on the bus down until a power cycle
- [x] 28 debounced diagnostic status bits, four fault classes per peripheral
- [x] Persistent parameters in DFLASH0: two-sector ping-pong with CRC, strict
      RAM/NVM split so a calibration experiment can never brick the stored set

**Engineering practice**

- [x] Host unit tests — `src/` compiled with GCC against hand-written iLLD fakes,
      runs in milliseconds with no hardware attached
- [x] MISRA C:2012 gate in CI (cppcheck, pinned), legacy findings baselined so
      only *new* violations fail the build
- [x] A2L drift gate in CI — the committed A2L must match what the structs generate
- [x] Headless build and flash scripts (`build.bat`, `flash.bat`) — no IDE needed
- [x] Distilled vendor documentation under `docs/` (`ILLD_NOTES.md`, per-peripheral
      notes, `PINNING.md` as the pin SSoT) so findings are written down once

### Open

- [ ] **`Xcp_Data` is full.** The last field ends exactly on the 256-byte
      boundary; the next measurement collides with `Xcp_Cal`. The block needs
      enlarging or the memory map re-spacing — and that is a coordinated change
      across firmware, A2L and GUI (`docs/CODEMAP.md`).
- [ ] **Diagnostic bit word is full.** All 32 bits are allocated (bit 31 is
      `DIAG_CAL_INVALID`). A fifth peripheral needs a second bitmask word.
- [ ] **CPU1–CPU5 idle at 0.00 %** while CPU0 carries the entire load. Moving the
      sensor and fusion chain off CPU0 is the main open architectural item; the
      pattern to extend already exists in `CoreStats.h`.
- [ ] **DShot ESC output** — bidirectional DShot300 on GTM ATOM0/TIM, with RPM
      feedback for a notch filter. Pins allocated in `docs/PINNING.md`, driver
      not yet written.
- [ ] **CAN (MCMCAN) and EVADC** — planned in `docs/CAN_ADC_PLAN.md`, blocked on
      test hardware.
- [ ] **Sensor fusion** — the current AHRS is a provisional complementary filter:
      NED, sign convention verified on the bench (nose-up = +90°), but yaw is
      gyro-only and drifts, and neither the magnetometer nor GNSS is fused yet.
      It will be **reworked as part of the full sensor fusion** — a quaternion
      estimator at 1 kHz over all four sensors, verified against the Simulink
      model through the replay harness. Treat the present attitude output as
      indicative, not as a finished estimate.
- [ ] **Branch protection** — CI checks are informational rather than required,
      which needs GitHub Pro on a private repository (`docs/TESTING_PLAN.md`
      stage 6). A versioned `pre-push` hook is the free substitute.
- [ ] **EUI-48 MAC from the I2C EEPROM** instead of the hard-coded address.

---

## Hard-won gotchas

Each of these cost real time. Full accounts live in the linked docs.

**1. P22.7 and P22.8 are unusable on this TriBoard.** Driven as plain GPIO with
nothing attached they read back stuck-high, while unwired neighbours on the same
port work perfectly. The IMU returned `WHO_AM_I = 0x00` because SCLK never
reached it. SCLK moved to **P20.13**, costing CPU2 its LED. The technique that
found it — the **pad self-test** (`padSelfTestPort()` in `Cpu0_Main.c`: drive a
pin, read its own pad back, always with unwired control pins) — is meter-free and
worth reusing.

**2. iLLD I2C emits no STOP by default.** The module's `SOPE`
(`stopOnPacketEnd`) defaults to 0 and the per-device `enableRepeatedStart` field
is dead code. The MPU-6050 froze its data registers mid-burst and looked like a
dead die for days. `src/bsw/I2c.c` is an in-house **bounded** transfer engine
(10 ms deadline on every wait) because the iLLD spins are unbounded and hung CPU0
forever.

**3. `IfxI2c_I2c_initModule()` does not clear the I2C kernel or FIFOs.** After a
slave brown-out the master's RX path stays wedged permanently — writes succeed,
reads fail, and it *never NAKs* (that's the tell). One wedged master kills every
device on the bus. Recovery needs `IfxI2c_resetModule()` + `IfxI2c_resetFifo()`
plus 9 SCL pulses.

**4. STM runs at ~100 MHz after PLL init** (automatic via
`IFX_CFG_SSW_ENABLE_PLL_INIT = 1`), so `50 000 000 ticks ≈ 500 ms`.

**5. Both watchdogs must be disabled** — CPU watchdog on every core, safety
watchdog on CPU0.

**6. New iLLD subsystems are `.cproject`-excluded by default.** Adding a bus
means un-excluding its directory *and* using root-relative include paths.

---

## Contributing

- **Conventional Commits** (`type(scope): description`) — full table in `CLAUDE.md`.
- Update `docs/PINNING.md` in the *same* change that moves a pin.
- Bump `src/bsw/Version.h` on a release, then **confirm the version over XCP** —
  a stale `.src` will ship the old string without complaint.

---

## Safety notice

This is a personal engineering project, not a product. Nothing here is
functionally safe, safety-certified, or qualified to any automotive or aviation
standard (ISO 26262, DO-178C or otherwise). The MISRA C:2012 gate and the unit
tests raise code quality; they do not make the software airworthy.

`src/asw/` contains flight-control code and `docs/PINNING.md` allocates pins for
DShot ESC output. Spinning propellers are dangerous. If you run any of it on real
hardware: **remove the propellers**, secure the frame, keep the battery
disconnected until the last moment, and understand that you do so entirely at
your own risk. See the warranty and liability disclaimer in `LICENSE`.

---

## License

[MIT](LICENSE) © 2026 Chris Riedl — for the original work in this repository:
`src/`, `tools/`, `docs/`, `configs/`, `test/` (excluding `test/unity/`),
`.github/`, the linker scripts and the build scripts.

`Libraries/` and `Configurations/` are vendored third-party code that stays under
its own licences — Infineon iLLD, SSW/SFR infrastructure, the CpuGeneric service
layer and the Ethernet port under the **Boost Software License 1.0**, lwIP under
**BSD-3-Clause**, Unity under **MIT**. All are permissive and redistributable,
and every original licence header is preserved. Full table with SPDX identifiers
and upstream sources: [THIRD_PARTY.md](THIRD_PARTY.md).

Infineon datasheets and reference manuals are **not** redistributed here.

AURIX™, TriCore™ and Infineon® are trademarks of Infineon Technologies AG, used
here descriptively to identify the target hardware. No affiliation or
endorsement is implied.

---

## Resources

- [AURIX Development Studio](https://www.infineon.com/cms/en/tools/aurix-development-studio/)
- [TriBoard TC3X9 User Manual](https://www.manualslib.com/manual/1496611/Infineon-Triboard-Tc3x9-Th.html)
- [Infineon AURIX Community](https://community.infineon.com/t5/AURIX/bd-p/AURIX)
