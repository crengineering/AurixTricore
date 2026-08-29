---
name: bringup
description: Get code onto the TC399 board and observe it running — clean build, flash, then read self-chosen debug values back over Ethernet (XCP) and UART, and off the pads if needed. Use whenever something must be checked on real silicon rather than reasoned about: a new driver, a suspect register, a value that looks wrong, "does this actually work on the board".
---

# Board bring-up loop

**build clean → flash → observe.** The point of this skill is the observe step:
getting a number *you chose* out of a running TC399 with the least ceremony.

Three channels, in the order to reach for them:

| Channel | Reach for it when | Cost |
|---|---|---|
| **Ethernet / XCP** | reading any global while the board runs | no firmware change at all |
| **UART** | boot-time events, anything before the network is up, one-shot dumps | needs a rebuild |
| **Pads** | you suspect wiring or a dead pin, not code | needs a rebuild |

---

## 1. Build

```
build.bat clean     # after ANY header edit
build.bat           # incremental, source-only edits
```

**`clean` after every header edit, no exceptions.** amk's incremental dependency
tracking does not reliably recompile on a header change, and a stale `.o` — an
old struct layout, an old array size — links silently into the ELF. You then
debug a binary that is not your code.

If a full clean is too slow and you must force one file: delete its `.src`
**and** `.o` **and** `.d`. Deleting only the `.o` lets astc reuse a stale
`.src`.

Expect `0 errors` and only the two known vendor warnings. `build.bat` deletes
the old ELF first, so a skipped build cannot masquerade as a successful one.

**Bump `SW_VERSION_STEP` in `src/bsw/Version.h`** whenever the flash is meant to
change behaviour. It is the only way to prove afterwards that the board is
running *this* build — and the version is readable over both channels below.

## 2. Flash

Start the UART capture **before** flashing, or the reset banner scrolls past:

```
python tools/uart_cap.py          # background; auto-detects "Infineon DAS JDS COM"
flash.bat                         # AURIXFlasher: erase + prog + verify
```

`flash.bat` uses the `.hex`, not the `.elf` — build first. Close the ADS
debugger and the GUI first; they hold the DAP and the COM port.

## 3. Observe — Ethernet / XCP (the default)

**Any global in the map file can be read live, with no firmware change.** The
slave's `SHORT_UPLOAD` takes an arbitrary address — only *writes* are
range-checked (`xcpWriteAllowed` in `Xcp.c`). No `Xcp_Data` field, no A2L
entry, no AurixGUI edit is needed to watch a value.

```
python tools/xcp_read.py --find dbg              # what's in the map
python tools/xcp_read.py g_TickCount_1ms         # one read
python tools/xcp_read.py g_dbgRaw:hex:16 g_dbgVal:f32 --watch 0.5
python tools/xcp_read.py 0x70030000:hex:32       # raw address
```

Board is `192.168.0.10:5555`. Types: `u8 u16 u32 i8 i16 i32 f32 hex str`,
default `u32`; 63 bytes max per read (`XCP_MAX_CTO - 1`).

### Self-chosen debug values, the cheap way

Declare a scratch global — **non-`static`**, or it never reaches the map:

```c
volatile uint32  g_dbgWho;      /* whatever you need to see */
volatile float32 g_dbgScratch[4];
```

Assign to it wherever the question is, build, flash, then `xcp_read.py` it.
Watching four floats at 2 Hz costs nothing on the target and no plumbing.

⚠️ **Symbol addresses move on every build.** `xcp_read.py` re-reads the map on
every run for exactly that reason. Never carry an address across a rebuild —
`tools/xcp_test.py` still has a hardcoded `0x70000170` for a symbol that has
since moved to `0x70000180`.

⚠️ **Take the scaffolding out before the MISRA gate.** A debug global touched
in one TU trips 8.7. Guard it with `#if BRINGUP_DEBUG` or delete it.

Fixed blocks, if you want them instead: `Xcp_Data` @ `0x70030000`, `Xcp_Cal` @
`0x70030100`, `Xcp_Nvm` @ `0x70030200` (layouts in `Measurements.h`,
`docs/DIAGNOSTICS.md`). `tools/xcp_test.py` is the connectivity smoke test.

**If XCP does not answer:** ping `192.168.0.10` first. No ping = link/PHY, not
XCP. Ping but no CONNECT = the firmware did not reach the lwIP init, so switch
to UART — that is the boot-order channel.

## 4. Observe — UART

ASCLIN0, 115200 8N1, **CPU0 only** (it owns the module), blocking writes.

```
python tools/uart_cap.py                       # auto-detect, appends tools/uart.log
python tools/uart_cap.py --port COM4 --out bringup.log
```

`Uart_print` / `Uart_println` take **strings only**. There is no printf. For
numbers use `uartHexByte()` (`Cpu0_Main.c:44`) — and for anything wider,
print it as bytes rather than writing a formatter:

```c
Uart_print("who=");  uartHexByte(who);  Uart_println("");
```

For floats, either send the raw word out as four hex bytes and decode on the
PC, or — usually better — put the float in a global and read it with
`xcp_read.py:f32` instead. Do not build a float formatter for a debug session.

Use UART when the answer is at boot: init return codes, chip-ID reads, which
branch ran, whether a hang happened before the scheduler. `<device>DebugDump()`
functions (`bmp581DebugDump`, `icm42688DebugDump`, …) already exist and dump
config registers plus one raw burst at reset — read those before adding prints.

⚠️ Blocking output in a fast task changes the timing it is meant to measure.
For anything periodic, prefer the XCP path.

## 5. Observe — pads

Only when the suspicion is **wiring or a dead pin, not code**. This is the step
that turns a stalled multimeter session into one flash.

`padSelfTest()` in `Cpu0_Main.c`, behind `#define SPI_PAD_TEST` (line ~29,
default `0`). Set it to `1`, list the pins per port, build clean, flash, read
the result in the UART capture. It drives each pin push-pull as GPIO and reads
its own pad back — a pad that works reports `hi=1 lo=0 DRIVES`.

**Always include unwired pins on the same port as a control group.** That is
the entire diagnostic value: it separates "this pad cannot drive" from
"something external is holding it". P22.4/5/6 unwired reading `DRIVES` while
P22.7/P22.8 read stuck-high *with nothing attached* is what proved those two
pads are unusable on this TriBoard — after a day lost to `WHO_AM_I = 0x00`.

Reading: two pins clamped high and one clamped low means a wire is on a supply
pin, GND, or another driven output. No resistor network does that.

Set `SPI_PAD_TEST` back to `0` before committing.

## 6. Reading the result

Prefer an invariant that does not depend on how the board is lying:
`|a|` = 1 g at rest, `|B|` constant while rotating (~0.48 G in Munich), die
temperature agreeing across sensors. A scaling bug shows up as a clean 2×/4×,
not as noise. Then confirm `diagStatus == 0` with everything fitted
(`docs/DIAGNOSTICS.md`).

Before concluding anything, check the version: `xcp_read.py Xcp_Data:hex:8`
shows magic + version, and the UART banner prints it at reset. **A result from
the wrong build is worse than no result.**

## Triage — symptom to first suspicion

| Symptom | Look here first |
|---|---|
| behaviour unchanged after a flash | stale build — did `build.bat clean` run? does the version match? |
| symbol "not in the map" | it is `static`, or was optimised out — make it `volatile` and non-static |
| XCP reads plausible garbage | address from a previous build; re-run (the tool re-reads the map) |
| no ping | link/PHY, not XCP |
| ping but no CONNECT | firmware died before lwIP init → go to UART |
| UART port will not open | ADS debugger, AurixGUI or another capture holds it |
| UART silent from reset | CPU0 hung before `Uart_init`, or all six cores did not sync (watchdog reset) |
| ID register `0x00` / `0xFF` | wiring — go to §5 before touching the driver |
| values frozen, every read ACKs | missing I2C **STOP**: iLLD defaults module SOPE to 0 and the per-device `enableRepeatedStart` field is dead code |
| write OK, read FAIL, **never a NAK** | wedged I2C master — `IfxI2c_I2c_initModule()` does not clear the kernel/FIFOs; recovery needs `IfxI2c_resetModule()` + `IfxI2c_resetFifo()` |
| all sensors vanish after a warm reset | bus recovery missing (9 SCL pulses) |
| `STAT=0x01` / all-`0x7F` burst in a BMP581 boot dump | both **normal** at boot |

## Before committing

- `SPI_PAD_TEST` back to `0`, debug globals and prints removed or guarded
- `python tools/misra_check.py` — debug scaffolding trips 8.7
- do not `git commit` / `git push` unless explicitly asked
