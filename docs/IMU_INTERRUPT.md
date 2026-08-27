# IMU_INTERRUPT.md — ICM-42688-P data-ready interrupt: pin, wiring, measurement

**Status: design only.** Nothing under `src/` is changed by this document.
Purpose: bring the ICM-42688-P's `INT1` (data-ready) out to a TC399 pin and
**measure the real interval distribution** at which it fires, so decision **D5**
in `docs/REFACTORING_PLAN.md` §7 (`flight_ctrl` wants `Ts = 0.001f`, the IMU tick
is 50 Hz) is settled by data instead of argument.

> **Pin-allocation SSoT is `docs/PINNING.md`.** This file does *not* edit it.
> §6 below gives the exact lines to add there once the pin is hardware-proven.
> Electrical rules come from `PINNING.md` §2.5 and `docs/ICM42688P.md` §2 — where
> they and this file could disagree, **§2.5 wins**.

---

## 1. THE ANSWER — pin, wire, hazard

| | |
|---|---|
| **TC399 pin** | **P10.7** |
| **ERU path** | `IfxScu_REQ0C_P10_7_IN` → ERU **input channel 0** → OGU0 → `SRC_SCUERU0` |
| **SRPN** | **106** (next free; 105 = ASCLIN4 RX) |
| **Core** | **CPU0** for the measurement (that is where `Task_Imu` and `SysTime`/STM0 live today). One-line move to CPU1 when `NavTask` lands — §5.4 |
| **IMU pin** | **`INT1`**, EVB **CN1 pin 3** |
| **Wire** | `CN1 pin 3` → **P10.7**. Single wire. **No divider, no series resistor.** |
| **Optional** | 10 kΩ from that net to GND (`PINNING.md` §2.5). Redundant if firmware uses the internal pulldown — see §3.3. Skipping it is fine. |
| **Header hole** | `PINNING.md` §5 lists P10.7 = **X702·73** — ⚠️ **UNVERIFIED**, same connector table that cost a day on the IMU. **Wire by pin name.** Identify the hole with the meter-free probe in §2.2 *before* soldering. |
| **Fallbacks** | **P10.8** (`IfxScu_REQ1C_P10_8_IN`, ERU ch1, `SRC_SCUERU1`, X702·72) then **P10.3** (`IfxScu_REQ3A_P10_3_IN`, ERU ch3, X702·74). Both free, both Port 10, both ERU-capable. |

### ⚠️ Voltage hazard — read before powering

- **P10.7 is a VEXT = 5 V pad** (only Port 11 is 3.3 V — `PINNING.md` board-wide
  note). The ICM-42688-P's absolute maximum on any I/O pin is **VDDIO + 0.3 V =
  3.6 V**. **If P10.7 is ever configured as an output while the wire is on, it
  puts 5 V into `INT1` and destroys the part.**
  → Consequence for the order of work: **run the pad self-test (§2.1) BEFORE the
  wire exists**, never after. After wiring, P10.7 is input-only forever
  (`IfxScuEru_initReqPin` sets it to input, and nothing else may touch it).
- **The other direction is safe.** `INT1` drives 3.3 V push-pull into a 5 V pad —
  no divider needed, exactly like `MRST`/MISO. But the pad's default CMOS
  threshold is ≈3.5 V and would *never* see a high, so the pad **must** be put in
  TTL mode (VIH = 2.0 V):
  ```c
  IfxPort_setPinPadDriver(&MODULE_P10, 7u, IfxPort_PadDriver_ttlSpeed1);
  ```
  Same technique as `I2c.c` and the QSPI MISO line.
- **Do not use the pad's internal pull-*up*** — it pulls to the 5 V VEXT rail and
  would fight the IMU's 3.3 V push-pull output. Pulldown or no pull device only.
- No change to the IMU supply. `INT1` is referenced to `VDDIO` = the injected
  **3.3 V** at JP1 pin 2 (`ICM42688P.md` §1); the jumpers stay **open**.

### Why P10.7 and not something else

Only 18 pads on this package can reach the ERU at all
(`IfxScu_PinMap_TC39xB_516.h:112-128`). Nearly all are taken:

| REQ pin object | Pad | Verdict |
|---|---|---|
| `REQ0A_P15_4` | P15.4 | I2C EEPROM SCL, board-fixed (§4) |
| `REQ1A_P14_3` | P14.3 | **TLF35584 WDI** — the watchdog line (§4) |
| `REQ2A_P10_2`, `REQ2B_P02_1`, `REQ3C_P02_0` | | ERAY-A alt footprints, not in the pool |
| `REQ2C_P00_4` | P00.4 | allocated to the GPIO/PWM feature (`gpio_cfg.h`) |
| `REQ3B_P14_1` | P14.1 | ASCLIN0 RX — the debug console |
| `REQ4A_P33_7` | P33.7 | LED D305 + **DAP_SCR** debug line |
| `REQ4D_P15_5` | P15.5 | I2C EEPROM SDA |
| `REQ5A_P15_8` | P15.8 | free, **but** carries the Ethernet-MDINT footprint stub (R377) |
| `REQ6A_P20_0` | P20.0 | OCDS/DAP JTAG + HSCT stub |
| `REQ6D_P11_10` | P11.10 | RGMII RXD1 |
| `REQ7A_P20_9` | P20.9 | **ERAY-B ERRN** — this is exactly the pin `PINNING.md` §2.5 already rejected for the IMU INT once |
| `REQ7C_P15_1` | P15.1 | LIN1 RXD |
| **`REQ0C_P10_7`** | **P10.7** | **free pool §5, no footprint stub, no strap** ✅ |
| `REQ1C_P10_8` | P10.8 | free pool §5 ✅ (fallback) |
| `REQ3A_P10_3` | P10.3 | free pool §5 ✅ (fallback) |

Cross-checks done, all clear:
- **Not** P22.7 / P22.8 — hardware-proven unusable (`PINNING.md` §2.2).
- **Not** DShot P22.0-3, **not** GNSS ASCLIN4 P22.5/P22.6, **not** QSPI0
  (P20.13/P22.9/10/11), **not** I2C0 (P13.1/P13.2), **not** Ethernet (P11.x,
  P12.0/1). P10.7 appears in **none** of them.
- `grep -rn "MODULE_P10" src/` → **no hits**. Port 10 is untouched by firmware
  today, so there is no drift between `PINNING.md` and the code here.
- The `⚠ avoid TIM0.0` note against P10.7 in `PINNING.md` §5 is about the **GTM
  timer** mapping and is irrelevant to an ERU input.
- ⚠️ **P10.5 (HWCFG4) and P10.6 (HWCFG5) are boot straps on the same port.**
  P10.7 is not a strap, and P10.5 sits at X702·23 — nowhere near ·73 — so a
  mis-landed wire is unlikely to hit it. Do not include either in the self-test.

---

## 2. Before you solder

### 2.1 Pad self-test (meter-free) — the technique that exposed P22.7/P22.8

Do this with **nothing connected to Port 10**, i.e. **before** the INT1 wire
exists. This is the step that protects both the day and the part.

Temporary code, in `BringUp.c` or `Cpu0_Main.c`, run once after the core sync:

```c
/* TEMPORARY — pad self-test. Remove before merge.
 * Control group: P10.3 and P10.8 are free and unwired (PINNING.md §5).
 * P10.5/P10.6 are HWCFG straps — DO NOT include them. */
volatile uint32 g_padProbeP10;          /* non-static, so it reaches the map */

static uint32 padSelfTestP10(void)
{
    static const uint8 pins[3] = { 3u, 7u, 8u };
    uint32 result = 0u;                  /* bit i set = pin i toggled cleanly */
    uint8  i;

    for (i = 0u; i < 3u; i++)
    {
        boolean lowOk;
        boolean highOk;
        IfxPort_setPinModeOutput(&MODULE_P10, pins[i],
                                 IfxPort_OutputMode_pushPull,
                                 IfxPort_OutputIdx_general);

        IfxPort_setPinState(&MODULE_P10, pins[i], IfxPort_State_low);
        waitTicks(1000u);                                  /* ~10 us settle */
        lowOk  = (IfxPort_getPinState(&MODULE_P10, pins[i]) == FALSE);

        IfxPort_setPinState(&MODULE_P10, pins[i], IfxPort_State_high);
        waitTicks(1000u);
        highOk = (IfxPort_getPinState(&MODULE_P10, pins[i]) != FALSE);

        if (lowOk && highOk) { result |= (1u << i); }

        IfxPort_setPinModeInput(&MODULE_P10, pins[i],
                                IfxPort_InputMode_pullDown);   /* leave safe */
    }
    return result;
}
```

Read it back without a debugger:

```
python tools/xcp_read.py g_padProbeP10
```
or just `Uart_println` it at boot.

**Pass = `0x7`** (all three toggle). Interpretation:

| Result | Meaning | Action |
|---|---|---|
| `0x7` | P10.7 is good, control group is good | proceed to §2.2 |
| `0x5` (bit 1 clear, others set) | **P10.7 is committed by the board**, exactly the P22.7/P22.8 signature | fall back to **P10.8** (bit 2), re-run |
| `0x0` | the test itself is wrong (pad driver, ENDINIT, settle time), not the board | fix the test — a whole port being dead is not a real failure mode |

⚠️ **Do not skip the control group.** A single pin that "works" proves nothing;
the P22.7/P22.8 finding only became a finding because P22.4/5/6 on the same port
drove perfectly at the same instant.

### 2.2 Which hole is P10.7? — meter-free, and before the wire

`X702·73` comes from the connector table that is marked unverified, and a wrong
hole here is a rework. Confirm it from the *other* side, safely:

1. Configure P10.7 as **input with internal pull-up** and publish it:
   ```c
   IfxPort_setPinModeInput(&MODULE_P10, 7u, IfxPort_InputMode_pullUp);
   /* in the 100 ms housekeeping task: */
   g_padProbeP10 = (uint32)IfxPort_getPinState(&MODULE_P10, 7u);
   ```
2. `python tools/xcp_read.py g_padProbeP10 --watch 0.2` — it reads **1**.
3. Touch a **1 kΩ** lead from GND to the candidate hole (X702·73), one hole at a
   time. The hole where the value drops to **0** is P10.7.
   1 kΩ from a 5 V pad = 5 mA, well inside the pad; a bare short would also be
   survivable but 1 kΩ removes the argument.
4. Then set the pull back to `IfxPort_InputMode_pullDown` (§3.3) before wiring.

If you would rather use the meter: drive P10.7 as an output at 2 Hz (self-test
code above, in a loop) and find the hole that swings. **Only while nothing is
connected** — see the hazard note in §1.

---

## 3. The IMU side

### 3.1 Which INT pin

**`INT1`**, EVB **CN1 pin 3**. `INT2`/`FSYNC` (CN1 pins 5/6) stay open.
`INT1` runs straight from U1 pin 4 to CN1 pin 3 with nothing on it — `R6`/`R7`
on the EVB are NM and sit on the RESV pins anyway (`ICM42688P.md` §2). So
**CN1 pin 3 → GND reads OL on a bare board, and that is correct**, not a fault.

### 3.2 Register configuration (bank 0)

Written once in `Icm42688_init()` **after** `PWR_MGMT0`/`GYRO_CONFIG0`/
`ACCEL_CONFIG0` (`Icm42688.c:180-189`), before the first read.

| Reg | Addr | Value | Meaning |
|---|---|---|---|
| `INT_CONFIG` | `0x14` | `0x03` | `INT1_POLARITY`=1 **active high**, `INT1_DRIVE_CIRCUIT`=1 **push-pull**, `INT1_MODE`=0 **pulsed** |
| `INT_CONFIG1` | `0x64` | `0x00` | ⚠️ clears `INT_ASYNC_RESET` (**default is 1; the datasheet requires 0 for correct INT1/INT2 operation** — this is the classic silent-no-interrupt bug). `INT_TPULSE_DURATION`=0 → 100 µs pulse, valid below 4 kHz ODR |
| `INT_CONFIG0` | `0x63` | `0x00` | `UI_DRDY_INT_CLEAR` = clear on `INT_STATUS` read (moot in pulsed mode; set it deterministically anyway) |
| `INT_SOURCE0` | `0x65` | `0x08` | `UI_DRDY_INT1_EN` — data-ready routed to INT1 |

- **Push-pull, not open-drain.** Open-drain would need a pull-up, and the only
  rail available at the pad is 5 V VEXT — which would put 5 V on `INT1`. Push-pull
  removes that failure mode entirely. **This is a safety choice, not a preference.**
- **Active high** so the 10 kΩ / internal pulldown defines the idle state.
- **Pulsed, not latched**, so a missed `INT_STATUS` read cannot wedge the line.
- ⚠️ The addresses and bit positions above are from **DS-000347 rev 1.2** §14
  (`Quadrocopter/doc/IMU_chip_datasheet.pdf`, not in this repo — no-vendor-PDFs
  rule). **Re-check them in the PDF before writing the register**; a wrong write
  to `0x64` is harmless but a wrong one elsewhere in bank 0 is not.
- **ODR is already 1 kHz** — `ICM42688_GYRO_CONFIG0_VAL`/`ACCEL_CONFIG0_VAL` =
  `0x06` (`Icm42688.c:46-56`). So the expected interval is **1000 µs**, and the
  measurement is really about *how good* that 1 kHz is (the part runs off an
  internal RC oscillator, no `CLKIN` on this build) and whether any edges are lost.

### 3.3 Pad configuration on the MCU

```c
IfxScuEru_initReqPin(&IfxScu_REQ0C_P10_7_IN, IfxPort_InputMode_pullDown);
IfxPort_setPinPadDriver(&MODULE_P10, 7u, IfxPort_PadDriver_ttlSpeed1);
```

`IfxScuEru_initReqPin` does two things (`IfxScuEru.h:776`): sets the pin to input
with the given mode, and programs the ERU external-input multiplexer from the pin
object's `select` field (`Ifx_RxSel_c` for `REQ0C_P10_7`,
`IfxScu_PinMap_TC39xB_516.c:61`). The pad-driver call must come **after** it.
`_pullDown` holds the line low during the window between MCU boot and the IMU
configuring `INT1` as an output, which is what the external 10 kΩ would otherwise
be for.

---

## 4. ERU + interrupt plumbing

Recipe verified against `Libraries/iLLD/TC3xx/Tricore/Scu/Std/IfxScuEru.h`
(prototypes at lines 327-746, worked example in the file header at lines 89-111).
`Scu/Std` is **not** in the `.cproject` exclusion list — unlike `Qspi` and `I2c`
there is **no un-exclude step**.

```c
#define IMU_ERU_IN   IfxScuEru_InputChannel_0     /* REQ0C = P10.7  */
#define IMU_ERU_OUT  IfxScuEru_OutputChannel_0    /* OGU0 -> SRC_SCUERU0 */

IfxScuEru_initReqPin(&IfxScu_REQ0C_P10_7_IN, IfxPort_InputMode_pullDown);
IfxPort_setPinPadDriver(&MODULE_P10, 7u, IfxPort_PadDriver_ttlSpeed1);

IfxScuEru_disableFallingEdgeDetection(IMU_ERU_IN);
IfxScuEru_enableRisingEdgeDetection(IMU_ERU_IN);
IfxScuEru_enableAutoClear(IMU_ERU_IN);
IfxScuEru_enableTriggerPulse(IMU_ERU_IN);
IfxScuEru_connectTrigger(IMU_ERU_IN, IfxScuEru_InputNodePointer_0);   /* -> OGU0 */

IfxScuEru_setFlagPatternDetection(IMU_ERU_OUT, IMU_ERU_IN, TRUE);
IfxScuEru_enablePatternDetectionTrigger(IMU_ERU_OUT);
IfxScuEru_setInterruptGatingPattern(IMU_ERU_OUT,
                                    IfxScuEru_InterruptGatingPattern_alwaysActive);

IfxSrc_init(&SRC_SCUERU0, IfxSrc_Tos_cpu0, ISR_PRIORITY_IMU_DRDY);
IfxSrc_enable(&SRC_SCUERU0);
```

- **Only OGU0-3 have service-request nodes** — `SRC_SCUERU0..3` at
  `0xF0038880`/`84`/`88`/`8C` (`IfxSrc_reg.h:3489-3514`). `InputNodePointer_4..7`
  exist in the enum but have no SRC node on this device; do not use them.
- The input channel number is **fixed by the pad**: P10.7 → REQ**0**C → input
  channel **0**. The *output* channel (and therefore the SRC node) is free choice;
  OGU0 is picked because nothing in this project uses the ERU at all today
  (`grep -rn "IfxScuEru" src/` → no hits).
- **Glitch filter: leave disabled.** `SCU_EIFILT` is global to all ERU inputs,
  not per channel (`IfxScuEru_setInputFilterDepth` writes `SCU_EIFILT.B.DEPTH`),
  and it is ENDINIT-protected. A 100 µs pulse from a push-pull driver over a short
  wire needs no filtering, and enabling it would silently change behaviour for any
  future ERU user. If a filter ever proves necessary, that fact is itself a
  finding — record it.
- **New ISR priority** in `Configurations/ConfigurationIsr.h`:
  ```c
  #define ISR_PRIORITY_IMU_DRDY       106   /* ERU/OGU0, ICM-42688-P INT1 data-ready */
  ```
  106 is above `GETH_RX` (101) and the QSPI nodes (102-104), so the ISR preempts
  everything and the timestamp reflects the **pin edge**, not scheduler latency.
  That is the point of the measurement. Cost: a ~1 µs ISR at 1 kHz ≈ **0.1 %** of
  CPU0 and up to ~1 µs of added lwIP/QSPI latency — acceptable, and it is the
  honest configuration. If a lower priority is ever chosen instead, the measured
  jitter stops being the sensor's and becomes CPU0's.
- CPU0 already calls `IfxCpu_enableInterrupts()` (`Cpu0_Main.c:129`).

---

## 5. Measuring the interval — the distribution, not the average

### 5.1 Zero cost to `Xcp_Data`, the A2L, the GUI, or `diagStatus`

**No new `Xcp_Data` field. No new A2L entry. No new GUI signal. No new diagnostic
bit.** `tools/xcp_read.py` reads **any non-static global by name** out of the map
file over `SHORT_UPLOAD` — that is the sanctioned bring-up channel and it exists
precisely for this (`tools/xcp_read.py` header comment). So the whole measurement
lives in plain globals in a temporary module.

This matters because both budgets are effectively exhausted:
- `Xcp_Data` ends at `0xF8` with **8 bytes** before `Xcp_Cal` at `0x70030100`
  (`Measurements.h`, the `Xcp_Fusion` comment). The "~40 bytes free" figure in
  memory predates the fusion fields — **it is now 8**. Nothing useful fits.
- `diagStatus` is **full**: bits 27-30 were the last four and were spent on the
  peripheral diagnostics (`docs/DIAGNOSTICS.md`).

**If a diag bit is ever wanted for "DRDY stopped", say so explicitly — it is not
free and there is no bit left.** For the measurement it is not needed:
`g_imuDrdyCount` standing still is the same information, visible from the host.

### 5.2 The data, and where the timestamp comes from

`SysTime_getTicks()` → lower 32 bits of **STM0** at 100 MHz → **10 ns
resolution**, wraps at 42.9 s. Unsigned subtraction stays correct across one
wrap, so `dt` is always right (`SysTime.h`). At a 1 ms nominal interval, 10 ns is
5 decimal digits of headroom — resolution is a non-issue.

New module `src/bsw/ImuInt.{c,h}` (BSW: it owns hardware; ASW never sees it):

```c
/* All non-static so xcp_read.py can find them in the map. */
volatile uint32 g_imuDrdyCount;        /* edges since reset                     */
volatile uint32 g_imuDrdyLastTicks;    /* STM0 tick of the last edge            */
volatile uint32 g_imuDrdyDtTicks;      /* most recent interval                  */
volatile uint32 g_imuDrdyDtMin;        /* min interval, ticks                   */
volatile uint32 g_imuDrdyDtMax;        /* max interval, ticks                   */
volatile uint64 g_imuDrdyDtSum;        /* for the mean; uint64 — see note       */
volatile uint32 g_imuDrdyHist[32];     /* 1 us bins, base auto-set — see below  */
volatile uint32 g_imuDrdyHistBase;     /* ticks; bin i = [base+100i, base+100i) */
volatile uint32 g_imuDrdyUnder;        /* dt below the histogram range          */
volatile uint32 g_imuDrdyOver;         /* dt above the histogram range          */
volatile uint32 g_imuDrdyStaleTicks;   /* edge -> Task_Imu read latency, §5.5   */
volatile uint32 g_imuDrdyReset;        /* host writes... no: see note below     */
```

- `uint64` for the sum: at 1 kHz, `dt ≈ 100 000` ticks, so a `uint32` sum
  overflows after **43 s** — too short for a meaningful window. A 64-bit add in
  the ISR is a handful of cycles on TriCore. Mean = `g_imuDrdyDtSum /
  g_imuDrdyCount`, computed **on the host**, never in the ISR.
- **Histogram, because the ask is a distribution.** 32 bins × 1 µs (100 ticks)
  = a 32 µs window. `g_imuDrdyHistBase` starts at 0 meaning *not yet centred*; on
  the **100th** edge the ISR sets it once to `g_imuDrdyDtMin - 1600` (16 bins
  below the observed minimum) and zeroes the bins. That is bounded, O(1), and it
  works even if the RC oscillator is 3 % off nominal — which a hardcoded
  1000 µs centre would not survive.
- **`g_imuDrdyReset` is deliberately NOT host-writable.** XCP writes are
  range-checked to the cal block (`xcpWriteAllowed` in `Xcp.c`); reaching a BSW
  global would mean widening that check, which is a security/robustness
  regression for a bring-up convenience. **Reset by re-flashing, or by a boot-time
  window.** Restart the run instead.

### 5.3 What the ISR may and may not do

```c
IFX_INTERRUPT(imuDrdyIsr, 0, ISR_PRIORITY_IMU_DRDY)
{
    uint32 now = SysTime_getTicks();      /* one volatile SFR read */
    uint32 dt  = now - g_imuDrdyLastTicks;
    g_imuDrdyLastTicks = now;
    g_imuDrdyCount++;
    if (g_imuDrdyCount > 1u) { /* first edge has no valid dt */
        g_imuDrdyDtTicks = dt;
        if (dt < g_imuDrdyDtMin) { g_imuDrdyDtMin = dt; }
        if (dt > g_imuDrdyDtMax) { g_imuDrdyDtMax = dt; }
        g_imuDrdyDtSum += (uint64)dt;
        /* one bounded bin update, no loop */
    }
}
```

**MAY:** read STM0, integer arithmetic, one array store at a computed-and-clamped
index, set a `volatile boolean` flag for `Task_Imu`.

**MAY NOT:** touch QSPI (`Spi.c` owns it, and its own ISRs sit at 102-104 —
this ISR preempts them, so a single word touched in common would be a race);
call into lwIP, `Xcp`, `Uart`, `Nvm`, `PeriphDiag`, or the AHRS/fusion; use
floating point; loop unboundedly; allocate. **The actual SPI burst read stays in
`Task_Imu`/`NavTask_step`, in task context, exactly as today.** Worst case must
be a fixed instruction count — no branch in it depends on data beyond the two
min/max compares and one clamp.

Fail-operational note: if `INT1` ever chatters (a bad joint), the ISR at priority
106 is the highest thing on CPU0 and could starve it. The ISR is ~1 µs, so it
would take **>100 kHz** of chatter to matter, which a 100 µs pulse width makes
physically impossible. No livelock path is introduced.

### 5.4 Should this interrupt later *drive* the estimator tick?

**Not in this step, and probably not directly even later. Recommendation: no.**

- **Now (measurement phase):** the ISR only timestamps. `Task_Imu` keeps its
  20 ms scheduler slot. Nothing about the flight chain changes, so the board
  still flies at every commit.
- **After the measurement, if 1 kHz is confirmed:** the right structure is *not*
  "do the control loop in the ISR". It is: **`NavTask` gets a 1 ms scheduler slot
  on CPU1** (a rate change on one `Scheduler_addTask` line, exactly as
  `REFACTORING_PLAN.md` §7/D5 says the refactor was designed for), and the ISR's
  timestamp is used for two things only — (a) a `newSample` flag so the task can
  skip a slot when the sensor has not produced one, and (b) the **true `dt`**
  handed to the AHRS/KF instead of a nominal constant. That keeps a cooperative,
  inspectable, bounded task chain and still removes jitter as a control input.
- **Why not ISR-driven directly:** it would put the SPI burst, the AHRS and the
  KF at interrupt priority above lwIP and the QSPI driver itself — a re-entrancy
  problem with the very bus it needs — and it would make the flight chain's worst
  case invisible to `CoreStats`. The determinism argument runs the other way from
  the intuition here.

**Effect on `REFACTORING_PLAN.md` §3** (noted here, **not** edited there — T9 is
live in that file). Only if the measurement lands on outcome **A** in §5.6:

| row | today in §3 | would become |
|---|---|---|
| `NavTask_step` | 50 Hz / 20 ms, typ ~0.30 ms, worst 2.0 ms, CPU1 | **1 kHz / 1 ms**, typ ~**0.15 ms** (one 14-byte burst + AHRS + KF), worst **0.5 ms** budget, CPU1 |
| new row | — | `imuDrdyIsr` — 1 kHz, ~1 µs, CPU0 (later CPU1), preempts everything; overrun impossible |
| CPU1 load line | "CPU1 ≈ 1.5 %, with a single 20 ms slot to itself" | **CPU1 ≈ 15 %**, a 1 ms slot to itself. Still one core, still nothing else on it. |
| D5 row in §7 | "needs your call" | resolved by this measurement |

The 2.0 ms `NavTask` budget **cannot survive a 1 ms period** — that row must be
re-costed, not just re-rated. That is the one real consequence and it is why the
SPI burst duration in §5.5 has to be measured at the same time.

### 5.5 Two more numbers worth taking in the same session

1. **DRDY → read staleness.** In `Task_Imu`, right before the burst:
   `g_imuDrdyStaleTicks = SysTime_getTicks() - g_imuDrdyLastTicks;`
   At the current 50 Hz this will read up to ~20 ms. **That single number settles
   D5 on its own** — a rate loop cannot be fed data that is up to 20 ms old.
2. **SPI burst duration.** Bracket the `Icm42688_readRegs(TEMP_DATA1, …, 14)` in
   `Task_Imu` with `SysTime_getTicks()` and keep a max. This is the input to the
   1 kHz feasibility question: the plan's §3 estimate is 112 µs per read. If the
   real figure is much worse, outcome A branches to 500 Hz (§5.6).

### 5.6 Reading it out, and the decision tree

```
python tools/xcp_read.py g_imuDrdyCount g_imuDrdyDtMin:u32 g_imuDrdyDtMax:u32 \
                         g_imuDrdyStaleTicks:u32 --watch 1.0
```

⚠️ `xcp_read.py` caps a single read at **63 bytes** (`XCP_MAX_CTO - 1`,
`tools/xcp_read.py:79`) and has no chunking, so the 128-byte histogram cannot be
fetched in one call. **Task: add an array form** (`name:u32x32`) that splits into
63-byte reads, plus a small tools/imu_int_stats.py (**to be created in I6 — not in
the tree yet**) that pulls the block, prints
min / max / mean / stddev / p99 in **µs** and an ASCII histogram, and supports
`--watch`. Tools are not `src/` — this is allowed and is the smallest possible
host-side change.

**Window:** at least **60 s at rest** (60 000 edges) before reading anything; then
a second run with the frame hand-shaken, and a third with props spinning once
that is safe. Prop wash is already an open item in the fusion work.

```
count == 0 after 10 s?
├── yes → no edges at all. Pad was already cleared in §2.1, so it is the
│         solder joint or the registers. Check INT_ASYNC_RESET (0x64 bit 4) FIRST
│         — it is the #1 cause. Then INT_SOURCE0=0x08, then continuity CN1·3→P10.7.
└── no
    ├── mean ≈ 1000 us (±3 %) AND (max−min) < ~20 us AND no dt > 1.5 ms
    │   → (A) THE IMU REALLY DELIVERS 1 kHz.
    │     • 50 Hz is discarding 19 of every 20 samples, and everything above
    │       25 Hz — frame and prop vibration lives at 80-300 Hz — ALIASES into
    │       the gyro band. That is the substantive reason 50 Hz is not enough,
    │       independent of this measurement.
    │     • Action: NavTask -> 1 kHz on CPU1 (§5.4 table). flight_ctrl KEEPS
    │       Ts = 0.001f, unchanged. D5 resolved: the IMU rate moves, not the
    │       controller.
    │     • Gate: SPI burst (§5.5) must be << 1 ms. If it measures > ~300 us,
    │       branch to (A'): ODR 500 Hz, NavTask 500 Hz, flight_ctrl Ts = 0.002f.
    ├── mean off nominal by > 1 % (RC oscillator drift)
    │   → (B) Harmless to the ESTIMATOR (it already uses measured dt), fatal to
    │     any fixed-Ts assumption. Action: keep flight_ctrl on the SCHEDULER's
    │     fixed rate (STM-derived, accurate), feed the AHRS/KF the measured dt,
    │     and treat DRDY as a "sample ready" flag, never as the clock. Record the
    │     measured ppm in this file.
    ├── (max−min) > ~100 us, or occasional dt ≈ 2 ms (a dropped edge)
    │   → (C) DO NOT TRUST THE PATH YET. Suspects in order: INT_ASYNC_RESET,
    │     TTL pad mode missing (marginal VIH), 100 us pulse vs a floating line,
    │     a long unshielded wire. Fix and re-measure before any rate decision.
    └── g_imuDrdyStaleTicks routinely ~2 000 000 ticks (20 ms)
        → confirms, from the other end, that the 50 Hz poll is the bottleneck
          rather than the sensor. Expected; it is the evidence for D5.
```

---

## 6. Follow-up edits this design does NOT make

Deliberately left as tasks — `PINNING.md`, `CODEMAP.md` and
`REFACTORING_PLAN.md` are not edited by this file.

**`docs/PINNING.md` §2.2** — add to the IMU table **only once §2.1's self-test
passes** (`Status` column must stay honest):

```markdown
| **P10.7** | **INT1 (data-ready)** | `IfxScu_REQ0C_P10_7_IN` (ERU ch0 → OGU0 → SRC_SCUERU0) | X702·73 ⚠️ unverified | plan |
```

and remove **P10.7** from the §5 free pool, with the same "Removed from the pool"
line style already used for P22.5/P22.6:

```markdown
**P10.7** → IMU INT1 / ERU channel 0 (§2.2, taken 2026-08-xx).
```

**`docs/ICM42688P.md` §3** — the `CN1 3 | INT1 | leave unwired` row becomes
`→ P10.7, direct, TTL pad, no divider`, and §2's "P22.7" reference updates.

**`docs/CODEMAP.md`** — one row: *"IMU data-ready interrupt → `ImuInt.c`,
`ConfigurationIsr.h` (new SRPN), `PINNING.md` §2.2/§5, `ICM42688P.md` §3. **No
A2L, no GUI, no `Xcp_Data`, no diag bit** — the measurement is read by name with
`tools/xcp_read.py`."* (Not edited here: T9 is live in that file.)

**`docs/ILLD_NOTES.md`** — the ERU was a gap; a distilled `IfxScuEru` section has
been added there by this design pass, per the "write it back" rule.

---

## 7. Ordered tasks

Each builds and flies on its own. No big-bang.

| # | files | change | acceptance |
|---|---|---|---|
| **I1** | `BringUp.c` (temp) | Pad self-test on P10.3 / **P10.7** / P10.8 (§2.1) | `g_padProbeP10 == 0x7` over `xcp_read.py`. **Nothing wired to Port 10.** |
| **I2** | — | Identify the header hole (§2.2), then solder CN1·3 → P10.7 | pull-up probe flips on the right hole; continuity CN1·3 ↔ pad |
| **I3** | `Icm42688.c/.h` | INT1 registers §3.2 (`0x14`,`0x63`,`0x64`,`0x65`) | boot dump shows the four values; build + MISRA clean; **`|a|` still 0.998 g** — nothing else regresses |
| **I4** | `ImuInt.c/.h` (new), `ConfigurationIsr.h`, `Cpu0_Main.c` | ERU init §4 + timestamp ISR §5.3 + globals §5.2 | `g_imuDrdyCount` climbs at ~1000/s; CPU0 load in `CoreStats` rises by ≤ 0.3 % |
| **I5** | `SensorTask.c` | `g_imuDrdyStaleTicks` + SPI burst max (§5.5) | both plausible (stale ≈ 0-20 ms; burst ~100 µs) |
| **I6** | `tools/xcp_read.py`, tools/imu_int_stats.py (new) | array reads + the stats/histogram script | 60 s run prints min/max/mean/stddev/p99 + histogram |
| **I7** | this file | paste the measured numbers into §5.6, mark the branch taken | D5 closed with data |
| **I8** | `PINNING.md`, `ICM42688P.md`, `CODEMAP.md` | the §6 edits, once I1-I2 pass | SSoT matches the board |

Unit-testable on the host: the binning/statistics arithmetic factors into a pure
`ImuInt_accumulate(stats*, dt)` with no iLLD include — a fake is not even needed.
The ERU/ISR half is hardware-only by nature.

---

## 8. Risks

- **Wrong header hole** — mitigated by §2.2. This is the single most likely way
  to lose a day, and the mitigation costs five minutes.
- **P10.7 turns out to be committed like P22.7/P22.8** — mitigated by §2.1 running
  *before* the wire, and by two documented fallbacks on the same port.
- **5 V into `INT1`** — the only irreversible risk. Enforced by: self-test first,
  input-only after, push-pull chosen so no 5 V pull-up is ever needed.
- **Priority 106 preempts Ethernet/QSPI.** Deliberate and quantified (~0.1 % of
  CPU0, ≤1 µs added latency). If the XCP link degrades measurably, that is itself
  worth recording — but at 1 µs it will not.
- **`INT_ASYNC_RESET` (0x64 bit 4) left at its default 1** — produces a silent,
  total absence of interrupts that looks exactly like a bad solder joint. Checked
  first in the decision tree for that reason.
- **A2L / GUI contract: untouched.** No `Xcp_Data` field, no A2L regeneration, no
  `AurixGUI` edit, no `diagStatus` bit. Confirm with `a2l.yml` staying green.
- **Nothing here is irreversible in software.** Every task is revertible; only the
  soldered wire is physical, and it is a single signal that the firmware ignores
  unless `ImuInt_init()` is called.
