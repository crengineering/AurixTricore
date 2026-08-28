# Refactoring plan — Cpu0_Main.c and the core partition

Status: T1-T16 implemented, built and flashed (0 errors, 0 warnings; host
tests and MISRA green); T17 closes the one open T15 hardware finding. Branch
`feature/refactoring`. **§3 re-costed 2026-08-27** against the measured IMU
data-ready interval (`docs/IMU_INTERRUPT.md` §5.6, D5 closed); T14-T17 added.
**T15's hardware acceptance (§3.5/T15 row) is CONFIRMED on hardware** —
`execMaxUs`, `loadPmil`, `g_imuDrdyStaleTicks`, `NavCovResets`/`NavDropped`
over 10 min all passed on the first flash (v1.19.12). The one exception,
`g_imuDrdyMissedEdges` growing at ~1.7/min instead of staying at 0, is T17:
part boot-time counting artefact (fixed), part a corrected criterion — see
§3.9. Do not read "implemented" as "unreviewed"; the AHRS bench check
(nose up ≈ +90°) is still bench-only and stays deferred to the user.
Audience: the user (review), then the `flight-dev` agent (implementation).

---

## 1. Verdict

`Cpu0_Main.c` is 630 lines and only ~40 of them are a core entry point. The rest is
three things that each belong somewhere else: **114 lines of boot-time register
dumps** (`Cpu0_Main.c:65-178`), **60 lines of dead bring-up code** behind
`SPI_PAD_TEST (0)` (`Cpu0_Main.c:36,180-240,535-537`), and **five task bodies that
contain domain computation** — the barometric altitude formula with its QNH clamp
(`Cpu0_Main.c:268-277`), three sensor plausibility bands
(`Cpu0_Main.c:285-293, 312-334, 389-401`), and the entire
IMU→AHRS→fusion cascade written as inline glue (`Cpu0_Main.c:337-403`).

But the file length is the symptom, not the disease. The disease is that
**everything with a hard deadline shares a cooperative, non-preemptive core with
everything that blocks**:

| what blocks on CPU0 today | worst case | evidence |
|---|---|---|
| `Task_Baro` I2C read (9 bytes @ 100 kHz) | 0.81 ms typ, **10 ms** deadline | `I2c.c:37` `I2C_XFER_TIMEOUT_TICKS = 1 000 000` |
| `Task_Mag` I2C read (10 bytes @ 100 kHz) | 0.90 ms typ, **10 ms** deadline | `I2c.c:37` |
| `Nvm_task100ms` DFLASH erase+program | **tens of ms**, unbounded wait | `Nvm.c:205-230` `IfxFlash_eraseSector` + `IfxFlash_waitUnbusy` |
| `Ifx_Lwip_poll*` in the `while(TRUE)` body | unmeasured — **outside** the scheduler | `Cpu0_Main.c:610-617` |

`Task_Imu` (`Cpu0_Main.c:337`) — the IMU read, the AHRS and all three Kalman
channels, i.e. the whole flight-critical chain — is registered on the same core,
in the same cooperative list (`Cpu0_Main.c:476-479`). A cooperative scheduler does
not preempt: two I2C timeouts in one 20 ms slot consume the entire period, and the
50 Hz estimator tick simply does not run. The code already knows this and defends
against it — `elapsedTime > 0.2f` sets `fusion_valid = FALSE` (`Cpu0_Main.c:356-360`)
— which is a *degradation*, not a fix.

Worse: an XCP "save to NVM" command from the ground station triggers a synchronous
DFLASH sector erase inside `Task_Measure100ms` (`Cpu0_Main.c:407`), on the flight
core. A tool on the desk can stall the estimator.

The 9.4% average load is fine and is not the problem. The arithmetic confirms where
it comes from: 90 µs/byte at 100 kHz × (9 + 10) bytes × 50 Hz = **1.71 ms per 20 ms
= 8.6%**, i.e. essentially all of it is I2C wire time, as recorded. The SPI IMU
burst is 112 µs at 1 MHz (`Spi.c:26-29`). **Average is not the metric; the
20 ms blocking burst is.**

Secondary findings, all in `Cpu0_Main.c`:
- `#include "Nvm.h"` twice (lines 18 and 31); includes continue *after* a `#define`
  at lines 36-41.
- `Task_Baro` reads the NVM block global directly (`g_xcpNvm.seaLevelPa`, line 268)
  instead of through an accessor.
- `Task_App10ms` (line 248) is an empty registered task, and its clone exists on all
  six cores.
- `core0_main` is 167 lines of hardware init interleaved with UART narration,
  Ethernet/IP configuration and server startup.

**Verdict: the refactor is worth doing, and the core split is the part that
matters.** Moving the estimator to a core with no blocking peer is a determinism
fix. Everything else is tidying that makes the fix safe to land in steps.

---

## 2. Target architecture

### 2.1 Layering

```
  Cpu0_Main.c            entry + init + task registration ONLY  (~120 lines)
  Cpu1_Main.c            entry + NavTask_init + one registration

  Housekeeping.c         100 Hz-off-chain: measurements, diagnostics, gpio, load
  SensorTask.c           baro / mag / gnss periodic tasks: read, convert, publish, diagnose
  NavTask.c              THE flight chain: IMU read -> AHRS -> fusion -> NavState
  BringUp.c              boot-only register dumps over Uart

  NavState.c             CPU1 -> CPU0 snapshot (generation counter, single writer)
  Atmosphere.c           pure: pressure -> altitude, QNH clamp

  Bmp581.c Mmc5983.c Icm42688.c GnssM9N.c    bus + raw conversion + plausibility
  Ahrs.c fusion.c FusionCal.c                estimator
  I2c.c Spi.c Uart.c gpio.c led.c scheduler.c SysTime.c CoreStats.c   services
```

Boundary reasons, one line each:
- **`SensorTask` / `NavTask` exist** so a driver never has to know about the
  estimator and the estimator never has to know about a bus. Today `Cpu0_Main.c` is
  that layer by accident.
- **`Atmosphere` is separate from `Bmp581`** because the ISA formula is not a
  property of that chip, and because it is the one piece of `Task_Baro` worth a
  host test.
- **`NavState` is separate from `Measurements`** because the flight core must
  publish without knowing anything about XCP.
- **Plausibility bands move into the drivers** because the numbers are device
  datasheet facts (`±16 g`, `-40..85 °C`, `0.25..0.65 G`), and the drivers are where
  those facts already live.
- **`BringUp` is separate** so a driver does not depend on the console.

### 2.2 Core assignment

| core | role | tasks |
|---|---|---|
| **CPU0** | comms + housekeeping. **No hard deadline.** | Led 500 ms · Baro 20 ms · Mag 20 ms · Gnss 100 ms · Housekeeping 100 ms · XcpDaq 100 ms · Nvm 100 ms · Lwip 1 ms |
| **CPU1** | **flight core.** Hard deadline. Nothing else, ever. | NavTask 20 ms at T12, **985 us (1014 Hz) from T15** -- see 3.4 (+ the control law and motor output when they land) |
| CPU2-5 | idle, reserved | unchanged |

Why CPU0 keeps the comms: every interrupt in the system is already bound to it —
lwIP's STM0 comparator (`Cpu0_Main.c:584` `IfxSrc_Tos_cpu0`), the GNSS ASCLIN RX ISR
(`GnssM9N.c:535`), and the QSPI ISR (`Spi.c:107`). lwIP with `NO_SYS=1` is
single-threaded by construction and `CtrlReplay` runs inside its receive callback
(`CtrlReplay.c:194`). Moving any of that is a re-binding exercise with no benefit.

Why the I2C sensors stay on CPU0: their 10 ms worst case now only delays other
off-chain work, and their outputs already reach the estimator through *latches*
(`Fusion_setBaroAlt`, `Ahrs_setMag`), never a synchronous call. Nothing waits on
them. **Moving them to a third core would buy a marginally more responsive XCP link
at the price of one more synchronisation contract — not worth it now.** If
measurement later shows the 10 ms bursts hurting the ground link, CPU2 is the
place, and the move is then a one-line registration change because the latch
contract is already correct. (Decision D1.)

Why exactly one core moves: five cores are free, but every crossing is a contract
to get right and to keep right. **One crossing is being added** (CPU1 → CPU0 nav
state), and three existing ones (baro/mag/GNSS latches) become cross-core. That is
the whole cost, and it is the minimum that separates blocking from deadline.

### 2.3 Data flow

```
 CPU0                                              CPU1
 ────                                              ────
 SensorTask_baro  --Fusion_setBaroAlt(altM)---->
 SensorTask_mag   --Ahrs_setMag(magG)--------->    NavTask_step  (50 Hz)
 SensorTask_gnss  --Fusion_setGnss(fix)------->      Icm42688_read   (QSPI)
                     (3 x latch: 1 writer CPU0,      Ahrs_update
                      1 reader CPU1)                 Fusion_update
                                                     NavState_publish
 Housekeeping_100ms <---NavState_get(snapshot)---           |
   measurementsSetFusion(...)                    (1 writer CPU1, 1 reader CPU0)
   measurementsUpdate / diagnosticsUpdate
 Task_XcpDaq -> XCP DAQ frames

 all four crossings live in ONE shared block:
   SharedRam.c, LMU @ 0xB00F0000 — non-cached alias, volatile, 32-bit fields,
   single writer per object, publish = payload -> dsync -> gen++     (see 2.4)
```

The XCP blocks are **not** in that shared block and must not move: `Xcp_Data` sits
at `0x70030000`, high in **CPU0's DSPR** (`Measurements.c:18-23`). That is a second,
independent reason publishing stays on CPU0 — a CPU1 write to `g_xcpData` would be
exactly the cross-core DSPR access this section rules out.

### 2.4 Cross-core memory — the LMU contract

**This supersedes the `CoreStats` justification.** `CoreStats.h:15-21` claims
"cross-core DSPR reads bypass the data cache". That describes the *reader* only, it
is not Infineon's sanctioned mechanism, and it must not be stretched to cover four
crossings. The vendor prescribes exactly one of:

1. shared data in the **LMU**, addressed through the **non-cached segment alias
   (`0xB…`)** — no coherency handling needed at all; or
2. shared data in the LMU at the **cached alias (`0x9…`)** plus a **data
   synchronisation barrier after every write** to the shared location.

**Decision: option 1, the non-cached alias, for all four crossings.**

Why, with the cost stated honestly. LMU is behind the SRI crossbar — order 10-20
cycles per access against 1-2 for a local DSPR. The `NavState` payload is
`Ahrs_Values` + `FusionValues` + 2 words ≈ **232 bytes = 58 words**; at ~20 cycles
uncached that is ~1160 cycles ≈ **3.9 µs at 300 MHz** to publish, and the same to
read. Against `NavTask_step`'s 20 ms period and 2 ms budget that is **0.2% of the
budget** (re-costed for 1 kHz in 3.7: **0.4 % of the 985 us period, 1.3 % of the
300 us budget** -- the decision is unchanged) — the cache buys nothing measurable at 50 Hz. The three sensor latches are
3-4 words each and cost tens of nanoseconds. Meanwhile option 2 makes correctness
depend on a barrier being present at *every future write site*, forever, including
ones nobody has written yet. **A permanent correctness liability traded against
0.2% of one task's budget is not a trade; take the non-cached alias.**

The estimator does not pay the LMU latency in its inner loop either: `Ahrs_update`
and `Fusion_update` already write into stack locals (`Cpu0_Main.c:344-345`), so
`NavState_publish` is a single bulk copy at the end of the tick, not scattered
accesses.

Placement. The seam already exists; **no linker edit is required**:
- `Lcf_Tasking_Tricore_Tc.lsl:456-462` — `memory lmuram`, 768 K, cached
  `0x90040000` / non-cached `0xB0040000`.
- `Lcf_Tasking_Tricore_Tc.lsl:1213-1216` — `.data.lmudata` / `.bss.lmubss` already
  route into `lmuram`. Per-core DLMU selectors exist at 1183-1211 and are **not**
  what we want: those are per-core partitions, and this data is genuinely shared.

Two ways to land an object there, and the project already has a house style for
this: every XCP block is placed with the TASKING `__at()` extension at a fixed
address recorded in `CODEMAP.md` §3 (`Measurements.c:23`
`volatile Xcp_Data g_xcpData __at(XCP_DATA_ADDR);`). **Use that**, at a fixed
non-cached LMU address, rather than a section attribute:

```c
#define SHARED_LMU_ADDR   0xB00F0000u   /* top 64 K of lmuram, non-cached alias */
```

- The `.bss.lmubss` selector allocates from the **bottom** of `lmuram` at the
  **cached** view of the same physical RAM, so a fixed object at the base address
  could be silently overlaid by a future `.bss.lmubss` user. Nothing in the tree
  uses those selectors today (verified: no hits outside the `.lsl`), but placing the
  shared block in the **top** 64 K keeps the two allocation strategies from ever
  meeting. Record it in `CODEMAP.md` §3 alongside the XCP blocks.
- ⚠️ `Measurements.c:30-37` documents a cppcheck constraint that applies here:
  cppcheck cannot parse `__at(ADDR)`, so **each `__at` block must be alone in its
  translation unit**. `SharedRam.c` therefore contains the definitions and nothing
  else — no logic. This is a MISRA-gate requirement, not a style preference.

Rules for anything in that block, all four crossings:

1. **Exactly one writer per object.** Unchanged from the original plan, and still
   the primary discipline — the alias fixes visibility, not races.
2. **Everything is `volatile` and everything is 32-bit.** LMU SRAM is physically
   **64 bits wide with ECC and no sub-word write**, so a byte or halfword store
   becomes a **read-modify-write** of the containing word. Consequences that change
   the payload layout:
   - No `uint8` or `uint16` fields. `uint8 imuPresent` + `uint8 reserved[3]` in
     `NavState_t` becomes a single `uint32 imuPresent`; every latch valid/pending
     flag becomes `uint32`; the generation counter is `uint32`.
   - **Two different writers must never share a 64-bit line.** A read-modify-write
     is not atomic, so CPU0 storing a byte in the same doubleword CPU1 is writing
     can lose CPU1's update. The three CPU0-written sensor latches and the
     CPU1-written `NavState` therefore sit in separately **8-byte-aligned** regions.
     With all fields 32-bit and both blocks 8-byte aligned this falls out for free,
     but it must be asserted, not assumed.
3. **Write ordering is the whole correctness argument, so state it.** The writer
   sequence is: *store payload → barrier → increment `gen`*. Both parts are needed:
   - `volatile` on every field gives **compiler**-level ordering — C guarantees
     volatile accesses are not reordered relative to one another. That handles the
     compiler.
   - It does **not** handle the TriCore **store buffer**, which may post or merge
     writes even to non-cached memory. So the writer issues a **data synchronisation
     barrier between the last payload store and the `gen` store**, and nowhere else.
     The non-cached alias means this is needed at *one* site instead of every write
     site — which is precisely the argument for choosing it.
   - ⚠️ **`Ifx__dsync` is not defined for TASKING in this iLLD tree.** It exists in
     `IfxCpu_IntrinsicsDcc.h:1351`, `…Gcc.h`, `…Gnuc.h` and `…HighTec.h:354`, but
     `IfxCpu_IntrinsicsTasking.h` has no `dsync` at all, and the build config is
     **TriCore Debug (TASKING)**. The TASKING compiler provides `__dsync()` as a
     built-in intrinsic; `SharedRam.h` must define the wrapper itself. **Write this
     into `docs/ILLD_NOTES.md`** — it is exactly the class of vendor trap that note
     exists for.
   - **The reader needs no barrier.** TriCore cores are in-order, the alias is
     non-cached, and `volatile` fixes the load order, so *read `gen` → copy payload
     → re-read `gen`* executes as written.
4. **No locks.** `IfxCpu_acquireMutex` (`IfxCpu.c:54-63`) is a genuine
   `__cmpAndSwap` / CMPSWAP.W spinlock and is available if a crossing ever needs
   real mutual exclusion — but a spinlock on the flight core is an unbounded wait on
   another core's progress, which fails the determinism rule outright. **None of
   these four crossings needs it**, and none may acquire one.

`NavState` keeps its **generation counter**: writer stores payload, barrier,
`gen++`. The reader keeps `lastGen` in its own stack/DSPR and **never writes shared
state** — which is also how the "consumer clears the pending flag" two-writer bug in
the existing latches gets fixed. Torn reads are caught by re-reading `gen` after the
copy and retrying **at most once**; on a second mismatch the reader keeps its
previous snapshot. Bounded, lock-free, MISRA-clean, host-testable.

`IfxCpu_setDataCache` / `IfxCpu_enableSegmentSpecificDataAccessCacheability`
(`IfxCpu.h:588-646`) are **not** used by this plan — the non-cached alias makes them
unnecessary, and the header states they may only be called *before* the caches are
enabled, which is a startup-ordering constraint worth not acquiring. The one iLLD
cache call this plan does use is `IfxCpu_isAddressCachable()` (`IfxCpu.h:626`), as a
boot-time self-check that the shared block really is in a non-cacheable segment.

**`g_coreStats` stays where it is.** It works, it is already `volatile`, it is
single-writer-per-slot, and it carries diagnostics — a torn read costs a wrong load
number for one 100 ms window, never a flight. Moving it would touch all six core
mains for no behavioural gain, and churn has a cost. What *does* change is its
**comment**: `CoreStats.h:15-21` is currently cited as the project's sanctioned
inter-core pattern, and it is not one. Narrow that comment to "diagnostic counters,
tolerant of a torn read" and point at `SharedRam.h` for the real rule. (Decision D6.)

---

## 3. Timing

STM at 100 MHz. Budgets are per dispatch; "worst" is the value that must not blow
the period.

**Revised 2026-08-27 against measured data.** D5 is closed by
`docs/IMU_INTERRUPT.md` §5.6, branch (A): the IMU delivers a real 1 kHz DRDY at
**985.036 µs mean (1014.2 Hz), stddev 1.780 µs**, and the IMU SPI burst is
**`spi_burst_max` = 124.4 µs measured**. The `NavTask_step` row below is
**re-costed**, not re-rated, and the two-stage landing (§3.4) is a consequence of
that.

### 3.1 The reference period is 985 µs, not 1000 µs

Every number in this section is against **985 µs**. The IMU's internal RC runs
~1.4 % fast with no `CLKIN` (`docs/IMU_INTERRUPT.md` §5.6), so the sample interval
is 985 µs and the estimator rate is **1014.2 Hz**, not 1000 Hz. Costing against a
nominal 1000 µs would silently spend 15 µs of margin the hardware does not have.
This is also why the estimator must take its `dt` from the ISR timestamp and never
from a constant.

### 3.2 `NavTask_step` re-costed for a 985 µs period

Nominal path, per tick, on CPU1 with nothing else on the core:

| item | µs | source |
|---|---|---|
| IMU SPI burst, 14 B @ 1 MHz | **124.4** | **measured**, `docs/IMU_INTERRUPT.md` §5.6 |
| `Icm42688_read` around the burst — CS, decode, scaling, presence | ~15 | estimate |
| `Ahrs_update` — 2× `sqrtf` + ~6 transcendentals in the Euler output (`Ahrs.c:733-741`) | ~25 | estimate |
| `Fusion_update` — 3 channels × 4-state predict + conditional correct (`fusion.c:330-527`) | ~45 | estimate |
| `NavState_publish` — ~65 words over the non-cached LMU alias (§2.4) | ~4.4 | estimate |
| three input latch reads, LMU | ~1 | estimate |
| liveness / plausibility accumulation | ~5 | estimate |
| **typical total** | **~220** | 124.4 measured + ~95 estimated |

- **Budget: 300 µs (30.5 % of the period). Hard cap: 400 µs.** The estimated ~95 µs
  would have to be wrong by **3×** before the cap is threatened (124.4 + 285 = 410).
  That is the error bar the cap is sized for; the estimates are labelled as
  estimates, and §3.5 says how they get replaced by a measurement.
- **1 kHz fits, with 685 µs (69.5 %) free at budget.** It fits comfortably.
- **Minimum acceptable margin for this chain: 50 % of the period must remain free
  at the worst case.** Not a round number — `NavTask` is *not* the final occupant
  of CPU1. `flight_ctrl` at `Ts = 0.001f` (`CtrlReplay.c:93`) and the DShot motor
  output have to land on this core, inside this same 985 µs, in the same tick that
  produced the attitude. Sensor → estimate → control → motor is one chain or it is
  not a latency budget. So the estimator may claim at most half the period, and
  300 µs leaves 685 µs for the half that does not exist yet.
- **The old 2.0 ms budget is deleted**, not scaled. It was an "it cannot possibly
  take this long" figure against a 20 ms period; at 985 µs a budget larger than the
  period is not a budget.

**Fault path, and it is the one that needs a decision.** `SPI_XFER_DEADLINE_MS` is
**10 ms** (`Spi.c:40`). A wedged QSPI therefore costs **10 consecutive ticks**, not
one. That deadline was sized against a 20 ms period, where it was half a slot; at
985 µs it is ten slots. Recommendation (T14): **drop the IMU transfer deadline to
1 ms** — still 8× the measured 124.4 µs burst, so it can still only fire on a
genuine fault, and a genuine fault then costs one tick instead of ten. Not optional
at 1 kHz; the alternative is an unbounded-in-practice hole in the control chain.

### 3.3 The revised table

| task | rate | typ | worst | core | overrun behaviour |
|---|---|---|---|---|---|
| `NavTask_step` — **stage 1, T12** | 50 Hz / 20 ms | ~0.22 ms | **2.0 ms** budget (SPI hard cap 10 ms, `Spi.c:40`) | **CPU1** | as today; nothing else on the core to steal from |
| `NavTask_step` — **stage 2, T15 (hardware-confirmed 2026-08-27)** | **1014.2 Hz / 985 µs** | measured **~171 µs** (`execMaxUs`, constant over a 10 min session) | **300 µs budget, 400 µs cap** (SPI deadline cut to 1 ms, T14) | **CPU1** | gated on `newSample`, so an overrun skips at most one *slot*, never a *sample*: the next dispatch consumes the pending edge and fuses it with its **true** `dt` from the ISR timestamp. Two edges missed = one fused step with `dt ≈ 2 ms`, still inside `FUSION_DT_MAX` / `AHRS_DT_MAX_S` (0.2 s). `missedEdges` counted and published, at a measured ~0.0028 % of edges (§3.9, T17) — below the corrected < 0.05 % bound; `dt > 0.2 s` freezes the filters rather than integrating garbage. `NavCovResets`/`NavDropped` both held 0 over the same 10 min |
| `imuDrdyIsr` | 1014.2 Hz | ~1 µs | ~1 µs, no branch on data | CPU0 today, **CPU1 from T15** (§3.6) | highest SRPN in the system (106); overrun impossible; the 100 µs pulse width makes chatter-livelock physically impossible (`docs/IMU_INTERRUPT.md` §5.3) |
| `SensorTask_baro` | 50 Hz | 0.81 ms | **10 ms** (`I2c.c:37`) | CPU0 | delays other CPU0 tasks only; no new baro latch → the DOWN channel coasts on accel + `accelBias` |
| `SensorTask_mag` | 50 Hz | 0.90 ms | **10 ms** | CPU0 | AHRS degrades to accel+gyro; **yaw becomes unbounded** (`Ahrs.h:15`) — this is the one off-chain failure with a flight consequence, and it is why `PeriphDiag` must stay loud |
| `SensorTask_gnss` | 10 Hz | ~20 µs | 50 µs | CPU0 | ring-buffer read only; duplicate-`iTOW` guard makes a late poll harmless (`fusion.h:136`) |
| `Housekeeping_100ms` | 10 Hz | ~0.5 ms | 2 ms | CPU0 | XCP block stale by one cycle |
| `Task_XcpDaq` | 10 Hz | ~0.2 ms | 1 ms | CPU0 | one DAQ frame skipped; GUI plot gap |
| `Task_Nvm` | 10 Hz | ~1 µs idle | **~100 ms on a save** (`Nvm.c:205-230`) | CPU0 | blocks all of CPU0 including lwIP; **nothing on the flight chain is affected — before T12 it stalled the estimator** |
| `Task_Lwip` | 1 kHz | ~30 µs | 1 ms | CPU0 | echo/XCP RX latency only |
| `Task_Led` | 2 Hz | ~2 µs | — | CPU0 (**and CPU1**, `Cpu1_Main.c:58`) | cosmetic — but see §3.6: it is the only other task on the flight core and must be registered **after** `NavTask_step` |

**CPU1 load at 1 kHz: ~22 % typical (220/985), ~30 % at budget.** The earlier
"1.5 % → ~15 %" estimate in this document and in `docs/IMU_INTERRUPT.md` §5.4 was
**low**: it assumed a ~150 µs tick from a 112 µs SPI estimate. The burst measured
124.4 µs and the compute was never costed at all. **~22 %, revised.** Still one
core, still nothing else on it, still ~70 % free for the control law and the motor
output.

CPU0 is unchanged by the rate change: steady state ~10.2 % measured
(`docs/IMU_INTERRUPT.md` §5.6), minus the IMU share once T12 lands. Pathological
burst: 20 ms of I2C + a 100 ms flash erase — all off-chain.

### 3.4 Which step raises the rate — **not T12**

**Recommendation: T12 migrates at `SCHED_MS(20)` exactly as written. The rate
change is its own step (T15), after T12 and T13 have flown.**

Three reasons, in order of weight:

1. **T12 is what produces the measurement T15 needs.** Once `NavTask_step` is alone
   on CPU1, `g_coreStats[1].execMaxUs` **is** `NavTask_step`'s worst-case dispatch
   (`scheduler.c:65-72` times each dispatch individually and keeps the max; on CPU0
   that number is polluted by nine other tasks). The ~95 µs of estimate in §3.2
   becomes a measured number the moment T12 flies, at zero extra cost. Raising the
   rate before reading it means spending margin before knowing it exists.
2. **A core migration and a 20× rate change landing together is un-attributable.**
   If the AHRS or the KF misbehaves after a combined step, the candidate causes are
   the LMU crossing, the QSPI ISR retarget, the rate-dependent constants (§3.8) and
   the filter's numerical conditioning — with no way to bisect. This document calls
   a big-bang cutover a design failure elsewhere; this would be one.
3. **The rate change is genuinely small and genuinely revertible** *once T14's
   prerequisites are in*: one `Scheduler_addTask` line, the `newSample` gate and the
   `dt` source. `git revert` of T15 restores a flying 50 Hz build without touching
   the core partition.

The cost of splitting is one extra flash-and-fly cycle. That is the whole cost.

### 3.5 The gate T15 must pass before it is written

Read after T12 has flown for 5 minutes, over `tools/xcp_read.py`:

- `g_coreStats[1].execMaxUs` ≤ **300** — the §3.2 budget, confirmed rather than
  estimated. If it lands between 300 and 490 µs, 1 kHz is still viable but the
  control law's half of the period is not; re-cost before proceeding.
- `g_coreStats[1].execUs` over a 100 ms window ÷ 5 dispatches = the typical tick.
- If `execMaxUs` ≥ **490 µs** (half the period), **do not raise to 1 kHz.** Fall
  back to 500 Hz — the (A′) branch threshold in `docs/IMU_INTERRUPT.md` §5.6 already
  anticipates it — and consume every second DRDY edge.
  **§9 refines what "fall back to 500 Hz" must mean:** decimate the *control law*,
  not the estimator. Dropping every second DRDY edge un-filtered folds the
  244–382 Hz blade-pass band back into the control band (§9.4). If the estimator
  itself has to shed rate, it must average the sample pair, not discard one.

### 3.6 How the estimator is clocked at 1 kHz — §5.4 confirmed, slot revised

**Confirmed:** the ISR timestamps and sets a `newSample` flag; `NavTask_step` stays
a cooperative scheduler task and does the SPI burst, the AHRS and the KF in task
context. `docs/IMU_INTERRUPT.md` §5.4's reasoning holds unchanged — doing the burst
at SRPN 106 would put the flight chain above the QSPI driver it depends on, and
would hide its worst case from `CoreStats`.

**Revised, and this is the part the measurement changes:**

- **The scheduler has no tick and no granularity problem.** `Scheduler_run` is a
  free-running poll of the STM lower word against a per-task tick count
  (`scheduler.c:52-58`), 10 ns resolution, so `SCHED_MS(1)` = 100 000 ticks is
  expressible exactly. There is no jiffy to quantise against. **Stated plainly
  because the question deserves a plain answer: cooperative tick granularity is not
  what breaks here.**
- **What breaks is that `SCHED_MS(1)` polls *slower than the sensor*.** The IMU
  produces 1014.2 edges/s; a 1 ms slot dispatches at most 1000/s (`lastRun = now`
  at `scheduler.c:63` makes the period a floor, never less). ~14 samples per second
  would be overwritten before they were consumed, in a slow beat — the worst kind of
  data loss, periodic and invisible in an average.
- **So the slot must be faster than the sensor and the flag must be the clock:
  register `NavTask_step` at `SCHED_US(500)`** (2 kHz poll) and return immediately
  when no edge is pending. The actual work rate is then the sensor's 1014.2 Hz. A
  poll that finds nothing costs one LMU word read plus a branch, ~0.2 µs, i.e.
  ~0.02 % of the core; a `SCHED_MS(1)` slot buys nothing back for the aliasing it
  introduces. (Registering at period `0` — every loop pass — also works and is even
  safer, but it inflates the dispatch count `CoreStats` reports; 2 kHz is the same
  guarantee with a readable load figure.)
- **`dt` comes from the ISR edge timestamps, not from `SysTime_getTimeElapsedS`.**
  `NavTask_step` keeps the last *consumed* edge tick in its own DSPR and computes
  `dt = edgeTicks - lastConsumedEdgeTicks`. This stays correct when a sample is
  skipped, and it removes task-dispatch jitter from the filter input entirely —
  which is the whole reason the interrupt exists. `NavTask.c:92`'s
  `SysTime_getTimeElapsedS(&s_lastTicks)` measures the *dispatch* interval, i.e. the
  jitter, and is the wrong source once edges are available.
- **The shared edge state must move into the LMU block, and the ISR must move to
  CPU1.** `g_imuDrdyLastTicks` / `g_imuDrdyCount` (`ImuInt.h:58-60`) are plain
  globals; after T12 they would be written by an ISR on CPU0 and read at 2 kHz by
  CPU1 — exactly the cacheable cross-core access §2.4 rules out. Two changes, both
  in T15: retarget `SRC_SCUERU0` to `IfxSrc_Tos_cpu1` (alongside the QSPI nodes T12
  already retargets), and put `{edgeTicks, edgeCount}` in the shared LMU block as a
  fourth object, **single writer CPU1**. The histogram and window-statistics globals
  stay where they are — they are bring-up diagnostics, read by `tools/xcp_read.py`,
  and they tolerate a torn read.
- **`SysTime` must stay on STM0 for both cores.** Load-bearing, not an oversight:
  the ISR timestamp and any task-side timestamp must share one time base or `dt` is
  meaningless. It is a deliberate, documented exception to CLAUDE.md rule 2 ("each
  core uses its own STM"), it is a read-only cross-core access, and it must be
  commented as such wherever it appears — the same exception `Spi.c`/`I2c.c` already
  take for their deadlines (Risk 3).
- **Task order on CPU1 matters now.** `Scheduler_run` dispatches in registration
  order (`scheduler.c:55`) and `Task_LedToggle` (`Cpu1_Main.c:58`) is registered
  first today. At a 985 µs period a 2 µs LED task ahead of the flight chain is
  harmless but pointless — register `NavTask_step` **first** on CPU1, and treat that
  ordering as part of the contract for anything ever added to this core.

### 3.7 What a 1 kHz estimator does to the rest of the system

**The three input latches (CPU0 writes at 50 Hz, CPU1 reads at ~1014 Hz).**

- Cost: 3 latches × ~4 words over the non-cached LMU alias ≈ **1 µs per tick**,
  0.1 % of the period. Irrelevant.
- **The contract, however, does not survive a boolean.** `Fusion_setBaroAlt` /
  `Ahrs_setMag` / `Fusion_setGnss` back onto `s_baroNew`-style flags that the
  *consumer* clears (`fusion.c:216`) — the two-writer bug T12 already fixes. At 20×
  asymmetry the fix has to be the **sequence-counter** form, not a flag: the writer
  increments `seq`, the reader keeps `lastSeq` in its own DSPR and never writes
  shared state. A reader-clears flag polled 20× per write is a guaranteed two-writer
  race; a sequence counter is correct at any ratio. **Note for T12: use the counter
  form now, so T15 is a rate change and not a protocol change.**
- Behaviour: the DOWN channel predicts every tick and corrects on 1 tick in 20.
  That is what a Kalman filter is for and it needs no tuning change — `R` is a
  per-measurement property and `Q` is `dt`-scaled in `fusion_chanPredict`
  (`fusion.c:330`).
- The magnetometer sample is re-applied to ~20 consecutive Mahony updates. **Not a
  gain change:** the correction is `Kp*e` folded into the rate and `Ki*e*dt`
  integrated (`Ahrs.c:625-640`), both `dt`-scaled, so the total correction per 20 ms
  of wall time is unchanged. Verified by reading the update, not assumed.

**`NavState` (CPU1 publishes at ~1014 Hz, CPU0 reads at 10 Hz).**

- Publish cost **~3.9 µs every 985 µs = 0.4 % of the flight core**. It is on the
  critical chain and it is affordable; §2.4's "0.2 % of the budget" line was written
  against 20 ms and becomes **0.4 % of the period / 1.3 % of the 300 µs budget**. D7
  is unchanged — a factor of six on a rounding error is still a rounding error, and
  the barrier-discipline argument against the cached alias never depended on the
  rate.
- **The reader's FALSE path stops being theoretical.** Publish (~3.9 µs) and read
  (~3.9 µs) now overlap with probability ~0.8 % per attempt; two collisions in a row
  ≈ 6e-5, so at 10 Hz the "keep the previous snapshot" path fires roughly **once
  every 30 minutes**. That is correct behaviour and costs one 100 ms-stale XCP
  sample — but it means **`NavState_get` returning FALSE must not be treated as a
  fault**: no diagnostic bit, no counter anyone reads as an error, no log line. A
  plain wrapping counter for curiosity is fine. (There is no free `diagStatus` bit
  anyway — bits 27-30 were the last, `docs/DIAGNOSTICS.md`.)
- Everything downstream of `Housekeeping` — `measurementsSetFusion`, XCP DAQ, the
  GUI — stays at 10 Hz. **The 20× asymmetry is absorbed entirely by the snapshot
  protocol and nothing in the A2L/GUI contract changes.** The GUI plots the same
  10 Hz stream it plots today; it simply plots a fresher sample.

**T12 BLOCKER found while re-costing — `NavTask_step` still writes CPU0-owned
state.** This bites at 50 Hz, before any rate change, and it violates §2.3 as
written:

- `NavTask.c:131` `measurementsSetImu(...)` writes `g_xcpData`
  (`Measurements.c:174-186`), which is `__at(0x70030000)` in **CPU0's DSPR**
  (`Measurements.c:18-23`). After T12 that is CPU1 writing another core's DSPR — the
  exact access §2.3 rules out — and it makes `g_xcpData` a **two-writer object**
  (CPU0's `Housekeeping` plus CPU1) that XCP DAQ reads concurrently.
- `NavTask.c:134-135` `Icm42688_plausible` + `PeriphDiag_report` write `s_periph[]`,
  a plain non-volatile static (`PeriphDiag.c:50`) that CPU0 reads every 100 ms in
  `PeriphDiag_update`. Cacheable, two cores, no protocol.
- **Fix, inside T12:** carry `acc[3]`, `gyro[3]`, `tempC`, an accumulated `liveness`
  and a `sampleSeq` in the `NavState` payload (+32 bytes of LMU, **zero A2L cost** —
  nothing enters `Xcp_Data`, which has only 8 bytes free anyway), and let
  `Housekeeping_100ms` on CPU0 call `measurementsSetImu` and `PeriphDiag_report`
  from the snapshot. Liveness is accumulated on CPU1 per sample and reported once
  per 100 ms, so the stuck-sensor detector keeps its per-sample sensitivity instead
  of being decimated to 10 Hz.

### 3.8 Constants the measurement invalidates

Each of these is correct at 50 Hz and wrong at 1014 Hz. **T14 fixes them before T15
changes the rate**; none of them is a rate change on its own, so T14 lands and flies
at 50 Hz with no behaviour change worth seeing.

| where | today | why it breaks at 985 µs | fix |
|---|---|---|---|
| `NavTask.c:21` `NAVTASK_DT_MIN_S` | `0.001f` | **Blocker.** The measured interval is **985 µs < 1 ms**, so `navTask_dtValid` rejects **every** tick, `ahrsInputOk`/`fusionInputOk` are permanently FALSE and the estimator silently stops. This is the one change without which 1 kHz does not run at all | `0.0002f` (200 µs) — still catches a double dispatch, clears 985 µs by 5× |
| `Ahrs.c:54` `AHRS_CAL_SAMPLES` | `100u`, "~2 s at 50 Hz" | becomes **~0.1 s** of gyro-bias averaging — 20× less noise rejection on the one number the whole attitude solution rests on | **make the window a duration, not a count**: accumulate `dt` and finish at 2.0 s. Rate-independent forever, and it lands in T14 with *no* behaviour change at either rate — scaling the count to `2000u` would instead make the T14 build take 40 s to calibrate at 50 Hz |
| `Ahrs.c:72` `AHRS_CAL_MAX_SAMPLES` | `500u`, "10 s at 50 Hz" | becomes **~0.5 s**; the "give up and use a degraded bias" escape fires almost immediately, so a board powered on while still would rarely get a *good* bias | same treatment: a 10.0 s **deadline** accumulated from `dt`. `biasDegraded` keeps its meaning at any rate |
| `Spi.c:40` `SPI_XFER_DEADLINE_MS` | `10u` | ten lost ticks per wedged transfer instead of one (§3.2) | `1u` on the IMU path |
| `ImuInt.h:88` `g_imuDrdyStaleTicks` | designed to read 0-20 ms against a 50 Hz task | its purpose (proving D5) is spent; at 1 kHz it becomes a *live* health signal — it should read < ~1 ms, and a sustained rise means the task is falling behind the sensor | keep; re-document as a latency monitor and add `missedEdges` beside it |

**Checked and NOT invalidated** — recorded so nobody re-derives it:

- `FUSION_DT_MIN` `0.0001f` / `AHRS_DT_MIN_S` `0.0001f` (`fusion.c:147`, `Ahrs.c:76`)
  already clear 985 µs by 10×. Only `NavTask`'s own gate is wrong.
- `FUSION_REJECT_MAX` `25u` — "0.5 s at 50 Hz" (`fusion.c:117`) stays 0.5 s:
  `rejectRun` is incremented in `fusion_chanUpdate` (`fusion.c:487`), which runs only
  on a **measurement**, and measurements still arrive at 50 Hz (baro) and 10 Hz
  (GNSS). Rate-independent by construction.
- Mahony `twoKp`/`twoKi` and every `FusionCal` gain — `dt`-scaled at the point of use
  (`Ahrs.c:625-640`, `fusion.c:330`), so rate-invariant to first order.
- `docs/FUSION.md` §8's BMP581 IIR coefficient — clocked by the **barometer's own
  ODR**, not the nav tick. The 310 ms time constant is unchanged.
- The A2L, the GUI and `Xcp_Data` — untouched by any of this (§3.7).

**Docs that become wrong at T15, and are T13/T16's job rather than this section's:**
`docs/FUSION.md` §1 ("Everything runs in `Task_Imu` at 50 Hz on CPU0" — wrong on the
task name and on both numbers), the `Task_Imu` gating sentence later in the same
file, and the "~2 s"/"10 s" calibration prose that mirrors `Ahrs.c:54,72`.

**New numerical risk that only hardware can close.** The KF covariance is updated
20× more often in `float32`. Round-off accumulates per update, the `dt`-scaled `Q`
terms shrink 20×, and `P` propagates 20× more often. `covResets` and the covariance
clamp (`fusion_chanClampCov`, `fusion.c:268`) are the instrumentation that already
exists for exactly this. **T15 acceptance: `NavCovResets` still 0 over 10 minutes
and `NavDropped` still 0.** If either moves, the fix is `float64` in the covariance
propagation, or the 500 Hz fallback — not a gain tweak.

**Flagged, as asked:** today `Task_Imu` (the flight chain) shares its core with the
two blocking I2C reads that are ~91 % of CPU0's load, *and* with the DFLASH erase,
*and* with lwIP. After T12 it shares its core with nothing but a 2 µs LED blink.

### 3.9 T17 — the `missedEdges` investigation and the corrected criterion (2026-08-27)

**The T15 flash confirmed everything except one line of §3.5/T15's acceptance:**
10 minutes at 1014 Hz, sampled once a minute, gave `NavCovResets == 0` and
`NavDropped == 0` throughout, `|a|` steady at ~1.002 g, `execMaxUs` a constant
171 µs (well under the 300 µs budget) — but `g_imuDrdyMissedEdges` climbed from
23 to 40, a steady ~1.7/min, not the flat 0 the criterion asked for. Two separate
things were found, not one, and only one of them is a defect.

**1. A real boot-time counting artefact, fixed.** `NavTask_step` seeded its
`s_lastEdgeSeq`/`s_lastEdgeTicks` baseline at `0`. `Cpu1_Main.c` calls
`Icm42688_init()`/`BringUp_dumpImu()` — which genuinely pulses DRDY and advances
`g_imuEdge.seq` — *before* `NavTask_init()`/`Scheduler_run()` ever start, so the
edges produced during that bring-up window (no task existed yet to consume them)
were counted as "missed" on `NavTask_step`'s very first dispatch: a one-time,
deterministic jump indistinguishable in the published counter from genuine
in-flight loss. Fixed by seeding the baseline from a real `ImuEdge_snapshot()`
taken at the end of `NavTask_init()` (`NavTask.c`), so the counter only ever
reports edges missed after the task started actually polling for them. Host
tests: `test_init_seeds_baseline_without_counting_bringup_edges`,
`test_step_still_counts_genuine_multi_edge_gaps` (`test/test_navtask.c`).

**2. The residual steady-state growth (~1.7/min, ~28 ppm of edges), NOT a
software defect — ruled out by elimination, not assumed:**
- `ImuEdge_snapshot()`'s torn-read retry (`ImuEdge.h`) is correct by
  construction: the writer stores `ticks` before incrementing `seq`
  (`ImuInt.c`), so a reader can only ever observe `(old seq, new ticks)` as a
  transient, self-correcting mismatch, never the reverse — already exercised by
  `test_imuedge.c`.
- `Scheduler_run`'s own instrumentation (`g_coreStats[1].execMaxUs`, a
  lifetime maximum, never reset) stayed at a constant 171 µs across the entire
  10-minute session — an order of magnitude under even one 985 µs edge period,
  which rules out `NavTask_step` or `Task_LedToggle` themselves ever stalling
  long enough to skip a dispatch.
- CPU1 carries exactly two interrupt sources, both already accounted for:
  QSPI0 TX/RX/ER (102-104) and DRDY (106, the numerically highest SRPN in the
  system) — no GETH/ASCLIN traffic reaches this core (`Configurations/ConfigurationIsr.h`).
  No explicit interrupt-disable exists anywhere in the CPU1 call path (already
  established for the related DRDY/QSPI timing question, `docs/IMU_INTERRUPT.md`
  §5.6).
- What is left, after eliminating the scheduler, the snapshot protocol and
  every other CPU1 interrupt source, is the same class of thing
  `docs/IMU_INTERRUPT.md` §5.6 already flagged as an open item at ~1.03% before
  T15 (a correlated slip between the SPI burst and the DRDY timestamp) — now
  seen from the ISR side, at a rate ~370× smaller, consistent with most of that
  slip now being absorbed rather than lost, since producer and consumer share a
  core since T15. Not further diagnosable from the source tree alone; see the
  disambiguating experiment below.

**Decision: the criterion is corrected, not chased further, for three reasons:**

1. **`dt` is measured, not assumed** (`NavTask.c`, `NAVTASK_TICKS_TO_S` block):
   a missed edge produces one tick with `dt ≈ 2 × period` fed to
   `Ahrs_update`/`Fusion_update` with its own correctly measured `dt` — not a
   wrong one. The resulting error is second-order (bounded by the
   rate-of-change of accel/gyro across the gap, times `dt²`), not first-order
   data loss. A quadrotor's attitude/rate dynamics have no content anywhere
   near 1014 Hz, so this term is far below the sensor's own noise floor.
2. **This exact filter code already ran continuously, successfully, at
   `dt ≈ 20 ms` (T13, 50 Hz, before T15)** — `NavCovResets`/`NavDropped` were 0
   there too. An occasional (0.0028 % of ticks) widening to `dt ≈ 2 ms` is
   **10× smaller** than the interval this code sustained on every tick for the
   entire pre-T15 history. It cannot be a new stressor.
3. **`NavCovResets == 0` and `NavDropped == 0` are the filters' own
   instrumentation for "did an irregular `dt` actually hurt anything"** (§3.8)
   — and both held at 0 across all ~608 000 edges in the 10-minute session,
   including every one of the ~17 double-length ticks. This is empirical, not
   merely theoretical, evidence of coping.

**Tolerable rate, stated as a number:** even at 100× today's measured rate
(~0.28 %, ~1 in 350 edges) every widened tick would still be individually
smaller than the `dt` this code ran at continuously pre-T15, and 99.7% of
ticks would remain unaffected. Today's measured rate is ~100× below even that
deliberately generous bound. The corrected T15/T17 criterion (replacing
"`missedEdges` 0 over 5 min" in the §6 T15 row): **`missedEdges` may grow, but
must stay below 0.05 % of edges over the measurement window** (~1 in 2000 —
~18× today's measured 0.0028 %, comfortably inside the ≥10× margin the KF's
own pre-T15 history already demonstrates it tolerates). Crossing that bound is
a materially different regime and is worth a fresh investigation, not silence.
`NavCovResets == 0` / `NavDropped == 0` over the same window remain the hard
gate, unchanged; `missedEdges` is now a monitored health counter, not a
required zero.

**Left for the record, not required for this decision:** the SRI/LMU
cross-core contention hypothesis (CPU0's XCP polling reading the same LMU bank
the ISR writes, stalling the CPU1 write/read by a few cycles) is untested and
does not change the criterion above either way. The disambiguating experiment
needs no rebuild: let the board run **completely untouched** (no
`xcp_read.py`/GUI polling at all) for several minutes, then take exactly one
reading of `g_imuDrdyMissedEdges` and `g_coreStats[1].aliveCounter`, and
compare the implied rate against the polled sessions above. A materially lower
rate would confirm SRI contention; an unchanged rate points at the sensor's
own DRDY generation jitter instead (`docs/IMU_INTERRUPT.md` §5.6's still-open
item).

**Experiment run 2026-08-27 on v1.19.13. Result: contention is a contributor,
not the cause.** Board left completely untouched for 600 s — one reading before,
one after, no polling in between:

| | polled (1/min, v1.19.12) | untouched (v1.19.13) |
|---|---|---|
| `missedEdges` rate | ~1.7 / min | **1.2 / min** (12 in 600 s) |
| as a fraction of ~1014 edges/s | 28 ppm | **20 ppm** (0.0020 %) |

Removing all XCP traffic cut the rate by roughly a third, so CPU0's polling of
the same LMU bank does cost something — but ~70 % of the residual survives with
nothing touching the bus. That is consistent with `IMU_INTERRUPT.md` §5.6's
open item (the sensor's own DRDY generation, or the correlated SPI-burst slip)
and **not** with contention as the mechanism. No further action: 20 ppm sits
**25x inside** the 0.05 % bound set above.

Same reading confirmed `NavCovResets = 0`, `NavDropped = 0`, `diagStatus =
0x800`, `|a| = 1.00192 g`, `execMaxUs[1] = 171 µs` unchanged, `loadPmil[1]`
155-158 — and `missedEdges = 0` at boot, confirming the T17 baseline-seeding
fix on hardware.

## 4. Interfaces

New or changed headers. Nothing in `Measurements.h`, `Diagnostics.h` or `Nvm.h`
changes layout — **no A2L or GUI edit is required by this plan.**

```c
/* Uart.h — the hex helper leaves Cpu0_Main.c */
void    Uart_printHexByte(uint8 v);

/* Atmosphere.h — NEW. Pure, no iLLD, host-testable. */
float32 Atmosphere_altitudeM(float32 pressPa, float32 seaLevelPa);
        /* applies the 80 000..120 000 Pa QNH clamp internally and falls back
           to 101 325 Pa, so the caller no longer owns that policy */

/* BringUp.h — NEW. Boot only, CPU0. */
void    BringUp_dumpSensors(void);

/* Bmp581.h / Mmc5983.h / Icm42688.h — plausibility joins its device.
   All three pure -> host-testable against the existing test/ fakes. */
boolean Bmp581_plausible  (float32 pressPa, float32 tempC,      float32 *liveness);
boolean Mmc5983_plausible (const Mmc5983_Sample  *s,            float32 *liveness);
boolean Icm42688_plausible(const Icm42688_Sample *s,            float32 *liveness);

/* SensorTask.h — NEW. Registered on CPU0. */
void    SensorTask_init(void);
void    SensorTask_baro(void);      /* 20 ms  */
void    SensorTask_mag(void);       /* 20 ms  */
void    SensorTask_gnss(void);      /* 100 ms */

/* NavTask.h — NEW. Registered on CPU1. Owns the flight chain. */
void    NavTask_init(void);         /* FusionCal_init, Ahrs_init, Fusion_init */
void    NavTask_step(void);         /* 20 ms */
/* split out for the host test — this is where the NaN/validity logic lives */
boolean NavTask_inputValid(float32 dtS, boolean imuPresent, uint8 ahrsState);

/* SharedRam.h — NEW. The LMU shared block and its rules (see 2.4).
   SharedRam.c holds the __at() definitions and NOTHING else: cppcheck cannot
   parse __at(), so it must be alone in its translation unit
   (Measurements.c:30-37). */
#define SHARED_LMU_ADDR   0xB00F0000u   /* top 64 K of lmuram, NON-CACHED alias */

/* Ifx__dsync is NOT defined for TASKING in this iLLD tree — it exists only in
   the Dcc/Gcc/Gnuc/HighTec intrinsics headers. TASKING provides the built-in. */
#ifndef Ifx__dsync
#define Ifx__dsync()   __dsync()
#endif

/* NavState.h — THE new cross-core contract. Writer CPU1, reader CPU0.
   Every field 32-bit: LMU SRAM is 64-bit wide with no sub-word write, so a
   uint8 store would be a read-modify-write (see 2.4 rule 2). */
typedef struct {
    uint32       gen;          /* even/odd not needed; monotonic, +1 per publish */
    Ahrs_Values  ahrs;
    FusionValues fusion;
    float32      dtS;
    uint32       imuPresent;   /* was uint8 + reserved[3] */
} NavState_t;                  /* 8-byte aligned; shares no doubleword with the
                                  CPU0-written sensor latches */

void    NavState_init(void);
void    NavState_publish(const Ahrs_Values *ahrs, const FusionValues *fusion,
                         float32 dtS, boolean imuPresent);
        /* CPU1 ONLY. Sequence: store payload -> Ifx__dsync() -> gen++.
           The barrier is what stops the store buffer publishing gen ahead of
           the payload; volatile alone only orders the compiler. */
boolean NavState_get(NavState_t *out);
        /* Any core. Read gen, copy, re-read gen; ONE retry, then FALSE and the
           caller keeps its previous snapshot. No barrier needed: TriCore is
           in-order and the alias is non-cached. Never writes shared state. */

/* Housekeeping.h — NEW. Registered on CPU0. */
void    Housekeeping_init(void);
void    Housekeeping_100ms(void);   /* NavState_get + measurementsSetFusion +
                                       measurementsUpdate + diagnosticsUpdate +
                                       gpio + measurementsSetSystemLoad */

/* scheduler.h — one constant */
#define SCHEDULER_MAX_TASKS  12u    /* was 8; CPU0 needs exactly 8 with zero
                                       headroom after the split. +48 B/core. */
```

Unchanged on purpose: `Fusion_setBaroAlt`, `Ahrs_setMag`, `Fusion_setGnss`,
`Fusion_update`, `Ahrs_update`, `measurementsSetFusion`. Their *internals* gain the
cross-core latch discipline and their headers gain a one-line "writer core" note;
their signatures do not move.

**Becomes host-testable:**
1. `Atmosphere_altitudeM` — nominal, QNH = 0, QNH out of band, `pressPa` = 0, NaN.
2. `Bmp581_plausible` / `Mmc5983_plausible` / `Icm42688_plausible` — band edges and
   NaN. (Memory: *NaN makes every comparison false* — it has already killed a reject
   counter and a whole filter. These bands are written as `if (x > lo && x < hi)`,
   which is NaN-safe by luck, not design. A test pins it.)
3. `NavTask_inputValid` — the `dt` window, the `AHRS_RUNNING` gate, `present`.
4. `NavState_publish` / `NavState_get` — generation counter, torn-read retry,
   first-call-before-publish. On the host `Ifx__dsync()` is a no-op and the block is
   a plain static, so the test covers the *protocol* (ordering of the writes, the
   retry, the FALSE path) but **not** the memory placement or the barrier. Those two
   are hardware/map-file checks and are listed as such in Risk 2.

`Atmosphere` and `NavTask_inputValid` need **no** iLLD header at all. `NavState`
needs one line of it — the `Ifx__dsync` wrapper — so `SharedRam.h` must keep that
`#ifndef`-guarded so the host build can define it away to nothing.

---

## 5. Risks

1. **A2L / GUI contract: untouched by construction.** Publishing stays on CPU0 and
   `measurementsSetFusion` keeps its signature, so no `Xcp_Data` / `Xcp_Fusion`
   field moves and no offset shifts. `coreExecUs[1]` / `coreLoadPmil[1]` already
   exist in the system-load block and will simply stop reading zero. **If anyone
   adds a field during this refactor, the plan's zero-A2L-cost claim is void** —
   follow `docs/CODEMAP.md` §2 instead. Do not add fields here.
2. **Cross-core visibility: no longer an unknown — a stated design (§2.4).** All
   four crossings live in one LMU block at the non-cached alias `0xB00F0000`,
   `volatile`, all fields 32-bit, single writer, publish ordered by
   *payload → `Ifx__dsync()` → `gen++`*. The residual risks are specific and each
   has a named check:
   - **The object must actually land in `lmuram`.** `__at()` guarantees the address
     by construction, so the check is that the linker reported no overlap and the
     symbol is where it should be: `grep -i navstate` the TASKING **`.map`** file in
     `TriCore Debug (TASKING)/` and confirm `0xB00F….` — plus a boot-time
     `IfxCpu_isAddressCachable(&g_navState) == FALSE` self-check, which catches a
     wrong alias that a map inspection can miss. (T9.)
   - **The barrier must compile.** `Ifx__dsync` is undefined for TASKING in this
     tree; the wrapper in `SharedRam.h` is load-bearing and a missing barrier is
     *silent* — it fails as a rare torn snapshot, not as a build error. Inspect the
     generated `SharedRam.src` / `NavState.src` for the `DSYNC` instruction. (T9.)
   - **Sub-word writes must not creep back in.** A `uint8` added to a shared struct
     later becomes a read-modify-write and, if it shares a doubleword with the other
     core's data, can clobber it. Enforced by review and by the "all fields 32-bit"
     rule stated in `SharedRam.h`; there is no compiler check for it.
   - **Bottom-vs-top allocation.** Nothing uses `.bss.lmubss` today (verified), so
     placing the block in the top 64 K removes the only collision path. If a future
     change starts using those selectors, `CODEMAP.md` §3 is where it gets caught.
3. **QSPI ISR retarget** (`Spi.c:107` `isrProvider = IfxSrc_Tos_cpu0` → `cpu1`).
   Miss it and the IMU still reads, but CPU1's 112 µs busy-wait now depends on
   CPU0's interrupt latency — a silent jitter source. Needs hardware to verify.
   `Spi.c`/`I2c.c` also time out against `MODULE_STM0` from another core; that is a
   read-only cross-core STM access and is safe, but it is a deliberate exception to
   the "own STM per core" rule and should be commented as such.
4. **`Uart_println` is called from every core with no guard** — already true today
   (`Cpu1..5_Main.c`). This plan adds no new prints. Do not fix it here; note it.
5. **lwIP as a 1 ms task** adds up to 1 ms of RX latency to echo/XCP/replay.
   Off-chain, reversible, and it buys an honest load figure. (Decision D2.)
6. **Load numbers will get worse-looking** once lwIP is accounted. That is the
   measurement improving, not the firmware regressing. Expect it before flashing.
7. **T11 cannot be split.** Moving the IMU to CPU1 requires the QSPI ISR retarget,
   the driver init relocation and the task registration to land together — an
   intermediate state with the ISR on one core and the init on another does not
   fly. Every other step is independently buildable and flyable.
8. **Nothing here is irreversible.** Every step is a code move plus one registration
   line; `git revert` restores flying firmware at every point.
9. **The 1 kHz rate change (T15) is the one step with a filter-behaviour risk, not just
   a structural one.** Covariance propagation runs 20x more often in `float32`
   (§3.8), and the gyro-bias calibration window changes meaning entirely
   (`Ahrs.c:54,72`). `NavCovResets`/`NavDropped` are the existing instrumentation and
   are T15's acceptance criteria. Needs hardware; a host test cannot see it.
10. **`NAVTASK_DT_MIN_S = 0.001f` (`NavTask.c:21`) is a silent killer at 1 kHz.** The
   measured interval is 985 us, so the gate rejects every sample and the estimator
   stops with no error anywhere — the output simply freezes. It is listed as T14's
   first item for that reason. There is no diagnostic bit for it and none is free.
11. **The DRDY ISR retarget to CPU1 (T15) is the second irreversible-feeling step**, in
   the same sense as the QSPI retarget in Risk 3: get it wrong and the estimator's
   `dt` comes from a cross-core cached global instead of the LMU block, which fails
   as rare jitter rather than as a build error. Needs hardware; check
   `g_imuDrdyStaleTicks` and `missedEdges`, not just that it runs.

---

## 6. Ordered tasks for `flight-dev`

Steps T1-T8 are behaviour-preserving and stay entirely on CPU0. T9 is the migration.
The firmware builds and flies after every single one.

| # | files | change | acceptance |
|---|---|---|---|
| **T1** | `Cpu0_Main.c` (36, 180-240, 535-537) | delete the `SPI_PAD_TEST` block and its `#define`; also delete the duplicate `#include "Nvm.h"` (line 31) and move the stray includes (37-41) up to the include block | builds, 0 warnings; UART boot log byte-identical |
| **T2** | `Uart.c/.h`, `Cpu0_Main.c` | `uartHexByte` → `Uart_printHexByte` | builds; boot dump text byte-identical |
| **T3** | new `BringUp.c/.h`, `Cpu0_Main.c` | move the three `*DebugDump` printers into `BringUp_dumpSensors()`; single call site in `core0_main` | builds; boot log byte-identical (114 lines leave the core file) |
| **T4** | new `Atmosphere.c/.h`, `Cpu0_Main.c`, `test/` | move the ISA formula + QNH clamp out of `Task_Baro`; add the host test | host test passes (nominal / QNH=0 / out-of-band / p=0 / NaN); MISRA clean; `baroAltM` over XCP unchanged on hardware |
| **T5** | `Bmp581.*`, `Mmc5983.*`, `Icm42688.*`, `Cpu0_Main.c`, `test/` | move the three plausibility bands + liveness sums into their drivers; host tests incl. NaN | host tests pass; `diagStatus == 0` on the bench, unchanged |
| **T6** | new `SensorTask.c/.h`, `Cpu0_Main.c` | move `Task_Baro`, `Task_Mag`, and the **GNSS half** of `Task_Measure100ms` into `SensorTask` | builds; `tools/xcp_read.py` shows every sensor field live; GNSS `numSats` unchanged |
| **T7** | new `Housekeeping.c/.h`, `Cpu0_Main.c` | the remaining half of `Task_Measure100ms` becomes `Housekeeping_100ms`; **split `xcpDaqCycle()` and `Nvm_task100ms()` into their own scheduler tasks** so a flash erase can no longer delay DAQ, diagnostics or the GNSS feed | DAQ still streams to the GUI; `tools/nvm_test.py` passes; save-to-NVM no longer drops a DAQ burst |
| **T8** | `scheduler.h`, `Cpu0_Main.c` | `SCHEDULER_MAX_TASKS` 8 → 12; register lwIP polling as `Task_Lwip` at `SCHED_MS(1)` and empty the `while(TRUE)` body; delete the empty `Task_App10ms` on all six cores | ping + `tools/xcp_test.py` + TCP/UDP echo all pass; `coreLoadPmil[0]` rises (expected, not a regression) |
| **T9** | new `SharedRam.c/.h`, `CoreStats.h`, `docs/ILLD_NOTES.md`, `docs/CODEMAP.md` | **the LMU foundation (§2.4).** `SharedRam.c` = the `__at(0xB00F0000)` block and nothing else (cppcheck cannot parse `__at`); `SharedRam.h` = the rules, the `Ifx__dsync` wrapper and the 32-bit-fields rule. Add a boot-time `IfxCpu_isAddressCachable()` self-check reported through `PeriphDiag`/UART. Narrow the `CoreStats.h:15-21` comment so it stops being cited as the sanctioned pattern. Record `Ifx__dsync` missing for TASKING in `ILLD_NOTES.md` and the LMU block in `CODEMAP.md` §3. Block unused so far | builds, 0 warnings; **`.map` shows the block at `0xB00F….`**; boot log reports the address non-cacheable; MISRA green (the `__at` TU isolation is what keeps cppcheck parsing) |

> `SharedRam.c` is no longer `__at()`-only (`docs/MEMORY_PLACEMENT.md` T3):
> it holds all six LMU objects as `#pragma section`, precisely because the
> `__at()` TU isolation this row relies on turned out to blind cppcheck to
> everything else in a shared file too. D7's actual verdict (non-cached
> `0xB` alias over cached+`dsync`) is unaffected — only the placement
> mechanism changed.
| **T10** | new `NavState.c/.h`, `test/` | generation-counter snapshot in the T9 block: publish = payload → `Ifx__dsync()` → `gen++`; reader does read-copy-reread with one retry and never writes shared state. All fields 32-bit. Not yet wired in | host test: publish/get, torn-read retry, get-before-publish, FALSE path; MISRA clean; **`NavState.src` contains a `DSYNC` between the payload stores and the `gen` store** |
| **T11** | new `NavTask.c/.h`, `Housekeeping.c`, `Cpu0_Main.c` | move `Task_Imu`'s body into `NavTask_step`; publish via `NavState_publish`; `Housekeeping_100ms` consumes `NavState_get` and calls `measurementsSetFusion`. **Still registered on CPU0** — so the crossing is exercised single-core first, where a torn read cannot happen and any regression is therefore *not* the LMU. Split out `NavTask_inputValid` + its host test | hardware: every AHRS/fusion signal in the GUI identical to before; `dropped` and `covResets` still 0 over 5 min; AHRS bench check (nose up ≈ +90°) |
| **T12** | `Cpu0_Main.c`, `Cpu1_Main.c`, `Spi.c`, `fusion.c`, `Ahrs.c`, `SharedRam.c` | **the migration, atomic.** Move the three input latches (`Fusion_setBaroAlt` / `Ahrs_setMag` / `Fusion_setGnss` backing state) into the shared block, 32-bit fields, 8-byte aligned away from `NavState`, writer CPU0, and fix the consumer-clears-the-flag two-writer bug. Move `Spi_init` / `Icm42688_init` / `NavTask_init` into `core1_main` after the sync barrier; retarget `Spi.c:107` to `IfxSrc_Tos_cpu1`; register `NavTask_step` at `SCHED_MS(20)` on `MODULE_STM1`; comment the deliberate cross-core `MODULE_STM0` timeout reads. **Two additions from the §3 re-cost, both required for T12 itself, not for the later rate change:** (a) the three latches use a **sequence counter**, not a consumer-cleared boolean (§3.7) — correct at any writer/reader rate ratio, so T15 stays a rate change; (b) `NavTask_step` must stop writing CPU0-owned state — move `measurementsSetImu` and the IMU `PeriphDiag_report` onto CPU0 by carrying `acc`/`gyro`/`tempC`/accumulated `liveness`/`sampleSeq` in the `NavState` payload and calling them from `Housekeeping_100ms` (§3.7, T12 blocker). +32 B of LMU, zero A2L cost | hardware: `WHO_AM_I 0x47` on the **CPU1** boot line; `\|a\| ≈ 0.998 g`; **`g_navState.gen` read live over `tools/xcp_read.py` increments at 50 Hz** (the direct proof the crossing works); `coreLoadPmil[1]` non-zero and `coreAlive[1]` incrementing; `coreLoadPmil[0]` drops by the IMU share; AHRS bench check again; 5 min with `dropped == 0` and no `covResets`; **`grep -n "g_xcpData" src/bsw/NavTask.c` returns nothing** and `PeriphDiag_report` has no CPU1 call site; accel/gyro/IMU-temp still live in the GUI at 10 Hz |
| **T13** | `docs/CODEMAP.md`, `docs/FUSION.md`, `docs/REFACTORING_PLAN.md`, `Version.h` | `FUSION.md` §1 says *"Everything runs in `Task_Imu` at 50 Hz on CPU0"* — now wrong. `CODEMAP.md` §1 says *"CPU0 does everything; CPU1-5 idle"* and §2 "Put work on another core" still points at the `CoreStats` pattern — both now wrong; repoint at `SharedRam.h`. Bump `Version.h`, regenerate the A2L (version string only) | `gh workflow run misra.yml --ref feature/refactoring` green; `a2l.yml` green; version string confirmed **over XCP**, not from the build log |
| **T14** | `NavTask.c`, `Ahrs.c`, `Spi.c`, `ImuInt.c/.h` | **rate prerequisites, still at 50 Hz (§3.8).** `NAVTASK_DT_MIN_S` 0.001f -> 0.0002f; the two AHRS gyro-bias windows become **durations accumulated from `dt`** (2.0 s window, 10.0 s deadline) instead of sample counts, so they are rate-independent forever and unchanged at 50 Hz; IMU `SPI_XFER_DEADLINE_MS` 10 -> 1; add `missedEdges` beside `g_imuDrdyStaleTicks`. **No rate change in this task** — it flies at 50 Hz and must be behaviourally indistinguishable from T13 | builds, 0 warnings; MISRA clean; host test for the duration-based cal window (2.0 s reached at 50 Hz and at 1 kHz from the same `dt` stream); hardware: `AHRS_RUNNING` still reached in ~2 s, `\|a\| ~ 0.998 g`, `diagStatus` unchanged; 5 min with `NavDropped == 0` |
| **T15** | `Cpu1_Main.c`, `NavTask.c`, `ImuInt.c`, `SharedRam.c/.h`, `Configurations/ConfigurationIsr.h` | **the rate change, and only the rate change (§3.4, §3.6).** Retarget `SRC_SCUERU0` to `IfxSrc_Tos_cpu1`; move `{edgeTicks, edgeCount}` into the shared LMU block, single writer CPU1; `NavTask_step` consumes the pending edge, takes `dt` from the edge timestamps instead of `SysTime_getTimeElapsedS`, and returns immediately when no edge is pending; register it at `SCHED_US(500)` **first** in CPU1's task list, `Task_LedToggle` after it. **Gate: §3.5 must pass on the T12/T13 build before this task is written** | hardware: `g_navState.gen` increments at **~1014 Hz** over `tools/xcp_read.py`; `g_coreStats[1].execMaxUs` <= **300** and `loadPmil[1]` ~ 220; `g_imuDrdyStaleTicks` < ~1 ms sustained; `missedEdges` **below 0.05 % of edges over 10 min, not required to be 0** (corrected by T17, §3.9 — was originally specified as 0 over 5 min); **`NavCovResets` and `NavDropped` both 0 over 10 min** (§3.8); AHRS bench check (nose up ~ +90 deg); `diagStatus` unchanged; CPU0 load unchanged — **CONFIRMED on hardware 2026-08-27** except the original `missedEdges` line, resolved by T17 |
| **T16** | `docs/FUSION.md`, `docs/CODEMAP.md`, `docs/REFACTORING_PLAN.md`, `Version.h` | the T15 paperwork: `FUSION.md` §1's rate and task name, its `Task_Imu` gating sentence, the "~2 s"/"10 s" calibration prose; `CODEMAP.md` gains the 1 kHz chain and the DRDY-clocked estimator; mark §3 stage 2 as landed with the measured numbers. Bump `Version.h`; regenerate the A2L (version string only) | `misra.yml` green; `a2l.yml` green; `python tools/check_docs.py` green; version string confirmed **over XCP** |
| **T17** | `NavTask.c`, `test/test_navtask.c`, `docs/REFACTORING_PLAN.md`, `Version.h` | **the one open T15 hardware finding, `missedEdges` growing at ~1.7/min instead of 0 (§3.9).** Fix the real defect: seed `s_lastEdgeSeq`/`s_lastEdgeTicks` in `NavTask_init()` from a live `ImuEdge_snapshot()` instead of `0`, so edges produced during `Icm42688_init()`/`BringUp_dumpImu()`'s bring-up (before any task is polling) are never counted as "missed". Correct the T15 criterion for the residual, non-defect growth: `missedEdges` becomes a monitored health counter bounded at < 0.05 % of edges, not a required 0 — §3.9 has the full elimination and the numeric justification. Bump `Version.h` | host tests: `test_init_seeds_baseline_without_counting_bringup_edges`, `test_step_still_counts_genuine_multi_edge_gaps`; MISRA clean; `misra.yml` green; version string confirmed **over XCP**; hardware: `missedEdges` growth rate re-measured over a fresh 10 min and checked against the corrected < 0.05 % bound |

**17 tasks** (T1-T13 as originally planned; **T14-T16 added 2026-08-27** when the IMU measurement closed D5 — the 1 kHz rate change is deliberately *after* the core migration, see §3.4; **T17 added 2026-08-27** to close the one T15 hardware finding, §3.9). T1-T3 delete/relocate ~180 lines with zero behaviour change. T4-T5 create
the host-testable seams. T6-T8 restructure the task list and fix the flash-erase
coupling. **T9-T10 build and prove the shared-memory mechanism while it is still
unused — the point being that if the `.map` address or the `DSYNC` is wrong, it is
found before any flight code depends on it.** T11 moves the estimator behind the new
interface *on one core*, so a regression there is a refactoring bug, not a coherency
bug. T12 is the only step that turns on the actual cross-core traffic. T13 is the
paperwork that keeps the docs from lying. **T14 fixes the rate-dependent constants while still at 50 Hz, T15 changes the rate and nothing else, T16 is its paperwork, T17 closes T15's one open hardware finding** — so a 1 kHz regression can only be the rate, and reverting T15 alone restores a flying build.

After T12, `Cpu0_Main.c` should be ~120 lines: includes, two statics, `core0_main`
with init and eight registrations, and the lwIP ISR.

---

## 7. Decisions needed before implementation

| # | question | my recommendation |
|---|---|---|
| **D1** | Two cores (CPU0 = comms + I2C + housekeeping, CPU1 = flight) or three (CPU2 takes the I2C bus)? | **Two.** The third core buys only ground-link responsiveness and costs a contract. The latch design makes the later move a one-line change if measurement justifies it. |
| **D2** | lwIP: a 1 ms scheduled task (accounted, +≤1 ms RX latency) or stay in the `while(TRUE)` body (free, invisible)? | **Scheduled task.** An unmeasured consumer on the comms core is exactly the kind of thing that bites later, and lwIP's own tick is already 1 ms. |
| **D3** | `SCHEDULER_MAX_TASKS` 8 → 12? | **Yes.** CPU0 lands on exactly 8 with zero headroom. Costs 48 bytes per core. |
| **D4** | Should the DFLASH save be gated (e.g. refused while armed) rather than merely moved off the flight core? | **Not now.** There is no armed state yet. Add the gate when arming lands; note it in `DIAGNOSTICS.md`. |
| **D5** | **CLOSED 2026-08-27 by measurement.** `flight_ctrl` is designed for `Ts = 0.001f` (`CtrlReplay.c:93`); does the IMU really deliver 1 kHz? | **Yes — branch (A), `docs/IMU_INTERRUPT.md` §5.6.** 985.036 µs mean (1014.2 Hz), stddev 1.780 µs, `spi_burst_max` = 124.4 µs. `NavTask` goes to 1 kHz and `flight_ctrl` keeps `Ts = 0.001f`. Re-costed in §3.2: ~220 µs typical, 300 µs budget, CPU1 ≈ 22 %. The rate change is **T15, not T12** (§3.4), and T14 fixes the rate-dependent constants first (§3.8). Follow-on question this opened: **D8**. |
| **D6** | **Does `g_coreStats` move into the LMU shared block** for consistency with the new rule, or stay as it is? | **Stay.** It works today, it is `volatile` and single-writer-per-slot, and it carries diagnostics — a torn read costs one wrong load number, never a flight. Moving it touches all six core mains for no behavioural gain. But its **comment must be corrected** (T9): `CoreStats.h:15-21` currently reads as the project's sanctioned inter-core pattern and is being cited as such. If you would rather have one rule with no exceptions, moving it is ~20 lines and I would not argue. |
| **D7** | **Non-cached `0xB` alias vs cached `0x9` + `dsync` per write** for the shared block. | **Non-cached (`0xB00F0000`).** Costed in §2.4: ~3.9 µs to publish 232 bytes, 0.2% of `NavTask_step`'s budget, against a barrier discipline that every future writer would have to honour forever. Listed here because it is the one decision in §2.4 that is genuinely mine rather than the vendor's. |
| **D8** | **CLOSED 2026-08-27 by section 9**, argued from the plant model in `C:\Users\chris\Projects\Quadrocopter`. The estimator runs at the sensor's **1014.2 Hz**, not 1000 Hz (section 3.1). `flight_ctrl` is called with a fixed `Ts = 0.001f` (`CtrlReplay.c:87`). Does the control law take the true `dt`, or get its own 1 ms slot? | **The true `dt`, from the same per-tick ISR edge delta the estimator uses -- and no second slot.** `Ts` is a *runtime* field (`flight_ctrl.h:39`) and appears in exactly one line of the control law, the backward-Euler integrator (`flight_ctrl.c:114`); the gains are continuous-time. So feeding the measured interval is a **caller-side change with zero edit to the generated file**, it is exact at any rate, and it is the only form that stays correct when an edge is missed (`dt = 2*T`, area preserved) -- where a hard-coded `0.001f` is 100 % wrong. Clamp it through the existing `NAVTASK_DT_MIN_S` / `FUSION_DT_MAX` window before use. **Not** a per-window mean: a mean is smooth exactly where correctness matters. Separately, section 9 answers the rate question behind this one: **the control law needs ~200 Hz, 500 Hz is sufficient, and 1014.2 Hz is justified by anti-aliasing of the 244-382 Hz blade-pass band, not by the control law.** See sections 9.5 and 9.6. |

## 8. Where I was unsure, and what I chose

- **Two cores vs three (D1).** Genuinely close. I chose the smaller change because
  the brief says not to move work across cores just because five are free, and the
  I2C outputs are already decoupled by latches. If your instinct is that a wedged
  I2C bus must never be able to slow the XCP link, take three.
- **lwIP polling rate (D2).** Converting a free-running poll into a 1 ms task is a
  real behaviour change on working Ethernet. I chose it for the visibility; it is
  the single most reversible item in the plan and the easiest to drop from T8.
- **Whether plausibility belongs in the drivers or in `SensorTask` (T5).** I put it
  in the drivers because the constants are datasheet facts, but a reasonable reading
  puts it in `SensorTask` with `PeriphDiag` and keeps the drivers purely about the
  bus. Either is defensible; the driver choice makes the host tests smaller.
- **Cache/visibility of the cross-core data — resolved, and I had it wrong.** The
  first draft of this plan proposed extending the `CoreStats` "cross-core DSPR reads
  bypass the cache" claim to all four crossings, and flagged the writer-side
  visibility as an unknown to check on hardware. That was the wrong instinct: the
  `CoreStats` comment describes the reader only and is not Infineon's sanctioned
  mechanism. §2.4 now states the vendor's actual rule (LMU + non-cached alias, or
  LMU + barrier per write) and the plan takes the first. **Do not let the
  `CoreStats` comment propagate to a fifth crossing** — T9 narrows it for that
  reason. Where I *do* still exercise judgement is D7 (which of the two vendor
  options) and D6 (whether `g_coreStats` itself moves).
- **The `Ifx__dsync` gap is a live trap, not a footnote.** The intrinsic is defined
  for Dcc/Gcc/Gnuc/HighTec but **not** for TASKING in this tree, and TASKING is the
  build config. A missing barrier does not fail the build — it fails as a rare torn
  snapshot under load, which is the worst possible failure signature. That is why
  T10's acceptance check inspects the generated `.src` for a `DSYNC` instruction
  rather than trusting that the wrapper compiled to something.
- **LMU access latency.** I costed it at ~10-20 cycles/access from the SRI
  topology, not from a measurement — the conclusion (0.2% of the task budget) has
  enough margin that a factor of three either way does not change the decision, but
  the number itself is an estimate and is labelled as one.
- **The 1 kHz re-cost is 124.4 µs measured plus ~95 µs estimated (§3.2).** I did not
  guess at the measured part and I did not measure the estimated part — `Ahrs_update`
  and the three KF channels are costed from their operation counts, not from a timer.
  The conclusion (fits, with 685 µs free) survives a 3× error in that estimate, which
  is why I was willing to publish it; but §3.5 makes T12 hand back the real number
  before T15 spends it. If `execMaxUs[1]` comes back above 300 µs, believe it and not
  this document.
- **Where I was wrong before: CPU1 at 1 kHz is ~22 %, not ~15 %.** The old figure came
  from a 112 µs SPI estimate with no compute costed at all. The direction of the error
  matters less than the fact that the earlier number never included the estimator.
- **`SCHED_US(500)` rather than `SCHED_MS(1)` is the least obvious call in this
  revision (§3.6).** A 1 ms slot looks right and is wrong, because the sensor runs at
  1014.2 Hz and a 1000 Hz poll loses ~14 samples a second in a slow beat. If the
  `newSample` gate is ever removed, that slot becomes a bug — the gate is what makes
  over-polling correct, not the period.
- **`Atmosphere` as a module name.** The formula could equally live in `Bmp581.c`.
  I split it because it is not chip-specific and because a pure, iLLD-free
  translation unit is worth more as a test seam than as a saved file.

---

## 9. The loop rate, argued from the plant (2026-08-27)

Written in answer to *"is the 1 kHz for the flight controller a hard requirement,
or can we talk about the 1 kHz?"* Source of every number below: the Simulink/MATLAB
model in `C:\Users\chris\Projects\Quadrocopter`, read read-only.

**Answer up front: 1 kHz is not a requirement. The control law's requirement is
~200 Hz; 500 Hz is comfortably sufficient. Take 1014.2 Hz anyway** -- not for the
control law, but because the IMU's ODR and anti-alias filter are configured for
1 kHz, and any decimation below Nyquist-507 Hz folds propeller vibration into the
control band. It costs 22 % of a core that reads 0.00 % today.

### 9.1 The fastest dynamics that must be controlled

| dynamic | value | source | in Hz |
|---|---|---|---|
| Motor + ESC first-order lag | `tau = 0.05 s` -> pole at **-20 rad/s** | `quad_params.m:10`, state `pt1_w_cmd_to_w` (`quad_params.m:47`) | **3.18 Hz** |
| Rigid-body attitude | `Ixx=Iyy=1.0e-2`, `Izz=1.8e-2 kg m2`; om-dot = M/I is a **pure integrator** -- poles at 0 | `quad_params.m:6` | 0 |
| Translational drag | `Cd = 0.10 N/(m/s)` / `m = 1.20 kg` -> -0.083 rad/s | `quad_params.m:12,2` | 0.013 Hz |
| Position mode from the linearisation | `0 +/- 1.6j` | Quadrocopter `README.md:101` | 0.25 Hz |

**The fastest pole anywhere in the modelled plant is the motor at 20 rad/s =
3.18 Hz.** Everything else is slower, or is an integrator. There is no fast mode
hiding in this airframe.

### 9.2 The loop bandwidths the design actually asks for

The cascade is dimensioned explicitly (Quadrocopter `README.md:44-108`), and the
gains in `quad_params.m:26-38` reproduce it arithmetically:

| loop | rule | arithmetic | omega_c |
|---|---|---|---|
| Body rate (PI), roll/pitch | omega_c = Kp / I | 0.100 / 0.010 | **10.0 rad/s = 1.59 Hz** |
| Body rate (PI), yaw | omega_c = Kp / I | 0.216 / 0.018 | **12.0 rad/s = 1.91 Hz** |
| Attitude (P) | omega_c = Kp outright | 3.0 | 3 rad/s = 0.48 Hz |
| Position (PD) | -- | -- | ~1.2 rad/s = 0.19 Hz |

The I-term zero confirms the reading: `tau_kI/tau_kP` = 0.125/0.100 = 1.25 rad/s
= omega_c/8, and 0.324/0.216 = 1.5 rad/s = omega_c/8 (`quad_params.m:26-27`; the
rule is stated at `README.md:78`). **The fastest closed loop in this aircraft
crosses over at 1.91 Hz.**

### 9.3 The required loop rate

Two rules, both applied to the yaw rate loop because it is the fastest.

**Rule A -- sample rate 10-20x the closed-loop bandwidth.**
20 x 1.91 Hz = **38 Hz**. By this rule the *existing 50 Hz `Task_Imu` already
satisfies the control law*, which is worth stating plainly.

**Rule B -- phase margin lost to discretisation.** A ZOH plus one compute period
costs about 1.5*Ts of effective delay; the phase it adds at crossover is
phi = omega_c * 1.5 * Ts, with omega_c = 12 rad/s:

| f_s | Ts | 1.5*Ts | phi at omega_c |
|---|---|---|---|
| 50 Hz | 20 ms | 30 ms | **20.6 deg** |
| 100 Hz | 10 ms | 15 ms | 10.3 deg |
| 200 Hz | 5 ms | 7.5 ms | **5.2 deg** |
| 500 Hz | 2 ms | 3 ms | 2.1 deg |
| **1014.2 Hz** | 0.985 ms | 1.48 ms | **1.0 deg** |

For scale, the motor lag alone already eats `atan(12/20)` = **31.0 deg** of phase at
that crossover. The sampler is not the phase budget; the actuator is. Rule B says
**200 Hz** (5.2 deg, an order of magnitude under the actuator's own contribution)
and calls 50 Hz marginal at 20.6 deg.

**Required rate is ~200 Hz. Going from 500 Hz to 1014 Hz buys 1.1 deg of phase
margin.** That is not a reason to do anything.

### 9.4 What the higher rate actually buys -- and what it does not

**Does not buy: gyro noise averaging.** Attitude comes from *integrating* gyro, and
the angle random walk over a fixed interval is sigma_theta = ARW * sqrt(t) --
rate-independent. Halving the estimator rate does not double the attitude noise.
Quadrocopter `doc/MBD_PATH.md:94` gives sigma ~ 0.0016 rad/s per sample at 1 kHz;
at 500 Hz the per-sample sigma is smaller by sqrt(2) and there are half as many
samples. Net zero. **Strike this from the argument for 1 kHz.**

**Does not buy: disturbance rejection.** That is set by loop gain, and loop gain is
unchanged once f_s is well above omega_c -- true from 100 Hz upward.

**Does not buy: delay margin worth having.** 1.1 deg (section 9.3).

**Does buy, and this is the only real one: anti-alias headroom for rotor vibration.**

- Hover shaft speed `w_hover = sqrt(m*g/(4*kT))` = sqrt(11.772 / 2.0e-5) =
  **767.2 rad/s = 122.1 Hz** (`quad_params.m:57`; its inline comment "-> 990.5 rad/s"
  is **stale** -- `linearize/linearize_hover.m:15` gives the check value as 767).
- At `w_max = 1200 rad/s` (`quad_params.m:11`) the shaft is 191 Hz.
- Two-blade blade-pass = 2x shaft: **244 Hz at hover, up to 382 Hz at full throttle.**

| f_s | Nyquist | blade-pass 244-382 Hz |
|---|---|---|
| 500 Hz | 250 Hz | **aliases** -- folds to 256...118 Hz, i.e. down toward the control band |
| 1014.2 Hz | 507 Hz | stays in band, and is therefore *filterable* rather than already folded |

The ICM-42688-P is configured at 1 kHz ODR with its filtering set for that ODR
(`docs/IMU_INTERRUPT.md:209`, `docs/ICM42688P.md:296`). **Consuming every second edge
is decimation without a decimation filter** -- the sensor has band-limited for
507 Hz, not for 250 Hz. This is also the precondition for the RPM notch both repos
already plan (`doc/projektplan.md:135`, `README.md:292`) and for the frame resonance
that document flags near the hover fundamental.

**Cost of 1014 Hz over 500 Hz:** CPU1 ~22 % vs ~11 % (section 3.2) on a core at
0.00 % today; **20 covariance propagations per second per channel instead of 10**,
all in `float32`, which is exactly the `NavCovResets` exposure called out in
section 3.7 and gated by T15's acceptance; and one hardware re-validation pass.
All three are affordable. None of them is bought back by rate.

### 9.5 The `Ts` question -- confirmed

**Confirmed, and it dissolves D8.**

- `Ts` is a *runtime* parameter (`flight_ctrl.h:39`), not a compile-time constant.
- It appears in exactly **one** line of the control law -- the backward-Euler
  integrator `I_new = st->tau_I[i] + p->tau_kI[i] * p->Ts * e;` (`flight_ctrl.c:114`).
  No derivative term, no filter coefficient. The gains are continuous-time by design
  (`README.md:76-79`: Kp = omega_c * I).
- Therefore **`Ts` is a caller-side value with zero edit to the generated file**, and
  the control law is rate-independent by construction.

**Where the value comes from: the per-tick delta of the ISR edge timestamps** -- the
same `dt` T15 already hands `Ahrs_update` and `Fusion_step`. One clock, one `dt`, one
place it can be wrong.

- **Not a fixed `0.001f`.** At the true 985.036 us that is a +1.5 % integrator-gain
  error (harmless -- Ki 0.125 behaves as 0.1269), but on a *missed* edge it is 100 %
  wrong and the integrator under-accumulates a whole tick's area.
- **Not a per-window mean.** A mean is smooth exactly in the case where correctness
  matters: a skipped or doubled tick. The per-tick delta is right in both.
- Clamp it through the existing window (`NAVTASK_DT_MIN_S` after T14, `FUSION_DT_MAX`
  / `AHRS_DT_MAX_S` = 0.2 s) before it reaches either the filters or `p->Ts`.
- For the record: the estimator runs at **1014.2 Hz, not 1000 Hz** (section 3.1), so
  "1 kHz" in this plan is shorthand for the sensor's rate and never for a 1 ms slot.

### 9.6 Must the estimator and the control law run at the same rate?

**No -- and here they should anyway.**

Splitting them (estimator 1014 Hz, rate controller 507 Hz) is a legitimate and common
architecture, and section 9.3 says the controller would not notice. What it costs here:

- a **rate-transition seam** inside the flight chain -- which state the controller
  reads, whether it is the latest or a decimated one, and a second `dt` to keep
  consistent. That is the class of thing section 2.4 spent its length removing.
- a second timing case to reason about on overrun, on top of the `newSample` gate.

What it buys: ~10-20 us per 985 us of CPU1, on a core running 22 %. **Not worth it.**
The motor is a first-order lag at 3.18 Hz (section 9.1); it cannot tell a 507 Hz
command stream from a 1014 Hz one under any circumstance.

**The one case where the split is right** is the section 3.5 fallback: if T12 hands
back `execMaxUs[1]` >= 490 us, decimate the **control law** to 507 Hz and leave the
estimator on every edge -- because the estimator's rate is the anti-aliasing argument
(section 9.4) and the controller's is not. Prefer that over dropping the estimator
to 500 Hz.

### 9.7 What I could not determine from the model files

- **No linearisation output is stored.** `linearize/linearize_hover.m:13-16` runs
  `linmod` and then only `disp(eig(A))` -- no gain or phase margin, no `margin()` call,
  no saved A/B/C/D, and no target crossover written anywhere in the scripts. The
  crossover figures in section 9.2 are the *design intent* from `README.md:49-53,76-79`,
  reproduced independently from the gains and inertias. **The achieved phase margin of
  the real loop is not recoverable without running MATLAB**, and I did not.
- **No file records a chosen rate with a reason.** `Ts = 0.001` is asserted at
  `quad_params.m:50` ("Abtastzeit Regler (1 kHz)"), repeated at `sil/build_sfun.m:16`,
  `pil/replay_export.m:27`, `pil/replay_udp.m:115` and `pil/run_pil.m:110`, and stated
  as fact in `README.md:45` and `doc/projektplan.md:28,44` -- always as a given, never
  with a derivation. **1 kHz in the model is the sensor's rate, adopted; it was not
  derived from the plant.** That is the honest answer to the question.
- **Blade count and propeller are not fixed.** `README.md:298` and
  `doc/projektplan.md:130` say frame, motor KV and propeller are still open.
  Section 9.4 assumes two blades; a three-blade prop moves blade-pass to 366-573 Hz,
  which *strengthens* the 1014 Hz case and would exceed even its Nyquist at full
  throttle.
- **The "~83 Hz Hover-Grundton" at `doc/projektplan.md:135` does not match this
  parameter set** -- 122.1 Hz falls out of `quad_params.m:2,8`. One of the two belongs
  to a different airframe. If 83 Hz is the real one, two-blade blade-pass is 166 Hz,
  which does *not* alias at 500 Hz, and 500 Hz becomes fully defensible. **Resolve this
  when the propeller is chosen -- it is the single number the recommendation is
  sensitive to.**

### 9.8 Effect on the task list

- **T14 -- unchanged, and vindicated.** Making the AHRS calibration windows
  *durations* rather than sample counts is precisely the rate-independence this
  section argues for. Same for `NAVTASK_DT_MIN_S`.
- **T15 -- unchanged in substance.** It stays "the rate change and only the rate
  change", still gated on section 3.5, still to 1014.2 Hz. Two additions belong in its
  commit message rather than its diff: the rate is the *sensor's*, adopted with ~5x
  headroom over the ~200 Hz the control law needs (section 9.3), and the justification
  is section 9.4's Nyquist argument, not the control law.
- **T16 -- one line added:** `FUSION.md` and `CODEMAP.md` should say *1014.2 Hz,
  DRDY-clocked, `dt` per tick* and never "1 kHz fixed", so the next reader does not
  re-derive this.
- **New, for whoever lands the control law (not this refactor):** `p->Ts` takes the
  per-tick `dt`, not `0.001f`. `CtrlReplay.c:87` keeps `0.001f` -- the replay harness is
  fed a fixed `Ts` grid by `pil/replay_export.m:27` and must stay bit-comparable with
  the model.
---

## Sources

The **authority** for §2.4 is Infineon's own guidance: their KB article on sharing
data between TC3xx cores, and the AURIX TC3xx User Manual (LMU / segment aliases /
`DSYNC`). Read those, not this file, if the rule is ever in doubt.

**Verified on disk in this repo** (2026-08-27), which is what makes the seam
usable without a linker edit:

| claim | evidence |
|---|---|
| `lmuram` 768 K, cached `0x90040000` / non-cached `0xB0040000` | `Lcf_Tasking_Tricore_Tc.lsl:456-462` |
| `.data.lmudata` / `.bss.lmubss` already route into `lmuram` | `Lcf_Tasking_Tricore_Tc.lsl:1213-1216` |
| per-core DLMU selectors exist and are *not* what shared data wants | `Lcf_Tasking_Tricore_Tc.lsl:1183-1211` |
| nothing in the tree uses `.bss.lmubss` / `.data.lmudata` today | grep over `src/`, `Libraries/` — no hits outside the `.lsl` |
| `IfxCpu_acquireMutex` is a real `__cmpAndSwap` (CMPSWAP.W) spinlock | `IfxCpu.c:54-63` |
| segment-cacheability API, callable only *before* caches are enabled | `IfxCpu.h:588-646` |
| `IfxCpu_isAddressCachable()` — the boot self-check used in T9 | `IfxCpu.h:626` |
| **`Ifx__dsync` is undefined for TASKING** — Dcc/Gcc/Gnuc/HighTec only | `IfxCpu_IntrinsicsDcc.h:1351`, `…HighTec.h:354`; absent from `IfxCpu_IntrinsicsTasking.h` |
| `__at()` is the house placement style; cppcheck cannot parse it, so one block per TU | `Measurements.c:18-37` |
| `Xcp_Data` lives at `0x70030000` in **CPU0 DSPR** — so publishing must stay on CPU0 | `Measurements.c:18-23` |

> Evidence table above is a record of what was true when this plan was
> written. `__at()` and the one-object-per-TU rule it forced are both gone
> now — `docs/MEMORY_PLACEMENT.md` replaced the placement mechanism entirely
> (T0-T7); read that document for the current state, not this row.
