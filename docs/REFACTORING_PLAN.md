# Refactoring plan — Cpu0_Main.c and the core partition

Status: **design draft for review**. No code written. Branch `feature/refactoring`.
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
| **CPU1** | **flight core.** Hard deadline. Nothing else, ever. | NavTask 20 ms (+ the control law and motor output when they land) |
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
budget** — the cache buys nothing measurable at 50 Hz. The three sensor latches are
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

| task | rate | typ | worst | core | overrun behaviour |
|---|---|---|---|---|---|
| `NavTask_step` | **50 Hz / 20 ms** | ~0.30 ms | **2.0 ms** budget (SPI hard cap 10 ms, `Spi.c:36`) | **CPU1** | nothing else on the core to steal from; `dt` is measured, so a late tick is fused with its true `dt`; `dt > 0.2 s` freezes the filters instead of integrating garbage |
| `SensorTask_baro` | 50 Hz | 0.81 ms | **10 ms** (`I2c.c:37`) | CPU0 | delays other CPU0 tasks only; no new baro latch → the DOWN channel coasts on accel + `accelBias` |
| `SensorTask_mag` | 50 Hz | 0.90 ms | **10 ms** | CPU0 | AHRS degrades to accel+gyro; **yaw becomes unbounded** (`Ahrs.h:15`) — this is the one off-chain failure with a flight consequence, and it is why `PeriphDiag` must stay loud |
| `SensorTask_gnss` | 10 Hz | ~20 µs | 50 µs | CPU0 | ring-buffer read only; duplicate-`iTOW` guard makes a late poll harmless (`fusion.h:136`) |
| `Housekeeping_100ms` | 10 Hz | ~0.5 ms | 2 ms | CPU0 | XCP block stale by one cycle |
| `Task_XcpDaq` | 10 Hz | ~0.2 ms | 1 ms | CPU0 | one DAQ frame skipped; GUI plot gap |
| `Task_Nvm` | 10 Hz | ~1 µs idle | **~100 ms on a save** (`Nvm.c:205-230`) | CPU0 | blocks all of CPU0 including lwIP; **after the move, nothing on the flight chain is affected — today it stalls the estimator** |
| `Task_Lwip` | 1 kHz | ~30 µs | 1 ms | CPU0 | echo/XCP RX latency only |
| `Task_Led` | 2 Hz | ~2 µs | — | CPU0 | cosmetic |

CPU0 steady state ≈ 10-12% (the I2C wire time, plus lwIP which becomes *visible*
for the first time). CPU0 pathological burst: 20 ms of I2C + a 100 ms flash erase —
all off-chain. CPU1 ≈ 1.5%, with a single 20 ms slot to itself.

**Flagged, as asked:** today `Task_Imu` (the flight chain) shares its core with the
two blocking I2C reads that are ~91% of CPU0's load, *and* with the DFLASH erase,
*and* with unaccounted lwIP polling. After this plan it shares its core with
nothing.

**Open timing mismatch, needs a decision:** `flight_ctrl` was designed for
`Ts = 0.001f` — 1 kHz (`CtrlReplay.c:93`). The IMU tick is 50 Hz. A 1 kHz rate loop
cannot be fed by a 50 Hz gyro. On CPU1 a 1 kHz IMU tick costs 112 µs of SPI per
millisecond ≈ 11% of the core, which is affordable — but it is a separate decision
about sensor rate and filter tuning, not part of this refactor. (Decision D5.)

---

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
| **T10** | new `NavState.c/.h`, `test/` | generation-counter snapshot in the T9 block: publish = payload → `Ifx__dsync()` → `gen++`; reader does read-copy-reread with one retry and never writes shared state. All fields 32-bit. Not yet wired in | host test: publish/get, torn-read retry, get-before-publish, FALSE path; MISRA clean; **`NavState.src` contains a `DSYNC` between the payload stores and the `gen` store** |
| **T11** | new `NavTask.c/.h`, `Housekeeping.c`, `Cpu0_Main.c` | move `Task_Imu`'s body into `NavTask_step`; publish via `NavState_publish`; `Housekeeping_100ms` consumes `NavState_get` and calls `measurementsSetFusion`. **Still registered on CPU0** — so the crossing is exercised single-core first, where a torn read cannot happen and any regression is therefore *not* the LMU. Split out `NavTask_inputValid` + its host test | hardware: every AHRS/fusion signal in the GUI identical to before; `dropped` and `covResets` still 0 over 5 min; AHRS bench check (nose up ≈ +90°) |
| **T12** | `Cpu0_Main.c`, `Cpu1_Main.c`, `Spi.c`, `fusion.c`, `Ahrs.c`, `SharedRam.c` | **the migration, atomic.** Move the three input latches (`Fusion_setBaroAlt` / `Ahrs_setMag` / `Fusion_setGnss` backing state) into the shared block, 32-bit fields, 8-byte aligned away from `NavState`, writer CPU0, and fix the consumer-clears-the-flag two-writer bug. Move `Spi_init` / `Icm42688_init` / `NavTask_init` into `core1_main` after the sync barrier; retarget `Spi.c:107` to `IfxSrc_Tos_cpu1`; register `NavTask_step` at `SCHED_MS(20)` on `MODULE_STM1`; comment the deliberate cross-core `MODULE_STM0` timeout reads | hardware: `WHO_AM_I 0x47` on the **CPU1** boot line; `\|a\| ≈ 0.998 g`; **`g_navState.gen` read live over `tools/xcp_read.py` increments at 50 Hz** (the direct proof the crossing works); `coreLoadPmil[1]` non-zero and `coreAlive[1]` incrementing; `coreLoadPmil[0]` drops by the IMU share; AHRS bench check again; 5 min with `dropped == 0` and no `covResets` |
| **T13** | `docs/CODEMAP.md`, `docs/FUSION.md`, `docs/REFACTORING_PLAN.md`, `Version.h` | `FUSION.md` §1 says *"Everything runs in `Task_Imu` at 50 Hz on CPU0"* — now wrong. `CODEMAP.md` §1 says *"CPU0 does everything; CPU1-5 idle"* and §2 "Put work on another core" still points at the `CoreStats` pattern — both now wrong; repoint at `SharedRam.h`. Bump `Version.h`, regenerate the A2L (version string only) | `gh workflow run misra.yml --ref feature/refactoring` green; `a2l.yml` green; version string confirmed **over XCP**, not from the build log |

**13 tasks** (was 12; the LMU foundation is new as T9 and everything after it shifted
by one). T1-T3 delete/relocate ~180 lines with zero behaviour change. T4-T5 create
the host-testable seams. T6-T8 restructure the task list and fix the flash-erase
coupling. **T9-T10 build and prove the shared-memory mechanism while it is still
unused — the point being that if the `.map` address or the `DSYNC` is wrong, it is
found before any flight code depends on it.** T11 moves the estimator behind the new
interface *on one core*, so a regression there is a refactoring bug, not a coherency
bug. T12 is the only step that turns on the actual cross-core traffic. T13 is the
paperwork that keeps the docs from lying.

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
| **D5** | **The rate mismatch.** `flight_ctrl` is designed for `Ts = 0.001f` (1 kHz, `CtrlReplay.c:93`) and the IMU runs at 50 Hz. Which moves? | Needs your call, and it is **outside this refactor**. A 1 kHz IMU tick on CPU1 costs ~11% of that core and is affordable; retuning the rate loop for 50 Hz is the other option. The refactor is designed so either choice is a rate change on one registration line. |
| **D6** | **Does `g_coreStats` move into the LMU shared block** for consistency with the new rule, or stay as it is? | **Stay.** It works today, it is `volatile` and single-writer-per-slot, and it carries diagnostics — a torn read costs one wrong load number, never a flight. Moving it touches all six core mains for no behavioural gain. But its **comment must be corrected** (T9): `CoreStats.h:15-21` currently reads as the project's sanctioned inter-core pattern and is being cited as such. If you would rather have one rule with no exceptions, moving it is ~20 lines and I would not argue. |
| **D7** | **Non-cached `0xB` alias vs cached `0x9` + `dsync` per write** for the shared block. | **Non-cached (`0xB00F0000`).** Costed in §2.4: ~3.9 µs to publish 232 bytes, 0.2% of `NavTask_step`'s budget, against a barrier discipline that every future writer would have to honour forever. Listed here because it is the one decision in §2.4 that is genuinely mine rather than the vendor's. |

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
- **`Atmosphere` as a module name.** The formula could equally live in `Bmp581.c`.
  I split it because it is not chip-specific and because a pure, iLLD-free
  translation unit is worth more as a test seam than as a saved file.

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
