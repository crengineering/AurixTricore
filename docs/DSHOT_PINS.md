# DSHOT_PINS.md — Pin selection for bidirectional DShot300 (4× BLDC / AM32)

Target: TC399 on **TriBoard TC3X9 (LFBGA‑516 package)**, iLLD `iLLD_1_20_0`.
Use case: four bidirectional DShot300 lines, one signal wire per motor.
TX = GTM **ATOM** channel in PWM mode (DMA‑fed compare values), then pin flipped
to input, RX = GTM **TIM** channel in input‑capture mode (~375 kbit/s GCR reply),
cycle time 1 ms.

> Scope note: this document answers *which pins*, *what conflicts*, and *how fast
> the direction switch is*. It does **not** implement the driver. Where the
> in‑repo sources do not give an unambiguous answer, that is stated explicitly
> rather than guessed (see §9 *Open questions*).

> **Pin allocation SSoT:** the consolidated, authoritative pin map for the whole
> project is **`docs/PINNING.md`**. This document is the *rationale* for the DShot
> pins; when you (re)allocate a pin, update `PINNING.md` too.

---

## 0. Headline result

**A single‑pin bidirectional scheme is possible.** On the TC39x every GTM‑capable
port pin that can be an **ATOM output** is *also* reachable as a **TIM input** —
they share the same GTM I/O line set. A script over the iLLD pin map confirms the
intersection is complete:

```
ATOM‑TOUT‑capable pins : 261
TIM‑TIN‑capable pins   : 261
pins with BOTH         : 261   (100 %)
```
*(derived from `Libraries/iLLD/TC3xx/Tricore/_PinMap/TC39xB/IfxGtm_PinMap_TC39xB_516.h`)*

So the binding constraint is **not** the silicon — it is the TriBoard: which of
those pins are (a) routed to an 80‑pin header, (b) free of an assembled 0‑Ω
bridge / transceiver, and (c) not already claimed by this project. After applying
all three filters, **P22.0 / P22.1 / P22.2 / P22.3** are the recommended four —
contiguous, all on the **PERIPHERALS** header (X702/X802), all free, and they fall
out onto **one ATOM instance and one TIM instance with matching channel indices**.

The two‑pin fallback (§4) is therefore **not required**; it is documented only for
completeness, together with the iLLD tri‑state call that the single‑pin scheme
needs anyway.

⚠ **Hardware caveat — the ports are 5 V; level adaptation is mandatory (see §8).**
On this board the general ports (incl. Port 22) sit in the **VEXT / V_UC** supply
domain, confirmed **5 V** (R527 assembled, TLF35584QVVS1, measured P00.1 = 5.0 V).
A 3.3 V‑logic ESC (GOKU G45M) needs level adaptation. **§8** shows the recommended
**open‑drain + 1.5 kΩ pull‑up to 3.3 V** solution (register‑confirmed) and a
74LVC1T45 translator fallback.

---

## 1. Method & sources

| # | Source | What was read |
|---|---|---|
| S1 | `docs/infineon-tc39x-datasheet-en.pdf` — **Data Sheet V1.2, 2021‑03** | Table 2‑12 *Port 22 Functions*, pp. 100‑101 (ALT‑function O0…O7 and TIM mux inputs); LFBGA‑516 ball map (Port 22 balls, VEXT domain); supply‑pin naming |
| S2 | `docs/infineon-triboardmanual-tc3x9-um-en.pdf` — **TriBoard Manual V2.1, 2017‑11** | §6.2 Fig. 6‑1 *Connector pinout Part I* (BUS‑EXPANSION X701/X801, **PERIPHERALS X702/X802**), p. 6‑2; §6.2 Fig. 6‑2 (ADC X703/X803, GTM/PORTS X704/X804), p. 6‑3; §4.2 **Table 4‑3** *Resistors for peripherals*, pp. 4‑4/4‑5; **Table 6‑1** *On Board only used Signals*, p. 6‑1; Table 5‑5 supply names, p. 5‑? |
| S3 | `Libraries/iLLD/.../_PinMap/TC39xB/IfxGtm_PinMap_TC39xB_516.{h,c}` | ATOM `ToutMap` / TIM `TinMap` objects, channel indices, ALT‑output index |
| S4 | `Libraries/iLLD/.../Port/Std/IfxPort.c` | `IfxPort_setPinMode()` implementation (cost of the direction switch) |
| S5 | Project source | `src/bsw/gpio_cfg.h` (TOM0/TOM3 on **P00.0–P00.15**), `src/bsw/Cpu{0..3}_Main.c` (**P20.11–P20.14** = LEDs D306‑D309), `src/bsw/Uart.c` (**P14.0/P14.1** ASCLIN0), `docs/SENSORS.md` (**P22.7–P22.11** IMU/QSPI0, **P13.1/P13.2** I2C0, VFLEX group, 5 V pad swing) |

Package confirmed as **516‑ball** from `Configurations/Ifx_Cfg.h:68`
(`#define IFX_PIN_PACKAGE_516 1`), which selects `IfxGtm_PinMap_TC39xB_516.h`
via `IfxGtm_PinMap.h:50‑52`. `DEVICE_TC39XB` is set per project convention.

---

## 2. Part A — Recommended four pins (single‑pin bidirectional)

All four are on the **PERIPHERALS header X702 (top) / X802 (bottom)** — identical
pinout, S2 Fig. 6‑1. TX uses **ATOM0**, RX uses **TIM0** (channel index matches the
ATOM0 channel — one motor ↔ one channel number). `TIM7` is given as an alternative
that keeps TIM0 completely free.

| Motor | Port pin | Ball | Header pin (X702/X802) | TX: ATOM inst/ch (TOUT) | RX: TIM inst/ch (recommended) | RX alt (TIM7) |
|---|---|---|---|---|---|---|
| M1 | **P22.1** | W24 | **30** | ATOM0 / ch0 (TOUT48) | TIM0 / ch0 | TIM7 / ch2 |
| M2 | **P22.0** | W25 | **32** | ATOM0 / ch1 (TOUT47) | TIM0 / ch1 | TIM7 / ch3 |
| M3 | **P22.2** | Y25 | **36** | ATOM0 / ch3 (TOUT49) | TIM0 / ch3 | TIM7 / ch1 |
| M4 | **P22.3** | Y24 | **34** | ATOM0 / ch4 (TOUT50) | TIM0 / ch4 | TIM7 / ch0 |

**iLLD pin objects** (`IfxGtm_PinMap_TC39xB_516.h` / `.c`):

| Port pin | ATOM output object (`.h` line) | TIM input object — TIM0 (`.h` line) | TIM input object — TIM7 (`.h` line) |
|---|---|---|---|
| P22.1 | `IfxGtm_ATOM0_0_TOUT48_P22_1_OUT` (h:156) | `IfxGtm_TIM0_0_P22_1_IN` (h:1156) | `IfxGtm_TIM7_2_P22_1_IN` (h:1760) |
| P22.0 | `IfxGtm_ATOM0_1_TOUT47_P22_0_OUT` (h:176) | `IfxGtm_TIM0_1_P22_0_IN` (h:1166) | `IfxGtm_TIM7_3_P22_0_IN` (h:1767) |
| P22.2 | `IfxGtm_ATOM0_3_TOUT49_P22_2_OUT` (h:209) | `IfxGtm_TIM0_3_P22_2_IN` (h:1188) | `IfxGtm_TIM7_1_P22_2_IN` (h:1753) |
| P22.3 | `IfxGtm_ATOM0_4_TOUT50_P22_3_OUT` (h:223) | `IfxGtm_TIM0_4_P22_3_IN` (h:1196) | `IfxGtm_TIM7_0_P22_3_IN` (h:1745) |

The ATOM objects all carry `IfxPort_OutputIdx_alt1` (`.c:60/80/113/127`), i.e. the
GTM output is **ALT1** on these pins — matching the datasheet, which lists
`GTM_TOUT47…50` at output line **O1** (S1, Table 2‑12, pp. 100‑101). So the "drive"
mode is `IfxPort_Mode_outputPushPullAlt1`.

### 2.1 Why these four — filter trace

| Filter | Result for P22.0‑P22.3 | Source |
|---|---|---|
| ATOM output on the pin | ✅ TOUT47‑50 at O1 = ATOM0 ch1/0/3/4 | S1 Table 2‑12; S3 `.c:60/80/113/127` |
| TIM input on the **same** pin | ✅ TIM0 ch1/0/3/4 (mux input `_7/_8`) **and** TIM7 ch3/2/1/0 (input select 1) | S1 Table 2‑12 (`GTM_TIM0_INx`, `GTM_TIM7_INx`); S3 `.c:1649‑1671` |
| On an 80‑pin header, PERIPHERALS preferred | ✅ X702/X802 pins 30/32/34/36 | S2 Fig. 6‑1 |
| No assembled 0‑Ω / transceiver bridge | ✅ none of P22.0‑P22.3 appears in Table 4‑3 | S2 Table 4‑3 (only P22.0 as *CAN1 alt R314 “not assembled”* — footprint only, no populated part; see note) |
| Not an “on‑board only” signal | ✅ not in Table 6‑1 | S2 Table 6‑1 |
| Not used by this project | ✅ project uses P22.**7‑11** only (IMU); P22.0‑6 free | S5 `docs/SENSORS.md` §3 (“P22.4–P22.7 show no onboard transceiver”), lines 31‑39/106 |

> Note on P22.0: Table 4‑3 has **R314 “Connect P00.0 with TXD of CAN1 transceiver
> (not assembled)”** — that is P00.0, *not* P22.0. P22.0 has **no** entry in Table
> 4‑3 at all. So P22.0 is clean. (Verified: `grep P22 Table 4‑3` → no rows.)

### 2.2 Reserve pins (also all PERIPHERALS X702/X802, all free)

Use these if a pin is later needed elsewhere, or for a 5th/6th channel. Every row
was checked against Table 4‑3, Table 6‑1 and project usage the same way as §2.1.

| Port pin | Header pin | ATOM output object | TIM input object | Notes |
|---|---|---|---|---|
| P22.4 | 9 | `IfxGtm_ATOM1_4_TOUT130_P22_4_OUT` | `IfxGtm_TIM3_0_P22_4_IN` | only one TIM option (TIM3 ch0) |
| P22.5 | 11 | `IfxGtm_ATOM3_4_TOUT131_P22_5_OUT` | `IfxGtm_TIM3_1_P22_5_IN` | only one TIM option (TIM3 ch1) |
| P22.6 | 13 | `IfxGtm_ATOM3_5_TOUT132_P22_6_OUT` | `IfxGtm_TIM3_2_P22_6_IN` | TIM2 ch6 also available |
| P10.7 | 73 | `IfxGtm_ATOM1_0_TOUT109_P10_7_OUT` | `IfxGtm_TIM0_0_P10_7_IN` | TIM1 ch0 also |
| P10.8 | 72 | `IfxGtm_ATOM1_5_TOUT110_P10_8_OUT` | `IfxGtm_TIM0_5_P10_8_IN` | TIM1 ch5 / TIM4 ch0 |
| P10.3 | 74 | `IfxGtm_ATOM1_3_TOUT105_P10_3_OUT` | `IfxGtm_TIM0_3_P10_3_IN` | TIM1 ch3 / TIM4 ch6 |
| P10.4 | 25 | `IfxGtm_ATOM0_6_TOUT106_P10_4_OUT` | `IfxGtm_TIM0_6_P10_4_IN` | TIM1 ch6 / TIM4 ch7 |
| ~~P10.5~~ | 23 | — | — | **NOT free: HWCFG4 boot strap** (S201, R231). Do not use. |
| ~~P23.3~~ | 24 | — | — | **Allocated to ESC telemetry** (ASCLIN6 RX, §11.2) |
| P23.4 | 26 | `IfxGtm_ATOM0_7_TOUT45_P23_4_OUT` | `IfxGtm_TIM6_3_P23_4_IN` | free (P23.0/1 are CAN1) |

> **Board caveats on the DShot pins & reserves (see `PINNING.md` §4/§5):**
> - **P22.2 / P22.3 carry an HSCT footprint stub** (IEEE‑1394 sockets X201/X202,
>   unpopulated; Manual §3.13, R250‑R260). Functionally free, but the short stub is
>   why these pads are `LVDS_TX` class — the §8.6 open‑drain bench test covers exactly
>   this. P20.0 and P21.0‑P21.5 share the same footprint group.
> - **P10.5 is HWCFG4** and **P33.5 is on‑board LED D303** — neither is free; both were
>   removed from the reserve pool above.

> The **GTM/PORTS header (X704/X804)** carries even more free GTM pins
> (P02.x, P01.x, P33.8‑15, P34.x …) if you want to keep Port 22 fully for the IMU
> neighbourhood — see S2 Fig. 6‑2. They were not chosen because the user preferred
> PERIPHERALS and because the P22 block is cleaner (one ATOM, one TIM).

---

## 3. How the single pin is driven (per‑motor state machine)

Both the ATOM output line and the TIM input line stay *configured* the whole time;
only the pad direction (IOCR) is flipped:

1. **TX phase** — pad = output ALT1 → ATOM0 drives the 16 DShot bits.
   `IfxPort_setPinModeOutput(&MODULE_P22, n, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_alt1)`
2. **Turn‑around** — pad = input (high‑Z), DShot line idles high.
   `IfxPort_setPinModeInput(&MODULE_P22, n, IfxPort_InputMode_pullUp)`
   The pad output driver is now disabled; TIM0/TIM7 captures the ESC reply.
3. **Back to TX** — repeat step 1. Cycle = 1 ms.

Both calls reduce to a single `__ldmst` on the Port‑22 IOCR register (see §5).

---

## 4. Part B — Two‑pin fallback (NOT required here)

Because §0 shows every candidate pin does **both**, the split scheme is unnecessary.
It is documented only in case a future board revision or a signal‑integrity finding
forces separate TX and RX pads.

**Scheme:** per motor, one ATOM output pin and one TIM input pin, wired together
externally; during RX the ATOM pin is put to **high‑Z (input)** so it does not fight
the ESC. Eight suitable pins, same systematics (all PERIPHERALS X702/X802, all free):

| Role | Port pin | Header pin | iLLD object |
|---|---|---|---|
| M1 TX | P22.1 | 30 | `IfxGtm_ATOM0_0_TOUT48_P22_1_OUT` |
| M1 RX | P10.7 | 73 | `IfxGtm_TIM0_0_P10_7_IN` |
| M2 TX | P22.0 | 32 | `IfxGtm_ATOM0_1_TOUT47_P22_0_OUT` |
| M2 RX | P10.8 | 72 | `IfxGtm_TIM0_5_P10_8_IN` |
| M3 TX | P22.2 | 36 | `IfxGtm_ATOM0_3_TOUT49_P22_2_OUT` |
| M3 RX | P10.3 | 74 | `IfxGtm_TIM0_3_P10_3_IN` |
| M4 TX | P22.3 | 34 | `IfxGtm_ATOM0_4_TOUT50_P22_3_OUT` |
| M4 RX | P23.4 | 26 | `IfxGtm_TIM6_3_P23_4_IN` |

**Tri‑stating the ATOM output pin in iLLD** (also the exact call the single‑pin
scheme uses for its RX phase):

```c
/* High‑Z the ATOM output pad — disables the pad output driver, pad becomes input.
   IOCR direction bits go to "input"; the ATOM keeps toggling but is disconnected. */
IfxPort_setPinModeInput(&MODULE_P22, 1u, IfxPort_InputMode_pullUp);   /* line idles high */

/* Re‑enable driving as ATOM (ALT1) output: */
IfxPort_setPinModeOutput(&MODULE_P22, 1u,
                         IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_alt1);
```

There is **no** separate “tri‑state” enum — on AURIX a GPIO/ALT output pad is
tri‑stated by switching its IOCR to an **input** mode (the output driver is only
active in the `output*` modes). Both helpers are thin wrappers over the same
`__ldmst` on IOCR (`IfxPort.c`).

---

## 5. Part D — Is the output↔input switch fast enough for the ~30 µs gap?

**Yes, by a wide margin.** For any port other than P40/P41, `IfxPort_setPinMode()`
is literally one atomic instruction (`IfxPort.c`):

```c
void IfxPort_setPinMode(Ifx_P *port, uint8 pinIndex, IfxPort_Mode mode)
{
    ...
    /* P40/P41 special‑case (PDISC + ENDINIT) is skipped for Port 22 */
    __ldmst(&iocr[iocrIndex].U, (0xFFUL << shift), (mode << shift));   /* single RMW */
}
```

- Port 22 is **not** P40/P41, so the ENDINIT/PDISC branch is skipped — no
  watchdog‑password dance.
- Cost = **one `__ldmst`** (atomic load‑modify‑store) to a Port SFR on the SPB
  peripheral bus. Order of magnitude: a **handful to a few tens of core cycles**,
  i.e. **well under 1 µs** at 300 MHz (an SPB SFR access is a few bus cycles, not
  the full pipeline). `setPinModeInput`/`setPinModeOutput` are the same one write.
- Budget: the ~30 µs ESC turn‑around ≈ **9 000 core cycles** at 300 MHz. The
  direction flip consumes a tiny fraction of that. Even flipping all four motors
  costs four `__ldmst`s.

> Precise cycle count is not pinned down by the sources (it depends on SPB clock
> ratio and bus arbitration at that instant). What *is* certain from the code path:
> it is a single uninterruptible register write with no ENDINIT sequence — orders
> of magnitude below 30 µs. Marked here rather than fabricating a hard number.

Two implementation notes (not blockers):
- Because P22.0‑P22.3 share **one IOCR register per group of 4** (`IOCR0` covers
  pins 0‑3), the four `__ldmst`s are independent RMWs on the *same* register; do
  them from a single context to avoid a read‑modify‑write race, or use the
  group helper `IfxPort_setGroupModeInput/Output` which writes the group mask once.
- The 30 µs gap is defined by the ESC, not the MCU; the firmware just needs to be
  in input mode before the reply starts. A GTM/STM timestamp of TX‑end + a short
  guard is enough.

---

## 6. Part C — Resource conflicts with the existing project

### 6.1 GTM submodule usage — **no channel conflict**

| Resource | Used by project? | DShot needs | Verdict |
|---|---|---|---|
| **TOM0, TOM3** | ✅ `gpio_cfg.h` — P00.0‑P00.15 hardware PWM/digital | — | untouched (DShot uses ATOM, not TOM) |
| **ATOM0‑4** | ❌ not used anywhere in `src/` | ATOM0 (4 ch) | **free** |
| **TIM0‑7** | ❌ not used anywhere in `src/` | TIM0 or TIM7 (4 ch) | **free** |
| GTM module enable + CMU (GCLK, FXCLK0‑4) | ✅ `gpio.c:144‑148` enables GTM and FXCLK0‑4 for TOM | ATOM needs a CMU clock too | **shared, not exclusive** — see below |

`src/bsw/gpio.c` already calls `IfxGtm_enable()`, sets GCLK to the module frequency
and enables FXCLK0‑4 (`gpio.c:146‑148`). ATOM channels can source the **same**
CMU FXCLKs — multiple submodules sharing an FXCLK is normal and not a conflict. The
DShot init must therefore **not** re‑initialise the whole GTM from scratch if the
GPIO feature already ran; it should only configure its own ATOM0/TIM cluster and
pick an existing FXCLK whose frequency gives adequate resolution for the 3.33 µs
DShot300 bit (this frequency choice is an implementation detail, not decided here).

### 6.2 DMA — **free**

No general‑purpose DMA is used in `src/` (`grep IfxDma|MODULE_DMA` → nothing). The
Ethernet path uses the GETH MAC’s own dedicated DMA engine, **not** the central
`MODULE_DMA`. So the 128‑channel central DMA is entirely available for the
ATOM‑compare‑reload transfers (~4 channels, one per motor).
*(The GTM→DMA trigger routing via the interrupt/SRC path exists but its exact
wiring is an implementation detail — not resolved here.)*

### 6.3 Ethernet / XCP / STM tick — **no overlap**

- **Ethernet (RGMII)** uses pads P11.0‑P11.12 and the GETH module (S2 Table 6‑1);
  no GTM resource. Chosen DShot pins (P22.x) are disjoint.
- **XCP** runs over UDP/Ethernet — software + GETH, no GTM/TIM.
- **STM tick**: CPU0 uses `MODULE_STM0` comparator IR0 for the 1 ms scheduler
  (`Cpu0_Main.c:86‑92,133`); `SysTime.c` reads `STM0` lower. The DShot 1 ms cycle
  should hook the **existing** scheduler tick or use a different core’s STM /
  a GTM timer — it must not reprogram STM0’s comparator. No hardware conflict,
  just a “don’t double‑book STM0” note.

### 6.4 Pins to keep clear (project‑owned) — cross‑check

Confirmed occupied and therefore avoided: QSPI0 IMU **P22.8‑P22.11** + INT **P22.7**
(`SENSORS.md`); I2C0 **P13.1/P13.2**; ASCLIN0 console **P14.0/P14.1** (`Uart.c:31/34`);
QSPI2 TLF35584 **P14.2/P15.3/P15.6/P15.7**; RGMII **P11.0‑P11.12**; **plus** two the
user list did not mention but the code does:
- **P20.11‑P20.14 = core LEDs D306‑D309** (`Cpu0..3_Main.c` → `Led_init(&MODULE_P20, 11..14)`).
- **P00.0‑P00.15 = the GPIO/PWM feature** (`gpio_cfg.h`, TOM0/TOM3).

None of these overlaps P22.0‑P22.3 or the reserve pins.

---

## 7. Voltage‑domain caveat (must verify on the real board)

The logical selection is solid; the **electrical** level is the one thing the
in‑repo sources cannot fully close:

- Port 22 balls sit next to the **`VEXT` / `VDD`** supply pins in the LFBGA‑516 ball
  map (S1), i.e. the general microcontroller port I/O domain — **not** the always‑3.3 V
  `VFLEX` flex‑port domain.
- The manual (S2, Table 5‑5 / power section) states **`V_UC` = “5 V or 3.3 V depends
  on assembled TLF35584”**, and **`VEXT` is tied to `V_UC`** (R527 assembled when no
  external memory). `VFLEX` = 3.3 V always.
- `docs/SENSORS.md` (same board) independently found that Port‑22 **outputs swing to
  5 V** and added a level buffer for the IMU SPI — consistent with a 5 V‑TLF board.

Implications for DShot:
1. **TX**: a 5 V high into the GOKU G45M / AM32 signal input is usually tolerated,
   but confirm the ESC’s signal‑pin abs‑max; a small series resistor (~100 Ω) is
   cheap insurance.
2. **RX**: the ESC drives the line back at *its* logic level (AM32 MCU typically
   3.3 V). A 3.3 V high into a 5 V‑domain input sits near V_IH (≈0.6·VDDP ≈ 3.0 V) —
   marginal but generally readable. On a 3.3 V‑TLF board this is a non‑issue.
3. **Native‑3.3 V pins exist, but not four, and not conveniently** *(corrected)*.
   The `VFLEX` (3.3 V) domain is **Port 11 only** — verified against the Data Sheet
   **Pad List Table 2‑63**, where every P11.x pad carries the `VFLEX / ES` tag while
   P14.7/9/10, P15.7/8 and **P20.12/13 all carry `VEXT / ES` (5 V)**. (The earlier
   `SENSORS.md` claim that those P14/P15/P20 pins are VFLEX is **wrong**; see
   `PINNING.md` §5.1.) Of Port 11, P11.0–P11.12 are RGMII (board‑committed), leaving
   exactly **three free VFLEX pins: P11.13/14/15** (GTM/PORTS header X704·67/6/8).
   So a clean *four‑channel* native‑3.3 V DShot is **not** available, and even the
   three come with drawbacks (see §7.1). External level handling on P22 is still the
   realistic path for four channels.

### 7.1 Was evaluated: move DShot to the VFLEX (3.3 V) pins?

The three free VFLEX pins (P11.13/14/15) are GTM‑capable and would give the ESC a
**native 3.3 V** signal — eliminating the open‑drain trick, the four 1.5 kΩ pull‑ups
and the §8.6 bench test. **Rejected** for a 4‑motor driver because:

- **Only three free VFLEX pins** — you cannot get a 4th without stealing an RGMII pin
  (P11.0–12), which breaks Ethernet. A 4th channel would have to fall back to a 5 V
  VEXT pin anyway, re‑introducing the level problem.
- **SLOW pad class** (Table 2‑63: `SLOW / PU1`) — edge rate / drive strength is lower
  than the FAST/LVDS pads; not characterised here for the 3.33 µs DShot300 bit. Would
  need its own validation — trading the §8.6 bench test for a different one.
- **Wrong header** — X704 GTM/PORTS, not the PERIPHERALS block the other four DShot
  lines and the sensors live on; splits the harness across two connectors.
- **P20.12/P20.13 do not help** — they are VEXT (5 V), not VFLEX, despite being
  free‑able core LEDs.

Net: for **three or fewer** ESCs a native‑3.3 V build on P11.13/14/15 is worth
considering; for the **four‑motor** target, P22 + open‑drain (§8) remains best.
Channels don’t clash with the P22/ATOM0/TIM0 block (P11.13→ATOM5.6/TIM2.6,
P11.14→ATOM5.7/TIM2.7, P11.15→ATOM6.0/TIM0.7).

**Status:** resolved — `VEXT` = `V_UC` = **5 V** (TLF35584QVVS1, R527 assembled,
measured P00.1 = 5.0 V). Level adaptation is therefore required; see **§8
Pegelanpassung** for the recommended open‑drain solution and the translator fallback.

---

## 8. Pegelanpassung (level shifting: open‑drain vs. external translator)

**Problem** (from §7): Port 22 is in the 5 V VEXT/V_UC domain — confirmed on this
board (R527 assembled, TLF35584QVVS1, measured P00.1 high = 5.0 V). The GOKU G45M
(AT32 MCU) signal input is 3.3 V logic and 5 V would destroy it.

**Result up front:** the planned **open‑drain + external 1.5 kΩ pull‑up to 3.3 V**
is supported by the register model and the DC data, and is the recommended solution.
No level‑shifter IC is required. Because the datasheet does not spell out open‑drain
sink behaviour *specifically for an LVDS_TX pad while it is driven by a GTM ALT
function*, a one‑pin bench test (§8.6) should confirm it on silicon before all four
channels are populated. The 74LVC1T45 translator (§8.5) is the documented fallback.

> Register‑level note: this repo does **not** contain the AURIX TC3xx *User Manual*
> (only Data Sheet, Data Sheet Addendum, Errata Sheet, TriBoard Manual — see `docs/`).
> The IOCR/PDR **register semantics** below are therefore cited from the **iLLD source**
> (which encodes the exact register field values) plus the Data Sheet electrical
> tables, not from the User Manual’s PORTS chapter.

### 8.1 (A) Port output modes and where open‑drain is available

The TC3xx pad output stage is selected by the **`PC` field of `Pn_IOCRx`** (5 bits
per pin). iLLD enumerates every legal value (`IfxPort.h:105‑123`):

| Output driver | GPIO | ALT1 | … | ALT7 |
|---|---|---|---|---|
| **Push‑pull** (`PC` = `10xxx`) | `outputPushPullGeneral` 0x80 | `…Alt1` 0x88 | … | `…Alt7` 0xB8 |
| **Open‑drain** (`PC` = `11xxx`) | `outputOpenDrainGeneral` 0xC0 | `…Alt1` **0xC8** | … | `…Alt7` 0xF8 |

Decoding the field (enum value `>> 3` = `PC[4:0]`): **`PC[4]`=1** → output,
**`PC[3]`** → 0 = push‑pull / **1 = open‑drain**, **`PC[2:0]`** → output source
(0 = GPIO, 1…7 = ALT1…ALT7). So *push‑pull vs open‑drain* and *which function
drives the pad* are **independent bit fields of the same PC value**.

Open‑drain is a property of the **standard GPIO / LVDS_TX pad class**, not a special
pin. The DShot pins (P22.0‑3) are datasheet buffer type **`LVDS_TX`**, which is the
same **“Fast 5 V GPIO”** pad class documented in Data Sheet Table 3‑7 (p. 414) — the
class whose output driver DC parameters (RDSON, rise/fall) are specified. No per‑pin
“open‑drain not available” exception was found for these pins in the Data Sheet.
**→ open‑drain is available for all the pins proposed in this document.** (The one
caveat: the Data Sheet specifies the pad *output driver* generically; it does not
separately guarantee the sink path while the pad is in **ALT + open‑drain** — hence
the bench test.)

### 8.2 (B) Does open‑drain still apply when a GTM ATOM drives the pin? — **Yes**

This is the crux, and the answer is **positive at the register level**:

- A GTM ALT function supplies only the **output data** into the pad; the **pad output
  characteristic (push‑pull vs open‑drain) stays under IOCR `PC[3]`** regardless of
  the data source. That is exactly why iLLD defines discrete
  `IfxPort_Mode_outputOpenDrainAlt1…Alt7` values (`IfxPort.h:117‑123`) — they would be
  meaningless if open‑drain did not apply to alternate outputs.
- `IfxPort_setPinModeOutput()` is a thin wrapper that **ORs** the driver mode and the
  ALT index into one IOCR write (`IfxPort.h:768‑771`):
  ```c
  IfxPort_setPinMode(port, pin, (IfxPort_Mode)(index | mode));
  /* IfxPort_OutputMode_openDrain (0xC0) | IfxPort_OutputIdx_alt1 (0x88) = 0xC8
     = IfxPort_Mode_outputOpenDrainAlt1  → PC = 11001b                        */
  ```
  and `IfxPort_setPinMode()` writes it with a single `__ldmst` on `Pn_IOCRx`
  (`IfxPort.c`, `IfxPort_setPinMode`).

So the DShot **drive** mode for each motor becomes:
```c
/* P22.x driven by ATOM0 (ALT1, see §2), pad stage = open-drain */
IfxPort_setPinModeOutput(&MODULE_P22, n,
                         IfxPort_OutputMode_openDrain, IfxPort_OutputIdx_alt1);
```

**Bonus from open‑drain — the RX turn‑around may not even need a mode change.**
In open‑drain, a logic ‘1’ = pad released (Hi‑Z), line pulled to 3.3 V by the
external resistor; ‘0’ = pad sinks. If the ATOM idles at ‘1’ after the frame, the
pad is already Hi‑Z and the ESC can pull the line low for its reply while the pin
stays in `outputOpenDrainAlt1`; the input buffer / TIM capture still sees the pad.
(Switching to `inputPullUp` for RX, as in §3, remains a clean alternative.) This
also naturally matches bidirectional DShot’s **idle‑high** convention and keeps the
line defined during the input phase — the intended behaviour.

### 8.3 (C) Open‑drain **and** TTL input level together? — **Yes, different registers**

They do not interact — they live in **two separate registers**:

| Setting | Register / field | iLLD call | Bits |
|---|---|---|---|
| Push‑pull vs **open‑drain**, and ALT select | `Pn_IOCRx.PC` (5 bit/pin) | `IfxPort_setPinMode / …ModeOutput` | `PC[3]` = OD, `PC[2:0]` = ALT |
| **TTL** vs Automotive input level + driver speed | `Pn_PDRx.PDy` (4 bit/pin) | `IfxPort_setPinPadDriver` | `ttlSpeed1‑4` = 8‑11, `ttl3v3Speed1‑4` = 12‑15, `cmosAutomotive1‑4` = 0‑3 |

`IfxPort_setPinPadDriver()` writes **`Pn_PDRx`** with an `__ldmst` (ENDINIT‑protected)
— a completely different register from IOCR (`IfxPort.c`,
`IfxPort_setPinPadDriver`; enum `IfxPort.h:155‑166`). So **open‑drain output and TTL
input threshold are configured independently and coexist** on the same pin.

The TTL threshold is what makes the RX read work: Data Sheet Table 3‑7 (p. 414)
gives **`VIH` = 2.0 V (min) for TTL at VEXT = 4.5 V** (vs the Automotive‑Level
`VIH ≈ 0.7·VEXT ≈ 3.5 V`, which a 3.3 V high would fail). A 3.3 V ESC high therefore
reads as logic 1 with ~1.3 V margin. `VIL(TTL)` = 0.8 V (max).

> Practical note: `Pn_PDR` is ENDINIT‑protected, so set the pad driver mode **once at
> init** (not in the 1 ms hot loop). Only the IOCR direction flip runs in the loop,
> and that is a single unprotected `__ldmst` (§5).
>
> Errata `[PORTS_TC.H018]` (Errata Sheet v2.7, §4.134) only clarifies that the User
> Manual’s driver‑strength “sm” footnote is imprecise; it does **not** affect
> open‑drain or TTL functionality. Not a blocker.

### 8.4 (D) Rise‑time with the pull‑up, and the optimal value

Open‑drain rising edges are driven only by the pull‑up (RC). With **R = 1.5 kΩ** to
3.3 V and **C_load = 30–50 pF** (pad `CIO ≈ 2–3 pF` + 2.5 pF package per Table 3‑7,
plus wire/ESC input):

| Quantity | Formula | 30 pF | 50 pF |
|---|---|---|---|
| τ = R·C | — | 45 ns | 75 ns |
| edge to cross `VIH` = 2.0 V | 0.93·τ | 42 ns | 70 ns |
| 10–90 % rise | 2.2·τ | 99 ns | 165 ns |

Compare to the timing budget:
- **DShot300 TX:** bit 3.33 µs, shortest high element ≈ 1.25 µs → edge ≤ 165 ns is
  **≤ 13 %** of the shortest pulse. ✔
- **ESC reply 375 kbit/s:** bit 2.67 µs, driven by the **same** pull‑up → edge ≤ 165 ns
  is **≤ 6 %** of a bit. ✔

**Low level (sink):** with pad `RDSON` = 125/225/320 Ω (min/typ/max, Table 3‑7),
`V_OL = 3.3 V · RDSON/(R+RDSON)` = **0.25 / 0.43 / 0.58 V** — all below `VIL(TTL)`
= 0.8 V, with margin even at max RDSON. Sink current ≈ 3.3 V / 1.5 kΩ ≈ **2.2 mA**,
within the 2 mA DC test point (and far under the 8 mA strong driver).

**Optimal pull‑up:** 1.5 kΩ (the planned value) is a good balance.
- Do **not** go below ~1.2 kΩ: at 1.0 kΩ, worst‑case `V_OL` = 3.3·320/1320 = **0.80 V**,
  right at the TTL limit.
- Up to ~2.2 kΩ is also fine (10–90 % rise ≈ 150–240 ns, still ≪ 1.25 µs; lower sink
  current, lower `V_OL`).
- **Recommendation: 1.5 kΩ** (1.2 kΩ if you want faster edges on longer wires and
  can spend the extra sink current; keep `V_OL` < 0.8 V at max RDSON).

### 8.5 (E) Fallback: external 74LVC1T45 translators (if §8.6 fails)

If the bench test shows open‑drain is not honoured in ALT mode, or `V_OL` is
inadequate, use one **74LVC1T45** (single‑bit, dual‑supply, **explicit `DIR`**)
per channel:

- **Wiring:** `A`‑side `VCCA` = VEXT (5 V), pin A ↔ TC399 P22.x; `B`‑side `VCCB` =
  3.3 V, pin B ↔ ESC signal wire. The TC399 pad stays **push‑pull ALT** (ATOM for TX,
  TIM for RX as in §2) — the translator, not the pad, isolates the 5 V.
- **Extra GPIOs:** one `DIR` line per channel = **4 GPIOs**. Because all four motors
  share the same 1 ms frame cadence, a **single shared `DIR`** can drive all four
  translators if the driver switches them in lockstep — reducing it to **1 GPIO**.
  (Free pins are plentiful — pick any digital output from the DSHOT_PINS reserve/GTM
  headers; `DIR` needs no timer.)
- **DIR timing relative to frame end** (per 1 ms cycle):
  1. `DIR` = A→B (transmit) during the ≈ 53.3 µs, 16‑bit ATOM frame.
  2. After the last bit, within the ~30 µs ESC gap, flip `DIR` = B→A (receive) — leave
     a few‑µs guard after the final edge so the last bit is not clipped, but well
     before the reply starts (~30 µs). The `DIR` write is one OMR/IOCR write (tens of
     ns; `IfxPort_setPinState`/`setPinMode`), and the ’1T45 `DIR`‑to‑output valid is a
     few ns — timing is trivial inside the 30 µs window.
  3. After the ≈ 56 µs reply (~21 bits @ 2.67 µs), flip `DIR` back to A→B before the
     next frame.
- **Downsides vs open‑drain:** 4 extra parts + board area; the ’1T45 is push‑pull both
  ways, so a mistimed `DIR` at the turn‑around risks momentary contention, and it does
  **not** give idle‑high “for free” (the driven side must hold the line high). This is
  why open‑drain is preferred.

### 8.6 Bench test to confirm open‑drain‑in‑ALT on silicon (do this before populating 4×)

The one thing the sources cannot fully guarantee is that an **LVDS_TX pad sinks
correctly while driven by a GTM ATOM in open‑drain** (§8.1 caveat). Confirm it on
one pin:

| Item | Value |
|---|---|
| Pin | **P22.0**, header X702/X802 **pin 32** (ATOM0 ch1 / ALT1, see §2) |
| Firmware | ATOM0 ch1 PWM on P22.0 at DShot300 bit rate (or a plain 300 kHz square) |
| Pad mode | `IfxPort_setPinModeOutput(&MODULE_P22, 0, IfxPort_OutputMode_openDrain, IfxPort_OutputIdx_alt1)` |
| Pad driver | `IfxPort_setPinPadDriver(&MODULE_P22, 0, IfxPort_PadDriver_ttlSpeed1)` (set once) |
| External | 1.5 kΩ from the pin to a clean **3.3 V** bench rail; C_load ≈ 47 pF (scope probe + short wire) |

**Measure at the pin (scope):**
1. **High level** — must sit at **3.3 V**, *not* 5 V. This proves the pad *releases*
   in the ‘1’ state (open‑drain honoured) and does **not** source to VEXT.
2. **Low level `V_OL`** — must be **≤ 0.8 V** (expect ~0.4–0.6 V). Proves adequate sink.
3. **Rising edge** — 10–90 % and time‑to‑2.0 V; expect < ~0.2 µs.
4. **RX path** — hold the ATOM output idle‑high (or set the pad to `inputPullUp`),
   pull the node low from a 3.3 V source, and confirm `Pn_IN` / a TIM capture registers
   the low.

**Conclusions:**
- High = 3.3 V (no 5 V excursion) **and** `V_OL` ≤ 0.8 V → **open‑drain works in ALT
  mode → GO**, populate all four channels with 1.5 kΩ pull‑ups.
- High rails toward **5 V** → open‑drain **not** honoured for this pad in ALT mode on
  this silicon → **use the 74LVC1T45 fallback (§8.5).**
- High = 3.3 V but `V_OL` > 0.8 V → lower the pull‑up to 1.2 kΩ and/or select a
  stronger driver speed grade, re‑measure.

---

## 9. Open questions / where the sources are silent

- ~~**Exact TLF35584 variant on this board** → sets Port‑22 I/O voltage (§7).~~
  **Resolved:** VEXT = V_UC = **5 V** (R527 assembled, TLF35584QVVS1, measured
  P00.1 high = 5.0 V). This is why the level adaptation in §8 is required.
- **Open‑drain sink while a GTM ATOM drives an LVDS_TX pad in ALT mode** — supported
  per the register model and iLLD (§8.2), but not guaranteed by an explicit Data
  Sheet line for exactly this combination → confirm with the bench test in §8.6
  before populating all four channels.
- **Exact cycle count of the IOCR write** (§5). The code path is a single `__ldmst`
  with no ENDINIT; the absolute cycle count depends on SPB clock ratio/arbitration
  and is not specified. Bounded well under 1 µs; not fabricated.
- **GTM→DMA trigger routing and the CMU/FXCLK frequency** for the ATOM compare
  cadence (§6.1/§6.2) are implementation choices, not pin facts — deliberately left
  open here.
- **ESC signal‑pin voltage tolerance** (Flywoo GOKU G45M / AM32) is an external‑part
  spec, outside these sources.

---

## 10. Source index (exact locations)

| Ref | File | Location |
|---|---|---|
| Package = 516 | `Configurations/Ifx_Cfg.h` | line 68 |
| Pinmap dispatch | `Libraries/iLLD/TC3xx/Tricore/_PinMap/IfxGtm_PinMap.h` | lines 50‑52 |
| ATOM/TIM intersection (261/261) | `.../TC39xB/IfxGtm_PinMap_TC39xB_516.h` | full file |
| P22.0‑3 ATOM objects | `.../IfxGtm_PinMap_TC39xB_516.h` | h:156,176,209,223 |
| P22.0‑3 TIM0 objects | same | h:1156,1166,1188,1196 |
| P22.0‑3 TIM7 objects | same | h:1745,1753,1760,1767 |
| ALT1 output index | `.../IfxGtm_PinMap_TC39xB_516.c` | c:60,80,113,127 |
| `setPinMode` = one `__ldmst` | `Libraries/iLLD/.../Port/Std/IfxPort.c` | `IfxPort_setPinMode()` |
| Port 22 ALT/TIM functions | `docs/infineon-tc39x-datasheet-en.pdf` | Table 2‑12, pp. 100‑101 |
| Port 22 balls + VEXT domain | same | LFBGA‑516 ball map (Tables 2‑2/2‑3) |
| PERIPHERALS X702/X802 pinout | `docs/infineon-triboardmanual-tc3x9-um-en.pdf` | §6.2 Fig. 6‑1, p. 6‑2 |
| GTM/PORTS X704/X804 pinout | same | §6.2 Fig. 6‑2, p. 6‑3 |
| Table 4‑3 resistors (transceiver bridges) | same | §4.2, pp. 4‑4/4‑5 |
| Table 6‑1 on‑board‑only signals | same | §6.1, p. 6‑1 |
| V_UC/VEXT/VFLEX supply names | same | Table 5‑5 (“V_UC 5 V or 3.3 V …”, “VFLEX 3.3 V”) |
| GPIO feature on P00 (TOM) | `src/bsw/gpio_cfg.h`, `src/bsw/gpio.c` | table; gpio.c:144‑148 |
| Core LEDs on P20.11‑14 | `src/bsw/Cpu0..3_Main.c` | `Led_init(&MODULE_P20, 11..14)` |
| ASCLIN0 P14.0/1 | `src/bsw/Uart.c` | lines 31,34 |
| IMU/I2C pins, VFLEX group, 5 V swing | `docs/SENSORS.md` | §3, lines 31‑39, 73‑74, 102‑109 |
| IOCR `PC` modes (push‑pull / open‑drain × GPIO/ALT1‑7) | `Libraries/iLLD/.../Port/Std/IfxPort.h` | `IfxPort_Mode` enum, lines 105‑123 |
| `OutputMode`/`OutputIdx` OR‑construction | same | lines 130‑145; `IfxPort_setPinModeOutput` inline, 768‑771 |
| `IfxPort_PadDriver` (TTL / TTL‑3v3 / CMOS‑AL) | same | enum lines 155‑166 |
| IOCR write (`__ldmst`), PDR write (ENDINIT) | `Libraries/iLLD/.../Port/Std/IfxPort.c` | `IfxPort_setPinMode`, `IfxPort_setPinPadDriver` |
| Fast 5 V GPIO pad DC: RDSON, rise/fall, `CIO` | `docs/infineon-tc39x-datasheet-en.pdf` | Table 3‑7, pp. 414‑415 |
| TTL input `VIH` = 2.0 V / `VIL` = 0.8 V | same | Table 3‑7 (and PORST pad Table 3‑6), pp. 413‑414 |
| Pad classes support AL **or** TTL level | same | §3.5 “5 V / 3.3 V switchable Pads”, p. 413 |
| P22.0‑3 buffer type = LVDS_TX | same | Table 2‑12 *Port 22 Functions*, pp. 100‑101 |
| Errata: pad driver “sm” footnote (non‑blocking) | `docs/infineon-aurix-tc39x-bd-step-erratasheet-en.pdf` | `[PORTS_TC.H018]`, §4.134 |
| TC3xx **User Manual** (PORTS/IOCR chapter) | **not in repo** | register semantics cited from iLLD + Data Sheet instead |
| AN7/AN20/AN21/AN31/AN44/AN45 → EVADC group/ch | `docs/infineon-tc39x-datasheet-en.pdf` | Analog input pin defs (LFBGA‑516), pp. 162‑165 (AN7=G0CH7, AN20=G2CH4, AN21=G2CH5, AN31=G3CH7, AN44=G8CH12, AN45=G8CH13) |
| ADC header X703/X803 pinout | `docs/infineon-triboardmanual-tc3x9-um-en.pdf` | §6.2 Fig. 6‑2, p. 6‑3 |
| ASCLIN RX pin objects | `Libraries/iLLD/.../_PinMap/TC39xB/IfxAsclin_PinMap_TC39xB_516.h` | RXA…RXH `…_IN` (P23.3=ASCLIN6, l.162; P10.4=ASCLIN11, l.126; P22.6=ASCLIN4, l.156; P20.6=ASCLIN9, l.183; P13.0=ASCLIN10, l.123) |

---

## 11. Appendix — Companion signals for the actuator (NOT part of the pin task)

Outside the original “pins for DShot” brief, but needed to actually *run* the ESCs.
These break the “one connector for everything”: the DShot lines are on **PERIPHERALS
(X702/X802)**, but the current‑sense ADC is only on the **ADC header (X703/X803)**.

> Assumption: the GOKU G45M block exposes **one shared current‑sense pad “C”** and
> **one shared telemetry pad “T”** (4‑in‑1 style). If instead each ESC has its own C/T
> pad, replicate ×4 — the pools below have enough channels (6 pre‑filtered ADC inputs,
> many free ASCLIN RX pins).

### 11.1 Pad C — current sense → EVADC

The six pre‑filtered analog inputs on X703/X803 and their EVADC mapping (Data Sheet,
LFBGA‑516 analog pin definitions):

| Conn. label | Header pin (X703/X803) | Ball | EVADC (primary) | Also / shared |
|---|---|---|---|---|
| **AN7**  | 19 | AB13 | **G0CH7**  | G11CH3 |
| AN20 | 14 | AE8  | G2CH4  | EDSADC EDS2 pin A |
| AN21 | 16 | AE7  | G2CH5  | EDSADC EDS2 pin A |
| AN31 | 36 | Y9   | G3CH7  | G4CH7 (secondary) |
| AN44 | 54 | V6   | G8CH12 | EDSADC EDS1 pin C |
| AN45 | 56 | V7   | G8CH13 | EDSADC EDS1 pin C |

**Recommendation: AN7 → EVADC `G0CH7`, header pin 19.** Clean group‑0 primary channel,
no EDSADC entanglement, matches prior project notes. For a future 4‑channel scan
(one per ESC), AN20+AN21 (`G2CH4/CH5`) are adjacent in one group for a single queued
group scan; AN44/AN45 are additionally EDSADC‑capable (delta‑sigma) if wanted.

Notes:
- **AN7 is one of only six on‑board anti‑alias‑filtered inputs** (47 nF + **4.7 kΩ**
  series; Manual §3.14, Fig. 3‑4 — the set is AN7/AN20/AN21/AN31/AN44/AN45). Scarce
  resource; it is tracked in `PINNING.md` §2.4.
- **Sample‑time caveat:** the 4.7 kΩ series R raises source impedance; set the EVADC
  channel’s sample time long enough to fully charge the internal sampling capacitor,
  otherwise the result reads **systematically low**. Uncritical at ESC‑current
  bandwidths (~tens of Hz) but must be configured in the channel register.
- **VAREF = 5 V** on this board (R220: VDDM→V_VR, R222: VAREF1→VDDM). A 0–3.3 V ESC
  current output uses only **⅔ of the converter range** — factor into scaling, or add
  a gain stage. Add a divider if the ESC full‑scale exceeds VAREF. Common ground with
  the ESC required. VAREF1 is on X703/X803 pin 40.
- EVADC analog inputs need **no port ALT config** — group+channel is selected in the
  VADC registers; the ANx pin is fixed in silicon. Driver: `IfxEvadc_Adc`
  (`Libraries/iLLD/.../Evadc`, currently `.cproject`‑excluded — see `CAN_ADC_PLAN`,
  EVADC blocked on hardware; this is a forward reservation, not an active channel).

### 11.2 Pad T — ESC telemetry → ASCLIN RX (optional, later)

BLHeli/AM32/KISS telemetry is a **one‑wire UART TX from the ESC → FC RX only** (typ.
115200 8N1). ASCLIN0 (console P14.0/1) and ASCLIN1 (LIN P15.0/1) are taken, so use a
free module whose RX lands on a free header pin.

**Recommendation: P23.3 → ASCLIN6 RXA, PERIPHERALS X702/X802 pin 24**
(`IfxAsclin6_RXA_P23_3_IN`, `IfxAsclin_PinMap_TC39xB_516.h:162`). ASCLIN6 is otherwise
unused; P23.3 is free (not in Table 4‑3 / 6‑1, no project use). RX‑only — no TX pin.

Alternatives (all free):

| RX pin | Header pin | iLLD object (.h line) | Module |
|---|---|---|---|
| P10.4 | X702/X802 · 25 | `IfxAsclin11_RXB_P10_4_IN` (126) | ASCLIN11 |
| P22.6 | X702/X802 · 13 | `IfxAsclin4_RXC_P22_6_IN` (156) | ASCLIN4 |
| P20.6 | X702/X802 · 42 | `IfxAsclin9_RXE_P20_6_IN` (183) | ASCLIN9 |
| P13.0 | X702/X802 · 31 | `IfxAsclin10_RXC_P13_0_IN` (123) | ASCLIN10 |

Notes:
- Same 5 V level story as §8: ESC telemetry TX (3.3 V) into a 5 V‑VEXT input → set the
  pad to a **TTL** driver mode (`IfxPort_PadDriver_ttlSpeed*`, V_IH = 2.0 V, §8.3) so
  3.3 V reads as high. No open‑drain needed (RX‑only; the ESC drives the line).
- Whichever pin you take here leaves the DShot §2.2 reserve pool (ample reserves remain).
