# GTM.md — Bosch GTM-IP (TC39x generation): ATOM TX / TIM RX for DShot

Source: `docs/bosch-gtm-ip-specification-v3-1-5-1.pdf` (Bosch GTM-IP
Specification, Revision 3.1.5.1, 24.03.2016, 888 pages, no embedded
outline — printed TOC scraped; printed-page offset is **constant +20**
throughout, e.g. printed p154 = pdf p174) and
`docs/bosch-gtm-cookbook-v05.pdf` (Bosch GTM Cookbook v05, 54 pages, has
an outline). Distilled 2026-09-03. Re-extract if either source revision
changes.

This is the **generic Bosch IP** chapter set — the family behavioural
document `docs/DSHOT.md` §7 flagged as missing from the repo (fields there
said "Coding see Family Spec"). The **TC39x-specific** instance counts,
clock frequencies and pin routing stay in
`docs/infineon-aurix-tc39x-usermanual-en.pdf` and `docs/PINNING.md` — this
note does not repeat those, it fills the behavioural gap between them.
Written for the reader building the DShot driver against `docs/DSHOT.md`
§7 and `docs/PINNING.md` §2.1/§2.6.

---

## 0. Verdict — which mode, up front

- **TX: ATOM SOMP (Signal Output Mode PWM)**, not SOMS. SOMP already does a
  per-period compare-value update via shadow registers with zero CPU
  involvement beyond writing `CM0`/`CM1` (or letting DMA/ARU do it) once
  per bit; SOMS (serial shift, §1.3) exists to shift a *fixed pattern*
  register out bit by bit at a fixed clock and is the wrong tool for 16
  *different* pulse widths. See §1.
- **RX: TIEM (Input Event Mode)**, not TBCM/TSSM. TBCM multiplexes
  *all channels of one TIM sub-module* into one parallel word — wrong
  shape for a single-channel serial burst. TSSM needs an explicit shift
  clock at the bit rate with no self-timing tolerance; TIEM timestamps
  every edge into GPR0/GPR1 and is exactly the primitive AM32's own
  transmit side assumes (edge-timed, not clocked). See §2.
- **Pad flip is not the bottleneck.** GTM output-enable/disable
  (`ATOM[i]_AGC_OUTEN_CTRL`) is register-write speed, i.e. GTM-clock
  cycles; `PINNING.md` §2.6 already measured the AURIX-side port flip at
  "well under 1 us" against a 30 us ESC turnaround (`DSHOT.md` §3). The
  GTM spec doesn't change that verdict; see §4.

---

## 1. ATOM (chapter 13, printed p267-357, pdf p287-377) — TX mode choice

### 1.1 Channel modes (`ATOM[i]_CH[x]_CTRL.MODE`, bits 1:0)

| `MODE` | Mode | Fit for DShot TX |
|---|---|---|
| `0b00` | SOMI (Immediate) | No — jitters up to one ARU round trip per output action, no period/duty concept |
| `0b01` | SOMC (Compare) | No — compares against TBU global time base for complex/dependent sequences, not a simple per-bit PWM |
| `0b10` | **SOMP (PWM)** | **Yes — chosen.** Native period+duty generator, see 1.2 |
| `0b11` | SOMS (Serial) | No — see 1.3, rejected |
| (SOMB is a 5th mode, register value not in the 2-bit `MODE` field, printed p326) | Buffered Compare | No — same TBU-compare shape as SOMC |

### 1.2 SOMP — the TX mechanism

`ATOM[i]_CH[x]_CM0` = period (24-bit, clock ticks), `CM1` = high-time (24-bit,
ticks) of one of 8 `CMU_CLK[0..7]` selected by 3-bit `CLK_SRC`/`CLK_SRC_SR`
(`CH[x]_CTRL` bits 14:12). `SL` (bit 11) sets output polarity. On a
**synchronous update** (`CN0` reaches `CM0-1`, continuous up-count mode,
`RST_CCU0=0`) the shadow registers `SR0`/`SR1` (also 24-bit, own registers
`ATOM[i]_CH[x]_SR0`/`SR1`) are copied into `CM0`/`CM1` and `CLK_SRC` is
updated from `CLK_SRC_SR` — spec 13.3.3, `ATOM_805`. This is the **shadow
transfer**: write `SR1` with the next bit's high-time once per 3.33 us
period, `CM0` stays constant (the period never changes within a DShot
frame), and the hardware does the transfer for free at the period boundary
— **no CPU/DMA action needed inside the period**, only one register write
per bit.

**For DShot300, `CM0` is fixed** (period = 3.33 us in ticks) for the whole
16-bit frame; only `SR1` (high time: 1.25 us for a "0" bit, 2.50 us for a
"1" bit) changes per bit. This means **plain continuous SOMP**, not
one-shot, is the natural fit — 16 writes to `SR1`, one per period, is the
whole TX job.

**Update path — CPU/DMA via AEI, not ARU (see Traps T1):** spec 13.3.3.4
"CPU controlled update" — `SR0`/`SR1` are ordinary memory-mapped registers,
writable directly over the AEI bus with `ARU_EN=0`; a DMA channel can write
`SR1` once per period the same way a CPU store would, triggered by the
`ATOM[i]_CH[x]_IRQ_NOTIFY.CCU0TC` "trigger condition" event (bit 0, fires
at the sync-update point, 13.6.17) routed to the ICM/DMA. The **ARU-fed
route also exists** (13.3.3.3: `ARU_EN=1`, period+duty in one 48-bit ARU
word, bits 23:0 = period, 47:24 = duty, requested automatically every
period) but carries a hard constraint (**GTM_AI.353**, Traps) that the AEI
route does not — **recommendation: DMA-writes-SR1-directly-via-AEI, not
ARU**, for a 3.33 us period.

**Stopping after 16 bits, idle level:** disable via
`ATOM[i]_AGC_ENDIS_CTRL.ENDIS_CTRL[x]` (2-bit/channel, `0b01` = disable on
next update trigger, 13.6.2 printed p338). With `FREEZE=0` (`CH[x]_CTRL`
bit 31, default) disabling stops `CN0` and forces the output to **`!SL`**
— program `SL=1` (idle high) so idle = high = released on the open-drain
pad (`PINNING.md` §2.6). `ENDIS_CTRL` only takes effect on the *next*
update trigger (period boundary), so arm the disable one period ahead of
bit 16, or simpler: don't disable at all, just stop reloading `SR1` after
bit 16 and let `CM1` freeze at the last value for the remainder of the
1 ms cycle — no observer reads it once RX has taken the pad.

**Output enable, separately from channel enable:** `ATOM[i]_AGC_OUTEN_CTRL`
(13.6.5, printed p341) gates the physical pad independent of `ENDIS_CTRL` —
same 2-bit-per-channel encoding. Both `ENDIS` and `OUTEN` disabled leave
`ATOM_OUT[x]` at `!SL` (13.6.5 note). This is the register a driver would
use to hand the pad back for the RX-side port-mode flip (§3), though the
actual electrical tri-state on this board comes from the **port pad mode
switch** (open-drain output -> pull-up input), not from `OUTEN` alone.

### 1.3 SOMS — why not

SOMS (13.3.4, printed p318) shifts the **raw bit pattern already in `CM1`**
out one bit per `CMU_CLK[x]` tick (`ATOM_871`: "bits shifted is `CM0`,
visible at `ATOM_OUT` is `CM0+1`") — a binary NRZ waveform, not a
duty-varying one. Encoding DShot's two duty cycles (1.25/2.50 us high in a
3.33 us slot) would need several sub-bit-resolution shift-outs per DShot
bit, chained across `CM1`'s 24-bit width — several writes per bit versus
SOMP's one. Cookbook 4.3.3.4: SOMS generates signals from a "**predefined
bit pattern**" (fixed levels), not a two-duty-cycle waveform — wrong tool.

### 1.4 GTM clock assumption — CLK0 not configured yet

`ATOM`'s `CLK_SRC` selects one of `CMU_CLK[0..7]` (13, `ATOM_1174`). Grep of
`docs/ILLD_NOTES.md` §8 and `src/bsw/gpio.c:155-157`:

```c
IfxGtm_enable(gtm);
IfxGtm_Cmu_setGclkFrequency(gtm, IfxGtm_Cmu_getModuleFrequency(gtm));
IfxGtm_Cmu_enableClocks(gtm, IFXGTM_CMU_CLKEN_FXCLK);   /* FXCLK only */
```

sets `CMU_GCLK_EN` = the GTM module's own clock (Z/N = 1, no division —
CMU chapter 8.2, `CMU_GCLK_NUM`/`DEN`) but enables **only `FXCLK`**
(feeds TOM, §2 of `ILLD_NOTES.md`). **`CMU_CLK0` — the clock ATOM/TIM need
— is not enabled anywhere in this codebase today: not configured yet.**
A DShot driver must call `IfxGtm_Cmu_enableClocks(gtm, IFXGTM_CMU_CLKEN_CLK0)`
(or `CLK1` etc.) itself before using SOMP/TIEM, and pick a `CLK_CNT`
divider (8-bit field, `CMU_CLK_[z]_CTRL`, 8.7.4) such that
`CMU_CLK[x] = CMU_GCLK_EN / (CLK_CNT+1)` lands close to the tick rate
needed for 3.33/1.25/2.50 us (SOMP) and >=3.75 MHz / 267 ns (TIM RX, §2).
**`CMU_GCLK_EN` itself is not a fixed number in this repo's sources** — it
equals whatever `IfxGtm_Cmu_getModuleFrequency()` reads from the TC39x
clock tree at runtime (SPB/GTM cluster-0 clock); that number belongs in
the TC39x appendix/clock-tree note, not here.

## 2. TIM (chapter 11, printed p154-223, pdf p174-243) — RX mode choice

### 2.1 Channel modes (`TIM[i]_CH[x]_CTRL.TIM_MODE`, bits 3:1)

| `TIM_MODE` | Mode | Fit for 21-bit GCR RX |
|---|---|---|
| `0b000` | TPWM (PWM Measurement) | No — measures one duty+period pair per cycle, wrong shape for a multi-edge burst |
| `0b001` | TPIM (Pulse Integration) | No — accumulates high/low time, not per-edge timestamps |
| **`0b010`** | **TIEM (Input Event)** | **Yes — chosen.** Timestamps every edge, see 2.2 |
| `0b011` | TIPM (Input Prescaler) | No — only counts N edges before one interrupt, throws away individual bit timing |
| `0b100` | TBCM (Bit Compression) | No — see 2.3, rejected |
| `0b101` | TGPS (Gated Periodic Sampling) | No — periodic sampling of a level, not edge capture |
| `0b110` | TSSM (Serial Shift) | No — see 2.3, rejected |

### 2.2 TIEM — the RX mechanism

Per edge (rising/falling/both, selected by `DSL`/`ISL`, `CH[x]_CTRL` bits
13/15): the edge increments `CNT` and captures `GPR0`/`GPR1` with whatever
`EGPR0_SEL`/`GPR0_SEL` (bits 9:8) selects — `TBU_TS0/1/2` (a global 24-bit
time base), `ECNT` (extended edge count), `TIM_INP_VAL`, or `CNTS` — then
raises `TIM[i]_NEWVAL[x]_IRQ` if enabled (spec 11.4.2.3, `TIM_641-650`).
**This is exactly the primitive a GCR decoder needs: a timestamp per edge**,
which is also how AM32's own TX side reasons about the signal (`DSHOT.md`
§2 — bit-boundary timing, not a fixed shift clock).

**Overflow behaviour:** if the previous `GPR0`/`GPR1` capture was not yet
consumed (by CPU or ARU) when a new edge arrives, the channel sets the
**`GPROFL`** status bit, raises `GPROFL[x]_IRQ` if enabled, and
**overwrites** the old `GPR0`/`GPR1` with the new edge — silently, unless
the decoder watches `GPROFL` (`TIM_650`). Separately, `CNT` (edge counter)
and `ECNT` (extended counter) each raise their own
`CNTOFL[x]_IRQ`/`ECNTOFL[x]_IRQ` on wraparound (`TIM_1234`/`TIM_1235`) —
three distinct overflow signals, not one.

**Timestamp clock:** `TIEM` "does not depend on the bit field `CLK_SEL`"
(`TIM_1239`) when `GPR0_SEL` uses `TBU_TSx` (the TBU global time base, its
own clock — chapter 10, not `CMU_CLK[x]`) — i.e. **edge timestamps are
free-running against the TBU, not the per-channel `CLK_SEL`**; `CLK_SEL`
only matters for modes that count *this channel's own* clock (TPWM, TGPS).
For a 375 kbit/s GCR bit (2.67 us), any `>= 3.75 MHz` (267 ns) TBU/CMU tick
is enough resolution to place edges unambiguously inside a bit slot — the
TBU channels typically run near the GTM cluster clock (rated <=200 MHz per
`ILLD_NOTES.md`/erratasheet cross-check), comfortably above that floor, but
**the exact TBU_TSx tick rate on this board is TC39x-appendix/device
config, not stated as a number in the generic spec — treat as "needs the
device clock tree", same caveat as §1.4.**

**Filters:** `FLT_EN=0` by default (`F_IN` routed straight to `F_OUT`,
11.8.1) — leave it off for GCR capture. If enabled, `FLT_CNT_FRQ` (2-bit)
picks `CMU_CLK0/1/6/7` as the de-glitch counter clock; a coarse `CLK_CNT`
there would eat a 1.3 us GCR half-bit. Spec gives the mechanism, not a
recommended threshold for 375 kbit/s — design decision, not a spec
number.

**`TIM[i]_IN_SRC` (11.8.14) is not the pad-sharing mechanism.** It
selects, per channel, between `CICTRL`'s `TIM_IN(x)`/`TIM_IN(x-1)` chain
input, a `TIM_AUX_IN` cross-module signal, or a CPU-forced constant level —
**not** an arbitrary pad. The fact that **TIM0.x and TIM7.y both see the
same physical P22.n pad** (`PINNING.md` §2.1) is fixed by the TC39x's own
device-level pin multiplexing (`IfxGtm_PinMap_TC39xB_516.h`), which lives
in the device appendix/iLLD, not in this generic `TIM[i]_IN_SRC` register —
correcting `DSHOT.md` §7's speculation on this point.

**Timeout Detection Unit (TDU, §11.3)** matches the Bosch AN023 reference
`DSHOT.md` §7 cites — AN023 itself is not in this repo. `TOCTRL`
(`CH[x]_CTRL` bits 31:30) enables a timeout on rising/falling/both edges
via a separate counter/comparator slice (11.3.1-11.3.5, registers
`TIM[i]_CH[x]_TDUV`/`TDUC`) — the mechanism to bound a GCR read that never
completes (dead ESC). A worked timeout value for 375 kbit/s is not in the
spec — **gap, needs AN023 or a bench measurement.**

### 2.3 TBCM / TSSM — why not

**TBCM** combines **all channels of one TIM sub-module** into a single
parallel word sampled on an OR of their edges (spec 11.4.2.5, `ATOM`-style
figure with `TIM_IN(0)..TIM_IN(m-1)`) — it is a multi-channel-to-one-word
bus sampler, not a single-channel serial-to-parallel shifter. Using it for
one motor's GCR reply would require routing other TIM channels' inputs
into the same word, which conflicts with those channels being independent
motor RX lines. Wrong shape, as stated in the verdict.

**TSSM** shifts the **sampled level of the input pin** into `CNT` once
per selected clock tick (spec 11.4.2.7, `TIM_3211`) — a fixed-clock
sampler, the RX-side mirror of ATOM's SOMS. It needs the receiver's clock
to already be phase-aligned to the sender's bit boundaries (no self-timing
margin), unlike TIEM's edge-timestamp approach which tolerates jitter by
construction. TSSM also carries three errata specific to chained/derived
`TSSM_OUT` use (`GTM_AI.305`, `.309`, `.346`, Traps table) — none apply to
plain TIEM.

## 3. FIFO (chapter 5, printed p84-96, pdf p104-116) + ARU/F2A/AFD routing

One FIFO RAM = 1024 words x 29 bit (24 data + 5 control) shared across
8 logical channels per FIFO instance; each channel depth is a configurable
partition of that 1024 (`START_ADDR`/`END_ADDR`), not a fixed number.
Three access ports: F2A (ARU side), AFD (AEI/CPU side), direct AEI — AFD
always wins arbitration over direct AEI (5.1, FIFO_507).

F2A ("FIFO to ARU Unit", chapter 7) bridges 8 streams, each a read
(ARU->FIFO) or write (FIFO->ARU) stream (`F2A[i]_CH[z]_STR_CFG.TMODE`). A
TIM channel with `ARU_EN=1` routed to a FIFO write stream buffers GCR edge
timestamps without an interrupt per edge — not required for one 21-edge
burst/ms (TIEM + one ISR, or a post-burst ARU drain, is simpler), but the
mechanism exists if edge rate ever outpaces CPU response.

FIFO-full behaviour is not stated in the extracted pages — gap. Only
`FULL`/`EMPTY` flags and watermark interrupts (5.3) plus DMA hysteresis
mode (5.2.3) are defined. The ARU-level rule (2.3.4, ARCH_620, upstream
of the FIFO) is explicit: a source outpacing its destination gets its old
data overwritten, new data marked valid — matching TIM's GPROFL (2.2) and
errata GTM_AI.308 (Traps).

## 4. The pad flip — GTM side is not the bottleneck

`ATOM[i]_AGC_OUTEN_CTRL`/`ENDIS_CTRL` (1.2) act "on an update trigger" —
one GTM-clock-domain register write (low tens of ns; not a number the
generic spec states). Feasibility verdict: not the bottleneck.
`PINNING.md` §2.6 already measured the AURIX-side port mode switch
(`IfxPort_setPinModeOutput` <-> `setPinModeInput`, one unprotected
`__ldmst` on Port 22) at "well under 1 us" against the ESC's ~30 us
turnaround (`DSHOT.md` §3); GTM's `OUTEN`/`ENDIS` write is a strict subset
of that. `P22_IOCR0` (offset 010H) / `P22_OMR` (004H) exist in
`infineon-aurix-tc39x-usermanual-en.pdf` ch.14.3 "Pn Registers" (pdf p236,
p264-273) — addresses only, no generic PC-field/ALT-encoding table in
this appendix (the same "appendix, not family spec" pattern as `DSHOT.md`
D7) — the iLLD `IfxPort_*` calls `PINNING.md` cites remain authoritative.

## 5. Time base / clock enable sequence at a glance

CMU (ch.8, printed p106-119, pdf p126-139). `CMU_GCLK_EN` = a fractional
(Z/N) divide of the GTM cluster-0 clock (`CMU_GCLK_NUM`/`_DEN`, 8.7.2/3).
Each `CMU_CLK[0..7]` is an independent integer divide of `GCLK_EN`:
`TCMU_CLK[x]=(CLK_CNT[x]+1)*TCMU_GCLK_EN`, `CLK_CNT` an 8-bit field in
`CMU_CLK_[z]_CTRL` (z:0-5) / `_6_CTRL`/`_7_CTRL` (8.7.4-6), changeable
only while that clock's `EN_CLK[x]` is disabled (CMU_1394). `CMU_FXCLK[y]`
(fixed /2^0..2^16 divides, TOM/MON only) is what `src/bsw/gpio.c` already
enables; ATOM/TIM need a `CMU_CLK[x]`, a separate, currently-unused
enable (1.4).

TBU (ch.10, printed p142-153, pdf p162-173) provides up to 3 free-running
24-bit global time bases `TBU_TS0/1/2` — the source TIEM's `GPR0`/`GPR1`
timestamp against (2.2), independent of `CMU_CLK[x]`.

Enable order (`IfxGtm_enable` -> `Cmu_setGclkFrequency` ->
`Cmu_enableClocks` -> per-channel init, already `ILLD_NOTES.md` §8's rule)
matches the spec: `CMU_GCLK_NUM`/`_DEN` change only while all `CMU_CLK[x]`
and `EN_FXCLK` are disabled (8.7.2, CMU_682). A DShot driver adds one
step: after enabling FXCLK, also enable the `CMU_CLK[x]` instance(s)
ATOM0/TIM0/TIM7 need, sized per 1.4/2.2, before touching
`ATOM[i]_CH[x]_CTRL`/`TIM[i]_CH[x]_CTRL`.

## 6. Cookbook (`docs/bosch-gtm-cookbook-v05.pdf`, has an outline)

§4.3.3.1.1 "Init hints for SOMP + ARU" (pdf p28): leave `CLK_SRC` at
default 000 on first enable or the first ARU word can be lost before the
first shadow transfer; if pre-selecting a clock, set `CM0=1` first.
Relevant only to the ARU-fed route, not the recommended direct-AEI one.

§4.3.3.1 confirms the feed pattern generically: "a source could either be
a FIFO, where the CPU has to calculate a bunch of PWM characteristics and
store them in FIFO, or a MCS" — pre-compute the 16 `SR1` values once (two
duty constants, 16 lookups), hardware steps through them.

§4.3.5.3 "ATOM SOMP OSM" (pdf p33): worked C one-shot recipe — force
`CM1<=CN0<=CM0`, then write real `CM0`/`CM1`/`CN0` (`CN0=pulseLength+1`
triggers the pulse). Confirms the write-to-`CN0`-triggers mechanism with
code, useful only for a per-bit-retrigger design — not recommended here
(continuous SOMP: one write/bit; one-shot-retrigger: 3 writes/bit and
walks into GTM_AI.298/.299/.340/.341, Traps).

No cookbook recipe matches "capture a fast edge burst on TIM" — its TIM
coverage (§4.1) is the same 5-mode summary as the spec, no TIEM
timing-diagram worked example.

## 7. Traps

| # | Trap |
|---|---|
| T1 | **GTM_AI.353** (erratasheet v2.7, pdf p102): with `ARU_EN=1` in SOMP, a PWM period shorter than "ARU round trip time + 3 ARU clock cycles" loses new period/duty values silently, running the stale value on. TC3xx workaround: period > round-trip + 3 clocks, or reduce `ARU_CADDR_END`. At DShot300's 3.33 us period this is a real risk if ARU round-trip is not verified short enough — **use the direct-AEI DMA route (1.2) to sidestep it entirely.** |
| T2 | Shadow transfer only happens at the *next* period boundary (13.3.3, ATOM_805) — a `SR1` write during the last few clocks of a period may or may not make that boundary; budget one extra period of slack when the DMA trigger is `CCU0TC`. |
| T3 | `GTM_AI.320`: ATOM SOMS one-shot cycle restarts unexpectedly if `CM0` is written to 0 while `SR0` is nonzero — moot here (SOMS rejected, §1.3) but a trap if a future change revisits SOMS. |
| T4 | `GTM_AI.298`/`.299`/`.340`/`.341`: SOMP one-shot mode has four separate documented bugs around `TRIG_CCU0`/`TRIG_CCU1` generation when the one-shot pulse is triggered by `TIM_EXT_CAPTURE(x)` or `trig_[x-1]`, or when `OSM_TRIG=1` with `CM1=1` — **all avoided by the recommended continuous-SOMP, no-external-trigger design (1.2).** |
| T5 | `GTM_AI.352`: ATOM reload from ARU is wrong in SOMS/SOMP if `TIM_EXT_CAPTURE(x)` or `TRIGIN(x)` is the clock source — same avoidance as T4: use a free-running `CMU_CLK[x]`, not an external trigger, as the SOMP clock source. |
| T6 | `GTM_AI.308`: back-to-back TIM data transfers at full ARU clock rate can lose every second transfer under ARU dynamic routing. GCR RX at 375 kbit/s is far below "full ARU clock rate" — likely not triggered, but the actual ARU clock divider should be checked before trusting dynamic routing for the capture path. |
| T7 | `GTM_AI.305`/`.309`/`.346`: three TSSM-specific errata (signal delay into a chained channel's filter, unpredictable `TSSM_OUT` delay, shift cycle not executed correctly when `UPEN=0`) — moot here (TSSM rejected, §2.3) but confirm the RX mode choice stays off TSSM if this is ever revisited. |
| T8 | TIM `GPROFL` overwrite (2.2) and the FIFO/ARU overwrite-on-overrun (3) are the *same* failure mode at two layers — a decoder that only checks one will still lose data silently at the other if the consumer (CPU or DMA) falls behind. |
| T9 | `TIM_MODE` "should not be changed while the TIM channel is enabled" (11.8.1) — reconfiguring RX mode requires `TIM_EN=0` first, same as ATOM's "reset the channel first" rule for changing `MODE` (cookbook 4.3.3). |
| T10 | Filter defaults are safe (`FLT_EN=0` on reset, 2.2) but any future enabling of the TIM filter for noise rejection must size `FLT_CNT_FRQ`'s clock well above 375 kbit/s or it will eat GCR half-bits — no spec-given threshold, must be measured. |

## 8. Gaps

- GTM clock frequency numbers (actual Hz for `CMU_GCLK_EN`, TBU tick
  rate, a concrete `CLK_CNT` divider) are TC39x-device-config, not in
  either Bosch document — needs the clock-tree note or a bench read via
  `IfxGtm_Cmu_getModuleFrequency()`.
- ARU round trip time for this device's GTM instance (needed to evaluate
  GTM_AI.353, Traps T1) is "device specific Appendix B" per the spec
  (2.3.3) — not found in the extracted pages; not critical since the
  recommended TX design avoids ARU entirely (1.2).
- TDU timeout value for 375 kbit/s — register mechanism is in the spec
  (2.2), the recommended value is in Bosch AN023, not in this repo.
- FIFO-full write behaviour (3) — not stated in the extracted pages.
- `P22_IOCR0` bit-field meaning (PC/ALT encoding) — the appendix in this
  repo lists only the address, not the generic table; `PINNING.md`'s
  iLLD-level description remains the authoritative source (4).
- Everything needing a scope/logic-analyzer trace (actual ARU round-trip
  cycles, actual pad flip time, actual GTM_AI.353 margin at 3.33 us) is a
  bench task for flight-dev, not something this note can settle from
  documents alone.

## Cross-references

- `docs/DSHOT.md` §7 — the protocol-level numbers (bit timing, GCR
  table, turnaround budget) this note's register-level answers slot into.
- `docs/PINNING.md` §2.1 (pin map), §2.6 (single-pin bidirectional
  scheme, pad-flip electrical/timing) — this note supplies the GTM
  register mechanism those sections assume.
- `docs/ILLD_NOTES.md` §8 (`IfxGtm` — TOM PWM today; extend with an ATOM
  section once the DShot driver is written, per that file's own rule).
- `docs/infineon-aurix-tc39x-bd-step-erratasheet-en.pdf` v2.7 — errata
  ids in §7 Traps are quoted from this file; re-check on any revision
  bump.
- `QuadSE/requirements/SYS2/SYS2-ACT-001` (motor interface) — the
  flight-architect DShot driver design task this note was written to
  unblock.
