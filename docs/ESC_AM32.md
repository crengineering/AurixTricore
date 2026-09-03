# ESC_AM32.md — Flywoo GOKU G55M (AM32/AT32F421) integration facts

Source: **AM32 firmware** github.com/am32-firmware/AM32, commit `6b3ef3d`
(2026-05-08) - `Inc/targets.h` (`FLYWOO_GOKU_F421` target block),
`Inc/eeprom.h`, `Mcu/f421/Src/IO.c`, `Mcu/f421/Src/serial_telemetry.c`,
`README.md` - fetched 2026-09-03. **Artery AT32F421 datasheet** (GPIO 5V
tolerance), via WebSearch summary, fetched 2026-09-03 - not pulled as a raw
PDF, treat the tolerance claim as secondary-source (Gaps). ESC commercial
facts: `QuadSE/procurement/datasheets/esc-flywoo-goku-g55m.md` (2026-08-30),
`QuadSE/dispatch/SYS1-012 — Dispatch.md` §4b (2026-08-30, board photo pad
row read). AURIX side: `docs/PINNING.md` §2.1/§2.4/§2.6 (2026-09-03 state,
before this file's audit — see the PINNING.md diff for what changed).
Protocol facts this file assumes are sourced in `docs/DSHOT.md` — read that
first for anything about frame timing, telemetry format, or GTM mapping.

---

## 1. Pad map — G V 1 2 3 4 C T → AURIX

| Pad | Meaning | Wired to AURIX? | Pin / header | Status |
|---|---|---|---|---|
| G | ESC ground | Yes — common return | X702 GND pins (1-4, 37, 38, 41, 43, 44, 61, 62, 76) | must share ground with the AURIX board |
| V | **ESC VBAT pass-through/sense pad** — the AM32 4-in-1 board's own regulator/sense tap on the pack rail | **No — never wire this to any AURIX pin** | — | see §2 |
| 1 | M1 signal, single-pin bidirectional DShot300 | Yes | P22.1 / ATOM0.0 (TOUT48) / TIM0.0 or TIM7.2 | X702·30, `PINNING.md` §2.1 |
| 2 | M2 signal | Yes | P22.0 / ATOM0.1 (TOUT47) / TIM0.1 or TIM7.3 | X702·32 |
| 3 | M3 signal | Yes | P22.2 / ATOM0.3 (TOUT49) / TIM0.3 or TIM7.1 | X702·36 — **LVDS_TX/HSCT footprint stub, higher risk, PINNING.md §2.6/§4** |
| 4 | M4 signal | Yes | P22.3 / ATOM0.4 (TOUT50) / TIM0.4 or TIM7.0 | X702·34 — **same LVDS_TX/HSCT stub risk** |
| C | Analog current sense, ESC's own shunt/amp output | Yes | AN7 / EVADC G0CH7 | X703·19, `PINNING.md` §2.4 |
| T | KISS/telemetry UART TX, shared across all 4 channels (`docs/DSHOT.md` §6) | Yes | P23.3 / ASCLIN6 RXA | X702·24, `PINNING.md` §2.4 |

**Verdict on the existing pin map: correct, nothing to fix.** `PINNING.md`
§2.1/§2.4/§2.6 already commits to exactly this pad assignment and the
Flywoo board photo confirms the `G V 1 2 3 4 C T` row
(`esc-flywoo-goku-g55m.md`). The one gap is that **PINNING.md never states
that G/V exist and V must not be wired** — added below, marked
"2026-09-03 audit".

---

## 2. Electrical, pad by pad

**G — ground.** Common return with the AURIX board and the battery. No
open question.

**V — VBAT pass-through, NOT an AURIX signal.** On AM32 4-in-1 boards this
pad exposes the pack rail (here: 4S, up to ~16.8 V charged) directly or
through a light sense divider — it exists so a flight controller that wants
a battery-voltage tap can use it. **This project does not use it**: pack
voltage comes from the KISS telemetry frame instead (`docs/DSHOT.md` §6;
decided 2026-09-03, `dispatch/SYS1-012.md` §10). Landing 16.8 V on any
AURIX pin (all of them 5 V VEXT domain at most, `PINNING.md` board-wide
note) would exceed every pad's absolute maximum — **leave V unconnected on
the AURIX side, full stop.** This is exactly the class of mistake the
flash-safety gate's "newly driven pin" language exists to catch (§3).

**1–4 — DShot signal, AT32F421 GPIO, 5 V tolerant.** The AT32F421's GPIO
pins are documented as **5 V-tolerant "FT" pads** (all I/O except the 4
OSC/OSC32 pins) per the Artery AT32F421 datasheet (WebSearch summary,
2026-09-03 — see Gaps for the caveat on this source). This matters for the
opposite direction from what PINNING.md's divider analysis covers: the
AURIX drives the line in open-drain (never sources above the pull-up
rail), so the ESC input never sees more than 3.3 V from the AURIX side —
but it is reassuring that even a fault condition putting 5 V on this pad
would not be outside the AT32F421's own absolute maximum.

**Drive type during the ESC's turn to talk (bidirectional reply):** the
fetched AM32 source (`Mcu/f421/Src/IO.c`) reconfigures the **timer**
capture/compare mode and DMA direction on the `changeToOutput()`/
`changeToInput()` flip, but the GPIO **output-type register** (push-pull vs
open-drain) for the DShot signal pin itself is not written in that file —
it is set once at pin init, in a target-specific setup file this fetch did
not retrieve. **Indirect evidence it is push-pull, not open-drain:** the
telemetry UART pin on the same MCU family is explicitly configured
`GPIO_OUTPUT_PUSH_PULL` (`serial_telemetry.c:29`), and AM32/BLHeli_32 ESCs
generally drive DShot signal pins actively (push-pull) in both directions
because the pin sits at the end of a short PCB trace, not a shared
multi-drop bus — open-drain is this project's own choice on the *AURIX*
side (§2.6), driven by the AURIX pad's 5 V VEXT domain, not a requirement
imposed by the ESC. **Not proven from source — flag as Gap, verify on
arrival with a scope/meter per §5.**

**Does the 1.5 kΩ pull-up hold against the AT32F421's drive strength?**
Not the right question if the ESC drives push-pull (see above) — during
the ESC's transmit phase the pull-up is irrelevant, the ESC's own driver
sets the level. The pull-up only has to work in the genuine idle case
(both ends Hi-Z): 3.3 V ÷ 1.5 kΩ = 2.2 mA of pull-up current against pad
leakage/pull-down currents in the tens-of-µA range (TC39x datasheet Table
3-7, `IPDL` ≤130 µA typ for the AURIX's own fast-GPIO pad class) — three
orders of magnitude of margin, not a close call.

**Does the 1.5 kΩ pull-up keep up with the ~375 kbit/s (2.67 µs/bit) GCR
edge rate over a ~30 cm lead?** Only relevant on the AURIX side's own
rising edges out of open-drain (the ESC's push-pull drive has its own,
much faster edge rate, driven by the AT32F421 pad, not the pull-up). RC
time constant for the AURIX pull-up path: 1.5 kΩ × (a generous 50 pF for
pad + 30 cm lead + ESC pad capacitance) ≈ **75 ns** — under 3% of one GCR
bit period. **Not a bottleneck.**

**C — current sense, 0–3.3 V into AN7.** VAREF = 5 V on this board
(`PINNING.md` §2.4), so a 0–3.3 V ESC output uses ⅔ of the ADC's range —
already noted in PINNING.md, unchanged. **Scale in mV/A: UNKNOWN, not in
AM32 source** (no eeprom field for it — `Inc/eeprom.h` has no
current-scale/offset member, §4) — **must be read from the AM32
configurator on arrival**, exactly as the existing procurement note says.

**T — telemetry, 115200 8N1, half-duplex, push-pull with pull-up on the ESC
side.** Confirmed by source (`docs/DSHOT.md` §6) to be exactly the pin
map's assumption: P23.3/ASCLIN6 RX ← ESC TX. **Confirmed: shared across all
four channels, one physical wire** (`docs/DSHOT.md` §6) — the FC must
round-robin which channel's telemetry bit it sets, never two at once.

---

## 3. Hardware-safety consequences (flash-safety gate, `PLAN-001-agents.md` §4)

**Everything in this ESC's signal path is a "newly driven pin" and a
"power/actuation path" change** — under the flash-safety gate, **any
firmware that drives P22.0/1/2/3, reads AN7, or reads P23.3 in a new way
needs Chris's explicit OK before an agent flashes it**, independent of
whether the change looks logically safe. This was already true before this
audit; nothing here loosens it.

**Can the ESC back-feed the TriBoard?**

- **V pad, if ever mistakenly wired:** yes, catastrophically — pack voltage
  (up to ~16.8 V) directly onto an AURIX pad. Mitigation: don't wire it
  (§2). There is no legitimate reason for this project's design to connect
  V anywhere.
- **DShot lines 1–4 with the ESC powered and the AURIX board powered
  down:** the ESC's push-pull driver (if that is confirmed, §2) would drive
  up to ~3.3 V into an unpowered AURIX pad. Most TC39x pads tolerate this
  as a normal input-diode-clamp condition at low current, but **an
  unpowered AURIX board should not be connected to a powered ESC signal
  bus for extended periods** — normal bring-up hygiene, not a new finding.
  The pull-up (§2.1, 1.5 kΩ to the AURIX's own 3.3 V rail) is powered from
  the AURIX side; with the AURIX off, that rail is also off, so the pull-up
  itself sources nothing extra.
- **Current-sense pad (C) with the ESC powered, AURIX off:** AN7 would see
  whatever 0–3.3 V analog level the ESC's sense amp outputs while the
  ESC has pack power — again, an unpowered ADC pin seeing an active analog
  source is a normal condition MCUs tolerate at the pad's clamp-diode
  current rating, not a design flaw, but is exactly why the flash-safety
  gate treats "powered-test lockout" (`PLAN-001-agents.md` §4.1, "the day
  DShot/arming lands, this rule hardens") as the trigger event it is.
- **Telemetry pad (T) with the ESC powered, AURIX off:** same category —
  the ESC's idle-high push-pull UART TX line presents 3.3 V into an
  unpowered RX pad. Same tolerance argument, same "don't leave it that way"
  hygiene rule.

**What the XT90-S kill stage (SYS1-005) means for ESC power-on/arming
state:** the XT90-S is the pack-side kill switch — with it open, the ESC
has no power at all and none of the above back-feed paths exist (the ESC
MCU itself is unpowered, cannot drive anything). This is the actually
load-bearing safety property: **as long as the XT90-S stays open during any
bench work that is not an explicitly authorised powered test, the entire
back-feed question above is moot.** Combined with AM32's own 500 ms/1 s
arm-and-signal-loss behaviour (`docs/DSHOT.md` §5), there are now two
independent layers before the motors can spin: the physical kill switch
(pack side, always available, no software involved) and AM32's own
1-second sustained-zero-throttle arm sequence (ESC side, requires the FC to
behave correctly for a full second before it will spin at all). SYS2-SAF-003
("kill chain testable without props") is satisfied by testing the XT90-S
layer alone, with the props off, at any point — it does not need the DShot
driver to exist first for that specific requirement, only for the "software
disarm over XCP" layer (SYS2-SAF-001) which is a third, independent layer
still to be designed.

---

## 4. AM32 configuration facts relevant to the driver

- **Protocol autodetect:** AM32 does not need to be told which DShot rate
  is coming — it measures the frame and classifies it (`docs/DSHOT.md`
  §2). No FC-side "tell the ESC what protocol" step exists.
- **Bidirectional DShot enable:** `eepromBuffer.bi_direction` (`Inc/eeprom.h`
  offset 18), toggled by DShot commands 9 (off)/10 (on) — see
  `docs/DSHOT.md` §4/Traps D2. **Also auto-detected from idle line level**
  independent of this eeprom flag (`docs/DSHOT.md` §1) — the eeprom bit
  appears to gate whether AM32 *offers* a reply, not whether it *notices*
  the FC wants one; not fully disambiguated from source alone. Read the
  actual setting from the AM32 configurator on arrival (§5) rather than
  assuming it defaults to on.
- **PWM frequency:** `eepromBuffer.pwm_frequency` (offset 24), motor-drive
  PWM (24–128 kHz per the ESC's spec sheet), unrelated to the DShot signal
  rate — do not confuse the two.
- **Current-sense scale/offset:** **not an eeprom field at all** — no
  member in `Inc/eeprom.h` for it. Either it is compiled into the firmware
  per hardware target (a `HARDWARE_GROUP_AT_*` macro this fetch did not
  fully resolve) or the configurator computes/displays it without storing
  a raw scale the FC can read. **Read the number from the AM32
  configurator's current-sense display on arrival — do not assume a
  linear formula without it.**
- **Telemetry baud:** fixed at 115200 8N1 in the fetched source
  (`serial_telemetry.c:56`), not an eeprom-configurable field found.
- **Target confirmed:** `Inc/targets.h` has a `FLYWOO_GOKU_F421` block
  (`DEAD_TIME 50`, `HARDWARE_GROUP_AT_B`, `HARDWARE_GROUP_AT_540`,
  `USE_SERIAL_TELEMETRY`) — this is the exact target for the GOKU G-series
  4-in-1 boards, strong corroboration that the ESC ships genuine AM32 with
  serial telemetry enabled by default.

---

## 5. Chris's on-arrival checklist (multimeter, before anything is wired)

1. **Pad functions** — confirm the silkscreen row reads `G V 1 2 3 4 C T`
   exactly as photographed (`esc-flywoo-goku-g55m.md`); a running board
   change could differ from the product photo.
2. **V pad voltage, ESC unpowered:** should read open/floating or a few kΩ
   to G (whatever the board's own bleed/sense network is) — **do not probe
   this with the pack connected and a meter set to anything but voltage.**
3. **V pad voltage, ESC powered (pack connected, meter only, nothing else
   wired):** expect close to pack voltage (~16.8 V at full charge, 4S) —
   confirms it really is the VBAT tap and reinforces why it must stay
   unconnected to the AURIX.
4. **Idle logic level on pads 1–4, ESC powered, nothing else connected:**
   expect a steady level (not toggling) — bidirectional DShot's idle-high
   convention (`docs/DSHOT.md` §1) means this should read close to the
   ESC's own logic high (~3.3 V) if AM32 defaults to bidirectional-ready,
   or could read low/floating if it defaults to unidirectional. **Record
   what is actually measured — it settles the D1 auto-detect question from
   the FC side's perspective before any signal is ever sent.**
5. **Telemetry pad (T) idle level, ESC powered:** UART TX idle-high is the
   UART convention — expect close to 3.3 V, steady, per §2's push-pull
   claim. A pad that reads 0 V idle would falsify that claim outright.
6. **Current-sense pad (C) idle voltage, ESC powered, zero current draw (no
   motor spinning):** expect close to 0 V or a small fixed offset (many
   sense amps have a non-zero zero-current bias) — **record the exact
   idle voltage, it is the offset half of the mV/A scale question.**
7. **Firmware version string** — read via the AM32 configurator
   (am32.ca or the desktop tool, `README.md`) — confirms AM32 (not
   BLHeli_32) and the exact version, since this note's page/line citations
   are pinned to commit `6b3ef3d` (2026-05-08) and a materially older or
   newer firmware on the physical board could differ in any of the numbers
   above.
8. **Current-sense scale** — read directly from the configurator's current
   display against a known load, or from its settings page if it exposes
   a raw mV/A figure. This is the one number nothing in source code
   supplies (§4).
9. **Bidirectional DShot setting** — read the configurator's protocol page
   to confirm whether bidirectional is on by default on this specific
   firmware build, rather than relying on the auto-detect behaviour alone.

---

## Cross-references

- `docs/DSHOT.md` — protocol reference this file assumes throughout.
- `docs/PINNING.md` §2.1 (DShot pins), §2.4 (current sense + telemetry),
  §2.6 (electrical/drive scheme) — audited 2026-09-03, see the diff for the
  one addition (G/V pad note).
- `QuadSE/plans/PLAN-001-agents.md` §4 — flash-safety gate, board
  discipline, powered-test lockout.
- `QuadSE/requirements/SYS2/SYS2-ACT-001`, `SYS2-PWR-001`, `SYS2-SAF-001/-002/-003`.
- `QuadSE/procurement/datasheets/esc-flywoo-goku-g55m.md` — commercial
  facts, pad photo, gaps this file closes (protocol/telemetry/current-sense
  authority) and the one gap it does not (current-sense scale).

## Gaps

- **DShot signal pin GPIO output type (push-pull vs open-drain) on the
  AT32F421 side** — inferred from the telemetry UART pin's config and
  general AM32/BLHeli_32 practice, not read directly from a DShot-pin GPIO
  init call in the fetched source. Verify with a scope/meter per §5 item 4
  (a push-pull reply will show a hard, fast-edged waveform; open-drain
  would show pull-up-limited rise times only on the ESC's own high edges,
  which would be unusual for this MCU family).
- **AT32F421 5 V-tolerance claim** — sourced to a WebSearch summary of the
  Artery datasheet, not a directly fetched/read PDF. Treat as
  high-confidence but not independently re-verified against the primary
  document; re-check if this fact ever becomes safety-load-bearing (it
  currently is not — the AURIX-side open-drain scheme means the AURIX
  never sources above 3.3 V toward the ESC regardless).
- **Current-sense scale (mV/A) and zero-current offset** — genuinely not
  in AM32 source; must come from the physical board + configurator. Not a
  documentation gap, a hardware-in-hand gap.
- **Bidirectional-DShot-on-by-default question** — the interaction between
  the `bi_direction` eeprom flag and the idle-line auto-detect (`docs/DSHOT.md`
  Traps D1) is not fully resolved from source; the checklist item 4/9
  above is how it gets settled on arrival.
- **Exact GPIO pin letter/number for the DShot and telemetry pins on the
  physical GOKU G55M board** — `Mcu/f421/Src/serial_telemetry.c` shows
  `GPIOB` pin 6 for telemetry in the generic target file, but the actual
  `FLYWOO_GOKU_F421` target's specific pin assignment (which of the AT32F421's
  physical pins map to Pad T, Pad 1-4) is defined in board-specific header
  content this fetch did not retrieve. **Irrelevant to the AURIX side** (the
  AURIX only cares about its own pins, already correct per §1) — noted only
  because it means this file cannot independently cross-check "does Pad T
  really carry the KISS UART on *this* board" beyond the `USE_SERIAL_TELEMETRY`
  target flag and the board-photo pad labelling already in the procurement note.
