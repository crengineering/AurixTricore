# Implementation Plan — MCMCAN and EVADC

Status: **planned, not started** (2026-07-15). Hardware validation blocked until
test hardware is available; code can be written and build/MISRA-verified before that.

Order: **Phase 1 (MCMCAN) first**, then Phase 2 (EVADC). Each phase is its own
feature branch and ends with: build `0 errors, 0 warnings`, MISRA clean
(no new baseline entries), hardware validation, docs, version bump — merged
before the next phase starts.

---

## Board facts (from docs/infineon-triboardmanual-tc3x9-um-en.pdf)

- **CAN0 header (X301)** → CAN module 0, node 0: TX = P20.8 (`TXDCAN0`),
  RX = P20.7 (`RXDCAN0B`), via TLE9251VSJ FD-capable transceiver.
  **120 Ω termination is populated on-board** (R307), so a PEAK adapter on a
  short cable works directly.
  PEAK DB9 side: CAN_H = pin 7, CAN_L = pin 2, GND = pin 3.
  Board IDC10 pinout: manual Fig. 6-7.
- Second header **X302 ("CAN1")** → CAN module 1 node 0: TX = P23.1
  (`TXDCAN4`), RX = P23.0 (`RXDCAN4C`). Reserve for a two-node loopback later.
- **Analog inputs with on-board RC low-pass filter**: AN7, AN20, AN21, AN31,
  AN44, AN45 — brought out on header **X405** (manual p. 15).
  First channel: **AN7 = EVADC group 0, channel 7**.
- VAREF on the TriBoard is the 5 V rail (verify on hardware).

## Build-system prerequisite (both phases)

The iLLD sources are **excluded from the build** in `.cproject`
(TriCore Debug (TASKING) config, `<entry excluding=...>` list):

- Phase 1: remove `Libraries/iLLD/TC3xx/Tricore/Can/Can` and `.../Can/Std`
- Phase 2: remove `Libraries/iLLD/TC3xx/Tricore/Evadc/Adc` and `.../Evadc/Std`

Same procedure as the GTM dirs for the PWM feature. Use root-relative
includes (`#include "Can/Can/IfxCan_Can.h"`).

---

## Phase 1 — MCMCAN (branch `feature/can_implementation`)

1. **Build enablement** — un-exclude `Can/Can` + `Can/Std` (see above).
2. **New BSW module `src/bsw/Can.c/.h` + `Can_cfg.h`** (gpio-module pattern):
   - CAN module 0, node 0, pins `IfxCan_TXD00_P20_8_OUT` /
     `IfxCan_RXD00B_P20_7_IN`.
   - **Classic CAN, 500 kbit/s** first. Check the PEAK model before adding FD:
     plain PCAN-USB is classic-only; only PCAN-USB FD can do CAN FD.
   - Message RAM: one TX FIFO, RX FIFO 0 with accept-all filter into FIFO 0.
   - **No interrupts** — polled from the scheduler, matching the current
     architecture (the lwIP tick stays the only ISR).
3. **TX path** — 100 ms status frame, ID 0x100, 8 bytes (tick, die temp,
   diagStatus), sent from `Task_Measure100ms` reusing `measurementsUpdate()`
   data.
4. **RX path** — drain RX FIFO 0 in `Task_App10ms` (currently an empty TODO).
   Command frame ID 0x200 sets a P00 pin / PWM duty via the existing gpio API
   to prove reception end-to-end.
5. **Diagnostics** — read node error counters / bus-off flag, add a
   `DIAG_CAN_BUSOFF` bit in `Diagnostics.c`, document in `docs/DIAGNOSTICS.md`.
6. **Validation (needs hardware)** — new `tools/can_test.py` using
   `python-can` with `interface='pcan'`:
   - assert the 0x100 frame arrives at 10 Hz with sane content,
   - send 0x200, confirm the LED/PWM reacts,
   - UART log as secondary evidence.
7. Optional later: CAN FD (if adapter supports it), X302 as second node,
   CAN rx/tx counters in `Xcp_Data`.

### Phase 1 risks

- MCMCAN message-RAM/filter config is the fiddliest part of the iLLD CAN
  driver — wrong section offsets silently drop frames. Start from the iLLD
  MCMCAN example config.
- Verify the CAN module clock (fCAN ≠ 0) after module init before debugging
  "no traffic".

---

## Phase 2 — EVADC (branch `feature/adc_implementation`)

1. **Build enablement** — un-exclude `Evadc/Adc` + `Evadc/Std`.
2. **New BSW module `src/bsw/Adc.c/.h`** — group 0, channel 7 (AN7 on X405):
   queued request source, 12-bit, software-triggered from the 100 ms task,
   polled result readout. No interrupts, no DMA.
3. **Scaling** — counts → volts against VAREF (5 V, verify on hardware).
   Full-scale factor goes into `Xcp_Cal` next to `fsVdd`/`fsVddp3`/`fsVext`
   so it stays XCP-tunable.
4. **XCP integration** — extend `Xcp_Data` with `uint16 rawAn7` (+ pad for
   4-byte alignment) and `float32 an7Volts`; update the block comment in
   `Measurements.h`, `docs/AurixTricore.a2l`, `tools/xcp_test.py`.
   Bump `Version.h`; after header changes do a clean build (amk reuses stale
   `.src` intermediates — delete `.src`+`.o`+`.d` or full clean) and verify
   the version via XCP.
5. **Diagnostics (optional)** — over/under-voltage threshold on AN7 → diag bit.
6. **Validation (needs hardware)** — jumper AN7 to GND / 3.3 V / VAREF,
   confirm raw counts ≈ 0 / mid / full-scale via pyXCP.
   Stretch: add the channel to a 10 ms DAQ list and watch a live waveform.
