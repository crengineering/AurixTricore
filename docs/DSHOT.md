# DSHOT.md — DShot protocol reference (300, bidirectional, AM32)

Source: **AM32 firmware** github.com/am32-firmware/AM32, commit `6b3ef3d`
(2026-05-08) - `Src/dshot.c`, `Src/signal.c`, `Src/kiss_telemetry.c`,
`Src/functions.c`, `Src/main.c`, `Inc/dshot.h`, `Inc/kiss_telemetry.h`,
`Inc/eeprom.h`, `Inc/targets.h`, `Mcu/f421/Src/IO.c`,
`Mcu/f421/Src/serial_telemetry.c`, `README.md` - fetched 2026-09-03 via raw
GitHub. **Betaflight wiki** `DSHOT-ESC-Protocol` and
`Bidirectional-DSHOT-and-RPM-Filter` (github.com/betaflight/betaflight/wiki,
fetched 2026-09-03 - wiki pages render as the repo root over the fetch path
used here, so treated as corroboration via a search-result quote, not a
direct page fetch - see Gaps). **brushlesswhoop.com "DSHOT - the missing
Handbook"**, fetched 2026-09-03 (independent write-up, widely cited,
agrees with AM32 source on every figure checked). AURIX side: AURIX
TC39x-B User's Manual Appendix `docs/infineon-aurix-tc39x-usermanual-en.pdf`
Section 26 (GTM), pdf p506-921, and
`docs/infineon-aurix-tc39x-bd-step-erratasheet-en.pdf` v2.7 (2025-12-10).
Re-extract if any source revision changes.

Every number below is cited to one of the two independent sources (AM32
source code, or the Betaflight/brushlesswhoop write-ups) - where they agree
that is stated; where only one source has a figure, that is stated too.
**Nothing here is invented.**

---

## 1. DShot frame - 16 bits

| Field | Bits | Meaning | Source |
|---|---|---|---|
| Throttle | 15:5 (11 bits, MSB first) | `0` = disarmed/stop, `1-47` = special commands, `48-2047` = throttle (2000 steps) | AM32 `dshot.c:102` (`tocheck` built from `dpulse[0..10]`); brushlesswhoop |
| Telemetry request | bit 4 (`dpulse[11]`) | `1` = this ESC must reply with a KISS/EDT frame after this frame | AM32 `dshot.c:107-109` |
| CRC | bits 3:0 (`dpulse[12..15]`) | 4-bit checksum, XOR of the three preceding nibbles | AM32 `dshot.c:84-85` |

**CRC (command direction, uninverted):**
```
calcCRC = (b0^b4^b8)<<3 | (b1^b5^b9)<<2 | (b2^b6^b10)<<1 | (b3^b7^b11)
```
`dshot.c:84`. This is XOR of the throttle+telem nibbles taken 3 at a time
(bit i of nibble 0 XOR bit i of nibble 1 XOR bit i of nibble 2), **not** a
CRC-4 polynomial - "CRC" is the DShot spec's own name for it.

**CRC (bidirectional / inverted mode):** `checkCRC = ~checkCRC + 16` before
comparison (`dshot.c:99`) - equivalent to bitwise-inverting the received
4-bit CRC and masking to 4 bits. AM32 self-detects inversion rather than
trusting a config bit: while disarmed, if the idle line reads **high**
between pulses for >100 consecutive samples it sets `dshot_telemetry = 1`
and switches to inverted-CRC checking (`dshot.c:87-97`, `high_pin_count`).
This is the **normal-DShot-idles-low / bidirectional-DShot-idles-high**
convention, detected automatically, not configured.

---

## 2. Bit timing - DShot150/300/600/1200

| Variant | Bit rate | Bit period | T1H | T0H | ~Frame period (16 bit) |
|---|---|---|---|---|---|
| DShot150 | 150 kbit/s | 6.67 us | 5.00 us | 2.50 us | ~106.7 us |
| **DShot300** | **300 kbit/s** | **3.33 us** | **2.50 us** | **1.25 us** | **~53.3 us** |
| DShot600 | 600 kbit/s | 1.67 us | 1.25 us | 0.625 us | ~26.7 us |
| DShot1200 | 1200 kbit/s | 0.83 us | 0.625 us | 0.313 us | ~13.3 us |

Source: brushlesswhoop.com (cross-checked against the well-known Betaflight
figures; DShot150/600/1200 are exact ratio scalings of DShot300 - 2x/half x/quarter x
bit period - stated explicitly on that page). **AM32 itself does not hard-code
these numbers** - `signal.c` measures the actual frame time on the wire
(`dma_buffer[31]-dma_buffer[0]`) over 8 packets and derives
`dshot_frametime_low/high` from the *measured* average
(`signal.c:167-173`), then auto-classifies the rate via relative input-capture
thresholds in `checkDshot()` (`signal.c:201-227`) rather than assuming a
protocol. **Practical consequence: AM32 does not care whether the FC's bit
timing is exactly 3.33 us - it self-calibrates to whatever period it
measures**, so a modest jitter/tolerance on the AURIX TX side is absorbed;
it does not relax the *nominal* T0H/T1H split within one frame, which must
still be unambiguous (T1H clearly above, T0H clearly below,
`halfpulsetime = dshot_frametime>>5`, i.e. the ESC's own decision threshold
is the mid-point of the measured average, `signal.c:74-83`, `dshot.c:82`).

**Minimum idle / frame gap:** not a hard-coded constant in AM32 either - the
ESC re-arms its input-capture DMA (`receiveDshotDma()`) immediately after
each processed frame and is ready for the next one; the practical
lower bound is whatever the FC's driver produces between frames.
brushlesswhoop states >=2 us gap for DShot600; not independently confirmed
in AM32 source - **treat as an unverified secondary-source figure**, not a
hard AM32 requirement.

---

## 3. Bidirectional DShot - eRPM telemetry reply

**Turnaround.** After a frame with the telemetry bit set, AM32 flips its
own signal pin from output to input-capture and the FC must do the mirror
flip (output to input) to receive the reply. brushlesswhoop states a **30 us**
break for "line, DMA and timer" switching. AM32's own turnaround is a
handful of register writes (`changeToInput()`/`changeToOutput()` in
`Mcu/f421/Src/IO.c:21-44` - timer reset, capture/compare mode swap, DMA
direction swap; no explicit delay). **Budget 30 us on the AURIX RX-arm side
as the governing number** (ESC-side turnaround, not AURIX-side - the AURIX
pad flip itself is sub-us, PINNING.md Section 2.6).

**Reply bit rate: 5/4 x the command rate.** For DShot300 that is
**375 kbit/s** (bit period ~2.67 us) - brushlesswhoop, matches this
project's plan and PINNING.md Section 2.1/Section 2.6.

**Reply payload - 16 bits, GCR-decoded to a 12-bit eRPM-period value + 4-bit
CRC, sent uninverted:**

```
12 bit: eeemmmmmmmm   (3-bit exponent e, 9-bit mantissa m)
 4 bit: CRC, same XOR-of-nibbles algorithm as Section 1, NOT inverted for the reply
```
Comment in AM32 `dshot.c:287-291` (`make_dshot_package`): "format eee mmm
mmm mmm ... allows for a range of up to 65408 microseconds"; brushlesswhoop
independently states the same 3+9 split and "CRC calculated exactly as with
uninverted DShot, sent back uninverted" - **two independent sources agree.**

**Decode (period in us, then eRPM, then mechanical RPM):**
```
period_us = mantissa << exponent                 // AM32 dshot.c:291-301, brushlesswhoop
eRPM      = 60,000,000 / period_us                // period is time for one ELECTRICAL revolution
RPM_mech  = eRPM / pole_pairs                      // pole_pairs = 7 for the X2216 (12N14P), dispatch SYS1-012 Section 6
```
This matches the formula already adopted in `QuadSE/dispatch/SYS1-012 -
Dispatch.md` Section 6 (`omega [rad/s] = 2*pi*eRPM / (60*pole_pairs)`) - no change
needed there, this note is the independent confirmation it cites.

**GCR 4-bit to 5-bit encode table** (`dshot.c:20-23`, identical to the table
independently published on brushlesswhoop - cross-checked digit by digit):

| nibble | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | A | B | C | D | E | F |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| GCR (5b, binary) | 11001 | 11011 | 10010 | 10011 | 11101 | 10101 | 10110 | 10111 | 11010 | 01001 | 01010 | 01011 | 11110 | 01101 | 01110 | 01111 |

**20 to 21 bit step (differential/NRZI-style, non-NXP targets incl. AT32F421):**
each output bit = the GCR bit XOR'd with the *previous output bit*, MSB
first, with one extra leading `1` inserted (`gcr[1+pad]=128`, i.e. a `1`
bit ahead of the 20 encoded bits - `dshot.c:339-344`, the `#else` branch, which
is what the F421/AT32 target compiles). Decode side (FC) undoes this by
re-differentiating: consecutive received bits are XOR'd to recover the raw
GCR digit stream, per brushlesswhoop's `gcr = value ^ (value >> 1)` recipe
- the "eRPM decoding formula" the FC side needs to implement is the inverse
of AM32's forward encode above, not a separately invented scheme.

**Extended DShot Telemetry (EDT), AM32-specific - how to tell it apart from
an eRPM frame:** EDT frames carry a **4-bit type tag in the top nibble of
the 12-bit payload**, which is otherwise impossible for a plain eRPM value
(the top 3 bits are the exponent, and exponent `111` together with a set
type-tag bit pattern cannot occur from a real period). Types seen in
`dshot.c:256-277`:

| top nibble (binary) | meaning | payload (low byte) |
|---|---|---|
| `0010` | temperature | degC, `(uint8_t)degrees_celsius` |
| `0100` | voltage | `battery_voltage/25` (units: 0.25 V steps - **not** the KISS 0.01 V; see Gaps) |
| `0110` | current | `actual_current/100` (already-divided byte - verify scale against a wire capture, not re-derived here) |
| `111000000000` (`send_EDT_init`) | EDT stream starting | sentinel, no payload |
| `111011111111` (`send_EDT_deinit`) | EDT stream stopping | sentinel, no payload |

EDT frames are **interleaved with eRPM frames, never sent back-to-back**
(`telem_scheduler.last_sent_extended` forces one plain-eRPM frame between
any two EDT frames, `dshot.c:246-269`) - a decoder that assumes every
bidirectional reply is an eRPM period will misread roughly 1-in-N EDT
frames as garbage eRPM unless it checks the type-tag pattern first.
EDT is only sent if the FC issued **DShot command 13** (enable) and not yet
command 14 (disable) - Section 4.

---

## 4. Special commands (`dshot.c:110-233`)

All commands 1-47 except `0` need **6 consecutive identical frames** before
AM32 executes them (`command_count` debounce, `dshot.c:157-166`) - **except
beacons (1-5), which execute on the very first frame** (`command_count` is
force-set to 6 immediately for `dshotcommand<=5`, then incremented past the
`>=6` gate on that same call). Commands only process while `armed` and
`!running` (motor stopped) - `dshot.c:157`.

| Cmd | Action | Repeats needed | Notes |
|---|---|---|---|
| 0 | disarm / normal throttle path resumes | - | also clears `EDT_ARMED` if `EDT_ARM_ENABLE` |
| 1-5 | beacon tones 1-5 | 1 (immediate) | `play_tone_flag = 1..5` |
| 6 | send ESC info packet | 6 | `send_esc_info_flag=1` - the 48-byte eeprom dump via `makeInfoPacket()`, not the 10-byte KISS frame |
| 7 | spin direction "1" (`dir_reversed=0`) | 6 | |
| 8 | spin direction "2" (`dir_reversed=1`) | 6 | |
| 9 | bidirectional DShot **off** (`bi_direction=0`) | 6 | AM32 uses 9/10 for the bidir toggle, not "3D mode" - see Traps |
| 10 | bidirectional DShot **on** (`bi_direction=1`) | 6 | |
| 12 | save settings to EEPROM | 6 | `saveEEpromSettings()` |
| 13 | Extended DShot Telemetry **enable** | 6 | arms EDT stream, Section 3 |
| 14 | Extended DShot Telemetry **disable** | 6 | |
| 20 | spin direction normal | 6 | |
| 21 | spin direction reversed | 6 | |
| 36 | enter programming mode (EEPROM byte-write over DShot) | 6 | not a normal-flight command |

**Programming mode (36) is a 3-step DShot-encoded EEPROM write**
(`dshot.c:110-128`): frame 1 = EEPROM byte position, frame 2 = new byte
value, frame 3 = magic value `37` to commit - `dshotcommand`'s normal
6-frame debounce is bypassed while `programming_mode>0`. Not relevant to
flight operation; listed because a driver that blindly forwards raw
throttle values above 47 while "in programming mode" could corrupt the
ESC's EEPROM.

---

## 5. Arming sequence and signal-loss behaviour (AM32, `main.c`)

**To arm:** the FC must hold a **valid zero-throttle DShot frame
continuously for >=1.0 s** (`armed_timeout_count > LOOP_FREQUENCY_HZ`,
`LOOP_FREQUENCY_HZ=20000` at the 20 kHz main loop = 1.000 s,
`main.c:1342-1356`, `Inc/targets.h:177` for the constant), **and** at least
30 zero-throttle samples must have been seen (`zero_input_count>30`). Any
nonzero throttle during that window resets the 1 s timer to zero
(`main.c:1387-1389`) - there is no separate "arm command", arming is purely
a function of sustained zero throttle.

**Signal loss (`signaltimeout`, incremented once per 20 kHz tick,
`main.c:1550-1558`):**

| State | Timeout | Action |
|---|---|---|
| **Armed** | `signaltimeout > LOOP_FREQUENCY_HZ>>1` = **10 000 ticks = 500 ms** | `allOff()`, `armed=0`, all duty cycles forced 0, then **`NVIC_SystemReset()`** - the ESC MCU reboots |
| Not armed | `signaltimeout > LOOP_FREQUENCY_HZ<<1` = 40 000 ticks = **2.0 s** | same `allOff()` + reset |

`main.c:1977-2003`. **This is the number for SYS2-SAF-002**: an AM32 ESC
that stops receiving valid DShot frames while armed cuts motor drive and
resets itself within **500 ms** on its own, independent of anything the FC
does - a link-loss failsafe on the FC side only needs to beat or match this
window if it wants to be the one that decides the outcome; if the FC does
nothing, AM32 still stops the motor unilaterally at 500 ms. (Changelog
`main.c:178`: "1.95 - reduce timeout to 0.5 seconds when armed" - an
earlier AM32 revision used a longer window; 500 ms is the current shipped
behaviour, not a historical value.)

---

## 6. KISS / BLHeli_32-style serial telemetry, Pad T

**Wire:** USART1, single-line half-duplex, **115200 8N1**, TX pin push-pull
with an internal pull-up (`Mcu/f421/Src/serial_telemetry.c:19-64`:
`usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT)`,
`usart_single_line_halfduplex_select(USART1, TRUE)`,
`gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL`). This is the
**AT32F421 target's own config**, which is the MCU on the GOKU G55M - high
confidence this is what ships, though the exact GPIO pin letter/number in
that file is generic-target and not necessarily the GOKU's silkscreen pad.

**One MCU, one USART, one `aTxBuffer` - all 4 motor channels share this one
physical wire and one transmit buffer** (`Src/kiss_telemetry.c:6`,
`Mcu/f421/Src/serial_telemetry.c` - a single `USART1`/`DMA1_CHANNEL2`
instance, no per-channel indexing visible in the fetched source). **This
directly answers the open question in `dispatch/SYS1-012.md` Section 8**: yes, the
four channels share one T pad, and the addressing mechanism is exactly the
per-channel DShot telemetry-request bit (Section 1) - whichever motor's DShot
frame most recently had bit 11 set is the one whose `makeTelemPackage()`
result goes out next on the shared UART. **The FC must not set the
telemetry bit on two channels within the same reply window** or the two
replies will collide on the wire; round-robin polling (one channel's bit
set per DShot frame batch) is not an optional nicety, it is required by
this architecture.

**Frame - 10 bytes, big-endian 16-bit fields, `__attribute__((packed))`**
(`Inc/kiss_telemetry.h:6-19`):

| Offset | Field | Type | Units |
|---|---|---|---|
| 0 | `temperature` | `int8_t` | degC, signed |
| 1 | `voltage_h` | `uint8_t` | voltage MSB |
| 2 | `voltage_l` | `uint8_t` | voltage LSB - combined `voltage` = **centivolts (0.01 V)** |
| 3 | `current_h` | `uint8_t` | current MSB |
| 4 | `current_l` | `uint8_t` | current LSB - combined = **centiamps (0.01 A)** |
| 5 | `consumption_h` | `uint8_t` | mAh MSB |
| 6 | `consumption_l` | `uint8_t` | mAh LSB - combined = **mAh, accumulated** |
| 7 | `erpm_h` | `uint8_t` | eRPM MSB |
| 8 | `erpm_l` | `uint8_t` | eRPM LSB - combined = **eRPM / 100** (comment `main.c:540`: "1 in the packet means 100 eRPM") |
| 9 | `crc` | `uint8_t` | CRC-8 over bytes 0-8 |

**CRC-8** (`Src/functions.c:109-127`): polynomial **0x07** (x^8+x^2+x+1,
"CRC-8/SMBUS" family), MSB-first, seed `0`, no input/output reflection:
```c
crc_u = crc_u ^ crc_seed;
for (8 bits) crc_u = (crc_u & 0x80) ? (0x7 ^ (crc_u<<1)) : (crc_u<<1);
```
applied byte-by-byte over the 9 payload bytes (`get_crc8`, `Src/functions.c:120-127`).

**Request mechanism:** identical to bidirectional DShot's telemetry bit -
there is no separate "request byte" on the T-wire; the ESC decides to send
based on the DShot telemetry-request bit of its own motor channel, then
transmits the 10-byte frame on the shared UART some time after. **No
explicit AM32 source states the minimum spacing between requests to
different ESCs** - treat the per-frame turnaround (Section 3, 30 us) plus the
10-byte-at-115200-baud transmit time (10 bytes x 10 bits/byte / 115200 ~
**870 us**) as the practical floor between two different channels' KISS
replies; **not independently confirmed in source - Gap.**

`makeInfoPacket()` (command 6, Section 4) sends a **different, 49-byte** frame -
the raw 48-byte EEPROM dump plus a CRC-8 byte - not the 10-byte KISS frame.
A decoder keyed only on frame length can use this to distinguish an
ESC-info reply from a telemetry reply.

---

## 7. AURIX GTM mapping - ATOM (TX) / TIM (RX)

Per `docs/PINNING.md` Section 2.1/Section 2.6: one pad per motor, ATOM0 channel drives TX
(16 DShot bits), the same pad flips to TIM0/TIM7 input-capture for the
~375 kbit/s GCR reply.

**TX - ATOM PWM.** The iLLD ships a ready-made driver,
`Gtm/Atom/Pwm/IfxGtm_Atom_Pwm.{c,h}` (`IfxGtm_Atom_Pwm_init/_start/_stop`),
distinct from the `IfxGtm_Tom_Pwm_*` driver already used for the GPIO/PWM
feature (`docs/ILLD_NOTES.md` Section 8) - **use the ATOM driver, not TOM,** for
DShot. Per the driver's own header-comment example
(`IfxGtm_Atom_Pwm.h:79-105`): ATOM's clock is **CMU CLK0**, not FXCLK (FXCLK
feeds TOM); `period`/`dutyCycle` are raw counter ticks against that clock,
set via `IfxGtm_Atom_Pwm_Config.period/dutyCycle`, output pin via
`IfxGtm_ATOM0_x_TOUTn_Pxx_x_OUT` pin objects (`_PinMap/IfxGtm_PinMap.h`).
**A single fixed-period/fixed-duty PWM channel does not by itself produce
16 different bit widths in one frame** - driving 16 distinct compare values
per frame (the actual DShot waveform) needs either 16 back-to-back one-shot
reprogrammings of the compare register (CPU- or DMA-fed) or the ATOM's
one-shot/serial modes; **this appendix does not document that generation
technique in prose - see Gaps.**

**RX - TIM input capture.** Section "26.3.1 GTM IP Registers Specific Settings",
pdf p591-598 (`docs/infineon-aurix-tc39x-usermanual-en.pdf`), documents the
`TIMi_CHx_CTRL` register directly (not just a connectivity table - the
mode-select field `TIM_MODE` at bits 3:1 is present in this appendix, an
exception to the "appendix has no operating-mode prose" pattern below):

| `TIM_MODE` | Mode | Relevance |
|---|---|---|
| `000` | PWM Measurement (TPWM) | pulse-width capture - could time individual GCR bit widths |
| `010` | Input Event (TIEM) | timestamps every edge into `GPR0`/`GPR1` - the natural mode for a variable-length serial burst like the 21-bit GCR reply |
| `100` | Bit Compression (TBCM) | packs sampled bits - candidate for direct serial-to-parallel capture of the GCR stream |
| `110` | Serial Shift (TSSM) | shift-register mode - another candidate |

Four plausible modes exist for capturing a 21-bit variable-rate serial
burst; **this appendix gives the register bit layout but not a worked
example or timing diagram for any of them** - the actual mode choice
(TIEM vs TBCM vs TSSM) is a `flight-architect` design decision against the
register semantics above, not something this note can settle. Each TIM
channel exposes only `GPR0`/`GPR1` (2x32-bit) plus `ECNT` - **no FIFO**
beyond that; capturing a 21-edge burst without loss needs either an ARU-fed
DMA transfer per edge (Section 26.3's ARU chapter, pdf p916) or a mode (TBCM/TSSM)
that accumulates multiple bits into one register read. Timestamp clock is
`CMU_CLKx` (selectable 0-7, `CLK_SEL` field) - **the achievable resolution
depends on which CMU clock is routed and its divider, not stated as a fixed
number here**; GTM clusters CCM0-4 are rated **200 MHz max**
(usermanual.pdf pdf p507) - if CLK0 runs at or near that, timestamp
resolution is <=5 ns, comfortably above the required >=3.75 MHz (267 ns) for
a 375 kbit/s GCR bit - but this is a capability ceiling, not a measurement.

**Errata (`infineon-aurix-tc39x-bd-step-erratasheet-en.pdf` v2.7,
2025-12-10) relevant to this design:**

| ID | Scope | Effect | Relevance here |
|---|---|---|---|
| `GTM_AI.308` | TIM, ARU | back-to-back TIM data transfers **at full ARU clock rate** can lose every second transfer in ARU dynamic-routing mode | GCR RX is 375 kbit/s, far below "full ARU clock rate" (which tracks the ~100-200 MHz GTM cluster clock) - **very likely not triggered, but not proven; re-check the actual ARU clock divider chosen before trusting dynamic routing for this capture** |
| `GTM_AI.320` | ATOM | a SOMS one-shot cycle **restarts unexpectedly** if `ATOM[i]_CH[x]_CM0` is written to zero while `SR0` is written nonzero | relevant only if the DShot TX generation technique uses SOMS one-shot mode with a possible zero compare value - DShot's own bit encoding never needs CM0=0 (even a "0" bit has ~1.25 us high time), but **worth a defensive check in the TX generator, not just an assumption** |
| `GTM_AI.298`/`.299` | TOM/ATOM | wrong one-shot output behaviour when the one-shot trigger is `TIM_EXT_CAPTURE(x)` or `trig_[x-1]` | only relevant if TX bit-chaining uses external-capture-triggered one-shots between bits - avoid that triggering scheme if the per-bit generation design considers it |

No erratasheet hits for `ASCLIN` or `EVADC` (searched, case-insensitive, no
matches) - nothing known-broken on the telemetry-UART or current-sense ADC
paths specific to this use.

---

## Traps

| # | Trap |
|---|---|
| D1 | Bidirectional-DShot signal polarity is **not a config bit AM32 asks the FC to set first** - AM32 auto-detects it from idle-line level while disarmed (Section 1). A driver that hard-codes "send inverted CRC because bidir is configured" without also matching the idle-high line convention will desync. |
| D2 | AM32's `9`/`10` DShot commands are **bidirectional-DShot on/off**, not "3D mode" - do not assume the widely-quoted generic DShot command table's semantics for these two numbers without checking the firmware that will actually run (`dshot.c:198-203`). |
| D3 | Command 6 ("ESC info") and the KISS telemetry-bit-triggered reply are **two different frame formats and lengths** (49 bytes vs 10 bytes) sharing the same wire - a parser must key off length/framing, not assume every reply is a KISS frame. |
| D4 | EDT (extended telemetry) frames are interleaved with plain-eRPM frames and carry a type-tag in the top nibble that is otherwise impossible for a real eRPM period (Section 3) - a decoder that always treats a 12-bit reply as `eee mmmmmmmmm` will occasionally misdecode an EDT frame as a bogus eRPM value unless it checks the tag first. |
| D5 | Vendor listing (Flywoo product page) claims DShot1200 support; **the upstream AM32 README only advertises DShot300/600** - the higher rate is either firmware-version-dependent or a listing error. Confirm the actual protocol menu in the AM32 configurator on arrival rather than trusting the vendor page (Part 3 checklist). |
| D6 | AM32 **self-calibrates** its expected bit timing from the measured frame (Section 2) - it does not strictly validate the FC's timing against the DShot300 spec numbers. This is forgiving for a first bring-up, but it also means a badly-timed AURIX TX generator could "work" against AM32 while failing the nominal spec - do not use "the ESC accepted it" as proof the timing is spec-correct. |
| D7 | The GTM appendix in this repo is genuinely an **appendix** - device-specific register/connectivity tables, not the GTM family behavioural chapter. Several fields say **"Coding see Family Spec"** verbatim (e.g. filter mode fields, pdf p597) - that document is not in this repo. Do not assume every GTM question is answerable from the PDFs on disk; see Gaps. |

## Cross-references

- `docs/PINNING.md` Section 2.1 (pin map), Section 2.4 (current sense + telemetry pins),
  Section 2.6 (single-pin bidirectional scheme, electrical) - this note supplies
  the protocol-level facts that section assumes.
- `docs/ESC_AM32.md` - ESC-specific pad map, electrical checks, config
  facts, on-arrival checklist.
- `docs/ILLD_NOTES.md` Section 8 (`IfxGtm` - TOM PWM; extend with an ATOM section
  once the DShot driver is actually written, per that file's own "write it
  back" rule).
- `QuadSE/requirements/SYS2/SYS2-ACT-001` (motor interface), `SYS2-SAF-002`
  (link-loss failsafe - Section 5's 500 ms AM32 timeout is the number to compare
  against), `QuadSE/dispatch/SYS1-012 - Dispatch.md` Section 6 (kT measurement via
  eRPM), Section 8 (KISS telemetry as the pack-voltage source, decided 2026-09-03).

## Gaps

- **AURIX-side DShot TX bit-generation technique** (16 distinct compare
  values per frame via ATOM) is not documented anywhere in this repo's
  PDFs or the iLLD driver's own example - the header shows only a
  fixed-period/fixed-duty PWM use case. This is a genuine design gap for
  `flight-architect`, not a missing-note problem: the GTM family
  specification that would show the one-shot/serial-chaining technique for
  a varying-width burst is not in this repo.
- **AURIX-side GCR RX capture mode** (TIEM vs TBCM vs TSSM, Section 7) - register
  bit layout is here, a worked example is not.
- KISS telemetry minimum spacing between two different ESCs' replies on the
  shared line - no explicit AM32 source constant found; Section 6 gives a derived
  floor (~870 us transmit + 30 us turnaround), not a stated minimum.
- Frame-gap / minimum-idle number (Section 2) is sourced only to brushlesswhoop,
  not corroborated in AM32 source - flag as secondary-source only.
- Current-sense ADC scale (mV/A) - not in the DShot/telemetry path at all;
  see `docs/ESC_AM32.md` Gaps.
- DShot1200 support discrepancy (D5) - unresolved without the physical ESC
  and its configurator.
- Betaflight wiki pages did not render technical content through the
  fetch path used (returned repository-overview boilerplate); the figures
  attributed to "Betaflight wiki" above come from a WebSearch result
  snippet quoting that wiki, not a direct page fetch - treat as
  second-hand until re-verified by an agent with working wiki access.
