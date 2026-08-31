# MMC5983MA.md — MEMSIC MMC5983MA magnetometer, wiring & bring-up

**ASPICE:** SWE.3 — driver detailed design + bring-up evidence · supports SYS1-010 · process: QuadSE/requirements/README.md

Wiring and bring-up notes for the **MMC5983MA** 3-axis magnetometer on **I2C0**,
joining the BMP581 on the shared sensor bus.

**Board: MEMSIC MMC5983-B prototyping board** (die marked `MMC5983MA`, `U1` on
the board schematic).

> **Pin-allocation SSoT:** the authoritative MCU-side pin map is
> **[PINNING.md §2.3](PINNING.md)** (I2C0 SCL/SDA) and **§2.5** (electrical). This file
> covers the *breakout side* and does not allocate MCU pins.

Driver: `src/bsw/Mmc5983.{c,h}`, firmware **v1.15.0**.

> ✅ **HARDWARE-VALIDATED 2026-08-01, with one caveat.** `PRODUCT_ID` = `0x30`,
> continuous mode running, data live, no `MAG_*` diagnostics bits, BMP581
> undisturbed on the shared bus. A 1609-sample sphere fit came out at **0.9 %
> residual**, which confirms the 18-bit assembly, the byte order and the
> relative per-axis scaling (§8.5).
> ⚠️ **Caveat: the absolute counts-per-gauss constant is still unconfirmed**,
> because the validation was done indoors where the building's steel distorts
> the field. One outdoor sphere fit closes it — see §8.5.

---

## 1. Wiring — five wires, no new MCU pins

The magnetometer joins the two pins the BMP581 already uses. Nothing on the MCU
side changes, so `I2c.c` needs no edit.

```
   +3V3 rail (X702·78/80 or X703·70) ─────────── VDD   (P1-3)
   GND (X702·1-4 etc.)               ─────────── GND   (P1-4)
   TC399 P13.1  SCL (X702·29)        ─────────── SCL   (P2-3)
   TC399 P13.2  SDA (X702·35)        ─────────── SDA   (P2-2)
   +3V3                              ─────────── CS    (P1-1)  ⚠️ see §3
```

Header pinout, from the board's user guide (silkscreen names in brackets where
they differ):

| Pin | Name | Wire to |
|---|---|---|
| P1-1 | `SPI_CS` (**CS**) | ⚠️ **+3V3** — selects I2C, §3 |
| P1-2 | `NC` (silkscreen: **VDDIO**) | leave open, §2 |
| P1-3 | `VDD` | **+3V3** |
| P1-4 | `GND` | GND |
| P2-1 | `SPI_SDO` (**SDO**) | leave open |
| P2-2 | `SDA/SPI_SDI` (**SDA**) | **P13.2**, X702·35 |
| P2-3 | `SCL/SPI_SCK` (**SCL**) | **P13.1**, X702·29 |
| P2-4 | `INT` | leave open — polled at 50 Hz |

The two headers are on opposite edges: `CS / VDDIO / VDD / GND` on one side,
`SDO / SDA / SCL / INT` on the other.

---

## 2. Power — +3V3, and VDDIO needs nothing

Feed `VDD` (P1-3) from the board's regulated **+3V3** rail, the same rail the
BMP581 uses.

The pin table calls P1-2 `NC` while the PCB silkscreen calls it `VDDIO`. Either
way **leave it open**: the schematic ties the die's `VDDIO` (pin 13) directly to
`VDD`, so the I/O rail is already powered from P1-3. There is no second supply
to provide.

---

## 3. ⚠️ CS must be tied to +3V3 — this board does NOT do it for you

`CS` selects the interface: **high = I2C, low (or floating) = SPI**. On the
MMC5983-B schematic **`CS` runs straight from header pin P1-1 to the die's
`SPI_CS` pin with no pull-up anywhere on the board.** It floats unless you wire
it.

This is the single most likely reason a correctly wired part never ACKs, and it
is the same trap the BMP388 sprang. Note this is a **step backwards from the
BMP581**: the Adafruit board strapped CS high internally, so that one worked
with four wires. This one needs five.

**Tie P1-1 to +3V3.**

---

## 4. Address — `0x30`, fixed

The MMC5983MA has **no address select pin**, so there is exactly one address and
nothing to get wrong — a pleasant change from the BMP581's `0x46`/`0x47`.

| Device | Address |
|---|---|
| **MMC5983MA** | **`0x30`** |
| BMP581 (fitted) | `0x47` |
| NEO-M9N (planned) | `0x42` |
| ICM-42688-P (planned) | QSPI0, not on this bus |

No collision with anything on the bus now or planned.

Confusingly, the **PRODUCT_ID register (0x2F) also reads `0x30`** — the same
value as the address. That is a coincidence, not a mistake.

---

## 5. ⚠️ Bus loading changes again — now ~2.1 kΩ

Track this every time a device joins or leaves; it has already caused one
multi-day debug.

| Configuration | Combined pull-up |
|---|---|
| BMP388 + MPU-6050 (until 2026-07-31) | ~3.2 kΩ |
| BMP581 alone (2026-08-01) | 10 kΩ |
| **BMP581 + MMC5983MA (this change)** | **~2.1 kΩ** |

The MMC5983-B carries **2.7 kΩ** pull-ups (`R1`, `R2`) on SDA and SCL to VDD,
read off its schematic. In parallel with the BMP581's 10 kΩ that gives ~2.1 kΩ.

That is stronger than anything this bus has run before but still well within
what the TC399 pads sink at 3.3 V — roughly 1.6 mA per line — and it makes edges
faster, not slower. No action needed; recorded so the number is known rather
than guessed at the next change.

---

## 6. ⚠️ There is no device datasheet — only the board guide

`Quadrocopter/doc/Magnetometer.pdf` is the **MMC5983-B Prototyping Board User's
Guide**, one page, Rev A. It states plainly: *"For complete performance
specifications of MMC5983 refer to MMC5983 datasheet."*

What it **does** give, and what §1–§5 above are built on:
- the full header pinout,
- the board schematic — from which the 2.7 kΩ pull-up values, the absent CS
  pull-up and the `VDDIO`→`VDD` tie were read directly,
- the die marking `MMC5983MA`.

What it does **not** give: **any register information at all** — no register
map, no addresses, no bit fields, no scaling.

So every register constant in `Mmc5983.c` is written from the MMC5983MA
register map without a datasheet to check it against, exactly as the BMP581 was.
`Mmc5983_debugDump()` is the instrument that closes that gap (§8). If you obtain
the device datasheet, §7 is the list to verify against it first.

---

## 7. What the driver assumes

| Register | Assumed | Used for |
|---|---|---|
| `0x00..0x06` | X/Y/Z, 2 bytes each, then `XYZout2` | 7-byte burst read |
| `0x08` | `STATUS`, bit 0 `Meas_M_Done` | boot dump only |
| `0x09` | `CTRL0`, bit 5 `Auto_SR_en` | automatic set/reset |
| `0x0A` | `CTRL1`, bit 7 `SW_RST`, bits 1:0 bandwidth | reset, 100 Hz BW |
| `0x0B` | `CTRL2`, bit 3 `Cmm_en`, bits 2:0 `CM_freq` | continuous 50 Hz |
| `0x2F` | `PRODUCT_ID` = `0x30` | bring-up gate |

Scaling assumed: unsigned **18-bit** per axis, zero field at mid-scale
**2¹⁷ = 131072**, **16384 counts per gauss**, i.e.

```
    gauss = (raw18 - 131072) / 16384
```

Axis assembly: `[17:10]` from the first byte, `[9:2]` from the second, `[1:0]`
from `XYZout2` at bit offset 6 (X), 4 (Y), 2 (Z).

Two behaviours worth knowing before debugging anything:

1. **The control registers 0x09–0x0C do not read back what you wrote.**
   Measured 2026-08-01: `CTRL0`, `CTRL1` and `CTRL2` **all returned `0x61`** —
   identical to each other, and matching neither the written values (`0x20`,
   `0x00`, `0x0C`) nor zero. Three distinct registers reporting one value means
   the readback path does not return configuration at all. **Never verify
   configuration by reading these back.** That the writes *do* land is proven
   another way: continuous mode runs and the field data fits a sphere to 0.9 %,
   neither of which is possible if `CTRL2` had been ignored.
   *(An earlier revision of this document claimed they read `0x00`. They do
   not — corrected against hardware.)*
2. **Automatic set/reset is not optional.** An AMR bridge drifts its own
   magnetisation, so a raw reading carries an offset that wanders with
   temperature and with any strong field the part has seen. `Auto_SR_en`
   (CTRL0 bit 5) makes the device do the set/reset pair internally around every
   measurement and output the compensated value. Without it the output looks
   perfectly alive and is quietly wrong — the worst failure mode there is.

Temperature (`Tout`, 0x07) is **not** read: it requires a one-shot `TM_T`
measurement that interrupts continuous magnetic conversion, and the barometer
already provides an ambient temperature.

---

## 8. Bring-up order — one step at a time

### 8.1 Bus idle
Boot and look for `I2C0 bus idle (SCL+SDA released)`. With two devices and
~2.1 kΩ of pull-up this should be emphatic. `HELD` means a short or a jammed
slave — stop there.

### 8.2 The BMP581 must still work
A new device on a shared bus can break the existing one (address clash, a
shorted line, a wedged master). Confirm `BMP581 detected` and a sane pressure
**before** looking at the magnetometer at all. If the barometer broke, the
magnetometer wiring is the suspect.

### 8.3 PRODUCT_ID
```
MMC5983 detected (PRODUCT_ID 0x30)
MMC5983 not found - check wiring/CS strap (CS must be tied to +3V3)
```
This one transaction proves pins, pull-ups, address, **CS strap** and driver
together. Failure here is the CS strap (§3) far more often than anything else.

### 8.4 The boot dump

Known-good reference, measured 2026-08-01 on firmware v1.15.0:

```
MMC5983 detected (PRODUCT_ID 0x30)
MMC5983 PID/STATUS/CTRL0/CTRL1/CTRL2=30 10 61 61 61  burst=00 00 00 00 00 00 00
```

| Field | Observed | Meaning |
|---|---|---|
| `PID` | `0x30` | must match — proves pins, pull-ups, address, **CS strap** and driver |
| `STATUS` | `0x10` | bit 4; no measurement completed yet, expected this close to init |
| `CTRL0/1/2` | `61 61 61` | ⚠️ **expected, and not the written values** — see §7. Do not read these back to verify configuration |
| `burst` | all `00` | ⚠️ **normal here** — continuous mode has only just started and no conversion has completed. (The BMP581 shows `0x7F` in the same situation.) All-zero once the 50 Hz task is running is the real fault |

The dump is confirmation, not discovery: by the time you read it, `magPresent`
over XCP has already told you the part is present and configured.

### 8.5 Plausibility — sphere fit, NOT a single magnitude

⚠️ **Do not validate against "0.48 gauss in Munich" indoors.** That is the
*free-field* value. Inside a building — especially a multi-storey one, with
reinforcing steel in every floor and wall — the local field is routinely
distorted past 2× and varies from room to room and floor to floor. Measured
2026-08-01 indoors: `|B|` swung **0.75 → 1.10 G** simply from moving the sensor
around, which is the building's steel measured directly. A single indoor
magnitude proves nothing about the scaling constant.

**The test that does work indoors is a sphere fit.** Sample while tumbling the
board through all orientations, then least-squares fit a sphere to the cloud:

```
    x² + y² + z² = 2ax + 2by + 2cz + d      centre (a,b,c), r = sqrt(d + a²+b²+c²)
```

The **residual spread** is the diagnostic, and it is independent of the ambient
field's strength:

| Residual | Meaning |
|---|---|
| **< ~2 % of radius** | the cloud is a sphere → the 18-bit assembly, the byte order within each axis and the *relative* per-axis scaling are all correct |
| large / lumpy | axis assembly or byte order wrong |
| ellipsoidal | per-axis scale mismatch (soft iron, or a per-axis code bug) |

The fitted **centre** is the hard-iron offset — field sources fixed relative to
the sensor. The fitted **radius** is the true local field magnitude with that
offset removed.

Reference result, 2026-08-01, indoors, firmware v1.15.0:
`centre (-0.4044, -0.0848, +0.1441) G, |offset| 0.4376 G, radius 1.0414 G,`
**`residual 0.9 %`** — a clean sphere, so the data path is confirmed.

> ⚠️ **That reference disagrees with the 2026-08-26 calibration and is the
> weaker of the two.** `tools/mag_cal.py` measured
> `centre (-0.1940, -0.0722, -0.8510) G, radius 0.4655 G` on identical firmware
> — `MMC5983_COUNTS_PER_GAUSS` has never changed (one commit, still 16384).
> The radii differ by a factor of **2.24**.
>
> The 2026-08-26 figure is the one to trust: its **octant coverage was verified
> at 8/8**, and it lands on ~0.48 G, the field actually expected in Munich. The
> 2026-08-01 run recorded no coverage figure, and an incompletely covered sphere
> extrapolates both centre and radius — a low residual does not rescue it,
> because a well-fitted ring is still a ring. This is exactly the trap the live
> `corners n/8` display was added to prevent.
>
> The **centre** moving so far (Z from +0.144 to −0.851 G) is expected and is
> not a contradiction: the ICM-42688-P and the NEO-M9N were both added to the
> assembly in between, and hard iron is a property of the assembly. It is
> precisely why these numbers live in NVM rather than in a header.

### ⚠️ OPEN ITEM: absolute scale — very likely settled, one outdoor run to confirm

`MMC5983_COUNTS_PER_GAUSS` (16384) was **plausible but unconfirmed**. It is now
*probably* confirmed: the 2026-08-26 fit, with verified 8/8 octant coverage,
gives a radius of **0.4655 G** against the ~0.48 G expected in Munich. A wrong
constant would show as a clean 2x or 4x error, not as 3 %.

The caveat is that this run was **indoors**, which is the very condition this
section says cannot be trusted — building steel distorts the field. So treat it
as strong evidence rather than proof.

**To close it properly**, run the calibration outdoors, away from the building,
vehicles and reinforced concrete:

```
python tools/mag_cal.py --seconds 120 --write --decl 3.9
```

Tumble until the display reads `corners 8/8`; it will not finish before that,
and it refuses to write a fit with incomplete coverage or a residual above 5 %.
The raw samples are always dumped first, so a run can be re-fitted later with
`--load` instead of re-tumbled. `corrected |B|` is the number that matters: it
should come out at ~0.48 G. No reference instrument needed; that is why this
beats any indoor comparison.

**Why it could not be settled indoors (attempted 2026-08-01):**

| Source | Reading |
|---|---|
| Sensor, sphere-fit radius (offset removed) | **104 µT** |
| Phone (phyphox, Android accuracy "high"), elsewhere in the room | **61 µT** |
| Munich free field | 48 µT |

Both readings exceed 48 µT, so the building's distortion is confirmed by two
independent instruments. But the comparison **cannot decide the scaling**,
because the two hypotheses are both consistent with the data:

- scaling **correct** → true field is 104 µT at the sensor, 61 µT at the phone;
- scaling **2× wrong** → true field is 52 µT at the sensor, 61 µT at the phone.

Co-locating the two instruments to better than a few cm is not practical, and
the field varies over exactly that scale (85 → 109 µT was measured across the
room). A phone also cannot remove the 0.44 G hard-iron offset, which the sphere
fit does for free.

**If an indoor answer is ever needed**, the way to get one is to bound the
position error rather than assume it away: rotate the board *in place* for a
spot-local sphere fit, then use the phone to sample |B| at that spot and ±5 cm
in each direction. If the sensor's radius falls outside the phone's spread, the
scaling is wrong; if the spread is itself huge, the gradient there defeats any
indoor test and that is worth knowing too.

Note that a *wrong* scaling constant would be off by a clean **2× or 4×** (the
16-bit vs 18-bit conventions), so this is a binary question, not a calibration
tweak.

Magnitude sane but drifting over minutes → `Auto_SR_en` did not take (§7).

### 8.6 Heading sweep
`magHeadingDeg` is a bring-up aid only — level-only, uncalibrated, no
declination (see `Mmc5983.h`). Rotating a **level** board through a full turn
must sweep it through the whole 0–360° range, roughly linearly. That confirms
the two horizontal axes are both live and correctly signed.

### 8.7 Liveness
`PeriphDiag` raises `MAG_STUCK_DATA` after 5 s of a bit-identical reading.
Sensor noise should move the low bits every sample.

---

## 9. Datasheet status

**Not in the repo, and not obtainable from the PDF that is** (§6). The board
guide lives at `Quadrocopter/doc/Magnetometer.pdf`. Per
[the no-vendor-PDFs rule](PINNING.md) it is deliberately **not** copied into
this repository.

If the MMC5983MA device datasheet turns up, check §7 against it first — that
table is the entire set of unverified assumptions in this driver.
