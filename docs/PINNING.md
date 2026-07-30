# PINNING.md — TC399 TriBoard pin map (single source of truth)

**Authoritative pin allocation** for this project on the **TriBoard TC3X9
(TC399, LFBGA‑516)**. When a pin question arises, this file wins. The detailed
*rationale* lives in the companion docs — this file is the lookup.

- Actuator / DShot pins, driver & level shifting → §2.1 / §2.4 / §2.6 below
  (**DSHOT_PINS.md retired — folded in here**)
- IMU / baro / mag / GNSS sensor pins & electrical → §2.2 / §2.3 / §2.5 below
  (**SENSORS.md retired — folded in here**)
- Diagnostics / cal blocks → `docs/DIAGNOSTICS.md`

> **Maintenance rule:** when you add or move a peripheral pin, update *this* file in
> the same change. Keep the `Status` column honest (code vs plan vs board).

### Status legend

| Status | Meaning |
|---|---|
| **impl** | Configured in firmware today (source cited) |
| **plan** | Documented & pin‑verified, not yet in code |
| **board** | Fixed by TriBoard hardware (transceiver / PHY / TLF / EVRC / debug) — assembled 0 Ω bridge or dedicated part; **do not reuse** |
| **free** | Available on a header, unallocated (see §5 pool) |

### Board‑wide electrical note

Most general ports are in the **VEXT / V_UC = 5 V** domain (confirmed: R527 assembled,
TLF35584QVVS1, measured P00.1 high = 5.0 V). Two exceptions:
- **VFLEX = 3.3 V — this is Port 11 only** (the Ethernet/RGMII “flex port”). Per the
  Data Sheet **Pad List Table 2‑63** each P11.x pad carries the `VFLEX / ES` supply
  tag; P11.0–P11.12 are the RGMII pins (board‑committed), and **P11.13/14/15 are the
  only free VFLEX pins** (see §5.1). *Correction:* the earlier claim (in the retired
  SENSORS.md and prior PINNING versions) that P14.7/9/10, P15.7/8, P20.12/13 are VFLEX is **wrong** —
  Table 2‑63 tags all of those `VEXT / ES` (5 V). In particular **the core‑LED pins
  P20.12/P20.13 are VEXT (5 V), not 3.3 V.**
- **VEVRSB (TLF35584 standby rail) = Port 33** — Manual §3.16: “port 33 is powered by
  VEVRSB pin”. Also 5 V on this board, but a *different* rail that stays powered when
  the main branch is off. Relevant for any Port‑33 I/O.

Any 3.3 V‑logic peripheral on a VEXT pin needs level adaptation — see §2.5/§2.6.
The three free VFLEX pins (§5.1) are native 3.3 V but few and SLOW‑class.

Header names: **X701/X801** = BUS‑EXPANSION · **X702/X802** = PERIPHERALS ·
**X703/X803** = ADC · **X704/X804** = GTM/PORTS · **X705/X805** = ADC2.
(Top/bottom connectors share one pinout; “X702·30” = header X702/X802, pin 30.)

---

## 1. Firmware‑assigned pins (impl)

### 1.1 Core health LEDs — `src/bsw/Cpu{0..3}_Main.c`

| Pin | LED | Owner | Header·pin | Status |
|---|---|---|---|---|
| P20.11 | D306 | CPU0 | X702·63 | impl |
| P20.12 | D307 | CPU1 | X702·67 | impl |
| P20.13 | D308 | CPU2 | X702·39 | impl |
| P20.14 | D309 | CPU3 | X702·65 | impl |

*(CPU4/CPU5 sync + idle, no pins.)*

### 1.2 GPIO / PWM feature — `src/bsw/gpio_cfg.h` (GTM **TOM0/TOM3**)

All of Port 00. P00.0 is reserved as the Diagnostics error output (not
user‑controllable). Mode = current default in `gpio_cfg.h` (editable there).

| Pin | Mode (default) | TOM ch | Header·pin | Status |
|---|---|---|---|---|
| P00.0 | digital (Diag error, reserved) | TOM0.4 | X702·45 | impl |
| P00.1 | PWM 1 kHz | TOM0.9 | X702·47 | impl |
| P00.2 | PWM 5 Hz | TOM0.5 | X702·48 | impl |
| P00.3 | digital | TOM0.10 | X702·46 | impl |
| P00.4 | digital | TOM0.11 | X702·40 | impl |
| P00.5 | digital | TOM0.12 | X702·68 | impl |
| P00.6 | digital | TOM0.13 | X703·72 | impl |
| P00.7 | digital | TOM0.14 | X703·74 | impl |
| P00.8 | digital | TOM0.15 | X703·76 | impl |
| P00.9 | digital | TOM0.0 | X704·5 | impl |
| P00.10 | digital | TOM0.1 | X704·7 | impl |
| P00.11 | digital | TOM0.2 | X702·66 | impl |
| P00.12 | digital | TOM0.3 | X702·64 | impl |
| P00.13 | digital | TOM3.12 | X705·64 | impl |
| P00.14 | digital | TOM3.11 | X705·66 | impl |
| P00.15 | digital | TOM3.13 | X705·68 | impl |

### 1.3 Debug console — `src/bsw/Uart.c` (ASCLIN0, X109 USB‑to‑UART)

| Pin | Function | iLLD object | Header·pin | Status |
|---|---|---|---|---|
| P14.0 | ASCLIN0 TX | `IfxAsclin0_TX_P14_0_OUT` | X702·61 | impl |
| P14.1 | ASCLIN0 RX | `IfxAsclin0_RXA_P14_1_IN` | X702·59 | impl |

### 1.4 Ethernet RGMII + MDIO — board‑routed, driven by GETH/lwIP

Pins are board‑fixed (see §4) but actively used by the firmware Ethernet stack:
**P11.0–P11.12** (RGMII) + **P12.0** (MDC) / **P12.1** (MDIO).

---

## 2. Planned / reserved pins (plan)

### 2.1 DShot300 bidirectional, 4× ESC — GTM **ATOM0** / **TIM** (driver: §2.6)

| Pin | Motor | ATOM out | TIM in | Header·pin | Status |
|---|---|---|---|---|---|
| P22.1 | M1 | ATOM0.0 (TOUT48) | TIM0.0 / TIM7.2 | X702·30 | plan |
| P22.0 | M2 | ATOM0.1 (TOUT47) | TIM0.1 / TIM7.3 | X702·32 | plan |
| P22.2 | M3 | ATOM0.3 (TOUT49) | TIM0.3 / TIM7.1 | X702·36 | plan |
| P22.3 | M4 | ATOM0.4 (TOUT50) | TIM0.4 / TIM7.0 | X702·34 | plan |

Drive mode = open‑drain ALT1 + 1.5 kΩ pull‑up to 3.3 V (driver & verification: §2.6).

### 2.2 IMU (ICM‑42688‑P) — **QSPI0** (electrical: §2.5)

| Pin | Function | iLLD object | Header·pin | Status |
|---|---|---|---|---|
| P22.8 | SCLK | `IfxQspi0_SCLK_P22_8_OUT` | X702·10 | plan |
| P22.9 | MRST (MISO) | `IfxQspi0_MRST_P22_9_OUT` | X702·12 | plan |
| P22.10 | MTSR (MOSI) | `IfxQspi0_MTSR_P22_10_OUT` | X702·14 | plan |
| P22.11 | CS (SLSO10) | `IfxQspi0_SLSO10_P22_11_OUT` | X702·18 | plan |
| P22.7 | INT1 (data‑ready) | GPIO input | X702·17 | plan |

> **Interim IMU in code — MPU‑6050 (GY‑521) on I2C0, not this QSPI plan.** The
> ICM‑42688‑P above is still `plan` (not delivered). A borrowed **MPU‑6050** stands
> in as the flyable IMU and rides the **shared I2C0 bus** (§2.3) at address **`0x68`**
> — no new MCU pins, no clash with the BMP388 (`0x77`). Driver `src/bsw/Mpu6050.c`,
> read at 50 Hz, exposed over XCP (`accelX…`, `gyroX…`, `imuTempC`, firmware ≥ v1.10.0).
> **Hardware‑unverified.** Bring‑up gate = `Mpu6050_readWhoAmI()` == 0x68. Wiring /
> address / power: **`docs/MPU6050.md`**. When the ICM‑42688‑P arrives it moves to
> QSPI0 as planned here; the MPU‑6050 driver stays in‑tree as the I2C fallback.

### 2.3 Baro / Mag / GNSS — **I2C0**, shared bus (electrical: §2.5)

| Pin | Function | iLLD object | Header·pin | Status |
|---|---|---|---|---|
| P13.1 | SCL | `IfxI2c0_SCL_P13_1_INOUT` | X702·29 | **impl** (bus) |
| P13.2 | SDA | `IfxI2c0_SDA_P13_2_INOUT` | X702·35 | **impl** (bus) |

> **Bus + sensors in code.** `src/bsw/I2c.c` brings up I2C0 at 100 kHz on TTL pads
> (SCL alt6), with bus recovery and a bounded transfer engine (see the file header —
> the iLLD default emits **no STOP**, and `IfxI2c_I2c_initModule()` does **not**
> clear the kernel, both of which cost days). The iLLD `I2c` source tree was
> un-excluded in `.cproject`.
>
> **From 2026-07-31 the active baro is the BMP581 at `0x46`** (`src/bsw/Bmp581.c`,
> wiring **`docs/BMP581.md`**). It replaces the BMP388/CJMCU-388 (`0x77`,
> `docs/BMP388.md`) and the interim MPU-6050/GY-521 IMU (`0x68`,
> `docs/MPU6050.md`), **both physically removed**. Those two are hardware-validated
> and their docs stay as reference; the BMP581 driver is written but
> **hardware-unverified**, every register marked `TODO(hw)`.
>
> The flight IMU (ICM-42688-P, §2.2), mag (MMC5983MA `0x30`) and GNSS (NEO-M9N
> `0x42`) are still `plan`. All four addresses are distinct, so no collision.
>
> ⚠️ **Bus loading changed.** The old bus was held up by two breakouts in parallel
> — CJMCU-388 ~10 kΩ and GY-521 ~4.7 kΩ, about **3.2 kΩ**. Removing both removes
> **all** pull-up: whatever the BMP581 breakout carries is now the entire bus. If
> it has none, the bus never idles high and nothing ACKs, which looks exactly like
> a dead sensor — add **4.7 kΩ from SCL and SDA to +3V3**. The boot line
> `I2C0 bus idle (SCL+SDA released)` confirms it either way.

### 2.4 Actuator companion signals — ESC current sense + telemetry

| Signal | Pin / channel | Resource | Header·pin | Status |
|---|---|---|---|---|
| ESC current sense (Pad C) | AN7 | EVADC **G0CH7** | X703·19 | plan |
| ESC telemetry RX (Pad T) | P23.3 | ASCLIN6 RXA (`IfxAsclin6_RXA_P23_3_IN`) | X702·24 | plan |

**ADC header (X703/X803) — scarce filtered inputs + references:**
- **Only 6 analog inputs are on‑board anti‑alias filtered** (47 nF + 4.7 kΩ series;
  Manual §3.14, Fig. 3‑4): **AN7·19, AN20·14, AN21·16, AN31·36, AN44·54, AN45·56**.
  This is a shared, scarce resource — do not silently grab one; record it here.
- **Sample‑time caveat:** the 4.7 kΩ series R raises the source impedance; the EVADC
  channel’s sample time must be long enough to fully charge the internal sampling cap,
  or the reading is systematically low. Uncritical at the ~25 Hz sensor bandwidths
  here, but set the channel sample time accordingly.
- **VAREF = 5 V** on this board (R220 ties VDDM→V_VR, R222 VAREF1→VDDM). A 0–3.3 V
  signal therefore uses only ⅔ of the converter range — factor into scaling.
- **References / supply on X703/X803:** VDDM·39, VAREF1·40, VAREF2·42; VEXT·69,
  +3V3·70; GND 1‑4, 37, 38, 41, 43, 44, 61, 62.

### 2.5 Sensor electrical & bus notes (folded in from the retired SENSORS.md)

**Bus / driver assignment**

| Role | Part | Bus | iLLD driver | Rate |
|---|---|---|---|---|
| IMU (planned) | ICM‑42688‑P (EV board) | **QSPI0** (SPI) | `Qspi/SpiMaster/IfxQspi_SpiMaster.c` | 1 kHz |
| IMU (interim, in code) | MPU‑6050 (GY‑521) | **I2C0** (shared) | `I2c/I2c/IfxI2c_I2c.c` | 50 Hz |
| Baro | BMP388 (BMP581 planned) | **I2C0** (shared) | `I2c/I2c/IfxI2c_I2c.c` | 50 Hz |
| Mag | MMC5983MA | **I2C0** (shared) | (shared) | 100 Hz |
| GNSS | u‑blox NEO‑M9N | **I2C0** (DDC, shared) | (shared) | 10 Hz |

**I2C addresses (no collisions):** MPU‑6050 `0x68`/`0x69`, BMP388 `0x77`/`0x76`
(BMP581 `0x46`/`0x47`), MMC5983MA `0x30`, NEO‑M9N `0x42`.

**Supply — tap the regulated +3V3 rail** (X702·78/80 or X703·70), **not** `VCC_IN`
(X702·5‑8, raw board input 3.5–40 V) and **not** `V_UC` (MCU supply, 5 V here). All
breakouts run at 3.3 V. **IMU EV board:** feed 3.3 V to JP1/JP2 **pin 2**, jumpers left
**open** so the on‑board LDOs stay out (they would otherwise force 3.0 V). CN1: pin 4 CS→P22.11,
pin 16 SCLK→P22.8, pin 18 SDIO→P22.10, pin 20 SDO→P22.9, pin 3 INT1→P22.7.

**Level handling (ports are 5 V VEXT — see board‑wide note & §2.6):**
- **SPI outputs SCLK/MOSI/CS:** 5 V→3.3 V via a **1 kΩ/2 kΩ divider** per line
  (bench‑verified 3.30 V). Fallback: 74LVC1T45. *(Supersedes the earlier buffer‑IC plan.)*
- **SPI input MISO + IMU INT1:** read **directly** via TTL pad —
  `IfxPort_setPinPadDriver(port, pin, IfxPort_PadDriver_ttlSpeed1)` (VIH = 2.0 V). Pulldown 10 kΩ on INT1.
- **I2C SCL/SDA:** **no shifter** — open‑drain + the breakouts' 3.3 V pull‑ups; set the two
  I2C pads to a TTL driver mode. Add external ~4.7 kΩ only if the bus needs strengthening.
- **DShot:** open‑drain ALT1 + 1.5 kΩ pull‑up to 3.3 V (§2.1, §2.6).

**IMU INT relocation:** P20.9 was the first choice but is **ERAY‑B ERRN (R344, board‑fixed,
§4)** → moved to **P22.7** (free, same PERIPHERALS header). Read via TTL pad like MISO.

### 2.6 DShot driver & verification (folded in from the retired DSHOT_PINS.md)

**Single‑pin bidirectional scheme.** Per motor one pad does both: TX = GTM **ATOM0**
channel in PWM mode (DMA‑fed compare values) drives the 16 DShot300 bits; the pad is then
flipped to input and RX = GTM **TIM0** (or TIM7) input‑capture reads the ~375 kbit/s GCR
reply. Cycle = 1 ms. Every P22.0‑3 pad is both ATOM‑out and TIM‑in capable (verified against
`IfxGtm_PinMap_TC39xB_516.h`), so no second pin per motor is needed.

**Direction flip (per cycle).** Only the IOCR direction is toggled:
`IfxPort_setPinModeOutput(&MODULE_P22, n, IfxPort_OutputMode_openDrain, IfxPort_OutputIdx_alt1)`
for TX, `IfxPort_setPinModeInput(..., IfxPort_InputMode_pullUp)` for RX. Port 22 is **not**
P40/P41, so each is a single unprotected `__ldmst` (no ENDINIT sequence) — well under 1 µs,
easily inside the ~30 µs ESC turn‑around.

**Drive electrical.** Open‑drain ALT1 + external **1.5 kΩ pull‑up to 3.3 V** per line. Pad
driver set to **TTL once at init** (`IfxPort_setPinPadDriver(..., ttlSpeed1)`, ENDINIT‑protected
`Pn_PDR`): the 3.3 V reply reads high (VIH = 2.0 V) and V_OL (~0.4 V) stays under VIL(TTL) =
0.8 V. In open‑drain the idle‑high '1' releases the pad, so the RX phase needs no extra drive.

**GTM / resource conflicts — none.** DShot uses **ATOM0** (ch 0/1/3/4) + **TIM0/TIM7**, both
unused elsewhere (the GPIO/PWM feature uses **TOM0/TOM3** on P00). Central `MODULE_DMA` is free
(Ethernet uses the GETH MAC's own DMA engine). Do **not** reprogram **STM0** (CPU0's 1 ms
scheduler). GTM module + FXCLK are already enabled by `gpio.c`; DShot init must only add its
ATOM0/TIM cluster, not re‑init the whole GTM.

**⚠ Open verification before populating all four channels** (project handoff §7.3): bench‑test
that an **LVDS_TX pad sinks correctly in open‑drain while driven by a GTM ATOM in ALT mode** —
on **P22.0** and especially **P22.2/P22.3** (LVDS_TX + HSCT footprint stub, §4 — higher risk).
Confirm pin high = 3.3 V (not 5 V) and V_OL ≤ 0.8 V. **Fallback if it fails:** one **74LVC1T45**
per line (5 V↔3.3 V, push‑pull) instead of open‑drain.

**Why P22 + open‑drain, not the 3.3 V VFLEX pins.** The only native‑3.3 V pins are
P11.13/14/15 (§5.1) — only **three**, and SLOW‑class; a 4‑channel driver can't fit and a 4th
line would fall back to a 5 V pin anyway. So all four ESCs stay on P22 with open‑drain.

---

## 3. Reserved / debug (do not use for application I/O)

| Pin | Function | Header·pin | Note |
|---|---|---|---|
| P20.0 | OCDS / DAP JTAG (miniWiggler) | X701·72 | debug |
| P20.1 | OCDS1 / miniWiggler (R424/R427 opt) | X704·30 | debug |
| P20.2 | /TESTMODE | X702·44 | reset‑strap; R202/R426 |
| P21.6 | DAP3 (R429 assembled) | X701·6 | debug (DAP) |
| P21.7 | ETK (R439 not assembled) | X701·8 | debug |

---

## 4. Board‑fixed pins (TriBoard hardware — `board`, do not reuse)

Source: TriBoard Manual §4.2 Table 4‑3 (assembled 0 Ω bridges) + §6.1 Table 6‑1
(on‑board‑only signals). “onboard” = not brought to any header.

| Pin(s) | Committed to | Bridge / ref | Header·pin |
|---|---|---|---|
| P11.0–P11.3 | RGMII TXD3‑0 | RN302 | onboard |
| P11.4 | RGMII TXC | R373 | onboard |
| P11.5 | RGMII ref‑clk in | R310 | onboard |
| P11.6 | RGMII TXCTL | R374 | onboard |
| P11.7–P11.10 | RGMII RXD3‑0 | RN301 | onboard |
| P11.11 | RGMII RXCTL | R372 | onboard |
| P11.12 | RGMII RXC | R370 | onboard |
| P12.0 | Ethernet MDC | R376 | X704·37 |
| P12.1 | Ethernet MDIO | R375 | X704·39 |
| P20.7 | CAN0 RXD | R301 | X702·69 |
| P20.8 | CAN0 TXD | R302 | X702·70 |
| P23.0 | CAN1 RXD | R311 | X704·36 |
| P23.1 | CAN1 TXD | R312 | X702·28 |
| P14.5 | ERAY‑B TXD (+HWCFG1) | R340 | onboard |
| P14.6 | ERAY‑B TXDEN | R341 | onboard |
| P14.7 | ERAY‑B RXD | R342 | X704·18 |
| P20.9 | ERAY‑B ERRN | R344 | X702·37 |
| P20.10 | ERAY‑B EN | R343 | X702·38 |
| P14.8 | ERAY‑A RXD | R322 | X704·20 |
| P14.9 | ERAY‑A TXDEN | R321 | X704·22 |
| P14.10 | ERAY‑A TXD | R320 | X704·24 |
| P32.2 | ERAY‑A ERRN | R324 | X704·50 |
| P32.3 | ERAY‑A EN | R323 | X704·52 |
| P15.0 | LIN1 TXD | R365 | X702·62 |
| P15.1 | LIN1 RXD | R364 | X702·60 |
| P15.4 | I2C EEPROM SCL | R348 | X702·58 |
| P15.5 | I2C EEPROM SDA | R349 | X702·56 |
| P14.2 | TLF35584 SLSO21 / HWCFG2 (QSPI2) | — | onboard |
| P14.3 | **TLF35584 WDI** (watchdog trigger) / HWCFG3 | — | X703·78 |
| P14.4 | HWCFG6 (default pad state: tristate/pull‑up) | — | onboard |
| P15.3 | TLF35584 (QSPI2) | — | X702·55 |
| P15.6 | TLF35584 (QSPI2) | — | X704·26 |
| P15.7 | TLF35584 (QSPI2) | — | X704·28 |
| P32.0 | EVRC VGATE1N | — | onboard |
| P32.1 | EVRC VGATE1P | — | onboard |
| P33.4 | LED D302 (standby/VEVRSB) | R395 (470 Ω) | X703·68 |
| P33.5 | LED D303 (standby/VEVRSB) | R396 (470 Ω) | X702·41 |
| P33.6 | LED D304 + **DAP_SCR line 0** | R397 (470 Ω) | X703·64 |
| P33.7 | LED D305 + **DAP_SCR line 1** | R398 (470 Ω) | X703·77 |
| P33.8 | **TLF35584 safety: SMU_FSP0 → error input** | — | X703·56 |
| P33.9 | **TLF35584 safety: Safe‑State‑1 input** | — | X703·58 |
| P33.10 | **TLF35584 safety: Wake/Inhibit output** | — | X703·60 |
| P33.11 | **Button S202** (+ standby wake of TLF35584) | S202 | X703·79 |

Notes:
- **P33.4–P33.7 drive on‑board LEDs D302–D305** (Manual §3.4; 470 Ω R395–R398, supply
  bridge R390). Usable as *outputs* (the LED just lights); **not usable as inputs**.
  P33.6/P33.7 are additionally the **DAP_SCR** standby debug access (Manual §3.18.4).
  Port 33 is on the **VEVRSB** standby rail (see top note).
- **HWCFG boot straps** (Manual Table 5‑3, DIP switch S201, sampled at reset):
  **P14.5** (HWCFG1), **P14.2** (HWCFG2), **P14.3** (HWCFG3), **P10.5** (HWCFG4),
  **P10.6** (HWCFG5), **P14.4** (HWCFG6). External circuitry on P10.5 (X702·23) or
  P10.6 (X704·35) fights the switch/pull‑up (R231, 47 kΩ) and **can alter the boot
  mode** — usable only after reset, and only carefully.
- **TLF35584 watchdog = P14.3** (resolved). The Manual contradicts itself — Table 5‑6
  says “P14.3 Output for Watchdog Input”, Table 6‑1 says “P14.4 … Output to Watchdog
  Input”. The schematic **Supply.sch** (Manual p. 7‑6) settles it: the net **P14.3 → pin
  16 (WDI) of U501 (TLF35584)**. So P14.3 is the live watchdog trigger; P14.4 is only
  HWCFG6. Both are board‑committed either way — do not reuse.

> Footprint‑only (not assembled → electrically free but wired to a part pad; avoid
> unless you populate/depopulate deliberately):
> - P00.0/P00.1 (CAN1 alt R313/R314), P02.0/1/4 & P10.1/2 (ERAY‑A alt),
>   P14.0/1 (CAN0 alt R303/R304 — already console), P15.2/P15.3 (USB‑UART alt
>   R438/R440), P15.8 (Eth MDINT R377).
> - **HSCT / IEEE‑1394 socket stubs (X201/X202, unpopulated)** — Manual §3.13:
>   **P20.0, P21.0–P21.5, P22.2, P22.3** connect to the HSCT footprints via
>   R250–R254 / R257–R260. Functionally free, but each carries a short stub to the
>   empty socket. **This includes DShot pins P22.2/P22.3** and explains their
>   `LVDS_TX` pad class — exactly what the §2.6 open‑drain bench test must confirm
>   on silicon.

---

## 5. Free pins on the two primary headers (the pool)

Unallocated and clear of §3/§4. Every pin below is **GTM‑capable — has both an ATOM
output and a TIM input** (verified against `IfxGtm_PinMap_TC39xB_516.h`). Voltage =
5 V VEXT (§5.1 lists the 3.3 V exceptions). Pick DShot spares, extra sensors, or GPIO
from here.

> **Timer‑channel collisions with the DShot allocation (§2.1).**
> DShot occupies **ATOM0 ch{0,1,3,4}** (TX) and **TIM0 ch{0,1,3,4}** (RX; or TIM7
> ch{0,1,2,3} in the alt mapping). The ATOM/TIM columns below give a **non‑colliding**
> instance/channel per pin; ⚠ marks a pin whose *default* iLLD object would clash if
> chosen naively (pick the listed alternate, or move DShot RX to TIM7):
> - **P10.7** default TIM0.0 = DShot M1 → use **TIM1.0**.
> - **P10.3** default TIM0.3 = DShot M3 → use **TIM1.3 / TIM4.6**.
> - **P20.3** default ATOM0.4 = DShot M4 → use **ATOM1.4 / ATOM2.7N**.
> - **P23.4** ATOM0.3N shares ch3 (M3) → use **ATOM0.7 / ATOM1.7**.

### PERIPHERALS (X702/X802)

| Pin | Header·pin | ATOM (safe) | TIM (safe) | Notes |
|---|---|---|---|---|
| P22.4 | 9 | ATOM5.0 | TIM3.0 | ASCLIN7 RXF |
| P22.5 | 11 | ATOM5.1 | TIM3.1 | — |
| P22.6 | 13 | ATOM5.2 | TIM2.6 / TIM3.2 | ASCLIN4 RXC |
| P13.0 | 31 | ATOM2.5 | TIM2.5 / TIM3.5 | ASCLIN10 RXC |
| P13.3 | 33 | ATOM2.0 | TIM2.0 / TIM3.0 | — |
| P20.3 | 43 | ATOM1.4 ⚠ | TIM3.4 / TIM4.5 | ⚠ avoid ATOM0.4 (=M4); ASCLIN3 RXC |
| P20.6 | 42 | ATOM2.6 / ATOM3.6 | TIM6.0 | ASCLIN9 RXE |
| P32.4 | 52 | ATOM0.5 / ATOM1.5 | TIM0.5 / TIM1.5 | ch5 — no DShot clash |
| P23.4 | 26 | ATOM0.7 ⚠ | TIM6.3 | ⚠ avoid ATOM0.3N (ch3=M3) |
| P10.3 | 74 | ATOM1.3 | TIM1.3 ⚠ | ⚠ avoid TIM0.3 (=M3) |
| P10.4 | 25 | ATOM0.6 / ATOM1.6 | TIM1.6 / TIM4.7 | ch6 — no DShot clash; ASCLIN11 RXB |
| P10.7 | 73 | ATOM1.0 | TIM1.0 ⚠ | ⚠ avoid TIM0.0 (=M1) |
| P10.8 | 72 | ATOM1.5 | TIM4.0 / TIM1.5 | — |
| P15.2 | 57 | ATOM1.5 / ATOM4.5 | TIM2.5 / TIM3.5 | footprint of USB‑UART alt (not assembled) |
| P15.8 | 71 | ATOM4.1 | TIM0.2 / TIM1.2 | footprint of Eth MDINT (not assembled) |

> **ATOM-Instanz P22.4/5/6:** die ATOM-Spalte oben nennt je *eine* gültige Instanz
> (ATOM5.0/5.1/5.2 = TOUT130/131/132, verifiziert in `IfxGtm_PinMap_TC39xB_516.h`
> Z.801/821/841). Dieselben Pads besitzen weitere gültige ATOM-Objekte —
> ATOM1.4/ATOM3.4/ATOM3.5 (identisches TOUT, andere Instanz, Z.393/631/647).
> Kein Widerspruch, nur alternative Routings.

**Removed from the pool** (were listed here, are NOT free): **P33.5** → LED D303 (§4);
**P10.5** → HWCFG4 boot strap (§4 note — conditionally usable after reset only).

### 5.1 Free **VFLEX (3.3 V)** pins — native 3.3 V, no level shifter

The only free 3.3 V pins on the board. All Port 11, all `VFLEX / ES` per Data Sheet
Table 2‑63, all **SLOW** pad class (edge‑rate/drive limited — verify for fast signals).
On the **GTM/PORTS header X704/X804**.

| Pin | Header·pin | ATOM | TIM | Ref |
|---|---|---|---|---|
| P11.14 | X704·6 | ATOM5.7 (TOUT126) | TIM2.7 / TIM4.7 | Table 2‑63 pad 95 |
| P11.13 | X704·67 | ATOM5.6 (TOUT125) | TIM2.6 / TIM4.6 | Table 2‑63 pad 96 |
| P11.15 | X704·8 | ATOM6.0 (TOUT127) | TIM0.7 / TIM4.7 | Table 2‑63 pad 97 |

Only **three** exist (P11.0–P11.12 are RGMII, board‑committed) — not enough for a
4‑channel native‑3.3 V DShot. None of their channels clash with the DShot ATOM0/TIM0
block. See §2.6 for why DShot stays on P22 + open‑drain rather than moving here.

### ⚠ Do NOT land a header on these X702/X802 positions

Same connector as the sensors/DShot — a shifted ribbon here is expensive:

| Pos. | Signal | Pos. | Signal |
|---|---|---|---|
| 15 / 16 | /ESR1 · /ESR0 (emergency stop) | 22 | /PORST (power‑on reset) |
| 51 / 53 | XTAL1 · XTAL2 (oscillator) | **75** | **VDDSB (1.25 V standby, emu‑SRAM)** — GND next on 76 |

Supply taps on X702/X802: **GND** 1‑4, 19, 20, 49, 50, 76 · **VCC_IN** 5‑8 ·
**VDDSB** 75 · **VEXT** 77, 79 · **+3V3** 78, 80.

### GTM/PORTS (X704/X804) — more free GTM pins

Free VFLEX (3.3 V): **P11.13** (·67), **P11.14** (·6), **P11.15** (·8) — see §5.1.
Free VEXT (5 V): P02.x, P01.x, P34.1–P34.5, P23.2/5/6/7, P32.5/6/7
… (see TriBoard Manual §6.2 Fig. 6‑2). Use when Port 22 / PERIPHERALS is full.

> **⚠ Port 33 is mostly NOT free (corrected):** **P33.8/9/10 are the TLF35584 safety
> interface** (SMU_FSP0 error / Safe‑State‑1 / Wake‑Inhibit — Manual Table 6‑1) and
> **P33.11 is button S202** — all board‑committed, see §4. **P33.12–P33.15** are routed to
> X704/X703 but their board function is not yet verified — **confirm before use.** Earlier
> versions wrongly listed "P33.8–P33.15" as free VEXT here.

> **P25.0/P25.1 are NOT on X704** — Fig. 6‑1 puts them on **BUS‑EXPANSION X701/X801**
> (P25.1 = X701·69, P25.0 = X701·78). They are EBU signals (BFCLKO, /RD); free only
> because no external memory is populated. (Corrected from an earlier X704 listing.)
Note: **P10.6 (X704·35) is HWCFG5** — boot strap, same caveat as P10.5.

---

## 6. Sources

| Topic | Source |
|---|---|
| LEDs, console, GPIO/PWM, Diag pin | `src/bsw/Cpu{0..3}_Main.c`, `src/bsw/Uart.c`, `src/bsw/gpio_cfg.h` |
| DShot pins, driver, level shifting, ADC/telemetry (§2.1/§2.4/§2.6) | folded in from retired `DSHOT_PINS.md`; verified against iLLD `IfxGtm_PinMap_TC39xB_516.*`, Data Sheet Table 2‑12, TriBoard Manual Fig. 6‑1 |
| IMU / I2C sensor pins & electrical (§2.5) | folded in from retired `SENSORS.md`; bench‑verified 1 kΩ/2 kΩ divider + IMU 3.3 V feed per project handoff |
| P33.8/9/10 = TLF35584 safety interface; P33.11 = button S202 | TriBoard Manual **Table 6‑1** (P33.8 SMU_FSP0, P33.9 Safe‑State‑1, P33.10 Wake/Inhibit); §3.15 (S202) |
| Connector pinouts (all header·pin numbers) | TriBoard Manual V2.1, §6.2 Figs. 6‑1/6‑2/6‑3 |
| Board‑fixed bridges / transceivers | TriBoard Manual V2.1, §4.2 Table 4‑3; §6.1 Table 6‑1 |
| P33.4‑7 LEDs D302‑D305 / DAP_SCR | TriBoard Manual §3.4, §3.18.4; Table 5‑6 |
| Port 33 = VEVRSB standby rail | TriBoard Manual §3.16 |
| HWCFG boot straps (P14.5/2/3, P10.5/6, P14.4) | TriBoard Manual Table 5‑3; §4.1.2; DIP S201 |
| P14.3 = TLF35584 WDI (watchdog) — resolves Tbl 5‑6 vs 6‑1 conflict | Schematic **Supply.sch** (Manual p. 7‑6): net P14.3 → U501 pin 16 WDI |
| VFLEX = Port 11 only; P11.13/14/15 free 3.3 V; P14/P15/P20 pins are VEXT | Data Sheet V1.2 **Table 2‑63 Pad List** (`VFLEX / ES` vs `VEXT / ES` per pad; pads 95/96/97) |
| P11.13/14/15 SLOW pad class | Data Sheet Table 2‑63 (`SLOW / PU1`) |
| 6 filtered ADC inputs (47 nF + 4.7 kΩ); VAREF=5 V | TriBoard Manual §3.14, Fig. 3‑4; R220/R222 |
| P25.0/P25.1 on X701 (not X704); EBU signals | TriBoard Manual §6.2 Fig. 6‑1 |
| HSCT footprint stubs (P20.0/P21.0‑5/P22.2/P22.3) | TriBoard Manual §3.13 (X201/X202, R250‑R254/R257‑R260) |
| ATOM/TIM/ASCLIN/EVADC pin objects | `Libraries/iLLD/.../_PinMap/TC39xB/IfxGtm_PinMap_TC39xB_516.*`, `IfxAsclin_PinMap_TC39xB_516.h`; Data Sheet V1.2 (analog pin defs pp. 162‑165) |
