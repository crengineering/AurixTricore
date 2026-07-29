# SENSORS.md — Sensor Integration Feasibility (Phase 0)

**Purpose:** answer *"is adding the four flight sensors to the TC399 possible?"*
**before** ordering parts. Covers device support, pinout, electrical, timing budget,
and a go/no-go. Companion: the full plan and the Simulink strand
(`../../Quadrocopter/doc/MBD_PATH.md`).

Board: **TriBoard TC3X9 (TH) V2.0**, device **TC399 B-step, LFBGA-516**.
Toolchain: TASKING (AURIX Development Studio). Ethernet/lwIP/XCP already run on CPU0.

> **Pin allocation SSoT:** the consolidated, authoritative pin map for the whole
> project is **`docs/PINNING.md`**. This document is the *rationale* for the sensor
> pins; when you (re)allocate a pin, update `PINNING.md` too.

---

## 1. Sensors & bus assignment

| Role | Part | Bus | iLLD driver (present, unused) | Rate |
|---|---|---|---|---|
| IMU | ICM-42688-P (6-axis) | **SPI** — QSPI0 | `Qspi/SpiMaster/IfxQspi_SpiMaster.c` | 1 kHz |
| Baro | Bosch BMP581 | **I2C0** | `I2c/I2c/IfxI2c_I2c.c` | 50 Hz |
| Mag | MMC5983MA | **I2C0** | (shared) | 100 Hz |
| GNSS | u-blox NEO-M9N | **I2C0** (DDC 0x42) | (shared) | 10 Hz |

All four bus drivers already ship in the in-tree iLLD (`Libraries/iLLD/TC3xx/
Tricore/{Qspi,I2c}`), so **no new low-level stack is required** — only thin BSW
wrappers + per-sensor register drivers.

## 2. Pinout — single connector: PERIPHERALS header (X702 / X802)

From the iLLD LFBGA-516 pin-map and the TriBoard manual §6.2. Every sensor pin is
on **one** header, so a single ribbon/breakout carries all wiring.

**IMU — SPI (QSPI0), Port 22:**

| Signal | Pin | Header pin | iLLD pin object |
|---|---|---|---|
| SCLK | P22.8 | 10 | `IfxQspi0_SCLK_P22_8` |
| MRST (MISO) | P22.9 | 12 | `IfxQspi0_MRST_P22_9` |
| MTSR (MOSI) | P22.10 | 14 | `IfxQspi0_MTSR_P22_10` |
| CS (SLSO10) | P22.11 | 18 | `IfxQspi0_SLSO10_P22_11` |
| IMU INT1 (optional, data-ready) | **P22.7** | 17 | GPIO input (P20.9 is taken — see §3) |

**Baro + Mag + GNSS — shared I2C0, Port 13:**

| Signal | Pin | Header pin | iLLD pin object |
|---|---|---|---|
| SCL | P13.1 | 29 | `IfxI2c0_SCL_P13_1` |
| SDA | P13.2 | 35 | `IfxI2c0_SDA_P13_2` |

**Power/GND (same header):** `GND` pins 1–4, 19, 20. Do **not** power the sensors
from the header's `VCC_IN` pins (5–8) — `VCC_IN` is the **raw board input, 3.5–40 V**
(manual §, "Supply Input 3,5V…40V"), and would destroy the sensors. Also avoid
`V_UC`: it is the MCU supply and is **5 V or 3.3 V depending on the fitted TLF35584
variant** — not guaranteed 3.3 V. Instead tap the board's dedicated **regulated +3V3
rail** (green LED D506 "+3V3", LDO-fed, also feeds the Ethernet PHY) with a jumper
wire to the sensors' VCC. All four breakouts run at 3.3 V.

**I2C addresses (no collisions):** BMP581 `0x46`/`0x47`, MMC5983MA `0x30`,
NEO-M9N `0x42`.

## 3. Electrical notes / to-verify on the bench

- **I2C pull-ups:** I2C is open-drain and needs pull-ups **once per bus**. Port 13
  has none on-board, **but the breakouts do**: Adafruit BMP581 (STEMMA QT, ~10 kΩ)
  and SparkFun NEO-M9N (Qwiic, ~2.2 kΩ) already pull the bus up — the bare MMC5983-B
  proto board does not. So **wire it up first**; two Qwiic/STEMMA boards already give
  ~1.8 kΩ effective (plenty). Only add external resistors (~4.7 kΩ, or 2.2 kΩ for a
  long/fast bus) if the effective pull-up is too weak — unlikely here. Keep the
  combined parallel pull-up roughly in the 1–4.7 kΩ range (≤ ~3 mA sink at 3.3 V).
- **Port I/O = 5 V on this board — now fully characterised and resolved (below).**
  The ports are the **VEXT** domain; VEXT is tied to `V_UC` by bridge **R527** (default,
  manual **Table 4-2, p.4-2**), and `V_UC = 5 V` on the fitted **TLF35584QVVS1** (manual
  **§3.3, p.3-1**). Bench-confirmed: P00.1 high = **5 V**. Datasheet: VEXT = "External
  Power Supply (5 V / 3.3 V)" (**TC39x DS V1.2**, pin/supply table). **All sensor pins
  are VEXT:** P22.7–P22.11 (SPI + INT) and P13.1/P13.2 (I2C) are **not** in the VFLEX
  Flex-port group — so they follow VEXT = 5 V.
  *(Correction: the VFLEX group is **Port 11 only** — Data Sheet Table 2‑63 tags each
  P11.x pad `VFLEX / ES`. The earlier list here — P14.7/9/10, P15.7/8, P20.12/13 — is
  wrong; those pads are tagged `VEXT / ES` (5 V). This does not change the conclusion:
  the sensor pins are VEXT and read a 3.3 V high via the TTL threshold. See
  `PINNING.md` §5.1 and `DSHOT_PINS.md` §7.)*
- **Inputs — SOLVED by TTL pad level (no shifter on any MCU input).** These are
  "5 V / 3.3 V switchable Fast 5V GPIO" pads with a selectable **TTL input threshold:
  VIH(min) = 2.0 V at VEXT = 4.5 V** (**TC39x DS V1.2**, Table "Fast 5V GPIO", p.413).
  A 3.3 V sensor high (≥ 2.0 V) is therefore read reliably as logic-high **even at
  VEXT = 5 V**. (In the default **Automotive** level VIH = 0.7·VEXT ≈ 3.5 V, which 3.3 V
  would fail — so the pads MUST be switched to TTL.) iLLD does this per pin via
  **`IfxPort_setPinPadDriver(port, pin, IfxPort_PadDriver_ttlSpeed1)`** (enum
  `ttlSpeed1..4` / `ttl3v3Speed1..4`, `IfxPort.h`). Apply TTL to MRST/MISO, the IMU INT,
  and the two I2C pads.
- **I2C (baro/mag/GNSS) — NO level shifter needed.** I2C pads are open-drain (iLLD
  `IfxI2c` drives them open-drain); with the breakouts' 3.3 V pull-ups, SDA/SCL never
  exceed 3.3 V → **no over-voltage to the sensors**, and the TTL input reads the 3.3 V
  highs. Just set the two I2C pads to a TTL pad-driver mode. **V_IH is resolved, not open.**
- **SPI (IMU) — the ONE remaining issue: outputs swing to 5 V.** SCLK/MOSI/CS are
  push-pull; their high level = VEXT = 5 V and cannot be lowered independently of VEXT.
  5 V on the ICM-42688-P's 3.3 V inputs exceeds its abs-max (≈ VDDIO+0.3 ≈ 3.6 V) and
  would damage it. Fix = a small **3-line 5 V→3.3 V buffer** on SCLK/MOSI/CS (fast,
  e.g. 74LVC1T45 ×3 or a single 74LVC244A). **MISO stays 3.3 V and is read via TTL — no
  shifting.** So the only added part is a 3-channel down-buffer.
  **Alternative — R527→R526 rebuild** ("Connect VEBU/VEXT to +3V3", Table 4-2): makes
  all VEXT ports 3.3 V and removes even the SPI buffer. Sanctioned config (default with
  external memory; only caveat "not usable with TC389" — device restriction, not TC399),
  **but verify** (not confirmable from in-repo docs): +3V3 LDO current headroom (LDO
  part/rating not in manual — needs Supply.sch), debug level (OCDS1 "work with the port
  supply … +5 V", manual **§3.18.1**), and removal of any 0 Ω transceiver links feeding a
  used pin ("must be removed to use the ports outside", manual **§3.8**).
- **⚠ P20.9 is NOT free (corrected).** **R344 (assembled by default) connects P20.9 to
  ERRN of the ERAY-B (FlexRay) transceiver** (manual transceiver R-table; **§3.8**
  "error state … read out via P20.9"; signal list "P20.9 = E-Ray Channel B Error
  Input"). Use **P22.7 (header pin 17)** for the IMU data-ready INT — free, same
  PERIPHERALS header, next to the SPI block (P22.4–P22.7 show no onboard transceiver
  links in the manual R-tables). Being a sensor→MCU input, it needs the same
  V_IH/level handling as MISO.
- **Occupied pins avoided:** QSPI2 (TLF35584 safety supply, P14.2/P15.3/P15.6/P15.7),
  ASCLIN0 USB console (P14.0/P14.1), LIN (P15.0/P15.1).
- **Ethernet conflict: NONE.** RGMII is hard-wired to P11.0–P11.12 and the manual
  states those pins are **not routed to any connector** — collision is impossible.

## 4. Timing budget

**SPI (IMU), QSPI0 @ 10 MHz, 1 kHz sampling:** accel+gyro burst ≈ 14 bytes ≈
**11 µs/read** → **~1.1 %** of the 1 ms tick (≈0.5 % at 24 MHz). Dedicated bus.

**I2C0 @ 400 kHz (Fast-mode), shared:**

| Sensor | Bytes/read | Time/read | Rate | Bus load |
|---|---|---|---|---|
| BMP581 (P+T) | ~9 | ~0.20 ms | 50 Hz | ~1.0 % |
| MMC5983 (3×18-bit) | ~10 | ~0.22 ms | 100 Hz | ~2.3 % |
| NEO-M9N (UBX-NAV-PVT ~100 B) | ~108 | ~2.4 ms | 10 Hz | ~2.4 % |
| **Total I2C0** | | | | **~5.7 %** |

**CPU:** the verified controller uses only single-digit-µs per 1 ms tick (PIL
`t_exec` stats). The complementary-filter estimator at 1 kHz (quaternion update +
accel/mag corrections) is ~5–20 µs on the 300 MHz TriCore FPU. Combined headroom is
large — well under 20 % of one core, and the estimator runs on **CPU1** (CPU0 keeps
comms).

**Verdict:** every bus fits with wide margin; no rate is close to saturating.

## 5. Risks / open checks

| Item | Status |
|---|---|
| iLLD QSPI/I2C driver availability | ✅ present in-tree |
| Single-connector wiring | ✅ all on PERIPHERALS header |
| Ethernet pin conflict | ✅ none (RGMII P11-only, off-connector) |
| Timing budget | ✅ ample margin |
| Port I/O = 5 V (VEXT domain) | ✅ resolved — TTL input pads for all reads; 3-line 5V→3.3V buffer on SPI SCLK/MOSI/CS (or R526 rebuild). See §3 |
| Input threshold V_IH | ✅ resolved — TTL VIH(min)=2.0 V @4.5 V (DS V1.2 p.413); 3.3 V reads high at VEXT=5 V |
| I2C over-voltage / shifter | ✅ none needed — open-drain, lines ≤3.3 V, TTL reads them |
| P20.9 free for IMU INT | ❌ taken (FlexRay ERAY-B ERRN, R344) → moved to **P22.7** (§3) |
| Build clean under TASKING + MISRA | ⬜ pending (write BSW skeletons, build in ADS) |
| QSPI loopback / I2C no-ACK bench test | ⬜ pending (needs board, no sensors) |
| 3V3 sensor rail | ⬜ wiring task (tap +3V3, not VCC_IN/V_UC) |

## 6. Go / no-go

**GO.** Bus support, pinout, single-connector wiring, Ethernet non-conflict and timing
all hold, and the 5 V port-level issue (which invalidated the first draft's naive
"I/O is 3.3 V" claim) now has a confirmed, minimal solution:

- **I2C (baro/mag/GNSS):** nothing added — open-drain + breakout 3.3 V pull-ups + TTL
  input pads.
- **SPI (IMU):** one **3-channel 5 V→3.3 V buffer** on SCLK/MOSI/CS; MISO read via TTL.
- **Pad config:** set MRST/INT/SDA/SCL pads to `IfxPort_PadDriver_ttlSpeed*`.
- **IMU INT** moved P20.9 → **P22.7** (P20.9 is FlexRay ERRN).
- **Optional:** the R527→R526 rebuild eliminates the SPI buffer if you'd rather have an
  all-3.3 V board (verify LDO/debug/transceiver-link items in §3 first).

All electrical facts are datasheet/manual-sourced (§3). Remaining Phase-0 items
(build-clean skeletons + bench loopback) need only the board + toolchain.

## 7. Order list (breakouts already selected)

- TDK **ICM-42688-P** eval board (EV_ICM-42688-P) — SPI IMU
- Adafruit **BMP581** (PID 6407, STEMMA QT / I2C)
- MEMSIC **MMC5983MA** proto board (MMC5983-B) — I2C
- SparkFun **NEO-M9N** GPS breakout (GPS-15733, Qwiic / I2C)
- Passives: wiring to the PERIPHERALS header + a jumper to the board's +3V3 rail.
  I2C pull-ups optional — the Qwiic/STEMMA breakouts already include them; keep a
  couple of 4.7 kΩ on hand only in case the bus needs strengthening.
- **Level translation — SPI outputs only** (I2C needs none; see §3):
  - **SPI (IMU):** one **3-channel 5 V→3.3 V buffer** on SCLK/MOSI/CS — e.g. 3×
    74LVC1T45 (direction-fixed) or a single 74LVC244A. MISO/INT need no part (TTL pad).
  - Not needed: I2C translator (open-drain + breakout pull-ups + TTL input).
  - **GTM/DShot (later, MCU→ESC):** ESCs generally accept a 3.3–5 V DShot input, so the
    5 V output is usually fine — confirm per ESC; add a 74LVC1T45 per line only if the
    ESC needs 3.3 V.
  - **Alternative:** the R527→R526 rebuild makes all ports 3.3 V → even the SPI buffer
    is unnecessary (trade: board rework + LDO/debug/transceiver checks in §3).

## 8. Documentation status

- ✅ `infineon-triboardmanual-tc3x9-um-en.pdf` — board manual (cited above).
- ✅ `infineon-tc39x-datasheet-en.pdf` — **TC39x Data Sheet V1.2 (2021-03), BC/BD-Step**
  — the electrical source: pad V_IH ("Fast 5V GPIO", p.413) and per-pin supply domains.
- ➖ `infineon-tc39x-bd-step-datasheet-addendum-datasheet-en.pdf` — variants addendum
  (memory/feature variants only); not electrically relevant.
- ➖ `infineon-aurix-tc39x-bd-step-erratasheet-en.pdf` — errata (silicon deviations).
- ❌ Still missing (only needed IF doing the R526 rebuild): board **schematics**
  (Supply.sch for the +3V3 LDO part/rating, Wiggler_Dbg_con.sch for the debug-level
  reference).
