# GNSS — u-blox UBX protocol (NEO-M9N)

Source: `docs/NEO-M9N_InterfaceDescription_UBX-19035940.pdf` (R03, interface
version **32.01**, 247 pages) — §3 UBX protocol, §5 Configuration interface.
Distilled 2026-08-20. Re-extract if the receiver's protocol version changes.

Driver: `src/bsw/GnssM9N.c` (ASCLIN4, 38400 8N1). Wiring: `docs/PINNING.md` §2.7.

> The PDF is gitignored (`docs/*.pdf`). Re-download if missing; u-blox's own
> `content.u-blox.com` path 404s, the working copy came from a mirror.

---

## 1. Frame format (§3.2, §3.4)

```
B5 62 | class | id | len_lo len_hi | payload[len] | CK_A CK_B
```

| part | size | notes |
|---|---|---|
| sync | 2 | always `0xB5 0x62` |
| class | 1 | message family |
| id | 1 | message within the family |
| length | 2 | **U2, little-endian, payload only** — excludes sync, class, id, length and checksum |
| payload | *len* | |
| checksum | 2 | `CK_A`, `CK_B` |

**Checksum = 8-bit Fletcher (RFC 1145).** Runs from the **class field up to but
excluding the checksum** — i.e. class, id, both length bytes, payload. **Not the
sync bytes.**

```
CK_A = 0, CK_B = 0
for (I = 0; I < N; I++) {
    CK_A = CK_A + Buffer[I]
    CK_B = CK_B + CK_A
}
```

⚠️ The doc says it explicitly: both are **8-bit unsigned only**. With wider
integers you must mask with `0xff` after *both* operations in the loop.

Reference frame — `CFG-VALSET`, RAM layer, `CFG-MSGOUT-NMEA_ID_GSV_UART1 = 0`
(use as a unit-test golden value):

```
B5 62 06 8A 09 00 00 01 00 00 C5 00 91 20 00 10 FD
```

### Data types (§3.3.5)

`U1 U2 U4 U8` unsigned LE · `I1 I2 I4 I8` signed LE two's complement ·
`X1 X2 X4` bitfields · `R4 R8` IEEE-754 · `E1 E2 E4` enums · `L` bool as U1 ·
`CH` ASCII/ISO-8859-1.

§3.3.1 guarantees **natural alignment inside payloads**: 2-byte values start on
a multiple of 2, 4-byte on a multiple of 4. So NAV-PVT can be decoded by offset
without worrying about packing — but see the TASKING trap in §7.

§3.3.6: a *scale* is the smallest storage unit. `1e-7 deg` means the stored I4
is degrees × 10⁷.

---

## 2. Message classes (§3.8)

| class | family |
|---|---|
| `0x01` | NAV — navigation results |
| `0x02` | RXM — receiver manager |
| `0x04` | INF — text/information |
| `0x05` | ACK — acknowledgements |
| `0x06` | CFG — configuration |
| `0x09` | UPD — firmware update |
| `0x0A` | MON — monitoring |
| `0x0D` | TIM — timing |
| `0x27` | SEC — security |

Messages used here:

| message | class/id |
|---|---|
| UBX-ACK-ACK | `05 01` |
| UBX-ACK-NAK | `05 00` |
| UBX-CFG-VALSET | `06 8A` |
| UBX-CFG-VALGET | `06 8B` |
| UBX-CFG-VALDEL | `06 8C` |
| UBX-NAV-PVT | `01 07` |

### ACK / NAK (§3.9)

Payload is **2 bytes** for both:

| offset | type | name |
|---|---|---|
| 0 | U1 | `clsID` — class of the acknowledged message |
| 1 | U1 | `msgID` — id of the acknowledged message |

§3.5.1: **every CFG message sent to the receiver gets an ACK-ACK or ACK-NAK
back.** That is the definitive test that a config frame was received *and*
parsed — far better than inferring it from traffic changes.

The two complete frames to hunt for in the RX stream after sending a
`CFG-VALSET` (checksums computed, verified):

```
ACK  B5 62 05 01 02 00 06 8A 98 C1
NAK  B5 62 05 00 02 00 06 8A 97 BC
```

---

## 3. CFG-VALSET (§3.10.28)

Payload = 4-byte header + one or more key/value pairs:

| offset | type | name | value |
|---|---|---|---|
| 0 | U1 | `version` | `0x00` (no transaction) |
| 1 | X1 | `layers` | bit0 `ram`, bit1 `bbr`, bit2 `flash` |
| 2 | U1[2] | `reserved0` | `0x00 0x00` |
| 4+ | U1[] | `cfgData` | key (4 B, LE) + value (1/2/4/8 B, LE), repeated |

Limit: **max 64 key/value pairs per message.**

**Returns ACK-NAK and applies nothing if:**
- any key is unknown to the firmware
- the `layers` bitfield names no layer
- the configuration is invalid *(validity is only checked when RAM is targeted)*

If the same key appears twice in one message, the **last** value wins.

### Layers (§5.3), highest priority first

| layer | bit | persistence |
|---|---|---|
| RAM | `0x01` | volatile, **effective immediately** |
| BBR | `0x02` | battery-backed, effective **after restart** |
| Flash | `0x04` | permanent, effective **after restart**; only if external flash fitted |
| Default | — | read-only, hard-coded |

⚠️ **Use RAM (`0x01`) alone while developing.** A wrong value is undone by a
power cycle. Writing a baud-rate key to BBR/Flash while the firmware hardcodes
38400 orphans the receiver — that is the classic way to brick one of these.

---

## 4. Configuration key ID encoding (§5.2)

```
bit  31        30..28        27..24     23..16      15..12    11..0
     reserved  storage size  reserved   group ID    reserved  item ID
```

⚠️ Note the **two reserved gaps** — the group is bits 23..16 (8 bits) and the
item is bits 11..0 (12 bits), *not* one contiguous 12/16 split.

Storage size IDs (bits 30..28):

| id | value size |
|---|---|
| `0x01` | one bit (stored in a byte, LSB used) |
| `0x02` | one byte |
| `0x03` | two bytes |
| `0x04` | four bytes |
| `0x05` | eight bytes |

Worked example — `0x209100C5`:

```
0x209100C5 = 0010 0000 1001 0001 0000 0000 1100 0101
              ^^^           ^^^^ ^^^^           (group 0x91 = MSGOUT)
              size 2 = 1 byte        item 0x0C5
```

**Free self-check:** the key states its own value size, so the payload length is
`4 + 4 + size`. If your computed length disagrees with the key's size field, the
key is wrong. All `CFG-MSGOUT-*` keys are `0x2091xxxx` → always a 1-byte value →
always a 9-byte VALSET payload.

---

## 5. Keys used by this project

### CFG-MSGOUT — output rate on UART1 (§5.9.12)

All `U1`. Value = output rate in navigation epochs (0 = off, 1 = every epoch,
2 = every other, …).

| key | ID |
|---|---|
| `CFG-MSGOUT-UBX_NAV_PVT_UART1` | `0x20910007` |
| `CFG-MSGOUT-UBX_NAV_SAT_UART1` | `0x20910016` |
| `CFG-MSGOUT-NMEA_ID_DTM_UART1` | `0x209100A7` |
| `CFG-MSGOUT-NMEA_ID_GBS_UART1` | `0x209100DE` |
| `CFG-MSGOUT-NMEA_ID_GGA_UART1` | `0x209100BB` |
| `CFG-MSGOUT-NMEA_ID_GLL_UART1` | `0x209100CA` |
| `CFG-MSGOUT-NMEA_ID_GNS_UART1` | `0x209100B6` |
| `CFG-MSGOUT-NMEA_ID_GRS_UART1` | `0x209100CF` |
| `CFG-MSGOUT-NMEA_ID_GSA_UART1` | `0x209100C0` |
| `CFG-MSGOUT-NMEA_ID_GST_UART1` | `0x209100D4` |
| `CFG-MSGOUT-NMEA_ID_GSV_UART1` | `0x209100C5` |
| `CFG-MSGOUT-NMEA_ID_RMC_UART1` | `0x209100AC` |
| `CFG-MSGOUT-NMEA_ID_VLW_UART1` | `0x209100E8` |
| `CFG-MSGOUT-NMEA_ID_VTG_UART1` | `0x209100B1` |
| `CFG-MSGOUT-NMEA_ID_ZDA_UART1` | `0x209100D9` |

Port suffixes differ per key — I2C/SPI/UART2/USB are **not** a fixed offset from
UART1. Look each one up; do not compute it.

### CFG-RATE (§5.9.18)

| key | ID | type | scale/unit |
|---|---|---|---|
| `CFG-RATE-MEAS` | `0x30210001` | U2 | 0.001 s — time between measurements. 100 ms → 10 Hz, 1000 ms → 1 Hz |
| `CFG-RATE-NAV` | `0x30210002` | U2 | measurements per navigation solution, max 128 |
| `CFG-RATE-TIMEREF` | `0x20210003` | E1 | time system alignment |

### CFG-NAVSPG-DYNMODEL (§5.9.13) — `0x20110021`, E1

| constant | value | meaning |
|---|---|---|
| `PORT` | 0 | Portable — **the default** |
| `STAT` | 2 | Stationary |
| `PED` | 3 | Pedestrian |
| `AUTOMOT` | 4 | Automotive |
| `SEA` | 5 | Sea |
| `AIR1` | 6 | Airborne, <1 g acceleration |
| `AIR2` | 7 | Airborne, <2 g |
| `AIR4` | 8 | Airborne, <4 g |
| `WRIST` | 9 | Wrist-worn watch |

⚠️ Relevant to the quadcopter: the default *portable* model constrains the
solution to low dynamics. `AIR1`/`AIR2` are the realistic settings for a
multirotor.

### CFG-UART1 / protocol enables

| key | ID | type |
|---|---|---|
| `CFG-UART1-BAUDRATE` | `0x40520001` | U4 |
| `CFG-UART1OUTPROT-UBX` | `0x10740001` | L — default **1 (true)** |
| `CFG-UART1OUTPROT-NMEA` | `0x10740002` | L — default **1 (true)** |

UBX output is already enabled on UART1 by default; NAV-PVT is silent only
because its `CFG-MSGOUT` rate is 0. Turning NMEA off wholesale is one `L` write
rather than 14 `CFG-MSGOUT` writes — but then GGA disappears too, so keep the
NMEA decoder's expectations in mind.

---

## 6. UBX-NAV-PVT (§3.15.10) — class/id `01 07`, payload **92 bytes**

| off | type | name | scale | unit | meaning |
|---|---|---|---|---|---|
| 0 | U4 | `iTOW` | — | ms | GPS time of week of the nav epoch |
| 4 | U2 | `year` | — | y | UTC |
| 6 | U1 | `month` | — | | 1..12 |
| 7 | U1 | `day` | — | | 1..31 |
| 8 | U1 | `hour` | — | | 0..23 |
| 9 | U1 | `min` | — | | 0..59 |
| 10 | U1 | `sec` | — | s | 0..60 |
| 11 | X1 | `valid` | — | | b0 `validDate`, b1 `validTime`, b2 `fullyResolved`, b3 `validMag` |
| 12 | U4 | `tAcc` | — | ns | time accuracy estimate |
| 16 | I4 | `nano` | — | ns | fraction of second, −1e9..1e9 |
| 20 | U1 | `fixType` | — | | 0 none, 1 DR only, **2 = 2D**, **3 = 3D**, 4 GNSS+DR, 5 time only |
| 21 | X1 | `flags` | — | | b0 `gnssFixOK`, b1 `diffSoln`, b2..4 `psmState`, b5 `headVehValid`, b6..7 `carrSoln` |
| 22 | X1 | `flags2` | — | | b5 `confirmedAvai`, b6 `confirmedDate`, b7 `confirmedTime` |
| 23 | U1 | `numSV` | — | | satellites used in the solution |
| 24 | I4 | `lon` | 1e-7 | deg | |
| 28 | I4 | `lat` | 1e-7 | deg | |
| 32 | I4 | `height` | — | mm | above ellipsoid |
| 36 | I4 | `hMSL` | — | mm | above mean sea level |
| 40 | U4 | `hAcc` | — | mm | horizontal accuracy estimate |
| 44 | U4 | `vAcc` | — | mm | vertical accuracy estimate |
| 48 | I4 | `velN` | — | mm/s | NED north |
| 52 | I4 | `velE` | — | mm/s | NED east |
| 56 | I4 | `velD` | — | mm/s | NED down |
| 60 | I4 | `gSpeed` | — | mm/s | ground speed (2-D) |
| 64 | I4 | `headMot` | 1e-5 | deg | heading of motion (2-D) |
| 68 | U4 | `sAcc` | — | mm/s | speed accuracy estimate |
| 72 | U4 | `headAcc` | 1e-5 | deg | heading accuracy |
| 76 | U2 | `pDOP` | 0.01 | — | position DOP |
| 78 | X1 | `flags3` | — | | b0 `invalidLlh` = lon/lat/height/hMSL invalid |
| 79 | U1[5] | `reserved0` | — | | |
| 84 | I4 | `headVeh` | 1e-5 | deg | vehicle heading; valid only if `headVehValid` |
| 88 | I2 | `magDec` | 1e-2 | deg | magnetic declination (ADR 4.10+ only) |
| 90 | U2 | `magAcc` | 1e-2 | deg | declination accuracy (ADR 4.10+ only) |

Offsets sum to 92 — use that as the decoder's sanity check.

⚠️ `lat`/`lon` scale is **1e-7 deg ≈ 1.1 cm**. NMEA GGA's `ddmm.mmm` resolves to
~1.85 m, so this is the single biggest reason to switch.

⚠️ §3.3.4: value fields are only meaningful when their validity flag is set.
Check `flags.gnssFixOK` before trusting position, and `flags3.invalidLlh`
independently. `fixType != 0` alone is **not** sufficient.

---

## 7. Traps

| trap | consequence |
|---|---|
| Checksum must exclude the sync bytes | receiver silently ignores the frame; **no NAK, no error** |
| `CK_A`/`CK_B` must be masked to 8 bits | same silent rejection |
| Length counts the payload only | same |
| `CFG-PRT` / `CFG-MSG` are **deprecated** on M9 (protocol 27+) | may still be honoured; do not build on them. Use VALSET/VALGET/VALDEL |
| `layers = 0` | ACK-NAK, nothing applied |
| Any unknown key in a batch | ACK-NAK, **entire message discarded** — so batch only known-good keys |
| Baud key written to BBR/Flash | receiver comes up at a rate the firmware does not use |
| `fixType != 0` used as "position valid" | must also check `flags.gnssFixOK` and `flags3.invalidLlh` |
| Port suffix arithmetic on `CFG-MSGOUT` keys | UART2/I2C/SPI/USB ids are not offsets — look them up |

⚠️ **TASKING alignment** — unrelated to UBX but it bites the same code:
the compiler aligns `uint32` in a struct to **2** bytes while `tools/gen_a2l.py`
assumes 4. Never overlay a struct on a NAV-PVT buffer; decode by explicit byte
offset. See `docs/CODEMAP.md` and the `Xcp_Data` `*Reserved[]` convention in
`src/bsw/Measurements.h`.

⚠️ **ASCLIN traps** for the transmit path (`IfxAsclin_Asc_write` hits the
software FIFO and is useless here; `IfxAsclin_Status_noError = 1`) — see
`docs/ILLD_NOTES.md` and the driver's own comments.

---

## 8. Gaps — not extracted

Trust this note's silence only within the list above.

- **NMEA message definitions** (§2.7) — GGA/RMC/VTG/GSA/GSV field tables. The
  current decoder only uses GGA fields 6 and 7; if NMEA parsing is extended,
  extract §2.7 rather than guessing.
- **CFG-VALGET / CFG-VALDEL payloads** (§3.10.26–27) — class/ids recorded above,
  layouts not extracted. VALGET is worth having for read-back verification.
- **CFG-VALSET with transactions** (§3.10.28.2, version `0x01`) — only the
  non-transactional version 0 is documented here.
- **All other NAV messages** — NAV-SAT, NAV-DOP, NAV-STATUS, NAV-VELNED etc.
- **RTCM** (§4), **UBX-MON / UBX-TIM / UBX-LOG / UBX-RXM** classes.
- **Full CFG group list** (§5.9) — only MSGOUT/RATE/NAVSPG/UART1 keys above.
- The **Integration manual** is a separate document and is *not* on this machine;
  §3.3.4, power management, iTOW semantics and antenna supervision all refer to
  it.

---

## 9. See also

- `docs/PINNING.md` §2.7 — ASCLIN4 pins, the 1 kΩ/2 kΩ divider on P22.5, supply
- `docs/ILLD_NOTES.md` — ASCLIN driver traps
- `docs/SENSORS.md:133` — the NAV-PVT bandwidth/CPU budget this note enables
- `src/bsw/GnssM9N.c` — driver; `test/test_GnssM9N.c` — host tests
