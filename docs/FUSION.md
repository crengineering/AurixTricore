# Sensor fusion — attitude and navigation

How five sensors become one state. Read `src/bsw/Ahrs.h` and `src/bsw/fusion.h`
alongside this; the headers carry the derivations, this file carries the
bench numbers, the failure modes and the calibration procedure.

Frame is **NED** throughout: x forward, y right, **z DOWN**. So `posD` is
positive downward and `velD` is positive descending. Barometric and GNSS
altitude both count *upward*; the sign flip happens exactly once per sensor,
where it enters.

---

## 1. Structure

```
  ICM-42688-P ──acc,gyro──┐
  MMC5983MA   ──mag───────┤  Ahrs.c    quaternion Mahony filter
                          └──────────► roll/pitch/yaw, body rates,
                                       gyro bias, accNed[3]
                                              │
                          accNed (gravity removed, m/s^2)
                                              │
  BMP581      ──altitude──┐                   ▼
  NEO-M9N     ──pos,vel───┤  fusion.c   three 4-state Kalman channels
                          └──────────► DOWN / NORTH / EAST
                                       position, velocity, accel bias,
                                       measurement bias
```

Everything runs in `Task_Imu` at 50 Hz on CPU0. The barometer, magnetometer and
GNSS tasks **latch** their samples (`Fusion_setBaroAlt`, `Ahrs_setMag`,
`Fusion_setGnss`) and the IMU tick consumes whatever has arrived. That keeps a
single writer per state and needs no locking.

### Why a cascade rather than one big EKF

A 15-state EKF would be the textbook answer and is the wrong answer here.
Attitude converges in seconds from two vectors that never drift; position takes
minutes and depends on a receiver that is absent indoors. Splitting them means
the vertical channel can be validated on a desk with a 30 cm lift, months before
anything flies — and when the horizontal channel misbehaves outdoors, attitude
is not implicated.

### The four states per channel

| state | what it is |
|---|---|
| `position` | metres along that NED axis from the channel origin |
| `velocity` | m/s |
| `accelBias` | accelerometer offset on that axis [m/s²] |
| `measBias` | offset of the RELATIVE sensor against the ABSOLUTE one [m] |

`accelBias` is what makes this better than integrating twice: a **+0.018 m/s²**
offset (measured on this board) becomes 32 m of imaginary descent in 60 s if
nothing estimates it.

`measBias` is only meaningful on the DOWN channel, where two sensors measure the
same thing differently: the barometer sees `d + measBias`, GNSS altitude sees
`d`. That pair makes the barometer's weather drift observable so it stops
leaking into altitude. NORTH and EAST have no relative sensor, so their
`measBias` gets zero process noise and stays exactly zero.

---

## 2. Bench numbers (2026-08-26, this board)

Measured, not assumed. These are what the tuning constants are derived from.

| quantity | value | where it is used |
|---|---|---|
| accelerometer noise, down axis | 0.0144 m/s² | floor for `FUSION_SIGMA_A_D` |
| accelerometer bias, down axis | **+0.018 m/s²** | why `accelBias` exists |
| accel bias drift over 60 s | −0.0003 m/s² | `FUSION_SIGMA_ACC_RW` |
| barometer short-term noise | **0.0197 m** | `FUSION_SIGMA_BARO` (R is this *squared*) |
| barometer wander, board at rest | **0.19 m/min** | `FUSION_SIGMA_BARO_RW` |
| \|a\| at rest | 0.998 g | health check |
| \|B\| at rest, uncalibrated | 0.456 G | see §4 |
| gyro bias after boot calibration | ~0.2–0.4 °/s | residual for the Mahony integral |

---

## 3. Mounting transforms — measured, not assumed

`Ahrs.c` maps sensor axes to body axes as a **permutation with signs**, not a
sign flip. For the ICM-42688-P eval board on this airframe:

```
body x (forward) = +sensor y
body y (right)   = +sensor x
body z (down)    = -sensor z
```

Determined on the bench by holding two orientations and reading the result back:

| orientation held | what the sensor reads | conclusion |
|---|---|---|
| level | sensor Z = +1 g | sensor Z is **up** |
| nose up | sensor Y = +1 g | sensor Y is **forward** |
| right wing down | sensor X = −1 g | sensor X is **right** |

The determinant is **+1**, so the frame stays right-handed. That is not
cosmetic: the gyro is a *pseudovector* and transforms differently from the
accelerometer under an improper transform, so a left-handed mapping leaves the
two halves of the filter disagreeing about which way is up. **Negate zero or
two axes, never one** — and if you permute, check the determinant.

> ⚠️ The AHRS deleted at `50ef619` used `(+1, −1, −1)`, a pure sign flip. That
> was correct **for the MPU-6050 on a GY-521**, which is what it was written and
> HW-verified against. It was never revisited when the ICM-42688-P replaced that
> part. Do not take those constants as evidence for this sensor — this cost a
> bench session to rediscover.

### Bench check after any remount

Board held still:

| orientation | expect |
|---|---|
| level, chip up | roll ≈ 0, pitch ≈ 0 |
| nose up | **pitch ≈ +90** |
| right wing down | **roll ≈ +90** |
| rotate 90° level | yaw advances 90, \|B\| unchanged |

Wrong signs here mean the controller will drive the wrong way.

**Verified on hardware 2026-08-26, v1.19.3:** nose up gave **pitch +80** (held
at roughly 80°) with roll staying inside −8…+9; right wing down gave **roll
+92** with pitch staying at +3; `|a|` held 0.99–1.01 through both. Excursions of
`|a|` to 0.69 and 1.51 *during* the movements are the hand accelerating the
board, and are exactly what the accelerometer trust window is there to reject.

---

## 4. Magnetometer calibration — required before yaw means anything

The magnetometer does not measure Earth's field. It measures Earth's field
**plus the board's own magnetics**, which is a constant vector *in the sensor
frame*. As the board turns, that traces an **off-centre sphere**.

Measured uncalibrated on this board: `|B|` swung **0.428 … 0.984 G** with
orientation, against a true field of ~0.48 G in Munich. That is a heading error
which varies with heading, and no amount of filtering removes it.

```
python tools/mag_cal.py --seconds 90 --write --decl 3.9
```

Turn the board through **all six faces** while it collects, tumbling between
them — rotating about one axis gives a ring, and a ring does not determine a
centre in three dimensions. The tool refuses to `--write` a fit with incomplete
octant coverage or a residual above 5 %.

Results are stored in `Xcp_Nvm` (`magOffX/Y/Z`, `magScaleX/Y/Z`, `magDeclDeg`),
survive a power cycle, and default to a **no-op** so an uncalibrated board
behaves exactly as it did before the fields existed.

Declination is applied at the AHRS *output*, not inside the filter — it is a
property of the location, not the board, and folding it in would make the stored
hard-iron offsets location-dependent too.

**The magnetometer mounting transform is still a hypothesis.** It cannot be
measured until hard iron is corrected, because the 2× `|B|` swing swamps any
axis check. Calibrate first, then verify that yaw tracks a physical 90° rotation.

**Symptom to expect until this is done:** yaw drifts. Measured on v1.19.3 with
the board sitting still and level, `magTrusted = 1`: yaw walked **+0.37 °/s**
(127.7° → 136.2° over 23 s). With an uncalibrated hard-iron offset the mag
correction pulls toward a heading that is itself wrong by an
orientation-dependent amount, so it cannot hold yaw — it only replaces gyro
drift with its own. Roll and pitch are unaffected, which is the signature: the
mag correction is confined to the vertical axis by construction.

---

## 5. Failure modes that are already handled

Each of these was observed on hardware, not imagined.

**Barometer outlier gate that fires on real motion.** With `p00` converged to
4.06e−05, a 5σ gate is only ±0.104 m — and an ordinary 30 cm hand lift produced
a 0.064 m innovation, 62 % of the way to rejection. A gate that fires on real
dynamics is worse than no gate: it drops the barometer exactly when the filter
needs it. Fixed with an absolute floor, `FUSION_GATE_MIN_M = 2.0` — the gate
exists to catch a *corrupt* reading, which is wrong by tens of metres, not to
police manoeuvres.

**Rejection deadlock.** Rejecting a sample also skips the covariance update, so
a filter thrown far off rejects everything afterwards and never recovers — the
innovation grows with the diverging state faster than the gate grows with P.
Observed 2026-08-25: a knock (−8.2 g) left the estimate at 293 m and climbing
with 4282 consecutive rejections. Two layers now: every rejection inflates P so
the gate reopens on its own, and after `FUSION_REJECT_MAX` (0.5 s) the channel
re-acquires from the sensor rather than defending a fiction.

**Covariance corruption.** `P = T·Fᵀ` gathering the wrong columns drove `varD`
to **−inf**, after which S went negative, the gate stopped meaning anything and
the estimate free-integrated to +166 m while reporting a 1.8 m/s descent. Now
guarded: `covResets` in the published block **must stay zero** — any other value
is a bug, not something to tune.

**NaN killing the escape hatch.** Every comparison against NaN is false, so the
rejection counter never advanced and re-acquisition never fired: 11255 rejects,
1 reset, stuck forever. The health check is written as `!(p > 0)` precisely
because that is *true* for NaN, where `p <= 0` would be false.

**Partial covariance reset.** Re-acquisition that resets only the diagonal
leaves the off-diagonals holding whatever they diverged to, and the next predict
mixes them straight back in. If you throw the covariance away, throw all of it.

**Unbounded variance with no absolute reference.** Only the *sum* `d + measBias`
is observable from a barometer, so split individually the two wander in opposite
directions forever and `var(d)` grows without bound — it reached 0.51 m² in 25 s
and would eventually have tripped the health check for no real reason. The
barometer bias is therefore modelled as **mean-reverting** (`FUSION_TAU_BARO_BIAS_S`
= 600 s), which is also the honest model: weather pressure does not wander to
infinity. `varD` now settles around 0.42 m².

`varN` growing without bound indoors is **correct and not a fault** — with no
GNSS fix the filter genuinely does not know where it is. It clamps at
`FUSION_P_MAX` rather than resetting, because resetting would falsely claim the
estimate had improved.

---

## 6. Reading the state

The full navigation state is `Xcp_Fusion` at **`0x70030500`** (180 bytes,
layout in `src/bsw/Measurements.h`). The seven legacy fusion fields in
`Xcp_Data` are still written so existing A2L entries, GUI plots and tool
addresses keep working.

A2L entries are generated — `python tools/gen_a2l.py`, and CI runs `--check`.

```
python tools/xcp_read.py 0x70030500:hex:8        # magic "FUSN" + uptime
python tools/xcp_read.py 0x70030508:f32 0x7003050C:f32 0x70030510:f32   # roll/pitch/yaw
python tools/xcp_read.py 0x700305B0:u32          # covResets -- must be 0
```

Quick health read, in order of what to distrust first:

| field | offset | healthy |
|---|---|---|
| `covResets` | `0xB0` | **0**, always |
| `ahrsState` | `0x50` | 2 = RUNNING |
| `accMagG` | `0x48` | ≈ 1.0 at rest |
| `magFieldG` | `0x4C` | constant as the board rotates (after §4) |
| `baroRejects` | `0x9C` | not climbing |
| `velD` | `0x58` | ≈ 0 at rest |
| `varD` | `0x68` | bounded, ≈ 0.4 m² |

---

## 7. What is not validated yet

- **The whole horizontal channel.** It never executes without a GNSS fix, and
  there is no fix indoors. Needs an outdoor run: check `originSet` goes to 1,
  `gnssUpdates` increments at 1 Hz (not 10 — that would mean the `iTOW` new-fix
  check is broken), and `posN`/`posE` track a walked rectangle.
- **GNSS altitude anchoring the barometer bias.** Same reason.
- **The magnetometer mounting transform** — see §4.
- **Yaw accuracy against a known heading.** Needs §4 done first, then an
  outdoor check against a real bearing.
