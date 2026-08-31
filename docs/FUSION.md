# Sensor fusion — attitude and navigation

**ASPICE:** SWE.3 — detailed design, estimator (`Ahrs.c` + `fusion.c`) + SYS.4 bench evidence · realizes SYS2-NAV-001/002, SYS2-TIM-001/002 · process: QuadSE/requirements/README.md

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

Everything runs in `NavTask_step` on **CPU1**, the dedicated flight core
(`docs/REFACTORING_PLAN.md` T12) — nothing else is registered on that core but
a 2 µs LED blink. **The rate is 1014.2 Hz, DRDY-clocked, not a fixed 1 kHz**
(T15, `docs/REFACTORING_PLAN.md` §3.1/§3.6/§9): the IMU's own INT1 edge — timed
by `imuDrdyIsr`, now on CPU1 too — is the clock, and `NavTask_step` (registered
at `SCHED_US(500)`, a 2 kHz poll well above the sensor) takes its `dt` from the
edge timestamps every time, never from a constant or from its own dispatch
interval. Adopted at ~5x the ~200 Hz the control law actually needs (§9.3),
justified by anti-aliasing the 244-382 Hz propeller blade-pass band rather
than by the control law (§9.4) — a fixed 1000 µs would be 1.4 % wrong on a
normal tick and 100 % wrong across a missed edge. The barometer, magnetometer
and GNSS tasks run on CPU0 and **latch** their samples into the shared LMU
block (`Fusion_setBaroAlt`, `Ahrs_setMag`, `Fusion_setGnss`; backing state in
`FusionLatch.h`/`AhrsLatch.h`, see `docs/CODEMAP.md` §3); `NavTask_step`
consumes whatever has arrived and publishes the result via `NavState_publish`
for CPU0 to read with `NavState_get`. That keeps a single writer per state and
needs no locking.

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

### Result on this board, 2026-08-26

This set is versioned in **`calibration/board.json`** — after any reflash
(`flash.bat` erases DFLASH) restore it with
`python tools/mag_cal.py --restore-from calibration/board.json`, which also
verifies by read-back. A later `--write` updates the JSON automatically.

| | |
|---|---|
| hard iron | X **−0.1940**, Y **−0.0722**, Z **−0.8510** G |
| soft iron | 1.0 / 1.0 / 1.0 — sphere fit, see below |
| declination | 3.9° (Munich) |
| raw \|B\| | 0.383 … 1.373 G, spread **135 %** |
| corrected \|B\| | 0.432 … 0.493 G, spread **13 %**, mean **0.4655 G** |
| fit residual | 3.51 % |

The **Z offset of −0.85 G is larger than Earth's entire field**. Something on
the board close to the sensor is strongly magnetic, and without correction the
magnetometer was reporting mostly that. The corrected magnitude landing within
3.5 % of the true local field (~0.48 G in Munich) is the independent check that
the fit is physical rather than merely self-consistent.

The soft-iron scales came out at exactly 1.0 because the ellipsoid fit was
rejected in favour of the sphere fallback — a hand rotation rarely conditions
the three squared terms well enough, and applying an ill-determined soft-iron
correction is worse than applying none. Hard iron dominates anyway.

### Two traps in the tool, both fixed after they bit

**A stationary board scored 100 % coverage.** The octant check measured only
which side of the *mean* each sample fell on, and sensor noise puts samples on
all sides of the mean — so a board sitting still on the desk looked like perfect
spherical coverage and would have written a garbage calibration into flash. The
check now requires each sample to be genuinely out at ~25 % of the field
magnitude before its octant counts.

**A failed fit threw the whole rotation away.** Two runs were lost before the
raw samples were dumped unconditionally, before fitting, so a bad run can be
re-fitted offline instead of re-tumbled.

The live `corners n/8 [########]` display exists because blind instructions do
not work: the first successful run sat at 5/8 for a minute, and the pattern
`#.#.#.#.` said immediately that every missing octant had +Z — the board was
never being turned upside down. That is a one-second diagnosis from the display
and an unanswerable question without it.

Declination is applied at the AHRS *output*, not inside the filter — it is a
property of the location, not the board, and folding it in would make the stored
hard-iron offsets location-dependent too.

**The magnetometer mounting transform.** It could not be measured until hard
iron was corrected, because the 2× `|B|` swing swamped any axis check. After
calibration, a level 360° rotation produced a smooth, continuous, monotonic
sweep through all 360° of yaw with `magTrusted = 1` throughout.

**Confirmed 2026-08-26:** the rotation was CLOCKWISE seen from above, and yaw
INCREASED through it. In NED that is correct — yaw runs north to east to south
to west, i.e. clockwise from above. The transform is validated.

Smooth tracking is corroborating evidence for a reason worth understanding: the
Mahony filter *fuses* gyro and magnetometer. If the mag transform had the wrong
handedness, the mag correction would oppose the gyro integration on every single
sample, and yaw would stall, jitter or lock rather than sweep cleanly. A clean
monotonic sweep means the two agree with each other; the rotation direction is
what pins down which way both of them are pointing.

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

### 6.1 Roll and yaw are meaningless near pitch = ±90° — this is not a bug

**Measured 2026-08-27 on v1.19.13.** Pitching the board nose-up through 90°
makes `rollDeg` leap tens or hundreds of degrees and slam between +180 and
−180. It looks alarming on a plot and it is **not** a filter fault. Do not
re-investigate this; the evidence is below.

`rollDeg` is a projection of the quaternion, `Ahrs.c:818`:

```c
out->rollRad = atan2f(2.0f * ((q0*q1) + (q2*q3)),
                      1.0f - (2.0f * ((q1*q1) + (q2*q2))));
```

At pitch → ±90° **both arguments approach zero**, so the value is
`atan2f(0, 0)` — mathematically undefined, and arbitrarily small noise picks a
different answer each tick. The two triples (roll 0°, pitch 90°, yaw ψ) and
(roll 180°, pitch 90°, yaw ψ+180°) describe **the same physical orientation**;
the filter is free to report either. This is gimbal lock, inherent to any
three-angle representation, not something this code introduced.

The measurement that settles it — one sample step at pitch ≈ 87-89°:

| | roll [deg] | q0 | q1 | q2 | q3 |
|---|---|---|---|---|---|
| t | **6.08** | 0.6587 | −0.2835 | 0.6251 | 0.3084 |
| t+1 | **62.53** | 0.6453 | −0.2842 | 0.6446 | 0.2954 |

`|Δq| = 0.0270`, i.e. a **2.94° physical rotation**, while the reported roll
changed **56.5°**. The quaternion — which *is* the filter state — stayed
continuous through the whole event, and so did `pitchDeg` (86.6 → 89.1 → 88.7
→ 87.7). Only the Euler projection blew up.

**What follows from this:**

- **The Euler triple is telemetry. `q[4]` (offset `0x14`) is the state.** When
  an attitude looks wrong above ~80° of pitch, read the quaternion before
  suspecting the filter. It is already published, so no firmware change is
  needed to look at it.
- **Left as is, deliberately.** Nothing in the estimator is wrong, and no diag
  bit is spent on it — `DIAGNOSTICS.md` records bits 27-30 as the last four and
  they are allocated. A quadrocopter that reaches 90° of pitch is in an upset,
  and the correct response to an upset is a defined failsafe action, not a
  better roll number. That belongs with the actuator/arming safety design, not
  here.
- **The rate loop is unaffected.** It consumes body rates (`rateDps`, `0x24`),
  which are never degenerate. Only the outer attitude loop consumes Euler
  angles, and only in an orientation the aircraft should never hold.

---

## 7. Outdoor validation, 2026-08-26 (v1.19.5)

240 s record, 13 satellites, hAcc 2.4 m, walked loop of roughly 15 x 9 m.
Raw data: `tools/nav_outdoor_check.py --mf4`.

| check | result |
|---|---|
| fixes fused | 2398 in 240 s = **10.00 /s** |
| iTOW advance | 239 800 ms over 240 s — tracks the clock to 99.9 % |
| duplicate polls | 0 |
| GNSS rejected / `covResets` | **0 / 0** |
| velocity while standing still | 0.065 and 0.099 m/s |
| loop closure | **2.59 m** (dN −1.23, dE +2.28) |
| `varN` | 0.09 m² |
| `baroBias` | **+1.92 → +0.01 m** |

The `baroBias` line is the one that matters: that state had never executed
before, because it is only observable with barometer AND GNSS altitude both
present. It converged from 1.9 m to zero, which is the barometer's weather
offset being separated from true altitude exactly as designed.

Closure of 2.59 m against hAcc 2.4 m means the loop closed as well as GNSS
permits. Be honest about what that proves: it is *consistent* with a correct
filter, but at this noise level it cannot distinguish "perfect" from "2.5 m of
accumulated drift".

### ⚠️ The filter is roughly 8x overconfident

`varN` settles at 0.09 m², claiming sigma = 0.3 m, while the actual closure
error was 2.59 m. The mechanism is visible in the innovations: they came in at
1.12 m (N) and 0.96 m (E) RMS, against the sqrt(P+R) ~ 2.4 m the filter
expected. Innovations *half* the predicted size is the signature of measurements
that are not independent — the filter counts ten correlated NAV-PVT solutions
per second as ten independent draws, so P collapses below the true error.

The position itself is fine. What is not trustworthy is `varN` **as a quality
signal**, so nothing downstream should gate a decision on it yet.

Decimating to 1 Hz is the obvious fix and is the wrong one, for a reason worth
recording: GNSS POSITION error is dominated by slowly-varying bias
(ionosphere, multipath) so the extra samples carry little, but GNSS VELOCITY is
Doppler-derived — about 0.05 m/s on an M9N, far less correlated, and genuinely
fresh at 10 Hz. It is also the measurement that stabilises a position loop on an
aircraft. So: keep 10 Hz, inflate the POSITION R only, and raise the trust in
velocity (`FUSION_SIGMA_GNSS_VEL` is 0.3 m/s against a receiver an order of
magnitude better than that).

### ⚠️ Measurement latency is unmodelled

NAV-PVT is already ~100–180 ms old when it is fused (receiver processing, plus
100 bytes at 38400 baud ~ 26 ms, plus up to 100 ms of poll phase) and is applied
as though current. At walking pace that is ~0.2 m and invisible in this record.
At 15 m/s it is 1.5–2.7 m, always lagging the direction of travel — a systematic
error a position controller would fight. `gnssITow` is published so the fix can
be timestamped when this is addressed.

---

## 8. Barometer filtering — do not filter twice

`BMP581_DSP_IIR_VAL` carried the BMP388 indoor-navigation preset (pressure IIR
coefficient 15) until 2026-08-26. That is the wrong preset for a flight vehicle.

The IIR is `y[n] = (c*y[n-1] + x[n])/(c+1)`, so at coefficient 15 and 50 Hz it is
an exponential average with alpha = 1/16 and a time constant of ~310 ms.
`Fusion_setBaroAlt` receives that already-smoothed value and treats it as an
INSTANTANEOUS altitude, because the Kalman filter has no model of the lag.

Oversampling and IIR are not interchangeable. `OSR_p` averages within one
conversion and has no memory, so it reduces noise for free. The IIR reduces
noise by REMEMBERING, and that memory is the lag. The Kalman filter is already
the optimal smoother and `FUSION_SIGMA_BARO` tells it how much to trust each
sample, so a hand-tuned IIR in front of it adds lag the filter cannot account
for and buys nothing the filter is not doing better.

Changed to coefficient 1 (tau ~29 ms), `OSR_p x16` unchanged.

### What the change actually cost and bought (measured)

| | IIR 15 | IIR 1 |
|---|---|---|
| innovation std, at rest | 0.0181 m | 0.0196 m |
| baro white-noise sigma | 0.0197 m | 0.0188 m |
| slope of innovation on `v_d` | **−0.0606 s** (r −0.16) | **+0.0058 s** (r +0.03) |

The lag signature — innovation correlated with vertical velocity, which is how
a lagging position sensor betrays itself — is **gone**, and the noise cost was
8 %. `FUSION_SIGMA_BARO` needed no change.

⚠️ **The noise did not rise 4x as predicted.** An EMA at alpha=1/16 attenuates
white noise ~5.6x versus ~1.7x at alpha=1/2, so removing it should have been
obvious. It was not, which means the barometer's apparent scatter is NOT sensor
white noise — it is real low-frequency pressure fluctuation (air movement in the
room), which an IIR barely touches. The coefficient-15 filter was paying 310 ms
of lag for almost no noise reduction at all.

⚠️ **Peak innovation is the WRONG metric for lag** and went *up* (0.064 →
0.081 m). It is dominated by noise spikes, and removing a low-pass necessarily
makes sample-to-sample scatter worse. Lag shows as `z ~ d - tau*v_d`, i.e. as
innovation regressed on velocity — use that.

⚠️ The 61 ms implied by the regression is a LOWER BOUND on the true sensor lag,
not a measurement of it. The filter's gain is low, so it follows the barometer
loosely and part of the lag lands in the output rather than the residual.
Measuring the absolute figure needs an independent altitude reference, which the
bench does not have.

### The vertical channel is fusing correctly (measured at rest, 60 s)

| | total std | first-difference std |
|---|---|---|
| raw baro altitude | 0.07162 m | 0.02659 m |
| filter output `posD` | 0.06983 m | **0.00739 m** |

**3.6x high-frequency attenuation** — that is the accelerometer doing real work.
Innovation std (0.0196) matching sigma_R (0.0197) means `P_pred << R`, so the
gain is LOW and the filter trusts its prediction over each individual sample.
Climb-rate noise at rest is **0.0184 m/s**, which is the figure that matters for
an altitude inner loop.

`posD` total std matching the raw baro is not a failure to smooth: both are
dominated by real slow pressure drift, which on sub-600 s timescales is genuinely
indistinguishable from altitude change. That is what the GNSS-anchored
`baroBias` state resolves, and it did (1.92 -> 0.01 m outdoors).

### ⚠️ The barometer measures AIR, not height — verified 2026-08-26

Reported as "random spikes" in the GUI on `NavPosDown` and `NavInnovDown`. They
are neither random nor a fault, and the distinction took two recordings to
establish.

**Abrupt handling**, 46 Hz record of the raw barometer:

```
t=31.5  632.24   baseline
t=31.9  634.32   +2.3 m peak
t=32.1  631.41   -0.6 m undershoot
t=32.6  632.03   back to baseline, exactly
```

**A smooth 30 cm lift, held 20 s**, same board and firmware:

| | |
|---|---|
| barometer step | +0.27 m, **sustained** for the whole hold |
| `posD` step | −0.27 m (positive is down, so up) |
| innovation throughout | **±0.02 m**, no spike anywhere |
| barometer peak excursion | +0.34 m — no overshoot |

Same displacement, opposite results. The difference is entirely *how* the board
was moved: an abrupt lift shoves air across the pressure port and produces a
transient **seven times** the real displacement, which then rings and decays to
baseline. A smooth one tracks the true height to a centimetre.

**The tell is the sustained step.** A real altitude change holds; an
aerodynamic artefact returns to baseline within about a second. Twenty seconds
of holding makes them impossible to confuse — always hold, never just tap.

**Do not "fix" this by tuning.** Raising `FusSigmaBaro` blunts the spikes and
blunts the genuine 0.27 m step equally: a worse filter, bought to suppress a
symptom the sensor is right about. The accelerometer is the arbiter, because
moving air cannot push on it — during the artefact it stayed flat, during the
real lift it dipped to 0.913 g and peaked at 1.124 g.

This also revises §8: innovation transients during the earlier lift tests were
partly this, not only sensor lag.

### Still untestable on the bench

- **Prop wash.** The barometer is a pressure sensor sitting in a downwash, and
  the finding above is a preview of it — a rotor does continuously what a hand
  did once. Needs foam over the port or a static port; no amount of filtering
  substitutes, because the sensor genuinely is measuring the pressure it is
  exposed to.
- **Motor vibration** on the accelerometer will force `FUSION_SIGMA_A_D` up from
  0.0424 (a PSD, m/s²/sqrt(Hz) — see `docs/NAV_TUNING.md`, not the per-tick
  sigma this used to be) and may need the ICM-42688-P's own anti-alias
  filtering revisited.

---

## 9. Tuning live, and the controller feedback

### `Xcp_FusionCal` @ `0x70030600` — every tuning constant, XCP-writable

RAM only, like `Xcp_Cal`, and deliberately: a tuning experiment must never be
able to leave the vehicle in a state a power cycle cannot recover. The compiled
defaults in `FusionCal_init()` are the measured values in §2. When a value earns
its place, change the default in code — do not rely on the block.

Read every tick, not latched, so a write takes effect on the next filter update.
That also means a nonsense value takes effect just as fast, so every consumer
goes through `FusionCal_positive()`, which is written as *"is it above the
floor"* rather than *"is it below"* — the comparison is FALSE for NaN either
way, so a NaN arriving over XCP takes the compiled default instead of poisoning
a covariance.

The A2L descriptions carry the reasoning for each knob, so the GUI shows *why* a
parameter exists, not just its name. `FusSigmaAccDownPsd` is the one to reach
for first once motors are turning: 0.0424 (a PSD, m/s²/sqrt(Hz) — see
`docs/NAV_TUNING.md`) was measured on a desk and vibration is not in it.
Renamed from `FusSigmaAccDown`/`FusSigmaAccHoriz` when the process noise was
reparametrised from a per-tick sigma to a PSD, so a stale saved tuning file
under the old name fails to find the field instead of writing a value ~32x
too large.

**`gnssPosRScale` defaults to 8.0**, not 1.0 — see §7. It is a stopgap with an
honest justification, not a derived constant: the correct fix is a GNSS
position-bias state, exactly as the barometer has.

### Controller feedback — `Xcp_Fusion` @ `0xC0`..`0xF0`

Everything else in the block is published for humans: degrees, NED.
`src/asw/flight_ctrl.h` wants neither.

| controller input | wants | published at |
|---|---|---|
| `phi_ist[3]` | roll/pitch/yaw, **rad** | `0xC0` |
| `om_ist[3]` | p/q/r, **rad/s** | `0xCC` |
| `p_ned_ist[3]` | N/E/D, m | `0xD8` |
| `v_b_ist[3]` | u/v/w, **BODY frame** | `0xE4` |

The frame difference is the one that bites. `v_b_ist` is a damping term, so it
has to be in the same frame as the rates; feeding it NED velocity would make the
damping wrong by the heading angle. The rotation happens once here, using the
same quaternion and the same code the filter runs on (`Ahrs_nedToBody`), so
there is exactly one definition of "the state the controller sees" and it cannot
drift away from the estimate that produced it.

### The AHRS no longer blocks forever on gyro calibration

`ahrs_calibrate()` restarts its window whenever the gyro spread exceeds 3 °/s.
With no bound that never completes on a board powered on while moving — in a
vehicle, on a vibrating bench, in a hand — and `NavTask_step` gates the entire
navigation filter on `AHRS_RUNNING`: no attitude, no position, no velocity,
indefinitely, with nothing saying why.

There is now a 10 s deadline, and a 2 s still-window to accept a clean bias
before it. Both are **durations accumulated from `dt`**, not sample counts
(T14, `docs/REFACTORING_PLAN.md` §3.8) — a count sized "100 samples, ~2 s at
50 Hz" would silently become ~0.1 s of averaging the moment the task rate
changed, 20x less noise rejection on the one number the whole attitude
solution rests on. Accumulating `dt` instead means the same 2 s/10 s at any
rate this task ever runs at, past (50 Hz) or present (1014.2 Hz). A bias
averaged over a moving window is worse than one averaged over a still window,
and enormously better than no estimate at all; the Mahony integral converges
the remainder within seconds once the accelerometer and magnetometer start
correcting. The degraded result is flagged, not hidden: `ahrsBiasDegraded`
(`0xBC`) says the calibration was taken while the board was moving. It also
divides by the samples actually accumulated rather than a nominal count,
because at the deadline the window is usually partial and dividing by a fixed
number regardless would scale the bias toward zero and make it look
deceptively good.

---

## 10. What is not validated yet

- **The whole horizontal channel.** It never executes without a GNSS fix, and
  there is no fix indoors. Run `tools/nav_outdoor_check.py`: `originSet` must go
  to 1, `gnssUpdates` must keep climbing with `gnssITow` advancing alongside it,
  and `posN`/`posE` must track a walked rectangle that closes on itself.

  ⚠️ **The receiver runs NAV-PVT at 10 Hz**, not 1 Hz — `CFG_RATE_MEAS` is
  100 ms with one message per epoch (`GnssM9N.c`). That is the same rate
  `Task_Measure100ms` polls at, on an independent clock, so the update RATE
  cannot distinguish a working `iTOW` guard from a missing one; both give ~10/s.
  What it does catch is `iTOW` failing to decode at all, which freezes
  `gnssUpdates` after the first fix while the position silently goes stale.

- **GNSS measurement correlation at 10 Hz.** Consecutive NAV-PVT solutions are
  NOT independent: the receiver filters internally at the nav rate, so
  successive fixes share most of their information. The channel filters treat
  each as a fresh independent measurement, which overstates the evidence and
  drives the covariance below the truth. The observable consequence is a gate
  that tightens until good fixes start being rejected, so watch `gnssRejects`
  on the outdoor run. The fix is to decimate to 1–2 Hz for fusion or to inflate
  the GNSS R; which one is better is not yet decided, and the outdoor numbers
  are what should decide it.
- **GNSS altitude anchoring the barometer bias.** Same reason.
- **Yaw accuracy against a known TRUE heading.** The transform and the
  calibration are both validated, but nothing has yet checked that yaw reads the
  correct ABSOLUTE bearing — only that it advances correctly and that `|B|` is
  orientation-independent. Indoors that check is not worth making: building
  steel shifts the field. Do it outdoors against a known bearing, and expect the
  3.9° declination in `Xcp_Nvm` to be part of what is being tested.
