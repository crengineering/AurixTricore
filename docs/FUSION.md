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

> ⚠️ **Retracted, 2026-09-13 (SYS1-001 B9).** "The transform is validated"
> above only ever checked the horizontal rotation SENSE at level — and the old
> and new mappings both preserve that sense, so the clean 360° sweep never
> distinguished them. It did not check the VERTICAL component, which only
> shows up once the board is tilted: rolled 90° the old mapping put yaw
> ~180° off (Chris, bench, 2026-09-13).
>
> **Measured mount (SYS1-001 B9):** a yaw-independent search over all 48
> signed axis permutations against three six-position recordings found
> angle(magnetic field, gravity-down) constant only for **body = (+sensor X,
> −sensor Y, +sensor Z)** — spread 3.4–4.9° across the three datasets, mean
> 33° from down (Munich dip 64° ⇒ 26° expected, 7° residual is soft/hard-iron
> leftover). The old mapping (+sensor Y, +sensor X, −sensor Z, i.e. the same
> table as the IMU mount above) ranked 32nd of 48, spread 29.5°, field
> pointing UP instead of down. See `docs/MMC5983MA.md` §7 for why the fix
> lives in this table (determinant −1, legitimate for a polar vector) rather
> than in `Mmc5983.c` — there is no datasheet to name which axis the driver
> actually mirrors.
>
> **Level heading shifts by +90°** relative to every number quoted above
> (the rotation SENSE is unchanged, so `magTrusted`/sweep-smoothness evidence
> above still holds). Declination (3.9°, Munich) is unaffected — it is
> applied after the mount, at the AHRS output. **Any "yaw points north"
> claim needs a handheld-compass re-check at level; this is not yet done**
> (SYS1-001 B9 task 4, bench, Chris).

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

**One rejected IMU sample re-initialising the whole attitude (SYS1-001).**
`Ahrs_update()`'s `valid` parameter used to have exactly two states: usable, or
`AHRS_NO_SENSOR` on the spot. At the measured ~1014 Hz DRDY rate a duplicate
edge (`deltaTicks` under `NAVTASK_DT_MIN_S`, ~1 per 2400 edges — the sibling of
the ERU `LDEN` double-trigger documented in `docs/ILLD_NOTES.md`) produced
exactly that: one tick with `valid == FALSE`, `AHRS_NO_SENSOR`, then the next
good tick re-aligned — `ahrs_align()` sets yaw **deadbeat** from the latched
magnetometer and zeroes all three Mahony integrals. Two ticks, ~2 ms; at 100 ms
GUI polling only the jump survives. Evidence row `65E61FC20BD73055`
(2026-09-11, fw v1.19.13): 52 of these in 125 s stationary, a 40-step yaw
sawtooth (0.33–2.59°) riding a +0.5°/s residual gyro-z rate, all three
`gyroBias*` collapsing to ≈0 at every step.

**Fix: separate "no usable input this tick" from "the sensor is gone."**
`Ahrs_update()` now debounces the fault instead of latching it on the first bad
tick:

```
good tick  ──────────────────────────────────────────────┐
   │                                                       │ s_faultHoldS = 0
   ▼                                                       │
 RUNNING ──invalid tick──▶ FROZEN (still RUNNING) ─────────┘
   ▲            s_faultHoldS += dt         │
   │                                       │ s_faultHoldS >= AHRS_FAULT_HOLD_S
   │                                       ▼
   └──── ahrs_align(), s_fbI = 0 ──── AHRS_NO_SENSOR
              (exactly once)
```

Below `AHRS_FAULT_HOLD_S` (0.05 s) an invalid tick **freezes**: the quaternion
and the Mahony integral `s_fbI` are left untouched, `rate`/`accTrusted`/
`magTrusted` publish as zero for that tick, and the state stays whatever it
was. Only once bad input has *persisted* for `AHRS_FAULT_HOLD_S` — a duration
accumulated from `dt`, the same "duration not a sample count" idiom
`ahrs_calibrate()`'s window already uses (§9, T14) — does the estimator declare
`AHRS_NO_SENSOR` and let the next good sample re-align. A single glitch, or a
short run of them, never reaches that; a genuine outage (IMU unplugged, SPI
wedged) still does, just 0.05 s later than before. Consequence: after a
genuine ≥ 50 ms outage, yaw is corrected over the ~7 s time constant below
instead of instantly — the trade this fix makes on purpose. Bench, same
protocol as the evidence row above but fw v1.19.15: `g_dbgAhrsRealigns` = 1 in
125 s (the boot alignment only), 0 steps > 0.3°, 0 bias-collapse events, yaw
p2p 0.18° (was 2.78°).

**Why the residual yaw error decays with τ ≈ 7 s.** Between a re-align and the
next one (pre-fix), or after a genuine outage (post-fix), yaw error decays as
`A·(1 − e^(−T/τ))` under the magnetometer correction alone. The correction's
authority is not the nominal `twoKpMag` (`AHRS_TWO_KP_MAG = 0.5`, `Ahrs.c`) —
only the *horizontal* component of the field can rotate yaw
(`ahrs_errorVector()` builds the reference from
`ref = [sqrt(h0²+h1²), 0, h2]`, i.e. the horizontal magnitude flattened to a
reference bearing, so only that horizontal part ever disagrees with the
measured field about bearing), so the effective gain is
`kp_eff = twoKpMag · h_r²`, where `h_r` is the horizontal fraction of the
total field (`h_r = sqrt(h0²+h1²) / |B|`, set by the local magnetic
inclination — µ well under 1 away from the magnetic equator). Measured on this
board (dispatch `SYS1-001`, fit over the 40 pre-fix steps): `A = 3.00°`,
`τ = 7.10 s`, rms 0.17° — worked example at a 13.2 s gap predicts 2.53°,
measured 2.59°. This time constant is a property of the mounting/site
geometry and `AHRS_TWO_KP_MAG`, **unchanged by the debounce fix** — the fix
changes how *often* the estimator restarts this decay, not how fast the decay
itself runs.

**Correction (round 2):** an earlier status note here quoted `ωn = 0.075
rad/s, ζ ≈ 0.94` for this pole. That pair describes a critically-damped
*second-order* system and does not apply — the debounce fix touches neither
gain nor pole, only how often the single first-order yaw pole above restarts;
there is no ωn/ζ to quote for it. The editorial error is corrected here, not
in the requirement text (no stale-parent).

**One rejected mag heading writing a false roll/pitch bias (SYS1-001 strand
B).** The debounce fix above stops the estimator from *re-initialising*, but
the residual defect was structural, not transient: `ahrs_errorVector()`'s mag
term built `e_mag = kp · (m × w)` and summed it, unprojected, into the same
`e[]` the accelerometer uses — and `m × w` is **not** a rotation about the
vertical. For a heading error `ψ` its NED components are

```
(h_r·h_z·sinψ,  h_r·h_z·(1−cosψ),  −h_r²·sinψ)
```

— dominant along **NORTH**, not DOWN. (An earlier version of this file's
sibling comment in `Ahrs.c` at the mag block claimed the opposite — "a
rotation about the vertical only" — which is the false claim this fix
deletes.) Summed into the single Mahony integral `s_fbI[3]`, a standing
heading error therefore wrote a false ROLL/PITCH gyro-bias at
`twoKi · kp · h_r · h_z · sinψ`: measured on this board, 15 s at a
20° heading error moved `gyroBias0` by +1.17 °/s; back at level the standing
error `θ_ss = s_fbI/kp_eff` reached 2.8°, decaying on the slow `twoKi` root
(τ ≈ 45–50 s measured, 44.6–50 s fit). Evidence: `AEA6EC75E42AE1B5`,
`FB1CDAC83CFA37B0`.

**Fix — three parts, in `Ahrs.c`:**

1. **Project `e_mag` onto the estimated vertical.** `d_b = nedToBody(0,0,1)`;
   `eMagD = (m×w)·d_b`; the correction becomes `kp·eMagD·d_b` instead of the
   raw cross product. `eMagD` **is** exactly `−h_r²·sinψ` — the same
   `kp_eff,mag = twoKpMag·h_r²` the τ ≈ 7.1 s fit above already measures — so
   yaw dynamics are bit-identical and the north-axis parasite is deleted. At
   level, `d_b = [0,0,1]` exactly, so the projection reduces to keeping only
   the z-component of the raw cross product — algebraically identical to the
   pre-fix formula there, host-tested bit-for-bit.
2. **Split the integrator.** `s_fbI[3]` (body-frame gyro-bias, unchanged
   name/meaning) is now fed **only** by the accelerometer's `eAcc`; a new
   scalar `s_fbIYaw` (about `d_b`, i.e. heading) is fed **only** by `eMagD`,
   applied as `s_fbIYaw · d_b`. Published `gyroBias[i] = s_bias[i] −
   (s_fbI[i] + s_fbIYaw·d_b[i])·RAD_TO_DEG` — same field, same meaning, no
   A2L move, bit-identical at level. Without this, task 1 alone still left a
   heading error's *transient* (while yaw converges) baked into the
   body-frame-fixed `s_fbI`, which does not rotate back when the board
   returns to level.
3. **Clamp both integrals, as a backstop, not the fix.** `s_fbI` ≤ 2.0°/s per
   body axis (the worst standing error this defect produced), `s_fbIYaw` ≤
   1.0°/s (4× the observed boot-to-boot heading spread). Bounds a residual
   too small to trip the plausibility checks; does not address the mechanism.

**Continuous accelerometer weight, same file.** The hard `|a|` window
(`AHRS_ACC_MIN_G`/`MAX_G`, §3) accepted a lateral disturbance at full gain
right up to its edge: 0.15 g sideways on 1 g gives `|a| = 1.011` g,
comfortably inside `[0.85, 1.15]`, while tilting the apparent vertical by
8.5°. Two continuous weights replace the single cut:

| weight | formula | full trust | zero trust |
|---|---|---|---|
| `w_norm` | `clamp(1 − (\|\|a\|−1\| − 0.05)/0.10, 0, 1)` | within ±5 % of 1 g | at ±15 % (today's old hard edge, `AHRS_ACC_MAX_G`) |
| `w_rate` | `clamp(1 − (gyroLp − 15°/s)/45, 0, 1)` | `gyroLp` ≤ 15°/s | at 60°/s |
| `w_acc` | `w_norm · w_rate` | — scales `twoKpAcc`'s P **and** I contribution together | |

`accTrusted` is `(w_acc > 0)` — the same outer edge as before; the weight
itself publishes as `accWeightPct` (0–100) in `Xcp_Fusion`'s previously
reserved byte at `0x53` (zero offset change). Hover (slow, near-1 g) is
bit-identical to before this change (`w_acc = 1`). Task 12b (below) adds a
third, time-asymmetric hold-off gate on top of `w_norm · w_rate` — see that
section for why `w_rate` alone is not enough.

**Round 1 (2026-09-12): `w_rate` retuned, and reads a low-pass, not the
instantaneous sample.** `gyroLp` is a 50 ms one-pole low-pass of `|gyro|`
(`s_gyroLpDps`, `AHRS_GYRO_LP_TAU_S`), seeded from the instantaneous rate on
every re-align so a recovery does not start the filter from a stale zero.
Two reasons for filtering rather than reading the raw sample:

1. **Mean vs. peak vibration.** An instantaneous `|gyro|` sample on a real
   airframe is dominated by vibration spikes riding on top of the genuine
   angular rate — gating on the instantaneous value chatters the weight
   tick-to-tick on noise the accelerometer correction never needed
   protecting against. The low-pass tracks the real motion, not the noise
   floor sitting on top of it.
2. **~150 ms re-engagement hold-off.** `τ = 0.05 s` means roughly 3τ
   (~150 ms) after a fast rotation ends before the filtered value decays
   back under the full-trust threshold and the accel correction re-engages
   at full weight. Deliberate: right after a fast manoeuvre is exactly when
   the accelerometer is least trustworthy (settling structural vibration,
   residual specific force), so re-arming instantly would undo the point of
   gating on rate at all. Measured (host test): fully suppressed through a
   sustained fast tumble, back to full trust within 150 ms of it stopping,
   not on the very next tick.

The original constants (30/90°/s, zero at 120°/s, instantaneous sample)
were too permissive at a sustained 60°/s roll: `w_norm` stayed at full trust
(1.1 % deviation, inside the 5 % band) and `w_rate` alone only reached 0.667
— comfortably nonzero — so 5.4°/2.0°/0.28° passed through against a
≤2.0°/1.0°/0.5° target (baseline pre-fix: 6.7°/2.5°/0.35°). The retuned
constants (15/45°/s, zero at 60°/s) with the low-pass measure
**0.52°/0.20°/0.03°** on the same constant-60°/s scenario — comfortably
inside target and close to the architect's own ≈0.5° prediction.

**Finding, round 1 (closed by task 12b, below): the half-sine (smooth
accel/decel) profile still fails.** A smoother velocity profile over the same
90°/1.5 s motion (peak
~94°/s, *higher* than the constant-rate case's 60°/s, so not a softer test)
measures 2.84°/1.06°/0.15° — failing the first two clauses, worse than the
constant-rate result despite the higher peak rate. Traced (roll-vs-time
trace, not left in the test suite): not accel lag — with `w_acc = 0` for the
whole high-rate middle portion, the attitude free-integrates the commanded
gyro rate essentially exactly. The excess is a small but *persistent* rate
bias charged into the body integrator (`s_fbI`) during the transition
windows at the START and END of the motion, where `w_acc` sits strictly
between 0 and 1 (the low-pass has not yet suppressed it, or has already let
it back up) — a proportionally-reduced but still nonzero `eAcc`, computed
against a *disturbed* accel reading, still integrates `ki·eAcc·dt` into
`s_fbI` on every one of those ticks. A half-sine accelerates/decelerates far
more slowly than a trapezoid, so it spends roughly 2–3× longer in that
partial-trust band at each end — more time to charge the slow integrator,
which does not un-charge before the motion ends. This is an emergent
property of the two-parameter (gain, low-pass) design as specified here, not
an implementation defect; a fix (e.g. gating the *integral* path on a
stricter trust threshold than the proportional path, or reshaping `w_acc`'s
own transition) is a design decision for the flight-architect, same
footing as the constants themselves — not made unilaterally here.

**Task 12b (B6.5, 2026-09-12): the half-sine finding closed with a hold-off,
not a third rate parameter.** The residual after a manoeuvre follows a single
exponential, `residual = 8.53°·(1 − e^(−twoKpAcc·∫w dt))`, so the ≤2.0° clause
at motion end is exactly a **budget on `∫w dt` over the whole motion:
`∫w dt ≤ 0.267 s`**. A trapezoidal 90°/1.5 s roll spends `∫w dt = 0.062 s`
there — comfortable margin. The half-sine of the same angle and duration
spends **0.246 s** — not in the middle, where the rate is highest and `w = 0`
already, but at the two *ends*, where the rate is genuinely low (so `w_rate`
is non-zero) while the angular *acceleration*, and with it the tangential
accelerometer disturbance, is at its maximum — a blind spot structural to any
weight `w(|ω|)` with `w(0) = 1`, closed by making the gate asymmetric in time
rather than adding a third parameter:

```
AHRS_ACC_HOLDOFF_S = 0.3 s
s_accHoldS: set to 0.3 s whenever gyroLp >= 60°/s (the upper knee);
            held flat, no countdown, while 15 < gyroLp < 60°/s;
            counted down by dt only while gyroLp <= 15°/s.
wAcc = (s_accHoldS > 0) ? 0 : w_norm * w_rate
```

Once the low-passed rate has reached the upper knee — a manoeuvre, not
vibration or a gust — the accelerometer is held out of **both** the P and I
paths (they share `wAcc`) for a flat 0.3 s regardless of how quickly the rate
then falls back through the partial-weight band, which is exactly the window
the half-sine's tail ends open. The hold can only **arm** above 60°/s, a rate
hover never reaches, so hover is untouched, and it resets to 0 on every
re-align (`AHRS_ALIGNING`, beside `s_gyroLpDps`) so a recovery does not start
artificially held off.

| profile (90°/1.5 s + 0.15 g lateral) | `∫w dt` | motion end | +1 s | +3 s | clause (≤2.0/1.0/0.5°) |
|---|---|---|---|---|---|
| trapezoid (round 1, no hold-off) | 0.062 s | 0.52° | 0.20° | 0.03° | pass |
| half-sine (round 1, no hold-off) | 0.498 s | 2.84° | 1.06° | 0.15° | **fail** (clauses 1–2) |
| trapezoid (task 12b, hold-off) | 0.062 s | 0.52° | 0.20° | 0.03° | pass (limits only, not pinned bit-identical — see below) |
| half-sine (task 12b, hold-off) | 0.246 s | 1.92° | 0.95° | 0.13° | pass |

The trapezoid's own `∫w dt` barely moves (0.062 s here vs the architect's
0.064 s prediction; a sustained rate sitting almost exactly on the 60°/s knee
is a floating-point knife-edge — measured host results are asserted against
the ≤2.0/1.0/0.5° limits, not pinned to the pre-task-12b values, per the
dispatch). Hover (`|ω|_lp50 < 15°/s`, `||a|−1| < 0.05 g`) never arms the hold
and stays bit-identical to task 11. A 300 s stationary run (`|ω| ≈ 0`
throughout) never arms it either, so `s_fbI` remains exactly as free to move
as before task 12b (host-tested: the 45°-persistent-error anti-windup test
runs at zero gyro rate for 300 s and is unaffected).

**Task 12c (B6.6, review round 2 major, 2026-09-12): the 15–60°/s band was a
LATCH, not a hold-off — fixed to a wall-clock duration.** Task 12b's middle
branch ("held flat, no countdown, while 15 < gyroLp < 60°/s") has a release
condition — "the rate falls back under 15°/s" — that a vehicle can simply
decline to meet: a **sustained 30°/s turn** (squarely inside that band)
never satisfies it, so `accWeightPct` measured **0 for 60913 of 60913 ticks
over 60 s** — the accelerometer locked out of both the P and I path for the
whole turn, an unbounded regression against SWE1-FW-007 and against the
pre-strand-B behaviour on exactly the manoeuvre the hold-off was never meant
to touch. No gate on this chain may depend on a condition the vehicle can
decline to meet; the bound must be a duration, for every input. Fix — two
branches, no band:

```
AHRS_ACC_RATE_ZERO_DPS = AHRS_ACC_RATE_FULL_DPS + AHRS_ACC_RATE_SPAN_DPS   /* = 60 deg/s */
if (gyroLp >= AHRS_ACC_RATE_ZERO_DPS) { s_accHoldS = AHRS_ACC_HOLDOFF_S; }
else                                  { s_accHoldS -= dt; floor at 0; }
wAcc = (s_accHoldS > 0) ? 0 : w_norm * w_rate
```

The hold now **expires 0.3 s of wall clock after the rate was last at or
above the arming knee** — not after it has returned to the full-trust band.
A manoeuvre that keeps dwelling in the 15–60°/s band *after* the hold
expires is then covered by the ordinary `w_rate` ramp, which is *correct*
rather than merely tolerable: the tangential disturbance is proportional to
the angular *acceleration*, so a manoeuvre slow enough to linger in the band
has a small disturbance — the 0.3 s budget only has to bind while the motion
is fast, and a fast motion leaves the band quickly. The half-sine's own
falling end (60 → 15°/s) takes 0.236 s, still inside the 0.3 s bound, so
`∫w dt` and the motion-end error are unchanged; the +1 s figure *improves*
(re-engages ~0.24 s earlier, since the countdown now starts at the 60°/s
crossing instead of waiting for the 15°/s one).

| profile | motion end | +1 s | +3 s | note |
|---|---|---|---|---|
| half-sine (task 12b, latching band) | 1.921° | 0.946° | 0.132° | |
| half-sine (task 12c, wall-clock expiry) | 1.921° | **0.732°** | **0.102°** | |
| trapezoid (task 12c) | 0.520° | 0.200° | 0.028° | unaffected — the trapezoid's rate never lingers in the band long enough to show a difference |

**Coverage added for the fixed rule** (host, `test/test_ahrs.c`):
arm-then-linger (0.5 s at 80°/s then a *sustained* 30°/s turn — the exact
regression scenario) recovers to the ramp value (`w_rate(30°/s)` = 66.7 %)
within 0.5 s of leaving ≥60°/s and never drops back to 0 over a further
60 s, with `s_fbI` resuming motion; a randomised (seeded) sequence of
sustained rate segments alternating arming (70–150°/s) and non-arming
(0–50°/s) bursts never shows `w_acc = 0` for more than 0.35 s after the
low-passed rate was last ≥60°/s (checked against a shadow replica of the
one-pole low-pass, away from the ±2°/s band around the knee where the
*ordinary ramp itself* — not the hold — genuinely publishes a rounded 0 %);
hover and the 300 s stationary case are unaffected (the hold can still only
arm above 60°/s).

**A third profile, below the arming knee entirely.** The two profiles above
both peak above 60°/s and so exercise the hold-off; SWE1-FW-006's clause (a)
now adds a **sub-60 half-sine** (peak 45°/s, 90° in `rampS = π ≈ 3.14 s`) that
never arms the hold at all — the only case exercising the bare `w_rate` ramp
end to end. Its lateral disturbance is *derived*, not copied, from its own
peak angular acceleration at a **0.43 m lever arm** (`a_t = α·r`): the
existing 94°/s half-sine's peak `α = rollDeg·π²/(2·rampS²) = 197°/s²`, and
`a_t = α_rad·r = 0.15 g` at `r = 0.43 m` — the same arm the trapezoid and
half-sine's shared 0.15 g already imply, made explicit rather than copied
verbatim into a slower case where it would imply an unphysical 1.9 m arm.
For the 45°/s half-sine, `α = 90·π²/(2·π²) = 45°/s² → a_t = 0.0344 g`.
Measured: 1.766° / 0.661° / 0.091° against the same ≤2.0/1.0/0.5° limits
(architect's prediction ≈1.6/0.59/0.08°), `s_accHoldS` never armed (host
tests it via `accWeightPct` never reading exactly 0 during the motion).

Bench, 125 s stationary, fw v1.19.18 (tasks 1–4, calibration restored from
`calibration/board.json` after the reflash): `g_dbgAhrsRealigns` = 1
throughout multiple back-to-back recordings, 0 stuck-presence drops, all
samples `state = RUNNING`/`accTrusted`/`magTrusted` = 1, `gyroBias0`/
`gyroBias1` (roll/pitch) peak-to-peak under 0.05 °/s. Yaw itself showed two
real, single-tick heading shifts (1.9–8.7°) uncorrelated with any re-align
(`g_dbgAhrsRealigns` never moved) or with a visible change in the polled raw
magnetometer reading — consistent with a genuine, brief environmental
magnetic disturbance on the open bench rather than an estimator defect (the
flight-architect separately traced run 1's 8.7° shift to bench motion and
`_r2`'s 1.9° shift to a single out-of-range gyro-z word integrated for one
tick — a pre-existing gap, no gyro-word bound anywhere in `Ahrs.c`, tracked
separately, not fixed here); a third, back-to-back recording without an
intervening disturbance measured yaw p2p 0.56°, roll p2p 0.33°, pitch p2p
1.97° (two isolated single-tick outliers, likely bench vibration), 0
sawtooth steps.

**Mag calibration is a bench prerequisite for the remaining yaw-while-tilted
sensitivity, not code** (deferred, needs Chris's hands): remount, full 3D
`tools/mag_cal.py`, corrected `|B|` spread < 5 % across six positions.

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

> `NavVarNorth` is short-term/relative uncertainty. It excludes the
> common-mode GNSS error, measured at 1.85 m sigma with a 32 s correlation
> time, and is not an absolute-position uncertainty. (`docs/NAV_TUNING.md`
> section 4.4; the sentence promised there, added 2026-09-14.)

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

**`gnssPosRScale` defaults to 1.0** (SWE1-FW-012, `docs/NAV_STRAND_2026-09.md`
§3.2/§9; was 8.0 — see §7 for why it started there). The offline sweep found
no setting puts NIS in 0.5-2.0 together with a fused-vs-raw ratio near 1x —
the receiver's GNSS error is coloured, not white, so that is not a tuning
failure — and 1.0 is the value that minimises the 2-D scatter ratio against
the raw fix (t1 1.65x -> 1.27x, every other recording <= 1.02x). `NavVarNorth`
is correspondingly smaller and reads as more confident; that confidence is
relative to the (still metre-class) raw fix, not an absolute accuracy claim.
The correct long-term fix remains a GNSS position-bias state, exactly as the
barometer has — this scale is still a stopgap, just a better-measured one.

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

---

## 11. The NAV filter strand, 2026-09 — slew clamp, trust gate, stationary lock

`docs/NAV_STRAND_2026-09.md` is the design record (SWE1-FW-010/-011/-014/-015);
this is the short version for whoever is reading this file to understand what
the estimator does today, not why.

**GNSS-altitude slew clamp (SWE1-FW-010).** The GNSS-altitude update
(`fusion_correctGnss()`, the `d`/`measBias` split) may move `d` by at most
`gnssAltSlewMps * dtFix` per fix — `gnssAltSlewMps` default 0.001 m/s
(`Xcp_FusionCal` `0x38`, `FusGnssAltSlewMps`), **0 is a defined value meaning
"the update is off"**, read directly, never through `FusionCal_positive()`. A
clamp that binds scales the Kalman gain, which is no longer optimal, so the
covariance update switches from the usual `P -= K(HP)` shortcut to the full
Joseph form (`fusion_chanJosephUpdate()`) — the shortcut is only valid for the
unscaled gain. Measured indoors (02F0B754CCA975C6, cold replay): unclamped
12.635 m peak-to-peak / 4.551 m max-per-60 s; at the 0.001 m/s default,
0.417 m / 0.289 m alone — the binding `<= 0.25 m` max-per-60-s clause on its
own narrowly misses on this recording (0.289 m), but combined with the trust
gate below (which refuses this recording's GNSS outright) the real number is
0.227 m and passes; see `docs/NAV_STRAND_2026-09.md` and the SWE1-FW-010 item
for the full replay evidence.

**Trust gate (SWE1-FW-011).** `fusion_correctGnss()` keeps its own trust
decision, `gnssHAccMax` (`0x3C`, default 4.0 m — separates every recorded
outdoor fix from every recorded indoor one), debounced on 30 consecutive good
fixes to enter and 10 consecutive bad ones to leave (never on level, so a
chattering `hAcc` cannot flip it). Untrusted skips position, velocity AND
altitude together; the origin, once latched, is kept. `horizontalOk`
(`Xcp_Fusion`, unchanged offset) is **no longer a latch** — it is
`originSet AND gnssTrusted AND a trusted fix fused within 2.0 s`, recomputed
every tick so a total GNSS outage clears it purely from time passing, not only
when a new (untrusted) fix happens to arrive. Past that 2.0 s the horizontal
channels hold position and velocity while `P` keeps growing from process
noise, which is what lets the gate reopen on its own. `gnssTrusted` publishes
at `Xcp_Fusion 0xBD` (`NavGnssTrusted`).

**Stationary lock (SWE1-FW-014) and its release (SWE1-FW-015).** While
`|omega| <= lockGyroDps` (2.0 deg/s) and `abs(|a|-1g) <= lockAccG` (0.03 g)
hold together for `lockWindowS` (1.0 s) **and** the ASW has called
`Fusion_setOnGround(TRUE)` — the airborne interlock, default TRUE, no ASW
owner calls it yet — the filter locks: a zero-velocity update (all three
channels, at the barometer's own rate) pins velocity without discarding its
covariance, and GNSS position/velocity/altitude are all skipped, so the
`d`/`measBias` split and the horizontal position are untouched by GNSS while
locked. Release needs 2 CONSECUTIVE samples past `relGyroDps`/`relAccG`
(3.0 deg/s / 0.05 g) — one corrupt IMU tick (a measured failure mode,
SWE1-FW-009) cannot release it. **There are two ways to release, and both go
through the same mechanism.** An IMU-detected release and an airborne-
interlock-driven one (`Fusion_setOnGround(FALSE)`) both land in
`Fusion_update()`'s own before/after check on `s_stationaryLocked`, the one
place `fusion_releaseGnssBias()` is ever called from — `Fusion_setOnGround()`
itself only raises a pending-release flag, it does not clear the lock
directly (flight-reviewer BLOCKER 1, 2026-09-14: it used to, which let an
interlock-driven release at arming skip the bias entirely and step position
by the whole frozen-vs-GNSS gap in one tick — measured 3.568 m in 1 s on a
5 m accumulated offset, against 0.0197 m through the IMU path). On either
release, the frozen-versus-GNSS difference becomes a bias (`Xcp_FusionCal
0x54-0x5C`: `sigmaZupt`, `tauGnssBiasS` = 60 s, `gnssBiasRateMax` = 0.05 m/s;
`gnssBiasMaxM` = 10 m stays a compiled `#define`) subtracted from every later
position fix and decayed over one GNSS error correlation time, so the
liftoff setpoint is the frozen point and nothing jumps, however the lock let
go. Measured: indoor 02F0B754CCA975C6 locks 99.79 % of a 469 s run,
`|v_horiz|` max 0.0004 m/s, posN/posE peak-to-peak 0.0027/0.0021 m — against
the 21.9 m north peak-to-peak this recording showed before any of this
strand existed.

`Xcp_FusionCal` grew `0x40 -> 128` bytes for the lock/release fields; the
struct itself only reaches `0x60` (96 bytes), so 32 bytes of headroom remain
before the next field needs `docs/CODEMAP.md`'s "next free slot" rule.
