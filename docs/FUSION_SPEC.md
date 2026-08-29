# Sensor fusion — behavioural specification

**ASPICE:** SWE.1 (pre-ASPICE) — behavioural spec of the estimator, to be absorbed into SWR-FW items · parent SYS-NAV-001/002, SYS-VER-001 · process: QuadSE/requirements/README.md

**This document exists to be tested against.** It states what the estimator must
do, never how it does it. Nothing here is derived from reading `fusion.c`,
`Ahrs.c` or `FusionCal.c`; it comes from Kalman filter theory, rigid-body
geometry, the interface contracts in the headers, and numbers measured on the
hardware.

> ⚠️ **If you are writing tests from this document, do not read those three .c
> files.** Tests derived from an implementation encode whatever it does,
> including its defects, and can then only detect regressions — never the bug
> that was there all along. This is not hypothetical: on 2026-08-26 a covariance
> update gathered the wrong matrix columns and drove the position variance to
> −inf while the estimate ran to +166 m. A suite written from that code would
> have been green.
>
> You **may** read `fusion.h`, `Ahrs.h`, `FusionCal.h` and `Measurements.h` —
> those are the contract, not the implementation.

---

## 1. What the thing is

Three sensors and a receiver feed two estimators.

```
accelerometer + gyro + magnetometer ──► attitude (quaternion)
                                              │
                          acceleration in NED, gravity removed
                                              │
barometer + GNSS ─────────────────────► three Kalman channels: DOWN, NORTH, EAST
                                        each: [position, velocity, accelBias, measBias]
```

Frame is **NED**: x north/forward, y east/right, **z DOWN**. Position `d` and
velocity `v_d` are positive *downward*. Barometric and GNSS altitude count
*upward*, so exactly one sign inversion happens per sensor as it enters.

Interfaces, units and argument order are in `fusion.h` and `Ahrs.h`. Read those.

---

## 2. Invariants — must hold for every input, always

These follow from theory, not from this codebase. They are the highest-value
tests because they can be checked on random inputs, indefinitely.

### 2.1 Covariance

For any sequence of predict and update steps, with any admissible inputs:

- **P is symmetric.** `P[i][j] == P[j][i]` to within floating-point tolerance.
- **P is positive semi-definite.** Every eigenvalue ≥ 0; equivalently every
  leading principal minor ≥ 0. A negative variance is meaningless and makes the
  gate and the gain meaningless with it.
- **Diagonal entries are non-negative and finite.** Never NaN, never ±inf.
- **A measurement update must not increase** the variance of the state it
  observes. Information cannot reduce certainty.
- **A predict step must not decrease** any diagonal entry when process noise is
  positive. Time cannot increase certainty.

### 2.2 Attitude

- **The quaternion stays unit norm**, to within tolerance, after any number of
  updates. A drifting norm silently becomes a rotation-plus-scale, and then the
  projected acceleration carries a gain error.
- **A rotation preserves magnitude**: for any vector v, `|R(q)·v| == |v|`.
- **Body↔NED round-trips**: `nedToBody(bodyToNed(v)) == v`.
- **The mounting transform has determinant +1.** A determinant of −1 is a
  left-handed frame; the gyro is a pseudovector and would then transform
  differently from the accelerometer, leaving the two halves of the filter
  disagreeing about which way is up. Verify the transform as a matrix.

### 2.3 Numerical robustness

- **No input produces NaN or inf in an output.** Including: dt of 0, negative,
  or absurdly large; accelerations of ±1e30; a zero-magnitude magnetic field; a
  zero-magnitude acceleration vector.
- **NaN arriving from outside must not propagate.** Calibration values reach the
  filter from an XCP master, which can write anything. A NaN tuning parameter
  must be rejected, not multiplied into a covariance. Note the trap: every
  comparison against NaN is false, so `x <= lo` does *not* catch it while
  `!(x > lo)` does.
- **A diverged filter must recover.** After being driven far from truth by
  corrupt measurements, the estimator must return to tracking within a bounded
  number of good samples. It must not reach a state where it rejects every
  subsequent measurement forever.

---

## 3. Analytic cases — known answers from physics

### 3.1 Attitude, board held still

Specific force at rest is `a_b = [sinθ, −cosθ·sinφ, −cosθ·cosφ]` in g.

| orientation held | expected |
|---|---|
| level, chip up | roll ≈ 0, pitch ≈ 0 |
| nose up | **pitch ≈ +90°** |
| right wing down | **roll ≈ +90°** |
| rotated 90° clockwise seen from above, level | yaw advances **+90°** |

Yaw increases clockwise from above (north → east → south → west), and is
reported in [0, 2π).

### 3.2 Acceleration projection

With the board level and at rest, gravity must be fully removed: the NED
acceleration output is **[0, 0, 0]** to within sensor noise. Tilting the board
without moving it must still give ≈ **[0, 0, 0]** — that is the entire point of
projecting through the attitude rather than assuming level.

### 3.3 Degenerate-tuning limits

These are exact, and a correct Kalman filter satisfies them by construction:

- **Measurement noise → 0**: the position estimate must equal the measurement
  after one update, and the corresponding variance must go to 0.
- **Process noise → 0, no measurements**: the state must follow the
  deterministic model exactly (free integration of the input).
- **Constant input, constant measurement, many steps**: the estimate must
  converge and stop moving.

### 3.4 Bias observability

The vertical channel carries two bias states, and they are observable only
under specific conditions. A correct implementation reflects that:

- With **only** a relative altitude sensor (barometer), the split between true
  altitude and that sensor's offset is **not** individually observable — only
  their sum is. The individual variance may therefore grow, but must remain
  **bounded** (the offset is modelled as mean-reverting, because atmospheric
  pressure does not wander to infinity).
- ⚠️ **The offset state is not zero without an absolute reference.** It carries
  process noise, so it random-walks; only the *sum* is pinned. A test asserting
  it stays at zero is asserting the opposite of the paragraph above.
- ⚠️ **The split does not recover to truth after a fault.** Once corrupt input
  has driven position and offset apart, only their sum can be restored — there
  is nothing to recover the individual values against. A correct implementation
  therefore **bounds** the excursion; it cannot undo it. Requiring recovery to
  zero is requiring the unobservable to be observed.
- Adding an **absolute** altitude source makes the offset observable and it must
  then converge.
- On the horizontal channels there is no relative sensor, so that bias state
  must stay **exactly zero** forever.

---

## 4. Differential test against an independent reference

The strongest layer. **Write your own Kalman filter in Python from the equations
in a textbook**, matching only the interface in `fusion.h` — the state vector,
the measurement model, the tuning parameters. Then drive both with identical
input sequences and compare.

The C and the reference must agree on state and covariance to within
floating-point tolerance over long runs. Where they disagree, one of them is
wrong; find out which by hand before changing anything.

This layer exists because it is the only one that reliably catches an error in
the matrix algebra itself. `P = F·P·Fᵀ + Q` gathering the wrong columns produces
a covariance that is still symmetric-looking and still finite for a while — it
passes casual inspection and fails a reference comparison on the first step.

---

## 5. Measured hardware values

Independent ground truth: these were measured on the board, not chosen. Use them
as acceptance bounds, not as exact expectations.

| quantity | value | conditions |
|---|---|---|
| \|a\| at rest | **0.998–1.005 g** | board stationary |
| accelerometer noise, down axis | 0.0144 m/s² | 30 s at rest |
| accelerometer bias, down axis | +0.018 m/s² | 30 s at rest |
| barometer short-term noise | **0.0197 m** | 60 s at rest, first-difference method |
| barometer drift | 0.19 m/min | still air, still board |
| \|B\| after calibration | **0.46 ± 0.03 G** | any orientation; ~0.48 G expected in Munich |
| climb-rate noise at rest | 0.0184 m/s | filter output |
| filter HF attenuation vs raw barometer | **3.6×** | first-difference std |
| 30 cm lift, held | +0.27 m sustained, innovation within ±0.02 m | smooth movement |
| GNSS fix rate | 10.00 /s | NAV-PVT at 100 ms |
| loop closure, ~15 × 9 m walk | 2.59 m | against hAcc 2.4 m |

Recorded datasets suitable for replay live in the scratchpad from the
2026-08-26 session (`walk2.mf4`, `walk2.csv`, lift and at-rest logs).

---

## 6. Defects that were real — a suite worth having catches these

Each of these shipped and was found on hardware, not by testing. They are stated
as *behaviour a correct implementation must not exhibit*, with no hint of how
the current code avoids them.

1. **Covariance propagation gathering the wrong terms.** Symptom: position
   variance reached −inf; the estimate free-integrated to +166 m while reporting
   a 1.8 m/s descent. Catchable by §2.1 and §4.
2. **A NaN froze an outlier counter.** Every comparison against NaN is false, so
   a rejection counter never advanced and the recovery path never fired: 11255
   rejections, one reset, stuck permanently. Catchable by §2.3.
3. **Partial covariance reset.** Re-acquisition restored the diagonal but left
   the off-diagonals holding diverged values, which the next predict mixed
   straight back in. Catchable by §2.1 after a forced re-acquisition.
4. **An outlier gate that fired on real motion.** The gate narrowed as the
   filter grew confident until an ordinary 30 cm hand movement was 62% of the
   way to being rejected. A gate must reject *corrupt* data, not *dynamics*.
5. **Unbounded variance from an unobservable split.** See §3.4.
6. **A mounting transform assumed rather than measured**, carried over from a
   different sensor on a different breakout. Catchable by §2.2 and §3.1.
7. **A calibration value of zero or NaN reaching a divisor.** Catchable by §2.3.

---

## 7. Scope

**In scope:** `fusion.c`, `Ahrs.c`, `FusionCal.c` — the estimator maths. These
are nearly pure functions of their inputs and are where the defects have been.

**Out of scope:** device drivers (they need hardware), the XCP transport, lwIP.

**Deliberately not specified here:** internal function names, file structure,
the order of operations inside an update, and which intermediate quantities are
stored. Those are implementation choices. If a test needs to know one of them,
the test is testing the implementation and not the requirement.
