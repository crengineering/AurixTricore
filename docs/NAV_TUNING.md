# NAV_TUNING.md — outdoor tuning pass on the navigation filter

Design draft. Status: **proposal, nothing implemented.** Evidence is the five
MF4 traces flown 2026-08-28 (`t1_5minutes_still`, `t2_rechteck_20_10`,
`t3_50m`, `t4_figure_8`, `t5_stop_and_go_20m`) plus a Riccati replay of
`fusion.c`'s own covariance recursion.

Scope: `src/bsw/fusion.c`, `src/bsw/FusionCal.c/.h`, `docs/FUSION.md` sections
2 and 7. Out of scope: the magnetometer (|B| spread 55 % while turning) — noted
in section 7 only where it bounds what this tuning can achieve.

---

## 1. Verdict

**The estimator's acceleration process noise is specified per *tick*, not per
*second*, so raising the rate from 50 Hz to 1014 Hz in PR #15 silently divided
the effective acceleration noise PSD by 20.3 and the velocity Kalman gain by
4.1.** `fusion.c:407-412` implements the discrete piecewise-constant-acceleration
Q, in which `sigmaA` is the acceleration error *held over one interval*:

```c
ch->p[FS_POS][FS_POS] += (qa * dt2 * dt2) / 4.0f;   /* fusion.c:407 */
ch->p[FS_POS][FS_VEL] += (qa * dt2 * dt) / 2.0f;    /* fusion.c:408 */
ch->p[FS_VEL][FS_VEL] += qa * dt2;                  /* fusion.c:410 */
```

Accumulated over one second the velocity term is `f * qa * dt^2 = qa/f`. It is
**inversely proportional to the update rate**. With `FCAL_SIGMA_ACC_H = 0.5`
(`FusionCal.c:37`) the effective PSD was 5.0e-3 (m/s^2)^2/Hz at 50 Hz and is
2.46e-4 at the measured `FusionDt` = 0.000985 s (t1). Consequence, from the
covariance recursion (data-independent, so this is exact, not fitted):

| | 50 Hz (pre-PR #15) | 1015 Hz (as flown) |
|---|---|---|
| `P[vel][vel]` steady state | 0.00647 | **0.00151** |
| GNSS-velocity gain `K_vel` | 0.0671 | **0.0165** |
| velocity correction time constant | 1.49 s | **6.05 s** |
| `P[pos][pos]` (t1, hAcc 2.28) | 0.192 | 0.191 |

The recursion predicts `P[pos][pos]` = 0.1911 against a logged `NavVarNorth`
mean of 0.1863, and 0.1012 against 0.1023 for the dynamic runs — the model of
the filter is right, so the numbers above can be trusted.

**Confidence: high** for the velocity finding (Finding 2 and t5). The other two
observations resolve differently:

- **Finding 3 is not a defect — close it.** `NavVarNorth` did not diverge in
  t1; it tracked `GnssHAccuracy`, which drifted 1.135 -> 3.347 m over the run.
  `corr(NavVarNorth, GnssHAccuracy) = 0.988`, ratio `varN/hAcc` = 0.0812 +/-
  0.0043 (0.5 % scatter over 3009 samples). The recursion at hAcc 1.135 gives
  0.095 and at 3.347 gives 0.28, against a logged 0.0927 -> 0.2760. It is
  `R = hAcc^2 * gnssPosRScale` moving, nothing else. In the dynamic runs hAcc
  sat at 1.25 and `NavVarNorth` sat at 0.102, as it should.
- **Finding 1 is real but pre-existing and rate-independent**, and the stated
  interpretation is wrong — see section 3. `P[pos][pos]` is unchanged by PR #15.

---

## 2. What the velocity error actually costs — the cleanest single number

In the **static** run the filter makes east position **3.2x worse than the raw
receiver**:

| t1, 301 s stationary | raw GNSS | filter | ratio |
|---|---|---|---|
| north position sigma | 1.852 m | 1.800 m | 0.97 |
| east position sigma | **0.918 m** | **2.916 m** | **3.18** |
| north peak-to-peak | 7.63 m | 6.65 m | |
| east peak-to-peak | 4.40 m | 11.78 m | 2.7 |

(raw north/east computed from `GnssLatitude`/`GnssLongitude` on the same
tangent plane, 111132 and 111320*cos(48.92 deg) m/deg.)

The mechanism is arithmetic: static `NavVelEast` sigma = 0.145 m/s, and the
position channel's own correction time constant is `0.1/K_pos` = 21.9 s, so
free-integrated velocity error contributes 0.145 * 21.9 = **3.2 m** of position
error. Observed 2.92 m. Velocity error is being converted into position error
faster than GNSS position removes it.

Walking runs, comparing like with like (filter `hypot(NavVelNorth, NavVelEast)`
against `GnssGroundSpeed`, both 2-D magnitudes):

| | filter p95 | GNSS p95 | filter max | GNSS max |
|---|---|---|---|---|
| t2 rectangle | 3.07 m/s | 1.29 m/s | 3.68 | 1.53 |
| t4 figure-8 | 3.92 m/s | 1.56 m/s | 5.17 | 1.89 |
| t1 static | 0.36 m/s | 0.42 m/s | 0.55 | 0.99 |

2.4-2.5x on p95, so the overshoot is not a peak artefact. Note it is **absent
statically** — the filter only overshoots when there is gait or manoeuvre
content to integrate, which is exactly what a 6 s velocity correction time
constant predicts.

---

## 3. Finding 1 re-diagnosed: coloured GNSS noise, not over-trust

The hypothesis under test was that innovation sigma ~ scatter sigma means "the
filter is being dragged by GNSS". **The data refutes that.** `K_pos` = 0.0046
per fix: the filter moves 0.46 % of the way to each GNSS position. It is barely
dragged at all. What the three near-equal numbers mean instead:

```
raw GNSS north sigma  1.852 m
filter posN    sigma  1.800 m      <- smoothing removed about 3 %
innovation     sigma  1.891 m      <- the full GNSS error is still in the innovation
```

A 21.9 s smoother that removes nothing is telling you the GNSS error has no
white component on that timescale. Measured directly: the **1/e
autocorrelation time of the raw GNSS north error in t1 is 32.4 s** (white noise
would give 0.1 s). Multipath and ionospheric wander, not receiver noise.

Two consequences, both worth writing down:

1. **No value of `R` fixes the `P`-vs-scatter gap.** `P[pos][pos]` scales as
   sqrt(R), so reaching an honest 3.24 m^2 from 0.191 m^2 would need
   `gnssPosRScale` ~ 2300. `gnssPosRScale = 8` (`FusionCal.c:50`) was set
   against this symptom and is the wrong lever: it buys sqrt(8) = 2.8x of `P`
   and pays 8x of position lag.
2. **`R` is meanwhile far too *large* for the innovations it sees.** Normalised
   innovation squared, `NIS = var(y)/(P + R)`, over the 3009-sample static run:

   ```
   R = hAcc^2 * 8 = 2.283^2 * 8 = 46.1 m^2,  P = 0.186
   NIS_north = 3.578 / 46.3 = 0.077
   NIS_east  = 5.765 / 46.3 = 0.124
   ```

   A correctly scaled `R` gives NIS ~ 1. The filter is 8-13x over-conservative
   about each individual fix while being 17x over-confident about its own
   answer. Those are not contradictory — they are the two halves of modelling
   coloured noise as white.

**The DOWN column of Finding 1 is an apples-to-oranges comparison and should be
withdrawn.** `NavVarDown` is `p[FS_POS][FS_POS]` (`fusion.c:1086`), but the
barometer observes `h = [1,0,0,1]` (`fusion.c:779`), i.e. `d + measBias`. The
innovation covariance is `p00 + 2*p03 + p33 + R`, not `p00 + R`. Measured:
`var(NavInnovDown)` = 3.65e-4 against `sigmaBaro^2` = 3.88e-4 — the
**observable** vertical quantity is consistent to 6 %. The 1.19 m of
`NavPosDown` scatter is the unobservable `d`/`measBias` split wandering with
GNSS altitude, which itself drifted 5.66 m with sigma 3.75 m in t1. The
vertical channel is fine.

---

## 4. Recommended changes

Four cal writes, then one rebuild. Ordered so the firmware flies at every step
and each step is independently measurable and reversible by a power cycle
(`FusionCal.h:16` — the block is RAM-only).

### 4.1 Live, through `Xcp_FusionCal` — no rebuild

Rate-scale factor: `sqrt(dt_50Hz / dt_now)` = `sqrt(0.02 / 0.000985)` = **4.506**.

| field | offset | now | proposed | arithmetic |
|---|---|---|---|---|
| `sigmaAccH` | 0x20 | 0.5 | **2.25** | 0.5 * 4.506 = 2.253 — restores the 50 Hz PSD exactly |
| `sigmaAccD` | 0x10 | 0.3 | **1.35** | 0.3 * 4.506 = 1.352 |
| `sigmaGnssVel` | 0x24 | 0.3 | **0.15** | halves the velocity R; u-blox M9N `sAcc` under a clear sky is typically under 0.1 m/s, and 0.3 was a placeholder (`fusion.c:70-74`) |
| `gnssPosRScale` | 0x28 | 8.0 | **1.0** | drives NIS 0.077 -> 0.68 (north) / 1.10 (east) |

Predicted effect (Riccati, hAcc = 2.283):

| sigmaAccH | sigmaGnssVel | rScale | P_pos | K_vel | **tau_vel** | tau_pos | NIS_N |
|---|---|---|---|---|---|---|---|
| 0.5 | 0.3 | 8 (today) | 0.191 | 0.0165 | **6.05 s** | 21.9 s | 0.08 |
| 2.25 | 0.3 | 8 (step 1) | 0.195 | 0.0700 | **1.43 s** | 21.5 s | 0.09 |
| 2.25 | 0.15 | 8 (step 2) | 0.098 | 0.1318 | **0.76 s** | 42.9 s | 0.09 |
| 2.25 | 0.15 | 1 (step 3) | 0.034 | 0.1317 | **0.76 s** | 15.2 s | 0.68 |

`sigmaGnssVel` and `gnssPosRScale` are near-orthogonal in their effect — the
first sets tau_vel, the second sets NIS and tau_pos — so they can be swept
independently in one bench session.

tau_pos rising at step 2 is an artefact of `P_pos` falling, not of slower
tracking; position is corrected mostly *through* velocity once tau_vel is
0.76 s. Step 3 brings it back to 15 s.

**Do steps 1 and 3 first and separately.** Step 1 is a pure regression fix
(undo the rate change). Step 3 is a genuine retune of a number that was set
against a misread symptom. Step 2 is optional and the least certain, because
the true `sAcc` is not decoded.

### 4.2 Rebuild — make the tuning rate-invariant

The cal writes above are correct for 1014 Hz and wrong for any other rate. Fix
the parametrisation so this cannot happen again. In `fusion_chanPredict`
(`fusion.c:407-410`), replace the piecewise-constant-acceleration Q with the
continuous white-noise-acceleration Q:

```c
/* sa is now a PSD: [m/s^2 / sqrt(Hz)]. q*dt is rate-invariant. */
const float32 q = sa * sa;
ch->p[FS_POS][FS_POS] += (q * dt2 * dt) / 3.0f;
ch->p[FS_POS][FS_VEL] += (q * dt2) / 2.0f;
ch->p[FS_VEL][FS_POS]  = ch->p[FS_POS][FS_VEL];
ch->p[FS_VEL][FS_VEL] += q * dt;
```

New defaults, equal to the values chosen in 4.1 at the current rate:

| | old (per-tick) | new (PSD) | arithmetic |
|---|---|---|---|
| `FCAL_SIGMA_ACC_H` | 0.5 | **0.0707** | 0.5 * sqrt(0.02) |
| `FCAL_SIGMA_ACC_D` | 0.3 | **0.0424** | 0.3 * sqrt(0.02) |

Unit changes from `m/s^2` to `m/s^2/sqrt(Hz)`. **Struct offsets do not move**
(same two `float32` at 0x10 and 0x20), so this is a value + name + unit change
in the A2L and the GUI label, not a layout change — but it *is* an A2L/GUI
contract change and any saved tuning file becomes wrong by 7x. See section 7.

### 4.3 Rebuild — two things the tuning pass needs and cannot get today

- **Publish the GNSS velocity innovation.** `fusion.c:976-980` discards `yv`
  for both channels. Without it the velocity NIS cannot be computed, and the
  velocity channel is the one being retuned. Costs two `float32` in
  `FusionValues` / `Xcp_Fusion`; append at the end so nothing moves
  (`docs/CODEMAP.md`).
- **Make `FUSION_SIGMA_ACC_RW` live-tunable.** It is hardcoded at
  `fusion.c:411` (1.0e-4). The bias state has to absorb attitude-driven
  horizontal acceleration error — measured `NavAccBiasEast` ranged -0.197
  (t2) to +0.307 (t5) — while `P[accB][accB]` converges to 2e-6, i.e. sigma =
  0.0014 m/s^2. It is being asked to move 100x its own claimed uncertainty.
  `FusionCal.h:87` has `float32 reserved[3]` at 0x34; taking `reserved[0]`
  moves nothing.

**Not recommended:** modelling the GNSS position bias as an extra
mean-reverting state per horizontal channel. It is unobservable with one
absolute source, so its variance would be a constant the filter learns nothing
about — the same information as a published floor (4.4) at the cost of a fifth
state, a wider `P`, and per-tick work on the 1014 Hz path. Reject.

### 4.4 Optional — publish an honest position variance

`pNN` is the filter's *modelled* variance and will stay around 0.03-0.19 m^2
however this is tuned, because the 32 s GNSS bias is outside the model. If
anything downstream ever consumes it (nothing does today; the gate is
R-dominated), it must be `pNN + (k*hAcc)^2`. Do this **only** when a consumer
exists — otherwise it is a field in the A2L that nobody reads. Until then, one
sentence in `docs/FUSION.md` section 7: *"`NavVarNorth` is short-term/relative
uncertainty. It excludes the common-mode GNSS error, measured at 1.85 m sigma
with a 32 s correlation time, and is not an absolute-position uncertainty."*

---

## 5. Validation — same five manoeuvres, numeric pass criteria

Compute offline from the MF4; the reference implementation is
`tools/mf4_stats.py` plus the tangent-plane conversion used in section 2.

**Consistency (the primary test).** `NIS = var(y) / (P + R)`, per channel, over
a run of at least 1000 fixes. For the GNSS position channels `S` is
reconstructable from logged signals:
`S = NavVarNorth + GnssHAccuracy^2 * gnssPosRScale`.

| test | today | **accept** |
|---|---|---|
| NIS north, t1 | 0.077 | **0.5 - 2.0** |
| NIS east, t1 | 0.124 | **0.5 - 2.0** |
| NIS baro (`var(NavInnovDown)/sigmaBaro^2`) | 0.94 | 0.5 - 2.0 (already passes — must not regress) |
| NIS GNSS velocity | not logged | 0.5 - 2.0 (needs 4.3) |

A band rather than a point: the sampling CI at n = 3000 is +/-3 %, so anything
tighter would be testing the weather, not the filter. Outside 0.5-2.0 the model
is wrong by more than a factor of two and there is something to fix.

**Static, t1.**
- `std(NavPosEast)` <= 1.2 * raw GNSS east sigma. Today 2.916 vs 0.918 (**3.18x**).
- `std(NavPosNorth)` <= 1.2 * raw GNSS north sigma. Today 1.800 vs 1.852 (0.97 — passes).
- filter `|v|` p95 <= 0.15 m/s, max <= 0.40 m/s. Today 0.36 / 0.55.
- `std(NavVelDown)` <= 0.05 m/s. Today 0.0139 — must not regress when
  `sigmaAccD` goes to 1.35.
- `std(NavInnovDown)` in 0.015 - 0.030 m. Today 0.0191.

**Walking, t2 and t4.**
- p95 filter `|v|` <= 1.25 * p95 `GnssGroundSpeed`. Today 2.38x (t2), 2.51x (t4).
- max filter `|v|` <= 1.5 * max `GnssGroundSpeed`. Today 2.41x, 2.74x.
- `NavGnssRejects` = 0. Today 0 — must not regress.

**Sharp dynamics, t5.**
- `NavGnssRejects` = 0. Today 6.
- max `|NavInnovEast|` <= 5 m. Today 21.9 m.
- max `NavVarNorth` <= 0.5 m^2. Today 3.81 (rejection inflation).

**Structural invariants, all five runs.** `NavDropped` = 0, `NavCovResets` = 0
(the 2 at t1 startup are the boot transient and are accepted),
`NavBaroRejects` = 0, `NavBaroResets` = 0, `NavGnssUpdates` increments within
1 % of `delta(gnssITow)/100 ms`. All hold today; any of them breaking means the
tuning has destabilised the filter, not merely mistuned it.

**Order of the bench session.** Re-fly t1 after step 1 (velocity only should
change), t1 again after step 3 (NIS only should change), then t2/t4/t5. Five
short runs per step, not one long one — the GNSS bias has a 32 s correlation
time, so a 60 s run gives roughly two independent samples of it.

---

## 6. The t5 latency question — not latency, and the data cannot measure latency

**t5 is not a GNSS-latency signature. It is section 1 at its worst.** The
window around the excursion, from the 10 Hz log:

```
 t[s]   GnssGroundSpeed  |a_h|   NavVelN  NavVelE  InnovEast  NavVarN  rejects
 23.2       2.01          3.46    -0.71    -4.48     16.90     0.101      0
 23.4       1.74          2.34    -0.54    -4.88     17.91     0.201      1
 23.6       1.74          1.20    -1.02    -5.12     19.27     0.751      3
 23.9       1.06          3.56    -0.63    -5.72     21.30     3.814      6
 24.0       0.73          7.49    -0.96    -5.06     21.93     2.860      6   <- re-accepted
 24.3       0.36          5.01    -1.85    -1.57      8.32     1.657      6
 24.6       0.34          3.02    -2.10     0.02      0.09     1.172      6
```

`NavVelEast` = **-5.1 m/s while the receiver reported 1.7 m/s of ground speed
in total**. `InnovEast` grows at 0.68 m per 0.1 s = 6.8 m/s, which is that
velocity error, not a GNSS delay: at the true 2 m/s, a latency large enough to
explain 21.9 m would have to be **11 seconds**. The 5-sigma gate,
`sqrt(25*(P+R))` with `R = 1.26^2 * 8 = 12.7`, sits at 17.9 m; the innovation
crossed it at t = 23.4 s, six samples were rejected, `FUSION_REJECT_INFLATE`
(`fusion.c:118`) doubled `P` each time until the gate reopened at t = 24.0 s
(`NavGnssUpdates` resumes 3236 -> 3237), and the velocity was pulled back from
-5.06 to -0.26 m/s in 0.5 s. **The escape hatch worked exactly as designed.**
Fix the velocity and the excursion does not happen; the rejects and the
`NavVarNorth` = 3.81 spike are consequences, not causes.

**The data cannot support a latency number, and none is estimated here.**
Cross-correlating `d(GnssGroundSpeed)/dt` against horizontal `|a|` peaks at lag
+1 sample (0.10 s) with rho = **-0.267** — sign inverted and far too weak to be
a measurement; adjacent lags 0 and +2 are within 0.04 of it, so the resolution
is worse than the 0.1 s logging period anyway. The `GnssGroundSpeed`-versus-
filter-`|v|` cross-correlation peaks at **-1.4 s** (rho = 0.475), which is the
filter leading by its own overshoot, not the receiver lagging.

**Recommendation: documentation, not a fix.** Re-measure after the tuning
change (task T4). If a real number is then wanted it needs a separate change —
the estimator logged at 50 Hz or better, and the fix timestamped against `iTOW`
at the decoder rather than at consumption — and that should be justified by a
requirement, not by t5.

---

## 7. Risks

- **`gnssPosRScale` 8 -> 1 makes `NavVarNorth` *smaller* (0.19 -> 0.034 m^2),
  not larger.** Anyone reading that field as "how well do I know where I am"
  will read a worse number after a change that improves the filter. It also
  narrows the GNSS gate from +/-34 m to +/-12 m at hAcc 2.28, which is the
  change most likely to produce new `NavGnssRejects`. Watch that criterion
  specifically; `FUSION_GNSS_GATE_MIN_M` = 10 m (`fusion.c:102`) is the floor
  that protects it. **This is the riskiest of the four cal writes — do it last
  and alone.**
- **`sigmaAccD` 0.3 -> 1.35 makes the vertical channel noisier.** The barometer
  is the one part of this filter that is currently well tuned. The
  `std(NavVelDown)` <= 0.05 m/s and `std(NavInnovDown)` criteria exist to catch
  it. If it regresses, leave `sigmaAccD` alone: the vertical channel is
  measurement-dominated and does not have the horizontal channel's problem.
- **A2L/GUI contract.** Section 4.2 changes the *meaning* of
  `sigmaAccD`/`sigmaAccH` by a factor of 31.9 without moving a byte — the most
  dangerous kind of change, because a stale GUI writes a plausible-looking
  number that is 32x wrong. Mitigation: rename the A2L characteristics
  (`sigmaAccD` -> `sigmaAccDPsd`) so a stale GUI fails to find them instead of
  writing to them. Section 4.3 grows `Xcp_Fusion` by two `float32` appended at
  the end — three GUI edits per the ICM-42688 experience, and
  `tools/gen_a2l.py` + `a2l_meta.json` must be regenerated or `a2l.yml` fails.
- **Not reversible without a reflash:** 4.2 and 4.3. Everything in 4.1 is
  RAM-only and reverts on a power cycle.
- **Needs hardware and clear sky:** every acceptance number in section 5. None
  of this is testable on the bench, and the host tests in `test/` can only cover
  the Q reparametrisation (4.2) — which is exactly why that one is worth a unit
  test.
- **Magnetometer interaction, bounding what 4.1 can achieve.** |B| spread is
  0.5 % static but 55 % in t2/t4. A yaw error rotates the NED acceleration, and
  the horizontal channels integrate the result — so part of the walking-run
  velocity error is upstream of `fusion.c` and will not be tuned away. Raising
  `sigmaAccH` is the honest response to it (the filter *should* distrust a
  tilt-contaminated acceleration) but not a cure. If the t2/t4 criteria fail
  after the tuning, the magnetometer is the next suspect, not more `sigmaAccH`.
  Handled separately.
- **`FusionDt` minimum is 0.000118 s in t2** (mean 0.000978). The Q terms stay
  finite and `FUSION_DT_MIN` = 1e-4 (`fusion.c:150`) is not tripped, but the
  comment at `fusion.c:147` still says "The IMU task runs at 50 Hz" — stale
  since PR #15 and worth correcting while in the file.

---

## 8. Ordered tasks

Each builds and flies on its own. T1-T4 are XCP writes with no build at all.

| # | files | change | acceptance |
|---|---|---|---|
| **T1** | none (XCP write) | `sigmaAccH` 0.5 -> 2.25, `sigmaAccD` 0.3 -> 1.35 | re-fly t1 + t2: static filter speed p95 <= 0.15 m/s; t2 p95 <= 1.6x p95 `GnssGroundSpeed`; `std(NavVelDown)` <= 0.05; `NavCovResets` unchanged. Hardware. |
| **T2** | none (XCP write) | `gnssPosRScale` 8.0 -> 1.0 | re-fly t1: NIS_N and NIS_E in 0.5-2.0; `std(NavPosEast)` <= 1.2x raw GNSS east sigma; `NavGnssRejects` = 0. Hardware. |
| **T3** | none (XCP write) | `sigmaGnssVel` 0.3 -> 0.15, only if T1 left t2/t4 outside 1.25x | t2 + t4 p95 criteria in section 5. Hardware. Skip if T1 already passes. |
| **T4** | none | re-fly t5 unchanged from T1-T3 | `NavGnssRejects` = 0, max abs `NavInnovEast` <= 5 m, max `NavVarNorth` <= 0.5. Hardware. Closes section 6. |
| **T5** | `FusionCal.c` (`FCAL_SIGMA_ACC_D/H`), `docs/FUSION.md` section 2 | bake the T1/T3 winners in as compiled defaults, still in per-tick units | builds; XCP read-back of `g_fusionCal` matches; MISRA clean. |
| **T6** | `fusion.c:407-410`, `FusionCal.c:32/37`, `FusionCal.h` comment block, `test/` | reparametrise Q to a PSD (4.2); defaults 0.0424 / 0.0707 | **host unit test**: for the same wall-clock interval, `P[vel][vel]` growth agrees within 1 % across dt = 0.02, 0.001 and 0.0005 — that test is the point of the task. Builds, MISRA clean. |
| **T7** | `tools/a2l_meta.json`, generated `docs/AurixTricore.a2l`, GUI (delegate to `aurix-gui`) | rename `sigmaAccD`/`sigmaAccH` -> `sigmaAccDPsd`/`sigmaAccHPsd`, unit `m/s^2/sqrt(Hz)` | `a2l.yml` green; GUI shows the new names; a stale GUI finds nothing rather than writing a 32x-wrong value. |
| **T8** | `fusion.c` (publish `yv`), `fusion.h` (`FusionValues` + 2 `float32` **appended**), `Measurements.c` / A2L / GUI per `docs/CODEMAP.md` | publish `innovVelN` / `innovVelE` | offsets of every existing field unchanged in `fusion.src`; velocity NIS computable from a new trace. |
| **T9** | `fusion.c:411`, `FusionCal.h` (`reserved[0]` -> `sigmaAccRw`), A2L | make the accel-bias random walk live-tunable | offsets unchanged (0x34 was reserved); sweep on hardware only after T1-T4 have settled. |
| **T10** | `fusion.c:147-151` comment, `docs/FUSION.md` section 7 | correct "the IMU task runs at 50 Hz"; add the coloured-GNSS-error note from 4.4 and the `NavVarNorth`-is-relative sentence | `python tools/check_docs.py` green. |

T1-T4 are one afternoon with a laptop and clear sky, and are where all the
benefit is. T5-T10 are what stops it happening again.
