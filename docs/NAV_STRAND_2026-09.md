# NAV_STRAND_2026-09.md — the navigation filter strand, September 2026

**ASPICE:** SWE.2/3 — firmware design against
`QuadSE/requirements/SWE1/SWE1-FW-010`, `-011`, `-012`, `-013` ·
chain `SYS1-002 / SYS1-004 / SYS1-016 -> SYS2-NAV-001 / -002 / -003 +
SYS2-VER-001 -> SWE1-FW-010..013` · dispatch
`QuadSE/dispatch/SYS1-002 — Dispatch.md` · process
`QuadSE/requirements/README.md`

**Gate 2 approved by Chris, 2026-09-14** with four decisions, folded in below:
`gnssAltSlewMps` = 0.001 m/s; **`gnssAltSlewMps` = 0 means "GNSS-altitude
update off"**, so section 3.1 alternative (c) becomes a calibration value
rather than a code change; the `vAcc/hAcc` criterion is deferred to two more
indoor recordings in task 8; the `horizontalOk = 0` fallback is a separate
SYS2-SAF / SYS2-CTRL-002 item. `SYS2-NAV-001` and `SYS2-NAV-003` now carry the
amendments proposed in section 9. **Nothing implemented, nothing flashed,
nothing written to the board by this note.** Supersedes nothing: `docs/NAV_TUNING.md` stays valid for what
it measured, and this note says explicitly where its numbers are stale.

---

## GATE 2b — what Chris is being asked to approve, in one package

Two things, deliberately together, because the second changes what the first is
worth.

**2b-1. The horizontal amendment (section 3.2, revised after the offline
sweep).** `gnssPosRScale` 8.0 -> 1.0 stands. Three things approved at gate 2
change: `sigmaGnssVel` **stays at 0.30** (the 0.12 proposal is withdrawn — it
made the fused position worse, 1.59x against 1.27x); **NIS is demoted from a
gate to a reported diagnostic** (no setting satisfies NIS 0.5-2.0 together with
the ratio — that is coloured noise, not mistuning); and the never-worse-than-raw
criterion moves from **per channel to 2-D**, at <= 1.35x and never worse than
today. Measured 2-D ratio today -> recommended: t1 **1.65 -> 1.27**, t2 1.00,
t4 1.02, t5 0.99, indoor 0.96. **Stated plainly: 1.0x raw is not reachable with
this receiver.**

**2b-2. The stationary lock (section 10, new).** SWE1-FW-014 and SWE1-FW-015.
While the IMU says the vehicle is standing still AND the application says it is
on the ground, the filter stops integrating: velocity zeroed by a ZUPT,
position untouched by GNSS, accelerometer bias free to learn — which is the
payoff, about 20x better bias and open-loop drift falling from 32 m to 1.8 m
per 60 s. Release is continuous: the frozen-versus-GNSS difference becomes a
declared bias that decays over one error-correlation time (60 s), so **the
liftoff setpoint is the frozen point and nothing jumps**.

Why together: 2b-1 says the tuning cannot make the fused position better than
the receiver. 2b-2 says that at rest it does not have to — the receiver is
simply not consulted. The residual metre-class wander in 2b-1 is then confined
to the part of the flight where the vehicle is actually moving.

**One thing to take to Chris now rather than after the first flight:** his
"hover wander <= 1 m over 2 minutes" target is **not reachable with the
NEO-M9N**. The RAW fix walks a 2.50 m median / 5.99 m maximum radius over
sliding 120 s windows on the outdoor t1 recording. That is the receiver. The
stick-push target is reachable and the 2 m pattern is reachable; section 10.6
has all three with numbers.

---

Constraint that shapes the whole verification plan: **outdoor recording is not
currently possible** (Chris, 2026-09-14). Every number below is either measured
on a recording that already exists or produced by replaying one.

---

## 1. Verdict

**The altitude estimate is not drifting — the barometer is perfect and the
filter is throwing its answer away on an unobservable split that GNSS altitude
drags around.** Over the 469 s indoor recording (`02F0B754CCA975C6`, fw
v1.19.24) `NavPosDown + NavBaroBias` is constant to **0.267 m peak-to-peak**
while each of the two moves **9.3 m**, with
`corr(NavPosDown, -NavBaroBias) = 0.9998`. The barometer itself moved 0.320 m
peak-to-peak and its innovation never exceeded 0.067 m. The only thing pushing
the split is the GNSS-altitude update at `fusion.c:1027-1044`, fed by a
`GnssAltitude` that swung **35.84 m** with a 64.8 s 1/e autocorrelation time;
the fitted per-fix gain is 4.87e-4 against a theoretical `P/(P+R)` of 5.70e-4,
which at 10 Hz and an 8 m innovation is 0.039 m/s — exactly the observed
maximum of 3.50 m per 60 s. The weight is not mistuned: that update's NIS is
**0.547**, inside the 0.5-2.0 consistency band. It is the *rate* that is wrong,
because a white-noise-optimal gain applied to an error with a 65 s correlation
time transports the whole error into the state. Horizontally the picture is
different and equally clear: at rest indoors the **raw** receiver sat a mean
14.90 m (max 27.53 m) from its own latched origin while reporting `hAcc`
5.45 m, and the fused output was *better* than raw (`std` 6.30 vs 7.58 m north,
3.65 vs 3.90 m east). There is no tuning for a 15 m receiver bias — and
`horizontalOk` said `1` for all 469 s of it, because it is a latch
(`fusion.c:560` sets `anchored`, `:1045` reads it, nothing ever clears it).

---

## 2. The recording, in numbers

`2026-09-14_SYS2-NAV-003_lift-0p5m.mf4`, 469.1 s, 4691 samples at 10.00 Hz,
`GnssFixType` 3 throughout, 10-15 satellites, `GnssNavOk` 1,
`NavVerticalOk` 1, `NavHorizontalOk` 1, `NavGnssRejects` 0, `NavBaroRejects` 0,
`NavCovResets` 0, `NavDropped` 0.

### 2.1 What moved Up — the answer is "both, because they are one thing"

| signal | mean | std | min | max | p2p | drift |
|---|---|---|---|---|---|---|
| `NavPosDown` | 6.357 | 3.227 | 2.597 | 11.938 | **9.342** | +4.495 |
| `NavBaroBias` | -6.394 | 3.218 | -11.899 | -2.598 | **9.301** | -4.454 |
| `NavPosDown + NavBaroBias` | -0.038 | 0.068 | | | **0.267** | |
| `BaroAltitude` | 427.82 | 0.070 | 427.66 | 427.98 | **0.320** | -0.043 |
| `NavInnovDown` | -0.0001 | 0.0181 | -0.067 | 0.065 | 0.132 | |
| `GnssAltitude` | 548.01 | 9.346 | 528.97 | 564.81 | **35.84** | -4.405 |

`corr(NavPosDown, -NavBaroBias) = 0.9998`; `corr(NavPosDown, zD) = 0.544`;
`std(NavPosDown)/std(zD) = 0.345`. Baro wander over any 60 s window: max
**0.217 m**; `NavPosDown` over any 60 s window: max **3.50 m**, median 1.90 m.
GNSS-altitude innovation `yd = zD - d`: mean 1.76 m, std 8.06 m, max 16.4 m.
1/e autocorrelation of the raw GNSS altitude error: **64.8 s**.

So: the GNSS-altitude update moved `d`, and `measBias` moved by exactly minus
the same amount because the barometer re-imposes the sum on the next tick. It
is one mechanism with two published faces, not two findings.

### 2.2 The receiver's own accuracy claims — and why vAcc is not the lever

| run | hAcc min-max [m] | vAcc min-max [m] | vAcc/hAcc median |
|---|---|---|---|
| indoor 2026-09-14 | 4.612 - 6.849 | 5.107 - 6.829 | 1.069 |
| outdoor t1 (still) | 1.135 - 3.347 | 2.331 - 5.552 | 1.833 |
| outdoor t2 (rectangle) | 1.149 - 1.413 | 2.131 - 2.576 | 1.819 |
| outdoor t3 (50 m) | 1.413 - 2.580 | 2.201 - 3.510 | 1.376 |
| outdoor t4 (figure 8) | 1.174 - 1.419 | 2.004 - 2.200 | 1.629 |
| outdoor t5 (stop-and-go) | 1.161 - 1.373 | 1.923 - 2.232 | 1.705 |

`2*hAcc` is larger than `vAcc` in **every** run. The present proxy is therefore
uniformly the *conservative* choice, and the dispatch's premise ("weight by
vAcc instead of a proxy") would roughly quadruple the gain: `R` falls 3.47x
indoors. Replayed, that takes indoor Up wander from 11.41 m to **23.39 m**.

### 2.3 Horizontal, at rest indoors

| | raw GNSS | fused | ratio |
|---|---|---|---|
| north std / p2p | 7.582 / 24.59 m | 6.302 / 21.87 m | 0.83 |
| east std / p2p | 3.899 / 12.98 m | 3.651 / 12.17 m | 0.94 |
| radius from origin, mean / max | 14.90 / 27.53 m | 13.27 / 20.43 m | |

`NIS` at `gnssPosRScale = 8`: north 0.160, east 0.039; at 1.0: 1.263 / 0.306.
Raw error autocorrelation: north 60.3 s, east 56.7 s. Velocity innovations at
rest: `std(NavInnovVelNorth)` 0.110 m/s, `std(NavInnovVelEast)` 0.050 m/s
against `sqrt(R)` = 0.3 m/s; filter speed p95 0.294 m/s vs `GnssGroundSpeed`
p95 0.409 m/s.

### 2.4 For the record

The **0.5 m lift the file is named after is not in it** — `BaroAltitude` moves
0.320 m over the whole 469 s. That finding stands for `lift-0p5m.mf4`;
SYS2-NAV-003's vertical step bullet is evidenced by a later bench round, row
**78EF295E854465E4** (two lifts, +0.448 / +0.574 m), which is therefore also a
replay input for task 1. And `docs/NAV_TUNING.md` section 2's headline ("fused east 3.18x
worse than raw") was measured **before T6**, the PSD reparametrisation now in
the firmware, which is the very mechanism it blamed (velocity correction time
constant 6.05 s -> about 0.76 s). It must be re-measured by replay, not carried
forward.

---

## 3. The four decisions

### 3.1 Vertical channel vs GNSS altitude

**Recommendation: leave the weight alone, and clamp the rate at which the
GNSS-altitude update may move the state.** One new live-tunable field,
`gnssAltSlewMps`, default **0.001 m/s**, taking `Xcp_FusionCal.reserved[0]` at
0x38 — no offset moves, no block grows. `vAcc` is NOT plumbed into the fusion
(SWE1-FW-010). **`gnssAltSlewMps` = 0 is a defined value meaning "GNSS-altitude
update off"** (gate 2): the clamp permits no increment, so alternative (c)
below — deleting the update — is reachable by writing one number over XCP and
revertible by a power cycle. Consequence for the implementation: the 0 must
survive the read. `FusionCal_positive()` substitutes the compiled default for
anything at or below its lower bound, so this field is read directly and
bounded only against NaN and the absurd.

```
/* fusion_correctGnss(), altitude part -- replaces fusion.c:1027-1044 */
zD    = -(gnssAlt - originAlt) + originAltOffset;
R     = (2*hAcc)^2;                       /* unchanged */
y     = zD - x[FS_POS];
if (y*y > max(gateSigmaSq*(P00 + R), gateMinSq))  -> reject, as today
dx    = K * y;                            /* K = P h' / (h P h' + R), as today */
lim   = gnssAltSlewMps * dtFix;           /* dtFix = s since the last fused fix */
scale = (|dx[FS_POS]| > lim) ? (lim / |dx[FS_POS]|) : 1.0;
x    += scale * dx;                       /* slew 0 -> scale 0 -> no change    */
P     = Joseph(P, scale*K, h, R);         /* a scaled gain is sub-optimal ->
                                             Joseph form, or P stops being PSD.
                                             At scale 0 it reduces to P = P    */
```

*Alternatives.* (a) Weight by `vAcc` — rejected, see 2.2: it makes the drift
worse in all six recordings. (b) Inflate `R` by a `gnssAltRScale` the way the
horizontal channel does — works (`R = vAcc^2 * 8` replays to 6.75 m p2p) but
the bound then depends on the receiver reporting honestly, and indoors it does
not; the clamp bounds the worst case unconditionally. (c) Delete the
GNSS-altitude update entirely — replays best of all (0.267 m p2p, i.e. the
barometer alone) and is tempting under "prefer deleting", but it leaves the
barometer reference with nothing to anchor it over a long flight except the
600 s mean reversion toward zero. The clamp gets within 0.15 m of (c) and keeps
the anchor. **Gate 2 resolved this by making (c) a value of the same
parameter:** `gnssAltSlewMps` = 0 is "off", so the choice can be made on the
bench with a cal write instead of in a review, and both options share one code
path and one host test. (d) Decimate the GNSS-altitude update to 1 Hz — a factor of ten on
a problem that is a factor of forty out, and it reshapes the answer's spectrum
rather than bounding it.

*Consequences.* The `d`/`measBias` split now converges at at most 0.06 m/min,
so a 6 m mis-split would take 100 minutes to unwind — acceptable only because
the clamp means the split can never get there (replayed `|measBias|` max
0.243 m over 469 s, against 10.39 m today), and because the split is
re-anchored at origin latch (`s_originAltOffset`) and after every barometer
re-acquisition. `FUSION_MEASB_MAX` (50 m) stays as it is: with the clamp the
split is self-bounded at `gnssAltSlewMps * tauBaroBias` = 0.6 m and the 50 m
net is never approached. Long-term absolute altitude now follows the weather at
up to 0.19 m/min, which is what a barometric altitude hold does and is the
honest trade for a receiver whose vertical error is 36 m peak-to-peak.

*Replayed result* (mechanism replay on the recorded `zD` and barometer; section
5 explains why a 10 Hz replay is legitimate):

| configuration | indoor Up p2p | max per 60 s | outdoor t1 p2p | max per 60 s |
|---|---|---|---|---|
| today | 11.41 m | 4.04 m | 7.42 m | 3.70 m |
| `R = vAcc^2` | 23.39 m | 10.23 m | | |
| `R = vAcc^2 * 8` | 6.75 m | 2.47 m | | |
| **clamp 0.001 m/s** | **0.415 m** | **0.236 m** | **0.310 m** | **0.123 m** |
| clamp 0.002 m/s | 0.632 m | 0.280 m | 0.172 m | 0.108 m |
| clamp 0.0032 m/s | 0.960 m | 0.332 m | | |
| no GNSS-alt update | 0.267 m | 0.193 m | | |

*Why 0.001 m/s and not 0.0032.* `measBias` is the barometer's zero drift: a
mean-reverting random walk, `sigmaBaroRw` = 0.025 m/sqrt(s), `tauBaroBias` =
600 s (`fusion.c:47-72`), steady-state sigma `0.025*sqrt(300)` = 0.433 m. The
anchor must be able to track that — at 0.001 m/s it travels 0.6 m per time
constant, more than one sigma — and must stay a minority contributor to the Up
drift: 0.06 m/min against the barometer's own measured 0.19 m/min. 0.0032 m/s
(the 1-sigma 60 s walk rate) satisfies the first and fails the second.

### 3.2 Horizontal channel vs coloured GNSS error

**Recommendation (REVISED 2026-09-14 after the task-3 sweep): `gnssPosRScale`
8.0 -> 1.0, `sigmaGnssVel` UNCHANGED at 0.30 m/s, and no new velocity gate**
(SWE1-FW-012). One parameter, not two; it already exists and is live-tunable,
so the decision costs no code until it is baked in as a compiled default.

*What the sweep found.* Replayed on t1 with `tools/nav_replay.py`, lowering
`sigmaGnssVel` to 0.12 makes the fused position **worse**, not better: the 2-D
scatter ratio against the raw fix goes 1.65x (today) -> 1.59x (at 0.12) ->
**1.27x (at 0.30)**. The receiver's Doppler error is coloured, and a more
trusted velocity is integrated into position faster than the position update
removes it. `sigmaAccH` is not a lever at all — 8x variation (0.0707 ->
0.0088) moves the t1 east ratio 1.98x -> 2.07x. Raising `sigmaGnssVel` further
does improve t1 (1.0 m/s gives east 1.26x) but wrecks the dynamic runs, where
the Doppler velocity is what carries the motion: t5's north ratio goes 1.16x at
0.30 to 1.80x at 1.0 and 1.95x at 1.5. 0.30 is the only value that is good on
both, and it is the value already compiled in.

**NIS is demoted from a gate to a reported diagnostic.** With a GNSS error
whose 1/e autocorrelation is 32-65 s, `NIS = var(y)/(P+R)` and
never-worse-than-raw are the two ends of one dial: follow the fix and NIS -> 0
with ratio -> 1, smooth and NIS -> 1 with ratio -> 3. No point in a 16-point
`gnssPosRScale` x `sigmaGnssVel` grid put NIS inside 0.5-2.0 with the ratio
under 1.2. That is not a tuning failure, it is `NAV_TUNING.md` section 3's own
conclusion arriving with a number: modelling coloured noise as white gives two
inconsistent halves and you may pick one. The product bar is the ratio.

**The ratio criterion moves from per-channel to 2-D.** Raw t1 north is 1.852 m
against raw east 0.919 m — a 2:1 anisotropy that is mid-latitude satellite
geometry, not the run. The filter's own added error is the integrated GNSS
velocity error, which has no axis preference: at the recommendation the fused
channels come out 1.890 and 1.820 m. A per-channel bound asks the filter to
beat the receiver's best axis with an error that does not know which axis that
is. Measured 2-D ratios, today -> recommended: **t1 1.65 -> 1.27**, t2 0.96 ->
1.00, t4 0.97 -> 1.02, t5 0.99 -> 0.99, indoor 0.82 -> 0.96. Bound:
**<= 1.35x on every run, and no worse than today on any run.**

*Alternatives.* Leave 8.0 — rejected: NIS 0.077-0.160 on every recording means
the filter is 6-13x over-conservative about each fix while its own `P` is 17x
optimistic, and it pays a 21.9 s position correction time constant for it.
Model the GNSS position bias as a fifth mean-reverting state — rejected in
`NAV_TUNING.md` section 4.3 and still rejected: unobservable with one absolute
source, and per-tick work on the 1014 Hz path for information a published floor
already carries. Add a velocity gate — rejected: the largest velocity
innovation in any recording is 1.32 m/s at rest, the existing 5 m/s floor is
never approached, and a tighter one would fire on manoeuvres.

*Consequences.* The GNSS position gate narrows: indoor `hAcc` 5.45 m gives
+-77.1 m -> +-27.4 m; outdoor t1 `hAcc` 2.28 m gives +-32.4 m -> +-11.6 m; on
t2/t4/t5 (`hAcc` about 1.25 m) `5*sqrt(P+R)` = 6.4 m drops below the 10 m
`FUSION_GNSS_GATE_MIN_M` floor, so there nothing changes. t5's 21.9 m east
innovation crosses the floor either way and is reported, not gated — rejecting
it earlier is the gate working. `NavVarNorth` gets **smaller** (0.465 -> about
0.06 m^2 indoors), which reads like a worse answer to anyone treating it as
absolute uncertainty; the sentence `NAV_TUNING.md` section 4.4 asks for belongs
in `FUSION.md` with this change, not after it.

### 3.3 Indoor / multipath guard

**Recommendation: an explicit trust decision in `fusion.c`, on `hAcc` with a
time debounce, and `horizontalOk` stops being a latch** (SWE1-FW-011).

```
trusted := hAcc <= gnssHAccMax          /* live, default 4.0 m, FusionCal 0x3C */
   enter after 30 consecutive fused fixes satisfying it   (3.0 s at 10 Hz)
   leave after 10 consecutive fixes not satisfying it     (1.0 s)
untrusted -> skip the position, velocity AND altitude updates; keep the origin
horizontalOk = originSet AND trusted AND (a fix was fused within 2.0 s)
> 2.0 s untrusted -> hold x[FS_POS], x[FS_VEL] of chN/chE (P keeps growing)
publish gnssTrusted in the FREE Xcp_Fusion byte at 0xBD (reserved3[0])
```

*Why `hAcc` and nothing else — the candidates were tested, not guessed:*

| candidate | indoor | outdoor (all five runs) | verdict |
|---|---|---|---|
| `hAcc` | 4.612 - 6.849 m | 1.135 - 3.347 m | **separates**; midpoint of the gap = 4.0 m |
| `vAcc` | 5.107 - 6.829 m | 1.923 - 5.552 m | overlaps — rejected |
| `numSats` / `fixType` | 10-15 / 3 | 11-16 / 3 | identical — rejected |
| `abs(innov)/hAcc` p95 N/E | 3.30 / 1.18 | up to 3.86 / 7.59 | does not discriminate — rejected |
| `vAcc/hAcc` | 0.996 - 1.212 | 1.243 - 2.218 | separates, one indoor run only — open question |

The `vAcc/hAcc` result has a physical story behind it (indoors only
near-overhead signals survive the roof, so the vertical geometry stops being
the worse one, which never happens under open sky) and it separates across five
outdoor runs on three different dynamics. It rests on **one** indoor recording,
so it is an open question for gate 2, not a gate today.

*Consequences.* The threshold's recorded margin is 0.65 m (outdoor worst 3.347,
indoor best 4.612), which is thin — hence a live-tunable field rather than a
`#define`, and hence the deferred outdoor clause. Indoors the vehicle now has
**no** horizontal estimate at all, declared: `horizontalOk = 0`, position
frozen, `gnssTrusted = 0`. The safety consequence, for the SYS2-SAF chain and
for `src/asw/flight_ctrl`: position hold must not arm on `horizontalOk = 0` and
must fall back to altitude hold if it is already active. That fallback is NOT
in this strand — this strand makes the flag tell the truth so the fallback has
something honest to trigger on. The vertical channel is unaffected in kind: it
runs on the barometer, which is what it was effectively doing anyway.
`GnssNavOk` keeps its present meaning ("the receiver has a usable 3-D fix") for
diagnostics and the GUI; `GNSS_HACC_USABLE_MM` (10000, `GnssM9N.c:110`) is not
touched. The receiver reports, the estimator decides — that is why the gate
lives in `fusion.c` and not in the driver.

### 3.4 Verification plan

**Recommendation: build the MF4 front end for the harness that already exists,
and move every decidable criterion offline** (SWE1-FW-013).

`test/gen_fusion_trace.c` already drives the **production** `fusion.c` from a
command stream through the public `fusion.h` interface, and
`test/ref/replay_hw.py` already replays canned hardware data through it
(`test/CMakeLists.txt:299-321`). Missing: `tools/nav_replay.py`, an MF4 ->
command-stream front end with `--cal name=value` and `--baseline`.

*What the replay can decide:* covariance, gains, NIS, the drift of `posD` and
`baroBias`, all gating and trust logic, every counter. Legitimate at the
recordings' 10 Hz because T6 made Q a PSD whose integral is rate-invariant
(`fusion.c:435-445`) — the same covariance growth per wall-clock second at any
tick rate. *What it cannot:* anything dominated by the acceleration input
between fixes. `AttAccNed` is logged at 10 Hz, so a replayed dynamic run is
aliased, and the walking/velocity-overshoot criteria of `NAV_TUNING.md` section
5 plus the SYS2-NAV-003 step test remain hardware evidence, deferred to the
tether trip. *Missing inputs:* the barometer reference `refM` is not logged
(latch it from the first sample — shifts absolute altitude, leaves every
relative criterion untouched); `sAcc` was never decoded.

*Alternatives.* Mirror the estimator in Simulink and verify there
(SYS2-VER-001) — the right long-term answer and not a substitute: it verifies
the model against the C, not the C against the receiver. Sweep on the board
over XCP first — that is what `NAV_TUNING.md` T1-T3 planned and it never
happened for lack of a clear sky; offline first inverts the dependency.

---

## 4. Target architecture and interfaces

Nothing moves between modules. The BSW/ASW split, the CPU allocation and the
latch protocol are untouched: `Fusion_setGnss()` still runs on CPU0 in
`SensorTask_gnss` and writes `g_gnssLatch`; `Fusion_update()` still runs on
CPU1 inside `NavTask_step`; the cross-core crossing stays the
generation-counter snapshot at `fusion.c:748-807`. The three changes are all
inside `fusion_correctGnss()` and its file-static state.

**`src/bsw/FusionCal.h` / `.c`** — two fields out of `reserved[2]`, no offset
moves, the block stays 64 bytes and is then **full**:

```c
    float32 sigmaAccRw;          /* 0x34, unchanged                            */
    float32 gnssAltSlewMps;      /* 0x38  was reserved[0]. Max rate the GNSS
                                    altitude update may move the DOWN position
                                    state [m/s]. 0.001 = 0.06 m/min, one third
                                    of the barometer's own measured wander.
                                    ZERO IS A DEFINED VALUE: it switches the
                                    GNSS-altitude update off entirely. Read it
                                    directly, NOT through FusionCal_positive(),
                                    which would substitute the default for 0.  */
    float32 gnssHAccMax;         /* 0x3C  was reserved[1]. GNSS is trusted only
                                    while the reported hAcc is at or below
                                    this [m]. 4.0 separates every recorded
                                    outdoor fix from every recorded indoor one */
```

**`src/bsw/fusion.h`** — `FusionValues` gains one `uint8 gnssTrusted;`, placed
so it consumes existing padding; the "1 Hz" comment at `fusion.h:12` is
corrected to 10 Hz (same for `GnssM9N.c:790`).

**`src/bsw/Measurements.h`** — `gnssTrusted` is published into the **existing
free byte** `reserved3[0]` at `Xcp_Fusion` offset **0xBD**. The block is at 252
of 256 bytes; nothing may be appended for this strand, and nothing needs to be.
`tools/a2l_meta.json` gains one entry, `tools/gen_a2l.py` is re-run, and
`LayoutAssert_gen.h` plus `tools/check_memmap.py` must show every existing
offset unchanged (verified against `Measurements.src`, never host `offsetof`).

**`src/bsw/fusion.c`** — new file statics: `s_gnssTrusted`, `s_trustRun` /
`s_untrustRun` (saturating `uint8`), `s_lastFixMs`. New helper
`fusion_chanUpdateSlewed()`: the existing `fusion_chanUpdate()` with a
`float32 maxStep` argument and the Joseph covariance form; the unslewed call
sites pass a negative `maxStep` meaning "no limit" (MISRA prefers not to pass
infinities around).

**Testable on the host, without hardware:** the slew clamp as a bound for every
input sequence; the trust debounce against a chattering `hAcc`; the freeze
timer; the Joseph form staying positive-definite under a scaled gain; and —
new, and the point of SWE1-FW-013 — the whole filter against a real recording.

## 5. Timing

Nothing periodic changes. `Fusion_update()` keeps running at IMU rate
(~1014 Hz, `FusionDt` mean 0.000985 s) inside `NavTask_step` on CPU1, budget
**300 µs** (SYS2-TIM-002, measured 170-180 µs).

| path | rate | core | added worst case | on overrun |
|---|---|---|---|---|
| `fusion_chanPredict` x3 | 1014 Hz | CPU1 | none — not touched | n/a |
| `fusion_correctBaro` | ~100 Hz | CPU1 | none — not touched | n/a |
| `fusion_correctGnss` | 10 Hz | CPU1 | one divide, one compare, one multiply per channel update (slew); two `uint8` counters and one branch (trust); the Joseph form costs one extra 4x4 multiply-add on the three GNSS updates | the scheduler is cooperative — an overrun steals from whatever follows it on CPU1. The added work is bounded, straight-line apart from the trust test, and lands on 1 tick in 100 |
| freeze branch | 1014 Hz | CPU1 | one compare against a tick count | n/a |
| `fusion_updateStationaryLock()` | **1014 Hz** | CPU1 | **5 `FusionCal_positive()` calls + 1 `sqrtf` + two counters**, every tick. This is the one addition of the whole strand that is NOT on a slow path — the detector has to see every sample, because a 1 s window of "every sample under threshold" is what it asserts | the scheduler is cooperative; an overrun steals from everything after it on CPU1. Straight-line, no loop, no branch that depends on data length |
| `fusion_decayGnssBias()` | **1014 Hz** | CPU1 | **2 `FusionCal_positive()` calls + 2 divides**, every tick | as above |
| `Fusion_setGnss` | 10 Hz | CPU0 | none — signature unchanged, `vAcc` is not plumbed | n/a |

**Estimated added dispatch** *(corrected 2026-09-14 after review round 1 — the
earlier line said "zero on 99 ticks in 100", which was true of SWE1-FW-010/-011
and is no longer true of the strand)*: single-digit microseconds on the 10 Hz
tick as before, **plus of order 2-4 µs on every 1014 Hz tick** from the two new
per-tick functions — single-digit percent of the 300 µs budget
(SYS2-TIM-002, measured user 170-180 µs). The divides and the `sqrtf` are the
cost; `FusionCal_positive()` is a compare and a select.

Two things follow, and both are requirements rather than observations.
**First, the estimate is not the acceptance.** `g_dbgNavStepMaxTicks` read back
over XCP is, and it must stay under 300 µs over a run of at least 300 s
containing at least one lock and one release — SWE1-FW-014 clause (c2), which
is a **bench (PIL)** clause for the obvious reason that a host build has no STM
and cannot measure a TriCore dispatch. **Second, if the read-back does move**,
the cheap fix is available before any redesign: the detector needs the
per-sample test but not the per-sample *parameter* read, so the five
`FusionCal_positive()` calls can be hoisted to the rate at which a cal write
can plausibly matter, and `sqrtf` can be removed by comparing squared norms
against a squared threshold. Neither changes behaviour.

## 6. Risks

- **A2L/GUI contract.** One new published byte at an offset that is already
  reserved, and two `Xcp_FusionCal` fields that were already reserved. No
  existing offset moves and no block grows. This is the cheapest possible shape
  of the change, and it is only possible because `Xcp_Fusion` has exactly 4
  spare bytes and 3 reserved ones left — **the next fusion signal after this
  one needs the next 256-byte slot**, and that is a bigger change than anything
  in this strand.
- **`Xcp_FusionCal.reserved[2]` is consumed here.** After this the calibration
  block is full at 64 bytes. That sentence belongs in `FusionCal.h`.
- **The 4.0 m trust threshold has 0.65 m of recorded margin** and is fitted to
  one indoor and five outdoor recordings from two locations. It can refuse a
  legitimate fix under trees or at cold start. Mitigated by making it live, by
  the 1 s leave-debounce and by the deferred outdoor clause — not eliminated.
- **`NavHorizontalOk` has CHANGED MEANING, and that is a contract change even
  though no byte moved.** It was a latch — "the tangent-plane origin has been
  set at least once", set at `fusion.c:560` and never cleared. It is now
  "the origin is set AND the GNSS is trusted AND a fix was fused within 2.0 s"
  (SWE1-FW-011). Same offset, same type, different question answered. Checked
  against the consumer: the GUI's anchoring display ANDs it with `GnssNavOk`
  (SWE1-GUI-007), so indoors it will now correctly show **dead reckoning**
  instead of claiming an anchored position that is 15 m out — **no GUI change
  is needed and none should be made**. The item text of SYS2-GUI-003 mentions
  the flag, so a one-line status note there is right; the system architect is
  adding it. Anything else reading `NavHorizontalOk` as "have I ever had a fix"
  would now be wrong, and there is nothing else today.
- **On the bench this will look like a regression until it is understood.**
  Indoors the position panel will say dead reckoning and the position will not
  move. That is the correct answer to a 14.90 m receiver bias.
- **A scaled Kalman gain is sub-optimal by construction.** With the standard
  `(I-KH)P` form a scaled gain can drive `P` non-PSD; the Joseph form is not
  optional here. The numerical health check (`fusion.c:455-470`) is the net,
  and it must stay at 0 in every acceptance run — a non-zero `NavCovResets`
  after this change is a defect *in* this change.
- **Position-hold behaviour indoors changes from "wrong" to "declared
  unavailable".** That is the intended fail-operational shape, but the ASW side
  of it (fall back to altitude hold) is not in this strand and must not be
  assumed to exist.
- **Irreversible:** tasks 6, 7 and 9 need a reflash to undo, and a reflash
  wipes DFLASH — magnetometer calibration restore plus read-back after every
  one. Everything in tasks 4 and 5 is an XCP write into a RAM-only block and
  reverts on a power cycle.
- **What still needs hardware:** every on-board confirmation row and the whole
  deferred outdoor set. The replay decides the design; the board confirms the
  firmware does what the replay said.
- **No pin change, no newly driven pin, no power change, no actuation, no
  DFLASH write outside the Nvm ping-pong, and no code that arms or drives
  motors** is involved anywhere in this strand. `docs/PINNING.md` is untouched.

---

## 7. Ordered tasks

Each task builds and flies on its own. Type: **(a)** XCP write of an existing
`Xcp_FusionCal` field — needs gate 2 and board time, no flash; **(b)** firmware
change with a flash — magnetometer calibration restore and read-back
afterwards; **(c)** docs or host tooling only, no board. Sequenced so the
offline evidence exists before anything is written to the board, and the
compiled defaults land last. **No task in this table touches a pin, a newly
driven pin, power, actuation, or code that arms motors.**

| # | files | change | acceptance | parent | type |
|---|---|---|---|---|---|
| 1 | `tools/nav_replay.py` (new), `test/CMakeLists.txt`, `test/data/` | MF4 into `gen_fusion_trace` front end; `--cal`, `--baseline`; metric table (p2p and max-per-60 s of `-posD` and of the baro bias, NIS per channel, counters). No `src/` change | fidelity: the unmodified filter replaying `02F0B754CCA975C6` reproduces logged `NavPosDown` and `NavBaroBias` within **0.5 m RMS** and the 9.34 m p2p within 20 %; two runs byte-identical; new ctest `nav_replay` green on a committed excerpt; firmware binary bit-identical | SWE1-FW-013 | (c) |
| 2 | none (`tools/nav_replay.py`) | baseline run of all six recordings, unmodified filter; numbers into `QuadSE/evidence/INDEX.md` as replay rows | the table of section 2 reproduces to 5 % | SWE1-FW-013 | (c) |
| 3 | none (`tools/nav_replay.py --cal`) | offline sweep: `gnssPosRScale` in {8, 4, 2, 1} and `sigmaGnssVel` in {0.3, 0.2, 0.12, 0.08}, on t1 and the indoor run | **DONE 2026-09-14** (`docs/nav_replay_baseline_2026-09-14.md`), and it changed the recommendation — see 3.2. Result: `gnssPosRScale` = 1.0 with `sigmaGnssVel` unchanged at 0.30; 2-D ratio t1 1.65 -> 1.27, all other runs <= 1.02; `NavGnssRejects`/`NavCovResets` 0 everywhere in the grid | SWE1-FW-012 | (c) |
| 3b | none (bench recording) | a from-boot indoor recording: GUI connected before power-on, logging started right after reconnect, >= 400 s at rest. Read-only — no XCP write, no flash | `NavGnssUpdates[0] < 50` and `NavVarDown[0] > 0.5` (it really is from boot); cold replay then reproduces the logged `NavPosDown` **and** `NavBaroBias` individually within 0.5 m RMS and the logged p2p within 20 % (SWE1-FW-013 b2) | SWE1-FW-013 | (a) board time, no write |
| 4 | none (XCP write) | `gnssPosRScale` 8.0 -> 1.0 on the board, alone | block read-back matches; 300 s indoors with `NavCovResets` = 0, `NavDropped` = 0, `g_dbgNavStepMaxTicks` < 300 µs. Indoors this can only show that nothing broke | SWE1-FW-012 | (a) |
| ~~5~~ | — | ~~`sigmaGnssVel` sweep on the board~~ | **DROPPED 2026-09-14:** the task-3 sweep kept `sigmaGnssVel` at its compiled 0.30, so there is nothing to write | SWE1-FW-012 | — |
| 6 | `fusion.c` (`fusion_chanUpdate` gains `maxStep` + Joseph form; altitude call site), `FusionCal.h/.c` (`reserved[0]` -> `gnssAltSlewMps` at 0x38, read directly, not via `FusionCal_positive()`), `test/test_fusion.c`, `tools/a2l_meta.json`, regenerated A2L, `Version.h` | the slew clamp of 3.1, default 0.001 m/s, **0 = off** | host: a 20 m offset held 600 s keeps `abs(d(t)-d(0)) <= slew*t*1.01` at every step; a 0.05 m offset is bit-identical to the pre-change build; **at `gnssAltSlewMps` = 0, `d`, `measBias` and all of `P` are bit-identical to a run in which the GNSS altitude is never offered, and the 0 is not replaced by the compiled default**; `P` stays PSD over 1e6 steps. Replay, binding clause is the RATE (the clamp bounds a rate; arithmetic worst case over 60 s is `2*slew*60` = 0.12 m): indoor and t1 **max-per-60 s <= 0.25 m**, against measured cold-replay baselines of **4.551 m (indoor)** and **3.572 m (t1)**. Reported, expected: indoor Up p2p <= 0.60 m (baseline 12.635 m), baro bias <= 0.50 m (baseline 12.610 m), t1 p2p <= 0.50 m (baseline 4.348 m). NIS baro stays 0.5-2.0 and `std(NavVelDown)` <= 0.05 m/s. MISRA clean, `gen_a2l.py --check` 0, every `Xcp_FusionCal` offset unchanged in `FusionCal.src` | SWE1-FW-010 | (b) |
| 7 | `fusion.c` (trust state, debounce, freeze, `horizontalOk`), `fusion.h`, `FusionCal.h/.c` (`reserved[1]` -> `gnssHAccMax` at 0x3C), `Measurements.h/.c` (`reserved3[0]` at 0xBD), `test/test_fusion.c`, `tools/a2l_meta.json`, regenerated A2L, `Version.h` | the trust gate of 3.3 | host: a chattering `hAcc` never flips trust; a 1.5 s outage keeps `horizontalOk` = 1 and the channels predicting; a 5.0 s outage clears it and freezes both; re-entry after exactly 30 fixes, origin never reset. Replay: `gnssTrusted` = 0 for >= 99 % of the indoor run and = 1 for 100 % of t1/t2/t4/t5 after entry; indoor `NavPosNorth/East` frozen, max delta <= 0.05 m. `Measurements.src` shows every `Xcp_Fusion` offset unchanged and the block still 252 bytes; `check_memmap.py` unchanged; MISRA clean | SWE1-FW-011 | (b) |
| 8 | none (bench) | flash tasks 6+7, restore calibration (`python tools/mag_cal.py --restore-from calibration/board.json`) and read back, then a >= 400 s at-rest indoor recording | on-board: `-NavPosDown` p2p <= 0.60 m and max-per-60 s <= 0.25 m; `GnssNavOk` = 1 with `NavGnssTrusted` = 0 and `NavHorizontalOk` = 0; `NavGnssUpdates` constant; `NavCovResets` = `NavDropped` = 0; `g_dbgNavStepMaxTicks` < 300 µs; version read-back matches. Evidence row | SWE1-FW-010, -011 | (b) |
| 9 | `FusionCal.c` (`FCAL_GNSS_POS_R_SCALE` only — `FCAL_SIGMA_GNSS_VEL` is unchanged), `docs/FUSION.md` section 2 | bake the task 3/4 winner in as the compiled default | builds; XCP read-back of `g_fusionCal` matches the new defaults after a power cycle; MISRA clean | SWE1-FW-012 | (b) |
| 10 | `docs/FUSION.md` sections 2 and 7, `docs/NAV_TUNING.md` (amendment box), `fusion.h:12`, `GnssM9N.c:790`, `FusionCal.h` | the `NavVarNorth`-is-relative sentence (NAV_TUNING 4.4); the note that section 2's 3.18x is pre-T6; "1 Hz" -> 10 Hz in both comments; the note that the cal block is now full | `python tools/check_docs.py` green | SWE1-FW-012, SYS2-NAV-003 | (c) |
| 11 (optional, low) | `GnssM9N.c` (decode `sAcc`, NAV-PVT offset 68), `FusionLatch.h`, `fusion.h`, `fusion.c` | make `sigmaGnssVel` measured instead of chosen | replay shows the velocity NIS in 0.5-2.0 with `R = sAcc^2` on t1; no offset moves | SWE1-FW-012 | (b) |
| 12 (deferred) | none (outdoor, Chris) | re-fly t1/t2/t5 plus the SYS2-NAV-003 step sequence | `NAV_TUNING.md` section 5 walking criteria; 0.5 m step within 0.20 m; out-and-back within 0.20 m; 5-min stationary radius recorded; `gnssHAccMax` = 4.0 refuses no legitimate fix | SWE1-FW-012, SYS2-NAV-003 | tether |

Tasks 1-3 need no board at all and produce every number the design is judged
on. Task 12 is the only thing that has to wait for weather.

---

## 8. What the first flights actually need (Chris, 2026-09-14)

The receiver decision is settled: **the NEO-M9N stays for the first test
flights**; optical flow and RTK are revision-2 topics. This strand adds no
sensor input. One forward-compatibility note, and no code for it: the
horizontal velocity update in `fusion_correctGnss()` already takes a NED
velocity pair through the generic `fusion_chanUpdate(ch, hVel, z, R, ...)`
path, and the GNSS-specific part is only the `speed`/`heading` to `vN`/`vE`
resolution immediately above it. Keep that split — a body-frame velocity
measurement (optical flow, rotated into NED by the AHRS yaw) would then enter
as a second caller of the same update with its own `R`, not as a restructure.
Do not generalise anything today for it.

That fixes what the acceptance numbers are FOR, and both halves are in the
SWE1 items in exactly these terms:

- **(a) A valid, stable z on the tether, indoors.** The tether flights come
  right after attitude commissioning and will happen where the board is —
  indoors, with a through-the-roof fix. `02F0B754CCA975C6` is the failing case
  to fix, and it is the primary acceptance recording of SWE1-FW-010: the
  barometer keeps short-term authority and GNSS altitude may not pull z.
  Numerically, at rest indoors: `-NavPosDown` peak-to-peak **<= 0.60 m** over
  >= 400 s and **<= 0.25 m over any 60 s window**, against 9.34 m and 3.50 m as
  recorded. That is altitude-hold-grade — an altitude loop cannot be commanded
  to hold something that walks metres under it.
- **(b) x/y as good as the raw M9N fix, never worse — outdoors, honestly
  metre-class.** That is the headline criterion of SWE1-FW-012, and it is a
  *ratio* against the raw fix on the same recording, not an absolute: `std` of
  the fused north and east each **<= 1.0 x** the raw receiver's own `std` on
  the same tangent plane (t1: raw 1.852 m north, 0.918 m east), with 1.2 x the
  tolerance band. Absolute accuracy stays metre-class and is not a target — the
  M9N cannot do better and SYS2-NAV-003 already records that. Indoors x/y is
  not improved at all: it is **refused** (SWE1-FW-011), because a 14.90 m raw
  bias is not a filter problem.

The two are deliberately asymmetric. z must be *good* for the first flights; x/y
must merely be *honest*, and honest includes saying "unavailable".

---

## 9. Gate-2 decisions (Chris, 2026-09-14) and what is still open

**Decided.** (1) `gnssAltSlewMps` = **0.001 m/s**. (2) Keep the clamp, and
define **`gnssAltSlewMps` = 0 as "GNSS-altitude update off"** — deletion is a
calibration value, not a code change. (3) The `vAcc/hAcc` criterion is
**deferred**: two more indoor recordings during task 8, decide then. (4) The
`horizontalOk = 0` fallback is a **separate SYS2-SAF / SYS2-CTRL-002 item**,
not this strand. (5) Both proposed amendments were applied by the system
architect — `SYS2-NAV-001` gained the measurable 0.25 m / 60 s clause with the
0.06 m/min and slew-0 wording and the "an untrusted fix contributes nothing
vertically" sentence; `SYS2-NAV-003` gained the clarification that the
stationary-wander bullet is only meaningful while `NavGnssTrusted` = 1, plus
the correction that its **vertical bullet IS evidenced** by row
78EF295E854465E4 (two lifts, +0.448 / +0.574 m). SWE1-FW-010..013 were
re-derived against the amended parents on the same day and their
`stale-parent` flags removed.

**Two findings from tasks 1-3 (flight-dev, branch `feat/nav-filter-strand` @
9a5bd85, `docs/nav_replay_baseline_2026-09-14.md`) — decided 2026-09-14, and
one of them needs Chris.**

- **Finding 1, fidelity: accepted as flight-dev framed it, plus one extra
  recording.** `NavPosDown` and `NavBaroBias` individually cannot be reproduced
  from `02F0B754CCA975C6` by a cold replay — the file starts at
  `NavGnssUpdates` = 6797 with `NavVarDown[0]` = 0.0648 against a
  `FUSION_P_POS_INIT` of 1.0, so the board's `d`/`measBias` split at t = 0 is
  not in the file, and only the SUM is barometer-observable. SWE1-FW-013 (b) is
  re-derived onto the sum (0.20 m RMS budget; measured 0.058 m) plus the
  structural counters, and a new clause **(b2)** puts the literal per-state
  criterion on a **from-boot recording** — new task 3b, board time, read-only,
  no XCP write, no flash. **This does not affect task 6.** The clamp is judged
  on the replay's own trajectory from a known cold start (split = 0), not on
  agreement with a logged trajectory that began from an unknown split; a cold
  start is in fact the honest stimulus, since the question is exactly whether
  GNSS can walk the split away. The one consequence is that task 6's
  "baseline must FAIL" numbers are now quoted against the **replayed** baseline
  (indoor 12.635 m p2p / 4.551 m per 60 s, t1 4.348 / 3.572), not the logged
  9.34 m, and SWE1-FW-010 carries that correction.
- **Finding 2, the sweep: the recommendation changed — this needs Chris.**
  Section 3.2 above is rewritten with the measured numbers. Three things that
  gate 2 approved are now different: `sigmaGnssVel` stays at **0.30** (the
  0.12 proposal is withdrawn, and board task 5 is dropped with it); the
  **NIS 0.5-2.0 band is demoted from a gate to a reported diagnostic** with a
  wide 0.02-5.0 sanity band, because no setting satisfies it together with the
  ratio; and the never-worse-than-raw criterion moves from **per channel** to
  **2-D**, at <= 1.35x and no worse than today on any run. What does NOT change
  is the product bar itself or the one change that matters: `gnssPosRScale`
  8 -> 1, which takes the t1 2-D ratio from 1.65x to 1.27x and improves or
  holds every other run. Stated plainly for the gate: **a fused 2-D scatter of
  1.0x raw is not reachable.** The only settings that approach it declare the
  receiver's Doppler velocity 5-30x worse than its datasheet
  (`sigmaGnssVel` >= 1.0 m/s), which costs the dynamic runs (t5 north ratio
  1.16x -> 1.80x) and removes the measurement SYS2-NAV-003's 0.5 m step
  depends on. Recommendation: accept 1.35x 2-D; reject the alternative.

**The four gate-2 questions as they were asked, with their answers.**

1. *(answered: 0.001)* **`gnssAltSlewMps` = 0.001 m/s or 0.002 m/s?** 0.001 gives 0.06 m/min of
   GNSS-driven Up drift (a third of the barometer's own) and tracks the baro
   bias at 0.6 m per mean-reversion time constant. 0.002 doubles the tracking
   authority for 0.12 m/min. Both pass the acceptance; the recommendation is
   the quieter one, because the barometer is the good sensor here.
2. *(deferred to task 8)* **Is the `vAcc/hAcc` ratio worth a second trust criterion?** It separates
   cleanly (indoor <= 1.212, outdoor >= 1.243) with a physical explanation, but
   on one indoor recording. Two more indoor recordings at different times of day
   would settle it without going outside. If it holds it is the better gate,
   because it is scale-free. Recommendation: record them during task 8 and
   decide then; do not gate on it now.
3. *(answered: it became a value of the same parameter)* **Should the GNSS-altitude update be deleted instead of clamped?** It
   replays best (0.267 m p2p versus 0.415 m) and removes a mechanism entirely.
   The recommendation keeps it because the barometer reference otherwise has no
   anchor over a long flight. If altitude hold is only ever relative to a
   setpoint captured at arming — which is what the first tether flights will do
   — deleting it is the simpler system. This is a product question, not a
   filter question.
4. *(answered: a separate item)* **Who owns the position-hold fallback?** `horizontalOk = 0` needs a
   consumer. Proposed as a separate SYS2-SAF / SYS2-CTRL-002 item, not folded
   into this strand.
5. *(applied 2026-09-14)* **The two proposed amendments to parent items**, as
   filed — kept as the record of what was asked for:
   - **SYS2-NAV-001**, new measurable clause: *"With the vehicle at rest and a
     valid barometer, the fused altitude `-NavPosDown` shall not drift by more
     than 0.25 m over any 60 s window, whatever the GNSS altitude does. The
     barometer holds short-term altitude; GNSS altitude may correct the
     barometer's bias at no more than 0.06 m/min."*
   - **SYS2-NAV-003**, factual correction: the recording named
     `lift-0p5m.mf4` contains no lift (`BaroAltitude` peak-to-peak 0.320 m over
     469 s), so the vertical 0.5 m bullet is **not** yet evidenced; and the
     stationary-wander bullet is only meaningful while `NavGnssTrusted` = 1,
     which indoors it will not be.

---

## 10. Stationary lock (ordered by Chris, 2026-09-14)

Chris watched the live position view, asked why the fusion drifts at all when
the IMU shows no motion, and rejected the fusion as it stands. He is right, and
the answer is not more tuning: **a filter that knows the vehicle is standing
still should not be integrating anything.** Items SWE1-FW-014 (the lock) and
SWE1-FW-015 (the release).

**Framing: indoors is the bench, never a flight case.** Every criterion that
decides the design is stated on an outdoor or a rest recording; the indoor
recordings are the worst-case stress input, not the target.

### 10.1 Detector

`|omega|` from `AttRate[0..2]` and `abs(AttAccMagnitude - 1)`, both already
published and both bias-corrected by the AHRS. Lock when both stay under their
thresholds for every sample of a 1.0 s window; release when either exceeds its
release threshold on **two consecutive** samples. The two-sample release is not
a refinement — a single corrupt IMU word is a measured failure mode of this
board (SWE1-FW-009, +1999.76 deg/s on one tick) and must not be able to release
the lock.

Thresholds from the 1-second-window maxima on file: at rest (five recordings,
including outdoor t1) the worst clean window is **1.242 deg/s** and
**0.0169 g**; the quietest motion on file — a 0.45 m hand lift, peak vertical
rate 0.357 m/s, about 0.12 g — is 7x above that; walking runs sit at 16-19
deg/s median. Defaults: `lockGyroDps` **2.0**, `lockAccG` **0.03 g**,
`relGyroDps` **3.0**, `relAccG` **0.05 g**, `lockWindowS` **1.0 s**. Full table
in SWE1-FW-014.

**The lock also requires an explicit `Fusion_setOnGround(TRUE)` from the ASW,
and that is the load-bearing part.** A stabilised hover in still air can sit
under 2 deg/s and 0.03 g for a second; a lock engaging in flight pins the
velocity to zero and freezes the position under a position controller that then
flies away blind. That hazard is made impossible by an interlock, not
improbable by a vibration argument. Default TRUE at boot, FALSE from arming to
confirmed touchdown. BSW/ASW split intact: the ASW calls a BSW setter.

### 10.2 While locked

Zero-velocity update as an ordinary measurement (`h = [0,1,0,0]`, `z = 0`,
`R = sigmaZupt^2`, `sigmaZupt` = 0.01 m/s), **decimated to the barometer rate,
never on the 1014 Hz path**. A hard `x[FS_VEL] = 0` is rejected: it leaves `P`
claiming the old uncertainty — the same incoherence `fusion.c`'s barometer
re-acquisition comment documents — and it teaches the bias states nothing.
GNSS position, velocity and altitude updates are all skipped; the barometer
keeps running, so the `d`/`measBias` split is untouched by GNSS exactly as
SYS2-NAV-001 requires.

**The payoff is the accelerometer bias.** With the velocity pinned, residual
specific force is attributed to the bias through `P[vel][accB]`. Expected after
60 s locked: bias sigma of order 1e-3 m/s^2, floored by `sigmaAccRw`
(7.7e-4 over 60 s), against today's 0.018 m/s^2 lump — about 20x. The open-loop
horizontal drift quoted in `fusion.h`, 32 m per 60 s, becomes about **1.8 m per
60 s**. That is what the vehicle carries into the first seconds after liftoff.

### 10.3 Release, and why the offset decays over 60 s

At release the frozen position and the current fix differ: measured on t1, the
raw fix walks a **2.50 m median / 5.99 m max radius over 120 s**. The
difference is installed as an explicit GNSS position bias, subtracted from
every later fix and decayed with `tauGnssBiasS` = **60 s**, rate-limited to
**0.05 m/s**, clamped at 10 m. The innovation at the release tick is exactly
zero, so the state is continuous by construction and **the liftoff setpoint is
the frozen point**. 60 s is the measured 1/e autocorrelation of the raw
horizontal error (north 60.3 s, east 56.7 s indoors; 32.4 s outdoor per
`NAV_TUNING.md` section 3): faster injects the bias back as a ramp before it
has decorrelated, much slower holds a stale offset after the receiver has moved
on. The two defaults agree with each other — a 3 m offset over 60 s is
0.05 m/s — so the rate limit binds only above about 3 m.

Alternatives: accept the jump (rejected — a metre-class step into the position
controller at the least stable moment of the flight, indistinguishable at the
controller input from a commanded move); re-origin at arming (rejected — it
discontinues the origin, every logged position, the GUI trail and the frozen
reference, and the GNSS error after the re-origin is the same realisation);
freeze velocity only (rejected — the position then keeps random-walking, which
is the defect).

**How the release is verified, and a criterion that was withdrawn**
*(2026-09-14, after SWE1-FW-015 (d) was executed for the first time)*. The
acceptance is on **the bias state**, not on the distance between the fused
position and the raw fix: `|gnssBias|` must never increase after release, must
be at `0.368 * |gnssBias|(release)` within 10 % at one time constant and under
10 % of it at three, and the position rate the decay implies must never exceed
`gnssBiasRateMax`. The original formulation — "converge to within 0.5 m of the
raw fix within 180 s" — is withdrawn, not relaxed, and the reason is worth
keeping because it is a general trap: **it measured the mechanism through an
instrument whose own noise is four times the threshold.** On t1 the raw fix has
a 2-D scatter of 2.07 m and, inside the observation window itself, walks about
6.5 m out and back; the executed run read 0.087 m at +10 s and then 0.77-4.21 m
as the *receiver* moved. No tuning of `tauGnssBiasS` could have passed it, and
retuning against it would have been fitting the time constant to one
realisation of the weather. The run could not even host the observation:
released at 150 s in a 300.9 s recording, the +150 s and +180 s rows of the
convergence trace are the same final sample. The mechanism was fine — the fused
state landed on the fix and tracked it in direction rather than sticking. A
criterion has to be resolvable by the measurement that checks it, and the
fused-versus-raw distance belongs to SWE1-FW-012's scatter ratio (1.27x raw on
t1), where it is already gated once.

### 10.4 In-flight IMU-consistency gate: assessed, NOT recommended

Rejecting a GNSS innovation the integrated IMU acceleration did not corroborate
is rejected. The largest innovation on file is t5's 21.9 m east excursion, and
`NAV_TUNING.md` section 6 established that there the **filter** was wrong
(`NavVelEast` -5.1 m/s against a reported 1.7 m/s of ground speed) and the GNSS
was right — the gate would have rejected the correct measurement and delayed a
recovery the existing escape hatch completed in 0.5 s. The error that actually
hurts, the slow multipath bias (14.90 m mean radius indoors), is invisible to
any innovation gate; SWE1-FW-011's trust gate answers it. `NavGnssRejects`
stayed 0 on t1/t2/t4 across the whole 16-point sweep, so the existing 5-sigma
gate with its 10 m floor is not straining. A second gate needs its own escape
hatch and adds a failure mode — rejecting good GNSS after an IMU fault — on the
critical chain, against no recorded defect. **Drop it.**

### 10.5 Memory, and why nothing moves

`stationaryLocked` takes the free `Xcp_Fusion` byte `reserved3[1]` at **0xBE**
(`gnssTrusted` has `[0]` at 0xBD, `[2]` at 0xBF stays free) — the block stays at
252 of 256 bytes. `Xcp_FusionCal` is full at 64 bytes after SWE1-FW-010/-011,
so the new tuning fields are appended **inside its existing 256-byte slot**:
`XCP_FUSIONCAL_SIZE` 64 -> 128, new fields from **0x40** upward
(`lockGyroDps`, `lockAccG`, `relGyroDps`, `relAccG`, `lockWindowS`,
`sigmaZupt`, `tauGnssBiasS`, `gnssBiasRateMax`; `gnssBiasMaxM` may stay a
`#define`). No existing offset moves and no new block is created.
`docs/CODEMAP.md`'s "take the next free slot" rule applies when a block would
overrun its slot — this one does not. Checks to run: `XCP_FUSIONCAL_SIZE`,
`Xcp.c`'s whitelist length, `Lcf_Tasking_Tricore_Tc.lsl`
(`LCF_XCP_FUSIONCAL_START` is a start, not a length — confirm), the A2L
regeneration and `LayoutAssert_gen.h`.

### 10.6 What the estimator can and cannot promise for Chris's flight targets

Framing only — these are controller targets, written down here so nobody
discovers the second one after the first flight.

| target | estimator verdict | number |
|---|---|---|
| 1 m stick push in hover, return within 0.3 m | **probably yes, marginal, unproven** | the raw fix's own 10 s wander radius is 0.56 m median / 1.48 m p95, so the return cannot rest on GNSS position — it rests on the Doppler velocity (0.05 m/s spec, 0.5 m open loop over 10 s) plus the 20x better accel bias the lock leaves behind. Expect 0.3 m on a short push (round trip under 5 s), marginal at 10-20 s |
| stationary hover wander <= 1 m over 2 min outdoors | **NO, not with the M9N** | the RAW fix walks a **2.50 m median / 5.99 m max** radius over sliding 120 s windows on t1. That is the receiver, not the filter, and nothing in this strand changes it. Needs optical flow or RTK — revision 2 |
| 2 m pattern (up 2 m, 2 m sides, diagonals, up to 5 m, land) | **vertical yes; horizontal relative yes; absolute metre-class** | vertical is barometric, 0.020 m scatter, and after SWE1-FW-010 it no longer walks. Horizontal moves of 2 m over a few seconds are the regime the M9N is good in — the same regime as SYS2-NAV-003's 0.5 m step |

Confirmed at the first outdoor session. **The hover-wander line should go to
Chris now, not then.**

### 10.7 Task rows (appended to section 7)

| # | files | change | acceptance | parent | type |
|---|---|---|---|---|---|
| 13 | `tools/nav_replay.py` (detector metrics), `test/ref/` | compute and print the detector inputs (`|omega|`, `abs(|a|-1g)`, their 1 s-window maxima) and the would-be lock/release timeline for any recording. No `src/` change | reproduces the SWE1-FW-014 threshold table on all eight recordings; the would-be lock is engaged >= 99 % on every rest recording and releases within 2 samples of each of the four hand events in 78EF295E854465E4 | SWE1-FW-014 | (c) |
| 14 | `fusion.c`, `fusion.h` (detector + `Fusion_setOnGround`), `FusionCal.h/.c` (size 64 -> 128, fields from 0x40), `Measurements.h/.c` (`reserved3[1]` at 0xBE), `test/test_fusion.c`, `tools/a2l_meta.json`, A2L, `Version.h` | the detector and the flag only — **no ZUPT yet, no skipping of GNSS**; the flag is observed against the board's existing behaviour | SWE1-FW-014 (b) and (c) on the host; replay shows the flag timeline of task 13 reproduced by the production code; every existing offset unchanged in `Measurements.src` and `FusionCal.src`; MISRA clean; NavStep < 300 us | SWE1-FW-014 | (b) |
| 15 | `fusion.c` (ZUPT at baro rate, skip GNSS while locked), `test/test_fusion.c` | the lock's effect | SWE1-FW-014 (a) and (d): rest velocities <= 0.03 m/s, position drift <= 0.05 m, bias converges to 2e-3 m/s^2 and 60 s open loop drifts < 3 m. Replay baseline on 02F0B754CCA975C6 is 21.9 m north p2p | SWE1-FW-014 | (b) |
| 16 | `fusion.c` (GNSS position bias, decay, rate limit), `FusionCal` fields, `test/test_fusion.c` | continuous release | SWE1-FW-015 (a)-(d): release step <= 0.02 m, 1/e in 60 +/- 3 s, rate <= 0.05 m/s, replay of 78EF295E854465E4 releases within 2 samples of each of the four events | SWE1-FW-015 | (b) |
| 17 | none (bench) | flash 14-16, calibration restore + read-back, >= 300 s at rest indoors, then a motors-at-idle recording **when the airframe exists** | SWE1-FW-014 (f); the idle recording sets the final `lockAccG` (clause g) | SWE1-FW-014 | (b) |

Ordering rationale: task 13 is host-only and produces the numbers; task 14
ships the detector as an OBSERVATION with no behaviour change, so the flag can
be judged on the bench before anything depends on it; tasks 15 and 16 then turn
on the two behaviours separately. Nothing arms or drives a motor, no pin
changes, and `Fusion_setOnGround` defaults TRUE so the pre-ASW build behaves
exactly as the bench expects.
