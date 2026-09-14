# nav_replay baseline and sweep — 2026-09-14

SWE1-FW-013 tasks 1-2, SWE1-FW-012 task 3 (`docs/NAV_STRAND_2026-09.md` section 7,
rows 1-3). Produced entirely offline with `tools/nav_replay.py` against
`test/build/gen_fusion_trace` (the production `fusion.c`/`FusionCal.c`,
compiled unchanged for the host). No board contact, no XCP write, no flash.

Evidence: `Measurement Data/20260914_nav_replay/` —
`baseline_2026-09-14.csv`, `section2_reproduction_indoor_2026-09-14.csv`,
`sweep_gnssPosRScale_sigmaGnssVel_2026-09-14.csv` (SHA-256, first 16 hex:
`CC4BDCA482B6FDC8`, `CB301674A4148F6E`, `E7FB16C95F7C40EB`).

## Task 1 — fidelity: what reproduces and what structurally cannot

**Inputs reproduce exactly.** `tools/nav_replay.py`'s reconstruction of
`GnssAltitude`/`BaroAltitude` matches `docs/NAV_STRAND_2026-09.md` section 2's
own published numbers to the reported precision (`GnssAltitude` mean 548.01,
std 9.346, p2p 35.84; `BaroAltitude` mean 427.82, p2p 0.320 — both exact).

**The barometer-observable combination reproduces tightly.** `fusion.c`'s
barometer update sees only `d + measBias` (`h = [1,0,0,1]`,
`fusion_correctBaro()`); replayed against the 2026-09-14 indoor recording
(02F0B754CCA975C6, 4691 samples), `(NavPosDown + NavBaroBias)` matches the
logged sum to **std 0.70 %, p2p 6.27 %, drift 0.45 %** (mean/min/max are
near-zero quantities where a millimetre difference reads as a large
percentage — see `section2_reproduction_indoor_2026-09-14.csv`).

**`NavPosDown`/`NavBaroBias` individually do NOT reproduce to the SWE1-FW-013
(b) bound (0.5 m RMS, 20 % p2p), and this is structural, not a tool defect.**
Measured: RMS 5.29 m, p2p 35.3 % off (12.63 m replayed vs 9.34 m logged).

Root cause, confirmed three independent ways:
1. The recording's own `NavGnssUpdates` starts at **6797**, not 0 — the board
   had already fused >11 minutes of GNSS fixes (at 10 Hz) before this MF4
   begins. `logged NavVarDown[0] = 0.0648`, far below the `Fusion_init()`
   covariance (`FUSION_P_POS_INIT = 1.0`) — the filter's covariance was
   already converged, not freshly reset, at t=0 of the recording.
2. `Fusion_setBaroAlt()`'s reference (`refM`) latches on the FIRST barometer
   sample after `Fusion_init()` (`fusion.c`, `FusionLatch.h`). A cold replay's
   `refM` is therefore a different absolute pressure reading than the real
   board's (set at an earlier, unlogged boot), which only the barometer
   ALONE cannot resolve — only the SUM `d + measBias` is observable from it
   (`fusion.h`'s own comment on `measBias`). This is exactly the
   "missing input" `tools/nav_replay.py`'s header and SWE1-FW-013's own text
   call out for `refM`.
3. The unobservable `d - measBias` split this creates decays only through the
   (unclamped, today) GNSS-altitude Kalman gain — measured at 4.87e-4/fix
   (`docs/NAV_STRAND_2026-09.md` section 1) — far too slow to unwind an
   initial mismatch within a 469 s run. This is the identical mechanism
   SWE1-FW-010 exists to fix.

**Consequence:** no cold-`Fusion_init()` replay of THIS recording can meet
the literal (b) criterion on `NavPosDown`/`NavBaroBias` absolute values; doing
so would need the filter's true pre-recording state, which is not present in
any MF4 and cannot be reconstructed without faking it. `test/ref/
nav_replay_fidelity.py` (ctest `nav_replay`) therefore asserts fidelity on the
OBSERVABLE combination instead (0.058 m RMS measured on its 400-fix excerpt,
budget 0.20 m) plus the structural counters — the part of the mechanism a
cold replay legitimately can judge. See that script's header for the full
argument. **Flagging back to flight-architect:** either redefine SWE1-FW-013
(b) onto the observable sum / a detrended form, or add a state-seeding
command to `gen_fusion_trace` (a `src`+`test` change outside this task's file
list) if literal absolute-value fidelity is required.

## Task 2 — baseline, all six replayable recordings

`aurix_log_20260822_175537_gnss_outdoor_test.mf4` (BF56B577C55E8EDC) is
**not replayable**: it predates `AttAccNed{0,1,2}` and every `Nav*` output
(pre-estimator GNSS validation log) — `nav_replay.py` detects this and
refuses cleanly rather than fabricating an acceleration input. The other six
recordings all replay:

| recording | n | dur [s] | posD p2p [m] | vs logged | NIS N / E | posN/E std [m] | ratio N / E to raw |
|---|---|---|---|---|---|---|---|
| indoor 02F0B754 | 4691 | 469.1 | 12.63 | +35.3 % (see task 1) | 0.178 / 0.040 | 6.01 / 3.64 | 0.79 / 0.93 |
| indoor lift 78EF2955 | 5746 | 574.7 | 8.82 | +11.1 % | 0.091 / 0.083 | 7.46 / 5.82 | 1.07 / 1.04 |
| t1 07B44DE1 | 3009 | 300.9 | 4.35 | +1.7 % | 0.077 / 0.117 | 1.82 / 2.89 | 0.98 / 3.14 |
| t2 951F68E6 | 1415 | 141.4 | 3.58 | +13.2 % | 0.017 / 0.014 | 4.21 / 7.09 | 0.99 / 0.95 |
| t4 13D56B26 | 653 | 65.2 | 2.76 | +27.6 % | 0.011 / 0.006 (n<1000) | 2.34 / 4.08 | 0.98 / 0.97 |
| t5 3F3D79AB | 531 | 53.0 | 2.54 | +18.6 % | 0.008 / 0.003 (n<1000) | 2.19 / 6.27 | 1.08 / 0.98 |

Full precision in `baseline_2026-09-14.csv`. `NIS_north`/`east` at
`gnssPosRScale = 8` (today's compiled default) for t1 — **0.077 / 0.117** —
match `docs/NAV_TUNING.md`'s own measured **0.077 / 0.124** to within 6 %,
cross-validating this tool's NIS methodology against an independently
produced number. `t1` east ratio (3.14x raw) essentially reproduces the
pre-T6 `NAV_TUNING.md` figure (3.18x) — confirming, as
`docs/NAV_STRAND_2026-09.md` section 2.4 already warned, that T6 alone did
not move this number; `gnssPosRScale`/`sigmaGnssVel` are what task 3 sweeps.
`baroRejects`, `baroResets`, `gnssRejects`, `covResets` are 0 on every
recording (structural invariant holds).

## Task 3 — offline sweep, `gnssPosRScale` x `sigmaGnssVel`

Grid `{8, 4, 2, 1} x {0.3, 0.2, 0.12, 0.08}`, replayed on t1 and indoor (the
required pair) plus t2/t4/t5 for the structural checks; full 80-row table in
`sweep_gnssPosRScale_sigmaGnssVel_2026-09-14.csv`.

**Structural invariants hold everywhere in the grid:** `NavGnssRejects` stays
at its baseline (0) on t1/t2/t4 for all 16 combinations — no regression from
narrowing the gate. `NavCovResets` stays 0 everywhere. t5 (reported, not
gated per SWE1-FW-012 e): `NavGnssRejects = 0` at every candidate, against
the board's own previously measured 6 — explained, not contradicted, by
`nav_replay.py`'s documented limitation that a 10 Hz replay aliases the
acceleration input on a dynamic run, muting exactly the velocity overshoot
that produced those rejects on the board. This is hardware evidence per
SWE1-FW-012 (g), not something this replay can decide either way.

**t1, the NIS-consistency criterion is met at the SWE1-FW-012-recommended
pair** (`gnssPosRScale = 1`, `sigmaGnssVel = 0.12`): NIS north **0.555**, NIS
east **0.812**, both inside 0.5-2.0. On the indoor run at the same pair: NIS
north 1.259 (inside band), NIS east 0.289 (below 0.5, expected indoors per
SWE1-FW-012 (b) — reported, not gated).

**t1, the "never worse than raw" ratio criterion (`std(NavPosEast) <= 1.2x`
raw 0.918 m) is NOT met by any of the 16 candidates, and the two criteria
pull in opposite directions:**

| gnssPosRScale | sigmaGnssVel | NIS north | NIS east | posE std [m] | ratio to raw |
|---|---|---|---|---|---|
| 1 | 0.30 | 0.208 | 0.212 | 1.820 | **1.98x** (best ratio in the grid) |
| 1 | 0.12 | 0.555 | 0.812 | 2.753 | 3.00x (SWE1-FW-012's own recommendation) |
| 1 | 0.08 | 0.759 | 1.224 | 3.162 | 3.44x |
| 8 | 0.30 | 0.077 | 0.117 | 2.888 | 3.14x (today) |

Lowering `sigmaGnssVel` (needed to push NIS up into the 0.5-2.0 band) makes
`posE_std` **worse**, not better: a smaller `sigmaGnssVel` trusts the raw,
noisy GNSS velocity more, and that noise couples into position through the
velocity-position cross-covariance term faster than the tighter position gate
removes it. The best ratio anywhere in the grid, 1.98x at
(`gnssPosRScale = 1`, `sigmaGnssVel = 0.3`), still fails the 1.2x band by
65 %. **Finding for flight-architect:** the two parameters this task was
scoped to sweep cannot jointly satisfy SWE1-FW-012's NIS-consistency and
never-worse-than-raw criteria on t1; achieving both looks like it needs a
third lever (`sigmaAccH`, which task 3 was not scoped to touch) or a review
of which criterion should be binding. No parameter is written anywhere but
in this report and the sweep CSV — the compiled defaults (`FusionCal.c`) and
the board are untouched.

**Recommendation, pending that review:** `gnssPosRScale = 1`,
`sigmaGnssVel = 0.12` — SWE1-FW-012's own proposed pair — is the best
candidate against the NIS-consistency criterion, which the design note treats
as primary ("the bar this exists to meet" language in SWE1-FW-012 is about
the ratio, so this is a recommendation under an open question, not a closed
one).
