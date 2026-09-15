# OUTDOOR_SESSION.md — evaluating an outdoor MF4 with `tools/nav_outdoor_eval.py`

**ASPICE:** SWE.3 — verification tooling for the navigation filter · realizes SYS2-NAV-003 · related SWE1-FW-011/-012/-013/-015 · process: QuadSE/requirements/README.md

`plans/PLAN-002-commissioning-vv.md` section 5 (QuadSE) defines the outdoor
session protocol: cold start, a 5-minute rest, three 0.5 m step-and-return
tests, two walks, a hand-held hover proxy, and a final rest, recorded as one
MF4 from the groundstation. Reading the numbers those requirements ask for
off a plot, by eye, is neither deterministic nor citable — `tools/
nav_outdoor_eval.py` is the one command that turns that MF4 into them.

## Usage

```bat
python tools\nav_outdoor_eval.py <session>.mf4
python tools\nav_outdoor_eval.py <session>.mf4 --rest 60 360 ^
    --step 420 431 466 --walk 520 750 --json outdoor1_metrics.json
```

`--rest T0 T1` selects the stationary window for item 2 (TC-COM-3 / SYS2-
NAV-003 / SWE1-FW-012 c); `--walk T0 T1` selects a moving segment for item 4;
`--step T_OUT T_TARGET T_BACK` pins the auto-detector's dwell boundaries onto
one specific step-and-return episode when the default detection (three
non-overlapping rest/target/rest dwell triples, by ground speed) gets a
recording's timing wrong. All three are optional — an omitted section is
reported as "not requested", never silently skipped without saying so, and
every gated number carries its own PASS/FAIL against the value fixed in the
script (`TRUST_ENTRY_DEBOUNCE_S`, `WANDER_LIMIT_REV1_M`, `SCATTER_RATIO_MAX`,
`STEP_TARGET_M`/`STEP_TOLERANCE_M`, `CLOSURE_TOLERANCE_M`,
`RELEASE_STEP_MAX_M` — grep the requirement id in a comment right next to
each one, not here, so the two cannot drift apart unnoticed).

Output is a markdown table on stdout (six sections, one per numbered item in
the tool's own module docstring) and, with `--json`, the full nested metrics
for `evidence/INDEX.md` and Chris's `tether`-level sign-off. The tool never
drives `fusion.c` — it is pure arithmetic over already-published `Nav*`/
`Gnss*`/`Att*` A2L channels, so it needs no build step and runs on any MF4
that carries them (see the tool header for the exact channel list and which
ones are optional). The window/step/radius math is unit-tested on synthetic
traces independent of any recording (`ctest -R nav_outdoor_eval`,
`test/ref/nav_outdoor_eval_test.py`).
