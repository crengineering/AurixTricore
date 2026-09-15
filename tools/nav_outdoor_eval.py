#!/usr/bin/env python3
"""One command that turns an outdoor MF4 recording into the numbers the
navigation-filter requirements ask for, so the outdoor session
(`docs/OUTDOOR_SESSION.md`, `plans/PLAN-002-commissioning-vv.md` section 5 in
QuadSE) is evaluated the same way every time -- deterministic and citable,
not read off a plot by eye.

Reuses the MF4-reading conventions of `tools/mf4_stats.py` (asammdf, numeric
channel iteration) and `tools/nav_replay.py` (zero-order hold onto one master
clock, the tangent-plane conversion for a raw fix, the sliding-window idiom).
It does NOT drive `fusion.c` -- there is no replay here, only arithmetic over
already-published `Nav*`/`Gnss*`/`Att*` channels, so it needs no build step
and runs on any MF4 that has the channels below.

Channels used (all published names, `docs/AurixTricore.a2l` at HEAD):

    REQUIRED: NavPosNorth/East/Down, NavVelNorth/East/Down, NavGnssTrusted,
    NavStationaryLocked, NavHorizontalOk, NavGnssUpdates, NavGnssRejects,
    NavCovResets, NavDiag, GnssLatitude/Longitude/Altitude/HAccuracy/
    VAccuracy/NumSats/FixType/NavOk, BaroAltitude, AttState, AttRoll/Pitch,
    ImuPresent.

    OPTIONAL (reported "not published" if absent, never faked):
    NavInnovNorth/East/Down, g_dbgNavStepMaxTicks.

What it computes -- one function per numbered item, each independently
callable and unit-testable without an MF4 (see `test/ref/nav_outdoor_eval_test.py`):

  1. trust_gate_metrics       SWE1-FW-011 (f): trusted fraction after the
                               first 3 s of fix, hAcc range, untrusted
                               intervals, NavDiag bit-0 fraction.
  2. stationary_wander_metrics  PLAN-002 TC-COM-3 (revision-1, 3 m / 2 min)
                               + SYS2-NAV-003 stationary bullet + SWE1-FW-012
                               (c): sliding-120s-window radius, fused and raw,
                               and the 2-D scatter ratio.
  3. step_response_metrics    SYS2-NAV-003 horizontal bullets: displacement,
                               return closure, peak ground speed.
  4. walk_segment_metrics     SWE1-FW-012 (c)/(e): 2-D scatter ratio over a
                               moving segment, NavGnssRejects delta, max
                               |innovation| if published.
  5. release_bias_metrics     SWE1-FW-015: step at every trusted
                               NavStationaryLocked 1->0 tick, and the fused-
                               vs-raw track over the following 60 s.
  6. health_metrics           AttState histogram (AHRS_NO_SENSOR episodes
                               flagged), ImuPresent, counter deltas,
                               g_dbgNavStepMaxTicks if present.

Determinism: no wall clock, no randomness, no network; two runs on the same
MF4 with the same arguments produce byte-identical JSON.

Usage
-----
    python tools/nav_outdoor_eval.py <file.mf4>
    python tools/nav_outdoor_eval.py <file.mf4> --rest 60 360
    python tools/nav_outdoor_eval.py <file.mf4> --rest 60 360 --step 420 431 466 \\
                                     --walk 520 750 --json outdoor1.json

--rest/--step/--walk are all optional: omitted sections are reported as "not
requested" (rest/walk) or run through the best-effort auto-detector (step) --
see `step_response_metrics`'s docstring for what the detector assumes and
when to override it with `--step`.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import deque
from pathlib import Path
from typing import Optional

import numpy as np

# asammdf is imported lazily, inside Recording.__init__ -- everything else in
# this module (every function below the loader) is plain numpy and is
# imported and unit-tested (test/ref/nav_outdoor_eval_test.py) without it,
# same as CI's C unit-test job, which never installs asammdf.

# --------------------------------------------------------------------------
# Requirement constants -- named after the clause that fixes them, not the
# script that reads them, so a value here can be grepped straight to its item.
# --------------------------------------------------------------------------

TRUST_ENTRY_DEBOUNCE_S = 3.0     # SWE1-FW-011 (a): 30 fixes at 10 Hz
WANDER_WINDOW_S = 120.0          # PLAN-002 TC-COM-3 / SWE1-FW-015 evidence
WANDER_LIMIT_REV1_M = 3.0        # TC-COM-3 revision-1 (Chris, 2026-09-14)
SCATTER_RATIO_MAX = 1.35         # SWE1-FW-012 (c)
STEP_TARGET_M = 0.5              # SYS2-NAV-003 horizontal bullet
STEP_TOLERANCE_M = 0.20
CLOSURE_TOLERANCE_M = 0.20
RELEASE_STEP_MAX_M = 0.02        # SWE1-FW-015 (a)/(d) clause 1
RELEASE_TRACK_OFFSETS_S = (0.0, 10.0, 30.0, 60.0)

AHRS_NO_SENSOR = 3               # Ahrs.h

REQUIRED_CHANNELS = [
    "NavPosNorth", "NavPosEast", "NavPosDown",
    "NavVelNorth", "NavVelEast", "NavVelDown",
    "NavGnssTrusted", "NavStationaryLocked", "NavHorizontalOk",
    "NavGnssUpdates", "NavGnssRejects", "NavCovResets", "NavDiag",
    "GnssLatitude", "GnssLongitude", "GnssAltitude",
    "GnssHAccuracy", "GnssVAccuracy", "GnssNumSats", "GnssFixType", "GnssNavOk",
    "BaroAltitude", "AttState", "AttRoll", "AttPitch", "ImuPresent",
]

# Reported as "not published" rather than faked when absent (SWE1-FW-013's
# own rule for a missing input, reused here).
OPTIONAL_CHANNELS = [
    "NavInnovNorth", "NavInnovEast", "NavInnovDown", "g_dbgNavStepMaxTicks",
]


# --------------------------------------------------------------------------
# MF4 loading -- zero-order hold onto one master clock, exactly nav_replay.py's
# Recording class (see its "Input alignment" docstring section), just with a
# Nav* channel as the master instead of the accelerometer: this tool never
# drives anything at the sensor's own rate, it only reads published outputs.
# --------------------------------------------------------------------------

class Recording:
    def __init__(self, path: Path):
        try:
            from asammdf import MDF
        except ImportError:
            sys.exit("asammdf is not installed:  pip install asammdf")

        raw: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        with MDF(path) as mdf:
            available = set(mdf.channels_db.keys())
            missing = [n for n in REQUIRED_CHANNELS if n not in available]
            if missing:
                raise SystemExit(
                    f"{path.name}: missing required channel(s) {missing} -- "
                    "not evaluable. Not faked: report this recording as not "
                    "evaluable rather than substituting a default.")
            self.hasChannel = {n: (n in available) for n in OPTIONAL_CHANNELS}
            names = REQUIRED_CHANNELS + [n for n in OPTIONAL_CHANNELS if n in available]
            for name in names:
                sig = mdf.get(name)
                raw[name] = (sig.timestamps.astype(np.float64),
                             sig.samples.astype(np.float64))

        t_master, _ = raw["NavPosNorth"]
        self.t = t_master
        self.sig: dict[str, np.ndarray] = {}
        for name, (t_src, x_src) in raw.items():
            self.sig[name] = hold_at(t_src, x_src, self.t)

    def has(self, name: str) -> bool:
        return self.hasChannel.get(name, name in self.sig)

    def duration(self) -> float:
        return float(self.t[-1] - self.t[0]) if len(self.t) > 1 else 0.0


def hold_at(t_src: np.ndarray, x_src: np.ndarray, t_query: np.ndarray) -> np.ndarray:
    """Zero-order hold: value of x_src at the last t_src <= each t_query."""
    idx = np.searchsorted(t_src, t_query, side="right") - 1
    idx = np.clip(idx, 0, len(t_src) - 1)
    return x_src[idx]


# --------------------------------------------------------------------------
# Pure math helpers -- unit-tested directly on synthetic arrays
# (test/ref/nav_outdoor_eval_test.py), no MF4 involved.
# --------------------------------------------------------------------------

def contiguous_runs(mask: np.ndarray, t: np.ndarray) -> list[dict]:
    """Every maximal run of True in `mask`, as {t_start, t_end, duration_s,
    i_start, i_end}. O(n), one pass."""
    runs = []
    n = len(mask)
    i = 0
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            runs.append({
                "t_start": float(t[i]), "t_end": float(t[j - 1]),
                "duration_s": float(t[j - 1] - t[i]),
                "i_start": i, "i_end": j - 1,
            })
            i = j
        else:
            i += 1
    return runs


def sliding_window_radius(t: np.ndarray, x: np.ndarray, y: np.ndarray,
                           window_s: float) -> np.ndarray:
    """Max distance from EACH window's OWN mean, over every window of length
    `window_s` that fully fits (i.e. only reported once t[i]-t[0] >= window_s)
    -- the same statistic SWE1-FW-015's evidence uses for the raw-GNSS wander
    figures ("sliding windows about each window's own mean"). Two-pointer over
    the window start, so it is O(n) in the number of window ENDS visited, with
    an O(window size) inner reduction done via numpy slicing."""
    n = len(t)
    out = []
    lo = 0
    for hi in range(n):
        while t[hi] - t[lo] > window_s:
            lo += 1
        if t[hi] - t[lo] < window_s:
            continue  # window not yet full length
        xs = x[lo:hi + 1]
        ys = y[lo:hi + 1]
        mx = xs.mean()
        my = ys.mean()
        out.append(float(np.max(np.hypot(xs - mx, ys - my))))
    return np.array(out, dtype=np.float64)


def tangent_plane(lat_deg: np.ndarray, lon_deg: np.ndarray,
                   origin_idx: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """Metres from lat_deg[origin_idx]/lon_deg[origin_idx] on a flat tangent
    plane -- same formula as fusion.c's fusion_correctGnss() /
    tools/nav_replay.py's tangent_plane(), just parameterised on the origin
    index instead of always the first sample (SWE1-FW-015's own wander figures
    use each window's own mean, not a single fixed origin, for exactly the
    reason this signature exists)."""
    lat0 = lat_deg[origin_idx]
    lon0 = lon_deg[origin_idx]
    m_per_deg_lat = 111132.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))
    north = (lat_deg - lat0) * m_per_deg_lat
    east = (lon_deg - lon0) * m_per_deg_lon
    return north, east


def detect_dwells(t: np.ndarray, speed: np.ndarray, speed_thresh: float = 0.15,
                   min_dwell_s: float = 3.0) -> list[tuple[int, int]]:
    """Contiguous index ranges (inclusive) where `speed <= speed_thresh` for
    at least `min_dwell_s`. speed_thresh is deliberately above GNSS Doppler
    noise at rest (measured std well under 0.1 m/s, SWE1-FW-012 evidence) and
    below a deliberate hand-carry translation."""
    below = speed <= speed_thresh
    dwells = []
    for run in contiguous_runs(below, t):
        if run["duration_s"] >= min_dwell_s:
            dwells.append((run["i_start"], run["i_end"]))
    return dwells


def dwell_mean_pos(t: np.ndarray, posN: np.ndarray, posE: np.ndarray,
                    i0: int, i1: int, settle_s: float = 1.0) -> tuple[float, float]:
    """Mean N/E position over a dwell, trimming `settle_s` off both ends so
    the tail of the preceding motion and the start of the next one do not
    bias the baseline."""
    lo, hi = i0, i1
    while lo < hi and (t[lo] - t[i0]) < settle_s:
        lo += 1
    while hi > lo and (t[i1] - t[hi]) < settle_s:
        hi -= 1
    return float(np.mean(posN[lo:hi + 1])), float(np.mean(posE[lo:hi + 1]))


def step_episode_from_dwells(t, posN, posE, speed, dwellA, dwellB, dwellC,
                              settle_s: float = 1.0) -> dict:
    """One out-and-back episode from three dwells (rest, target, rest): the
    displacement into the target, the closure back to the start, and the
    peak ground speed spanning both moving legs (the slice between the end
    of dwell A and the start of dwell C, which also contains dwell B's own
    near-zero speed and therefore does not need to exclude it)."""
    iA0, iA1 = dwellA
    iB0, iB1 = dwellB
    iC0, iC1 = dwellC
    aN, aE = dwell_mean_pos(t, posN, posE, iA0, iA1, settle_s)
    bN, bE = dwell_mean_pos(t, posN, posE, iB0, iB1, settle_s)
    cN, cE = dwell_mean_pos(t, posN, posE, iC0, iC1, settle_s)
    displacement = math.hypot(bN - aN, bE - aE)
    closure = math.hypot(cN - aN, cE - aE)
    peak_speed = float(np.max(speed[iA1:iC0 + 1])) if iC0 >= iA1 else 0.0
    return {
        "t_rest_start": float(t[iA0]), "t_out": float(t[iA1]),
        "t_target": float(t[iB0]), "t_back": float(t[iB1]),
        "t_rest_end": float(t[iC1]),
        "displacement_m": displacement,
        "displacement_pass": bool(abs(displacement - STEP_TARGET_M) <= STEP_TOLERANCE_M),
        "closure_m": closure,
        "closure_pass": bool(closure <= CLOSURE_TOLERANCE_M),
        "ground_speed_peak_mps": peak_speed,
    }


def find_dwell_near(dwells: list[tuple[int, int]], t: np.ndarray,
                     t_target: float) -> tuple[int, int]:
    """The dwell containing t_target, or else the one whose interval is
    nearest to it -- used to resolve a user-given `--step` timestamp onto the
    actual dwell the auto-detector already found, so the same displacement/
    closure math applies whether the boundaries came from the CLI or from
    the detector."""
    for (i0, i1) in dwells:
        if t[i0] <= t_target <= t[i1]:
            return (i0, i1)
    best = min(dwells, key=lambda d: min(abs(t[d[0]] - t_target), abs(t[d[1]] - t_target)))
    return best


# --------------------------------------------------------------------------
# Section 1 -- trust gate (SWE1-FW-011 f)
# --------------------------------------------------------------------------

def trust_gate_metrics(rec: Recording) -> dict:
    t = rec.t
    navok = rec.sig["GnssNavOk"] != 0.0
    trusted = rec.sig["NavGnssTrusted"] != 0.0
    diag_bit0 = (rec.sig["NavDiag"].astype(np.uint64) & np.uint64(1)) != 0

    fix_idx = np.flatnonzero(navok)
    if fix_idx.size == 0:
        return {"evaluated": False, "reason": "no GnssNavOk=1 sample in the recording"}

    t_fix0 = float(t[fix_idx[0]])
    eval_mask = t >= (t_fix0 + TRUST_ENTRY_DEBOUNCE_S)
    if not eval_mask.any():
        return {"evaluated": False,
                "reason": f"recording ends within {TRUST_ENTRY_DEBOUNCE_S} s of first fix"}

    trusted_eval = trusted[eval_mask]
    t_eval = t[eval_mask]
    untrusted_runs = contiguous_runs(~trusted_eval, t_eval)

    hacc_fix = rec.sig["GnssHAccuracy"][navok]

    return {
        "evaluated": True,
        "t_first_fix_s": t_fix0,
        "trusted_fraction_after_entry": float(np.mean(trusted_eval)),
        "hacc_min_m": float(np.min(hacc_fix)),
        "hacc_median_m": float(np.median(hacc_fix)),
        "hacc_max_m": float(np.max(hacc_fix)),
        "untrusted_interval_count": len(untrusted_runs),
        "untrusted_interval_total_s": float(sum(r["duration_s"] for r in untrusted_runs)),
        "untrusted_intervals": untrusted_runs,
        "navdiag_bit0_fraction": float(np.mean(diag_bit0)),
    }


# --------------------------------------------------------------------------
# Section 2 -- stationary wander (PLAN-002 TC-COM-3, SYS2-NAV-003, SWE1-FW-012 c)
# --------------------------------------------------------------------------

def stationary_wander_metrics(rec: Recording, t0: float, t1: float) -> dict:
    t = rec.t
    win = (t >= t0) & (t <= t1)
    if not win.any():
        return {"evaluated": False, "reason": f"--rest [{t0}, {t1}] selects no samples"}

    # SWE1-FW-012 (f2): the wander/ratio figures are only meaningful while
    # GNSS is trusted -- an untrusted or never-trusted window reports "not
    # evaluated", not a number.
    trusted = win & (rec.sig["NavGnssTrusted"] != 0.0)
    if not trusted.any():
        return {"evaluated": False,
                "reason": "no NavGnssTrusted=1 sample in the --rest window "
                          "(SWE1-FW-012 f2: not evaluated while untrusted)"}

    tt = t[trusted]
    fused_n = rec.sig["NavPosNorth"][trusted]
    fused_e = rec.sig["NavPosEast"][trusted]

    fused_radius = sliding_window_radius(tt, fused_n, fused_e, WANDER_WINDOW_S)

    navok = trusted & (rec.sig["GnssNavOk"] != 0.0)
    result: dict = {
        "evaluated": True,
        "window_s": [t0, t1],
        "n_trusted_samples": int(trusted.sum()),
        "fused_radius_120s_median_m": float(np.median(fused_radius)) if fused_radius.size else None,
        "fused_radius_120s_max_m": float(np.max(fused_radius)) if fused_radius.size else None,
        "fused_radius_120s_pass_rev1": (
            bool(np.max(fused_radius) <= WANDER_LIMIT_REV1_M) if fused_radius.size else None),
        "fused_posN_std_m": float(np.std(fused_n, ddof=1)) if fused_n.size > 1 else 0.0,
        "fused_posE_std_m": float(np.std(fused_e, ddof=1)) if fused_e.size > 1 else 0.0,
    }

    if navok.sum() > 1:
        lat = rec.sig["GnssLatitude"][navok]
        lon = rec.sig["GnssLongitude"][navok]
        t_raw = t[navok]
        raw_n, raw_e = tangent_plane(lat, lon, origin_idx=0)
        raw_radius = sliding_window_radius(t_raw, raw_n, raw_e, WANDER_WINDOW_S)
        raw_std_n = float(np.std(raw_n, ddof=1))
        raw_std_e = float(np.std(raw_e, ddof=1))
        fused_scatter = math.hypot(result["fused_posN_std_m"], result["fused_posE_std_m"])
        raw_scatter = math.hypot(raw_std_n, raw_std_e)
        ratio = (fused_scatter / raw_scatter) if raw_scatter > 0.0 else None
        result.update({
            "raw_radius_120s_median_m": float(np.median(raw_radius)) if raw_radius.size else None,
            "raw_radius_120s_max_m": float(np.max(raw_radius)) if raw_radius.size else None,
            "raw_posN_std_m": raw_std_n,
            "raw_posE_std_m": raw_std_e,
            "scatter_ratio_fused_over_raw": ratio,
            "scatter_ratio_pass": (bool(ratio <= SCATTER_RATIO_MAX) if ratio is not None else None),
        })
    else:
        result.update({
            "raw_radius_120s_median_m": None, "raw_radius_120s_max_m": None,
            "raw_posN_std_m": None, "raw_posE_std_m": None,
            "scatter_ratio_fused_over_raw": None, "scatter_ratio_pass": None,
        })

    return result


# --------------------------------------------------------------------------
# Section 3 -- step response (SYS2-NAV-003 horizontal bullets)
# --------------------------------------------------------------------------

def step_response_metrics(rec: Recording, explicit_step: Optional[tuple[float, float, float]] = None,
                           speed_thresh: float = 0.15, min_dwell_s: float = 3.0) -> dict:
    """One episode per --step triple (t_out, t_target, t_back), or, if none
    is given, every non-overlapping (rest, target, rest) triple the dwell
    detector finds -- a REPORTING aid on the protocol's own repeated pattern
    (rest 30 s / 0.5 m step / rest 10 s / step back / rest 30 s, times 3),
    not a guaranteed segmentation of an arbitrary recording: an unusual
    speed profile can merge or split dwells the way any threshold detector
    can. Pass `--step` for anything the auto-detector gets wrong."""
    t = rec.t
    speed = np.hypot(rec.sig["NavVelNorth"], rec.sig["NavVelEast"])
    posN, posE = rec.sig["NavPosNorth"], rec.sig["NavPosEast"]

    if explicit_step is not None:
        dwells = detect_dwells(t, speed, speed_thresh, min_dwell_s)
        if len(dwells) < 3:
            return {"evaluated": False,
                    "reason": "fewer than 3 dwells detected around --step; "
                              "cannot resolve out/target/back"}
        t_out, t_target, t_back = explicit_step
        dA = find_dwell_near(dwells, t, t_out)
        dB = find_dwell_near(dwells, t, t_target)
        dC = find_dwell_near(dwells, t, t_back)
        episodes = [step_episode_from_dwells(t, posN, posE, speed, dA, dB, dC)]
        return {"evaluated": True, "mode": "explicit", "episodes": episodes}

    dwells = detect_dwells(t, speed, speed_thresh, min_dwell_s)
    episodes = []
    for k in range(0, len(dwells) - 2, 2):
        episodes.append(step_episode_from_dwells(
            t, posN, posE, speed, dwells[k], dwells[k + 1], dwells[k + 2]))
    if not episodes:
        return {"evaluated": False,
                "reason": "auto-detector found fewer than 3 dwells "
                          "(no step pattern visible) -- use --step"}
    return {"evaluated": True, "mode": "auto", "episodes": episodes}


# --------------------------------------------------------------------------
# Section 4 -- walk segments (SWE1-FW-012 c/e)
# --------------------------------------------------------------------------

def walk_segment_metrics(rec: Recording, t0: float, t1: float) -> dict:
    t = rec.t
    win = (t >= t0) & (t <= t1)
    if not win.any():
        return {"evaluated": False, "reason": f"--walk [{t0}, {t1}] selects no samples"}

    trusted = win & (rec.sig["NavGnssTrusted"] != 0.0)
    eval_mask = trusted if trusted.any() else win
    note = None if trusted.any() else (
        "no NavGnssTrusted=1 sample in --walk window; reporting over the "
        "untrusted window anyway (SWE1-FW-012 f2 would say 'not evaluated' "
        "for the ratio, kept here as an explicit caveat, not silently applied)")

    fused_n = rec.sig["NavPosNorth"][eval_mask]
    fused_e = rec.sig["NavPosEast"][eval_mask]
    fused_std_n = float(np.std(fused_n, ddof=1)) if fused_n.size > 1 else 0.0
    fused_std_e = float(np.std(fused_e, ddof=1)) if fused_e.size > 1 else 0.0
    fused_scatter = math.hypot(fused_std_n, fused_std_e)

    navok = win & (rec.sig["GnssNavOk"] != 0.0)
    result: dict = {
        "evaluated": True, "window_s": [t0, t1], "note": note,
        "fused_posN_std_m": fused_std_n, "fused_posE_std_m": fused_std_e,
    }
    if navok.sum() > 1:
        lat = rec.sig["GnssLatitude"][navok]
        lon = rec.sig["GnssLongitude"][navok]
        raw_n, raw_e = tangent_plane(lat, lon, origin_idx=0)
        raw_std_n = float(np.std(raw_n, ddof=1))
        raw_std_e = float(np.std(raw_e, ddof=1))
        raw_scatter = math.hypot(raw_std_n, raw_std_e)
        ratio = (fused_scatter / raw_scatter) if raw_scatter > 0.0 else None
        result.update({
            "raw_posN_std_m": raw_std_n, "raw_posE_std_m": raw_std_e,
            "scatter_ratio_fused_over_raw": ratio,
            "scatter_ratio_pass": (bool(ratio <= SCATTER_RATIO_MAX) if ratio is not None else None),
        })
    else:
        result.update({"raw_posN_std_m": None, "raw_posE_std_m": None,
                        "scatter_ratio_fused_over_raw": None, "scatter_ratio_pass": None})

    rej = rec.sig["NavGnssRejects"][win]
    result["gnss_rejects_delta"] = float(rej[-1] - rej[0]) if rej.size else 0.0

    for axis, name in (("north", "NavInnovNorth"), ("east", "NavInnovEast"), ("down", "NavInnovDown")):
        if rec.has(name):
            result[f"max_abs_innov_{axis}_m"] = float(np.max(np.abs(rec.sig[name][win])))
        else:
            result[f"max_abs_innov_{axis}_m"] = None
    result["innovation_published"] = rec.has("NavInnovNorth")

    return result


# --------------------------------------------------------------------------
# Section 5 -- release bias (SWE1-FW-015, only trusted releases)
# --------------------------------------------------------------------------

def release_bias_metrics(rec: Recording) -> dict:
    t = rec.t
    locked = rec.sig["NavStationaryLocked"] != 0.0
    trusted = rec.sig["NavGnssTrusted"] != 0.0
    posN, posE = rec.sig["NavPosNorth"], rec.sig["NavPosEast"]
    navok = rec.sig["GnssNavOk"] != 0.0

    releases = []
    for i in range(1, len(t)):
        if locked[i - 1] and (not locked[i]) and trusted[i]:
            step = math.hypot(posN[i] - posN[i - 1], posE[i] - posE[i - 1])
            entry = {
                "t_release_s": float(t[i]), "i_release": i,
                "step_m": step, "step_pass": bool(step <= RELEASE_STEP_MAX_M),
                "track": [],
            }
            if navok[:i + 1].any():
                navok_idx = np.flatnonzero(navok)
                lat = rec.sig["GnssLatitude"][navok_idx]
                lon = rec.sig["GnssLongitude"][navok_idx]
                raw_n_all, raw_e_all = tangent_plane(lat, lon, origin_idx=0)
                t_navok = t[navok_idx]
                for dt in RELEASE_TRACK_OFFSETS_S:
                    t_target = t[i] + dt
                    if t_target > t[-1]:
                        break
                    j = int(np.searchsorted(t, t_target))
                    j = min(j, len(t) - 1)
                    rn = float(np.interp(t[j], t_navok, raw_n_all))
                    re_ = float(np.interp(t[j], t_navok, raw_e_all))
                    entry["track"].append({
                        "dt_after_release_s": dt, "t_s": float(t[j]),
                        "fused_n_m": float(posN[j]), "fused_e_m": float(posE[j]),
                        "raw_n_m": rn, "raw_e_m": re_,
                        "distance_m": float(math.hypot(posN[j] - rn, posE[j] - re_)),
                    })
            releases.append(entry)

    return {"n_trusted_releases": len(releases), "releases": releases}


# --------------------------------------------------------------------------
# Section 6 -- health
# --------------------------------------------------------------------------

def health_metrics(rec: Recording) -> dict:
    t = rec.t
    att_state = rec.sig["AttState"].astype(np.int64)
    duration = rec.duration()

    histogram = {}
    for state in (0, 1, 2, 3):
        mask = att_state == state
        histogram[str(state)] = {
            "fraction": float(np.mean(mask)),
            "duration_s": float(np.sum(mask) * (duration / len(t))) if len(t) else 0.0,
        }

    no_sensor_episodes = contiguous_runs(att_state == AHRS_NO_SENSOR, t)

    imu_present = rec.sig["ImuPresent"] != 0.0
    imu_absent_episodes = contiguous_runs(~imu_present, t)

    cov = rec.sig["NavCovResets"]
    rej = rec.sig["NavGnssRejects"]

    result = {
        "att_state_histogram": histogram,
        "att_no_sensor_episodes": no_sensor_episodes,
        "imu_present_fraction": float(np.mean(imu_present)),
        "imu_absent_episodes": imu_absent_episodes,
        "nav_cov_resets_delta": float(cov[-1] - cov[0]) if cov.size else 0.0,
        "nav_gnss_rejects_delta": float(rej[-1] - rej[0]) if rej.size else 0.0,
    }
    if rec.has("g_dbgNavStepMaxTicks"):
        result["dbg_nav_step_max_ticks"] = float(np.max(rec.sig["g_dbgNavStepMaxTicks"]))
    else:
        result["dbg_nav_step_max_ticks"] = None
    return result


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------

def _fmt(v) -> str:
    if v is None:
        return "n/a"
    if isinstance(v, bool):
        return "PASS" if v else "FAIL"
    if isinstance(v, float):
        return f"{v:.4f}"
    return str(v)


def print_markdown_report(path: Path, metrics: dict) -> None:
    print(f"# nav_outdoor_eval: {path}")
    print()
    print("| section | metric | value |")
    print("|---|---|---|")

    tg = metrics["trust_gate"]
    if tg["evaluated"]:
        print(f"| 1 trust gate | trusted fraction after entry | {_fmt(tg['trusted_fraction_after_entry'])} |")
        print(f"| 1 trust gate | hAcc min/median/max [m] | {_fmt(tg['hacc_min_m'])} / "
              f"{_fmt(tg['hacc_median_m'])} / {_fmt(tg['hacc_max_m'])} |")
        print(f"| 1 trust gate | untrusted intervals (count, total s) | "
              f"{tg['untrusted_interval_count']}, {_fmt(tg['untrusted_interval_total_s'])} |")
        print(f"| 1 trust gate | NavDiag bit-0 fraction | {_fmt(tg['navdiag_bit0_fraction'])} |")
    else:
        print(f"| 1 trust gate | not evaluated | {tg['reason']} |")

    sw = metrics["stationary_wander"]
    if sw.get("evaluated"):
        print(f"| 2 rest wander | fused radius 120s median/max [m] | "
              f"{_fmt(sw['fused_radius_120s_median_m'])} / {_fmt(sw['fused_radius_120s_max_m'])} |")
        print(f"| 2 rest wander | TC-COM-3 rev1 (<= {WANDER_LIMIT_REV1_M} m) | "
              f"{_fmt(sw['fused_radius_120s_pass_rev1'])} |")
        print(f"| 2 rest wander | raw radius 120s median/max [m] | "
              f"{_fmt(sw['raw_radius_120s_median_m'])} / {_fmt(sw['raw_radius_120s_max_m'])} |")
        print(f"| 2 rest wander | scatter ratio fused/raw (<= {SCATTER_RATIO_MAX}) | "
              f"{_fmt(sw['scatter_ratio_fused_over_raw'])} ({_fmt(sw['scatter_ratio_pass'])}) |")
    else:
        print(f"| 2 rest wander | not evaluated | {sw.get('reason', 'no --rest given')} |")

    sr = metrics["step_response"]
    if sr.get("evaluated"):
        for n, ep in enumerate(sr["episodes"], 1):
            print(f"| 3 step {n} | displacement [m] (target {STEP_TARGET_M}+-{STEP_TOLERANCE_M}) | "
                  f"{_fmt(ep['displacement_m'])} ({_fmt(ep['displacement_pass'])}) |")
            print(f"| 3 step {n} | closure [m] (<= {CLOSURE_TOLERANCE_M}) | "
                  f"{_fmt(ep['closure_m'])} ({_fmt(ep['closure_pass'])}) |")
            print(f"| 3 step {n} | ground speed peak [m/s] | {_fmt(ep['ground_speed_peak_mps'])} |")
    else:
        print(f"| 3 step | not evaluated | {sr.get('reason', 'no --step given')} |")

    wk = metrics["walk_segment"]
    if wk.get("evaluated"):
        print(f"| 4 walk | scatter ratio fused/raw | {_fmt(wk['scatter_ratio_fused_over_raw'])} "
              f"({_fmt(wk['scatter_ratio_pass'])}) |")
        print(f"| 4 walk | NavGnssRejects delta | {_fmt(wk['gnss_rejects_delta'])} |")
        print(f"| 4 walk | max abs innov N/E/D [m] | {_fmt(wk['max_abs_innov_north_m'])} / "
              f"{_fmt(wk['max_abs_innov_east_m'])} / {_fmt(wk['max_abs_innov_down_m'])} |")
    else:
        print(f"| 4 walk | not evaluated | {wk.get('reason', 'no --walk given')} |")

    rb = metrics["release_bias"]
    print(f"| 5 release bias | trusted releases found | {rb['n_trusted_releases']} |")
    for n, rel in enumerate(rb["releases"], 1):
        print(f"| 5 release {n} | step at release [m] (<= {RELEASE_STEP_MAX_M}) | "
              f"{_fmt(rel['step_m'])} ({_fmt(rel['step_pass'])}) |")

    hm = metrics["health"]
    hist = ", ".join(f"{k}:{_fmt(v['fraction'])}" for k, v in hm["att_state_histogram"].items())
    print(f"| 6 health | AttState fraction (0/1/2/3) | {hist} |")
    print(f"| 6 health | AttState=3 episodes | {len(hm['att_no_sensor_episodes'])} |")
    print(f"| 6 health | ImuPresent fraction | {_fmt(hm['imu_present_fraction'])} |")
    print(f"| 6 health | NavCovResets / NavGnssRejects delta | "
          f"{_fmt(hm['nav_cov_resets_delta'])} / {_fmt(hm['nav_gnss_rejects_delta'])} |")
    print(f"| 6 health | g_dbgNavStepMaxTicks | {_fmt(hm['dbg_nav_step_max_ticks'])} |")

    if hm["att_no_sensor_episodes"]:
        print()
        print("AttState=3 (AHRS_NO_SENSOR) episodes:")
        for ep in hm["att_no_sensor_episodes"]:
            print(f"  t={ep['t_start']:.1f}-{ep['t_end']:.1f} s ({ep['duration_s']:.1f} s)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", type=Path, help="MF4 recording")
    ap.add_argument("--rest", type=float, nargs=2, metavar=("T0", "T1"),
                     default=None, help="stationary rest window [s] for item 2")
    ap.add_argument("--step", type=float, nargs=3, metavar=("T_OUT", "T_TARGET", "T_BACK"),
                     default=None, help="explicit step timing for item 3 "
                                         "(default: auto-detect)")
    ap.add_argument("--walk", type=float, nargs=2, metavar=("T0", "T1"),
                     default=None, help="walk-segment window [s] for item 4")
    ap.add_argument("--json", type=Path, default=None, help="write full metrics as JSON")
    ap.add_argument("--quiet", action="store_true", help="suppress the markdown report")
    args = ap.parse_args()

    if not args.file.is_file():
        sys.exit(f"no such file: {args.file}")

    rec = Recording(args.file)

    metrics = {
        "file": str(args.file),
        "duration_s": rec.duration(),
        "trust_gate": trust_gate_metrics(rec),
        "stationary_wander": (stationary_wander_metrics(rec, *args.rest) if args.rest
                              else {"evaluated": False, "reason": "no --rest given"}),
        "step_response": step_response_metrics(
            rec, tuple(args.step) if args.step else None),
        "walk_segment": (walk_segment_metrics(rec, *args.walk) if args.walk
                         else {"evaluated": False, "reason": "no --walk given"}),
        "release_bias": release_bias_metrics(rec),
        "health": health_metrics(rec),
    }

    if args.json:
        args.json.write_text(json.dumps(metrics, indent=2, sort_keys=True), encoding="utf-8")

    if not args.quiet:
        print_markdown_report(args.file, metrics)

    return 0


if __name__ == "__main__":
    sys.exit(main())
