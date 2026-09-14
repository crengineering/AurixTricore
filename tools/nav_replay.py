#!/usr/bin/env python3
"""tools/nav_replay.py -- MF4 front end for the fusion.c host harness.

Drives the PRODUCTION `fusion.c` / `FusionCal.c` (compiled unchanged for the
host, `test/gen_fusion_trace.c`) from a recorded MF4, so a tuning or gating
change can be judged against real receiver behaviour without a board and
without a flight. Built for SWE1-FW-013, task 1 of
`docs/NAV_STRAND_2026-09.md` section 7.

WHAT THE REPLAY CAN AND CANNOT DECIDE -- read this before trusting a number.

Can: covariance, Kalman gains, NIS, the drift of `posD` / `baroBias`, every
gating decision and every counter `gen_fusion_trace` exposes. Since T6 the
acceleration process noise is a PSD and its Q integral is rate-invariant
(fusion.c:431-448), so replaying at the recording's logged rate (10 Hz here)
gives the same covariance growth per wall-clock second as the board's
~1014 Hz `Fusion_update()`.

Cannot: anything dominated by the acceleration input BETWEEN fixes.
`AttAccNed` is logged at 10 Hz, so a replayed dynamic manoeuvre is aliased.
The walking / velocity-overshoot criteria of `docs/NAV_TUNING.md` section 5
and the SYS2-NAV-003 step test stay hardware evidence.

Missing inputs, and what each one limits:
  - `refM` (the NVM sea-level barometer reference) is not logged. This tool
    latches it from the FIRST `BaroAltitude` sample instead, which shifts
    the absolute altitude origin and leaves every RELATIVE criterion in
    this strand (p2p, max-per-60s, RMS-against-logged) untouched.
  - `sAcc` (UBX-NAV-PVT velocity accuracy) was never decoded on the board,
    so it cannot be replayed; `sigmaGnssVel` stays a chosen, not a measured,
    value here exactly as it is on the board (SWE1-FW-012, item 11).
  - `GnssLatitude`/`GnssLongitude` are published as float32 DEGREES, which
    the a2l comment for `gnsslatDeg` already documents as costing ~0.4 m of
    the receiver's native 1.1 cm (1e-7 deg integer) resolution. This tool
    reconstructs the `Fusion_setGnss()` 1e-7 integer input by rounding that
    float32 back up, so every replayed horizontal position carries a
    constant, run-specific offset up to ~0.4 m against what the board's
    exact integers produced. It does NOT add scatter -- the raw reference
    sigmas used for the "never worse than raw" ratio (SWE1-FW-012 c) are
    computed from the SAME published channels, so the ratio is unaffected.
    It only limits an ABSOLUTE-position replay claim, which this strand
    never makes.
  - `Fusion_update()`'s own `valid` argument (IMU-read-ok AND AHRS running)
    is not published as a signal. This tool substitutes "accel and dt are
    both finite and dt is inside fusion.c's own [1e-4, 0.2] s sanity band",
    which is everything `Fusion_update()` checks except the AHRS-alignment
    state. The difference is confined to the first few seconds after boot
    (gyro-bias calibration / alignment), before which the AHRS output is
    not physically meaningful anyway; it is immaterial to a >= 400 s
    at-rest criterion and is called out here rather than silently assumed.
  - Neither `gen_fusion_trace.c`'s command language nor its CSV output
    carry `NavInnovVelNorth/East`, `NavDropped` or `NavGnssDupes` --
    extending them is a `src/`/`test/` change outside this task's file list
    (SWE1-FW-013 task 1: `tools/nav_replay.py`, `test/CMakeLists.txt`,
    `test/data/` only). The velocity-NIS clause of SWE1-FW-012(d) is
    therefore reported as "not computed" here, not gated.

Input alignment. Every recording so far logs the estimator's own outputs
and its raw sensor inputs on ONE shared 10 Hz timebase, but this tool does
not assume that: the command stream is driven off `AttAccNed{0,1,2}`'s own
timestamps (one `STEP` per accelerometer sample, exactly what
`Fusion_update()` does on the board), and every other signal (GNSS, baro) is
sampled with a zero-order hold at or before each such timestamp -- the same
"whatever is latched" semantics `Fusion_update()` itself uses. A GNSS/baro
channel that happens to log slower than the accelerometer is handled
correctly by this hold with no special-casing.

Usage
-----
    python tools/nav_replay.py <file.mf4>
    python tools/nav_replay.py <file.mf4> --cal gnssPosRScale=1.0 --cal sigmaGnssVel=0.12
    python tools/nav_replay.py <file.mf4> --start 0 --end 40 --dump-commands cmds.txt
    python tools/nav_replay.py <file.mf4> --dump-csv trace.csv --dump-metrics metrics.json
    python tools/nav_replay.py <file.mf4> --baseline metrics_before.json

Determinism (SWE1-FW-013 clause 4): no wall clock, no randomness, no network,
no dict/set iteration whose order depends on anything but insertion order.
Two runs on the same MF4 with the same `--cal` set produce byte-identical
CSV and metrics; CI asserts this by running twice and diffing.
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Optional

import numpy as np

try:
    from asammdf import MDF
except ImportError:
    sys.exit("asammdf is not installed:  pip install asammdf")

# Names gen_fusion_trace.c's setCal() accepts (test/gen_fusion_trace.c:57-70).
CAL_FIELDS = {
    "twoKpAcc", "twoKpMag", "twoKi",
    "sigmaAccD", "sigmaBaro", "sigmaBaroRw", "tauBaroBias",
    "sigmaAccH", "sigmaGnssVel", "gnssPosRScale",
    "gateSigmaSq", "gateMinM", "sigmaAccRw",
}

# Compiled defaults (FusionCal.c) -- used to reconstruct R when a --cal
# override was not given for that field, so NIS can be computed for the
# values the run actually used, sweep or not.
CAL_DEFAULTS = {
    "gnssPosRScale": 8.0,
    "sigmaGnssVel": 0.3,
}

FUSION_GNSS_HACC_MIN = 1.0    # fusion.c:96
WINDOW_60S = 60.0

CSV_COLUMNS = [
    "step", "d", "vd", "accBiasD", "baroBias", "innov", "p00", "aD",
    "posN", "posE", "velN", "velE", "accBiasN", "accBiasE", "innovN",
    "innovE", "pNN", "aN", "aE",
    "rejects", "resets", "gnssRejects", "gnssUpdates", "covResets",
    "verticalOk", "horizontalOk", "originSet",
]

FUSION_DT_MIN = 1.0e-4
FUSION_DT_MAX = 0.2
FUSION_INPUT_MAX = 1.0e6


def default_exe() -> Optional[Path]:
    """Locate the already-built gen_fusion_trace next to this script's repo,
    without assuming a particular build directory name (ninja vs make)."""
    root = Path(__file__).resolve().parent.parent / "test"
    for build in ("build", "build_cov", "build_ninja"):
        for name in ("gen_fusion_trace.exe", "gen_fusion_trace"):
            candidate = root / build / name
            if candidate.is_file():
                return candidate
    return None


def usable(v: float, limit: float = FUSION_INPUT_MAX) -> bool:
    """Mirrors fusion.c's fusion_usable(): finite and inside a sane band."""
    return math.isfinite(v) and (-limit < v < limit)


def hold_at(t_src: np.ndarray, x_src: np.ndarray, t_query: np.ndarray) -> np.ndarray:
    """Zero-order hold: value of x_src at the last t_src <= each t_query.
    Matches the latch semantics Fusion_update() itself reads through."""
    idx = np.searchsorted(t_src, t_query, side="right") - 1
    idx = np.clip(idx, 0, len(t_src) - 1)
    return x_src[idx]


class Recording:
    """Every signal nav_replay needs, aligned onto the accelerometer's own
    timestamps (see module docstring, 'Input alignment')."""

    NEEDED = [
        "AttAccNed0", "AttAccNed1", "AttAccNed2",
        "BaroAltitude", "BaroPresent",
        "GnssLatitude", "GnssLongitude", "GnssAltitude", "GnssGroundSpeed",
        "GnssHeading", "GnssHAccuracy", "GnssPresent", "GnssNavOk",
        "NavGnssITow",
        "NavPosDown", "NavBaroBias", "NavPosNorth", "NavPosEast",
        "NavVarNorth", "NavVarDown",
        "NavBaroRejects", "NavBaroResets", "NavGnssRejects", "NavGnssUpdates",
        "NavCovResets", "NavVerticalOk", "NavHorizontalOk", "NavOriginSet",
        "NavInnovDown", "NavInnovNorth", "NavInnovEast",
    ]

    def __init__(self, path: Path, start: Optional[float], end: Optional[float]):
        raw = {}
        with MDF(path) as mdf:
            available = set(mdf.channels_db.keys())
            missing = [name for name in self.NEEDED if name not in available]
            if missing:
                raise SystemExit(
                    f"{path.name}: not replayable -- missing channel(s) "
                    f"{missing}. This recording predates one or more of "
                    "AttAccNed{0,1,2} (needed to drive STEP) and the Nav* "
                    "estimator outputs (needed as ground truth); "
                    "nav_replay.py cannot reconstruct either from raw "
                    "IMU/GNSS alone (that IS the estimator). Not faked -- "
                    "report this recording as not replayable.")
            for name in self.NEEDED:
                sig = mdf.get(name)
                raw[name] = (sig.timestamps.astype(np.float64),
                             sig.samples.astype(np.float64))

        t_acc = raw["AttAccNed0"][0]
        lo = -np.inf if start is None else start
        hi = np.inf if end is None else end
        keep = (t_acc >= lo) & (t_acc <= hi)
        if not keep.any():
            raise SystemExit(f"--start/--end window [{start}, {end}] selects no samples")
        self.t = t_acc[keep]

        self.acc = {
            "N": raw["AttAccNed0"][1][keep],
            "E": raw["AttAccNed1"][1][keep],
            "D": raw["AttAccNed2"][1][keep],
        }

        self.sig = {}
        for name, (t_src, x_src) in raw.items():
            if name.startswith("AttAccNed"):
                continue
            self.sig[name] = hold_at(t_src, x_src, self.t)

    def dt(self) -> np.ndarray:
        d = np.empty_like(self.t)
        d[0] = self.t[0]
        d[1:] = np.diff(self.t)
        return d


def build_command_stream(rec: Recording, cal: dict) -> str:
    out = io.StringIO()
    out.write("INIT\n")
    for name, value in cal.items():
        out.write(f"CAL {name} {value:.9g}\n")

    dt = rec.dt()
    gnss_present = rec.sig["GnssPresent"]
    gnss_navok = rec.sig["GnssNavOk"]
    baro_present = rec.sig["BaroPresent"]

    for i in range(len(rec.t)):
        if baro_present[i] != 0.0:
            alt = rec.sig["BaroAltitude"][i]
            if usable(alt, 1.0e6):
                out.write(f"BARO {alt:.9g}\n")

        if (gnss_present[i] != 0.0) and (gnss_navok[i] != 0.0):
            lat1e7 = int(round(rec.sig["GnssLatitude"][i] * 1.0e7))
            lon1e7 = int(round(rec.sig["GnssLongitude"][i] * 1.0e7))
            alt = rec.sig["GnssAltitude"][i]
            spd = rec.sig["GnssGroundSpeed"][i]
            hdg = rec.sig["GnssHeading"][i]
            hacc = rec.sig["GnssHAccuracy"][i]
            itow = int(rec.sig["NavGnssITow"][i]) & 0xFFFFFFFF
            if usable(alt, 1.0e6) and usable(spd) and usable(hdg) and usable(hacc, 1.0e6):
                out.write(f"GNSS {lat1e7} {lon1e7} {alt:.9g} {spd:.9g} {hdg:.9g} "
                          f"{hacc:.9g} {itow}\n")

        aN, aE, aD = rec.acc["N"][i], rec.acc["E"][i], rec.acc["D"][i]
        d = dt[i]
        cmd = "STEP"
        if not (usable(aN) and usable(aE) and usable(aD)
                and (FUSION_DT_MIN < d < FUSION_DT_MAX)):
            cmd = "STEPBAD"
        out.write(f"{cmd} {aN:.9g} {aE:.9g} {aD:.9g} {d:.9g}\n")

    return out.getvalue()


def run_gen_fusion_trace(exe: Path, commands: str) -> list[dict]:
    result = subprocess.run([str(exe)], input=commands, capture_output=True,
                             text=True, check=False)
    if result.returncode != 0:
        raise SystemExit(f"gen_fusion_trace exited {result.returncode}:\n{result.stderr}")
    reader = csv.DictReader(io.StringIO(result.stdout))
    return list(reader)


def sliding_p2p(x: np.ndarray, t: np.ndarray, window_s: float) -> float:
    """Max over every window of length window_s of (max - min) inside it.
    O(n) via monotonic deques -- n is a few thousand here, but the method
    is correct for any length, not just what happens to be fast today."""
    from collections import deque
    n = len(x)
    if n == 0:
        return float("nan")
    max_dq: deque[int] = deque()
    min_dq: deque[int] = deque()
    lo = 0
    worst = 0.0
    for hi in range(n):
        while max_dq and x[max_dq[-1]] <= x[hi]:
            max_dq.pop()
        max_dq.append(hi)
        while min_dq and x[min_dq[-1]] >= x[hi]:
            min_dq.pop()
        min_dq.append(hi)

        while t[hi] - t[lo] > window_s:
            if max_dq[0] == lo:
                max_dq.popleft()
            if min_dq[0] == lo:
                min_dq.popleft()
            lo += 1

        worst = max(worst, x[max_dq[0]] - x[min_dq[0]])
    return worst


def tangent_plane(lat_deg: np.ndarray, lon_deg: np.ndarray, ok: np.ndarray):
    """Same conversion as fusion.c fusion_correctGnss(): metres from the
    first usable fix on a flat tangent plane. Used only to compute the RAW
    receiver sigma for the 'never worse than raw' ratio (SWE1-FW-012 c)."""
    idx_ok = np.flatnonzero(ok)
    if idx_ok.size == 0:
        return None, None
    i0 = idx_ok[0]
    lat0, lon0 = lat_deg[i0], lon_deg[i0]
    m_per_deg_lat = 111132.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))
    north = (lat_deg - lat0) * m_per_deg_lat
    east = (lon_deg - lon0) * m_per_deg_lon
    return north[idx_ok], east[idx_ok]


def detect_steps(t: np.ndarray, x: np.ndarray, min_delta: float = 0.35,
                  half_win_s: float = 3.0) -> list[dict]:
    """Coarse step detector for the vertical-lift acceptance clause
    (SWE1-FW-013 a): compares the mean of x over a window ending at each
    sample against the mean over a window starting `half_win_s` later, and
    reports a step wherever that difference exceeds min_delta and is a
    local extremum of the difference series. Adjacent detections within one
    window are merged. This is a REPORTING aid, not a gate -- 'any residual
    disagreement against the logged NavPosDown over a lift is reported, not
    gated' (SWE1-FW-013 a)."""
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.1
    win = max(1, int(round(half_win_s / dt)))
    n = len(x)
    if n < 2 * win + 1:
        return []
    before = np.array([x[max(0, i - win):i + 1].mean() for i in range(n)])
    after = np.array([x[i:i + win + 1].mean() for i in range(n)])
    diff = after - before

    events = []
    i = win
    while i < n - win:
        if abs(diff[i]) >= min_delta:
            j = i
            while j < n - win and abs(diff[j]) >= min_delta:
                j += 1
            k = i + int(np.argmax(np.abs(diff[i:j])))
            events.append({
                "t_start": float(t[max(0, k - win)]),
                "t_end": float(t[min(n - 1, k + win)]),
                "delta": float(diff[k]),
            })
            i = j
        else:
            i += 1
    return events


def compute_metrics(rows: list[dict], rec: Recording, cal: dict) -> dict:
    col = {name: np.array([float(r[name]) for r in rows]) for name in CSV_COLUMNS}
    t = rec.t

    posD = col["d"]
    baroBias = col["baroBias"]
    posN = col["posN"]
    posE = col["posE"]
    pNN = col["pNN"]

    metrics: dict = {}
    metrics["n"] = len(rows)
    metrics["duration_s"] = float(t[-1] - t[0]) if len(t) > 1 else 0.0

    metrics["posD_p2p"] = float(posD.max() - posD.min())
    metrics["posD_max_per_60s"] = sliding_p2p(posD, t, WINDOW_60S)
    metrics["baroBias_p2p"] = float(baroBias.max() - baroBias.min())
    metrics["baroBias_max_per_60s"] = sliding_p2p(baroBias, t, WINDOW_60S)

    logged_posD = rec.sig["NavPosDown"]
    logged_baroBias = rec.sig["NavBaroBias"]
    metrics["posD_rms_vs_logged"] = float(np.sqrt(np.mean((posD - logged_posD) ** 2)))
    metrics["baroBias_rms_vs_logged"] = float(
        np.sqrt(np.mean((baroBias - logged_baroBias) ** 2)))
    logged_p2p = float(logged_posD.max() - logged_posD.min())
    metrics["posD_p2p_logged"] = logged_p2p
    metrics["posD_p2p_pct_of_logged"] = (
        100.0 * abs(metrics["posD_p2p"] - logged_p2p) / logged_p2p
        if logged_p2p != 0.0 else float("nan"))

    # NIS, north/east: var(innovation) / mean(P + R) over accepted fixes only
    # (gnssUpdates increments once per accepted N+E pair, fusion.c:965-968),
    # reconstructing R exactly as fusion_correctGnss does: R = hAcc^2 * rScale.
    r_scale = cal.get("gnssPosRScale", CAL_DEFAULTS["gnssPosRScale"])
    accepted = np.diff(col["gnssUpdates"], prepend=col["gnssUpdates"][0]) > 0
    n_accepted = int(accepted.sum())
    metrics["gnss_fixes_accepted"] = n_accepted
    if n_accepted >= 30:
        hAcc = np.maximum(rec.sig["GnssHAccuracy"][accepted], FUSION_GNSS_HACC_MIN)
        rPos = (hAcc ** 2) * r_scale
        # Only the north channel's variance is published (fusion.h pNN); the
        # east channel is structurally identical (same sigmaAccH, same R) so
        # this is the same approximation docs/NAV_TUNING.md section 5 makes
        # for its own NIS_east ("S = NavVarNorth + GnssHAccuracy^2 * rScale").
        s = pNN[accepted] + rPos
        metrics["NIS_north"] = float(np.var(col["innovN"][accepted]) / np.mean(s))
        metrics["NIS_east"] = float(np.var(col["innovE"][accepted]) / np.mean(s))
    else:
        metrics["NIS_north"] = None
        metrics["NIS_east"] = None
    if n_accepted < 1000:
        metrics["NIS_note"] = (f"only {n_accepted} accepted fixes (<1000): "
                                "NIS reported, not a pass/fail per SWE1-FW-012")
    else:
        metrics["NIS_note"] = None

    # 'std of each state' while the horizontal channel is anchored.
    # NavGnssTrusted (SWE1-FW-011) does not exist yet in this firmware
    # revision, so this is the pre-strand meaning ("anchored", not
    # "trusted") -- see (f2) note in the tool header / report.
    anchored = col["horizontalOk"] != 0
    if anchored.any():
        metrics["posN_std"] = float(np.std(posN[anchored], ddof=1))
        metrics["posE_std"] = float(np.std(posE[anchored], ddof=1))
    else:
        metrics["posN_std"] = None
        metrics["posE_std"] = None

    navok = (rec.sig["GnssPresent"] != 0) & (rec.sig["GnssNavOk"] != 0)
    raw_n, raw_e = tangent_plane(rec.sig["GnssLatitude"], rec.sig["GnssLongitude"], navok)
    if raw_n is not None and len(raw_n) > 1:
        metrics["raw_posN_std"] = float(np.std(raw_n, ddof=1))
        metrics["raw_posE_std"] = float(np.std(raw_e, ddof=1))
        if metrics["posN_std"] is not None:
            metrics["posN_ratio_to_raw"] = metrics["posN_std"] / metrics["raw_posN_std"]
            metrics["posE_ratio_to_raw"] = metrics["posE_std"] / metrics["raw_posE_std"]
    else:
        metrics["raw_posN_std"] = None
        metrics["raw_posE_std"] = None

    metrics["counters"] = {
        "baroRejects": int(col["rejects"][-1]),
        "baroResets": int(col["resets"][-1]),
        "gnssRejects": int(col["gnssRejects"][-1]),
        "gnssUpdates": int(col["gnssUpdates"][-1]),
        "covResets": int(col["covResets"][-1]),
        "verticalOk_final": int(col["verticalOk"][-1]),
        "horizontalOk_final": int(col["horizontalOk"][-1]),
        "originSet_final": int(col["originSet"][-1]),
    }

    metrics["lift_events"] = detect_steps(t, -posD)  # -posD: NED down -> up

    return metrics


def print_report(mf4_path: Path, cal: dict, metrics: dict) -> None:
    print(f"# nav_replay: {mf4_path}")
    print(f"# cal overrides: {cal if cal else '(none, compiled defaults)'}")
    print(f"n={metrics['n']}  duration={metrics['duration_s']:.1f} s")
    print()
    print(f"NavPosDown   p2p {metrics['posD_p2p']:.3f} m   "
          f"max/60s {metrics['posD_max_per_60s']:.3f} m   "
          f"RMS vs logged {metrics['posD_rms_vs_logged']:.4f} m   "
          f"p2p vs logged {metrics['posD_p2p_pct_of_logged']:.1f} %")
    print(f"NavBaroBias  p2p {metrics['baroBias_p2p']:.3f} m   "
          f"max/60s {metrics['baroBias_max_per_60s']:.3f} m   "
          f"RMS vs logged {metrics['baroBias_rms_vs_logged']:.4f} m")
    print()
    if metrics["NIS_north"] is not None:
        print(f"NIS north {metrics['NIS_north']:.3f}   NIS east {metrics['NIS_east']:.3f}"
              f"   ({metrics['gnss_fixes_accepted']} accepted fixes"
              f"{'  -- ' + metrics['NIS_note'] if metrics['NIS_note'] else ''})")
    else:
        print(f"NIS: not evaluated ({metrics['gnss_fixes_accepted']} accepted fixes)")
    print()
    if metrics["posN_std"] is not None:
        print(f"std(NavPosNorth) {metrics['posN_std']:.3f} m   "
              f"std(NavPosEast) {metrics['posE_std']:.3f} m   (post-anchor samples)")
        if metrics.get("raw_posN_std") is not None:
            print(f"raw std north {metrics['raw_posN_std']:.3f} m   "
                  f"raw std east {metrics['raw_posE_std']:.3f} m")
            print(f"ratio north {metrics['posN_ratio_to_raw']:.3f}x   "
                  f"ratio east {metrics['posE_ratio_to_raw']:.3f}x")
    else:
        print("std(NavPosNorth/East): not evaluated (never anchored, e.g. no GNSS fix)")
    print()
    c = metrics["counters"]
    print(f"counters: baroRejects={c['baroRejects']} baroResets={c['baroResets']} "
          f"gnssRejects={c['gnssRejects']} gnssUpdates={c['gnssUpdates']} "
          f"covResets={c['covResets']} verticalOk={c['verticalOk_final']} "
          f"horizontalOk={c['horizontalOk_final']} originSet={c['originSet_final']}")
    if metrics["lift_events"]:
        print()
        print("detected vertical steps (baro-driven, replayable per SWE1-FW-013 a):")
        for ev in metrics["lift_events"]:
            print(f"  t={ev['t_start']:.1f}-{ev['t_end']:.1f} s  delta={ev['delta']:+.3f} m")


def print_baseline_diff(baseline: dict, current: dict) -> None:
    print()
    print("# --baseline comparison (pre-change vs this run)")
    keys = ["posD_p2p", "posD_max_per_60s", "baroBias_p2p", "baroBias_max_per_60s",
            "NIS_north", "NIS_east", "posN_std", "posE_std"]
    for k in keys:
        b, c = baseline.get(k), current.get(k)
        if b is None or c is None:
            print(f"  {k}: baseline={b}  current={c}")
            continue
        print(f"  {k}: baseline={b:.4f}  current={c:.4f}  delta={c - b:+.4f}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mf4", type=Path, help="MF4 recording")
    ap.add_argument("--cal", action="append", default=[], metavar="name=value",
                     help="override one Xcp_FusionCal field before the run "
                          "(repeatable)")
    ap.add_argument("--baseline", type=Path, default=None,
                     help="a metrics JSON from a previous --dump-metrics run; "
                          "print a comparison table instead of/alongside "
                          "the plain report")
    ap.add_argument("--start", type=float, default=None, help="window start [s]")
    ap.add_argument("--end", type=float, default=None, help="window end [s]")
    ap.add_argument("--exe", type=Path, default=None,
                     help="path to gen_fusion_trace (default: auto-detect "
                          "under test/build*)")
    ap.add_argument("--dump-commands", type=Path, default=None,
                     help="write the generated command stream here")
    ap.add_argument("--dump-csv", type=Path, default=None,
                     help="write gen_fusion_trace's raw CSV output here")
    ap.add_argument("--dump-metrics", type=Path, default=None,
                     help="write the computed metrics as JSON here")
    ap.add_argument("--quiet", action="store_true",
                     help="suppress the human-readable report (for scripted use)")
    args = ap.parse_args()

    if not args.mf4.is_file():
        sys.exit(f"no such file: {args.mf4}")

    cal: dict = {}
    for item in args.cal:
        if "=" not in item:
            sys.exit(f"--cal expects name=value, got '{item}'")
        name, value = item.split("=", 1)
        if name not in CAL_FIELDS:
            sys.exit(f"unknown --cal field '{name}'; known: {sorted(CAL_FIELDS)}")
        cal[name] = float(value)

    exe = args.exe or default_exe()
    if exe is None or not exe.is_file():
        sys.exit("gen_fusion_trace not found -- build test/ first "
                  "(cmake --build test/build) or pass --exe")

    rec = Recording(args.mf4, args.start, args.end)
    commands = build_command_stream(rec, cal)

    if args.dump_commands:
        args.dump_commands.write_text(commands, encoding="utf-8")

    rows = run_gen_fusion_trace(exe, commands)

    if args.dump_csv:
        with args.dump_csv.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=CSV_COLUMNS)
            w.writeheader()
            w.writerows(rows)

    metrics = compute_metrics(rows, rec, cal)

    if args.dump_metrics:
        args.dump_metrics.write_text(json.dumps(metrics, indent=2, sort_keys=True),
                                      encoding="utf-8")

    if not args.quiet:
        print_report(args.mf4, cal, metrics)

    if args.baseline:
        baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
        print_baseline_diff(baseline, metrics)

    return 0


if __name__ == "__main__":
    sys.exit(main())
