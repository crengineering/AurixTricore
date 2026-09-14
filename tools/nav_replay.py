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
    python tools/nav_replay.py <file.csv> --start 770.334   # bench recorder
                                                             # CSV (task 3b);
                                                             # t_rel is since
                                                             # the recorder
                                                             # armed, not
                                                             # since boot --
                                                             # use --start to
                                                             # select the
                                                             # post-reconnect
                                                             # segment
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
    "gnssAltSlewMps", "gnssHAccMax",
    "lockGyroDps", "lockAccG", "relGyroDps", "relAccG", "lockWindowS",
    "sigmaZupt", "tauGnssBiasS", "gnssBiasRateMax",
}

# Compiled defaults (FusionCal.c) -- used to reconstruct R when a --cal
# override was not given for that field, so NIS can be computed for the
# values the run actually used, sweep or not.
#
# gnssPosRScale: 1.0 since SWE1-FW-012 task 9 (FusionCal.c, 2026-09-14) baked
# the task-3 sweep recommendation in as the compiled default -- was 8.0
# before that commit. A replay against a firmware build predating task 9
# would need --cal gnssPosRScale=8 to reconstruct the right R; every
# recording on file postdates the estimator, not the firmware default, so
# this is not a concern for the replays themselves, only for this constant.
CAL_DEFAULTS = {
    "gnssPosRScale": 1.0,
    "sigmaGnssVel": 0.3,
    "sigmaBaro": 0.0197,
}

# SWE1-FW-014: there is no dedicated "disable the lock" cal field -- the
# design deliberately did not add one (docs/NAV_STRAND_2026-09.md section
# 10 discusses only the airborne interlock, never a bench override). Setting
# lockWindowS far past any replay's duration means s_lockGoodS can never
# reach it, so the lock never engages, without needing a new field: every
# other lock/ZUPT/release mechanism is gated on stationaryLocked, which then
# simply never becomes TRUE. Used for FW-012's t1 evidence, which otherwise
# fuses zero GNSS fixes once locked (t1 is a rest recording).
CAL_LOCK_DISABLED = {"lockWindowS": 1.0e6}

FUSION_GNSS_HACC_MIN = 1.0    # fusion.c:96
WINDOW_60S = 60.0

# SWE1-FW-014 compiled defaults (docs/NAV_STRAND_2026-09.md section 10.1) --
# duplicated here (not imported) because task 13 makes NO src/ change and
# must run against recordings from before the detector exists in fusion.c;
# task 14 wires the SAME numbers into Xcp_FusionCal from 0x40, so a mismatch
# between this dict and FusionCal.c is a thing to notice, not silently drift
# on -- test/ref/lock_timeline_report.py's docstring says so too.
DETECTOR_DEFAULTS = {
    "lockGyroDps": 2.0,
    "lockAccG": 0.03,
    "relGyroDps": 3.0,
    "relAccG": 0.05,
    "lockWindowS": 1.0,
}
WINDOW_1S = 1.0

CSV_COLUMNS = [
    "step", "d", "vd", "accBiasD", "baroBias", "innov", "p00", "aD",
    "posN", "posE", "velN", "velE", "accBiasN", "accBiasE", "innovN",
    "innovE", "pNN", "aN", "aE",
    "rejects", "resets", "gnssRejects", "gnssUpdates", "covResets",
    "verticalOk", "horizontalOk", "originSet", "gnssTrusted", "stationaryLocked",
    "gnssBiasN", "gnssBiasE",  # SWE1-FW-015 (d), review round 2
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

    # SWE1-FW-014/-015/-016: the stationary-lock detector inputs. OPTIONAL,
    # not NEEDED -- several recordings this tool must still replay (the
    # from-boot CSV, task 3b) predate them entirely, and a recording missing
    # them simply cannot exercise the lock, which build_command_stream()
    # falls back on (a legacy 4-field STEP, "definitely moving" per
    # gen_fusion_trace.c's own default) rather than refusing the whole file.
    OPTIONAL = ["AttRate0", "AttRate1", "AttRate2", "AttAccMagnitude"]

    def __init__(self, path: Path, start: Optional[float], end: Optional[float]):
        if path.suffix.lower() == ".csv":
            raw, self.hasLockInputs = self._load_csv(path)
        else:
            raw, self.hasLockInputs = self._load_mf4(path)

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

    @classmethod
    def _load_mf4(cls, path: Path):
        raw = {}
        with MDF(path) as mdf:
            available = set(mdf.channels_db.keys())
            missing = [name for name in cls.NEEDED if name not in available]
            if missing:
                raise SystemExit(
                    f"{path.name}: not replayable -- missing channel(s) "
                    f"{missing}. This recording predates one or more of "
                    "AttAccNed{0,1,2} (needed to drive STEP) and the Nav* "
                    "estimator outputs (needed as ground truth); "
                    "nav_replay.py cannot reconstruct either from raw "
                    "IMU/GNSS alone (that IS the estimator). Not faked -- "
                    "report this recording as not replayable.")
            has_lock_inputs = all(name in available for name in cls.OPTIONAL)
            names = cls.NEEDED + (cls.OPTIONAL if has_lock_inputs else [])
            for name in names:
                sig = mdf.get(name)
                raw[name] = (sig.timestamps.astype(np.float64),
                             sig.samples.astype(np.float64))
        return raw, has_lock_inputs

    @classmethod
    def _load_csv(cls, path: Path) -> dict:
        """Minimal CSV front end (SWE1-FW-013 task 3b): a from-boot bench
        recording made by a plain XCP reconnect-loop poller has no AHRS/
        AttAccNed timebase alignment work to do -- every column is read on
        one shared clock already -- so this is a thin format swap, not a
        second data model. The CSV must carry a `t_rel` column (seconds
        since the recorder armed, NOT since boot -- use --start/--end to
        select the from-boot segment, e.g. after a reconnect) plus one
        column per name in NEEDED, named exactly like the MF4 channels."""
        with path.open(newline="", encoding="utf-8") as fh:
            reader = csv.DictReader(fh)
            fields = reader.fieldnames or []
            if "t_rel" not in fields:
                raise SystemExit(f"{path.name}: no 't_rel' column -- not a "
                                  "recognised from-boot recorder CSV")
            missing = [name for name in cls.NEEDED if name not in fields]
            if missing:
                raise SystemExit(
                    f"{path.name}: not replayable -- missing column(s) "
                    f"{missing}. Not faked -- report this recording as not "
                    "replayable.")
            has_lock_inputs = all(name in fields for name in cls.OPTIONAL)
            names = cls.NEEDED + (cls.OPTIONAL if has_lock_inputs else [])
            t: list[float] = []
            cols: dict[str, list[float]] = {name: [] for name in names}
            for row in reader:
                t.append(float(row["t_rel"]))
                for name in names:
                    cols[name].append(float(row[name]))
        t_arr = np.array(t, dtype=np.float64)
        return ({name: (t_arr, np.array(vals, dtype=np.float64))
                for name, vals in cols.items()}, has_lock_inputs)

    def dt(self) -> np.ndarray:
        d = np.empty_like(self.t)
        d[0] = self.t[0]
        d[1:] = np.diff(self.t)
        return d


def build_command_stream(rec: Recording, cal: dict, on_ground: bool = True,
                         force_release_at: Optional[float] = None) -> str:
    out = io.StringIO()
    out.write("INIT\n")
    for name, value in cal.items():
        out.write(f"CAL {name} {value:.9g}\n")
    if rec.hasLockInputs:
        # SWE1-FW-014: every recording this branch can run on is a bench/
        # outdoor-vehicle-stationary-or-hand-motion capture, never armed --
        # ONGROUND 1 unless the caller explicitly says otherwise (there is
        # no recording on file where it should be anything else).
        out.write(f"ONGROUND {1 if on_ground else 0}\n")

    # SWE1-FW-015 clause (d): a synthetic release on a recording where the
    # IMU itself never releases (t1, at rest throughout). ONGROUND 0 at
    # force_release_at and left there for the rest of the run -- simulating
    # an actual liftoff (stays "airborne"), not a momentary toggle: a
    # toggle-and-immediately-reopen on a genuinely-at-rest recording just
    # re-locks within one lockWindowS (the IMU never stopped agreeing), which
    # gives the bias no time to transfer to GNSS truth and does not exercise
    # what clause (d) is asking about. Fusion_setOnGround(FALSE) only RAISES
    # the pending-release flag (BLOCKER 1 fix); the very next Fusion_update()
    # (the STEP right after) performs the release, through the same
    # fusion_releaseGnssBias() path an IMU-detected release uses.
    released = force_release_at is None

    dt = rec.dt()
    gnss_present = rec.sig["GnssPresent"]
    gnss_navok = rec.sig["GnssNavOk"]
    baro_present = rec.sig["BaroPresent"]

    for i in range(len(rec.t)):
        if (not released) and (rec.t[i] >= force_release_at):
            out.write("ONGROUND 0\n")   # stays 0 for the rest of the run
            released = True

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
        if rec.hasLockInputs:
            r0 = math.radians(rec.sig["AttRate0"][i])
            r1 = math.radians(rec.sig["AttRate1"][i])
            r2 = math.radians(rec.sig["AttRate2"][i])
            accMagG = rec.sig["AttAccMagnitude"][i]
            out.write(f"{cmd} {aN:.9g} {aE:.9g} {aD:.9g} {d:.9g} "
                      f"{r0:.9g} {r1:.9g} {r2:.9g} {accMagG:.9g}\n")
        else:
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


def sliding_max(x: np.ndarray, t: np.ndarray, window_s: float) -> np.ndarray:
    """max(x) over the trailing window_s ending at each sample -- the
    per-sample series sliding_p2p's deque already visits, kept instead of
    collapsed to one worst-of-run number so the SWE1-FW-014 threshold table
    (the worst 1 s window on file) can be read off directly."""
    from collections import deque
    n = len(x)
    out = np.empty(n)
    dq: deque[int] = deque()
    lo = 0
    for hi in range(n):
        while dq and x[dq[-1]] <= x[hi]:
            dq.pop()
        dq.append(hi)
        while t[hi] - t[lo] > window_s:
            if dq[0] == lo:
                dq.popleft()
            lo += 1
        out[hi] = x[dq[0]]
    return out


class DetectorRecording:
    """Just the two SWE1-FW-014 detector inputs, from whichever schema the
    recording happens to use -- deliberately independent of the Recording
    class above (which needs the full Nav/GNSS/Baro channel set task 13 does
    not touch and several of these recordings do not even have logged).

    omega  = hypot(rate0, rate1, rate2)   [deg/s]
    accDev = abs(accMagnitude - 1.0)      [g]
    """

    def __init__(self, path: Path, start: Optional[float], end: Optional[float]):
        if path.suffix.lower() == ".csv":
            t, r0, r1, r2, acc = self._load_csv(path)
        else:
            t, r0, r1, r2, acc = self._load_mf4(path)

        t, r0, r1, r2, acc = self._dedup(t, r0, r1, r2, acc)

        lo = -np.inf if start is None else start
        hi = np.inf if end is None else end
        keep = (t >= lo) & (t <= hi)
        if not keep.any():
            raise SystemExit(f"--start/--end window [{start}, {end}] selects no samples")
        self.t = t[keep]
        self.rate0 = r0[keep]      # [deg/s], AttRate convention
        self.rate1 = r1[keep]
        self.rate2 = r2[keep]
        self.accMagG = acc[keep]   # [g]
        self.omega = np.sqrt(self.rate0 ** 2 + self.rate1 ** 2 + self.rate2 ** 2)
        self.accDev = np.abs(self.accMagG - 1.0)

    def dt(self) -> np.ndarray:
        d = np.empty_like(self.t)
        d[0] = self.t[1] - self.t[0] if len(self.t) > 1 else 0.01
        d[1:] = np.diff(self.t)
        return d

    @staticmethod
    def _load_mf4(path: Path):
        needed = ["AttRate0", "AttRate1", "AttRate2", "AttAccMagnitude"]
        with MDF(path) as mdf:
            available = set(mdf.channels_db.keys())
            missing = [n for n in needed if n not in available]
            if missing:
                raise SystemExit(
                    f"{path.name}: not replayable for the detector -- missing "
                    f"channel(s) {missing}. Not faked -- report this recording "
                    "as not replayable for SWE1-FW-014.")
            sigs = {n: mdf.get(n) for n in needed}
        t = sigs["AttRate0"].timestamps.astype(np.float64)
        r0 = sigs["AttRate0"].samples.astype(np.float64)
        r1 = sigs["AttRate1"].samples.astype(np.float64)
        r2 = sigs["AttRate2"].samples.astype(np.float64)
        acc = sigs["AttAccMagnitude"].samples.astype(np.float64)
        return t, r0, r1, r2, acc

    @staticmethod
    def _load_csv(path: Path):
        with path.open(newline="", encoding="utf-8") as fh:
            reader = csv.DictReader(fh)
            fields = reader.fieldnames or []
            time_col = "t_host" if "t_host" in fields else (
                "t_rel" if "t_rel" in fields else None)
            need = {"rate0_dps", "rate1_dps", "rate2_dps", "accMagG"}
            if time_col is None or not need.issubset(fields):
                raise SystemExit(
                    f"{path.name}: not replayable for the detector -- needs a "
                    f"time column (t_host/t_rel) plus {sorted(need)}. Not "
                    "faked -- report this recording as not replayable for "
                    "SWE1-FW-014.")
            t, r0, r1, r2, acc = [], [], [], [], []
            for row in reader:
                t.append(float(row[time_col]))
                r0.append(float(row["rate0_dps"]))
                r1.append(float(row["rate1_dps"]))
                r2.append(float(row["rate2_dps"]))
                acc.append(float(row["accMagG"]))
        t = np.array(t, dtype=np.float64)
        r0, r1, r2 = (np.array(a, dtype=np.float64) for a in (r0, r1, r2))
        acc = np.array(acc, dtype=np.float64)
        return t, r0, r1, r2, acc

    @staticmethod
    def _dedup(t: np.ndarray, r0: np.ndarray, r1: np.ndarray, r2: np.ndarray,
              acc: np.ndarray):
        """Collapse a sample that is bit-identical to the one right before it.

        Measured on 2026-09-12_strandB_level_r3.csv: its bench poller logs at
        ~20 Hz against a ~10 Hz AHRS update, so EXACTLY half of every row is
        the previous AHRS output polled again, not a second independent tick
        -- confirmed by 1250/2500 consecutive-identical rows and a poll dt of
        ~0.05 s against a value that only changes every other row. Without
        this, that single corrupt sample (the 'two pitch outliers' of
        SWE1-FW-014's evidence, one AHRS tick logged twice) reads as two
        independent bad samples and trips the 2-consecutive-sample release
        test the design deliberately built to survive exactly one corrupt
        tick (SWE1-FW-009). This is the same latch idea as fusion.c's own
        gnssDupes/baroLastGen: a repeated value is not new evidence, whatever
        rate it was polled at."""
        if len(t) < 2:
            return t, r0, r1, r2, acc
        keep = np.empty(len(t), dtype=bool)
        keep[0] = True
        keep[1:] = ((r0[1:] != r0[:-1]) | (r1[1:] != r1[:-1])
                    | (r2[1:] != r2[:-1]) | (acc[1:] != acc[:-1]))
        return t[keep], r0[keep], r1[keep], r2[keep], acc[keep]


def lock_release_timeline(t: np.ndarray, omega: np.ndarray, acc_dev: np.ndarray,
                          cal: dict) -> dict:
    """The would-be SWE1-FW-014/-015 lock state at every sample, using the
    SAME two counters fusion.c's task-14 implementation uses (a bounded,
    rate-invariant elapsed-good-time accumulator for engaging the lock, a
    2-consecutive-sample counter for releasing it) rather than a sliding
    window buffer -- the window-buffer form is used only for the REPORTED
    1 s-window maxima below, never for the lock decision itself, exactly the
    split the design note draws between 'the detector' and 'the threshold
    table'.

    Interlock (Fusion_setOnGround): not modelled here -- every recording in
    this table is a bench/outdoor-vehicle-stationary or hand-motion capture
    with the vehicle never armed, i.e. onGround = TRUE throughout, which is
    also fusion.c's boot default. A recording where that is not true would
    need the flag as a channel, which none of the eight logs.
    """
    lock_gyro = cal.get("lockGyroDps", DETECTOR_DEFAULTS["lockGyroDps"])
    lock_acc = cal.get("lockAccG", DETECTOR_DEFAULTS["lockAccG"])
    rel_gyro = cal.get("relGyroDps", DETECTOR_DEFAULTS["relGyroDps"])
    rel_acc = cal.get("relAccG", DETECTOR_DEFAULTS["relAccG"])
    window_s = cal.get("lockWindowS", DETECTOR_DEFAULTS["lockWindowS"])

    n = len(t)
    locked = np.zeros(n, dtype=bool)
    dt = np.empty(n)
    dt[0] = 0.0
    dt[1:] = np.diff(t)

    good_s = 0.0        # time accumulated with BOTH lock thresholds satisfied
    bad_run = 0         # consecutive samples with EITHER release threshold exceeded
    is_locked = False

    for i in range(n):
        clean = (omega[i] <= lock_gyro) and (acc_dev[i] <= lock_acc)
        good_s = (good_s + dt[i]) if clean else 0.0

        violate = (omega[i] > rel_gyro) or (acc_dev[i] > rel_acc)
        bad_run = (bad_run + 1) if violate else 0

        if is_locked:
            if bad_run >= 2:
                is_locked = False
                good_s = 0.0
        else:
            if good_s >= window_s:
                is_locked = True

        locked[i] = is_locked

    transitions = []
    for i in range(1, n):
        if locked[i] != locked[i - 1]:
            transitions.append({"t": float(t[i]), "to": "locked" if locked[i] else "released"})

    return {
        "locked": locked,
        "pct_locked": float(100.0 * locked.sum() / n) if n else 0.0,
        "transitions": transitions,
        "engaged_at_s": float(transitions[0]["t"] - t[0]) if transitions
                        and transitions[0]["to"] == "locked" else (
                            0.0 if n and locked[0] else None),
    }


def detector_metrics(rec: DetectorRecording, cal: dict) -> dict:
    omega_1s = sliding_max(rec.omega, rec.t, WINDOW_1S)
    acc_1s = sliding_max(rec.accDev, rec.t, WINDOW_1S)
    timeline = lock_release_timeline(rec.t, rec.omega, rec.accDev, cal)
    return {
        "n": len(rec.t),
        "duration_s": float(rec.t[-1] - rec.t[0]) if len(rec.t) > 1 else 0.0,
        "omega_1s_window_max_dps": float(omega_1s.max()) if len(omega_1s) else float("nan"),
        "accDev_1s_window_max_g": float(acc_1s.max()) if len(acc_1s) else float("nan"),
        "pct_locked": timeline["pct_locked"],
        "engaged_at_s": timeline["engaged_at_s"],
        "n_transitions": len(timeline["transitions"]),
        "transitions": timeline["transitions"],
    }


def build_detector_command_stream(rec: DetectorRecording, cal: dict, on_ground: bool) -> str:
    """STEP-only command stream for the PRODUCTION fusion.c (gen_fusion_trace),
    carrying the SWE1-FW-014 detector inputs -- no BARO/GNSS at all, since
    none of these attitude-only recordings have them (and the detector does
    not need them: task 14 makes no behaviour change, so the vertical/
    horizontal channels free-integrating in open loop for the run's short
    duration is immaterial to whether stationaryLocked matches). Confirms the
    PRODUCTION code reproduces this module's own Python reference
    (lock_release_timeline), not a second, independent implementation of it."""
    out = io.StringIO()
    out.write("INIT\n")
    for name, value in cal.items():
        out.write(f"CAL {name} {value:.9g}\n")
    out.write(f"ONGROUND {1 if on_ground else 0}\n")

    dt = rec.dt()
    for i in range(len(rec.t)):
        r0 = math.radians(rec.rate0[i])
        r1 = math.radians(rec.rate1[i])
        r2 = math.radians(rec.rate2[i])
        out.write(f"STEP 0 0 0 {dt[i]:.9g} {r0:.9g} {r1:.9g} {r2:.9g} {rec.accMagG[i]:.9g}\n")
    return out.getvalue()


def check_production_matches_reference(rec: DetectorRecording, cal: dict,
                                       timeline: dict, exe: Path) -> dict:
    """Run the SAME sequence through the production fusion.c and diff its
    published stationaryLocked column against this module's own
    lock_release_timeline() reference, sample for sample."""
    commands = build_detector_command_stream(rec, cal, on_ground=True)
    rows = run_gen_fusion_trace(exe, commands)
    if len(rows) != len(rec.t):
        return {"ok": False, "reason": f"row count {len(rows)} != {len(rec.t)}"}
    c_locked = np.array([int(r["stationaryLocked"]) for r in rows], dtype=bool)
    py_locked = timeline["locked"]
    mismatch = np.flatnonzero(c_locked != py_locked)
    return {
        "ok": len(mismatch) == 0,
        "n_mismatch": int(len(mismatch)),
        "first_mismatch_t": float(rec.t[mismatch[0]]) if len(mismatch) else None,
        "first_mismatch_t_end": float(rec.t[mismatch[-1]]) if len(mismatch) else None,
    }


def print_detector_report(path: Path, cal: dict, metrics: dict) -> None:
    print(f"# nav_replay --detector: {path}")
    print(f"# cal overrides: {cal if cal else '(none, compiled defaults)'}")
    print(f"n={metrics['n']}  duration={metrics['duration_s']:.1f} s")
    print(f"omega 1s-window max   {metrics['omega_1s_window_max_dps']:.4f} deg/s")
    print(f"|a|-1g 1s-window max  {metrics['accDev_1s_window_max_g']:.4f} g")
    print(f"would-be lock: {metrics['pct_locked']:.2f} % of samples, "
          f"{metrics['n_transitions']} transition(s)")
    if metrics["engaged_at_s"] is not None:
        print(f"first engaged at t+{metrics['engaged_at_s']:.2f} s")
    for tr in metrics["transitions"]:
        print(f"  t={tr['t']:.2f} s -> {tr['to']}")


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

    # SWE1-FW-010 (c): the barometer path must not regress under the slew
    # clamp -- NIS_baro = var(innovation)/sigmaBaro^2 stays 0.5-2.0 (reported,
    # not a hard gate at the tool level -- the requirement states the band),
    # and std(NavVelDown) <= 0.05 m/s.
    sigma_baro = cal.get("sigmaBaro", CAL_DEFAULTS["sigmaBaro"])
    metrics["NIS_baro"] = float(np.var(col["innov"]) / (sigma_baro ** 2))
    metrics["velD_std"] = float(np.std(col["vd"], ddof=1))
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
        "gnssTrusted_final": int(col["gnssTrusted"][-1]),
        "stationaryLocked_final": int(col["stationaryLocked"][-1]),
    }
    # SWE1-FW-014 (a): fraction locked, and the horizontal velocity/position
    # p2p while locked -- the whole point of the mechanism.
    locked = col["stationaryLocked"] != 0
    metrics["stationaryLocked_pct"] = float(100.0 * locked.sum() / len(locked)) if len(locked) else 0.0
    if locked.any():
        vHoriz = np.sqrt((col["velN"][locked] ** 2) + (col["velE"][locked] ** 2))
        metrics["locked_velHoriz_max"] = float(vHoriz.max())
        metrics["locked_posN_p2p"] = float(posN[locked].max() - posN[locked].min())
        metrics["locked_posE_p2p"] = float(posE[locked].max() - posE[locked].min())
    else:
        metrics["locked_velHoriz_max"] = None
        metrics["locked_posN_p2p"] = None
        metrics["locked_posE_p2p"] = None
    # SWE1-FW-011 (a)/(b): the fraction of the run GNSS was trusted, and the
    # largest sample-to-sample jump in the (supposedly frozen) horizontal
    # position while untrusted.
    trusted = col["gnssTrusted"] != 0
    metrics["gnssTrusted_pct"] = float(100.0 * trusted.sum() / len(trusted)) if len(trusted) else 0.0
    untrusted = ~trusted
    if untrusted.sum() > 1:
        dN = np.abs(np.diff(posN[untrusted]))
        dE = np.abs(np.diff(posE[untrusted]))
        metrics["untrusted_posN_max_delta"] = float(dN.max()) if len(dN) else 0.0
        metrics["untrusted_posE_max_delta"] = float(dE.max()) if len(dE) else 0.0
    else:
        metrics["untrusted_posN_max_delta"] = None
        metrics["untrusted_posE_max_delta"] = None

    metrics["lift_events"] = detect_steps(t, -posD)  # -posD: NED down -> up

    return metrics


def compute_release_metrics(rows: list[dict], rec: Recording,
                             t_release: float) -> dict:
    """SWE1-FW-015 (d)/(d2), measured on the BIAS STATE, not distance to the
    raw fix (the original distance-based formulation is withdrawn -- see the
    item file). Independent of compute_metrics()'s own numbers; called only
    with --force-release-at.

    (d)'s four binding clauses:
      1. no sample-to-sample step in posN/posE above 0.02 m, from the
         release tick onward (same bound as (a), asserted through this exit
         specifically);
      2. |gnssBias| never increases after the release tick;
      3. |gnssBias| <= 0.368 * delta0 * 1.10 at 60 s after release (one tau)
         and <= 0.10 * delta0 at 180 s (three tau);
      4. the position rate implied by the decay never exceeds
         gnssBiasRateMax (0.05 m/s default).

    (d2) is reported only: the fused-vs-raw distance track, on fusion.c's own
    tangent-plane origin (the raw fix converted with the SAME first-usable-fix
    origin nav_replay.py's own raw-sigma comparison already uses elsewhere in
    this file -- fusion.c defines its origin identically, "the first usable
    GNSS fix", so the two coincide by construction, not by re-deriving
    fusion.c's internal origin over the CSV, which the CSV does not publish).
    """
    t = rec.t
    posN = np.array([float(r["posN"]) for r in rows])
    posE = np.array([float(r["posE"]) for r in rows])
    biasN = np.array([float(r["gnssBiasN"]) for r in rows])
    biasE = np.array([float(r["gnssBiasE"]) for r in rows])
    locked = np.array([int(r["stationaryLocked"]) for r in rows])

    idx_candidates = np.nonzero((t >= t_release) & (locked == 0))[0]
    if len(idx_candidates) == 0:
        return {"error": "lock never released at/after the forced release time"}
    i_rel = int(idx_candidates[0])
    t_rel = float(t[i_rel])

    bias_mag = np.sqrt((biasN ** 2) + (biasE ** 2))
    delta0 = float(bias_mag[i_rel])

    post_mag = bias_mag[i_rel:]
    post_t = t[i_rel:]

    d_mag = np.diff(post_mag)
    # float noise tolerance, not a real increase: the decay is a shrinking
    # exponential rate-limited toward zero, never away from it.
    never_increases = bool(np.all(d_mag <= 1.0e-6))
    max_increase = float(d_mag.max()) if len(d_mag) else 0.0

    def bias_near(dt_target: float):
        target = t_rel + dt_target
        if target > post_t[-1]:
            return None, None
        j = int(np.searchsorted(post_t, target))
        j = min(j, len(post_mag) - 1)
        return float(post_mag[j]), float(post_t[j] - t_rel)

    bias_60, t_60 = bias_near(60.0)
    bias_180, t_180 = bias_near(180.0)
    bound_60 = 0.368 * delta0 * 1.10
    bound_180 = 0.10 * delta0

    # Clause (1)/(a): the step AT THE RELEASE TICK itself (this tick vs the
    # preceding one) -- NOT the largest step anywhere in the rest of the run,
    # which would also catch ordinary GNSS-driven corrections that have
    # nothing to do with the release mechanism and are governed by their own
    # requirements (SWE1-FW-010/-011/-012), not this one.
    if i_rel > 0:
        step_n = float(abs(posN[i_rel] - posN[i_rel - 1]))
        step_e = float(abs(posE[i_rel] - posE[i_rel - 1]))
    else:
        step_n = 0.0
        step_e = 0.0
    max_step = max(step_n, step_e)

    dt_arr = np.diff(post_t)
    with np.errstate(invalid="ignore", divide="ignore"):
        rate = np.where(dt_arr > 0.0, np.abs(d_mag) / dt_arr, 0.0)
    max_rate = float(rate.max()) if len(rate) else 0.0

    result = {
        "t_release_requested": t_release,
        "t_release_actual": t_rel,
        "i_release": i_rel,
        "delta0_m": delta0,
        "clause1_max_step_m": max_step,
        "clause1_pass": bool(max_step <= 0.02),
        "clause2_never_increases": never_increases,
        "clause2_max_increase_m": max_increase,
        "clause3_bias_at_60s_m": bias_60,
        "clause3_t_at_60s_actual_s": t_60,
        "clause3_bound_60s_m": bound_60,
        "clause3_pass_60s": (bias_60 is not None) and (bias_60 <= bound_60),
        "clause3_bias_at_180s_m": bias_180,
        "clause3_t_at_180s_actual_s": t_180,
        "clause3_bound_180s_m": bound_180,
        "clause3_pass_180s": (bias_180 is not None) and (bias_180 <= bound_180),
        "clause4_max_implied_rate_mps": max_rate,
        "clause4_pass": bool(max_rate <= 0.05 + 1.0e-6),
    }

    # (d2): fused-vs-raw distance track, reported not gated.
    navok = (rec.sig["GnssPresent"] != 0) & (rec.sig["GnssNavOk"] != 0)
    raw_n, raw_e = tangent_plane(rec.sig["GnssLatitude"], rec.sig["GnssLongitude"], navok)
    track = []
    if raw_n is not None and len(raw_n) > 1:
        idx_ok = np.flatnonzero(navok)
        t_ok = t[idx_ok]
        for dt_target in (0.0, 10.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0):
            target = t_rel + dt_target
            if target > t[-1]:
                break
            j = int(np.searchsorted(t, target))
            j = min(j, len(t) - 1)
            rn = float(np.interp(t[j], t_ok, raw_n))
            re_ = float(np.interp(t[j], t_ok, raw_e))
            dist = float(math.hypot(posN[j] - rn, posE[j] - re_))
            track.append({
                "dt_after_release_s": dt_target,
                "t_s": float(t[j]),
                "fused_n_m": float(posN[j]), "fused_e_m": float(posE[j]),
                "raw_n_m": rn, "raw_e_m": re_,
                "distance_m": dist,
            })
    result["d2_fused_vs_raw_track"] = track

    return result


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
    print(f"NIS_baro {metrics['NIS_baro']:.3f}   std(NavVelDown) {metrics['velD_std']:.4f} m/s")
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
          f"horizontalOk={c['horizontalOk_final']} originSet={c['originSet_final']} "
          f"gnssTrusted={c['gnssTrusted_final']}")
    print(f"gnssTrusted {metrics['gnssTrusted_pct']:.2f} % of the run"
          + (f"; while untrusted, max|delta| N={metrics['untrusted_posN_max_delta']:.4f} m "
             f"E={metrics['untrusted_posE_max_delta']:.4f} m"
             if metrics["untrusted_posN_max_delta"] is not None else ""))
    print(f"stationaryLocked {metrics['stationaryLocked_pct']:.2f} % of the run"
          + (f"; while locked, |v_horiz| max {metrics['locked_velHoriz_max']:.4f} m/s, "
             f"posN p2p {metrics['locked_posN_p2p']:.4f} m, posE p2p {metrics['locked_posE_p2p']:.4f} m"
             if metrics["locked_velHoriz_max"] is not None else " (never locked)"))
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


def print_release_report(r: dict) -> None:
    print()
    print("# SWE1-FW-015 (d)/(d2): forced release, measured on the bias state")
    if "error" in r:
        print(f"  {r['error']}")
        return
    print(f"release requested t={r['t_release_requested']:.1f} s, "
          f"actual t={r['t_release_actual']:.3f} s (row {r['i_release']})")
    print(f"delta0 (|gnssBias| at release) = {r['delta0_m']:.4f} m")
    print(f"(1) max sample-to-sample posN/posE step from release onward: "
          f"{r['clause1_max_step_m']:.5f} m  <= 0.02 m: "
          f"{'PASS' if r['clause1_pass'] else 'FAIL'}")
    print(f"(2) |gnssBias| never increases after release: "
          f"{'PASS' if r['clause2_never_increases'] else 'FAIL'} "
          f"(max increase {r['clause2_max_increase_m']:.6f} m)")
    if r["clause3_bias_at_60s_m"] is not None:
        print(f"(3) |gnssBias| at +60s (actual +{r['clause3_t_at_60s_actual_s']:.1f}s) = "
              f"{r['clause3_bias_at_60s_m']:.4f} m  <= {r['clause3_bound_60s_m']:.4f} m: "
              f"{'PASS' if r['clause3_pass_60s'] else 'FAIL'}")
    else:
        print("(3) +60s point not reached inside the recording")
    if r["clause3_bias_at_180s_m"] is not None:
        print(f"    |gnssBias| at +180s (actual +{r['clause3_t_at_180s_actual_s']:.1f}s) = "
              f"{r['clause3_bias_at_180s_m']:.4f} m  <= {r['clause3_bound_180s_m']:.4f} m: "
              f"{'PASS' if r['clause3_pass_180s'] else 'FAIL'}")
    else:
        print("    +180s point not reached inside the recording")
    print(f"(4) max position rate implied by the decay: "
          f"{r['clause4_max_implied_rate_mps']:.5f} m/s  <= 0.05 m/s: "
          f"{'PASS' if r['clause4_pass'] else 'FAIL'}")
    if r["d2_fused_vs_raw_track"]:
        print()
        print("(d2) fused-vs-raw distance track (reported, not gated):")
        for pt in r["d2_fused_vs_raw_track"]:
            print(f"  t=rel+{pt['dt_after_release_s']:.1f}s (t={pt['t_s']:.1f}s): "
                  f"fused=({pt['fused_n_m']:.3f},{pt['fused_e_m']:.3f}) "
                  f"raw=({pt['raw_n_m']:.3f},{pt['raw_e_m']:.3f}) "
                  f"dist={pt['distance_m']:.3f} m")


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
    ap.add_argument("--force-release-at", type=float, default=None,
                     help="SWE1-FW-015 clause (d): force a synthetic "
                          "stationary-lock release at this timestamp [s] "
                          "via the airborne interlock (ONGROUND 0, held for "
                          "the rest of the run -- simulates an actual "
                          "liftoff, not a momentary toggle), for a "
                          "recording where the IMU itself never releases "
                          "(e.g. outdoor t1, at rest throughout). Needs the "
                          "recording's own lock inputs (AttRate0/1/2 + "
                          "AttAccMagnitude)")
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
    ap.add_argument("--detector", action="store_true",
                     help="SWE1-FW-014 task 13: print the stationary-detector "
                          "1 s-window maxima and the would-be lock/release "
                          "timeline instead of the fusion.c replay. No src/ "
                          "change and no gen_fusion_trace involved -- reads "
                          "AttRate0/1/2 + AttAccMagnitude (or the CSV "
                          "equivalent) directly, so it also runs on "
                          "recordings the full replay cannot (no Nav*/GNSS/ "
                          "baro channels logged)")
    ap.add_argument("--cal-detector", action="append", default=[],
                     metavar="name=value",
                     help="override one SWE1-FW-014 threshold before a "
                          "--detector run (lockGyroDps, lockAccG, "
                          "relGyroDps, relAccG, lockWindowS); repeatable")
    ap.add_argument("--production", action="store_true",
                     help="with --detector: also drive the PRODUCTION "
                          "fusion.c (gen_fusion_trace, STEP-only, ONGROUND 1) "
                          "with the same inputs and diff its published "
                          "stationaryLocked column against this module's own "
                          "lock_release_timeline() sample for sample")
    args = ap.parse_args()

    if not args.mf4.is_file():
        sys.exit(f"no such file: {args.mf4}")

    if args.detector:
        det_cal: dict = {}
        for item in args.cal_detector:
            if "=" not in item:
                sys.exit(f"--cal-detector expects name=value, got '{item}'")
            name, value = item.split("=", 1)
            if name not in DETECTOR_DEFAULTS:
                sys.exit(f"unknown --cal-detector field '{name}'; known: "
                          f"{sorted(DETECTOR_DEFAULTS)}")
            det_cal[name] = float(value)

        rec = DetectorRecording(args.mf4, args.start, args.end)
        metrics = detector_metrics(rec, det_cal)

        if args.dump_metrics:
            args.dump_metrics.write_text(
                json.dumps(metrics, indent=2, sort_keys=True), encoding="utf-8")

        if not args.quiet:
            print_detector_report(args.mf4, det_cal, metrics)

        if args.production:
            exe = args.exe or default_exe()
            if exe is None or not exe.is_file():
                sys.exit("gen_fusion_trace not found -- build test/ first "
                          "(cmake --build test/build) or pass --exe")
            timeline = lock_release_timeline(rec.t, rec.omega, rec.accDev, det_cal)
            check = check_production_matches_reference(rec, det_cal, timeline, exe)
            if not args.quiet:
                print()
                if check["ok"]:
                    print("production fusion.c stationaryLocked matches the "
                          "Python reference on every sample")
                else:
                    print(f"MISMATCH: {check['n_mismatch']} sample(s) differ, "
                          f"t={check['first_mismatch_t']:.2f}..{check['first_mismatch_t_end']:.2f} s")
            if args.dump_metrics:
                merged = dict(metrics)
                merged["production_check"] = check
                args.dump_metrics.write_text(
                    json.dumps(merged, indent=2, sort_keys=True), encoding="utf-8")
            return 0 if check["ok"] else 1

        return 0

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
    if (args.force_release_at is not None) and (not rec.hasLockInputs):
        sys.exit(f"{args.mf4.name}: --force-release-at needs AttRate0/1/2 + "
                  "AttAccMagnitude, which this recording does not have")
    commands = build_command_stream(rec, cal, force_release_at=args.force_release_at)

    if args.dump_commands:
        args.dump_commands.write_text(commands, encoding="utf-8")

    rows = run_gen_fusion_trace(exe, commands)

    if args.dump_csv:
        with args.dump_csv.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=CSV_COLUMNS)
            w.writeheader()
            w.writerows(rows)

    metrics = compute_metrics(rows, rec, cal)

    if args.force_release_at is not None:
        metrics["fw015d_release"] = compute_release_metrics(rows, rec, args.force_release_at)

    if args.dump_metrics:
        args.dump_metrics.write_text(json.dumps(metrics, indent=2, sort_keys=True),
                                      encoding="utf-8")

    if not args.quiet:
        print_report(args.mf4, cal, metrics)
        if args.force_release_at is not None:
            print_release_report(metrics["fw015d_release"])

    if args.baseline:
        baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
        print_baseline_diff(baseline, metrics)

    return 0


if __name__ == "__main__":
    sys.exit(main())
