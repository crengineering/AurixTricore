#!/usr/bin/env python3
"""Yaw-independent search for the magnetometer mount table (AHRS_MAG_MOUNT_*).

Why a search, and why yaw-independent
-------------------------------------
A heading sweep at level cannot validate a mag mount: it only checks the
rotation SENSE of the horizontal field, and several wrong tables preserve
that. SYS1-001 B9 (2026-09-13) found the firmware's table wrong by 180 deg at
90 deg roll after a "validated" level sweep. The check that does discriminate
is attitude-independent: the angle between the magnetic field and gravity is
a constant of the location (90 deg - dip, about 26 deg from DOWN in Munich).
Held still in several attitudes, the hard-iron-corrected field mapped through
a CANDIDATE mount must make that angle with gravity (from the accelerometer,
mapped through the KNOWN IMU mount) in every position. The one table out of
all 48 signed axis permutations with the smallest spread wins; a mirror of the
true table puts the field pointing UP, which the mean angle exposes.

This is the method behind Ahrs.c's AHRS_MAG_MOUNT_* comment, committed as a
tool so a remount (2026-10-19: sensor turned 180 deg about Z on the breakout)
is re-measured the same way instead of a predicted sign being trusted.

Inputs
------
One or more recordings of the SIX-POSITION protocol (level, roll +90, roll
-90, nose up, nose down, level; hold each 10-20 s; at least 1 m from the
bench, which is magnetically contaminated):

  * polled CSV (xcp_read style) with columns magX_G magY_G magZ_G and
    imuAccX_g imuAccY_g imuAccZ_g (+ imuGyro?_dps for automatic windows,
    t_host for time)
  * GUI MF4 with channels MagX MagY MagZ ImuAccelX/Y/Z (+ ImuGyroX/Y/Z)
  * window-means JSON as written by --export-means (or the 2026-09-12 file)

All channels are RAW SENSOR frame as delivered by the drivers. Hard iron is
subtracted in that frame before any mount, as Ahrs.c does -- pass the set
that was live when recording (--board-json or --hard-iron).

Usage
-----
    python tools/mag_mount_search.py REC.csv --board-json calibration/board.json
    python tools/mag_mount_search.py REC.mf4 --hard-iron -0.18 -0.13 -0.79
    python tools/mag_mount_search.py REC.csv --windows "level=5:20,rollp=30:45,..."
    python tools/mag_mount_search.py A.csv B.mf4 --expect "+sX,-sY,+sZ"
    python tools/mag_mount_search.py REC.csv ... --export-means means.json

--expect makes the exit code a verdict (0 = winner matches) so the tool can
double as a regression check; --export-means writes the window means in the
format test/test_ahrs.c's mount test is seeded from.
"""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import sys
from pathlib import Path

AXES = ("X", "Y", "Z")

# Ahrs.c AHRS_MOUNT_*: body forward <- sensor Y, body right <- sensor X,
# body down <- -sensor Z. The IMU mount is NOT under test here; it is the
# reference that defines "down". Override with --imu-mount if Ahrs.c changes.
IMU_MOUNT_DEFAULT = "+sY,+sX,-sZ"

# MF4 channel names (A2L) and polled-CSV column names, same order X Y Z.
MF4_MAG = ("MagX", "MagY", "MagZ")
MF4_ACC = ("ImuAccelX", "ImuAccelY", "ImuAccelZ")
MF4_GYR = ("ImuGyroX", "ImuGyroY", "ImuGyroZ")
CSV_MAG = ("magX_G", "magY_G", "magZ_G")
CSV_ACC = ("imuAccX_g", "imuAccY_g", "imuAccZ_g")
CSV_GYR = ("imuGyroX_dps", "imuGyroY_dps", "imuGyroZ_dps")
CSV_T = "t_host"


# --- mount tables ----------------------------------------------------------

def parse_mount(spec: str) -> tuple:
    """'+sX,-sY,+sZ' -> ((0,+1),(1,-1),(2,+1)): body axis k <- sgn * sensor src."""
    out = []
    for tok in spec.replace(" ", "").split(","):
        if len(tok) != 3 or tok[0] not in "+-" or tok[1] != "s" or tok[2] not in AXES:
            sys.exit(f"bad mount token '{tok}' in '{spec}' (want e.g. +sX,-sY,+sZ)")
        out.append((AXES.index(tok[2]), 1 if tok[0] == "+" else -1))
    if len(out) != 3 or len({s for s, _ in out}) != 3:
        sys.exit(f"mount '{spec}' must use each sensor axis exactly once")
    return tuple(out)


def fmt_mount(m) -> str:
    return ",".join(("+" if g > 0 else "-") + "s" + AXES[s] for s, g in m)


def apply_mount(m, v):
    return [g * v[s] for s, g in m]


def det(m) -> int:
    perm = [s for s, _ in m]
    sign = 1
    for i in range(3):
        for j in range(i + 1, 3):
            if perm[i] > perm[j]:
                sign = -sign
    for _, g in m:
        sign *= g
    return sign


def all_mounts():
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            yield tuple(zip(perm, signs))


def fw_defines(m) -> str:
    """The AHRS_MAG_MOUNT_* lines Ahrs.c needs for this table."""
    lines = []
    for k, (s, g) in enumerate(m):
        sgn = "( 1.0f)" if g > 0 else "(-1.0f)"
        lines.append(f"#define AHRS_MAG_MOUNT_{AXES[k]}_SRC  ({s}u)")
        lines.append(f"#define AHRS_MAG_MOUNT_{AXES[k]}_SGN  {sgn}")
    return "\n".join(lines)


# --- recordings ------------------------------------------------------------

def load_csv(path: Path):
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"{path}: empty")
    cols = rows[0].keys()
    for c in CSV_MAG + CSV_ACC:
        if c not in cols:
            sys.exit(f"{path}: missing column {c}")
    has_gyr = all(c in cols for c in CSV_GYR)
    t = [float(r[CSV_T]) if CSV_T in cols else i * 0.05 for i, r in enumerate(rows)]
    mag = [[float(r[c]) for c in CSV_MAG] for r in rows]
    acc = [[float(r[c]) for c in CSV_ACC] for r in rows]
    gyr = [[float(r[c]) for c in CSV_GYR] for r in rows] if has_gyr else None
    return t, mag, acc, gyr


def load_mf4(path: Path):
    try:
        from asammdf import MDF
        import numpy as np
    except ImportError:
        sys.exit("asammdf/numpy not installed:  pip install asammdf numpy")
    with MDF(path) as mdf:
        def sig(name):
            s = mdf.get(name)
            return s.timestamps.astype(float), s.samples.astype(float)

        tm, mx = sig(MF4_MAG[0])
        _, my = sig(MF4_MAG[1])
        _, mz = sig(MF4_MAG[2])
        chans = set(mdf.channels_db.keys())
        has_gyr = all(c in chans for c in MF4_GYR)

        def on_mag_base(name):
            # nearest-earlier sample of another channel on the mag time base
            ts, xs = sig(name)
            idx = np.clip(np.searchsorted(ts, tm, side="right") - 1, 0, len(ts) - 1)
            return xs[idx]

        ax, ay, az = (on_mag_base(c) for c in MF4_ACC)
        t = tm.tolist()
        mag = [[float(a), float(b), float(c)] for a, b, c in zip(mx, my, mz)]
        acc = [[float(a), float(b), float(c)] for a, b, c in zip(ax, ay, az)]
        gyr = None
        if has_gyr:
            gx, gy, gz = (on_mag_base(c) for c in MF4_GYR)
            gyr = [[float(a), float(b), float(c)] for a, b, c in zip(gx, gy, gz)]
    return t, mag, acc, gyr


def load_means_json(path: Path, hard_iron):
    d = json.loads(path.read_text())
    hi = hard_iron
    if hi is None:
        for k, v in d.items():
            if k.startswith("hard_iron_G"):
                hi = [float(x) for x in v]
    if hi is None:
        sys.exit(f"{path}: no hard_iron_G* key and no --hard-iron given")
    wins = [(w.get("pos", f"w{i}"), [float(x) for x in w["magRaw_G"]],
             [float(x) for x in w["accRaw_g"]]) for i, w in enumerate(d["windows"])]
    return wins, hi


# --- windows ---------------------------------------------------------------

def norm(v):
    return math.sqrt(sum(x * x for x in v))


def mean_vec(vs):
    n = len(vs)
    return [sum(v[k] for v in vs) / n for k in range(3)]


def parse_windows(spec: str):
    out = []
    for tok in spec.split(","):
        label, rng = tok.split("=") if "=" in tok else (f"w{len(out)}", tok)
        a, b = rng.split(":")
        out.append((label.strip(), float(a), float(b)))
    return out


def auto_windows(t, gyr, gyro_thr, min_len, settle):
    """Stationary segments: 1 s moving average of |gyro| < thr continuously
    for >= min_len s. Hand-held tilted holds are not quiet -- 2026-09-12 r3
    had a median |gyro| of 1.5-4.5 dps in the roll windows against 0.6 dps
    on the table -- so the raw norm is smoothed first. The first `settle`
    seconds of each segment are dropped (the hands are still settling and
    the field is still moving with them)."""
    if gyr is None:
        sys.exit("no gyro channels: give --windows explicitly")
    n = len(gyr)
    raw = [norm(g) for g in gyr]
    dt = (t[-1] - t[0]) / max(1, n - 1)
    half = max(1, int(round(0.5 / dt)))
    smooth = [sum(raw[max(0, i - half):min(n, i + half + 1)]) / len(raw[max(0, i - half):min(n, i + half + 1)])
              for i in range(n)]
    segs, start = [], None
    for i, gs in enumerate(smooth):
        still = gs < gyro_thr
        if still and start is None:
            start = i
        if start is not None and (not still or i == n - 1):
            end = i if not still else i + 1
            if t[end - 1] - t[start] >= min_len:
                segs.append((start, end))
            start = None
    out = []
    for k, (a, b) in enumerate(segs):
        a2 = a
        while a2 < b and t[a2] - t[a] < settle:
            a2 += 1
        if b - a2 < 5:
            continue
        out.append((f"w{k}", t[a2], t[b - 1]))
    return out


def window_means(t, mag, acc, windows, acc_tol):
    wins = []
    for label, a, b in windows:
        idx = [i for i, ti in enumerate(t) if a <= ti <= b]
        if len(idx) < 3:
            print(f"  window {label} {a:.1f}-{b:.1f} s: too few samples, skipped")
            continue
        if any(norm(mag[i]) == 0.0 for i in idx):
            print(f"  window {label}: all-zero mag samples (sensor absent), skipped")
            continue
        m = mean_vec([mag[i] for i in idx])
        g = mean_vec([acc[i] for i in idx])
        if abs(norm(g) - 1.0) > acc_tol:
            print(f"  window {label}: |acc| = {norm(g):.3f} g, not at rest, skipped")
            continue
        wins.append((label, m, g))
    return wins


# --- the search ------------------------------------------------------------

def angle_deg(a, b):
    c = sum(x * y for x, y in zip(a, b)) / (norm(a) * norm(b))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def search(wins, hard_iron, soft_iron, imu_mount, dip):
    """wins: list of (label, magRaw, accRaw) in SENSOR frames. Returns the
    ranked list of (spread, mean, mount, per-window angles), the hard/soft-
    iron corrected sensor-frame field per window, and the largest angle
    between any two gravity directions (attitude diversity).

    A table and its full negation (-P) give identical spreads: the negation
    turns every angle a into 180 - a. The spread cannot tell them apart, the
    MEAN can -- the field dips DOWN where the inclination is positive (Munich
    +64 deg), so a candidate whose mean lies on the wrong side of 90 deg is
    the mirror and is ranked last, behind every physically possible table."""
    corrected = [[(m[k] - hard_iron[k]) * soft_iron[k] for k in range(3)] for _, m, _ in wins]
    down = [[-x for x in apply_mount(imu_mount, g)] for _, _, g in wins]
    diversity = max(angle_deg(a, b) for a in down for b in down)
    results = []
    for mt in all_mounts():
        angs = [angle_deg(apply_mount(mt, c), d) for c, d in zip(corrected, down)]
        n = len(angs)
        mean = sum(angs) / n
        spread = math.sqrt(sum((a - mean) ** 2 for a in angs) / n)
        wrong_hemisphere = (mean > 90.0) if dip > 0.0 else (mean < 90.0)
        results.append((spread, mean, mt, angs, wrong_hemisphere))
    results.sort(key=lambda r: (r[4], r[0]))
    return results, corrected, diversity


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="+", type=Path, help=".csv (polled), .mf4 (GUI) or .json (window means)")
    ap.add_argument("--board-json", type=Path, help="calibration set live during the recording (hard_iron_G, soft_iron)")
    ap.add_argument("--hard-iron", nargs=3, type=float, metavar=("X", "Y", "Z"), help="hard-iron offsets [G], sensor frame")
    ap.add_argument("--soft-iron", nargs=3, type=float, metavar=("X", "Y", "Z"), default=None)
    ap.add_argument("--imu-mount", default=IMU_MOUNT_DEFAULT, help=f"IMU mount, Ahrs.c AHRS_MOUNT_* (default {IMU_MOUNT_DEFAULT})")
    ap.add_argument("--windows", help="explicit windows 'label=t0:t1,...' in recording seconds (applies to every recording)")
    ap.add_argument("--gyro-thr", type=float, default=8.0, help="auto windows: 1 s mean |gyro| < thr dps (default 8)")
    ap.add_argument("--min-len", type=float, default=5.0, help="auto windows: minimum still time s (default 5)")
    ap.add_argument("--settle", type=float, default=2.0, help="auto windows: seconds dropped at each window start (default 2)")
    ap.add_argument("--acc-tol", type=float, default=0.08, help="reject windows with | |acc| - 1 g | above this (default 0.08)")
    ap.add_argument("--dip", type=float, default=64.0, help="local magnetic inclination deg, + = down (Munich 64)")
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--all", action="store_true", help="print all 48")
    ap.add_argument("--expect", help="assert the winner is this table, e.g. '+sX,-sY,+sZ'; exit 1 otherwise")
    ap.add_argument("--export-means", type=Path, help="write the window means (sensor frames) as JSON")
    args = ap.parse_args()

    hard_iron = args.hard_iron
    soft_iron = args.soft_iron
    if args.board_json:
        bj = json.loads(args.board_json.read_text())
        if hard_iron is None:
            h = bj["hard_iron_G"]
            hard_iron = [float(h["x"]), float(h["y"]), float(h["z"])]
        if soft_iron is None and "soft_iron" in bj:
            s = bj["soft_iron"]
            soft_iron = [float(s["x"]), float(s["y"]), float(s["z"])]
    if soft_iron is None:
        soft_iron = [1.0, 1.0, 1.0]
    imu_mount = parse_mount(args.imu_mount)
    if det(imu_mount) != 1:
        print(f"warning: IMU mount {args.imu_mount} has det -1; Ahrs.h requires a proper rotation for the IMU")

    wins = []
    for rec in args.recordings:
        print(f"{rec}:")
        if rec.suffix.lower() == ".json":
            w, hi = load_means_json(rec, hard_iron)
            if hard_iron is None:
                hard_iron = hi
                print(f"  hard iron from file: {hard_iron}")
            for lab, m, g in w:
                wins.append((f"{rec.stem}:{lab}", m, g))
            print(f"  {len(w)} windows")
            continue
        if hard_iron is None:
            sys.exit("CSV/MF4 input needs --board-json or --hard-iron (the set live during the recording)")
        t, mag, acc, gyr = load_mf4(rec) if rec.suffix.lower() == ".mf4" else load_csv(rec)
        t0 = t[0]
        t = [x - t0 for x in t]
        if args.windows:
            windows = parse_windows(args.windows)
        else:
            windows = auto_windows(t, gyr, args.gyro_thr, args.min_len, args.settle)
        for lab, a, b in windows:
            print(f"  window {lab}: {a:.1f}-{b:.1f} s")
        w = window_means(t, mag, acc, windows, args.acc_tol)
        for lab, m, g in w:
            wins.append((f"{rec.stem}:{lab}", m, g))
        print(f"  {len(w)} usable windows of {len(windows)}")
    if len(wins) < 4:
        sys.exit(f"only {len(wins)} windows; the six-position protocol needs at least 4 distinct attitudes")

    results, corrected, diversity = search(wins, hard_iron, soft_iron, imu_mount, args.dip)
    expected_from_down = 90.0 - args.dip
    print(f"\nhard iron {hard_iron} G, soft iron {soft_iron}, IMU mount {fmt_mount(imu_mount)}")
    print(f"{len(wins)} windows, corrected |B| {min(norm(c) for c in corrected):.3f}-{max(norm(c) for c in corrected):.3f} G, "
          f"attitude diversity {diversity:.0f} deg (largest angle between two gravity directions)")
    if diversity < 60.0:
        print("!! attitudes too similar -- the six-position protocol needs roll AND pitch of ~90 deg; result is weak")
    print(f"expected angle(field, down) ~ {expected_from_down:.0f} deg (dip {args.dip:.0f}); "
          f"a mean on the wrong side of 90 deg is the mirror table and is ranked last\n")
    print(f"{'rank':>4}  {'mount (body <- sensor)':<24} {'det':>3}  {'spread':>7}  {'mean':>6}   per-window angles [deg]")
    shown = results if args.all else results[: args.top]
    for i, (spread, mean, mt, angs, wrong) in enumerate(shown, 1):
        flag = "  (mirror)" if wrong else ""
        print(f"{i:>4}  {fmt_mount(mt):<24} {det(mt):>+3}  {spread:7.2f}  {mean:6.1f}   "
              + " ".join(f"{a:5.1f}" for a in angs) + flag)

    best = results[0]
    runner = results[1]
    print("\nwinner:", fmt_mount(best[2]), f"(det {det(best[2]):+d}, spread {best[0]:.2f} deg, mean {best[1]:.1f} deg from down)")
    print(f"runner-up: {fmt_mount(runner[2])} spread {runner[0]:.2f} deg -> margin {runner[0] - best[0]:.2f} deg")
    if runner[0] - best[0] < 1.0:
        print("!! margin under 1 deg -- not decisive; more attitudes or a better hard-iron set needed")
    if abs(best[1] - expected_from_down) > 20.0:
        print(f"!! winner's mean is {abs(best[1] - expected_from_down):.0f} deg from the expected dip -- "
              "hard iron likely stale (calibrate first) or IMU mount wrong")
    print("\nAhrs.c:\n" + fw_defines(best[2]))

    if args.export_means:
        out = {
            "source": ", ".join(str(r) for r in args.recordings) + " window means; sensor frames as delivered by the drivers",
            "hard_iron_G": hard_iron,
            "soft_iron": soft_iron,
            "imu_body_mapping": f"body=({fmt_mount(imu_mount)})",
            "winner_mag_mapping": f"body=({fmt_mount(best[2])})",
            "winner_spread_deg": round(best[0], 3),
            "winner_mean_deg_from_down": round(best[1], 2),
            "windows": [{"pos": lab, "magRaw_G": [round(x, 5) for x in m], "accRaw_g": [round(x, 5) for x in g]}
                        for lab, m, g in wins],
        }
        args.export_means.write_text(json.dumps(out, indent=1))
        print(f"\nwindow means written to {args.export_means}")

    if args.expect:
        exp = parse_mount(args.expect)
        ok = exp == best[2]
        print(f"\nexpect {fmt_mount(exp)}: {'MATCH' if ok else 'MISMATCH'}")
        return 0 if ok else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
