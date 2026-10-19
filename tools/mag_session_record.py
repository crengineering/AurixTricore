#!/usr/bin/env python3
"""Record one hand-held magnetometer session over XCP: raw mag + raw IMU.

One recording serves both offline steps of a mag (re)mount:

  * the calibration TUMBLE -> tools/mag_cal.py --load <file>_tumble.csv
    (ellipsoid fit, hard/soft iron; writing to NVM stays the user's step)
  * the SIX-POSITION holds -> tools/mag_mount_search.py <file>.csv
    (mount table from the field-to-gravity angle)

Why a separate recorder: mag_cal.py starts collecting the instant it
connects, and the operator cannot see the PC while handling the board on a
tether a metre away. This tool connects, then WAITS armed until the board
moves, so the operator starts on their own cue; it stops by itself after the
board has been set down and left still, or at --max seconds. No writes.

Columns (xcp_read-style polled CSV, sensor frames as the drivers deliver
them, 20 Hz): t_host, magX_G magY_G magZ_G, imuAccX_g imuAccY_g imuAccZ_g,
imuGyroX_dps imuGyroY_dps imuGyroZ_dps. The IMU values are the 100 ms
housekeeping copies in Xcp_Data (Measurements.h), good enough for window
means and a stillness detector, not for rate integration.

Usage
-----
    python tools/mag_session_record.py OUT.csv            # arm, record, stop
    python tools/mag_session_record.py OUT.csv --max 480 --still 25
    python tools/mag_session_record.py OUT.csv --no-arm   # start at once

A one-line status goes to stdout every second (face up, |a|, |gyro|, still
time) so a watcher can follow what the hands are doing. Also writes
OUT_tumble.csv (mag_cal --load format: magX,magY,magZ) covering the first
continuous motion segment -- the tumble -- when one is found.
"""
from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
import time
from pathlib import Path

from pyxcp.master import Master
from pyxcp.config import create_application_from_config

HOST, PORT = "192.168.0.10", 5555

# Xcp_Data (Measurements.h): accelX..gyroZ are six consecutive float32 from
# 0x38; magX..magZ three float32 from 0x90. Fixed layout, same as mag_cal.py.
XCP_DATA = 0x70030000
OFF_ACCEL = 0x38
OFF_MAG = 0x90


def read_sample(x):
    imu = bytes(x.shortUpload(24, XCP_DATA + OFF_ACCEL, 0))
    mag = bytes(x.shortUpload(12, XCP_DATA + OFF_MAG, 0))
    acc = struct.unpack("<fff", imu[:12])
    gyr = struct.unpack("<fff", imu[12:24])
    m = struct.unpack("<fff", mag[:12])
    return m, acc, gyr


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def face_up(acc):
    """Which SENSOR axis points up (against gravity), for the status line."""
    k = max(range(3), key=lambda i: abs(acc[i]))
    return ("+" if acc[k] > 0 else "-") + "XYZ"[k] + " up"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", type=Path)
    ap.add_argument("--rate", type=float, default=0.05, help="sample interval s (default 0.05)")
    ap.add_argument("--max", type=float, default=480.0, help="hard stop after this many recorded seconds")
    ap.add_argument("--still", type=float, default=25.0, help="stop after the board has been still this long (after --min)")
    ap.add_argument("--min", type=float, default=120.0, help="no auto-stop before this many recorded seconds")
    ap.add_argument("--move-thr", type=float, default=15.0, help="|gyro| dps that counts as moving (arm + stillness)")
    ap.add_argument("--no-arm", action="store_true", help="record from connect, do not wait for motion")
    args = ap.parse_args()

    conf = create_application_from_config({
        "Transport": {"Eth": {"host": HOST, "port": PORT, "protocol": "UDP", "ipv6": False}},
    })
    cols = ["t_host", "magX_G", "magY_G", "magZ_G", "imuAccX_g", "imuAccY_g", "imuAccZ_g",
            "imuGyroX_dps", "imuGyroY_dps", "imuGyroZ_dps"]
    rows = []
    with Master("eth", config=conf) as x:
        x.connect()
        m, a, g = read_sample(x)
        if norm(m) == 0.0:
            print("!! mag reads all zero -- sensor absent (magPresent 0)? aborting")
            return 2
        print(f"connected; first sample mag {m[0]:+.3f} {m[1]:+.3f} {m[2]:+.3f} G |B| {norm(m):.3f}, "
              f"acc |a| {norm(a):.3f} g ({face_up(a)})", flush=True)
        armed = not args.no_arm
        if armed:
            print("ARMED -- waiting for the board to move (|gyro| > %.0f dps)" % args.move_thr, flush=True)
            while True:
                m, a, g = read_sample(x)
                if norm(g) > args.move_thr:
                    break
                time.sleep(args.rate)
            print("motion -> RECORDING", flush=True)
        t0 = time.time()
        last_status = 0.0
        still_since = None
        moving_seen = False
        try:
            while True:
                m, a, g = read_sample(x)
                t = time.time() - t0
                rows.append([f"{t:.3f}"] + [f"{v:.6f}" for v in (*m, *a, *g)])
                gn = norm(g)
                if gn > args.move_thr:
                    still_since = None
                    moving_seen = True
                elif still_since is None:
                    still_since = t
                still_for = 0.0 if still_since is None else t - still_since
                if t - last_status >= 1.0:
                    last_status = t
                    print(f"{t:6.1f} s  {face_up(a):<6} |a| {norm(a):.3f} g  |gyro| {gn:6.1f} dps  "
                          f"|B|raw {norm(m):.3f} G  still {still_for:4.0f} s  n={len(rows)}", flush=True)
                if t >= args.max:
                    print("max time reached -> STOP", flush=True)
                    break
                if t >= args.min and moving_seen and still_for >= args.still:
                    print(f"still for {args.still:.0f} s -> STOP", flush=True)
                    break
                time.sleep(args.rate)
        except KeyboardInterrupt:
            print("interrupted -> STOP", flush=True)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cols)
        w.writerows(rows)
    print(f"{len(rows)} samples, {float(rows[-1][0]):.1f} s -> {args.out}")

    # Tumble segment = from the first sample until the first stillness of >= 4 s
    # after at least 30 s (the six-position holds are each much longer than 4 s
    # of stillness; the tumble has none). Written in mag_cal.py --load format.
    thr = args.move_thr
    t = [float(r[0]) for r in rows]
    gn = [norm(tuple(float(v) for v in r[7:10])) for r in rows]
    end = None
    quiet_start = None
    for i in range(len(rows)):
        if gn[i] > thr:
            quiet_start = None
        elif quiet_start is None:
            quiet_start = i
        elif t[i] - t[quiet_start] >= 4.0 and t[quiet_start] >= 30.0:
            end = quiet_start
            break
    if end is not None and end > 200:
        tp = args.out.with_name(args.out.stem + "_tumble.csv")
        with tp.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["magX", "magY", "magZ"])
            w.writerows(r[1:4] for r in rows[:end])
        print(f"tumble segment 0-{t[end]:.1f} s ({end} samples) -> {tp}")
    else:
        print("no tumble segment identified (no >= 4 s stillness after 30 s, or too short)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
