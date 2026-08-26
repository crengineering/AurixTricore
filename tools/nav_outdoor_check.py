#!/usr/bin/env python3
"""Outdoor validation of the horizontal navigation channel.

Nothing in the horizontal channel, and nothing in the GNSS-altitude path that
makes the barometer bias observable, executes without a GNSS fix -- and there
is no fix indoors. This script is the trip outside.

It does three things:

  1. waits for a usable fix, reporting numSats / fixType / navOk while it waits,
     so a receiver that is merely cold is distinguishable from one that is broken
  2. watches for the tangent-plane origin being latched (originSet -> 1)
  3. records the state and, at the end, checks the ONE thing most likely to be
     silently wrong

THE CHECK THAT MATTERS: gnssUpdates must keep incrementing, and gnssITow must
keep advancing with it.

The receiver is configured for a 100 ms measurement rate with one NAV-PVT per
epoch (CFG_RATE_MEAS / CFG_RATE_NAV in GnssM9N.c), so solutions arrive at 10 Hz
-- the SAME rate Task_Measure100ms polls at, on an independent clock.

That means the update RATE alone proves nothing: a working iTOW guard and a
removed one both give about 10 updates/s. What the rate does catch is the real
failure, which is iTOW not being decoded at all. Then the first fix is accepted,
every later one looks like a duplicate, and gnssUpdates FREEZES while the
position quietly goes stale. So: updates must keep climbing, gnssITow must keep
advancing, and gnssDupes must be small but non-zero (exactly zero over a long
run means iTOW is changing on every single poll, which at equal rates it should
not).

Usage
-----
    python tools/nav_outdoor_check.py                 # wait for fix, then 120 s
    python tools/nav_outdoor_check.py --seconds 300
    python tools/nav_outdoor_check.py --csv walk.csv

Suggested trip: stand still for the first 60 s (posN/posE should stay put and
velN/velE should sit near zero), then walk a rectangle back to the start and
check the track closes on itself.
"""
from __future__ import annotations

import argparse
import csv
import struct
import sys
import time

from pyxcp.master import Master
from pyxcp.config import create_application_from_config

HOST, PORT = "192.168.0.10", 5555

# Xcp_Data -- GNSS receiver state (see Measurements.h)
D_NAV_OK = 0x700300AF
D_FIX_TYPE = 0x700300BA
D_NUM_SATS = 0x700300BB
D_HACC = 0x700300D0

# Xcp_Fusion @ 0x70030500 (see Measurements.h)
F = 0x70030500
F_POS_D, F_VEL_D = F + 0x54, F + 0x58
F_BARO_BIAS = F + 0x60
F_POS_N, F_POS_E = F + 0x6C, F + 0x70
F_VEL_N, F_VEL_E = F + 0x74, F + 0x78
F_INNOV_N, F_INNOV_E = F + 0x84, F + 0x88
F_VAR_N = F + 0x8C
F_GNSS_REJ, F_GNSS_UPD = F + 0xA4, F + 0xA8
F_ORIGIN_SET = F + 0xAE
F_COV_RESETS = F + 0xB0
F_GNSS_ITOW = F + 0xB4
F_GNSS_DUPES = F + 0xB8


def u8(x, a):
    return bytes(x.shortUpload(1, a, 0))[0]


def u32(x, a):
    return struct.unpack("<I", bytes(x.shortUpload(4, a, 0))[:4])[0]


def f32(x, a):
    return struct.unpack("<f", bytes(x.shortUpload(4, a, 0))[:4])[0]


UNITS = {
    "posN": "m", "posE": "m", "posD": "m",
    "velN": "m/s", "velE": "m/s", "velD": "m/s",
    "innovN": "m", "innovE": "m",
    "varN": "m^2", "baroBias": "m",
    "upd": "", "rej": "", "cov": "", "itow": "ms", "dupes": "",
}


def write_mf4(path, rows):
    """Export the record as ASAM MDF4.

    Deliberately done at the END, from rows that are already safely on disk in
    the CSV. Writing MDF incrementally would put a heavier dependency in the
    sampling loop, and a failure there would cost the walk -- which is the one
    thing that cannot be repeated cheaply.
    """
    try:
        import numpy as np
        from asammdf import MDF, Signal
    except ImportError:
        print("asammdf/numpy not installed -- skipping MF4 "
              "(pip install asammdf); the CSV is unaffected.", file=sys.stderr)
        return False

    t = np.array([r["t"] for r in rows], dtype=float)
    sigs = []
    for name in rows[0]:
        if name == "t":
            continue
        sigs.append(Signal(
            samples=np.array([r[name] for r in rows], dtype=float),
            timestamps=t, name=name, unit=UNITS.get(name, "")))

    with MDF(version="4.10") as mdf:
        mdf.append(sigs, comment="AurixTricore navigation filter, outdoor run")
        mdf.save(path, overwrite=True)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=120.0,
                    help="recording time once the origin is set")
    ap.add_argument("--wait", type=float, default=300.0,
                    help="how long to wait for a usable fix")
    ap.add_argument("--rate", type=float, default=0.2, help="sample interval [s]")
    ap.add_argument("--csv", default="nav_walk.csv",
                    help="record destination; written INCREMENTALLY (always on)")
    ap.add_argument("--mf4", default=None,
                    help="also write an ASAM MDF4 file, for tools/mf4_stats.py "
                         "and any MDF viewer. Written at the END from the rows "
                         "already on disk, so it can never cost the walk.")
    args = ap.parse_args()

    conf = create_application_from_config({
        "Transport": {"Eth": {"host": HOST, "port": PORT,
                              "protocol": "UDP", "ipv6": False}},
    })

    with Master("eth", config=conf) as x:
        x.connect()

        # --- 1. wait for a fix ------------------------------------------
        print("waiting for a usable fix (navOk)...")
        deadline = time.time() + args.wait
        while time.time() < deadline:
            nav = u8(x, D_NAV_OK)
            sats = u8(x, D_NUM_SATS)
            fix = u8(x, D_FIX_TYPE)
            hacc = f32(x, D_HACC)
            print(f"  numSats {sats:2d}  fixType {fix}  navOk {nav}  "
                  f"hAcc {hacc:6.2f} m", end="\r")
            if nav:
                break
            time.sleep(1.0)
        else:
            sys.exit("\nno usable fix within the timeout. A cold receiver can "
                     "take minutes outdoors; numSats climbing means it is "
                     "working, numSats stuck at 0 means it is not.")
        print("\nfix acquired.")

        # --- 2. wait for the origin ------------------------------------
        deadline = time.time() + 30.0
        while time.time() < deadline and not u8(x, F_ORIGIN_SET):
            time.sleep(0.5)
        if not u8(x, F_ORIGIN_SET):
            sys.exit("navOk is set but originSet never went to 1 -- the fix is "
                     "not reaching Fusion_setGnss. Check the navOk gate in "
                     "Task_Measure100ms.")
        print("tangent-plane origin latched.\n")

        # --- 3. record ---------------------------------------------------
        # The CSV is opened here and flushed every row, not written at the end.
        # A walk is physical effort that cannot be replayed, so a crash in the
        # verdict arithmetic must not cost the data that was already gathered --
        # the same lesson tools/mag_cal.py learned by throwing away two
        # rotations.
        rows = []
        t0 = time.time()
        upd0 = u32(x, F_GNSS_UPD)
        fields = ["t", "posN", "posE", "velN", "velE", "innovN", "innovE",
                  "varN", "posD", "velD", "baroBias", "upd", "rej", "cov",
                  "itow", "dupes"]
        fh = open(args.csv, "w", newline="", encoding="utf-8")
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        print(f"recording {args.seconds:.0f} s "
              f"-- stand still first, then walk a closed loop\n")
        print("    t   posN    posE    velN    velE   innovN  innovE  "
              "varN    posD    baroBias  upd  rej")
        while time.time() - t0 < args.seconds:
            t = time.time() - t0
            r = dict(
                t=t,
                posN=f32(x, F_POS_N), posE=f32(x, F_POS_E),
                velN=f32(x, F_VEL_N), velE=f32(x, F_VEL_E),
                innovN=f32(x, F_INNOV_N), innovE=f32(x, F_INNOV_E),
                varN=f32(x, F_VAR_N),
                posD=f32(x, F_POS_D), velD=f32(x, F_VEL_D),
                baroBias=f32(x, F_BARO_BIAS),
                upd=u32(x, F_GNSS_UPD), rej=u32(x, F_GNSS_REJ),
                cov=u32(x, F_COV_RESETS),
                itow=u32(x, F_GNSS_ITOW), dupes=u32(x, F_GNSS_DUPES),
            )
            rows.append(r)
            writer.writerow(r)
            fh.flush()
            print(f"{t:6.1f} {r['posN']:+7.2f} {r['posE']:+7.2f} "
                  f"{r['velN']:+7.2f} {r['velE']:+7.2f} "
                  f"{r['innovN']:+7.2f} {r['innovE']:+7.2f} "
                  f"{r['varN']:8.2f} {r['posD']:+7.2f} {r['baroBias']:+8.3f} "
                  f"{r['upd']:5d} {r['rej']:4d}", end="\r")
            time.sleep(args.rate)
        print("\n")

        # --- 4. verdict ---------------------------------------------------
        dt = rows[-1]["t"] - rows[0]["t"]
        dupd = rows[-1]["upd"] - upd0
        rate = dupd / dt if dt > 0 else 0.0
        drej = rows[-1]["rej"] - rows[0]["rej"]

        ditow = rows[-1]["itow"] - rows[0]["itow"]
        ddup = rows[-1]["dupes"] - rows[0]["dupes"]

        print(f"duration          {dt:.0f} s")
        print(f"GNSS fixes fused  {dupd}  ->  {rate:.2f} /s   (10 Hz expected)")
        print(f"iTOW advanced     {ditow} ms over {dt:.0f} s "
              f"(should track wall clock)")
        print(f"duplicate polls   {ddup}")
        print(f"GNSS rejected     {drej}")
        print(f"covResets         {rows[-1]['cov']}")
        print(f"final varN        {rows[-1]['varN']:.2f} m^2 "
              f"(sqrt {rows[-1]['varN']**0.5:.1f} m)")
        print(f"baroBias          {rows[0]['baroBias']:+.3f} -> "
              f"{rows[-1]['baroBias']:+.3f} m")

        ok = True
        if rate < 1.0:
            print(f"\n!! FAIL: only {rate:.2f} fixes/s against the 10 Hz "
                  "expected. If iTOW is frozen too, it is not being decoded "
                  "and every fix after the first is discarded as a duplicate "
                  "-- the position then goes stale while still looking "
                  "entirely plausible.")
            ok = False
        elif ditow < (dt * 500.0):
            print(f"\n!! FAIL: iTOW advanced only {ditow} ms in {dt:.0f} s. It "
                  "should track the wall clock; a stalled iTOW means the field "
                  "is not being decoded even though fixes are arriving.")
            ok = False
        else:
            print(f"\nOK: {rate:.2f} fixes/s with iTOW tracking the clock -- "
                  "real fixes are reaching the filter.")

        # Correlation warning. Deliberately NOT pass/fail: it is a tuning
        # judgement that these numbers can inform but not settle.
        if drej > (0.05 * max(dupd, 1)):
            print(f"\n!! {drej} of {dupd} fixes rejected "
                  f"({100 * drej / max(dupd, 1):.0f}%). At 10 Hz consecutive "
                  "NAV-PVT solutions are NOT independent -- the receiver "
                  "filters internally at the nav rate, so successive fixes "
                  "share most of their information. Fusing every one at face "
                  "value shrinks the covariance below the truth, which "
                  "tightens the gate and starts rejecting good fixes. Fix by "
                  "decimating to 1-2 Hz for fusion, or by inflating the GNSS R.")

        if rows[-1]["cov"] != 0:
            print("!! FAIL: covResets is non-zero. That is a numerical bug, "
                  "not a tuning problem.")
            ok = False
        if rows[-1]["varN"] > 1000.0:
            print("!! varN is large -- position is not being constrained.")
            ok = False

        if args.csv:
            with open(args.csv, "w", newline="", encoding="utf-8") as fh:
                w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            print(f"\nwrote {args.csv} ({len(rows)} rows)")

        if args.mf4:
            try:
                if write_mf4(args.mf4, rows):
                    print(f"\nwrote {args.mf4} ({len(rows)} samples, "
                          f"{len(rows[0]) - 1} signals)")
                    print(f"  python tools/mf4_stats.py {args.mf4}")
            except Exception as exc:                # noqa: BLE001
                # Never let an export problem look like a failed run: the
                # measurement is already safe in the CSV.
                print(f"\nMF4 export failed ({exc}); the CSV is intact.",
                      file=sys.stderr)

        x.disconnect()
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
