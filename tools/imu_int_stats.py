#!/usr/bin/env python3
"""ICM-42688-P INT1 data-ready interval distribution, read live off the board.

I6 of docs/IMU_INTERRUPT.md SS7: pulls the histogram block ImuInt.c
accumulates (SS5.2) over XCP and prints min/max/mean/stddev/p99 in
microseconds plus an ASCII histogram -- the thing tools/xcp_read.py alone
cannot do in one call, because the 128-byte g_imuDrdyHist array is bigger
than a single 63-byte SHORT_UPLOAD (XCP_MAX_CTO - 1). Pulls it via
xcp_read.short_upload_chunked() using the new NAME:TYPExN array form
instead.

Usage
-----
    python tools/imu_int_stats.py                  # one snapshot
    python tools/imu_int_stats.py --watch 5         # repeat every 5 s, Ctrl-C to stop
    python tools/imu_int_stats.py --host 192.168.0.10 --port 5555

Reads (all non-static globals in src/bsw/ImuInt.h / Icm42688.h, no A2L, no
Xcp_Data, no diag bit -- docs/IMU_INTERRUPT.md SS5.1):

    g_imuDrdyCount, g_imuDrdyDtMin, g_imuDrdyDtMax, g_imuDrdyDtSum,
    g_imuDrdyHist[32], g_imuDrdyHistBase, g_imuDrdyUnder, g_imuDrdyOver,
    g_imuDrdyStaleTicks, g_imuSpiBurstMaxTicks

mean is EXACT (g_imuDrdyDtSum / intervals, per ImuInt.h's own comment that
the mean is meant to be computed on the host). stddev and p99 are
APPROXIMATE: the ISR keeps a histogram, not raw samples, so both are
computed from per-bin midpoints (bins are the one place this tool cannot
recover the exact values, by construction -- that is the cost of a bounded
O(1) ISR, and the whole reason SS5.2 chose a histogram over storing samples).
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from xcp_read import (  # noqa: E402
    HOST, PORT, connect, load_symbols, parse_target, short_upload_chunked,
)

# Must match src/bsw/ImuInt.h exactly.
HIST_BINS = 32
HIST_BIN_TICKS = 100          # 1 us at STM0 @ 100 MHz
HIST_CENTER_EDGE = 100        # intervals needed before the ISR centres it
TICKS_PER_US = 100.0
NOMINAL_US = 1000.0           # current ODR, Icm42688.c GYRO/ACCEL_CONFIG0

FIELDS = [
    ("g_imuDrdyCount", "u32"),
    ("g_imuDrdyDtMin", "u32"),
    ("g_imuDrdyDtMax", "u32"),
    ("g_imuDrdyDtSum", "u64"),
    ("g_imuDrdyHist", "u32x32"),
    ("g_imuDrdyHistBase", "u32"),
    ("g_imuDrdyUnder", "u32"),
    ("g_imuDrdyOver", "u32"),
    ("g_imuDrdyStaleTicks", "u32"),
    ("g_imuSpiBurstMaxTicks", "u32"),
]


def read_snapshot(x, syms):
    """One consistent-enough read of every FIELDS symbol. Not a single atomic
    XCP transaction (there isn't one for a multi-symbol block) -- each field
    is read separately, exactly like tools/xcp_read.py does for several
    targets. The ISR can advance between reads, which biases nothing this
    tool reports: count/min/max/sum are monotonic-ish snapshots of a live
    accumulator by design, same caveat as watching any of them individually
    with xcp_read.py --watch. """
    out = {}
    for name, typ in FIELDS:
        if name not in syms:
            sys.exit(f"'{name}' not in the map -- build with I4/I5 landed "
                     f"(src/bsw/ImuInt.c, src/bsw/Icm42688.c) and re-flash.")
        _, addr, typ, nbytes = parse_target(f"{name}:{typ}", syms)
        raw = short_upload_chunked(x, addr, nbytes) if nbytes > 63 \
            else bytes(x.shortUpload(nbytes, addr, 0))
        out[name] = raw
    return out


def unpack(raw_map):
    count      = struct.unpack("<I", raw_map["g_imuDrdyCount"])[0]
    dt_min     = struct.unpack("<I", raw_map["g_imuDrdyDtMin"])[0]
    dt_max     = struct.unpack("<I", raw_map["g_imuDrdyDtMax"])[0]
    dt_sum     = struct.unpack("<Q", raw_map["g_imuDrdyDtSum"])[0]
    hist       = list(struct.unpack("<32I", raw_map["g_imuDrdyHist"]))
    hist_base  = struct.unpack("<I", raw_map["g_imuDrdyHistBase"])[0]
    under      = struct.unpack("<I", raw_map["g_imuDrdyUnder"])[0]
    over       = struct.unpack("<I", raw_map["g_imuDrdyOver"])[0]
    stale      = struct.unpack("<I", raw_map["g_imuDrdyStaleTicks"])[0]
    burst_max  = struct.unpack("<I", raw_map["g_imuSpiBurstMaxTicks"])[0]
    return {
        "count": count, "dt_min": dt_min, "dt_max": dt_max, "dt_sum": dt_sum,
        "hist": hist, "hist_base": hist_base, "under": under, "over": over,
        "stale": stale, "burst_max": burst_max,
    }


def ticks_to_us(ticks):
    return ticks / TICKS_PER_US


def compute_stats(s):
    """-> dict with n, mean_us, min_us, max_us, stddev_us (approx or None),
    p99_us (approx or None, may carry a note), centred (bool)."""
    n = max(0, s["count"] - 1)   # first edge has no dt (ImuInt.c: `if count > 1u`)
    result = {"n": n, "centred": n >= HIST_CENTER_EDGE}

    if n == 0:
        return result

    result["mean_us"] = ticks_to_us(s["dt_sum"] / n)
    result["min_us"] = ticks_to_us(s["dt_min"])
    result["max_us"] = ticks_to_us(s["dt_max"])

    if not result["centred"]:
        return result

    base = s["hist_base"]
    buckets = [(base - HIST_BIN_TICKS / 2.0, s["under"], "under")]
    for i, cnt in enumerate(s["hist"]):
        lo = base + i * HIST_BIN_TICKS
        buckets.append(((lo + lo + HIST_BIN_TICKS) / 2.0, cnt, ("bin", lo, lo + HIST_BIN_TICKS)))
    over_lo = base + HIST_BINS * HIST_BIN_TICKS
    buckets.append((over_lo + HIST_BIN_TICKS / 2.0, s["over"], "over"))

    weighted_n = sum(cnt for _, cnt, _ in buckets)
    if weighted_n == 0:
        return result   # centred but somehow nothing binned yet

    mean_ticks = s["dt_sum"] / n   # reuse the EXACT mean as the centre
    var_ticks2 = sum(cnt * (mid - mean_ticks) ** 2 for mid, cnt, _ in buckets) / weighted_n
    result["stddev_us"] = math.sqrt(var_ticks2) / TICKS_PER_US

    target = 0.99 * weighted_n
    cum = 0.0
    p99_ticks, p99_note = None, ""
    for mid, cnt, kind in buckets:
        if cum + cnt >= target and cnt > 0:
            if kind == "under":
                p99_ticks, p99_note = base, " (>= all `under` samples; approximate)"
            elif kind == "over":
                p99_ticks = over_lo
                p99_note = f" (at/above the histogram window; real max = {result['max_us']:.1f} us)"
            else:
                _, lo, hi = kind
                frac = (target - cum) / cnt
                p99_ticks = lo + frac * (hi - lo)
            break
        cum += cnt
    if p99_ticks is not None:
        result["p99_us"] = ticks_to_us(p99_ticks)
        result["p99_note"] = p99_note

    return result


def classify(stats):
    """Mirrors the decision tree in docs/IMU_INTERRUPT.md SS5.6. An automatic
    hint only -- always cross-check against the real numbers and the tree
    itself before recording a branch in that document (I7). """
    if stats["n"] == 0:
        return ("NO EDGES", "count stayed 0 -- check INT_ASYNC_RESET (0x64 bit 4) "
                "first (the #1 cause), then INT_SOURCE0=0x08, then continuity "
                "CN1.3 -> P10.7.")
    if not stats["centred"]:
        return ("WARMING UP", f"only {stats['n']} interval(s) so far -- the "
                f"histogram centres at {HIST_CENTER_EDGE}; re-read shortly.")

    mean_us, min_us, max_us = stats["mean_us"], stats["min_us"], stats["max_us"]
    spread_us = max_us - min_us
    dev_pct = abs(mean_us - NOMINAL_US) / NOMINAL_US * 100.0

    if spread_us > 100.0 or max_us > 1500.0:
        return ("(C) DO NOT TRUST THE PATH YET",
                f"spread {spread_us:.1f} us or a dt > 1.5 ms seen -- check "
                f"INT_ASYNC_RESET, TTL pad mode, pulse width vs a floating "
                f"line, wire length.")
    if dev_pct <= 3.0 and spread_us < 20.0:
        return ("(A) THE IMU REALLY DELIVERS 1 kHz",
                "NavTask -> 1 kHz on CPU1, flight_ctrl keeps Ts = 0.001f -- "
                "gate: SPI burst max must be << 1 ms (see the burst_max_us "
                "line below).")
    if dev_pct > 1.0:
        return ("(B) OFF-NOMINAL RATE, HARMLESS TO THE ESTIMATOR",
                f"mean off nominal by {dev_pct:.2f}% -- keep flight_ctrl on "
                f"the scheduler's fixed rate, feed the AHRS/KF the measured "
                f"dt, treat DRDY as a sample-ready flag, not the clock.")
    return ("INCONCLUSIVE", "close to nominal but outside the (A) band -- "
            "re-read after a longer window (SS5.6 wants >= 60 s at rest).")


def render_histogram(s, stats, width=50):
    if not stats["centred"]:
        return "(histogram not centred yet)"
    base_us = ticks_to_us(s["hist_base"])
    peak = max([s["under"], s["over"]] + s["hist"]) or 1
    lines = []
    lines.append(f"  under (<{base_us:.1f} us): {bar(s['under'], peak, width)} {s['under']}")
    for i, cnt in enumerate(s["hist"]):
        lo_us = base_us + i
        lines.append(f"  [{lo_us:7.1f}, {lo_us + 1:7.1f}) us: {bar(cnt, peak, width)} {cnt}")
    over_us = base_us + HIST_BINS
    lines.append(f"  over  (>={over_us:.1f} us): {bar(s['over'], peak, width)} {s['over']}")
    return "\n".join(lines)


def bar(count, peak, width):
    n = round(width * count / peak) if peak else 0
    return "#" * n


def print_snapshot(s, stats, hist=True):
    print(f"edges={s['count']}  intervals={stats['n']}  "
         f"stale={ticks_to_us(s['stale']):.1f} us  "
         f"spi_burst_max={ticks_to_us(s['burst_max']):.1f} us")

    if stats["n"] == 0:
        label, why = classify(stats)
        print(f"  -> {label}: {why}")
        return

    print(f"min={stats['min_us']:.2f} us  max={stats['max_us']:.2f} us  "
         f"mean={stats['mean_us']:.3f} us (exact)")
    if "stddev_us" in stats:
        print(f"stddev~={stats['stddev_us']:.3f} us (approx, from histogram bins)")
    if "p99_us" in stats:
        print(f"p99~={stats['p99_us']:.2f} us{stats.get('p99_note', '')}")

    label, why = classify(stats)
    print(f"  -> {label}: {why}")

    if hist:
        print(render_histogram(s, stats))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--watch", type=float, metavar="SEC",
                    help="poll every SEC seconds until Ctrl-C")
    ap.add_argument("--no-hist", action="store_true", help="skip the ASCII histogram")
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    args = ap.parse_args()

    syms = load_symbols()

    with connect(args.host, args.port) as x:
        x.connect()
        try:
            while True:
                raw = read_snapshot(x, syms)
                s = unpack(raw)
                stats = compute_stats(s)
                print(f"--- [{time.strftime('%H:%M:%S')}] ---")
                print_snapshot(s, stats, hist=not args.no_hist)
                if args.watch is None:
                    break
                print()
                time.sleep(args.watch)
        except KeyboardInterrupt:
            print("\n--- stopped ---", file=sys.stderr)
        finally:
            x.disconnect()


if __name__ == "__main__":
    main()
