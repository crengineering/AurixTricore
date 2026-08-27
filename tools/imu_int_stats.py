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

    g_imuDrdyCount, g_imuDrdyDtMin, g_imuDrdyDtMax, g_imuDrdyWindowSum,
    g_imuDrdyWindowIntervals, g_imuDrdyHist[32], g_imuDrdyHistBase,
    g_imuDrdyUnder, g_imuDrdyOver, g_imuDrdyStaleTicks, g_imuSpiBurstMaxTicks

mean is the EXACT mean of the most recently COMPLETED
IMUINT_MEAN_WINDOW_EDGES-interval window (~1 s at ~1 kHz), from
g_imuDrdyWindowSum / g_imuDrdyWindowIntervals -- a uint32 that resets every
window, not a uint64 grand total since reset (2026-08-27, so overflow is
structurally impossible and a per-second time series -- poll with --watch --
shows RC-oscillator drift better than one grand mean ever could). Until the
first window closes, no exact mean is available yet and this tool says so
rather than guessing. stddev and p99 are APPROXIMATE and unrelated to the
window: the ISR keeps a histogram, not raw samples, so both are computed
from per-bin midpoints (bins are the one place this tool cannot recover the
exact values, by construction -- that is the cost of a bounded O(1) ISR, and
the whole reason SS5.2 chose a histogram over storing samples).
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
WARMUP_EDGES = 100            # intervals discarded before anything counts
                               # (boot transient -- ImuInt.h IMUINT_WARMUP_EDGES)
TICKS_PER_US = 100.0
NOMINAL_US = 1000.0           # current ODR, Icm42688.c GYRO/ACCEL_CONFIG0

FIELDS = [
    ("g_imuDrdyCount", "u32"),
    ("g_imuDrdyDtMin", "u32"),
    ("g_imuDrdyDtMax", "u32"),
    ("g_imuDrdyWindowSum", "u32"),
    ("g_imuDrdyWindowIntervals", "u32"),
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
    count       = struct.unpack("<I", raw_map["g_imuDrdyCount"])[0]
    dt_min      = struct.unpack("<I", raw_map["g_imuDrdyDtMin"])[0]
    dt_max      = struct.unpack("<I", raw_map["g_imuDrdyDtMax"])[0]
    window_sum  = struct.unpack("<I", raw_map["g_imuDrdyWindowSum"])[0]
    window_ivl  = struct.unpack("<I", raw_map["g_imuDrdyWindowIntervals"])[0]
    hist        = list(struct.unpack("<32I", raw_map["g_imuDrdyHist"]))
    hist_base   = struct.unpack("<I", raw_map["g_imuDrdyHistBase"])[0]
    under       = struct.unpack("<I", raw_map["g_imuDrdyUnder"])[0]
    over        = struct.unpack("<I", raw_map["g_imuDrdyOver"])[0]
    stale       = struct.unpack("<I", raw_map["g_imuDrdyStaleTicks"])[0]
    burst_max   = struct.unpack("<I", raw_map["g_imuSpiBurstMaxTicks"])[0]
    return {
        "count": count, "dt_min": dt_min, "dt_max": dt_max,
        "window_sum": window_sum, "window_intervals": window_ivl,
        "hist": hist, "hist_base": hist_base, "under": under, "over": over,
        "stale": stale, "burst_max": burst_max,
    }


def ticks_to_us(ticks):
    return ticks / TICKS_PER_US


def compute_stats(s):
    """-> dict with n, mean_us (exact, only once a window has closed),
    min_us, max_us, stddev_us (approx or None), p99_us (approx or None, may
    carry a note), centred (bool), window_intervals (0 until the first
    window closes).

    n excludes both the first edge (no dt: ImuInt.c `if count > 1u`) AND the
    first WARMUP_EDGES intervals after it -- the firmware discards those
    outright (ImuInt.h IMUINT_WARMUP_EDGES) and never publishes dt_min/max/
    hist_base for them. n is the whole-run (since reset) interval count that
    min/max/histogram cover; it is INDEPENDENT of window_sum/window_intervals,
    which only ever describe the most recently completed ~1 s window."""
    n = max(0, s["count"] - 1 - WARMUP_EDGES)
    result = {"n": n, "centred": n >= 1, "count": s["count"],
              "window_intervals": s["window_intervals"]}

    if n == 0:
        return result

    result["min_us"] = ticks_to_us(s["dt_min"])
    result["max_us"] = ticks_to_us(s["dt_max"])
    if s["window_intervals"] > 0:
        result["mean_us"] = ticks_to_us(s["window_sum"] / s["window_intervals"])

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

    if "mean_us" in result:
        # stddev needs a centre to measure distance from; only the (exact)
        # window mean is trustworthy enough to be that centre -- before the
        # first window closes, stddev is left out rather than centred on
        # something weaker (e.g. a bin midpoint), same "do not guess" rule
        # as everywhere else in this tool.
        mean_ticks = s["window_sum"] / s["window_intervals"]
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


# Absolute thresholds, not percentages of spread: a rare few-percent tail of
# measurement artefacts (SPI-burst-correlated, docs/IMU_INTERRUPT.md SS5.6)
# must not by itself flip a genuinely tight 1 kHz source to "do not trust".
STDDEV_TRUST_US    = 20.0   # (A) requires jitter this tight or better
STDDEV_DISTRUST_US = 100.0  # above this, the CORE distribution is noisy,
                             # not just its tails -- (C)
DROPOUT_RATIO      = 2.0    # max_us >= this * mean_us is a genuine missed
                             # edge, not an artefact -- flagged on its own,
                             # regardless of how tight everything else is


def classify(stats):
    """Mirrors the decision tree in docs/IMU_INTERRUPT.md SS5.6. An automatic
    hint only -- always cross-check against the real numbers and the tree
    itself before recording a branch in that document (I7).

    Classifies on stddev (the CORE distribution's tightness), not raw
    max-min spread: a real run found spread swamped by a small (~1 %),
    SPI-burst-correlated measurement-artefact tail -- 98.85 % of samples
    inside a 2 us band, but spread alone (239 us) said "do not trust",
    which was simply wrong. A genuine dropout (one edge actually missed)
    is checked separately, by ratio to the mean, so it is never hidden by
    a good stddev the way it would be if folded into the same threshold. """
    if stats["count"] <= 1:
        return ("NO EDGES", "count stayed 0 -- check INT_ASYNC_RESET (0x64 bit 4) "
                "first (the #1 cause), then INT_SOURCE0=0x08, then continuity "
                "CN1.3 -> P10.7.")
    if not stats["centred"]:
        return ("WARMING UP", "still inside the first "
                f"{WARMUP_EDGES} discarded intervals; re-read shortly.")
    if "mean_us" not in stats:
        return ("AWAITING FIRST WINDOW", "min/max/histogram are live, but no "
                f"IMUINT_MEAN_WINDOW_EDGES-interval window has closed yet "
                f"(~1 s at ~1 kHz) -- no exact mean to classify against. "
                f"Re-read shortly.")

    mean_us, max_us = stats["mean_us"], stats["max_us"]
    dev_pct = abs(mean_us - NOMINAL_US) / NOMINAL_US * 100.0

    if max_us >= (DROPOUT_RATIO * mean_us):
        return ("(!) DROPPED EDGE",
                f"max {max_us:.1f} us is >= {DROPOUT_RATIO:.0f}x the mean "
                f"({mean_us:.1f} us) -- a genuine missed sample, not the "
                f"SPI-burst-correlated tail (that tail tops out near "
                f"spi_burst_max, well under this ratio). Investigate before "
                f"trusting this run for a rate decision.")

    if "stddev_us" not in stats:
        return ("INCONCLUSIVE", "no stddev yet -- needs at least one "
                "completed window (~1 s); re-read.")

    stddev_us = stats["stddev_us"]

    if stddev_us > STDDEV_DISTRUST_US:
        return ("(C) DO NOT TRUST THE PATH YET",
                f"stddev {stddev_us:.1f} us -- the CORE distribution itself "
                f"is noisy, not just a small tail. Check INT_ASYNC_RESET, "
                f"TTL pad mode, pulse width vs a floating line, wire length.")
    if dev_pct <= 3.0 and stddev_us < STDDEV_TRUST_US:
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

    mean_str = (f"mean={stats['mean_us']:.3f} us (exact, last completed "
                f"~1 s window, {stats['window_intervals']} intervals)"
                if "mean_us" in stats else
                "mean=(no completed window yet)")
    print(f"min={stats['min_us']:.2f} us  max={stats['max_us']:.2f} us  "
         f"{mean_str}")
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
