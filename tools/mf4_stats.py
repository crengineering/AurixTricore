#!/usr/bin/env python3
"""Mean / spread / drift for every numeric signal in an MF4 recording.

Written for bench characterisation of fusion inputs: point it at a DAQ record
and it tells you, per signal, what the filter needs to know.

    mean    the constant offset -- for a_D at rest this is the accelerometer
            bias lumped together with gravity scale error and residual
            mounting tilt. Inseparable, and that is fine: the bias state
            absorbs the lump.
    std     sample-to-sample scatter. A FLOOR for sigma_a in Q, never the
            final value -- Q must also cover un-modelled dynamics.
    spread  max - min, i.e. peak-to-peak. Catches single outliers that std
            hides.
    drift   mean(last 10%) - mean(first 10%). If this is large compared to
            std, you are looking at bias instability rather than noise.

Usage
-----
    python tools/mf4_stats.py <file.mf4>
    python tools/mf4_stats.py <file.mf4> -s a_D            # substring filter
    python tools/mf4_stats.py <file.mf4> --start 5 --end 35
    python tools/mf4_stats.py <file.mf4> --csv stats.csv
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

try:
    from asammdf import MDF
except ImportError:
    sys.exit("asammdf is not installed:  pip install asammdf")

NUMERIC_KINDS = "fiub"


def stats_for(samples: np.ndarray, timestamps: np.ndarray) -> dict:
    x = samples.astype(np.float64, copy=False)
    n = x.size
    edge = max(1, n // 10)
    duration = float(timestamps[-1] - timestamps[0]) if n > 1 else 0.0
    return {
        "n": n,
        "rate": (n - 1) / duration if duration > 0 else float("nan"),
        "mean": float(np.mean(x)),
        "std": float(np.std(x, ddof=1)) if n > 1 else 0.0,
        "min": float(np.min(x)),
        "max": float(np.max(x)),
        "spread": float(np.max(x) - np.min(x)),
        "drift": float(np.mean(x[-edge:]) - np.mean(x[:edge])),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", type=Path, help="MF4 recording")
    ap.add_argument("-s", "--signals", default=None,
                    help="only signals whose name contains this (case-insensitive)")
    ap.add_argument("--start", type=float, default=None, help="window start [s]")
    ap.add_argument("--end", type=float, default=None, help="window end [s]")
    ap.add_argument("--csv", type=Path, default=None, help="also write a CSV")
    args = ap.parse_args()

    if not args.file.is_file():
        sys.exit(f"no such file: {args.file}")

    rows = []
    skipped = []
    with MDF(args.file) as mdf:
        for sig in mdf.iter_channels():
            if args.signals and args.signals.lower() not in sig.name.lower():
                continue
            if sig.samples.dtype.kind not in NUMERIC_KINDS or sig.samples.size == 0:
                skipped.append(sig.name)
                continue

            t, x = sig.timestamps, sig.samples
            if args.start is not None or args.end is not None:
                lo = -np.inf if args.start is None else args.start
                hi = np.inf if args.end is None else args.end
                keep = (t >= lo) & (t <= hi)
                if not keep.any():
                    skipped.append(f"{sig.name} (empty window)")
                    continue
                t, x = t[keep], x[keep]

            row = {"signal": sig.name, "unit": sig.unit or ""}
            row.update(stats_for(x, t))
            rows.append(row)

    if not rows:
        sys.exit("no numeric signals matched")

    width = max(len(r["signal"]) for r in rows)
    hdr = (f"{'signal':<{width}}  {'n':>6} {'Hz':>7} {'mean':>12} {'std':>11} "
           f"{'min':>12} {'max':>12} {'spread':>11} {'drift':>11}  unit")
    print(hdr)
    print("-" * len(hdr))
    for r in sorted(rows, key=lambda r: r["signal"]):
        print(f"{r['signal']:<{width}}  {r['n']:>6d} {r['rate']:>7.1f} "
              f"{r['mean']:>12.6g} {r['std']:>11.4g} {r['min']:>12.6g} "
              f"{r['max']:>12.6g} {r['spread']:>11.4g} {r['drift']:>11.4g}  {r['unit']}")

    if skipped:
        print(f"\nskipped {len(skipped)} non-numeric/empty: {', '.join(skipped[:8])}"
              + (" ..." if len(skipped) > 8 else ""))

    if args.csv:
        with args.csv.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"\nwrote {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
