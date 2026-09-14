#!/usr/bin/env python3
"""SWE1-FW-014 task 13 -- reproduce the design note's threshold table and the
would-be stationary-lock timeline across the eight recordings the design
note (docs/NAV_STRAND_2026-09.md section 10.7, row 13) names, and write the
result as a CSV under Measurement Data\\20260914_nav_replay\\ for the
evidence index.

Uses tools/nav_replay.py's DetectorRecording/detector_metrics (same code the
`--detector` CLI mode runs) so the two never drift apart -- this script is
the "test/ref/" half of task 13's file list, nav_replay.py's --detector mode
is the other half.

Deliberately outside `src/` and outside the ctest suite: it needs eight raw
recordings that live in `Measurement Data\\`, which stays outside git
(evidence/INDEX.md), so it cannot run in CI. Run it by hand and record the
printed hash in the evidence index.

Usage:
    python test/ref/lock_timeline_report.py [--data-dir "C:\\...\\Measurement Data"]
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "tools"))
from nav_replay import DetectorRecording, detector_metrics  # noqa: E402

# (evidence id, relative path under Measurement Data, state) -- exactly the
# eight rows of docs/NAV_STRAND_2026-09.md section 10, "Source / evidence"
# table (NOT the seven-rest-plus-one-CSV list quoted in the dispatch note,
# which does not match the design note it is supposed to summarise -- see
# the task report).
RECORDINGS = [
    ("79227578FFFE7798", "2026-09-11_yaw_at_rest_fixed2.csv", "rest"),
    ("24E52A424D1F78C3", "2026-09-12_strandB_level_r3.csv", "rest"),
    ("1E1CC203E9702454", "2026-09-12_strandB_level_r4.csv", "rest"),
    ("02F0B754CCA975C6", "2026-09-14_SYS2-NAV-003_lift-0p5m.mf4", "rest"),
    ("07B44DE1FEBB983A", "28082026_outdoor/t1_5minutes_still.mf4", "rest (outdoor)"),
    ("78EF295E854465E4", "2026-09-14_SYS2-NAV-003_lift-0p5m_r2.mf4", "rest + hand motion"),
    ("951F68E6FF2FBBF0", "28082026_outdoor/t2_rechteck_20_10.mf4", "walking"),
    ("3F3D79AB7274AC50", "28082026_outdoor/t5_stop_and_go_20m.mf4", "stop-and-go"),
]

# The four hand events in 78EF295E854465E4 (evidence/INDEX.md row), used only
# to annotate the transition list -- not to bias the detector.
EVENTS_78EF = [
    ("lift 1", 37.0, 47.0),
    ("lift 2", 53.0, 62.0),
    ("horizontal excursion", 215.0, 220.0),
    ("nose-up tilt", 328.0, 332.0),
]


def nearest_transition(transitions: list[dict], to: str, near_t: float) -> dict | None:
    cands = [tr for tr in transitions if tr["to"] == to]
    if not cands:
        return None
    return min(cands, key=lambda tr: abs(tr["t"] - near_t))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-dir", type=Path,
                    default=Path(r"C:\Users\chris\Projects\Measurement Data"),
                    help="root of the (out-of-git) Measurement Data tree")
    ap.add_argument("--out-dir", type=Path,
                    default=Path(r"C:\Users\chris\Projects\Measurement Data") / "20260914_nav_replay",
                    help="where to write the summary/events CSVs")
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    summary_path = args.out_dir / "stationary_lock_thresholds_2026-09-14.csv"
    events_path = args.out_dir / "stationary_lock_events_78EF295E854465E4_2026-09-14.csv"

    summary_rows = []
    events_rows = []

    for evid, rel, state in RECORDINGS:
        path = args.data_dir / rel
        if not path.is_file():
            print(f"MISSING: {evid} ({path})", file=sys.stderr)
            summary_rows.append({
                "evidence_id": evid, "file": rel, "state": state,
                "n": "", "duration_s": "", "omega_1s_window_max_dps": "",
                "accDev_1s_window_max_g": "", "pct_locked": "",
                "engaged_at_s": "", "n_transitions": "", "note": "file not found",
            })
            continue

        rec = DetectorRecording(path, None, None)
        m = detector_metrics(rec, {})
        print(f"{evid} ({rel}): omega_max={m['omega_1s_window_max_dps']:.4f} deg/s "
              f"accDev_max={m['accDev_1s_window_max_g']:.4f} g "
              f"pct_locked={m['pct_locked']:.2f}% "
              f"engaged_at={m['engaged_at_s']}")
        summary_rows.append({
            "evidence_id": evid, "file": rel, "state": state,
            "n": m["n"], "duration_s": f"{m['duration_s']:.1f}",
            "omega_1s_window_max_dps": f"{m['omega_1s_window_max_dps']:.4f}",
            "accDev_1s_window_max_g": f"{m['accDev_1s_window_max_g']:.4f}",
            "pct_locked": f"{m['pct_locked']:.2f}",
            "engaged_at_s": m["engaged_at_s"],
            "n_transitions": m["n_transitions"], "note": "",
        })

        if evid == "78EF295E854465E4":
            for name, t_start, t_end in EVENTS_78EF:
                rel_tr = nearest_transition(m["transitions"], "released", t_start)
                lock_tr = nearest_transition(m["transitions"], "locked", t_end)
                events_rows.append({
                    "event": name, "t_start_s": t_start, "t_end_s": t_end,
                    "release_t_s": rel_tr["t"] if rel_tr else "",
                    "release_delay_s": (rel_tr["t"] - t_start) if rel_tr else "",
                    "relock_t_s": lock_tr["t"] if lock_tr else "",
                    "relock_delay_s": (lock_tr["t"] - t_end) if lock_tr else "",
                })
            for tr in m["transitions"]:
                events_rows.append({
                    "event": f"(raw transition -> {tr['to']})",
                    "t_start_s": "", "t_end_s": "",
                    "release_t_s": tr["t"] if tr["to"] == "released" else "",
                    "release_delay_s": "",
                    "relock_t_s": tr["t"] if tr["to"] == "locked" else "",
                    "relock_delay_s": "",
                })

    with summary_path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(summary_rows[0].keys()))
        w.writeheader()
        w.writerows(summary_rows)

    if events_rows:
        with events_path.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(events_rows[0].keys()))
            w.writeheader()
            w.writerows(events_rows)

    for p in (summary_path, events_path):
        if p.is_file():
            h = hashlib.sha256(p.read_bytes()).hexdigest().upper()
            print(f"{p}  sha256[:16]={h[:16]}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
