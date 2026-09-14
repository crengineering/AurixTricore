"""ctest `nav_replay` -- SWE1-FW-013 task 1, clauses (b)/(c)/(d).

Usage:  python nav_replay_fidelity.py <gen_fusion_trace exe> <data dir>

Runs `gen_fusion_trace` on a small COMMITTED command-stream excerpt
(`data/nav_replay_excerpt.cmd`, 400 fixes / 40 s from the front of the
2026-09-14 indoor recording, `QuadSE/evidence/INDEX.md` row
02F0B754CCA975C6), and checks it against the excerpt's LOGGED ground truth
(`data/nav_replay_excerpt_expected.csv`) -- the same recording the raw MF4
stays outside git for (`evidence/INDEX.md`).

WHAT THIS CHECKS, AND WHY NOT NavPosDown/NavBaroBias DIRECTLY.

fusion.c's barometer reference (`refM`) latches on the FIRST sample after
`Fusion_init()` (fusion.c Fusion_setBaroAlt / FusionLatch.h), and
`Fusion_init()` always starts `d = measBias = 0`. This recording's counters
show the board had already been running for a long time when it started
(`NavGnssUpdates` begins at 6797, i.e. >11 minutes of prior fixes already
fused), so its OWN reference and its `d`/`measBias` split at t=0 reflect that
whole unlogged history -- which this MF4 does not, and cannot, contain. A
cold `Fusion_init()` replay therefore starts from a different, and
irrecoverable, `d`/`measBias` split than the board had: measured on the full
469 s run, `NavPosDown` alone differs from the replay by up to 7.6 m (RMS
5.29 m), which is the exact, structural, and unavoidable "missing input"
`tools/nav_replay.py`'s header warns about, NOT a defect in the replay. See
the flight-dev report for SWE1-FW-013 task 1.

What a cold-start replay CAN verify, and what this test asserts instead:
only the SUM `d + measBias` is what the barometer actually observes
(fusion.c fusion_correctBaro(): `h = [1,0,0,1]`), so it is insensitive to
which `d`/`measBias` split either run starts from -- both must agree on it
from the first tick, and do (measured: 0.058 m RMS over this excerpt,
against a 0.5 m RMS budget with margin for host-vs-board float rounding).
This is a real, tight fidelity check on the barometer Kalman update, the
gates and the counters -- the part of the mechanism a cold replay CAN judge.
"""
import csv
import subprocess
import sys
from pathlib import Path

SUM_RMS_MAX = 0.20   # m; measured 0.058 m, budget leaves headroom, see module docstring

failures = []


def fail(msg):
    failures.append(msg)
    print("FAIL: " + msg)


def note(msg):
    print("  " + msg)


def run_trace(exe: Path, commands: str) -> str:
    result = subprocess.run([str(exe)], input=commands, capture_output=True,
                             text=True, check=False)
    if result.returncode != 0:
        fail(f"gen_fusion_trace exited {result.returncode}: {result.stderr}")
        return ""
    return result.stdout


def main() -> int:
    if len(sys.argv) != 3:
        sys.exit(f"usage: {sys.argv[0]} <gen_fusion_trace exe> <data dir>")

    exe = Path(sys.argv[1])
    data = Path(sys.argv[2])
    commands = (data / "nav_replay_excerpt.cmd").read_text(encoding="utf-8")

    with (data / "nav_replay_excerpt_expected.csv").open(newline="", encoding="utf-8") as fh:
        expected = list(csv.DictReader(fh))

    out1 = run_trace(exe, commands)
    out2 = run_trace(exe, commands)
    if out1 != out2:
        fail("two runs on the same input produced different output "
             "(SWE1-FW-013 clause 4: determinism)")
    else:
        note("determinism: two runs byte-identical")

    rows = list(csv.DictReader(out1.splitlines()))
    if len(rows) != len(expected):
        fail(f"row count mismatch: gen_fusion_trace produced {len(rows)}, "
             f"expected {len(expected)}")
        return 1 if failures else 0

    sq_err = 0.0
    max_err = 0.0
    for row, exp in zip(rows, expected):
        replay_sum = float(row["d"]) + float(row["baroBias"])
        logged_sum = float(exp["sum"])
        err = replay_sum - logged_sum
        sq_err += err * err
        max_err = max(max_err, abs(err))

    rms = (sq_err / len(rows)) ** 0.5
    note(f"observable sum (d + baroBias) RMS vs logged: {rms:.4f} m "
         f"(max {max_err:.4f} m), budget {SUM_RMS_MAX} m")
    if rms > SUM_RMS_MAX:
        fail(f"observable-sum RMS {rms:.4f} m exceeds {SUM_RMS_MAX} m")

    # Structural invariants over the excerpt (all hold in the logged data,
    # see the recording's own NavBaroRejects/NavCovResets == 0 in this window).
    for name, col in (("rejects", "rejects"), ("resets", "resets"),
                       ("gnssRejects", "gnssRejects"), ("covResets", "covResets")):
        final = int(rows[-1][col])
        if final != 0:
            fail(f"{name} = {final} at the end of the excerpt, expected 0")
        else:
            note(f"{name} stayed 0")

    if int(rows[-1]["verticalOk"]) != 1:
        fail("verticalOk did not reach 1 over the excerpt")
    if int(rows[-1]["horizontalOk"]) != 1:
        fail("horizontalOk did not reach 1 over the excerpt")

    if failures:
        print(f"\n{len(failures)} FAILURE(S)")
        return 1

    print("\nPASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
