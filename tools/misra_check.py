"""MISRA C check for src/ using cppcheck's misra addon.

Runs cppcheck (>= 2.13) with the MISRA addon over src/ and compares the
findings against tools/misra_baseline.txt. Existing violations are
grandfathered per (rule, file, count); the check fails only when a rule
gains new violations in a file. This keeps CI green on legacy debt while
blocking new MISRA violations.

Usage:
    python tools/misra_check.py                    # check against baseline
    python tools/misra_check.py --update-baseline  # re-record current state

cppcheck is located via the CPPCHECK env var, then PATH. If the local
distribution does not ship addons/misra.py (this script now detects and
refuses to silently pass when that happens -- see task 18 below), set
CPPCHECK_MISRA_ADDON to the path of a misra.py from the SAME pinned
version (cppcheckdata.py and misra_9.py must sit next to it); CI never
needs this, it always builds cppcheck from source with the addon present.

Note on editions: the free cppcheck addon implements MISRA C:2012 (incl.
amendments), which MISRA C:2025 consolidates. With a Cppcheck Premium
licence, native MISRA C:2025 ids are available via
`--premium=misra-c-2025` (adapt the command below).

Rule texts are licensed by MISRA and must not be committed. To see full
rule descriptions locally, place your licensed copy at
tools/misra_rule_texts.txt (gitignored) and it is picked up automatically.

Task 18 (SYS1-001 strand B, B6.4 cleanup): a CI-only MISRA finding
(misra-c2012-10.8, Icm42688.c:589, misra.yml run 34702638061) was invisible
to a local run reporting "0 finding(s)" on the identical commit, both sides
claiming cppcheck 2.21.0. Root cause: the local Windows cppcheck
distribution does not ship addons/misra.py at all -- `--addon=misra` prints
"Did not find addon misra.py" to STDOUT and cppcheck then exits 0 having
run NO Misra analysis whatsoever, and the previous version of this script
only ever read stderr for findings, so that message -- and the fact that
the whole check had silently done nothing -- was discarded. Not a flag,
--std, include-path, --suppressions or platform difference: CI's
`.github/workflows/misra.yml` invocation and this script's are already
byte-identical (CI just sets the CPPCHECK env var this script already
honours); the two only ever differed in whether the addon FILE existed on
disk. This script now (1) verifies the cppcheck --version output before
running anything, and (2) inspects cppcheck's own stdout for "did not find
addon" and refuses to report a clean result if the addon never actually
loaded -- reproducing CI's finding, or failing loudly, is the only two
outcomes now; silently checking nothing is not a third one any more.

Follow-up (reviewer spot-check, same task): the ADDON_FAILURE_MARKERS check
above is a blocklist of specific strings, not a safety net -- a stub
cppcheck (or a crashed addon, or a truncated command line) that reports a
plausible version and then emits NO output at all still yielded
findings == [] and matched none of those markers, so the OLD baseline
comparison read "0 findings against a 234-violation baseline" as 234
violations having been FIXED and exited 0. evaluate() now enforces the
positive invariant instead: a non-empty baseline requires the run to have
found something recognisable, and refuses (exit 1) both when findings is
completely empty and when an implausible fraction (> 50%) of the baseline
disappeared in one run without --update-baseline. Run `python
tools/misra_check.py --selftest` to exercise this offline, no cppcheck
needed.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "tools" / "misra_baseline.txt"
RULE_TEXTS = ROOT / "tools" / "misra_rule_texts.txt"

# Task 18: must track .github/workflows/misra.yml's CPPCHECK_VERSION env var
# exactly -- the one thing this script cannot detect on its own is "the
# right cppcheck version but a distribution missing the misra addon", so a
# version check is necessary but not sufficient (see run_cppcheck()'s addon
# probe below for the other half).
EXPECTED_CPPCHECK_VERSION = "2.21.0"

# Substrings cppcheck itself prints (to STDOUT, not stderr) when --addon=misra
# could not be loaded -- seen locally: "Did not find addon misra.py". An
# early, SPECIFIC diagnosis when it matches -- but a blocklist of strings
# is not a safety net (reviewer spot-check on task 18): a stub cppcheck, a
# crashed addon that prints nothing, or a truncated command line all yield
# empty output that matches none of these markers, and the general guard
# below (evaluate()) is what actually has to catch those.
ADDON_FAILURE_MARKERS = (
    "did not find addon",
    "unable to load addon",
    "failed to execute addon",
)

# evaluate(): a run whose findings disappear against MORE than this
# fraction of the non-empty baseline, without --update-baseline, is judged
# broken rather than improved (see evaluate()'s own docstring for why).
IMPLAUSIBLE_DISAPPEARANCE_FRACTION = 0.5

CHECK_DIRS = ["src"]
DEFINES = ["DEVICE_TC39XB", "__TASKING__"]
INCLUDE_ROOTS = [  # every directory below these is passed as an include path
    "Libraries/iLLD",
    "Libraries/Infra",
]
INCLUDE_DIRS = [
    "src/bsw",
    "src/asw",
    "Configurations",
    "Libraries/Ethernet/lwip/src/include",
    "Libraries/Ethernet/lwip/port/include",
    "Libraries/Ethernet/Phy_Rtl8211f",
]

FINDING_RE = re.compile(r"^(?P<file>.+?)\|(?P<line>\d+)\|(?P<id>[\w\-.]+)\|(?P<msg>.*)$")


def find_cppcheck():
    exe = os.environ.get("CPPCHECK") or shutil.which("cppcheck")
    if not exe:
        sys.exit("error: cppcheck not found (set CPPCHECK or add it to PATH)")
    return exe


def check_cppcheck_version(exe):
    """Fail loudly on a version mismatch against
    .github/workflows/misra.yml's CPPCHECK_VERSION -- a matching version
    string is necessary for a reproducible gate but, as task 18 found, not
    sufficient (see the addon probe in run_cppcheck())."""
    proc = subprocess.run([exe, "--version"], capture_output=True, text=True)
    raw = (proc.stdout or proc.stderr).strip()
    m = re.search(r"(\d+\.\d+(?:\.\d+)?)", raw)
    found = m.group(1) if m else None
    if found != EXPECTED_CPPCHECK_VERSION:
        sys.exit(
            f"error: cppcheck version mismatch -- found "
            f"{found or raw!r}, expected {EXPECTED_CPPCHECK_VERSION} "
            f"(.github/workflows/misra.yml CPPCHECK_VERSION). Install the "
            f"pinned version or the gate is not reproducible against CI."
        )
    print(f"cppcheck version: {found} (matches CI's {EXPECTED_CPPCHECK_VERSION})")


def collect_include_dirs():
    dirs = [ROOT / d for d in INCLUDE_DIRS]
    for root in INCLUDE_ROOTS:
        base = ROOT / root
        dirs.append(base)
        dirs.extend(p for p in sorted(base.rglob("*")) if p.is_dir())
    return dirs


def run_cppcheck(exe):
    # CI's invocation is exactly "--addon=misra" (misra.yml builds cppcheck
    # from source, which always ships addons/misra.py next to the binary,
    # so the bare name always resolves there). CPPCHECK_MISRA_ADDON is a
    # LOCAL-ONLY escape hatch, never needed by CI: a distribution missing
    # the addon file (task 18's root cause) can point this at a manually
    # obtained copy of the same pinned version's misra.py -- CI's flags do
    # not change either way.
    addon = os.environ.get("CPPCHECK_MISRA_ADDON", "misra")
    cmd = [
        exe,
        f"--addon={addon}",
        "--enable=style",  # misra findings have style severity; without this they are filtered out
        "--std=c11",
        # without an explicit platform, cppcheck assumes the *host* type widths
        # and findings differ between Windows and the Linux CI runner; unix32
        # is ILP32 like the TriCore target
        "--platform=unix32",
        "--inline-suppr",
        "--suppress=*:Libraries/*",
        # LayoutAssert_gen.h (docs/MEMORY_PLACEMENT.md part 6) is GENERATED,
        # by tools/gen_a2l.py, not hand-written: ~170 negative-array-size
        # static-assert typedefs, the portable/MISRA-known idiom for a
        # compile-time check on a toolchain without _Static_assert
        # (confirmed unavailable here under this project's exact build
        # flags). Every one of those typedefs is, by the nature of the
        # idiom, never referenced anywhere -- misra-c2012-2.3 ("a project
        # should not contain unused type declarations") is expected to fire
        # on every single line. One blanket suppression for this one
        # generated file, not ~170 repeated inline comments the generator
        # would also have to emit -- the file's own header already says
        # "do not edit by hand" and states why this pattern exists.
        "--suppress=misra-c2012-2.3:src/bsw/LayoutAssert_gen.h",
        "--template={file}|{line}|{id}|{message}",
        "--quiet",
    ]
    if RULE_TEXTS.is_file():
        cmd.append(f"--rule-texts={RULE_TEXTS}")
    cmd += [f"-D{d}" for d in DEFINES]
    cmd += [f"-I{d}" for d in collect_include_dirs()]
    cmd += [str(ROOT / d) for d in CHECK_DIRS]

    proc = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)

    # Task 18: the failure mode that made this gate non-reproducible locally
    # -- cppcheck prints this to STDOUT (never stderr) and then exits 0
    # having run no MISRA analysis at all. Checked unconditionally, even
    # when findings come back empty and even when they don't: a clean
    # result from an addon that never loaded is not a clean result.
    stdout_lower = proc.stdout.lower()
    if any(marker in stdout_lower for marker in ADDON_FAILURE_MARKERS):
        sys.exit(
            "error: the misra addon did not load -- cppcheck ran with NO "
            "MISRA analysis, so a clean result here means nothing. "
            "cppcheck's own message:\n"
            f"{proc.stdout.strip()}\n\n"
            "This is not a --std/include-path/--suppressions/platform "
            "mismatch against .github/workflows/misra.yml -- that "
            "workflow's invocation is this exact script, unchanged. The "
            "local cppcheck distribution is missing addons/misra.py "
            "entirely. Build cppcheck from source the same way "
            "misra.yml does (see that file), or install a distribution "
            "that ships the addon."
        )

    findings = []
    for line in proc.stderr.splitlines():
        m = FINDING_RE.match(line.strip())
        if not m or not m["id"].startswith("misra-"):
            continue
        path = Path(m["file"])
        try:
            path = path.resolve().relative_to(ROOT)
        except ValueError:
            pass
        findings.append(
            (m["id"], path.as_posix(), int(m["line"]), m["msg"])
        )
    # tool errors (bad include path, addon failure) must not pass silently
    if proc.returncode not in (0, 1) and not findings:
        sys.exit(f"error: cppcheck failed (exit {proc.returncode}):\n{proc.stderr}")
    return findings


def load_baseline():
    counts = Counter()
    if not BASELINE.is_file():
        return counts
    for line in BASELINE.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        rule, path, count = line.split()
        counts[(rule, path)] = int(count)
    return counts


def save_baseline(counts):
    lines = [
        "# MISRA baseline: <rule> <file> <count of grandfathered violations>",
        "# Regenerate with: python tools/misra_check.py --update-baseline",
    ]
    for (rule, path), n in sorted(counts.items()):
        lines.append(f"{rule} {path} {n}")
    BASELINE.write_text("\n".join(lines) + "\n")


def evaluate(current, baseline, findings):
    """Pure decision logic: given this run's findings-by-(rule,path) Counter
    and the loaded baseline Counter, decide OK/FAIL and build the report.
    Separated from main() so selftest() below can exercise it directly with
    synthetic data -- no cppcheck, no subprocess, no filesystem.

    Reviewer spot-check on task 18: the ADDON_FAILURE_MARKERS blocklist in
    run_cppcheck() is an early, specific diagnosis, not a safety net -- a
    stub cppcheck, a crashed addon that prints nothing, or a truncated
    command line all produce EMPTY findings that match none of those
    markers, and the old code here read that as "234 baselined violations
    no longer occur" and exited 0. The positive invariant: a run cannot be
    trusted to represent "fixed" unless it found something close to what
    the baseline expects. Returns (exit_code, message).
    """
    baseline_total = sum(baseline.values())

    if baseline_total > 0:
        if not findings:
            return 1, (
                "error: cppcheck produced ZERO misra findings while the "
                f"baseline expects {baseline_total} across {len(baseline)} "
                "rule/file group(s) -- this is almost certainly a broken "
                "run (a crashed addon, a truncated command line, or no "
                "output at all -- see run_cppcheck()'s addon-marker check "
                "for the most common named cause), not a genuine "
                "improvement. Investigate before trusting this result; "
                "only use --update-baseline once the violations are "
                "verified to actually be fixed."
            )

        disappeared = sum(max(0, n - current.get(k, 0)) for k, n in baseline.items())
        disappeared_fraction = disappeared / baseline_total
        if disappeared_fraction > IMPLAUSIBLE_DISAPPEARANCE_FRACTION:
            return 1, (
                f"error: {disappeared} of {baseline_total} baselined "
                f"violation(s) ({disappeared_fraction:.0%}) disappeared in "
                "one run without --update-baseline -- implausible for a "
                "genuine fix and far more likely a broken run (partial "
                "output, wrong include paths, an addon that failed on most "
                "files). Investigate before trusting this result."
            )
    # else: nothing baselined yet -- an empty run is unremarkable.

    regressions = {k: (current[k], baseline.get(k, 0))
                   for k in current if current[k] > baseline.get(k, 0)}
    improvements = {k: (current.get(k, 0), baseline[k])
                    for k in baseline if current.get(k, 0) < baseline[k]}

    if regressions:
        lines = [f"FAIL: {len(regressions)} rule/file group(s) exceed the baseline", ""]
        for (rule, path), (now, base) in sorted(regressions.items()):
            lines.append(f"  {rule} in {path}: {now} violation(s), baseline allows {base}")
            for r, p, line, msg in findings:
                if (r, p) == (rule, path):
                    lines.append(f"    {p}:{line}: {msg}")
            lines.append("")
        lines.append("Fix the new violations, add a justified inline suppression\n"
                      "(/* cppcheck-suppress misra-c2012-X.Y ; deviation: ... */),\n"
                      "or intentionally re-baseline with --update-baseline.")
        return 1, "\n".join(lines)

    total = sum(current.values())
    lines = [f"OK: {total} finding(s), all covered by the baseline"]
    if improvements:
        fixed = sum(b - n for n, b in improvements.values())
        lines.append(f"note: {fixed} baselined violation(s) no longer occur - "
                      "run with --update-baseline to lock in the improvement")
    return 0, "\n".join(lines)


def selftest():
    """Task 18 follow-up (reviewer spot-check): prove evaluate() actually
    refuses the exact broken-run shape the reviewer reproduced (a stub
    cppcheck reporting a plausible version but empty output against a
    non-empty baseline), plus the neighbouring cases that must NOT trip
    the new guard. Cheap: pure function calls, no cppcheck needed."""
    cases = [
        ("empty findings, non-empty baseline (the reviewer's repro)",
         Counter(),
         Counter({("misra-c2012-10.8", "src/bsw/Icm42688.c"): 234}),
         [],
         1),
        ("empty findings, empty baseline -- nothing to check, unremarkable",
         Counter(), Counter(), [], 0),
        ("a small genuine fix, well under the disappearance bound",
         Counter({("misra-c2012-8.7", "src/bsw/X.c"): 200}),
         Counter({("misra-c2012-8.7", "src/bsw/X.c"): 216}),
         [("misra-c2012-8.7", "src/bsw/X.c", 1, "msg")] * 200,
         0),
        ("over half the baseline disappeared in one run -- implausible",
         Counter({("misra-c2012-8.7", "src/bsw/X.c"): 50}),
         Counter({("misra-c2012-8.7", "src/bsw/X.c"): 216}),
         [("misra-c2012-8.7", "src/bsw/X.c", 1, "msg")] * 50,
         1),
        ("a genuine new violation still fails -- the regression path is unaffected",
         Counter({("misra-c2012-8.7", "src/bsw/X.c"): 5}),
         Counter({("misra-c2012-8.7", "src/bsw/X.c"): 2}),
         [("misra-c2012-8.7", "src/bsw/X.c", 1, "msg")] * 5,
         1),
    ]
    failed = 0
    for name, current, baseline, findings, expected in cases:
        exit_code, _ = evaluate(current, baseline, findings)
        ok = exit_code == expected
        print(f"  [{'ok' if ok else 'FAIL'}] {name}: exit {exit_code} (expected {expected})")
        if not ok:
            failed += 1
    if failed:
        print(f"selftest: {failed}/{len(cases)} case(s) failed")
        return 1
    print(f"selftest: {len(cases)}/{len(cases)} case(s) passed")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--update-baseline", action="store_true",
                        help="record the current findings as the new baseline")
    parser.add_argument("--selftest", action="store_true",
                        help="run evaluate()'s offline decision-logic self-test "
                             "and exit -- no cppcheck required")
    args = parser.parse_args()

    if args.selftest:
        return selftest()

    exe = find_cppcheck()
    check_cppcheck_version(exe)
    findings = run_cppcheck(exe)
    current = Counter((rule, path) for rule, path, _, _ in findings)

    if args.update_baseline:
        save_baseline(current)
        print(f"baseline updated: {sum(current.values())} findings "
              f"in {len(current)} rule/file groups -> {BASELINE.relative_to(ROOT)}")
        return 0

    baseline = load_baseline()
    exit_code, message = evaluate(current, baseline, findings)
    print(message)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
