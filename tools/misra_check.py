"""MISRA C check for src/ using cppcheck's misra addon.

Runs cppcheck (>= 2.13) with the MISRA addon over src/ and compares the
findings against tools/misra_baseline.txt. Existing violations are
grandfathered per (rule, file, count); the check fails only when a rule
gains new violations in a file. This keeps CI green on legacy debt while
blocking new MISRA violations.

Usage:
    python tools/misra_check.py                    # check against baseline
    python tools/misra_check.py --update-baseline  # re-record current state

cppcheck is located via the CPPCHECK env var, then PATH.

Note on editions: the free cppcheck addon implements MISRA C:2012 (incl.
amendments), which MISRA C:2025 consolidates. With a Cppcheck Premium
licence, native MISRA C:2025 ids are available via
`--premium=misra-c-2025` (adapt the command below).

Rule texts are licensed by MISRA and must not be committed. To see full
rule descriptions locally, place your licensed copy at
tools/misra_rule_texts.txt (gitignored) and it is picked up automatically.
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
CHECK_DIRS = ["src"]
DEFINES = ["DEVICE_TC39XB", "__TASKING__"]
INCLUDE_ROOTS = [  # every directory below these is passed as an include path
    "Libraries/iLLD",
    "Libraries/Infra",
]
INCLUDE_DIRS = [
    "src",
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


def collect_include_dirs():
    dirs = [ROOT / d for d in INCLUDE_DIRS]
    for root in INCLUDE_ROOTS:
        base = ROOT / root
        dirs.append(base)
        dirs.extend(p for p in sorted(base.rglob("*")) if p.is_dir())
    return dirs


def run_cppcheck(exe):
    cmd = [
        exe,
        "--addon=misra",
        "--enable=style",  # misra findings have style severity; without this they are filtered out
        "--std=c11",
        "--inline-suppr",
        "--suppress=*:Libraries/*",
        "--template={file}|{line}|{id}|{message}",
        "--quiet",
    ]
    if RULE_TEXTS.is_file():
        cmd.append(f"--rule-texts={RULE_TEXTS}")
    cmd += [f"-D{d}" for d in DEFINES]
    cmd += [f"-I{d}" for d in collect_include_dirs()]
    cmd += [str(ROOT / d) for d in CHECK_DIRS]

    proc = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
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


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--update-baseline", action="store_true",
                        help="record the current findings as the new baseline")
    args = parser.parse_args()

    exe = find_cppcheck()
    findings = run_cppcheck(exe)
    current = Counter((rule, path) for rule, path, _, _ in findings)

    if args.update_baseline:
        save_baseline(current)
        print(f"baseline updated: {sum(current.values())} findings "
              f"in {len(current)} rule/file groups -> {BASELINE.relative_to(ROOT)}")
        return 0

    baseline = load_baseline()
    regressions = {k: (current[k], baseline.get(k, 0))
                   for k in current if current[k] > baseline.get(k, 0)}
    improvements = {k: (current.get(k, 0), baseline[k])
                    for k in baseline if current.get(k, 0) < baseline[k]}

    if regressions:
        print(f"FAIL: {len(regressions)} rule/file group(s) exceed the baseline\n")
        for (rule, path), (now, base) in sorted(regressions.items()):
            print(f"  {rule} in {path}: {now} violation(s), baseline allows {base}")
            for r, p, line, msg in findings:
                if (r, p) == (rule, path):
                    print(f"    {p}:{line}: {msg}")
            print()
        print("Fix the new violations, add a justified inline suppression\n"
              "(/* cppcheck-suppress misra-c2012-X.Y ; deviation: ... */),\n"
              "or intentionally re-baseline with --update-baseline.")
        return 1

    total = sum(current.values())
    print(f"OK: {total} finding(s), all covered by the baseline")
    if improvements:
        fixed = sum(b - n for n, b in improvements.values())
        print(f"note: {fixed} baselined violation(s) no longer occur - "
              "run with --update-baseline to lock in the improvement")
    return 0


if __name__ == "__main__":
    sys.exit(main())
