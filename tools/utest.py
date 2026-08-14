"""
Unit Tests for C Code
"""
import sys
import subprocess
from pathlib import Path
import argparse
import shutil
import xml.etree.ElementTree as ET

# Paths in dependency to this script
ROOT      = Path(__file__).resolve().parents[1]
TEST_DIR  = ROOT / "test"

# Flags for sanitizing
SAN_FLAGS = ("-fsanitize=address,undefined -fno-sanitize-recover=all "
             "-g -fno-omit-frame-pointer")

# Flags for coverage
COV_FLAGS = ("--coverage -g -O0")

# Flag dictionary
VARIANTS = {
    "plain"    : ("build", None),
    "sanitize" : ("build_asan", SAN_FLAGS),
    "coverage" : ("build_cov",  COV_FLAGS)
}

def run(cmd, check=True):
    """
        Terminal command as list of arguments -> independent from shell.
        check=False returns the exit code instead of aborting, so the caller
        can still write a report before failing.
    """
    print(">", " ".join(str(c) for c in cmd), flush=True)
    proc = subprocess.run(cmd, cwd=ROOT)
    if check and proc.returncode != 0:
        sys.exit(proc.returncode)
    return proc.returncode

def write_test_summary(xml_path, md_path):
    """
        Turn ctest's JUnit XML into a markdown table. The CI appends it to
        $GITHUB_STEP_SUMMARY, so the result shows up on the run's front page.
        A failed testcase carries a <failure> child and status="fail".
    """
    root   = ET.parse(xml_path).getroot()
    total  = int(root.get("tests", 0))
    failed = int(root.get("failures", 0))
    icon   = "🔴" if failed else "🟢"

    lines = ["# Unit tests", "",
             f"{icon} **{total - failed}/{total}** passed", "",
             "| Test | Status | Time |",
             "|------|--------|------|"]
    for tc in root.iter("testcase"):
        ok = tc.find("failure") is None
        lines.append(f"| `{tc.get('name')}` | "
                     f"{'🟢 pass' if ok else '🔴 fail'} | "
                     f"{float(tc.get('time', 0)):.2f}s |")

    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

def parse_args():
    """
    parse arguments from function python script call
    Currently supported:
    --clean           : deletes Build-Folder
    -R <testname>     : a choosen test will be executed
    --variant         : Test execution can be chosen between plain (regular), sanitize and coverage
    --werror          : Warning will be executed as errors inside CI
    --build-only      : only builds
    --opt             : compiler optimisation levels
    """
    p = argparse.ArgumentParser(description="Build and run the C unit tests")
    p.add_argument("--clean", action="store_true",
                   help="Build-Verzeichnis vorher loeschen")
    p.add_argument("-R", "--filter",
                   help="only tests, which names fit a real test")
    p.add_argument("--variant", choices=["plain", "sanitize", "coverage"], 
                   default="plain",
                   help="plain=normal, sanitize=ASan+UBSan (only Linux/CI, "
                    "MinGW has no libasan), coverage=gcov")
    p.add_argument("--werror", action="store_true",
                   help="Warning as errors (for CI)")
    p.add_argument("--build-only", action="store_true",
                   help="only configure and build, no test execution")
    p.add_argument("--opt", default="",
               help="optimisation level, e.g. -Og or -Os (default: compiler default)")
    return p.parse_args()

def main():
    """
        Step 1: Configuration
                reads CMakelist.txt
        Step 2: Building
                CMake building with Ninja
        Step 3: Execution
                Runs tests
        Step 4: Test report
                Only Coverage Report supported atm
    """
    # parse arguments used for script call
    args = parse_args()

    # check if build for tests/sanitize/coverage
    build_name, flags = VARIANTS[args.variant]
    build_dir = TEST_DIR / build_name

    # clean before building
    if args.clean and build_dir.exists():
        print(f"> removing {build_dir}", flush=True)
        shutil.rmtree(build_dir)

    # Step 1: Configuration
    cflags = flags or ""
    if args.opt:
        cflags = (cflags + " " + args.opt).strip()
    if args.werror:
        cflags = (cflags + " -Werror").strip()

    cmake_cmd = ["cmake", "-S", TEST_DIR, "-B", build_dir, "-G", "Ninja",
                 f"-DCMAKE_C_FLAGS={cflags}",
                 f"-DCMAKE_EXE_LINKER_FLAGS={flags or ''}"]
    
    run(cmake_cmd)

    # Step 2: Building
    cbuild_cmd = ["cmake", "--build", build_dir]
    run(cbuild_cmd)

    if args.build_only:
        return 0

    # Step 3: Execute tests. All tests without filter argument, otherwise only chosen tests.
    # check=False: the summary below matters most when tests fail, so do not abort here.
    junit = build_dir / "results.xml"
    ctest_cmd = ["ctest", "--test-dir", build_dir, "--output-on-failure",
                 "--output-junit", junit]
    if args.filter:
        ctest_cmd += ["-R", args.filter]
    rc = run(ctest_cmd, check=False)
    write_test_summary(junit, build_dir / "test_summary.md")

    # Step 4: Publish Test Report
    if args.variant == "coverage":
        report = build_dir / "coverage" / "index.html"
        report.parent.mkdir(parents=True, exist_ok=True)
        run(["gcovr", "--root", ROOT, build_dir,
             "--filter", "src/bsw/",
             "--html-details", report,
             "--print-summary",
             "--markdown", report.parent / "summary.md",])

    # propagate ctest's verdict: reports are written, now fail if tests failed
    return rc

if __name__ == "__main__":
    sys.exit(main())