"""
Unit Tests for C Code
"""
import sys
import subprocess
from pathlib import Path
import argparse
import shutil

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

def run(cmd):
    """
        Terminal command as list of arguments -> independent from shell
    """
    print(">", " ".join(str(c) for c in cmd), flush=True)
    proc = subprocess.run(cmd, cwd=ROOT)
    if proc.returncode != 0:
        sys.exit(proc.returncode)

def parse_args():
    """
    parse arguments from function python script call
    Currently supported:
    --clean           : deletes Build-Folder
    -R <testname>     : a choosen test will be executed
    --variant         : Test execution can be chosen between plain (regular), sanitize and coverage
    --werror          : Warning will be executed as errors inside CI
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

    # Step 3: Execute tests. All tests without filter argument, otherwise only chosen tests
    ctest_cmd = ["ctest", "--test-dir", build_dir, "--output-on-failure"]
    if args.filter:
        ctest_cmd += ["-R", args.filter]
    run(ctest_cmd)

    # Step 4: Publish Test Report
    if args.variant == "coverage":
        report = build_dir / "coverage" / "index.html"
        report.parent.mkdir(parents=True, exist_ok=True)
        run(["gcovr", "--root", ROOT, build_dir,
             "--filter", "src/bsw/",
             "--html-details", report,
             "--print-summary",
             "--markdown", report.parent / "summary.md",])

    return 0

if __name__ == "__main__":
    sys.exit(main())