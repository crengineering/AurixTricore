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
BUILD_DIR = TEST_DIR / "build"

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
    """
    p = argparse.ArgumentParser(description="Build and run the C unit tests")
    p.add_argument("--clean", action="store_true",
                   help="Build-Verzeichnis vorher loeschen")
    p.add_argument("-R", "--filter",
                   help="only tests, which names fit a real test")
    return p.parse_args()

def main():
    """
        Step 1: Configuration
                reads CMakelist.txt
        Step 2: Building
                CMake building with Ninja
        Step 3: Execution
                Runs tests
    """
    # clean before building
    args = parse_args()
    if args.clean and BUILD_DIR.exists():
        print(f"> removing {BUILD_DIR}", flush=True)
        shutil.rmtree(BUILD_DIR)

    # regular build 
    run(["cmake", "-S", TEST_DIR, "-B", BUILD_DIR, "-G", "Ninja"])
    run(["cmake", "--build", BUILD_DIR])

    ctest_cmd = ["ctest", "--test-dir", BUILD_DIR, "--output-on-failure"]
    if args.filter:
        ctest_cmd += ["-R", args.filter]
    run(ctest_cmd)

    return 0

if __name__ == "__main__":
    sys.exit(main())