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
    p.add_argument("--sanitize", action="store_true",
                   help="SW build with AdressSanitizer and UBSan")
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
    # parse arguments used for script call
    args = parse_args()

    # check if build for tests or sanitize
    build_dir = TEST_DIR / ("build-asan" if args.sanitize else "build")

    # clean before building
    if args.clean and build_dir.exists():
        print(f"> removing {build_dir}", flush=True)
        shutil.rmtree(build_dir)

    # Step 1: Configuration
    cmake_cmd = ["cmake", "-S", TEST_DIR, "-B", build_dir, "-G", "Ninja"]
    if args.sanitize:
        cmake_cmd += [f"-DCMAKE_C_FLAGS={SAN_FLAGS}",
                      f"-DCMAKE_EXE_LINKER_FLAGS={SAN_FLAGS}"]
    run(cmake_cmd)

    # Step 2: Building
    cbuild_cmd = ["cmake", "--build", build_dir]
    run(cbuild_cmd)

    # Step 3: Execute tests. All tests without filter argument, otherwise only chosen tests
    ctest_cmd = ["ctest", "--test-dir", build_dir, "--output-on-failure"]
    if args.filter:
        ctest_cmd += ["-R", args.filter]
    run(ctest_cmd)

    return 0

if __name__ == "__main__":
    sys.exit(main())