"""
Unit Tests for C Code
"""
import sys
import subprocess
from pathlib import Path

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

def main():
    """
        Step 1: Configuration
                reads CMakelist.txt
        Step 2: Building
                CMake building with Ninja
        Step 3: Execution
                Runs tests
    """
    run(["cmake", "-S", TEST_DIR, "-B", BUILD_DIR, "-G", "Ninja"])
    run(["cmake", "--build", BUILD_DIR])
    run(["ctest", "--test-dir", BUILD_DIR, "--output-on-failure"])

if __name__ == "__main__":
    sys.exit(main())