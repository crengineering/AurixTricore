"""
Unit Tests for C Code
"""
import sys
import subprocess
from pathlib import Path


ROOT      = Path(__file__).resolve().parents[1]
TEST_DIR  = ROOT / "test"
BUILD_DIR = TEST_DIR / "build"

def run(cmd):
    print(">", " ".join(str(c) for c in cmd), flush=True)
    proc = subprocess.run(cmd, cwd=ROOT)
    if proc.returncode != 0:
        sys.exit(proc.returncode)


def main():
    """
    cmake -S test -B test/build -G Ninja
    cmake --build test/build
    ctest --test-dir test/build --output-on-failure
    """
    run(["cmake", "-S", TEST_DIR, "-B", BUILD_DIR, "-G", "Ninja"])
    run(["cmake", "--build", BUILD_DIR])
    run(["ctest", "--test-dir", BUILD_DIR, "--output-on-failure"])


if __name__ == "__main__":
    sys.exit(main())