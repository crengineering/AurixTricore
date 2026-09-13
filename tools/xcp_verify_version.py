#!/usr/bin/env python3
"""Assert the running board's firmware version matches Version.h.

Why this exists
----------------
SYS1-001 B9 (2026-09-13): a flash carried v1.19.23 while Version.h already
said v1.19.24, because amk's incremental build silently skipped the three
translation units that #include Version.h (Cpu0_Main.c, Measurements.c,
Xcp.c) after a header-only edit -- "0 errors, 0 warnings" printed and the
mismatch was caught only by a human reading the version back over XCP.
build.bat now force-recompiles every TU that includes Version.h on every
build (same fix as LayoutAssert.c's forced recompile), which should make
this impossible -- but "should" is exactly the word that got skipped before,
so flash.bat also runs this script and refuses to call a flash successful
on a mismatch. No human read-back needed.

The offset of verMajor/verMinor/verStep inside Xcp_Data, and Xcp_Data's own
base address, are DERIVED the same way tools/gen_a2l.py derives every A2L
entry (CLAUDE.md: never hardcode a struct offset) -- so a struct edit that
moves these fields cannot silently desync this check from the real layout.

Usage:
    python tools/xcp_verify_version.py            # connect, compare, exit
    python tools/xcp_verify_version.py --host H --port P
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_a2l as a2l          # noqa: E402  (path must be set up first)
import xcp_read as xr          # noqa: E402  (reuses the connection recipe)


def board_version(host: str, port: int) -> tuple[int, int, int]:
    """Read (major, minor, step) from the live board's Xcp_Data.verMajor.."""
    fields, _size = a2l.parse_struct(a2l.BSW / "Measurements.h", "Xcp_Data")
    by_name = {f.name: f for f in fields}
    for name in ("verMajor", "verMinor", "verStep"):
        if name not in by_name:
            sys.exit(f"error: Xcp_Data has no field '{name}' -- "
                      "Measurements.h and this check have drifted apart")

    base = a2l.read_lsl_define("LCF_XCP_DATA_START")
    addr = base + by_name["verMajor"].offset
    # verMajor/verMinor/verStep are three consecutive uint8 (Measurements.h),
    # so one 3-byte SHORT_UPLOAD covers all of them.
    assert by_name["verMinor"].offset == by_name["verMajor"].offset + 1
    assert by_name["verStep"].offset == by_name["verMajor"].offset + 2

    with xr.connect(host, port) as x:
        x.connect()
        try:
            raw = bytes(x.shortUpload(3, addr, 0))
        finally:
            x.disconnect()
    return raw[0], raw[1], raw[2]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=xr.HOST)
    ap.add_argument("--port", type=int, default=xr.PORT)
    args = ap.parse_args()

    want = tuple(int(p) for p in a2l.parse_version(a2l.BSW / "Version.h").split("."))
    got = board_version(args.host, args.port)

    if got != want:
        print(f"error: board reports firmware v{got[0]}.{got[1]}.{got[2]}, "
              f"Version.h says v{want[0]}.{want[1]}.{want[2]} -- "
              "the flash did not carry the build you think it did "
              "(SYS1-001 B9: check for a skipped recompile, tools/gen_a2l.py "
              "--check, or a stale .hex).", file=sys.stderr)
        return 1

    print(f"board version confirmed: v{got[0]}.{got[1]}.{got[2]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
