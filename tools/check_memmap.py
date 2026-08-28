#!/usr/bin/env python3
"""Post-build .map verifier -- docs/MEMORY_PLACEMENT.md part 5 / T7.

Requires a build (reads the .map), so this is NOT an offline gate: it runs
from build.bat and as a flight-dev acceptance step, never in a2l.yml (which
must pass on a clean clone with no build -- see gen_a2l.py/check_docs.py,
which read Lcf_Tasking_Tricore_Tc.lsl instead).

What it asserts, all from the real linked output, never assumed:
  1. every pinned XCP block sits at its declared LCF_XCP_*_START address
     -- this is also what catches a stale .map: edit the .lsl and forget to
     rebuild, and the .map still shows the OLD address, which now disagrees
     with the .lsl you just read. That mismatch is exactly rule 1 failing.
  2. no two of the thirteen shared objects (seven XCP blocks, six LMU
     objects) overlap, given their real sizes
  3. every LMU object lies inside 0xB00F0000..0xB00FFFFF (lmuram_shared)
  4. the measurement block (Xcp_Data) has not outgrown its 256-byte slot

Usage:
    python tools/check_memmap.py                # uses the default .map path
    python tools/check_memmap.py --map PATH
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LSL_PATH = ROOT / "Lcf_Tasking_Tricore_Tc.lsl"
DEFAULT_MAP = ROOT / "TriCore Debug (TASKING)" / "AurixTricore.map"

# block -> .lsl define name -> the .bss section this project's naming
# convention gives it (docs/MEMORY_PLACEMENT.md T0/T2: `#pragma section
# farbss "X"` emits exactly `.bss.X`, measured, not guessed).
XCP_BLOCKS = {
    "Xcp_Data":      ("LCF_XCP_DATA_START",      ".bss.xcp_data"),
    "Xcp_Cal":       ("LCF_XCP_CAL_START",       ".bss.xcp_cal"),
    "Xcp_Nvm":       ("LCF_XCP_NVM_START",       ".bss.xcp_nvm"),
    "Xcp_Gpio":      ("LCF_XCP_GPIO_START",      ".bss.xcp_gpio"),
    "I2c_Debug":     ("LCF_XCP_I2CDBG_START",    ".bss.xcp_i2cdbg"),
    "Xcp_Fusion":    ("LCF_XCP_FUSION_START",    ".bss.xcp_fusion"),
    "Xcp_FusionCal": ("LCF_XCP_FUSIONCAL_START", ".bss.xcp_fusioncal"),
}

# LMU objects are NOT pinned (docs/MEMORY_PLACEMENT.md part 4: "free to
# float"), so there is no per-object address to assert -- only that every
# one of them lands inside the shared block. Section names, same convention.
LMU_SECTIONS = [
    ".bss.shared_lmu.corestats",
    ".bss.shared_lmu.navstate",
    ".bss.shared_lmu.barolatch",
    ".bss.shared_lmu.gnsslatch",
    ".bss.shared_lmu.maglatch",
    ".bss.shared_lmu.imuedge",
]

LMU_BASE = 0xB00F0000
LMU_SIZE = 0x00010000          # 64 K, lmuram_shared (Lcf_Tasking_Tricore_Tc.lsl)
XCP_DATA_SLOT = 256             # bytes between consecutive XCP block bases

# The .map's per-section placement row:
#   | mpe:<memory> | <group> | <section> (<id>) | <size> | <address> | <offset> | <align> |
# group can be blank (unrouted .bss). Reused shape, not the symbol-table row
# tools/xcp_read.py:46-47 uses (that one has no size column).
PLACEMENT_RE = re.compile(
    r"^\|\s*mpe:\S+\s*\|\s*\S*\s*\|\s*(\.\S+)\s*\(\d+\)\s*\|\s*"
    r"(0x[0-9a-fA-F]+)\s*\|\s*(0x[0-9a-fA-F]+)\s*\|"
)


def read(p: Path) -> str:
    data = p.read_bytes()
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return data.decode("latin-1")


def read_lsl_define(name: str) -> int:
    m = re.search(rf"^#define\s+{name}\s+(0x[0-9A-Fa-f]+)", read(LSL_PATH), re.M)
    if not m:
        raise SystemExit(f"error: {name} not found in {LSL_PATH.name}")
    return int(m.group(1), 16)


def parse_placements(map_path: Path) -> dict[str, tuple[int, int]]:
    """section name -> (size, address), first match per section wins (the
    .map lists both the run-time placement row and later summary rows for
    some sections; the placement row -- the one with a real memory pool and
    a byte offset -- always comes first)."""
    out: dict[str, tuple[int, int]] = {}
    for line in read(map_path).splitlines():
        m = PLACEMENT_RE.match(line)
        if not m:
            continue
        section, size_s, addr_s = m.group(1), m.group(2), m.group(3)
        if section not in out:
            out[section] = (int(size_s, 16), int(addr_s, 16))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", type=Path, default=DEFAULT_MAP,
                    help="path to AurixTricore.map (default: the TASKING build output)")
    args = ap.parse_args()

    if not args.map.exists():
        raise SystemExit(f"error: {args.map} does not exist -- build first "
                         f"(this check reads the linker output, it is not "
                         f"an offline gate)")

    placements = parse_placements(args.map)
    problems: list[str] = []

    # --- 1. every pinned XCP block sits at its declared address ------------
    xcp_objs: list[tuple[str, int, int]] = []   # (name, addr, size)
    for block, (lsl_define, section) in XCP_BLOCKS.items():
        want = read_lsl_define(lsl_define)
        if section not in placements:
            problems.append(f"{block}: section {section} not found in the "
                            f".map (build stale, or the object was renamed?)")
            continue
        size, addr = placements[section]
        if addr != want:
            problems.append(
                f"{block}: .lsl says {lsl_define}=0x{want:08X}, .map places "
                f"{section} at 0x{addr:08X} -- .lsl was edited without a "
                f"rebuild, or the group's run_addr stopped being honoured")
            continue
        xcp_objs.append((block, addr, size))

    # --- 3. every LMU object lies inside the shared block -------------------
    lmu_objs: list[tuple[str, int, int]] = []
    for section in LMU_SECTIONS:
        if section not in placements:
            problems.append(f"{section}: not found in the .map (build stale, "
                            f"or the object was renamed?)")
            continue
        size, addr = placements[section]
        if not (LMU_BASE <= addr and (addr + size) <= (LMU_BASE + LMU_SIZE)):
            problems.append(
                f"{section}: at 0x{addr:08X}, size {size} -- falls outside "
                f"lmuram_shared (0x{LMU_BASE:08X}..0x{LMU_BASE + LMU_SIZE - 1:08X})")
            continue
        lmu_objs.append((section, addr, size))

    # --- 2. no two of the thirteen objects overlap --------------------------
    all_objs = sorted(xcp_objs + lmu_objs, key=lambda o: o[1])
    for (name_a, addr_a, size_a), (name_b, addr_b, size_b) in zip(all_objs, all_objs[1:]):
        end_a = addr_a + size_a
        if end_a > addr_b:
            problems.append(
                f"overlap: {name_a} occupies 0x{addr_a:08X}..0x{end_a - 1:08X} "
                f"({size_a} B), {name_b} starts at 0x{addr_b:08X} -- "
                f"{end_a - addr_b} byte(s) shared")

    # --- 4. Xcp_Data has not outgrown its 256-byte slot ---------------------
    data_entry = next((o for o in xcp_objs if o[0] == "Xcp_Data"), None)
    if data_entry is not None:
        _, _, data_size = data_entry
        headroom = XCP_DATA_SLOT - data_size
        if headroom < 0:
            problems.append(
                f"Xcp_Data: {data_size} B, {-headroom} B OVER its "
                f"{XCP_DATA_SLOT} B slot before Xcp_Cal -- growing it further "
                f"collides with the next block")

    if problems:
        print("check_memmap: FAIL\n", file=sys.stderr)
        for p in problems:
            print(f"  {p}", file=sys.stderr)
        print(f"\n{len(problems)} problem(s), read from {args.map.name}",
              file=sys.stderr)
        return 1

    print(f"check_memmap: OK -- {len(xcp_objs)} XCP block(s) at their "
          f"pinned address, {len(lmu_objs)} LMU object(s) inside "
          f"lmuram_shared, no overlaps, Xcp_Data headroom "
          f"{XCP_DATA_SLOT - data_entry[2]} B "
          f"(read from {args.map.name})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
