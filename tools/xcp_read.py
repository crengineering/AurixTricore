"""Read any global variable off the running board over XCP/Ethernet, by name.

The XCP slave's SHORT_UPLOAD accepts an arbitrary address (only writes are
range-checked, see xcpWriteAllowed in Xcp.c). So ANY global in the map file can
be watched live -- no Xcp_Data field, no A2L entry, no AurixGUI change needed.
That is the fast path for self-chosen debug values during bring-up:

    volatile uint32 g_dbgFoo;   /* non-static, or it will not reach the map */

then build, flash, and:

    python tools/xcp_read.py g_dbgFoo
    python tools/xcp_read.py g_dbgFoo:u32 g_dbgBar:f32 --watch 0.5
    python tools/xcp_read.py 0x70030000:hex:64          # raw address + length
    python tools/xcp_read.py --find dbg                 # grep the map

Format is NAME[:TYPE] or 0xADDR[:TYPE][:LEN]. Types: u8 u16 u32 i8 i16 i32 f32
hex str (default u32).

!! The address of a symbol CHANGES ON EVERY BUILD. This tool re-reads the map
   each run for that reason -- never hardcode an address you looked up once.
   (tools/xcp_test.py still carries a stale 0x70000170 for a symbol that has
   since moved to 0x70000180.)
"""
import argparse
import re
import struct
import sys
import time
from pathlib import Path

from pyxcp.master import Master
from pyxcp.config import create_application_from_config

MAP_PATH = Path(__file__).resolve().parent.parent / "TriCore Debug (TASKING)" / "AurixTricore.map"
HOST, PORT = "192.168.0.10", 5555

# TASKING map symbol table rows:  | name | 0xaddress |  ... |
SYM_RE = re.compile(r"^\|\s*(\S+)\s*\|\s*(0x[0-9a-fA-F]+)\s*\|")

SIZES = {"u8": 1, "i8": 1, "u16": 2, "i16": 2, "u32": 4, "i32": 4, "f32": 4}
FMTS = {"u8": "<B", "i8": "<b", "u16": "<H", "i16": "<h",
        "u32": "<I", "i32": "<i", "f32": "<f"}


def load_symbols():
    if not MAP_PATH.exists():
        sys.exit(f"No map file at {MAP_PATH}\nBuild first (build.bat).")
    syms = {}
    for line in MAP_PATH.read_text(encoding="utf-8", errors="replace").splitlines():
        m = SYM_RE.match(line)
        if m:
            syms.setdefault(m.group(1), int(m.group(2), 16))
    return syms


def parse_target(spec, syms):
    """'name[:type]' or '0xaddr[:type][:len]' -> (label, addr, type, nbytes)."""
    parts = spec.split(":")
    head = parts[0]
    typ = parts[1] if len(parts) > 1 else "u32"

    if head.lower().startswith("0x"):
        addr = int(head, 16)
    else:
        if head not in syms:
            near = [s for s in syms if head.lower() in s.lower()][:8]
            hint = ("\nDid you mean: " + ", ".join(near)) if near else ""
            sys.exit(f"Symbol '{head}' not in the map."
                     f"\nIt must be non-static to appear there.{hint}")
        addr = syms[head]

    if typ in ("hex", "str"):
        nbytes = int(parts[2]) if len(parts) > 2 else 16
    elif typ in SIZES:
        nbytes = SIZES[typ]
    else:
        sys.exit(f"Unknown type '{typ}' (use {' '.join(list(SIZES) + ['hex', 'str'])})")

    if nbytes > 63:
        sys.exit(f"{spec}: {nbytes} bytes exceeds the 63-byte SHORT_UPLOAD limit "
                 f"(XCP_MAX_CTO - 1). Split it.")
    return head, addr, typ, nbytes


def render(raw, typ):
    if typ == "hex":
        return " ".join(f"{b:02X}" for b in raw)
    if typ == "str":
        return raw.split(b"\x00")[0].decode("ascii", errors="replace")
    val = struct.unpack(FMTS[typ], raw)[0]
    if typ == "f32":
        return f"{val:.6g}"
    if typ.startswith("u"):
        return f"{val} (0x{val:0{2 * SIZES[typ]}X})"
    return str(val)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("targets", nargs="*", help="NAME[:TYPE] or 0xADDR[:TYPE][:LEN]")
    ap.add_argument("--watch", type=float, metavar="SEC",
                    help="poll every SEC seconds until Ctrl-C")
    ap.add_argument("--find", metavar="SUBSTR", help="list matching map symbols and exit")
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    args = ap.parse_args()

    syms = load_symbols()

    if args.find:
        hits = sorted(s for s in syms if args.find.lower() in s.lower())
        for s in hits:
            print(f"  0x{syms[s]:08X}  {s}")
        print(f"{len(hits)} symbol(s) matching '{args.find}'")
        return
    if not args.targets:
        ap.error("give at least one target, or --find")

    targets = [parse_target(t, syms) for t in args.targets]

    conf = create_application_from_config({
        "Transport": {"Eth": {"host": args.host, "port": args.port,
                              "protocol": "UDP", "ipv6": False}},
    })

    with Master("eth", config=conf) as x:
        x.connect()
        try:
            while True:
                cols = []
                for label, addr, typ, nbytes in targets:
                    raw = bytes(x.shortUpload(nbytes, addr, 0))
                    cols.append(f"{label}@0x{addr:08X} = {render(raw, typ)}")
                print(f"[{time.strftime('%H:%M:%S')}] " + "   ".join(cols), flush=True)
                if args.watch is None:
                    break
                time.sleep(args.watch)
        except KeyboardInterrupt:
            print("\n--- stopped ---", file=sys.stderr)
        finally:
            x.disconnect()


if __name__ == "__main__":
    main()
