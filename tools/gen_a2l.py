#!/usr/bin/env python3
"""Generate docs/AurixTricore.a2l from the firmware headers.

Why this exists
---------------
The A2L has to agree with four C structs on the exact byte offset of every
field. Kept by hand it drifts, and the failure is silent: the GUI parses the
file, computes (ECU_ADDRESS - block base), and reads the wrong bytes -- or none
at all. Two real cases: the file sat at "firmware v1.3.0" for fourteen releases
while sensors were added, and the magnetometer read a constant 0 because the
block had outgrown what the GUI fetched.

So the layout is DERIVED here and never typed:

    Xcp_Data  0x70030000  Measurements.h  -> MEASUREMENT
    Xcp_Cal   0x70030100  Diagnostics.h   -> CHARACTERISTIC  (CAL_*)
    Xcp_Nvm   0x70030200  Nvm.h           -> CHARACTERISTIC  (NVM_*)
    Xcp_Gpio  0x70030300  gpio.h          -> CHARACTERISTIC  (GPIO_*)
    Xcp_Fusion 0x70030500 Measurements.h  -> MEASUREMENT
    DIAG_* defines        Diagnostics.h   -> MEASUREMENT with BIT_MASK

What is NOT derived is the prose: descriptions, units and display limits carry
knowledge that simply is not in the C code ("~0.48 G in Munich", "LEVEL ONLY,
uncalibrated"). Those live in tools/a2l_meta.json, keyed by struct field. A field
present in the struct but missing from the sidecar still appears in the output --
with a placeholder description and a loud warning -- so a new signal can never be
silently dropped, which is the whole point.

Usage
-----
    python tools/gen_a2l.py                 # write docs/AurixTricore.a2l
    python tools/gen_a2l.py --check         # exit 1 if the file is stale (CI)
    python tools/gen_a2l.py --seed          # rebuild the sidecar from the
                                            # existing A2L (one-off migration)
    python tools/gen_a2l.py --stdout        # print, write nothing
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BSW = ROOT / "src" / "bsw"
A2L_PATH = ROOT / "docs" / "AurixTricore.a2l"
META_PATH = ROOT / "tools" / "a2l_meta.json"

# ---------------------------------------------------------------------------
# C type model. Sizes and alignments are TriCore/TASKING: scalars are aligned
# to their own width, so natural packing reproduces the documented offsets.
# ---------------------------------------------------------------------------
CTYPES = {
    "uint8":   (1, "UBYTE",        "RL_UBYTE"),
    "boolean": (1, "UBYTE",        "RL_UBYTE"),
    "uint16":  (2, "UWORD",        "RL_UWORD"),
    "uint32":  (4, "ULONG",        "RL_ULONG"),
    "sint32":  (4, "SLONG",        "RL_SLONG"),
    "float32": (4, "FLOAT32_IEEE", "RL_FLOAT32"),
}

BLOCKS = {
    "Xcp_Data": ("Measurements.h", "XCP_DATA_ADDR"),
    "Xcp_Cal":  ("Diagnostics.h",  "XCP_CAL_ADDR"),
    "Xcp_Nvm":  ("Nvm.h",          "XCP_NVM_ADDR"),
    "Xcp_Gpio": ("gpio.h",         "XCP_GPIO_ADDR"),
    "Xcp_Fusion": ("Measurements.h", "XCP_FUSION_ADDR"),
}


class Field:
    """One struct member, resolved to an absolute address."""

    def __init__(self, name: str, ctype: str, count: int, offset: int):
        self.name = name
        self.ctype = ctype
        self.count = count          # 1 for scalars, N for arrays
        self.offset = offset

    @property
    def size(self) -> int:
        return CTYPES[self.ctype][0]

    @property
    def a2l_type(self) -> str:
        return CTYPES[self.ctype][1]

    @property
    def record_layout(self) -> str:
        return CTYPES[self.ctype][2]


def read(path: Path) -> str:
    """Source files are a mix of UTF-8 and Latin-1 (German comments, em-dashes),
    and Windows Python defaults to cp1252. Only structure is parsed out of them,
    so a tolerant decode is fine."""
    data = path.read_bytes()
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return data.decode("latin-1")


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


def read_define(header: Path, name: str) -> int:
    """Value of a #define <name> 0x...u"""
    m = re.search(rf"^#define\s+{name}\s+(0x[0-9A-Fa-f]+)u?", read(header), re.M)
    if not m:
        raise SystemExit(f"error: {name} not found in {header.name}")
    return int(m.group(1), 16)


def parse_struct(header: Path, tag: str) -> tuple[list[Field], int]:
    """Members of `typedef struct { ... } <tag>;` with natural alignment.

    Returns (fields, total size). Array members are kept as one Field with
    count > 1; the caller expands them, because the A2L names elements
    individually (coreExecUs[6] -> Core0ExecTime .. Core5ExecTime).
    """
    src = strip_comments(read(header))
    # [^{}]* rather than .*? : gpio.h declares gpio_cfg_t before Xcp_Gpio, and a
    # lazy any-match would start at the first struct and run through to our tag.
    # None of these blocks nest, so excluding braces pins it to the right one.
    m = re.search(r"typedef\s+struct\s*\{([^{}]*)\}\s*" + tag + r"\s*;", src, re.S)
    if not m:
        raise SystemExit(f"error: struct {tag} not found in {header.name}")

    fields: list[Field] = []
    offset = 0
    for decl in m.group(1).split(";"):
        decl = decl.strip()
        if not decl:
            continue
        mm = re.match(r"^(\w+)\s+(\w+)\s*(?:\[\s*([\w＿]+)\s*\])?$", decl)
        if not mm:
            raise SystemExit(f"error: cannot parse member '{decl}' of {tag}")

        ctype, name, dim = mm.group(1), mm.group(2), mm.group(3)
        if ctype not in CTYPES:
            raise SystemExit(f"error: unsupported type '{ctype}' in {tag}.{name}")

        count = 1
        if dim is not None:
            count = int(dim) if dim.isdigit() else resolve_enum_count(dim)

        size = CTYPES[ctype][0]
        offset += (-offset) % size              # natural alignment
        fields.append(Field(name, ctype, count, offset))
        offset += size * count

    offset += (-offset) % 4                     # trailing struct padding
    return fields, offset


def resolve_enum_count(symbol: str) -> int:
    """Array dimension given as an enum terminator, e.g. GPIO_P_00_END == 16.

    Scans every enum in every BSW header for the symbol and evaluates its value
    the way C does -- start at 0, increment, honour explicit '= N' -- so a
    terminator works whether or not the members are numbered by hand.
    """
    for header in sorted(BSW.glob("*.h")):
        src = strip_comments(read(header))
        for body in re.findall(r"typedef\s+enum\s*\{([^{}]*)\}", src, re.S):
            value = 0
            for item in body.split(","):
                item = item.strip()
                if not item:
                    continue
                if "=" in item:
                    name, expr = (p.strip() for p in item.split("=", 1))
                    try:
                        value = int(expr, 0)
                    except ValueError:
                        value = None            # non-literal: give up on this enum
                        break
                else:
                    name = item
                if name == symbol:
                    return value
                value += 1
    raise SystemExit(f"error: cannot resolve array dimension '{symbol}'")


def parse_diag_bits(header: Path) -> list[tuple[str, int]]:
    """(macro name, mask) for every DIAG_* define, in bit order."""
    bits = []
    for m in re.finditer(r"^#define\s+(DIAG_\w+)\s+(0x[0-9A-Fa-f]+)u?", read(header), re.M):
        bits.append((m.group(1), int(m.group(2), 16)))
    return sorted(bits, key=lambda b: b[1])


def parse_version(header: Path) -> str:
    src = read(header)

    def val(macro: str) -> str:
        m = re.search(rf"^#define\s+{macro}\s+(\d+)", src, re.M)
        return m.group(1) if m else "0"

    return f"{val('SW_VERSION_MAJOR')}.{val('SW_VERSION_MINOR')}.{val('SW_VERSION_STEP')}"


# ---------------------------------------------------------------------------
# Metadata sidecar
# ---------------------------------------------------------------------------
def load_meta() -> dict:
    if not META_PATH.exists():
        raise SystemExit(
            f"error: {META_PATH.relative_to(ROOT)} missing.\n"
            f"       Run 'python tools/gen_a2l.py --seed' once to build it from\n"
            f"       the existing A2L, then re-run."
        )
    return json.loads(read(META_PATH))


def meta_for(meta: dict, block: str, field: str) -> dict:
    return meta.get(block, {}).get(field, {})


def expand(entry: dict, field: Field, index: int | None) -> dict:
    """Resolve the index placeholders for one array element.

    {i}  -> 0, 1, 2 ...      (Core{i}ExecTime  -> Core0ExecTime)
    {i2} -> 00, 01, 02 ...   (GPIO_P00_{i2}_state -> GPIO_P00_01_state)
    """
    out = dict(entry)
    i = "" if index is None else str(index)
    i2 = "" if index is None else f"{index:02d}"
    for key in ("name", "desc"):
        if key in out:
            out[key] = out[key].replace("{i2}", i2).replace("{i}", i)
    if "name" not in out:
        out["name"] = field.name + ("" if index is None else f"_{index}")
    return out


# ---------------------------------------------------------------------------
# Emitters
# ---------------------------------------------------------------------------
def limits(entry: dict, field: Field) -> tuple[str, str]:
    if "lo" in entry and "hi" in entry:
        return str(entry["lo"]), str(entry["hi"])
    # Sensible default from the type, so a field with no metadata still works.
    if field.ctype == "float32":
        return "-1e12", "1e12"
    span = {1: 255, 2: 65535, 4: 4294967295}[field.size]
    return "0", str(span)


def emit_measurement(entry: dict, field: Field, addr: int, warn: list[str]) -> str:
    lo, hi = limits(entry, field)
    desc = entry.get("desc")
    if desc is None:
        desc = f"{field.name} (no description in a2l_meta.json)"
        warn.append(f"{entry['name']}: no metadata, using a placeholder description")

    out = [f'    /begin MEASUREMENT {entry["name"]} "{desc}"',
           f"      {field.a2l_type} NO_COMPU_METHOD 0 0 {lo} {hi}",
           f"      ECU_ADDRESS 0x{addr:08X}"]
    if entry.get("unit"):
        out.append(f'      PHYS_UNIT "{entry["unit"]}"')
    out.append("    /end MEASUREMENT")
    return "\n".join(out)


def emit_bit_measurement(entry: dict, addr: int, mask: int) -> str:
    return "\n".join([
        f'    /begin MEASUREMENT {entry["name"]} "{entry["desc"]}"',
        "      ULONG NO_COMPU_METHOD 0 0 0 1",
        f"      ECU_ADDRESS 0x{addr:08X}",
        f"      BIT_MASK 0x{mask:08X}",
        "    /end MEASUREMENT",
    ])


def emit_characteristic(entry: dict, field: Field, addr: int, warn: list[str]) -> str:
    lo, hi = limits(entry, field)
    desc = entry.get("desc")
    if desc is None:
        desc = f"{field.name} (no description in a2l_meta.json)"
        warn.append(f"{entry['name']}: no metadata, using a placeholder description")

    out = [f'    /begin CHARACTERISTIC {entry["name"]} "{desc}"',
           f"      VALUE 0x{addr:08X} {field.record_layout} "
           f"{entry.get('maxdiff', hi)} NO_COMPU_METHOD {lo} {hi}"]
    if entry.get("unit"):
        out.append(f'      PHYS_UNIT "{entry["unit"]}"')
    out.append("    /end CHARACTERISTIC")
    return "\n".join(out)


def block_objects(block: str, meta: dict, warn: list[str], kind: str) -> tuple[list[str], int]:
    header, addr_macro = BLOCKS[block]
    hdr = BSW / header
    base = read_define(hdr, addr_macro)
    fields, size = parse_struct(hdr, block)

    out: list[str] = []
    for f in fields:
        entry = meta_for(meta, block, f.name)
        if entry.get("skip"):
            continue
        skip_idx = set(entry.get("skip_index", []))
        for idx in range(f.count):
            if idx in skip_idx:
                continue                    # element deliberately not exposed
            e = expand(entry, f, None if f.count == 1 else idx)
            a = base + f.offset + idx * f.size
            out.append(emit_measurement(e, f, a, warn) if kind == "meas"
                       else emit_characteristic(e, f, a, warn))
    return out, size


PREAMBLE = """\
/* ASAP2/A2L description of the AurixTricore XCP slave (firmware v{version}).
 *
 * GENERATED by tools/gen_a2l.py -- do not edit by hand.
 * Layout comes from the C structs; descriptions, units and limits come from
 * tools/a2l_meta.json. Regenerate after changing either:
 *     python tools/gen_a2l.py
 * CI runs 'python tools/gen_a2l.py --check' to catch a stale file.
 *
 * All objects live at fixed addresses (TASKING __at), so this file does not
 * depend on the link run:
 *   Xcp_Data  (measurements)          0x70030000, {data_size} bytes, Measurements.h
 *   Xcp_Cal   (calibration, RAM only) 0x70030100, {cal_size} bytes, Diagnostics.h
 *   Xcp_Nvm   (persistent, DFLASH)    0x70030200, {nvm_size} bytes, Nvm.h
 *   Xcp_Gpio  (GPIO control, RAM only) 0x70030300, {gpio_size} bytes, gpio.h
 *   Xcp_Fusion (navigation state)     0x70030500, {fusion_size} bytes, Measurements.h
 * Transport: XCP on UDP/IP, port 5555, station 192.168.0.10.
 * DAQ: dynamic, 1 list, event channel 0 = 100 ms task.
 */
ASAP2_VERSION 1 61

/begin PROJECT AurixTricore "TC399 TriBoard bare-metal project"

  /begin MODULE XCP "AurixTricore XCP slave"

    /begin MOD_COMMON ""
      BYTE_ORDER MSB_LAST
      ALIGNMENT_BYTE 1
      ALIGNMENT_WORD 2
      ALIGNMENT_LONG 4
      ALIGNMENT_FLOAT32_IEEE 4
    /end MOD_COMMON

    /begin MOD_PAR "AurixTricore v{version}"
    /end MOD_PAR

    /begin IF_DATA XCP
      /begin PROTOCOL_LAYER
        0x0100                  /* protocol layer version 1.0             */
        1000 1000 1000 1000 1000 1000 1000   /* T1..T7 timeouts [ms]      */
        64                      /* MAX_CTO                                */
        64                      /* MAX_DTO                                */
        BYTE_ORDER_MSB_LAST
        ADDRESS_GRANULARITY_BYTE
        OPTIONAL_CMD GET_ID
        OPTIONAL_CMD SET_MTA
        OPTIONAL_CMD UPLOAD
        OPTIONAL_CMD SHORT_UPLOAD
        OPTIONAL_CMD DOWNLOAD
        OPTIONAL_CMD SHORT_DOWNLOAD
        OPTIONAL_CMD GET_DAQ_PROCESSOR_INFO
        OPTIONAL_CMD GET_DAQ_RESOLUTION_INFO
        OPTIONAL_CMD FREE_DAQ
        OPTIONAL_CMD ALLOC_DAQ
        OPTIONAL_CMD ALLOC_ODT
        OPTIONAL_CMD ALLOC_ODT_ENTRY
      /end PROTOCOL_LAYER
      /begin DAQ
        DYNAMIC
        1                       /* MAX_DAQ                                */
        1                       /* MAX_EVENT_CHANNEL                      */
        0                       /* MIN_DAQ                                */
        OPTIMISATION_TYPE_DEFAULT
        ADDRESS_EXTENSION_FREE
        IDENTIFICATION_FIELD_TYPE_ABSOLUTE
        GRANULARITY_ODT_ENTRY_SIZE_DAQ_BYTE
        63                      /* MAX_ODT_ENTRY_SIZE_DAQ                 */
        NO_OVERLOAD_INDICATION
        /begin EVENT "Task100ms" "100ms" 0x0000 DAQ 0xFF 100 6 0x00
        /end EVENT
      /end DAQ
      /begin XCP_ON_UDP_IP
        0x0100                  /* transport layer version 1.0            */
        5555                    /* port                                   */
        ADDRESS "192.168.0.10"
      /end XCP_ON_UDP_IP
    /end IF_DATA

    /begin RECORD_LAYOUT RL_FLOAT32
      FNC_VALUES 1 FLOAT32_IEEE ROW_DIR DIRECT
    /end RECORD_LAYOUT

    /begin RECORD_LAYOUT RL_ULONG
      FNC_VALUES 1 ULONG ROW_DIR DIRECT
    /end RECORD_LAYOUT

    /begin RECORD_LAYOUT RL_UWORD
      FNC_VALUES 1 UWORD ROW_DIR DIRECT
    /end RECORD_LAYOUT

    /begin RECORD_LAYOUT RL_UBYTE
      FNC_VALUES 1 UBYTE ROW_DIR DIRECT
    /end RECORD_LAYOUT
"""


def generate() -> tuple[str, list[str]]:
    meta = load_meta()
    warn: list[str] = []

    data_objs, data_size = block_objects("Xcp_Data", meta, warn, "meas")
    cal_objs, cal_size = block_objects("Xcp_Cal", meta, warn, "char")
    nvm_objs, nvm_size = block_objects("Xcp_Nvm", meta, warn, "char")
    gpio_objs, gpio_size = block_objects("Xcp_Gpio", meta, warn, "char")
    fusion_objs, fusion_size = block_objects("Xcp_Fusion", meta, warn, "meas")

    diag_base = read_define(BSW / "Measurements.h", "XCP_DATA_ADDR")
    diag_fields, _ = parse_struct(BSW / "Measurements.h", "Xcp_Data")
    diag_off = next(f.offset for f in diag_fields if f.name == "diagStatus")

    diag_objs = []
    for macro, mask in parse_diag_bits(BSW / "Diagnostics.h"):
        entry = meta_for(meta, "diag_bits", macro)
        if entry.get("skip"):
            continue
        if "name" not in entry:
            entry = dict(entry, name=macro,
                         desc=entry.get("desc", f"{macro} (no metadata)"))
            warn.append(f"{macro}: no metadata, using the macro name")
        diag_objs.append(emit_bit_measurement(entry, diag_base + diag_off, mask))

    parts = [PREAMBLE.format(version=parse_version(BSW / "Version.h"),
                             data_size=data_size, cal_size=cal_size,
                             nvm_size=nvm_size, gpio_size=gpio_size,
                             fusion_size=fusion_size)]
    parts.append("\n    /* ------------------------------------------------------------------ */"
                 "\n    /* Measurements: Xcp_Data @ 0x70030000 (see Measurements.h)           */"
                 "\n    /* ------------------------------------------------------------------ */\n")
    parts.append("\n\n".join(data_objs))
    parts.append("\n\n    /* individual diagnostics bits (BIT_MASK views on DiagStatus) */\n")
    parts.append("\n\n".join(diag_objs))
    parts.append("\n\n    /* Calibration: Xcp_Cal @ 0x70030100, RAM only (see Diagnostics.h) */\n")
    parts.append("\n\n".join(cal_objs))
    parts.append("\n\n    /* Persistent: Xcp_Nvm @ 0x70030200, DFLASH (see Nvm.h) */\n")
    parts.append("\n\n".join(nvm_objs))
    parts.append("\n\n    /* GPIO control: Xcp_Gpio @ 0x70030300, RAM only (see gpio.h) */\n")
    parts.append("\n\n".join(gpio_objs))
    parts.append("\n\n    /* Navigation state: Xcp_Fusion @ 0x70030500 (see Measurements.h) */\n")
    parts.append("\n\n".join(fusion_objs))
    parts.append("\n\n  /end MODULE\n/end PROJECT\n")
    return "".join(parts), warn


# ---------------------------------------------------------------------------
# Sidecar seeding: lift the hand-written prose out of the existing A2L
# ---------------------------------------------------------------------------
OBJ_RE = (r"/begin (MEASUREMENT|CHARACTERISTIC)\s+(\S+)\s+\"([^\"]*)\"(.*?)/end \1")


def a2l_objects(text: str) -> tuple[dict[int, dict], dict[int, dict]]:
    """Parse an existing A2L into {address: entry} and {bitmask: entry}.

    Matching by ADDRESS rather than by name is what makes seeding reliable: the
    A2L identifier (SwVersionMajor) rarely resembles the struct field it
    describes (verMajor), but the address is unambiguous.
    """
    by_addr: dict[int, dict] = {}
    by_mask: dict[int, dict] = {}

    for kind, name, desc, body in re.findall(OBJ_RE, text, re.S):
        entry: dict = {"name": name, "desc": desc}
        u = re.search(r'PHYS_UNIT\s+"([^"]*)"', body)
        if u:
            entry["unit"] = u.group(1)

        bm = re.search(r"BIT_MASK\s+(0x[0-9A-Fa-f]+)", body)
        if bm:
            by_mask[int(bm.group(1), 16)] = entry
            continue

        if kind == "MEASUREMENT":
            addr = re.search(r"ECU_ADDRESS\s+(0x[0-9A-Fa-f]+)", body)
            nums = re.search(r"NO_COMPU_METHOD\s+\S+\s+\S+\s+(\S+)\s+(\S+)", body)
            if nums:
                entry["lo"], entry["hi"] = nums.groups()
        else:
            addr = re.search(r"VALUE\s+(0x[0-9A-Fa-f]+)", body)
            md = re.search(r"VALUE\s+\S+\s+\S+\s+(\S+)\s+NO_COMPU_METHOD", body)
            nums = re.search(r"NO_COMPU_METHOD\s+(\S+)\s+(\S+)", body)
            if md:
                entry["maxdiff"] = md.group(1)
            if nums:
                entry["lo"], entry["hi"] = nums.groups()
        if addr:
            by_addr[int(addr.group(1), 16)] = entry

    return by_addr, by_mask


def seed() -> None:
    """Rebuild tools/a2l_meta.json from the existing A2L.

    One-off migration, kept in the tool so it is reproducible. Every struct
    field's address is computed and looked up in the old file, so all the
    hand-written prose transfers without being retyped. Array fields get a name
    template derived from two consecutive elements ({i} / {i2}), and elements
    the old file omitted become "skip_index".
    """
    if not A2L_PATH.exists():
        raise SystemExit("error: no existing A2L to seed from")
    by_addr, by_mask = a2l_objects(read(A2L_PATH))

    meta: dict = {}
    for block, (hdr, addr_macro) in BLOCKS.items():
        header = BSW / hdr
        base = read_define(header, addr_macro)
        fields, _ = parse_struct(header, block)
        meta[block] = {}

        for f in fields:
            if f.name == "magic" or "eserved" in f.name:
                meta[block][f.name] = {"skip": True}
                continue

            present = [i for i in range(f.count)
                       if base + f.offset + i * f.size in by_addr]
            if not present:
                meta[block][f.name] = {"__TODO__": "not in the previous A2L"}
                continue

            entry = dict(by_addr[base + f.offset + present[0] * f.size])
            if f.count > 1:
                n0 = entry["name"]
                if len(present) > 1:
                    n1 = by_addr[base + f.offset + present[1] * f.size]["name"]
                    # first differing run of characters is the index
                    for a, b in zip(re.findall(r"\d+|\D+", n0),
                                    re.findall(r"\d+|\D+", n1)):
                        if a != b:
                            entry["name"] = n0.replace(
                                a, "{i2}" if len(a) == 2 else "{i}", 1)
                            break
                entry["desc"] = re.sub(rf"\b{present[0]}\b", "{i}",
                                       entry["desc"], count=1)
                missing = [i for i in range(f.count) if i not in present]
                if missing:
                    entry["skip_index"] = missing
            meta[block][f.name] = entry

    meta["diag_bits"] = {}
    for macro, mask in parse_diag_bits(BSW / "Diagnostics.h"):
        meta["diag_bits"][macro] = by_mask.get(
            mask, {"__TODO__": f"mask {mask:#010x} not in the previous A2L"})

    META_PATH.write_text(json.dumps(meta, indent=2) + "\n",
                         encoding="utf-8", newline="\n")
    todo = [f"{b}.{k}" for b, v in meta.items() for k, e in v.items()
            if "__TODO__" in e]
    print(f"wrote {META_PATH.relative_to(ROOT)}")
    if todo:
        print("fields with no match in the previous A2L (describe them by hand):")
        for t in todo:
            print(f"  {t}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if docs/AurixTricore.a2l differs from generated")
    ap.add_argument("--stdout", action="store_true", help="print, write nothing")
    ap.add_argument("--seed", action="store_true",
                    help="dump descriptions from the existing A2L, for migration")
    args = ap.parse_args()

    if args.seed:
        seed()
        return 0

    text, warn = generate()
    for w in warn:
        print(f"warning: {w}", file=sys.stderr)

    if args.stdout:
        sys.stdout.write(text)
        return 0

    if args.check:
        current = read(A2L_PATH) if A2L_PATH.exists() else ""
        if current == text:
            print(f"{A2L_PATH.relative_to(ROOT)} is up to date")
            return 0
        print(f"error: {A2L_PATH.relative_to(ROOT)} is STALE.\n"
              f"       Run 'python tools/gen_a2l.py' and commit the result.",
              file=sys.stderr)
        import difflib
        diff = difflib.unified_diff(current.splitlines(True), text.splitlines(True),
                                    "committed", "generated", n=2)
        sys.stderr.writelines(list(diff)[:80])
        return 1

    A2L_PATH.write_text(text, encoding="utf-8", newline="\n")
    n_meas = text.count("/begin MEASUREMENT")
    n_char = text.count("/begin CHARACTERISTIC")
    print(f"wrote {A2L_PATH.relative_to(ROOT)}: "
          f"{n_meas} measurements, {n_char} characteristics"
          + (f", {len(warn)} warnings" if warn else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
