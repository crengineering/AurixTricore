#!/usr/bin/env python3
"""Consistency check across the markdown documentation.

Run before opening a pull request -- a hook does this automatically, see
.claude/settings.json.

WHY THIS EXISTS. Documentation drifts silently, and the failure is invisible
until someone acts on a stale claim. Real examples from this repository, each
found by hand long after the fact:

  - README claimed v1.17.0 for fourteen releases while sensors were added
  - DIAGNOSTICS.md described Xcp_Nvm as 12 bytes when it was 44, and its field
    table was missing a parameter entirely
  - BMP581.md documented a filter register value that had been changed
  - four docs referenced files that a branch did not contain
  - PINNING.md said the attitude fields "are published as zero", years after
    they stopped being

Every one of those is mechanically checkable. None of them needs judgement.

WHAT IT DOES NOT DO. It cannot tell you a sentence is wrong, only that a fact
it can cross-reference disagrees with the code. Passing this is necessary, not
sufficient -- read the diff.

    python tools/check_docs.py            # report and exit non-zero on failure
    python tools/check_docs.py --quiet    # only print problems
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BSW = ROOT / "src" / "bsw"
LSL_PATH = ROOT / "Lcf_Tasking_Tricore_Tc.lsl"

problems: list[str] = []
checked = 0


def fail(where: str, msg: str) -> None:
    problems.append(f"{where}: {msg}")


def read(p: Path) -> str:
    """Docs and sources mix UTF-8 and Latin-1 (German comments, em dashes)."""
    b = p.read_bytes()
    try:
        return b.decode("utf-8")
    except UnicodeDecodeError:
        return b.decode("latin-1")


def md_files() -> list[Path]:
    out = [p for p in ROOT.glob("*.md")]
    out += [p for p in (ROOT / "docs").glob("*.md")]
    out += [p for p in (ROOT / "src").rglob("*.md")]
    return sorted(out)


# ---------------------------------------------------------------------------
# 1. every referenced doc exists
# ---------------------------------------------------------------------------
def check_doc_references() -> None:
    global checked
    ref = re.compile(r"`?(docs/[A-Za-z_0-9]+\.md)`?")
    for f in md_files():
        text = read(f)
        for m in sorted(set(ref.findall(text))):
            checked += 1
            if not (ROOT / m).exists():
                fail(f.relative_to(ROOT).as_posix(),
                     f"references {m}, which does not exist")


# ---------------------------------------------------------------------------
# 2. the firmware version in prose matches Version.h
# ---------------------------------------------------------------------------
def firmware_version() -> str | None:
    src = read(BSW / "Version.h")
    parts = []
    for name in ("SW_VERSION_MAJOR", "SW_VERSION_MINOR", "SW_VERSION_STEP"):
        m = re.search(rf"#define\s+{name}\s+(\d+)", src)
        if not m:
            return None
        parts.append(m.group(1))
    return ".".join(parts)


def check_version() -> None:
    global checked
    ver = firmware_version()
    if ver is None:
        fail("src/bsw/Version.h", "cannot parse the version macros")
        return

    # Only the README states the current version as a fact about the build.
    # Other documents mention versions historically ("added in v1.15.0"), which
    # is correct and must not be flagged.
    text = read(ROOT / "README.md")
    m = re.search(r"\*\*Firmware version: v([0-9.]+)\*\*", text)
    checked += 1
    if not m:
        fail("README.md", "no '**Firmware version: vX.Y.Z**' line found")
    elif m.group(1) != ver:
        fail("README.md",
             f"says firmware v{m.group(1)}, Version.h says v{ver}")


# ---------------------------------------------------------------------------
# 3. documented XCP block addresses match the .lsl (docs/MEMORY_PLACEMENT.md
#    T6: the address SSoT moved from the header XCP_*_ADDR macros -- deleted,
#    T5 -- to the LCF_XCP_*_START defines in Lcf_Tasking_Tricore_Tc.lsl)
# ---------------------------------------------------------------------------
def check_block_addresses() -> None:
    global checked
    addrs: dict[str, str] = {}
    for name, val in re.findall(
            r"#define\s+(LCF_XCP_[A-Z_]*_START)\s+(0x[0-9A-Fa-f]+)",
            read(LSL_PATH)):
        addrs[name] = val.lower()

    # struct tag -> the .lsl define that pins it
    pairs = {
        "Xcp_Data": "LCF_XCP_DATA_START",
        "Xcp_Cal": "LCF_XCP_CAL_START",
        "Xcp_Nvm": "LCF_XCP_NVM_START",
        "Xcp_Gpio": "LCF_XCP_GPIO_START",
        "Xcp_Fusion": "LCF_XCP_FUSION_START",
        "Xcp_FusionCal": "LCF_XCP_FUSIONCAL_START",
    }

    # T6: a define missing from the .lsl means the SSoT itself is broken --
    # that must fail loudly, not leave every per-block check below silently
    # skipped the way a missing header macro used to (found during T6: this
    # checker quietly stopped checking a block whose macro had already been
    # deleted, rather than reporting anything wrong -- worse than failing).
    checked += 1
    missing = sorted(macro for macro in pairs.values() if macro not in addrs)
    if missing:
        fail(LSL_PATH.name,
             "missing define(s), address cross-check cannot run: "
             + ", ".join(missing))
        return

    # Longest tag first, and matched with a right-hand word boundary, so
    # Xcp_Fusion never matches inside Xcp_FusionCal. That false positive is
    # exactly the kind that gets a hook switched off.
    ordered = sorted(pairs.items(), key=lambda kv: -len(kv[0]))

    for f in md_files():
        for line in read(f).splitlines():
            # Only lines that BIND a tag to an address: the tag, then at most a
            # little punctuation, then the address. Prose that happens to name a
            # block and a different address ("Xcp_Data had 8 bytes left, so it
            # went to 0x70030500") is not an assertion about that block.
            for tag, macro in ordered:
                want = addrs[macro]
                # Only SEPARATORS may sit between the tag and the address --
                # backticks, spaces, an @, a dash, "at". Prose in between means
                # the line is telling a story, not binding a block to an
                # address: "Xcp_Data had 8 bytes left, so it went to
                # 0x70030500" names both and asserts neither.
                m = re.search(rf"{tag}(?![A-Za-z0-9_])"
                              r"[`\s@:|—–-]*(?:at\s+)?[`\s]*"
                              r"(0x7003[0-9A-Fa-f]{4})", line)
                if not m:
                    continue
                checked += 1
                if m.group(1).lower() != want:
                    fail(f.relative_to(ROOT).as_posix(),
                         f"{tag} shown at {m.group(1)}, {macro} is {want}")
                break       # longest tag on this line wins


# ---------------------------------------------------------------------------
# 4. documented register values match the driver
# ---------------------------------------------------------------------------
def check_register_values() -> None:
    global checked
    defines: dict[str, str] = {}
    for c in BSW.glob("*.c"):
        for name, val in re.findall(
                r"#define\s+([A-Z][A-Z0-9_]*_VAL)\s+\((0x[0-9A-Fa-f]+)u?\)",
                read(c)):
            defines[name] = val.lower()

    # A doc table row like: | `DSP_IIR` 0x31 | `0x0A` | ...
    row = re.compile(r"\|\s*`?([A-Z][A-Z0-9_]*)`?\s+0x[0-9A-Fa-f]+\s*\|\s*`(0x[0-9A-Fa-f]+)`")
    for f in md_files():
        for reg, shown in row.findall(read(f)):
            key = f"BMP581_{reg}_VAL"
            if key not in defines:
                continue
            checked += 1
            if shown.lower() != defines[key]:
                fail(f.relative_to(ROOT).as_posix(),
                     f"{reg} documented as {shown}, {key} is {defines[key]}")


# ---------------------------------------------------------------------------
# 5. every source/tool path a document names actually exists
# ---------------------------------------------------------------------------
def check_paths() -> None:
    """Catch documentation pointing at files that were renamed or deleted.

    Deliberately mechanical. An earlier version of this script tried to spot
    stale CLAIMS by regex ("attitude fields are published as zero") and flagged
    a paragraph that was explicitly describing history. Prose cannot be checked
    this way, and a checker that cries wolf is a checker that gets switched off.
    A path either exists or it does not.
    """
    global checked
    pat = re.compile(r"`((?:src|tools|test|Configurations)/[A-Za-z_0-9./]+"
                     r"\.(?:c|h|py|json|txt|bat))`")
    pair = re.compile(r"^(.*)\.(c|h)/\.(c|h)$")

    for f in md_files():
        # A plan legitimately names files that do not exist yet -- that is what
        # makes it a plan. Checking those would force every roadmap to be
        # deleted or the checker to be ignored.
        if f.name.endswith("_PLAN.md"):
            continue

        for rel in sorted(set(pat.findall(read(f)))):
            # "src/bsw/Adc.c/.h" is one token naming two files.
            m = pair.match(rel)
            targets = [f"{m.group(1)}.{m.group(2)}", f"{m.group(1)}.{m.group(3)}"]                 if m else [rel]
            for t in targets:
                checked += 1
                if not (ROOT / t).exists():
                    fail(f.relative_to(ROOT).as_posix(),
                         f"references {t}, which does not exist")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quiet", action="store_true",
                    help="only print problems")
    args = ap.parse_args()

    check_doc_references()
    check_version()
    check_block_addresses()
    check_register_values()
    check_paths()

    if problems:
        print("Documentation is inconsistent with the code:\n", file=sys.stderr)
        for p in problems:
            print(f"  {p}", file=sys.stderr)
        print(f"\n{len(problems)} problem(s) across {len(md_files())} markdown "
              f"files. Fix the document, or the code if the document is right.",
              file=sys.stderr)
        return 1

    if not args.quiet:
        print(f"docs consistent: {checked} cross-checks over "
              f"{len(md_files())} markdown files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
