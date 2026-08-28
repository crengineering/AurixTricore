# MEMORY_PLACEMENT — moving fixed placement from `__at()` to the linker

Design draft for a PR **after** #15 merges. Nothing here is implemented.
Written 2026-08-28 against `feature/refactoring` @ e276764.

Companion notes for `docs/CODEMAP.md` section 3 are in part 7 — that file is in
PR #15's diff and is not touched by this draft.

---

## 1. Verdict

**Do it, and go all the way — but not the way the brief sketched it.**

Three of the four costs raised are real; one is not.

- **Real, and the strongest argument here.** `__at()` destroys cppcheck's
  symbol table for the object it places. Measured, not assumed (part 7): a
  two-object test TU written with `__at()` yields **zero** global variables in
  cppcheck's dump and **six** unbound `g_*` tokens; the same TU with
  `#pragma section` yields both globals bound and zero unbound tokens. This is
  not merely the one-object-per-TU annoyance — it means the MISRA gate is
  currently **blind to `g_xcpData`, `g_xcpCal`, `g_xcpNvm`, `g_xcpGpio`,
  `g_i2cDebug`, `g_fusionCal`, `g_xcpFusion`, `g_coreStats`, `g_navState` and
  the three latches** — every shared object in the tree, everywhere they are
  used. Worth more than the tidiness.
- **Real.** Offsets are hand-computed. `src/bsw/NavStatePlace.c:22-30`
  documents a human doing 0xB00F0000 + 0x60 because `g_coreStats` is 96 B, and
  the "256 bytes apart" convention in `docs/CODEMAP.md` exists only to give
  that arithmetic slack. Growing `CoreStats_t` needs a human to redo it.
- **Real.** Seven placement TUs (`src/bsw/SharedRam.c`,
  `src/bsw/NavStatePlace.c`, `src/bsw/FusionLatchPlace.c`,
  `src/bsw/GnssLatchPlace.c`, `src/bsw/AhrsLatchPlace.c`,
  `src/bsw/ImuEdgePlace.c`, `src/bsw/XcpFusionPlace.c`) exist purely as a
  cppcheck workaround and cost two CI failures in #15.
- **Not real — checked twice, from two directions.** The claim that
  `tools/gen_a2l.py` hardcodes the bases a third time is **false**, in both the
  form given in the brief (lines 15-19) and the form in the XCPlite research
  note (lines 63-70). Lines 15-19 are the module docstring. The `BLOCKS` dict
  at `tools/gen_a2l.py:63-70` maps each block to a **header name and a macro
  name**, never an address; the value is read out of the C header at runtime by
  `read_define()` (`tools/gen_a2l.py:111-116`, called at 301 and 529).
  `tools/check_docs.py:114-165` then cross-checks every 0x7003xxxx written in a
  `.md` against those same macros. The existing chain is
  **C header -> A2L -> docs**, machine-checked at two of the three hops.

So the problem to solve is the **placement mechanism**, not address
bookkeeping. Fix the mechanism; keep the bookkeeping chain that already works
and merely move its root from a C macro to the linker script.

There is, separately, a real **layout** defect in the generator that has
nothing to do with placement — see part 6. It does not change the placement
design, but it changes what the A2L toolchain should be verified against.

---

## 2. Is the `.lsl` ours, and does ADS regenerate it?

**Ours to edit — confirmed by Chris, and the evidence agrees, with one trap.**

| Question | Answer | Evidence |
|---|---|---|
| Vendor code? | No. Repo root, not under `Libraries/`. | — |
| How is it selected? | By **filename convention**, not project config. `.cproject` contains the string `lsl` **zero** times; the generated makefile under `TriCore Debug (TASKING)/` passes `--lsl-file="../Lcf_Tasking_Tricore_Tc.lsl"` (line 118), and the linker invocation recorded in the `.map` header (line 8) shows `-d../Lcf_Tasking_Tricore_Tc.lsl`. | build outputs |
| Does a normal build overwrite it? | No. Untouched since the initial import (`00d63d8`); builds have run against it for months. | `git log -- Lcf_Tasking_Tricore_Tc.lsl` |
| Does anything else? | **Yes, one thing.** `.ads/clean-libraries.json` lists both `.lsl` files under `"type": "DELETE"`, and `.ads/backup-libraries.json` lists both under a `CONTENT` copy. ADS's *Clean AURIX Project* / library-restore action deletes and re-emits them from the toolchain template. | those two files |

Consequences:

- Do **not** rename the file. The name is the contract with the ADS builder.
- Running ADS's *Clean AURIX Project* silently reverts the edits; the next
  build then fails to locate the new sections. That is a loud failure, not a
  flight bug, and it is recoverable from git — but it belongs in `CLAUDE.md`
  next to *Project -> Clean* (the ordinary CDT clean, which is **not** the
  dangerous one).
- The `.lsl` is a checked-in source file. That is precisely what makes it a
  legitimate SSoT for addresses, unlike the `.map` (part 5).

**`Lcf_Gnuc_Tricore_Tc.lsl` — leave it alone.** It is not referenced by any
build config in this repo (the only config is *TriCore Debug (TASKING)*), it is
2987 lines of a different dialect, and mirroring the change into it creates a
second unbuilt, unverified copy of the memory map — exactly the "two
unsynchronised places" this work exists to remove. If a GNUC config is ever
added, that PR ports the region and the groups. Record the asymmetry as a
comment in the TASKING `.lsl` so it reads as deliberate rather than forgotten.

---

## 3. Region and section design

### 3.1 The LMU region — shrink `lmuram`, do not overlay it

The sketch in the brief adds a 64 K `lmuram_nc` while `lmuram` stays 768 K.
**Reject that.** It describes the same physical RAM to the locator twice
(768 K at 0x90040000 plus 64 K aliasing its top), so the locator believes it
has 832 K and may hand the same bytes to two objects. Silent corruption,
discovered in flight.

The `reserved` attribute is not the obstacle either.
`Lcf_Tasking_Tricore_Tc.lsl:456-462` marks the `not_cached` map `reserved`,
which suppresses *automatic* allocation through the 0xB view — that is why
`run_addr = mem:lmuram` at lines 1081 and 1213 resolves to the cached
0x90040000. `__at()` sidesteps it because the compiler emits an absolute
section that the locator simply honours.

**Correct move: carve the top 64 K out of `lmuram` into its own memory and flip
which alias is reserved.**

```
    memory lmuram                       /* 768K -> 704K */
    {
        mau = 8;
        size = 704K;
        type = ram;
        map     cached (dest=bus:sri, dest_offset=0x90040000,           size=704K);
        map not_cached (dest=bus:sri, dest_offset=0xb0040000, reserved, size=704K);
    }
```

> **Corrected by the T0 spike — the dual-map `lmuram_shared` sketched below
> does NOT work as written.** Original sketch (kept for the record):
> ```
>     memory lmuram_shared                /* top 64K of the same physical LMU */
>     {
>         mau = 8;
>         size = 64k;
>         type = ram;
>         map     cached (dest=bus:sri, dest_offset=0x900f0000, reserved, size=64k);
>         map not_cached (dest=bus:sri, dest_offset=0xb00f0000,           size=64k);
>     }
> ```
> Measured on real hardware toolchain (T0, throwaway branch, discarded per
> plan — findings kept here): `group shared_lmu (ordered, align = 8,
> run_addr = mem:lmuram_shared)` against this dual-map form resolved to the
> **cached** 0x900f0150, not the intended non-cached 0xb00f0150 — with
> `cached` flagged `reserved` and `not_cached` left allocatable, i.e. the
> *opposite* of what part 3.1's own model of `reserved` predicts. Tried
> swapping declaration order of the two `map` lines too (in case `reserved`
> is order-sensitive rather than semantic here) — no effect, still cached.
> `run_addr = mem:lmuram_shared:not_cached` (an attempt to name the map
> explicitly) is not valid syntax: `ltc E821: Could not find memory
> lmuram_shared:not_cached for memory reference`. This makes R3 real, not
> hypothetical, and the fix is a single-map memory, not a smarter selector:
>
> ```
>     memory lmuram_shared                /* top 64K of the same physical LMU,
>                                             non-cached alias ONLY -- see below */
>     {
>         mau = 8;
>         size = 64k;
>         type = ram;
>         map not_cached (dest=bus:sri, dest_offset=0xb00f0000, size=64k);
>     }
> ```
> No `cached` map is declared for this region at all. `run_addr =
> mem:lmuram_shared` (bare, no map to disambiguate) then has only one
> address to resolve to, and does — confirmed at 0xb00f0150 in the T0 build.
> This is stronger than "reserved" was ever going to be: any future attempt
> to place something at the 0x9 alias of this specific 64 K is not merely
> unreserved-but-avoided, it is **not in any declared memory at all** and
> fails the link, loud, instead of silently landing cacheable. Nothing in
> this tree needs a cached view of the shared block — that is the entire
> point of the block — so the missing map costs nothing.

Why carve rather than un-reserve the existing map: un-reserving `lmuram`'s
`not_cached` view lets any future `.bss` land non-cached by accident and leaves
*both* views of the same 768 K allocatable. The carve-out keeps exactly one
allocatable view of every byte, preserves the "everything else is cached"
invariant, and states the architectural fact — the cross-core block is the top
64 K, non-cached — in the one file that can enforce it.

**Cost of shrinking `lmuram`: zero.** The current `.map` reports `mpe:lmuram`
used = 0x1b0 (432 B), free = 0xbfe50, and every occupancy row is one of our six
shared objects. Nothing else in the firmware uses LMU at all.

Then one group, in `section_layout :vtc:linear` beside the existing
`run_addr=mem:lmuram` groups:

```
            group shared_lmu (ordered, align = 8, run_addr = mem:lmuram_shared)
            {
                select ".bss.shared_lmu.corestats*";
                select ".bss.shared_lmu.navstate*";
                select ".bss.shared_lmu.barolatch*";
                select ".bss.shared_lmu.gnsslatch*";
                select ".bss.shared_lmu.maglatch*";
                select ".bss.shared_lmu.imuedge*";
            }
```

`ordered` fixes the sequence so the `.map` stays readable and a diff means
something; `align = 8` preserves today's property that no two of these objects
share a 64-bit line. Offsets become the linker's problem.

**R4 answered by T0, and it is good news.** With five of the six objects
still on `__at()` (0xB00F0000..0xB00F0507) and only `g_imuEdge` moved into
`shared_lmu`, the group did not collide with them and did not need to be
told where the gaps are — the locator placed it at 0xb00f0150, exactly the
first free 8-byte slot (right after `g_navState`, which ends at
0xB00F0000+0x60+0xF0 = 0xB00F0150, and before `g_baroLatch` at 0xB00F0200).
Absolute (`__at()`) and locator-placed (`group`) allocation in the same
declared memory coexist correctly; the locator tracks bytes consumed by
both. This means T1-T2 do not have to happen atomically with T3 — a group
can carry a strict subset of the block's objects while the rest stay on
`__at()`, which is exactly the bridgehead this task list needs.

### 3.2 The XCP blocks — absolute groups, addresses unchanged

These live in `dsram0` (`Lcf_Tasking_Tricore_Tc.lsl:332-339`, a single
non-reserved SRI map at 0x70000000) and their addresses are a hard external
contract (part 4). Use the absolute-`run_addr` form the file already uses for
the trap vectors (lines 721-726, `run_addr=LCF_TRAPVEC0_START`) — one group per
block, with the literal defined once beside the existing `LCF_*_START` defines
(lines 120-154):

```
#define LCF_XCP_DATA_START      0x70030000
#define LCF_XCP_CAL_START       0x70030100
#define LCF_XCP_NVM_START       0x70030200
#define LCF_XCP_GPIO_START      0x70030300
#define LCF_XCP_I2CDBG_START    0x70030400
#define LCF_XCP_FUSION_START    0x70030500
#define LCF_XCP_FUSIONCAL_START 0x70030600
...
            group (ordered)
            {
                group xcp_data (run_addr = LCF_XCP_DATA_START)
                { select ".bss.xcp_data*"; }
                group xcp_cal (run_addr = LCF_XCP_CAL_START)
                { select ".bss.xcp_cal*"; }
                /* ... one per block ... */
            }
```

No `section "..." (size=0x100, ...)` wrapper around them: a sized section would
emit 256 B of fill for a `.bss` block. The 256-byte spacing stays a convention,
enforced by whoever writes the defines and by the post-build check in part 5.

### 3.3 The C side — `#pragma section`, not `__attribute__((section))`

> **Corrected by the T0 spike — the tag below is wrong.** `bss` is not a
> TASKING section-class keyword; `ctc` silently accepts and discards it:
> `ctc W509: ["ImuEdgePlace.c" ...] ignored unrecognized "#pragma section bss
> shared_lmu.imuedge"` — no error, the object just stays on the compiler's
> default section and nothing moves. Had this shipped without a build to
> catch it, T2-T4 would have looked like a no-op placement change: green
> build, wrong (unplaced) result. TASKING's tag for uninitialised far data is
> **`farbss`**, confirmed working (`IfxGeth_Phy_Rtl8211f.c:98`,
> `Ifx_Lwip.c:98`, both already in this tree): `#pragma section farbss
> "text"` / `#pragma section farbss restore`. `code` (used correctly in the
> original example, see `IfxCpu_Trap.c:318`) is fine as-is for text sections;
> only the data/bss tag was wrong.

```c
#pragma section farbss "xcp_data"
volatile Xcp_Data g_xcpData;
#pragma section farbss restore
```

Both forms parse cleanly in cppcheck (part 7). Prefer `#pragma section`
because it is the mechanism the TASKING toolchain already executes in this
build — `IfxCpu_Trap.c:318` under `Libraries/iLLD/TC3xx/Tricore/Cpu/Trap/` uses
`#pragma section code "traptab_cpu0"` and the trap tables land correctly.
`__attribute__((section(...)))` is a GNU form whose TASKING support on this
toolchain version is unverified.

**Answered by the T0 spike (was: "unknown needing one build", part 10).** The
real section name TASKING emits is exactly `.bss.<pragma-name>` — no
`.<object>` suffix. `#pragma section farbss "shared_lmu.imuedge"` on
`g_imuEdge` produced `.bss.shared_lmu.imuedge` in the `.map`
(`ImuEdgePlace.o | .bss.shared_lmu.imuedge (1159) | g_imuEdge | ...`), i.e.
the FIRST of the two candidates guessed here, not the second. Every `select`
pattern in this design already carries a trailing `*` for safety; that
margin turned out to be unnecessary but costs nothing to keep.

All seven placement TUs disappear. `src/bsw/SharedRam.c` becomes the one file
holding all six LMU definitions — logic still forbidden there, but now for a
better reason: they belong together and the group order should be visible in
one place. `g_xcpData` and `g_xcpFusion` return to `src/bsw/Measurements.c`,
`g_xcpCal` stays in `src/bsw/Diagnostics.c`, and so on: each definition sits
with its module again.

---

## 4. What must stay pinned, and what may float

**Pinned — external contract; the address may not change without a coordinated
firmware + A2L + GUI release:**

| object | address | who else knows it |
|---|---|---|
| `g_xcpData` (Xcp_Data) | `0x70030000` | A2L `ECU_ADDRESS` (178 entries), AurixGUI, `tools/xcp_read.py` raw-address mode |
| `g_xcpCal` (Xcp_Cal) | `0x70030100` | A2L, GUI, `src/bsw/Xcp.c:78` write whitelist |
| `g_xcpNvm` (Xcp_Nvm) | `0x70030200` | A2L, GUI, `src/bsw/Xcp.c:82`, DFLASH image layout |
| `g_xcpGpio` (Xcp_Gpio) | `0x70030300` | A2L, GUI, `src/bsw/Xcp.c:94-100` |
| `g_i2cDebug` | `0x70030400` | not in the A2L; read by raw address during bring-up |
| `g_xcpFusion` (Xcp_Fusion) | `0x70030500` | A2L, GUI |
| `g_fusionCal` (Xcp_FusionCal) | `0x70030600` | A2L, GUI, `src/bsw/Xcp.c:86` |

The estimator-tuning row at `0x70030600` is missing from `docs/CODEMAP.md`
section 3 today and should be added (part 8).

**Free to float — linker-packed; nothing outside the ELF addresses them:**
`g_coreStats`, `g_navState`, `g_baroLatch`, `g_gnssLatch`, `g_magLatch`,
`g_imuEdge`. Verified: no `0xB00F` string appears anywhere in `tools/` or in
`docs/AurixTricore.a2l`; `tools/xcp_read.py` resolves names from the `.map`
symbol table on every run and says so at lines 28-31 ("The address of a symbol
CHANGES ON EVERY BUILD ... never hardcode an address you looked up once"). The
only pinned things about the LMU block are its **base** 0xB00F0000 and the fact
that it is the non-cached alias — both of which become properties of the
`lmuram_shared` region, stated once.

**The C-side numeric dependency must go too.** `src/bsw/Xcp.c:78-100` uses
`XCP_CAL_ADDR`, `XCP_NVM_ADDR`, `XCP_FUSIONCAL_ADDR` and `XCP_GPIO_ADDR` as
runtime constants in the cal-write whitelist. Once the linker owns placement
these become `(uint32)&g_xcpCal` and friends — strictly more correct, since the
whitelist then cannot drift from the object it guards — at the cost of one
documented MISRA 11.4 deviation per site, in a file that already carries
deviations. The `XCP_*_SIZE` and `XCP_GPIO_*_OFFSET` macros are unaffected.

---

## 5. `gen_a2l.py` and the `.map` — no

**Do not make the A2L generator depend on the `.map`. Have it read the
`.lsl`.**

The concern raised in the brief is decisive, and one harder fact sits behind
it: `TriCore Debug (TASKING)/AurixTricore.map` is **gitignored**
(`.gitignore:58`). `a2l.yml` runs `python tools/gen_a2l.py --check` on a clean
checkout with no toolchain and no build. A `.map` dependency does not merely
mean "the A2L can only be regenerated after a build" — it means **the CI gate
stops working entirely**, and a committed artefact (the A2L) would be derived
from one that is not in the repository, is machine-local, and varies with
toolchain version. That is a worse SSoT than the literal it replaced.

The `.lsl` has none of those properties: checked in, no build required, and
after part 3.2 the only place the numbers are written. So:

- `tools/gen_a2l.py`: `BLOCKS` maps each block to an `LCF_XCP_*_START` name;
  `read_define()` (lines 111-116) gets a sibling that reads the `.lsl` instead
  of a header. Same regex, different file. `--check` keeps working offline.
  Also fix the six addresses hardcoded in the **generated comment header**
  (`tools/gen_a2l.py:331-336` and 448-461) — those *are* real literals and are
  the one instance of the drift the brief describes.
- `tools/check_docs.py`: `check_block_addresses()` (lines 114-165) retargets
  from the header macros to the same `.lsl` defines. Its regex and its careful
  "prose is not an assertion" logic are unchanged.
- `tools/a2l_meta.json` keeps prose only. Do **not** move bases into it; that
  would add a third place.

**The `.map` still gets a job — as arbiter, not as input.** A new post-build
checker (design name `check_memmap.py`, to live under `tools/`) parses the
`.lsl` defines and the `.map` symbol table — `tools/xcp_read.py:46-47` already
has the exact row regex for a `| name | 0xaddress |` line — and asserts:

1. every pinned symbol sits at its declared address;
2. no two blocks overlap, given `sizeof`;
3. every LMU object lies inside 0xB00F0000 .. 0xB00FFFFF;
4. the measurement block has not outgrown its 256 B slot (already within 8 B).

It runs from `build.bat` and as a flight-dev acceptance step — never in
`a2l.yml`. That is the *verify-against-the-map* option, and for **base
addresses** it is the right one: it makes the linker's output authoritative
where the linker is authoritative (did placement actually happen?) without
making an offline gate depend on a build. For **field offsets** the calculus is
different and worse — see part 6.

### 5.1 What Vector's XCPlite does and does not contribute

Chris asked us to orientate on XCPlite. Honest answer: **borrow the principle,
not the mechanism.**

XCPlite's address machinery answers *"where did the OS loader put this image
this time?"* — `A2lCreateMeasurement()` takes a live `&object` pointer at
runtime, and `ApplXcpGetAddr()` converts it to base + offset with the base
discovered from the OS (`dl_iterate_phdr`, `_dyld_get_image_header`, Windows
module handles), asserting if the delta exceeds 4 GB. A statically linked
bare-metal TC399 image does not have that question, and TriCore addresses are
already native 32-bit, so the truncation trick solves nothing here. On-target
A2L generation would add RAM, flash and code surface for a file we generate
host-side. `docs/CODEMAP.md` already states the correct position for this
target ("Addresses are fixed via TASKING `__at`, so the A2L does not depend on
the link") — the placement mechanism changes, the position does not.

The transferable idea is XCPlite's *other* A2L path: a build-time ELF-to-A2L
generator that derives addresses from the linked binary rather than from
hand-typed constants. (That behaviour is the upstream project's own claim from
its README and TECHNICAL.md; nobody here has read that tool's source, so treat
it as a claim, not as verified behaviour.) Applied to our constraints, "derive
from the linker's own output" becomes exactly the split above: **declare in the
`.lsl`, generate from the `.lsl`, verify against the `.map`.** Zero code
borrowed. XCPlite is MIT, which would permit copying; nothing is proposed for
copying.

Two notes recorded for later, both out of scope here:

- **Address extensions.** XCP's address-extension byte tags which memory space
  an offset is relative to. It matters only if a block ever lands in per-core
  local memory where two cores' offsets could collide. Every current block —
  the 0x7003xxxx series in `dsram0` and the LMU block at 0xB00F0000 — is
  globally addressable, so it is a non-issue today. Worth knowing before
  anything is ever placed in a core-local DSPR.
- **Multi-rate DAQ.** Ours is one static list on a single event channel
  (`src/bsw/Xcp.c:41`, the 100 ms task). If the 1014 Hz estimator ever needs to
  stream faster than housekeeping, the shape to copy is a **second named event
  channel, with measurements declaring their channel in the A2L** — a standard
  XCP pattern, no XCPlite code involved. Unrelated to placement; listed so it
  is not rediscovered.

Alternatives checked and rejected: `farrrb/XcpLight` is protocol-only with no
A2L generation; `feversky/Arduino-Xcp` is GPL-3.0 and unusable here regardless.

---

## 6. The struct-layout model in `gen_a2l.py` — checked, and it is safe today

Raised alongside this design: the generator models TASKING layout with the
wrong alignment rule. That is **correct as a statement about the model**, and
it is **not currently producing a wrong A2L**. Both halves matter.

The model: `tools/gen_a2l.py:51-53` claims "scalars are aligned to their own
width, so natural packing reproduces the documented offsets", and line 153
implements it as `offset += (-offset) % size`. The recorded project trap
(2026-08-19) is that TASKING packs `uint32` at **2-byte** alignment, so a
`uint8` before a `uint32` shifts every later field 2 bytes against that model —
which is why the `*Reserved[n]` arrays exist and why the rule is "read the real
offsets out of the generated `.src`, never host `offsetof`".

**Measured, on the current tree.** All six blocks were re-laid-out under both
models (natural alignment, and alignment capped at 2) and compared field by
field:

| block | fields | natural vs 2-byte-capped |
|---|---|---|
| measurement block | 71 | **no difference** |
| calibration block | 16 | **no difference** |
| persistent block | 11 | **no difference** |
| GPIO block | 4 | **no difference** |
| navigation block | 51 | **no difference** |
| tuning block | 14 | **no difference** |

**There is no live latent mismatch — nothing to raise as a bug.** Every 4-byte
member currently lands on a 4-aligned offset under either rule, because each
run of 1-byte members is padded to a multiple of 4 before the next wide field.
The single explicit hole in the tree — one byte at 0xA5, between `gnssPresent`
(uint8, 0xA4) and `gnssyear` (uint16, 0xA6) — is a 2-byte alignment and is the
same under both models.

**But the padding is what makes a wrong model accidentally right, and nothing
detects that.** `a2l.yml` compares generated A2L against committed A2L; both
come from the same model, so the check is blind to the model itself. Concretely
one careless edit away: change `imuReserved[3]` (0x35) to `imuReserved[4]` and
`accelX` moves to 0x3C under the generator's model but 0x3A under a 2-byte
rule — a 2-byte error propagating through the remaining 60 fields, surfacing in
the GUI as plausible garbage.

**Recommendation: make the compiler the arbiter, not DWARF and not the `.map`.**

`gen_a2l.py` additionally emits a generated header of layout assertions — one
`_Static_assert(offsetof(Xcp_Data, accelX) == 0x38, ...)` per emitted field,
plus one on each `sizeof` — included by one TU per block. If the compiler's
layout ever disagrees with the generator's model, the **build fails**, at the
earliest possible point, on the developer's machine, with the field named.

Why this beats the two alternatives that were on the table:

- **Better than switching generation to ELF/DWARF.** DWARF would remove the bug
  class, but it reintroduces the exact objection that kills the `.map` option
  (part 5): the A2L could then only be regenerated after a build, so
  `a2l.yml --check` stops working on a clean checkout. And DWARF carries types,
  not prose — descriptions, units and display limits still have to come from
  the headers and `tools/a2l_meta.json`, so the header parser survives either
  way and you end up maintaining two layout models instead of one.
- **Better than a post-build ELF cross-check.** Same safety, but it needs
  `pyelftools` or a `.src` parser, runs later, and only fires for whoever has a
  build. The static asserts need no tooling at all.

Costs, stated honestly: one generated header in the build, roughly 180
assertions, and **TASKING's `_Static_assert` support in this C dialect is
unverified** — if it is unavailable, the classic
`typedef char assert_x[(cond) ? 1 : -1]` form works and MISRA-deviates cleanly.

**Scope: this is not part of the placement PR.** It touches no address, no
linker script and no placement mechanism. It is a separate, smaller change with
its own acceptance (delete a `Reserved` byte, watch the build fail). Listed
here because the question was asked while this design was open, and because it
answers "should the A2L derive from the build output?" with: **bases from the
`.lsl`, offsets from the headers, and the compiler checking the offsets** —
which is stronger than any of them alone and keeps the offline gate.

---

## 7. Does cppcheck parse the section attribute? — measured, yes

Not assumed. Three minimal translation units, each with two placed globals plus
a function reading and writing both, run through the installed cppcheck
(`--dump --std=c11 --platform=unix32 --enable=style`), with the resulting
symbol table inspected:

| form | globals in symbol table | `g_*` tokens with no variable binding |
|---|---|---|
| `__at(0xB00F0000u)` | **0** (only the struct members `a`, `b`) | **6** |
| `#pragma section bss "shared_lmu"` | **2** (`g_one`, `g_two`, access=Global) | **0** |
| `__attribute__((section(...)))` | **2** (`g_one`, `g_two`, access=Global) | **0** |

(The `bss` tag in the row above is what cppcheck's own parser accepted for
this dump-level test; it is not what `ctc` accepts — see 3.3's T0 correction,
`farbss`. cppcheck's `#pragma section` handling does not appear to validate
the tag against TASKING's actual keyword set, so this row is still true as a
statement about cppcheck but must not be read as confirming build syntax.)

The `__at()` row is exactly the failure `src/bsw/NavStatePlace.c:6-17`
describes ("misra-config: Variable 'g_navState' is unknown"), and it reproduces
with **one** `__at()` object per TU, not two — so the one-object-per-TU rule
never bought coverage; it only suppressed a duplicate 8.2 report. Both
replacements restore the symbol table fully, and **two placed objects in one TU
is fine**.

Caveat on the evidence: the local cppcheck install has no `addons/misra.py`, so
the MISRA addon itself could not be run here. The dump-level result above is
the sound part, and the dump is what the addon consumes.

The acceptance check for the task list is stronger and simpler: **after the
migration, `python tools/misra_check.py` must report zero "misra-config:
Variable ... is unknown" messages**, where today it reports one per placed
object. Expect *new* MISRA findings on those objects for the first time — that
is the feature. Fix them; do not baseline them.

---

## 8. `docs/CODEMAP.md` section 3 — becomes checked, and gets shorter

Not touched in PR #15. In this PR:

- **The XCP table stays and becomes *checked*.** The existing block-address
  check in `tools/check_docs.py` keeps validating it, with the `.lsl` as
  reference instead of the header macros (part 5). Add the missing
  estimator-tuning row at `0x70030600`. Section 3 stops calling itself the
  SSoT and points at the `.lsl`; it remains the human-readable index,
  machine-verified.
- **The LMU table loses its address column.** Those addresses change with any
  struct growth, by design. Keep object / writer / defined-in and add the group
  order. The "each is 256 bytes clear of its neighbour" convention is deleted
  along with the hand arithmetic that needed it.
- **Delete** the paragraph saying every object placed there gets its own `.c`
  file because cppcheck cannot parse a second `__at()`, and its twin at
  `src/bsw/SharedRam.h:35-42`. Both become false.
- **Keep** "when a block fills, take the next free slot rather than re-spacing
  the map" — still true, still expensive, and now enforced by the checker.
- The migration also dates the T9 wording and decision D7 in
  `docs/REFACTORING_PLAN.md`. That file is a record of decisions; add a
  one-line forward reference at each site rather than rewriting history.

---

## 9. Proving "no behaviour change"

Acceptance is **byte-identical `.map` addresses**, with exactly one declared
exception (T3, the LMU repack).

A raw `.map` diff is useless — it embeds a build hash (line 8 of the current
file) and twelve thousand lines of ordering noise. Compare the **symbol table**
instead:

1. Before touching anything: build, then dump sorted `name address` pairs using
   the symbol-row regex at `tools/xcp_read.py:46-47`. Keep it as a baseline.
2. After each task: rebuild, dump again, diff.
3. **Pass = empty diff**, except where the task explicitly names the symbols it
   moves.

This catches the real hazard, which is not the placed objects (they are pinned)
but **everything else in `dsram0`**: turning an absolute section into a
locator-placed group could reshuffle how the remaining `.bss` fills around the
holes at 0x70030000-0x700306FF. The stale `0x70000170` in `tools/xcp_test.py`
(noted at `tools/xcp_read.py:30-31`) is the kind of thing that breaks silently
when that happens.

Hardware verification is required for T2 and T3 only.

---

## 10. Risks and unknowns

| # | Risk | Handling |
|---|---|---|
| R1 | ADS *Clean AURIX Project* deletes the edited `.lsl` (`.ads/clean-libraries.json`). | Documented in `CLAUDE.md`. Failure mode is a link error, not a flight bug; recoverable from git. |
| R2 | **ANSWERED (T0).** The section name TASKING emits from `#pragma section bss "x"` is unverified. | Turned out the tag itself was wrong (`bss` -> `farbss`, silently ignored otherwise, see 3.3). With the right tag, the emitted name is `.bss.<pragma-name>` exactly, no object suffix — measured, not guessed. Wildcards stay as cheap insurance. |
| R3 | **ANSWERED (T0), and real.** `run_addr = mem:lmuram_shared` might not resolve to the 0xB alias. | It didn't: the dual-map form resolved to the *cached* 0x9 alias regardless of which map was `reserved` or their declaration order, and there is no valid map-qualifier syntax to force it (`ltc E821`). Fix: `lmuram_shared` is a single-map (`not_cached` only) memory (3.1) — removes the ambiguity outright, and the boot self-check still stands as the T2/T3 hardware confirmation. |
| R4 | **ANSWERED (T0), and it is good news.** The locator refuses an absolute `run_addr` for a group, or reshuffles `dsram0` `.bss`. | Not observed. With five of six LMU objects still on `__at()`, the single-member group landed in the first free gap between them (0xb00f0150) with no collision and no manual gap bookkeeping needed. The part 9 symbol diff confirms only the moved object's address changed. |
| R5 | New MISRA findings appear on the seven XCP objects once cppcheck can see them. | Expected and desired. Budget for it in T4; fix, do not baseline. |
| R6 | **A2L / GUI contract.** If a pinned address moves by accident the GUI reads the wrong bytes and shows plausible garbage — the silent failure `tools/gen_a2l.py:1-30` exists to prevent. | Nothing here intends to move one. The part 9 diff, `a2l.yml --check`, and the new post-build checker are three independent gates. **No GUI change is required by this PR** — if one turns out to be, the design is wrong and should stop. |
| R7 | T3 changes five of six shared-object addresses. | One flash, one bench run. Nothing external addresses them (part 4). |
| R8 | `_Static_assert` may be unavailable in TASKING's C dialect (part 6, separate PR). | Fall back to the negative-array-size idiom. Does not block anything here. |

**Could not determine without a build:** R2, R3, R4, whether TASKING accepts
`__attribute__((section))`, and R8. **The T0 spike answered R2, R3 and R4** —
see their rows above and 3.1/3.3 for the measured results (R2 and R4 came
back better than assumed; R3 was real and needed a design change). Whether
TASKING accepts `__attribute__((section))` and R8 remain open; T0 used
`#pragma section` only, per the design's own preference, so neither was
exercised.

**Not worth doing** (explicitly rejected):

- `gen_a2l.py` reading the `.map` (part 5) or generating from DWARF (part 6).
- Mirroring the change into `Lcf_Gnuc_Tricore_Tc.lsl` (part 2).
- Carving the XCP blocks out of `dsram0` as their own `memory` — it needs
  `dsram0` split around a hole and moves every other CPU0 `.bss` object,
  violating part 9 for no gain.
- Anything from XCPlite's runtime address machinery (part 5.1).
- **Stopping after T3.** A half-migration that leaves the seven XCP blocks on
  `__at()` keeps the one-object-per-TU rule alive, keeps the documentation
  burden, keeps the MISRA blind spot on the most-touched objects in the tree,
  and adds a second mechanism. If T4-T6 are not going to happen, **do not
  start**: the LMU-only version is worse than today.

---

## 11. Ordered tasks

Each task builds and flies on its own. T0 is throwaway.

| # | Files | Change | Acceptance |
|---|---|---|---|
| **T0** | scratch branch, discarded | **DONE.** Spike: add `lmuram_shared` and a one-member `shared_lmu` group; move `g_imuEdge` only (8 B, newest, single core) from `__at()` to `#pragma section`. | Builds. `g_imuEdge` landed at **0xb00f0150** (not 0xb00f0000 — five other objects still occupy 0xB00F0000-0xB00F0507 via `__at()`, and the locator correctly filled the first free gap after `g_navState`). Real section name: **`.bss.<pragma-name>`** exactly, e.g. `.bss.shared_lmu.imuedge` (no `.g_xcpData`-style suffix). Symbol diff: exactly one symbol moved. Answers R2/R3/R4 — R2 and R4 came back clean, **R3 was real** (dual-map `lmuram_shared` resolves `run_addr=mem:X` to the cached alias regardless of `reserved`; fixed with a single-map region, 3.1). Branch deleted; findings written back into this file (3.1, 3.3, R2-R4 above). |
| **T1** | `Lcf_Tasking_Tricore_Tc.lsl` | `lmuram` 768K to 704K; add `lmuram_shared` (64 K, **single map, `not_cached` only at 0xb00f0000** — see T0 correction in 3.1, not the original cached+reserved sketch). No group, no C change. | Builds, 0 warnings. **Symbol diff empty** (part 9) — the region is still unused. |
| **T2** | `.lsl`, `src/bsw/SharedRam.c`, `src/bsw/SharedRam.h`, `src/bsw/ImuEdge.h`, delete `src/bsw/ImuEdgePlace.c`, `.cproject` | Add the `shared_lmu` group (`select` pattern `.bss.shared_lmu.imuedge*` per T0's confirmed section name). Move **`g_imuEdge` only** to `#pragma section farbss "shared_lmu.imuedge"` (T0 correction: `farbss`, not `bss`), into `SharedRam.c`. Lands wherever the locator puts it among the five still-`__at()` objects (T0: not necessarily 0xB00F0000). | Builds from clean. Symbol diff shows exactly one moved symbol. **Boot UART reports the block non-cacheable** (`src/bsw/Cpu0_Main.c:86`). Flash + bench: the missed-edge counter behaves as before. `misra-config` for `g_imuEdge` gone. |
| **T3** | `src/bsw/SharedRam.c`; delete `src/bsw/NavStatePlace.c`, `src/bsw/FusionLatchPlace.c`, `src/bsw/GnssLatchPlace.c`, `src/bsw/AhrsLatchPlace.c`; `src/bsw/NavState.h`, `src/bsw/FusionLatch.h`, `src/bsw/AhrsLatch.h`, `.cproject`, `.lsl` group | Move the remaining five LMU objects in, in the documented order. **The one task where addresses change by design** — list the five in the commit message. | Builds from clean. `.map`: six objects contiguous from 0xB00F0000, 0x1B0 total, 8-byte aligned, all inside `lmuram_shared`. Boot self-check passes. **Bench/flight run: nav converges; baro, GNSS and mag latches all update.** MISRA green, zero `misra-config` for all six. |
| **T4** | `src/bsw/Measurements.c`, `src/bsw/Diagnostics.c`, `src/bsw/Nvm.c`, `src/bsw/gpio.c`, `src/bsw/I2c.c`, `src/bsw/FusionCal.c`, delete `src/bsw/XcpFusionPlace.c`, `.lsl`, `.cproject` | Seven `xcp_*` groups at the `LCF_XCP_*_START` defines; the seven objects move to `#pragma section` in their own modules. The `XCP_*_ADDR` macros are **retained** this step, no longer used for placement. | **Symbol diff EMPTY.** `a2l.yml` green with no regeneration. Flash + `tools/xcp_test.py` + a GUI session: every block reads as before. Expect and fix new MISRA findings. |
| **T5** | `src/bsw/Xcp.c`, `src/bsw/Measurements.h`, `src/bsw/Diagnostics.h`, `src/bsw/Nvm.h`, `src/bsw/gpio.h`, `src/bsw/I2c.h`, `src/bsw/FusionCal.h` | Delete the seven `XCP_*_ADDR` macros. The `Xcp.c` whitelist switches to `(uint32)&g_xcpCal` and friends, one documented 11.4 deviation each. | Builds. Host tests green. Flash: a write inside each cal block still succeeds, one outside still rejects. |
| **T6** | `tools/gen_a2l.py`, `tools/check_docs.py` | Bases read from the `.lsl` defines instead of the header macros; fix the addresses hardcoded in the generated comment header. | `python tools/gen_a2l.py --check` passes **on a clean clone with no build**. `python tools/check_docs.py` passes. `a2l.yml` green. |
| **T7** | new checker under `tools/`, `build.bat` | Post-build `.map` verifier: pinned addresses, overlap, LMU containment, measurement-block headroom (part 5). | Passes on a fresh build; fails if a block base is edited in the `.lsl` without rebuilding. |
| **T8** | `docs/CODEMAP.md` section 3, `src/bsw/SharedRam.h`, `CLAUDE.md`, `docs/REFACTORING_PLAN.md` (one-line refs) | Documentation per part 8 plus the ADS-clean hazard per part 2. | `python tools/check_docs.py` green. No stale mention of `__at()` or of one-object-per-TU survives a grep. |

T1-T2 are one afternoon and independently valuable. T3 is the one needing a
flight. T4 is the largest but has the strictest acceptance (empty diff). T5-T8
are cleanup that cannot be skipped without leaving two mechanisms in the tree.

**Separate, smaller PR (not sequenced here):** the generated layout-assertion
header from part 6. Independent of everything above; do it whenever.
