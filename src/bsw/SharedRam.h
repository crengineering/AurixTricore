/**********************************************************************************************************************
 * \file SharedRam.h
 * \brief The LMU cross-core shared-memory block and its rules.
 *
 * See docs/REFACTORING_PLAN.md §2.4 for the full argument; this header states
 * the rule, SharedRam.c holds the placement.
 *
 * Vendor mechanism (Infineon KB + TC3xx User Manual, not invented here): data
 * genuinely shared between TriCores must live in the LMU, addressed through
 * either
 *   1. the non-cached segment alias (0xB...) -- no coherency handling needed, or
 *   2. the cached alias (0x9...) plus a data synchronisation barrier (DSYNC)
 *      after every write to the shared location.
 * This project takes option 1 for every crossing (Decision D7): the LMU
 * latency (~10-20 cycles/access, order microseconds per publish) is cheap
 * against a 20 ms flight-control period, and a barrier discipline that every
 * future writer would have to honour forever is not worth trading for it.
 *
 * `CoreStats.h:14-22` used to claim "cross-core DSPR reads bypass the data
 * cache" as the project's sanctioned inter-core pattern. That was wrong --
 * it described the reader only, is not Infineon's mechanism, and must not be
 * cited for a new crossing. This file is the real rule; CoreStats.h now
 * points here instead of re-explaining it.
 *
 * Placement: no linker edit needed. `lmuram` (768 K) already exists in
 * Lcf_Tasking_Tricore_Tc.lsl:456-462 with cached view at 0x90040000 and
 * non-cached view at 0xB0040000. This block sits in the TOP 64 K of that
 * range so it can never collide with a `.bss.lmubss` allocation, which grows
 * from the BOTTOM of the same physical RAM at the CACHED alias
 * (Lcf_Tasking_Tricore_Tc.lsl:1213-1216). Nothing in this tree uses
 * `.bss.lmubss` / `.data.lmudata` today (verified: grep finds no hits outside
 * the .lsl itself) -- if that ever changes, docs/CODEMAP.md §3 is where the
 * collision risk gets caught.
 *
 * Objects were originally placed here with the TASKING `__at()` extension,
 * the same house style every XCP block used too (Measurements.c and
 * friends) until docs/MEMORY_PLACEMENT.md T4 moved those onto the same
 * `#pragma section` mechanism, and T5 deleted the last `XCP_*_ADDR` macro --
 * `__at()` is gone from this entire tree now, not just this block.
 * `__at()` blocks had to be ALONE in their translation unit: cppcheck cannot
 * parse `__at(ADDR)` at all, and -- discovered only once this actually
 * mattered -- its recovery from that syntax error discards the symbol table
 * for the REST of the file too, not just the `__at()` line, so a shared TU
 * left every later field access on every object in it unresolved to
 * cppcheck's misra addon ("misra-config: Variable 'X' is unknown"). That is
 * why NavState (T10), the three input latches (T12) and the DRDY edge
 * handoff (T15) each used to get their own `.c` file.
 *
 * As of T3 (docs/MEMORY_PLACEMENT.md), all six objects in this block are
 * `#pragma section farbss "shared_lmu.<name>"` into the linker-managed
 * `shared_lmu` group (Lcf_Tasking_Tricore_Tc.lsl) instead -- `#pragma
 * section` parses cleanly in cppcheck regardless of how many placed objects
 * share a TU (measured, docs/MEMORY_PLACEMENT.md §7), so they all live
 * together in SharedRam.c now and every one of them is visible to the MISRA
 * gate. Addresses are locator-assigned, not literals -- read them from the
 * `.map`, never hardcode one (`tools/xcp_read.py` resolves by name for
 * exactly this reason). See docs/CODEMAP.md §3 for the full occupant list.
 *
 * Rules for anything placed in this block, restated from §2.4 (all four
 * planned crossings, not just this one):
 *   1. Exactly one writer per object. The alias fixes visibility, not races.
 *   2. Every field is `volatile` and 32-bit. LMU SRAM is physically 64 bits
 *      wide with ECC and no sub-word write, so a byte/halfword store becomes
 *      a read-modify-write of the containing doubleword. Two different
 *      writers must therefore never share a 64-bit line: keep each writer's
 *      region 8-byte aligned AND a multiple of 8 bytes in size, and that
 *      falls out for free -- but it must be asserted, not assumed, whenever
 *      a new object is added here.
 *   3. Write ordering is the whole correctness argument for a multi-field
 *      object: store payload -> Ifx__dsync() -> publish (e.g. `gen++`).
 *      `volatile` gives compiler-level ordering only; it does not stop the
 *      TriCore store buffer from posting/merging writes even to non-cached
 *      memory, which is what the barrier is for. A single-field object
 *      (like one CoreStats_t slot) has no ordering to get wrong and needs no
 *      barrier -- the barrier only matters where a reader must never see a
 *      "torn" combination of two fields written together.
 *   4. No locks. `IfxCpu_acquireMutex` is a real spinlock and is available,
 *      but a spin on the flight core is an unbounded wait on another core's
 *      progress -- none of these crossings may use one.
 *********************************************************************************************************************/
#ifndef SHAREDRAM_H
#define SHAREDRAM_H

/* Top 64 K of lmuram_shared (Lcf_Tasking_Tricore_Tc.lsl), non-cached alias:
 * 0xB00F0000 .. 0xB00FFFFF. Cached alias of the same physical RAM is
 * 0x90040000-0x900FFFFF (deliberately not modelled as a memory region for
 * this block -- see the .lsl). No C symbol for the base address as of T3
 * (docs/MEMORY_PLACEMENT.md): the group owns placement, nothing in code
 * needs the literal, and an unused one is a MISRA 2.5 finding waiting to
 * happen. Read an object's actual address from the `.map`. */

/* Ifx__dsync is NOT defined for TASKING in this iLLD tree -- it exists only
 * in IfxCpu_IntrinsicsDcc.h / ...Gcc.h / ...Gnuc.h / ...HighTec.h;
 * IfxCpu_IntrinsicsTasking.h has no dsync declaration at all, and the build
 * config is TriCore Debug (TASKING). The TASKING compiler still recognises
 * __dsync() as a built-in intrinsic without any declaration -- IfxFlash.c
 * and IfxFlash.h already call it bare, and that TU builds today -- so this
 * wrapper is all TASKING needs. See docs/ILLD_NOTES.md for the trap.
 *
 * Guarded with #ifndef so the host build can define Ifx__dsync away to
 * nothing (see test/fakes) without dragging in any iLLD header: NavState's
 * publish/get protocol is host-tested, but the barrier itself is not (and
 * cannot be) -- that is a hardware/.src check, see docs/REFACTORING_PLAN.md
 * §4 and Risk 2. */
#ifndef Ifx__dsync
#define Ifx__dsync()   __dsync()
#endif

#endif /* SHAREDRAM_H */
