# Unit Tests & CI/CD — Learning Plan

**ASPICE:** SWE.4 — unit verification strategy (host tests, fakes, CI) · serves all SWE1-FW items · process: QuadSE/requirements/README.md

Part A is the transferable part: principles that hold for any C/C++ project.
Part B is the first exercise — host-run unit tests for this firmware.
Part C carries the same ideas to AurixGUI afterwards.

Read Part A once, then work through Part B by hand, one stage per commit.

---

# Part A — What actually transfers

Frameworks, runners and yml syntax are disposable. These are what you carry to
every project.

**A1. Testability is a design property, not a testing problem.**
One question decides it: does this function take data and return data, or does
it reach out to the world? A function that reads a sensor register *and*
converts the reading is untestable. Split into a thin `read()` plus a pure
`convert()`, the half that holds the bugs becomes trivially testable.
Push I/O to the edges, keep computation in the middle, test the middle.

*You already do this deliberately:* `src/asw/README.md` rule 2 and `SysTime.h`
exist for exactly this reason. Part B is mostly cashing in that investment.

**A2. Test on the host, not the target.**
TriCore code cannot run on a GitHub runner and bench debugging is not a
regression suite. But C is C — the same `.c` files compile with GCC on a PC and
run in milliseconds. This only works where A1 holds, so host testing is not a
separate technique; it is the payoff for the separation. The ADS/TASKING build
stays untouched; a second, independent CMake build compiles the portable
sources plus tests for the host.

**A3. Seams and test doubles.**
Where pure logic must call hardware, define the dependency as a header
interface and supply a fake in the test build. In C the simplest mechanism
needs no framework: **link-time substitution** — compile `test/fakes/I2c.c`
instead of `src/bsw/I2c.c`. Same symbols, canned data, and you can now make a
sensor return garbage on demand.

**A4. The failure paths are the point.**
On the bench you test the happy path. What you *cannot* provoke on real
hardware is what kills you in the field: a sensor that NaNs, a counter rolling
over, a CRC mismatch in flash, a controller saturating, two NVM sectors both
claiming to be newest. On the host each is three lines of setup.
Test the boundaries and error returns *before* the normal case — the normal
case is what you already tried manually.

**A5. Prioritise by consequence, never by coverage percentage.**
Worth testing, in order: pure computation with real consequences (control laws,
compensation maths, CRC, framing, state machines); logic you cannot reproduce
by hand; and every bug you have already fixed once. Not worth testing: getters,
thin glue, tests that just restate the implementation. Coverage is for *finding*
untested branches, never a target.

**A6. A pipeline's value is what it blocks.**
A workflow nobody must act on is decoration. Value appears when a red check
prevents a merge. Your `misra.yml` runs — but is it a *required* check on
`main`? That is the gap.

**A7. The baseline ratchet.**
`tools/misra_baseline.txt` already does the most useful trick for introducing a
gate into an existing codebase: freeze current findings, fail only on new ones.
"Fix everything first" is impossible; "do not make it worse" is trivial and
improves quality monotonically. Reuse this for every future gate.

**A8. CI must be reproducible locally, and the yml must be thin.**
`misra.yml` gets this right: real work in `tools/misra_check.py`, cppcheck
version pinned and built from source so local findings match CI, build cached.
Generalise both halves — one command, run identically in both places; pinned
tool versions (an unpinned tool breaks your pipeline on a morning you changed
nothing, which teaches you to ignore red).

**A9. Fast feedback is a design constraint.**
A suite you avoid running does not exist. Keep unit tests sub-second: no sleeps,
no network, no hardware. Anything slow is an integration test and belongs in a
separate job. Cost follows the same axis — GitHub bills Linux at 1×, Windows 2×.

**Framework choice is the least important decision.** Unity for embedded C, Qt
Test for Qt, GoogleTest/Catch2 for general C++. What transfers is identical:
arrange–act–assert, table-driven cases, fixtures, one runner command, non-zero
exit on failure.

---

# Part B — AurixTricore: host unit tests

**Framework: Unity** (ThrowTheSwitch, MIT). Three files, no dependencies,
designed for embedded C. **Vendor it** into `test/unity/` rather than using
CMake `FetchContent` — no network in CI, byte-identical local and CI runs,
consistent with how you already pin cppcheck (A8).

**Runner: CTest. CI: GitHub Actions**, modelled on your existing `misra.yml`.

### How this relates to `tools/*_test.py`

`xcp_test.py`, `nvm_test.py` and `ctrl_replay_test.py` are **on-target
integration tests** — real board, real peripherals, pyXCP over Ethernet. They
stay exactly as they are. Host unit tests are a *different layer*: no hardware,
milliseconds, run on every push, and able to reach failure paths the board
cannot be talked into producing (A4). The two are complementary; neither
replaces the other.

---

### Stand (2026-08-14)

Gebaut und in `main`:

- `test/` mit vendortem Unity 2.7.0, drei Targets (`harness`, `corestats`,
  `scheduler`), `unity` als statische Bibliothek, `test_common` als
  INTERFACE-Träger für Include-Pfade und Warnungs-Flags
- `test/fakes/`: `Ifx_Types.h` (Typ-Shim, exakte Breiten aus `stdint.h`) und
  `IfxStm.h`/`.c` (steuerbare Uhr, `FakeStm_setTicks`/`FakeStm_advance`)
- `tools/utest.py` als einzige Ausführungsquelle für lokal und CI:
  `--clean`, `-R/--filter`, `--variant plain|sanitize|coverage`
  (Variante bestimmt Build-Verzeichnis *und* Flags über eine Tabelle),
  `--werror`, `--build-only`, `--opt=<stufe>`.
  `run(cmd, check=False)` gibt den Exit-Code zurück statt abzubrechen — nötig,
  damit die Testzusammenfassung auch bei roten Tests geschrieben wird; `main()`
  reicht den Code am Ende weiter.
- `.github/workflows/unit_tests.yml`, vier Jobs auf `ubuntu-latest`, fünf Checks:

  | Check | Frage |
  |---|---|
  | C Unit tests | Ist der Code korrekt? |
  | Sanitizers (ASan + UBSan) | Greift er unerlaubt auf Speicher zu? |
  | Coverage | Was ist überhaupt geprüft? |
  | Warnings (-Og) / (-Os) | Ist er sauber übersetzt, naiv und optimiert? |

  Dazu `timeout-minutes` (GitHubs Standard sind 360), `concurrency` mit
  `cancel-in-progress`, und Test- wie Coverage-Tabelle via
  `$GITHUB_STEP_SUMMARY` auf der Übersichtsseite jedes Laufs. Die
  Summary-Schritte brauchen `if: always()`, sonst erscheinen sie ausgerechnet
  bei roten Läufen nicht. Der gcovr-HTML-Bericht geht als Artefakt raus.

Coverage-Stand beim Einrichten: **34 % Zeilen, 21 % Zweige** über `src/bsw`
(gemessen werden nur `CoreStats.c` und `scheduler.c`, weil nur die in
Testprogrammen mitkompiliert werden; `scheduler.c` allein liegt bei 22 %).
Der gcovr-Filter muss `src/bsw/` lauten — relativ und mit Schrägstrichen, denn
Filter sind reguläre Ausdrücke, und ein Windows-Pfad mit Backslashes bricht als
ungültige Escape-Sequenz ab.

Nachgewiesen, nicht bloß eingerichtet: ein absichtlicher Zugriff hinter
`g_coreStats` liess den normalen Job grün und den Sanitizer-Job rot werden
(`index 6 out of bounds`, gemeldet von UBSan — nicht von ASan, weil die
Arraygröße statisch bekannt ist). Rot wurde er nur wegen
`-fno-sanitize-recover=all`; ohne dieses Flag druckt UBSan den Fund und läuft
mit Exit-Code 0 weiter. Sanitizer laufen ausschliesslich in der CI —
MinGW/MSYS2 liefert weder `libasan` noch `libubsan`.

Stolpersteine, die Zeit gekostet haben und wiederkommen werden:

- **Stale CMake-Cache.** Flags nur bedingt zu übergeben lässt alte Werte im
  Cache überleben. Deshalb wird `-DCMAKE_C_FLAGS=` *immer* gesetzt, notfalls leer.
- **`-O0` gegen `-00`.** Ziffer statt Buchstabe, optisch nicht unterscheidbar.
- **`--opt=-Og` statt `--opt -Og`.** argparse hält jeden Token mit führendem
  Bindestrich für eine Option; bei Werten im GCC-Stil ist die `=`-Form die
  einzige eindeutige.
- **`$?` nach einer Pipe** liefert den Status des letzten Programms, nicht des
  ersten — in Bash `${PIPESTATUS[0]}`. Führt sonst zu still grünen Prüfungen.
- **`flush=True`** bei jeder Protokollausgabe: Python puffert blockweise,
  sobald die Ausgabe nicht ins Terminal geht, und die Zeilen landen im CI-Log
  an der falschen Stelle.

**Offen und inhaltlich am wichtigsten:** die Tests selbst sind dünner als die
Infrastruktur. Der Wraparound-Test aus Stage 2 steht noch aus.

### Stage 1a — Harness only, no production code

Prove the toolchain before involving the firmware. One `.c` file that includes
**nothing but `unity.h`** and asserts `TEST_ASSERT_EQUAL_INT(2, 1 + 1)`.

1. `test/` with its own `CMakeLists.txt` — a **separate** CMake project. The ADS
   build is not touched and does not learn about CMake.
2. Vendor `unity.c`, `unity.h`, `unity_internals.h` into `test/unity/` (done).
3. One executable from `unity.c` + your test file; register with `add_test()`.

Fifteen minutes, and afterwards you know CMake, Ninja, Unity and ctest agree.
Every failure from here on is your code, not the scaffolding.

Local build: `cmake -S test -B build-test -G Ninja` (no `make` on this machine;
`cmake_minimum_required` must be ≥ 3.5, CMake 4 hard-errors on older).

Unity specifics to look up rather than copy: `setUp`/`tearDown` (called around
*every* test), the `RUN_TEST` main, and `TEST_ASSERT_FLOAT_WITHIN` — never
`TEST_ASSERT_EQUAL` on floats.

**Done when:** `ctest --test-dir build-test --output-on-failure` is green
locally. Then break the assertion and confirm ctest exits non-zero — that exit
code is the entire CI contract (A8).

### Stage 1b — The `Ifx_Types.h` shim, and `CoreStats.c`

A ten-minute warm-up that proves the shim before any fake is involved.

`CoreStats.c` is 18 lines with one function. Its only obstacle is that
`CoreStats.h` includes `Ifx_Types.h`, which drags in `Platform_Types.h` and
compiler-specific headers host GCC cannot build.

Do **not** edit the header. Write `test/fakes/Ifx_Types.h` containing only what
is actually used — `boolean`, `float32`, `uint8`, `uint16`, `uint32`, `TRUE`,
`FALSE` — and put `test/fakes/` on the host include path *ahead* of the iLLD
path. The target build never sees it. Eight lines, and the same shim unlocks
`scheduler.c`, `Diagnostics.c`, `Measurements.h`, `Nvm.h` and `Bmp581.c`.

Two tests: `CoreStats_init(0)` zeroes that slot, and **`CoreStats_init(6)` must
not write past the array** — a real boundary, and the reason the guard exists.

### Stage 2 — `scheduler.c`: your first fake, and the wraparound *(A3, A4)*

95 lines of plain integer logic with semantics you already know — "run this task
every N ms". Nothing to understand mathematically, which is what makes it the
right place to meet fakes.

`Scheduler_run()` calls `IfxStm_getLower(stm)`, the hardware clock. The fake is
a handful of lines: a stub `test/fakes/IfxStm.h` declaring an opaque `Ifx_STM`
plus the prototype, and an implementation returning a variable the test sets.
**That is the point — you now control time**, and controlling time is what makes
everything below testable.

- `Scheduler_addTask` 8× → `TRUE`, the 9th → `FALSE` (`SCHEDULER_MAX_TASKS`)
- a task does *not* fire before its period elapses, and fires exactly once after
  — advance the fake clock, call `Scheduler_run()`, count callback invocations
- **the wraparound.** `scheduler.c:57` claims *"Unsigned subtraction handles
  32-bit counter wraparound correctly"* — an assertion nobody has ever checked.
  Park the fake clock just below `0xFFFFFFFF`, roll it past zero, and prove the
  task still fires at the right delta. On hardware this is a 43-second
  experiment you would never repeat; here it is three lines (A4).
- `execUs = busyTicks / 100` and `loadPmil` over the window: known ticks in,
  known values out
- dispatch sets `lastRun = now`, not `lastRun += period`, so periods drift under
  load rather than catching up. Pin that intent with a test so the next reader
  knows it is a decision, not an oversight.

### Stage 3 — `Diagnostics.c`: fakes and the `__at` seam *(A3, A4)*

Here because it needs every seam technique at once — but the payoff is the
debounce logic, which is the best A4 example in the repo.

**Was the blocker: `__at()`**, a TASKING extension for absolute placement,
unknown to GCC. `docs/MEMORY_PLACEMENT.md` (T4/T5) removed `__at()` from this
entire tree, `Diagnostics.c` included -- `g_xcpCal` is `#pragma section
farbss "xcp_cal"` now, guarded by `#if defined(__TASKING__)` (same idiom
`SharedRam.c` uses), so on GCC that guard compiles out to nothing and
`g_xcpCal` is an ordinary global with no special handling needed at all. No
`target_compile_definitions(... "__at(x)=")` workaround required if this
stage is picked up now -- the seam this section describes may already be
free.

Then the fakes, about 40 lines total: a definition of `volatile Xcp_Data
g_xcpData` (this *is* your input lever — the test writes the measurements),
plus `PeriphDiag_init/update()`, `Nvm_hasFault()`, `Uart_heartbeatReceived()`.
`Measurements.h` includes only `Ifx_Types.h`, so with the stage 1b shim the real
header works and you need not rebuild the `Xcp_Data` struct.

Tests:
- **debounce:** `diagnosticsUpdate()` runs every 100 ms and `debounceSec`
  defaults to 1.0 s. Set `dieTempC` out of range, call 9× → bit clear; 10th →
  set. On the bench you would have to hold an overtemperature for a full second.
- one row per bit, table-driven, checked directly against `docs/DIAGNOSTICS.md`
- `g_xcpCal.magic` wrong → defaults reloaded, `DIAG_CAL_INVALID` set
- limits are calibratable: change `g_xcpCal.vddMax`, confirm the threshold moves

### Stage 4 — Link substitution: a fake I2C *(A3)*

`Bmp581.c` includes `I2c.h`. Compile the test binary against `test/fakes/I2c.c`
— same signatures, returning canned register frames — instead of the real one.
No mocking framework, just the linker.

Then test the compensation maths. **The BMP581 datasheet publishes worked
examples with expected outputs: those are golden test vectors, free.** Same
applies to `Bmp388.c` and `Icm42688.c`. Add the failure paths: sensor absent,
I2C error return, out-of-range raw values.

### Stage 5 — Put it in CI *(A8)*

Copy the shape of `misra.yml`: `ubuntu-latest`, thin yml, one command. No Qt, no
TriCore toolchain, no cppcheck build — just GCC and CMake, so this job is
fast and cheap.

Decide deliberately: a new `tests.yml`, or a second job inside `misra.yml`?
(Separate workflows give independent badges and clearer failures; one workflow
keeps related gates together. Justify your pick in the commit message.)

Check that `tools/misra_check.py` scopes to `src/` — test code and vendored
Unity must not enter the MISRA baseline. If it does scan them, fix the scope in
this stage.

**Done when:** you push a deliberately broken test, see red, fix it, see green.

### Stage 6 — Make the checks required *(A6)*

> **Blocked on this repo (checked 2026-08-13).** Branch protection needs GitHub
> Pro for *private* repositories — the API answers `403: Upgrade to GitHub Pro
> or make this repository public to enable this feature`. AurixGUI is private
> too, so there is no free repo to practise on. Three ways out: make the repo
> public (also lifts the Actions minute limit), pay for Pro, or substitute a
> versioned `pre-push` hook — `.githooks/pre-push` running configure/build/ctest
> plus a one-off `git config core.hooksPath .githooks`. The hook enforces the
> same contract one step earlier and is free. The rest of this stage applies
> unchanged once protection is available.

Branch protection on `main`, requiring **both** `MISRA` and the new test check.
A check only appears in that list after it has run once. Then work on branches
and merge via PR — your `misra.yml` already has a `pull_request` trigger built
for a workflow you are not yet using. Add a test badge to `README.md` next to
whatever MISRA reports.

### Stage 7 — Harder targets, and testing driving design

- **`Nvm.c`** — CRC plus 2-sector ping-pong selection. Severe consequences, and
  "both sectors valid", "both CRCs bad", "power loss mid-write" are effectively
  unprovokeable on the bench (A4). Needs a fake `IfxFlash` — a RAM array behind
  the flash API. The most valuable and most work; do not start here.
- **`flight_ctrl.c`** (ASW) — the cascade controller: pure maths, zero includes
  beyond `math.h`, so no build work at all. Deferred not because it is hard to
  wire up but because knowing *what to assert* needs the control theory in your
  head. Start with `rate_ctrl_step`: drive it into `tau_sat_up`, hold it, and
  prove the integrator stops growing — that is the clamping-anti-windup claim in
  the header, the classic PI bug, and on the bench it needs a saturated flight.
  Then `mixer_step`: demand more than it can deliver and assert the clamp holds
  so no `sqrt()` of a negative reaches `w_cmd`.
- **the attitude estimator** (`Ahrs.c`, rewritten 2026-08-26 as a quaternion
  Mahony filter and HW-validated — see `docs/FUSION.md`) — will need
  only the stage 1b shim, no fakes at all, so it is trivial to build. The work is
  conceptual: you must understand the filter before you can state an expectation.
  Best test is the accel-trust gate — feed `|a| = 2 g` and assert the
  accelerometer stops being trusted, which you cannot provoke on a desk.
- **`Xcp.c`** — packet framing/parsing. Note that `XcpClient` in AurixGUI parses
  the *same wire format*: once both repos have host tests, one shared set of
  golden frames pins the interface between them, which is the class of bug that
  otherwise only appears on the bench.
- **Wrap-safe time deltas.** `SysTime_getTicks()` wraps every ~42.9 s and
  `SysTime.h` documents that unsigned subtraction survives one wrap. If that
  delta arithmetic is currently written inline at each call site, extracting a
  `SysTime_elapsed(a, b)` helper makes it testable — and testing it across the
  wrap boundary is otherwise a 43-second bench experiment you will never repeat.
  That extraction is A1 working in reverse: the need to test improves the code.

### Stage 8 — Coverage *(A5)*

`--coverage` on the host build, `gcovr --html-details`, uploaded as an artifact.
Guard behind a CMake option. Then find one branch nothing executes and decide
whether it deserves a test or deserves deleting. Do not chase a number.

### Commit trail

| Stage | Commit message |
|---|---|
| 1a | `test: add Unity host test harness with CTest` |
| 1b | `test(corestats): host shim for Ifx_Types, cover the coreId bound` |
| 2 | `test(scheduler): fake STM clock, cover dispatch timing and counter wraparound` |
| 3 | `test(diag): fake the BSW seams, cover debounce and the status bits` |
| 4 | `test(bmp581): fake I2C and datasheet golden vectors` |
| 5 | `ci: run host unit tests on every push and PR` |
| 6 | `ci: require MISRA and unit tests before merging to main` |
| 7 | `test(nvm): fake flash, cover CRC and ping-pong selection` |
| 8 | `ci: publish a coverage report as a build artifact` |

One stage per session. Do not combine 1a and 1b — when the harness will not
link, you want to know whether it is the build or the test.

**Why this order:** 1a has no dependencies at all, 1b adds the type shim, 2 adds
your first fake, 3 adds `__at` neutralisation and multiple fakes, 4 adds link
substitution. Each stage introduces exactly one new technique, so a failure is
never ambiguous. `flight_ctrl.c` and the attitude estimator are easy to *build*
but hard to *reason about*, which is why they sit in stage 7 rather than at the
front.

---

# Part C — Then AurixGUI

Same principles, different tooling: **Qt Test** (ships inside qtbase, so no
extra dependency in CI) + CTest + the existing `build.yml`.

The one structural difference: AurixGUI compiles everything into a single
`add_executable(SerialMonitor ...)`, and a test binary cannot link against an
executable. So it needs an A1 refactor *first* — split into an `aui_core` static
library plus a thin exe holding only `main.cpp`. Keep the pure logic
(`a2lmodel`, `mf4writer`) in the library and the widgets in the executable; then
`aui_core` needs only `Qt6::Core` and the tests run headless on Linux at 1× cost.

Ordered targets once that is done:

1. **`A2lModel::decode()`** (`a2lmodel.cpp:212`) — pure, byte-level, real failure
   branches: buffer too short, `addr` below `blockBase` (`off < 0`), empty
   input. Learn Qt Test's data-driven `_data()` functions here.
2. **The A2L parser** against a small hand-written `tests/data/minimal.a2l` —
   unknown `RECORD_LAYOUT` → entry skipped; a `/* comment */` must not glue two
   tokens; a quoted description containing the word `VALUE` must not parse as a
   VALUE line.
3. **`Mf4Writer`** round-trip through `QTemporaryDir` — assert behaviour (a
   valid MDF) not implementation (byte 47 is 0x12).
4. **`XcpClient`** — parsing is private and welded to a UDP socket; extracting
   frame decoding into free functions is the refactor that testing forces, and
   it is where the shared golden frames from Part B stage 7 pay off.

Then the same CI progression: a second job, `ctest --output-on-failure`, branch
protection, and finally a release job on `v*` tags that runs `windeployqt` and
attaches a zip to a GitHub Release — the actual "CD", and where the existing
artifact upload belongs.

---

### The recipe for any future C/C++ project

1. Separate compute from I/O from day one (A1) — cheap now, expensive later.
2. Host-buildable core plus one test binary, from the first commit (A2).
3. One command runs everything, identically local and in CI (A8).
4. Make the check required before the codebase grows (A6).
5. Introduce every later gate via a baseline ratchet (A7).
6. Write a test for every bug you fix (A5).
