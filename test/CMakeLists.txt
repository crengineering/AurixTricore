cmake_minimum_required(VERSION 3.16)

project(aurix_tests C)

enable_testing()

# Pfad zum Firmware-Code. Die Tests kompilieren dieselben .c-Dateien,
# die auch auf den TriCore gehen - nur mit dem Host-Compiler.
set(BSW ../src/bsw)

#[[
Unity: einmal uebersetzt, von allen Tests mitbenutzt.
PUBLIC beim Include-Pfad heisst: gilt beim Bauen von Unity selbst UND
wird an jeden weitergereicht, der dagegen linkt. Deshalb findet jede
Testdatei ihr #include "unity.h", ohne den Pfad selbst zu nennen.
]]
add_library(unity STATIC unity/unity.c)
target_include_directories(unity PUBLIC unity)

#[[
Gemeinsame Einstellungen aller Tests. INTERFACE = diese Bibliothek baut
selbst nichts, sie traegt nur Konfiguration an die weiter, die sie linken.
fakes/ steht vorn: von dort kommen Ifx_Types.h und IfxStm.h statt aus iLLD.
]]
add_library(test_common INTERFACE)
target_include_directories(test_common INTERFACE fakes ${BSW})
target_compile_options(test_common INTERFACE -Wall -Wextra)

#[[
Smoke-Test: nur Unity, kein Produktivcode. Beweist, dass Compiler,
Unity, Linker und ctest zusammenspielen.
]]
add_executable(test_harness
    test_harness.c
)
target_link_libraries(test_harness PRIVATE unity test_common)
add_test(NAME harness COMMAND test_harness)

#[[
SharedRam.c (T9-T3, docs/REFACTORING_PLAN.md §2.4 / docs/MEMORY_PLACEMENT.md):
holds all six LMU shared-block objects as of T3. Used to need the same
tasking_shim.h -include the estimator sources use further down, for __at()
(a TASKING extension GCC does not know). As of T3 the last __at() left this
file -- every object is `#pragma section farbss "..."` now, which GCC just
warns about and ignores (-Wunknown-pragmas, part of -Wall, not fatal), so no
shim is needed here any more. Kept as a no-op-safe pattern would have been
harmless either way; removed because nothing in this file needs it.
]]

#[[
CoreStats: braucht nur den Ifx_Types-Shim, keinen Fake. g_coreStats selbst
kommt seit T9 aus SharedRam.c, nicht mehr aus CoreStats.c.
]]
add_executable(test_corestats
    test_corestats.c
    ${BSW}/CoreStats.c
    ${BSW}/SharedRam.c
)
target_link_libraries(test_corestats PRIVATE unity test_common)
add_test(NAME corestats COMMAND test_corestats)

#[[
GnssM9N: der Treiber steht NICHT in dieser Liste. test_GnssM9N.c zieht
GnssM9N.c per #include herein, weil die interessante Logik (Checksumme,
ASCII-Wandlung, Feldparser, Decoder) static ist und sonst unerreichbar waere.
Zusaetzlich hier aufzufuehren gaebe doppelte Symbole beim Linken.
fakes/ liefert ausserdem ConfigurationIsr.h und das IFX_INTERRUPT-Makro,
sodass die ISR auf dem Host eine ganz normale Funktion ist.
]]
add_executable(test_GnssM9N
    test_GnssM9N.c
    fakes/IfxAsclin.c
    fakes/Uart.c
    ${BSW}/SysTime.c
    fakes/IfxStm.c
)
target_link_libraries(test_GnssM9N PRIVATE unity test_common)
add_test(NAME GnssM9N COMMAND test_GnssM9N)

#[[
Atmosphere: pure ISA formula + QNH clamp, no iLLD, no fake needed.
]]
add_executable(test_atmosphere
    test_atmosphere.c
    ${BSW}/Atmosphere.c
)
target_link_libraries(test_atmosphere PRIVATE unity test_common m)
add_test(NAME atmosphere COMMAND test_atmosphere)

#[[
Sensor plausibility bands: Bmp581_plausible/Mmc5983_plausible/Icm42688_plausible
(T5, docs/REFACTORING_PLAN.md). Each driver .c still pulls I2c.h/Spi.h (and
Mmc5983/Icm42688 additionally IfxStm.h for their init delay) even though the
plausibility test never reaches the bus -- fakes/I2c.c, fakes/Spi.c and the
IfxStm_waitTicks stub in fakes/IfxStm.c exist purely to satisfy the linker.
]]
add_executable(test_bmp581
    test_bmp581.c
    ${BSW}/Bmp581.c
    fakes/I2c.c
)
target_link_libraries(test_bmp581 PRIVATE unity test_common m)
add_test(NAME bmp581 COMMAND test_bmp581)

add_executable(test_mmc5983
    test_mmc5983.c
    ${BSW}/Mmc5983.c
    fakes/I2c.c
    fakes/IfxStm.c
)
target_link_libraries(test_mmc5983 PRIVATE unity test_common m)
add_test(NAME mmc5983 COMMAND test_mmc5983)

#[[
Icm42688.c gained a SysTime_getTicks() call in I5 (docs/IMU_INTERRUPT.md) to
time the SPI burst, so this target now needs the real (iLLD-free) SysTime.c
plus its own fakes/IfxStm.c dependency, same pairing as test_GnssM9N above.
]]
add_executable(test_icm42688
    test_icm42688.c
    ${BSW}/Icm42688.c
    ${BSW}/SysTime.c
    fakes/Spi.c
    fakes/IfxStm.c
)
target_link_libraries(test_icm42688 PRIVATE unity test_common m)
add_test(NAME icm42688 COMMAND test_icm42688)

#[[
I4, docs/IMU_INTERRUPT.md §5.2/§7: ImuInt_accumulate() is a pure, header-only
(static inline) function with no iLLD include -- the ISR wrapper around it in
ImuInt.c (volatile globals, IFX_INTERRUPT vector entry, ERU/SRC calls) is
hardware-only and not built here. No fake needed, unlike every other target
above.
]]
add_executable(test_imuint
    test_imuint.c
)
target_link_libraries(test_imuint PRIVATE unity test_common)
add_test(NAME imuint COMMAND test_imuint)

#[[
ImuEdge (T15, docs/REFACTORING_PLAN.md §3.6): the ISR->NavTask_step edge
handoff protocol. ImuEdge_snapshot() is pure and iLLD-free (Ifx_Types.h only,
same static-inline-in-the-header idiom as ImuInt_accumulate() above); g_imuEdge
lives in SharedRam.c, which as of T3 holds all six LMU shared-block objects
and needs no shim (see the comment above) -- `#pragma section farbss "..."`
is not a macro, GCC just warns "ignoring #pragma section"
(-Wunknown-pragmas, part of -Wall, not fatal here) and the object lands in
ordinary .bss, same as every other host-side global.
]]
add_executable(test_imuedge
    test_imuedge.c
    ${BSW}/SharedRam.c
)
target_link_libraries(test_imuedge PRIVATE unity test_common)
add_test(NAME imuedge COMMAND test_imuedge)

#[[
NavState (T10, docs/REFACTORING_PLAN.md §2.4/§4): the generation-counter
publish/get protocol, still unwired to anything real. g_navState's placement
lives in SharedRam.c now (docs/MEMORY_PLACEMENT.md T3 -- NavStatePlace.c is
deleted; #pragma section needs no per-object TU the way __at() did, see the
SharedRam.c comment above). NavState.c itself still needs Ifx__dsync() (via
SharedRam.h) faked away -- SharedRam.c does not, it has no dsync call.
]]
set_source_files_properties(${BSW}/NavState.c PROPERTIES
    COMPILE_OPTIONS "-include;${CMAKE_CURRENT_SOURCE_DIR}/fakes/tasking_shim.h")

add_executable(test_navstate
    test_navstate.c
    ${BSW}/NavState.c
    ${BSW}/SharedRam.c
)
target_link_libraries(test_navstate PRIVATE unity test_common)
add_test(NAME navstate COMMAND test_navstate)

#[[
NavTask (T11, docs/REFACTORING_PLAN.md §4): NavTask_inputValid, the dt/NaN/
AHRS_RUNNING/present gate -- the one piece of NavTask_step worth a host test.
test_navtask.c #includes NavTask.c directly to reach navTask_dtValid (static),
same reason test_GnssM9N.c #includes GnssM9N.c. That still means every symbol
NavTask_init/NavTask_step reference must link even though this test never
calls them: `estimator` supplies the real FusionCal/Ahrs/fusion AND (as of
T3) SharedRam.c -- so g_coreStats/g_navState/g_baroLatch/g_gnssLatch/
g_magLatch/g_imuEdge all come from there; NavState.c and SysTime.c are the
real (iLLD-free) implementations, fakes/IfxStm.c satisfies SysTime.c's own
dependency -- only the genuinely target-only calls (IMU bus read, XCP
publish, peripheral diagnostics) are stubbed inside test_navtask.c itself,
purely to satisfy the linker. NavTask.c gained a g_imuDrdyStaleTicks/
g_imuDrdyLastTicks reference in I5 (docs/IMU_INTERRUPT.md) -- ImuInt.h's
declarations are iLLD-free and included fine, but the real storage is in
ImuInt.c, which pulls in ERU/SRC/Port headers with no host fakes, so those
two globals (plus g_imuDrdyMissedEdges, T14) are stubbed directly in
test_navtask.c instead, same treatment as the other target-only symbols
there. IMPORTANT: SharedRam.c is NOT listed as a source below -- it is
already compiled once into `estimator` (below); listing it again here would
link two definitions of g_coreStats et al. into the same executable.
]]
add_executable(test_navtask
    test_navtask.c
    ${BSW}/NavState.c
    ${BSW}/SysTime.c
    fakes/IfxStm.c
)
target_link_libraries(test_navtask PRIVATE unity test_common estimator)
add_test(NAME navtask COMMAND test_navtask)

#[[
Scheduler: erster echter Fake. fakes/IfxStm.c ersetzt die Hardware-Uhr,
sodass der Test die Zeit steuert - Voraussetzung fuer den Wraparound-Test.
CoreStats.c + SharedRam.c muessen mit, weil Scheduler_run() nach g_coreStats
schreibt (Speicher seit T9 in SharedRam.c, siehe oben).
]]
add_executable(test_scheduler
    test_scheduler.c
    ${BSW}/scheduler.c
    ${BSW}/CoreStats.c
    ${BSW}/SharedRam.c
    fakes/IfxStm.c
)
target_link_libraries(test_scheduler PRIVATE unity test_common)
add_test(NAME scheduler COMMAND test_scheduler)

#[[
================= Sensor-Fusion =================================================

Die drei Estimator-Dateien (fusion.c, Ahrs.c, FusionCal.c) werden hier
uebersetzt und gelinkt, aber NICHT gelesen: die Erwartungen stammen
ausschliesslich aus docs/FUSION_SPEC.md und den Headern. Siehe die Warnung
oben in FUSION_SPEC.md.

Ein Ding fehlt dem Host:
  NULL_PTR   kommt auf dem Target aus iLLDs Ifx_Types.h; fakes/Ifx_Types.h
             liefert es jetzt mit.
(g_fusionCal's Platzierung -- __at() bis T4, seither `#pragma section` --
brauchte frueher ebenfalls das __at()-Leermakro hier; docs/MEMORY_PLACEMENT.md
T5 loeschte das letzte __at() im ganzen Baum, GCC ignoriert die unbekannte
Pragma-Zeile jetzt einfach mit einer Warnung, kein Makro noetig.)

g_xcpNvm (Hard-Iron-Kalibrierung) kommt aus fakes/Nvm.c.
-m ist noetig, weil die Estimator-Mathematik gegen libm linkt (sinf, atan2f...).
]]
#[[
T12 (docs/REFACTORING_PLAN.md 2.4): the three input latches (g_baroLatch,
g_gnssLatch, g_magLatch) that fusion.c/Ahrs.c read and write. Through T2 these
lived in their own LMU __at() objects (FusionLatchPlace.c/GnssLatchPlace.c/
AhrsLatchPlace.c), one per TU per SharedRam.h's then-cppcheck constraint. As
of T3 (docs/MEMORY_PLACEMENT.md) all three moved into SharedRam.c along with
g_coreStats/g_navState/g_imuEdge -- those three *Place.c files are deleted,
SharedRam.c replaces them here. `-include tasking_shim.h` below is still
needed for fusion.c/Ahrs.c's own Ifx__dsync() calls -- the shim's __at()
entry is gone (docs/MEMORY_PLACEMENT.md T5 deleted the last __at() in the
tree); SharedRam.c doesn't need the shim at all any more, but picking up
the blanket -include here is harmless.
]]
add_library(estimator STATIC
    ${BSW}/fusion.c
    ${BSW}/Ahrs.c
    ${BSW}/FusionCal.c
    ${BSW}/SharedRam.c
    fakes/Nvm.c
)
target_include_directories(estimator PUBLIC fakes ${BSW})
target_compile_options(estimator PRIVATE -Wall -Wextra
    -include ${CMAKE_CURRENT_SOURCE_DIR}/fakes/tasking_shim.h)
target_link_libraries(estimator PUBLIC m)

#[[
FusionCal: der Sanitizer, durch den jeder XCP-schreibbare Parameter geht.
]]
add_executable(test_fusioncal test_fusioncal.c)
target_link_libraries(test_fusioncal PRIVATE unity test_common estimator)
add_test(NAME fusioncal COMMAND test_fusioncal)

#[[
Ahrs: Invarianten (Quaternion-Norm, Rotation, Mounting-Transform) und die
analytischen Lagen aus Spec 3.1/3.2.
]]
add_executable(test_ahrs test_ahrs.c)
target_link_libraries(test_ahrs PRIVATE unity test_common estimator)
add_test(NAME ahrs COMMAND test_ahrs)
set_tests_properties(ahrs PROPERTIES TIMEOUT 600)

#[[
fusion: Kovarianz-Invarianten, degenerierte Tuning-Grenzfaelle, Robustheit.
]]
add_executable(test_fusion test_fusion.c)
target_link_libraries(test_fusion PRIVATE unity test_common estimator)
add_test(NAME fusion COMMAND test_fusion)
set_tests_properties(fusion PROPERTIES TIMEOUT 600)

#[[
Differenztest gegen eine unabhaengige Referenz (Spec 4).
gen_fusion_trace schreibt eine CSV aus dem C-Filter; ref/compare_kf.py
rechnet dieselbe Eingabefolge mit einem in Python neu aufgeschriebenen
Kalman-Filter nach und vergleicht Schritt fuer Schritt.
]]
add_executable(gen_fusion_trace gen_fusion_trace.c)
target_link_libraries(gen_fusion_trace PRIVATE test_common estimator)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
    add_test(NAME fusion_differential
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/ref/compare_kf.py
                     $<TARGET_FILE:gen_fusion_trace>)
    set_tests_properties(fusion_differential PROPERTIES TIMEOUT 600)

    #[[
    Replay: aufgezeichnete Hardware-Daten aus der Session vom 2026-08-26.
    Prueft die Invarianten auf echten Eingaben und die gemessenen Werte
    aus Spec 5.
    ]]
    add_test(NAME fusion_replay
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/ref/replay_hw.py
                     $<TARGET_FILE:gen_fusion_trace>
                     ${CMAKE_CURRENT_SOURCE_DIR}/data)
    set_tests_properties(fusion_replay PROPERTIES TIMEOUT 600)
endif()
