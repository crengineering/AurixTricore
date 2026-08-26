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
CoreStats: braucht nur den Ifx_Types-Shim, keinen Fake.
]]
add_executable(test_corestats
    test_corestats.c
    ${BSW}/CoreStats.c
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
Scheduler: erster echter Fake. fakes/IfxStm.c ersetzt die Hardware-Uhr,
sodass der Test die Zeit steuert - Voraussetzung fuer den Wraparound-Test.
CoreStats.c muss mit, weil Scheduler_run() nach g_coreStats schreibt.
]]
add_executable(test_scheduler
    test_scheduler.c
    ${BSW}/scheduler.c
    ${BSW}/CoreStats.c
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

Zwei Dinge fehlen dem Host:
  __at()     TASKING-Erweiterung, mit der g_fusionCal auf eine feste Adresse
             gelegt wird. Auf dem Host ein leeres Makro -- das Objekt landet
             einfach im BSS.
  NULL_PTR   kommt auf dem Target aus iLLDs Ifx_Types.h; fakes/Ifx_Types.h
             liefert es jetzt mit.

g_xcpNvm (Hard-Iron-Kalibrierung) kommt aus fakes/Nvm.c.
-m ist noetig, weil die Estimator-Mathematik gegen libm linkt (sinf, atan2f...).
]]
add_library(estimator STATIC
    ${BSW}/fusion.c
    ${BSW}/Ahrs.c
    ${BSW}/FusionCal.c
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
