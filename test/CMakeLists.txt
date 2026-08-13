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
