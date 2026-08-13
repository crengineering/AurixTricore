cmake_minimum_required(VERSION 3.16)

project(aurix_tests C)

enable_testing()

# Smoke-Test: nur Unity, kein Produktivcode
add_executable(test_harness
    test_harness.c
    unity/unity.c
)

target_include_directories(test_harness PRIVATE unity)

target_compile_options(test_harness PRIVATE -Wall -Wextra)

add_test(NAME harness COMMAND test_harness)

#[[
Fist test: CoreStats
]]
add_executable(test_corestats
    test_corestats.c
    unity/unity.c
    ../src/bsw/CoreStats.c
)

target_include_directories(test_corestats PRIVATE 
    unity
    fakes
    ../src/bsw)

target_compile_options(test_corestats PRIVATE -Wall -Wextra)

add_test(NAME corestats COMMAND test_corestats)