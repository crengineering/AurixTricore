cmake_minimum_required(VERSION 3.16)

project(aurix_tests C)

enable_testing()

add_executable(test_harness
    test_harness.c
    unity/unity.c
)

target_include_directories(test_harness PRIVATE unity)

target_compile_options(test_harness PRIVATE -Wall -Wextra)

add_test(NAME harness COMMAND test_harness)