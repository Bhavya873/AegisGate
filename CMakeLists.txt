cmake_minimum_required(VERSION 3.20)
project(AegisGate C)

set(CMAKE_C_STANDARD 11)

add_library(unity third_party/unity/unity.c)
target_include_directories(unity PUBLIC third_party/unity)

add_compile_options(-Wall -Wextra -Werror)
include_directories(core/include config sim/include demo)

add_library(aegis_core
    core/src/types.c
    core/src/state_machine.c
    core/src/invariants.c
    core/src/arbiter.c
    core/src/estimator.c
    core/src/pid.c
    core/src/spsc.c
    core/src/safety.c
)

add_library(aegis_sim sim/src/plant.c sim/src/lockstep.c sim/src/arm.c)
target_link_libraries(aegis_sim aegis_core)

add_library(aegis_demo demo/demo_api.c)
target_link_libraries(aegis_demo aegis_sim)

add_executable(aegis_lockstep sim/src/lockstep_main.c)
target_link_libraries(aegis_lockstep aegis_sim)

add_executable(aegis_arm sim/src/arm_main.c)
target_link_libraries(aegis_arm aegis_sim)

add_executable(aegis_bench sim/src/bench_main.c)
target_link_libraries(aegis_bench aegis_sim)
target_compile_definitions(aegis_bench PRIVATE AEGIS_BUILD_TYPE="${CMAKE_BUILD_TYPE}")

find_package(Threads REQUIRED)
add_executable(aegis_tests tests/test_aegis.c)
target_link_libraries(aegis_tests aegis_sim unity Threads::Threads)

add_executable(aegis_demo_tests tests/test_demo.c)
target_link_libraries(aegis_demo_tests aegis_demo unity)

enable_testing()
add_test(NAME aegis_tests COMMAND aegis_tests)
add_test(NAME aegis_demo_tests COMMAND aegis_demo_tests)
