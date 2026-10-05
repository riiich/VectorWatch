cmake_minimum_required(VERSION 3.20)

project(VectorWatch VERSION 0.3.0 LANGUAGES CXX)

find_package(Threads REQUIRED)

option(
    VECTORWATCH_ENABLE_ASAN_UBSAN
    "Enable AddressSanitizer and UndefinedBehaviorSanitizer"
    OFF
)
option(
    VECTORWATCH_ENABLE_TSAN
    "Enable ThreadSanitizer"
    OFF
)

if(VECTORWATCH_ENABLE_ASAN_UBSAN AND VECTORWATCH_ENABLE_TSAN)
    message(FATAL_ERROR "ASan/UBSan and TSan must use separate builds")
endif()

function(vectorwatch_enable_warnings target)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(
            ${target}
            PRIVATE -Wall -Wextra -Wpedantic -Wconversion -pedantic-errors
        )
    elseif(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    endif()
endfunction()

function(vectorwatch_enable_sanitizers target)
    if(VECTORWATCH_ENABLE_ASAN_UBSAN)
        if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            message(FATAL_ERROR "ASan/UBSan configuration requires GCC or Clang")
        endif()
        target_compile_options(
            ${target}
            PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer
        )
        target_link_options(
            ${target}
            PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer
        )
    elseif(VECTORWATCH_ENABLE_TSAN)
        if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            message(FATAL_ERROR "TSan configuration requires GCC or Clang")
        endif()
        target_compile_options(
            ${target}
            PRIVATE -fsanitize=thread -fno-omit-frame-pointer
        )
        target_link_options(
            ${target}
            PRIVATE -fsanitize=thread -fno-omit-frame-pointer
        )
    endif()
endfunction()

add_library(vectorwatch_core
    src/app/CommandLineOptions.cpp
    src/detection/CollisionDetector.cpp
    src/detection/ConflictDetector.cpp
    src/model/Aircraft.cpp
    src/prediction/PredictionEngine.cpp
    src/prediction/PredictionWorkerPool.cpp
    src/scenarios/RandomEncounterGenerator.cpp
    src/scenarios/ScenarioCatalog.cpp
    src/scenarios/ScenarioSetup.cpp
    src/simulation/SimulationSession.cpp
    src/simulation/ThreadedSimulationPipeline.cpp
)

set_target_properties(vectorwatch_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(vectorwatch_core PUBLIC include)
target_compile_features(vectorwatch_core PUBLIC cxx_std_14)
set_target_properties(
    vectorwatch_core
    PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)
target_link_libraries(vectorwatch_core PUBLIC Threads::Threads)
vectorwatch_enable_warnings(vectorwatch_core)
vectorwatch_enable_sanitizers(vectorwatch_core)

add_executable(vectorwatch
    src/app/Application.cpp
    src/app/BenchmarkRunner.cpp
    src/main.cpp
    src/platform/TerminalCapabilities.cpp
    src/simulation/TerminalRadarRenderer.cpp
    src/simulation/TerminalSimulation.cpp
)

target_link_libraries(vectorwatch PRIVATE vectorwatch_core Threads::Threads)
set_target_properties(
    vectorwatch
    PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)
vectorwatch_enable_warnings(vectorwatch)
vectorwatch_enable_sanitizers(vectorwatch)

include(CTest)

if(BUILD_TESTING)
    add_executable(vectorwatch_tests
        tests/CollisionDetectorTests.cpp
        tests/CommandLineOptionsTests.cpp
        tests/ConflictDetectorTests.cpp
        tests/PredictionEngineTests.cpp
        tests/RandomEncounterTests.cpp
        tests/SimulationSessionTests.cpp
        tests/ThreadedSimulationPipelineTests.cpp
    )

    target_link_libraries(vectorwatch_tests PRIVATE vectorwatch_core)
    set_target_properties(
        vectorwatch_tests
        PROPERTIES
            CXX_STANDARD 14
            CXX_STANDARD_REQUIRED YES
            CXX_EXTENSIONS NO
    )
    vectorwatch_enable_warnings(vectorwatch_tests)
    vectorwatch_enable_sanitizers(vectorwatch_tests)

    add_test(NAME detection COMMAND vectorwatch_tests)
    set_tests_properties(detection PROPERTIES TIMEOUT 15)

    add_test(
        NAME simulation_head_on_crash
        COMMAND vectorwatch --simulate head-on 1000000
    )
    add_test(
        NAME simulation_crossing_crash
        COMMAND vectorwatch --simulate crossing 1000000
    )
    add_test(
        NAME simulation_initial_collision_crash
        COMMAND vectorwatch --simulate initial-collision 1000000
    )
    add_test(
        NAME simulation_offset_convergence_crash
        COMMAND vectorwatch --simulate offset-convergence 1000000
    )
    add_test(
        NAME simulation_vertical_convergence_crash
        COMMAND vectorwatch --simulate vertical-convergence 1000000
    )

    set_tests_properties(
        simulation_head_on_crash
        simulation_crossing_crash
        simulation_initial_collision_crash
        simulation_offset_convergence_crash
        simulation_vertical_convergence_crash
        PROPERTIES
            PASS_REGULAR_EXPRESSION "Status: CRASHED"
            FAIL_REGULAR_EXPRESSION "Simulation complete"
            TIMEOUT 5
    )

    set(vectorwatch_non_collision_scenarios
        parallel
        different-altitudes
        near-miss
        staggered-crossing
        diverging
        outside-lookahead
    )

    foreach(scenario IN LISTS vectorwatch_non_collision_scenarios)
        string(REPLACE "-" "_" test_suffix "${scenario}")
        add_test(
            NAME simulation_${test_suffix}_completes
            COMMAND vectorwatch --simulate ${scenario} 1000000
        )
        set_tests_properties(
            simulation_${test_suffix}_completes
            PROPERTIES
                PASS_REGULAR_EXPRESSION "Simulation complete"
                FAIL_REGULAR_EXPRESSION "Status: CRASHED"
                TIMEOUT 5
        )
    endforeach()

    add_test(
        NAME simulation_random_encounter_runtime
        COMMAND vectorwatch --simulate random-encounter 1000000
    )
    set_tests_properties(
        simulation_random_encounter_runtime
        PROPERTIES
            PASS_REGULAR_EXPRESSION "Random seed:"
            TIMEOUT 5
    )

    add_test(
        NAME prediction_benchmark_smoke
        COMMAND vectorwatch --benchmark --samples 1000 --repetitions 1 --threads 2
    )
    set_tests_properties(
        prediction_benchmark_smoke
        PROPERTIES
            PASS_REGULAR_EXPRESSION "All worker configurations verified identical counts"
            TIMEOUT 10
    )

    add_test(
        NAME simulation_random_seeded_crash
        COMMAND vectorwatch --simulate random-encounter
            --speed 1000000 --scenario-seed 10 --outcome collision
    )
    set_tests_properties(
        simulation_random_seeded_crash
        PROPERTIES
            PASS_REGULAR_EXPRESSION "Status: CRASHED"
            FAIL_REGULAR_EXPRESSION "Simulation complete"
            TIMEOUT 5
    )

    add_test(
        NAME simulation_random_seeded_miss
        COMMAND vectorwatch --simulate random-encounter
            --speed 1000000 --scenario-seed 8 --outcome pass
    )
    set_tests_properties(
        simulation_random_seeded_miss
        PROPERTIES
            PASS_REGULAR_EXPRESSION "Simulation complete"
            FAIL_REGULAR_EXPRESSION "Status: CRASHED"
            TIMEOUT 5
    )

    add_test(
        NAME simulation_threaded_prediction
        COMMAND vectorwatch
            --simulate random-encounter
            --speed 1000000
            --scenario-seed 8
            --outcome pass
            --prediction probabilistic
            --execution threaded
            --threads 2
            --samples 1000
    )
    set_tests_properties(
        simulation_threaded_prediction
        PROPERTIES
            PASS_REGULAR_EXPRESSION "Workers: 2"
            FAIL_REGULAR_EXPRESSION "Status: CRASHED"
            TIMEOUT 10
    )
endif()

add_library(vectorwatch_native SHARED src/bridge/VectorWatchBridge.cpp)
target_link_libraries(vectorwatch_native PRIVATE vectorwatch_core)
target_compile_definitions(vectorwatch_native PRIVATE VECTORWATCH_BRIDGE_EXPORTS)
set_target_properties(vectorwatch_native PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
vectorwatch_enable_warnings(vectorwatch_native)
vectorwatch_enable_sanitizers(vectorwatch_native)
