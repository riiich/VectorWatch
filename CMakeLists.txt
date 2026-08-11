cmake_minimum_required(VERSION 3.20)

project(VectorWatch VERSION 0.2.0 LANGUAGES CXX)

find_package(Threads REQUIRED)

function(vectorwatch_enable_warnings target)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
    elseif(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    endif()
endfunction()

add_library(vectorwatch_core
    src/detection/CollisionDetector.cpp
    src/detection/ConflictDetector.cpp
    src/model/Aircraft.cpp
)

target_include_directories(vectorwatch_core PUBLIC include)
target_compile_features(vectorwatch_core PUBLIC cxx_std_20)
vectorwatch_enable_warnings(vectorwatch_core)

add_executable(vectorwatch
    src/app/Application.cpp
    src/main.cpp
    src/scenarios/ScenarioCatalog.cpp
    src/simulation/TerminalRadarRenderer.cpp
    src/simulation/TerminalSimulation.cpp
)

target_link_libraries(vectorwatch PRIVATE vectorwatch_core Threads::Threads)
vectorwatch_enable_warnings(vectorwatch)

include(CTest)

if(BUILD_TESTING)
    add_executable(vectorwatch_tests
        tests/CollisionDetectorTests.cpp
        tests/ConflictDetectorTests.cpp
    )

    target_link_libraries(vectorwatch_tests PRIVATE vectorwatch_core)
    vectorwatch_enable_warnings(vectorwatch_tests)

    add_test(NAME detection COMMAND vectorwatch_tests)
endif()
