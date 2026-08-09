cmake_minimum_required(VERSION 3.20)

project(VectorWatch VERSION 0.2.0 LANGUAGES CXX)

add_executable(vectorwatch
    src/ConflictDetector.cpp
    src/TerminalSimulation.cpp
    src/main.cpp
)

target_include_directories(vectorwatch PRIVATE include)
target_compile_features(vectorwatch PRIVATE cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(vectorwatch PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(vectorwatch PRIVATE /W4 /permissive-)
endif()
