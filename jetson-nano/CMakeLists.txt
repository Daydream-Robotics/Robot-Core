cmake_minimum_required(VERSION 3.18)   # OSQP v1.0.0 needs 3.18

project(RobotCoreMPC LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# lets DreamSim link the static libs into its shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# matches the old Makefile (-O2, Eigen asserts still on) when no build type is given
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES AND NOT MSVC)
    add_compile_options(-O2)
endif()

# - - - - - - - - - - - - - - [OSQP] - - - - - - - - - - - - - -
# both pinned as git submodules; run `git submodule update --init` after cloning
set(EXTERNAL ${CMAKE_CURRENT_SOURCE_DIR}/external)
if(NOT EXISTS ${EXTERNAL}/osqp/CMakeLists.txt OR NOT EXISTS ${EXTERNAL}/qdldl/CMakeLists.txt)
    message(FATAL_ERROR "OSQP submodules missing. Run: git submodule update --init --recursive")
endif()

# OSQP would download QDLDL at configure time; use the pinned submodule instead
set(FETCHCONTENT_SOURCE_DIR_QDLDL ${EXTERNAL}/qdldl CACHE PATH "" FORCE)

# static lib only, same config as the old prebuilt one (double, 64-bit ints)
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_STATIC_LIB ON  CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE   OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS  OFF CACHE BOOL "" FORCE)
set(OSQP_USE_FLOAT        OFF CACHE BOOL "" FORCE)
set(OSQP_USE_LONG         ON  CACHE BOOL "" FORCE)
add_subdirectory(${EXTERNAL}/osqp ${CMAKE_CURRENT_BINARY_DIR}/osqp EXCLUDE_FROM_ALL)

# - - - - - - - - - - - - - - [MPC CORE] - - - - - - - - - - - - - -
# portable: Eigen + OSQP + std only. This is what DreamSim links.
add_library(robotcore_mpc STATIC
    src/motion/control/mpc.cpp
    src/motion/pathing/arclengthSplining.cpp
    src/motion/control/mpcReference.cpp
)
target_include_directories(robotcore_mpc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen/include
)
target_link_libraries(robotcore_mpc PUBLIC osqpstatic)
if(MSVC)
    target_compile_options(robotcore_mpc PRIVATE /bigobj)
endif()

# - - - - - - - - - - - - - - [NANO EXECUTABLE] - - - - - - - - - - - - - -
# only built when jetson-nano/ is the top-level project (DreamSim's add_subdirectory skips it)
# CMAKE_SOURCE_DIR check instead of PROJECT_IS_TOP_LEVEL, which needs cmake 3.21
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(_robotcore_top_level ON)
else()
    set(_robotcore_top_level OFF)
endif()
option(ROBOTCORE_BUILD_NANO_EXE "Build the Nano's MPCController serial program" ${_robotcore_top_level})

# serial link to the brain uses termios + /dev/ttyACM, so Linux (the Nano) only
if(ROBOTCORE_BUILD_NANO_EXE AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(MPCController
        src/main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../shared/src/serial/serialProtocol.cpp
    )
    target_compile_definitions(MPCController PRIVATE __microcontroller__)
    target_link_libraries(MPCController PRIVATE robotcore_mpc m)
endif()
