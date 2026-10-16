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
set(THIRD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
if(NOT EXISTS ${THIRD_PARTY}/osqp/CMakeLists.txt OR NOT EXISTS ${THIRD_PARTY}/qdldl/CMakeLists.txt)
    message(FATAL_ERROR "OSQP submodules missing. Run: git submodule update --init --recursive")
endif()

# OSQP would download QDLDL at configure time; use the pinned submodule instead
set(FETCHCONTENT_SOURCE_DIR_QDLDL ${THIRD_PARTY}/qdldl CACHE PATH "" FORCE)

# static lib only, same config as the old prebuilt one (double, 64-bit ints)
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_STATIC_LIB ON  CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE   OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS  OFF CACHE BOOL "" FORCE)
set(OSQP_USE_FLOAT        OFF CACHE BOOL "" FORCE)
set(OSQP_USE_LONG         ON  CACHE BOOL "" FORCE)
add_subdirectory(${THIRD_PARTY}/osqp ${CMAKE_CURRENT_BINARY_DIR}/osqp EXCLUDE_FROM_ALL)

# - - - - - - - - - - - - - - [MPC CORE] - - - - - - - - - - - - - -
# portable: Eigen + OSQP + std only. This is what DreamSim links.
add_library(robotcore_mpc STATIC
    src/mpc.cpp
    src/arclengthSplining.cpp
    src/mpcReference.cpp
)
target_include_directories(robotcore_mpc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/eigen
)
target_link_libraries(robotcore_mpc PUBLIC osqpstatic)
if(MSVC)
    target_compile_options(robotcore_mpc PRIVATE /bigobj)
endif()

# - - - - - - - - - - - - - - [NANO EXECUTABLE] - - - - - - - - - - - - - -
# serial link to the brain needs termios, so Linux/macOS only
if(UNIX)
    add_executable(MPCController
        src/microcontroller_main.cpp
        src/serialProtocol.cpp
    )
    target_link_libraries(MPCController PRIVATE robotcore_mpc m)
endif()
