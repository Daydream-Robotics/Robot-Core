cmake_minimum_required(VERSION 3.18)   # OSQP v1.0.0 needs 3.18

project(RobotCoreMPC LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# lets DreamSim link the static libs into its shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# writes build/compile_commands.json for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# - - - - - - - - - - - - - - [LIDAR CORE] - - - - - - - - - - - - - -
# portable: Eigen + std only, so the simulator and tests can link it too
add_library(robotcore_lidar STATIC
        src/perception/lidar/adaptiveThreshold.cpp
)
target_include_directories(robotcore_lidar PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../shared/include
        ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen/include
)

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
    find_package(Threads REQUIRED)
    add_executable(MPCController
            src/main.cpp
            src/link/serial.cpp
            src/link/serialProtocol.cpp
            ../shared/src/protocol/serialProtocol.cpp
    )
    target_compile_definitions(MPCController PRIVATE __jetson_nano__)
    target_link_libraries(MPCController PRIVATE robotcore_mpc robotcore_lidar Threads::Threads m)
endif()

# - - - - - - - - - - - - - - [LIDAR SDK DRIVER] - - - - - - - - - - - - - -
# Default off when included by DreamSim, on for standalone Nano builds.
option(ROBOTCORE_BUILD_LIDAR_DRIVER
        "Build the Slamtec SDK LiDAR driver" ${_robotcore_top_level})
option(ROBOTCORE_BUILD_LIDAR_TEST
        "Build the LiDAR SDK integration test" OFF)

# The SDK is needed only on supported host platforms when requested.
# The original platform-independent lidar target remains unchanged.

# A test requires the driver even if the driver option is OFF.
if((WIN32 OR CMAKE_SYSTEM_NAME STREQUAL "Linux")
        AND (ROBOTCORE_BUILD_LIDAR_DRIVER OR ROBOTCORE_BUILD_LIDAR_TEST))

    find_package(Threads REQUIRED)

    set(SLAMTEC_SDK_ROOT
            "${EXTERNAL}/rplidar_sdk"
            CACHE PATH "Slamtec rplidar_sdk root"
    )

    if(NOT EXISTS "${SLAMTEC_SDK_ROOT}/sdk/include/sl_lidar.h")
        message(FATAL_ERROR
                "Slamtec SDK missing. Run: git submodule update --init --recursive"
        )
    endif()

    # Apply the required SDK queue safety fix.
    # Safe to run on both fresh and previously configured builds.
    find_package(Git REQUIRED)

    set(SLAMTEC_SDK_PATCH
            "${CMAKE_CURRENT_LIST_DIR}/patches/rplidar_sdk-empty-queue.patch"
    )

    if(NOT EXISTS "${SLAMTEC_SDK_PATCH}")
        message(FATAL_ERROR
                "Missing SDK patch: ${SLAMTEC_SDK_PATCH}")
    endif()

    # A successful reverse check means the patch is already applied.
    execute_process(
            COMMAND "${GIT_EXECUTABLE}" -C "${SLAMTEC_SDK_ROOT}"
            apply --reverse --check "${SLAMTEC_SDK_PATCH}"
            RESULT_VARIABLE SDK_ALREADY_PATCHED
            OUTPUT_QUIET
            ERROR_QUIET
    )

    if(SDK_ALREADY_PATCHED EQUAL 0)
        message(STATUS "Slamtec SDK safety patch already applied")
    else()
        # Check compatibility before changing the SDK.
        execute_process(
                COMMAND "${GIT_EXECUTABLE}" -C "${SLAMTEC_SDK_ROOT}"
                apply --check "${SLAMTEC_SDK_PATCH}"
                RESULT_VARIABLE SDK_PATCH_CHECK
                ERROR_VARIABLE SDK_PATCH_ERROR
        )

        if(NOT SDK_PATCH_CHECK EQUAL 0)
            message(FATAL_ERROR
                    "Slamtec SDK patch incompatible: ${SDK_PATCH_ERROR}")
        endif()

        execute_process(
                COMMAND "${GIT_EXECUTABLE}" -C "${SLAMTEC_SDK_ROOT}"
                apply "${SLAMTEC_SDK_PATCH}"
                RESULT_VARIABLE SDK_PATCH_RESULT
                ERROR_VARIABLE SDK_PATCH_ERROR
        )

        if(NOT SDK_PATCH_RESULT EQUAL 0)
            message(FATAL_ERROR
                    "Failed to patch Slamtec SDK: ${SDK_PATCH_ERROR}")
        endif()

        message(STATUS "Slamtec SDK safety patch applied")
    endif()

    set(SLAMTEC_SDK_SOURCES
            "${SLAMTEC_SDK_ROOT}/sdk/src/sl_lidar_driver.cpp"
            "${SLAMTEC_SDK_ROOT}/sdk/src/hal/thread.cpp"
            "${SLAMTEC_SDK_ROOT}/sdk/src/sl_crc.cpp"
            "${SLAMTEC_SDK_ROOT}/sdk/src/sl_serial_channel.cpp"
            "${SLAMTEC_SDK_ROOT}/sdk/src/sl_lidarprotocol_codec.cpp"
            "${SLAMTEC_SDK_ROOT}/sdk/src/sl_async_transceiver.cpp"
            "${SLAMTEC_SDK_ROOT}/sdk/src/sl_tcp_channel.cpp"
            "${SLAMTEC_SDK_ROOT}/sdk/src/sl_udp_channel.cpp"
    )

    file(GLOB_RECURSE SLAMTEC_SDK_UNPACKERS
            CONFIGURE_DEPENDS
            "${SLAMTEC_SDK_ROOT}/sdk/src/dataunpacker/*.cpp"
    )

    if(WIN32)
        set(_slamtec_platform_dir
                "${SLAMTEC_SDK_ROOT}/sdk/src/arch/win32"
        )
    else()
        set(_slamtec_platform_dir
                "${SLAMTEC_SDK_ROOT}/sdk/src/arch/linux"
        )
    endif()

    set(SLAMTEC_SDK_PLATFORM_SOURCES
            "${_slamtec_platform_dir}/net_serial.cpp"
            "${_slamtec_platform_dir}/net_socket.cpp"
            "${_slamtec_platform_dir}/timer.cpp"
    )

    add_library(slamtec_lidar_sdk STATIC
            ${SLAMTEC_SDK_SOURCES}
            ${SLAMTEC_SDK_UNPACKERS}
            ${SLAMTEC_SDK_PLATFORM_SOURCES}
    )

    target_include_directories(slamtec_lidar_sdk PUBLIC
            "${SLAMTEC_SDK_ROOT}/sdk/include"
    )

    target_include_directories(slamtec_lidar_sdk PRIVATE
            "${SLAMTEC_SDK_ROOT}/sdk/src"
            "${SLAMTEC_SDK_ROOT}/sdk/src/dataunpacker"
            "${SLAMTEC_SDK_ROOT}/sdk/src/dataunpacker/unpacker"
    )

    target_link_libraries(slamtec_lidar_sdk PUBLIC
            Threads::Threads
    )

    if(WIN32)
        target_link_libraries(slamtec_lidar_sdk PUBLIC ws2_32)
        target_compile_definitions(slamtec_lidar_sdk PRIVATE WIN32)
    endif()

    add_library(robotcore_lidar_driver STATIC
            src/perception/lidar/lidarDriver.cpp
    )

    target_include_directories(robotcore_lidar_driver PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )

    target_link_libraries(robotcore_lidar_driver PUBLIC
            slamtec_lidar_sdk
            Threads::Threads
    )
endif()

# ============================================================
# LIDAR INTEGRATION TEST
# ============================================================

if(ROBOTCORE_BUILD_LIDAR_TEST
        AND TARGET robotcore_lidar_driver)

    add_executable(lidar_sdk_test
            src/perception/lidar/lidarSdkTest.cpp
    )

    target_link_libraries(lidar_sdk_test PRIVATE
            robotcore_lidar_driver
    )

elseif(ROBOTCORE_BUILD_LIDAR_TEST)
    message(STATUS
            "LiDAR SDK test unavailable on ${CMAKE_SYSTEM_NAME}"
    )
endif()

