
cmake_minimum_required(VERSION 3.18)

project(jetson_nano_lidar CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES AND NOT MSVC)
    add_compile_options(-O2)
endif()

# ============================================================
# DIRECTORIES
# ============================================================

set(EXTERNAL "${CMAKE_CURRENT_SOURCE_DIR}/external")
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(SHARED_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../shared/include")

# ============================================================
# OSQP
# ============================================================

if(NOT EXISTS "${EXTERNAL}/osqp/CMakeLists.txt"
        OR NOT EXISTS "${EXTERNAL}/qdldl/CMakeLists.txt")
    message(FATAL_ERROR
            "OSQP submodules missing. Run: git submodule update --init --recursive"
    )
endif()

set(FETCHCONTENT_SOURCE_DIR_QDLDL
        "${EXTERNAL}/qdldl" CACHE PATH "" FORCE
)

set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
set(OSQP_USE_FLOAT OFF CACHE BOOL "" FORCE)
set(OSQP_USE_LONG ON CACHE BOOL "" FORCE)

add_subdirectory(
        "${EXTERNAL}/osqp"
        "${CMAKE_CURRENT_BINARY_DIR}/osqp"
        EXCLUDE_FROM_ALL
)

# ============================================================
# MPC CORE
# ============================================================

add_library(robotcore_mpc STATIC
        src/motion/control/mpc.cpp
        src/motion/pathing/arclengthSplining.cpp
        src/motion/control/mpcReference.cpp
)

target_include_directories(robotcore_mpc PUBLIC
        "${INCLUDE_DIR}"
        "${SHARED_INCLUDE_DIR}"
        "${EXTERNAL}/eigen/include"
)

target_link_libraries(robotcore_mpc PUBLIC osqpstatic)

if(MSVC)
    target_compile_options(robotcore_mpc PRIVATE /bigobj)
endif()

# ============================================================
# PLATFORM-INDEPENDENT LIDAR UTILITIES
# ============================================================

# Safe for DreamSim/macOS; does not require Slamtec SDK.
add_library(robotcore_lidar STATIC
        src/perception/lidar/adaptiveThreshold.cpp
)

target_include_directories(robotcore_lidar PUBLIC
        "${INCLUDE_DIR}"
        "${SHARED_INCLUDE_DIR}"
        "${EXTERNAL}/eigen/include"
)

# ============================================================
# BUILD OPTIONS
# ============================================================

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(_robotcore_top_level ON)
else()
    set(_robotcore_top_level OFF)
endif()

option(ROBOTCORE_BUILD_NANO_EXE
        "Build the Linux Jetson Nano MPCController"
        ${_robotcore_top_level}
)

option(ROBOTCORE_BUILD_LIDAR_DRIVER
        "Build the Slamtec SDK-based LiDAR driver"
        ${_robotcore_top_level}
)

option(ROBOTCORE_BUILD_LIDAR_TEST
        "Build the LiDAR SDK integration test"
        ${_robotcore_top_level}
)

# ============================================================
# SLAMTEC SDK AND LIDAR DRIVER
# ============================================================

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
            "${INCLUDE_DIR}"
    )

    target_link_libraries(robotcore_lidar_driver PUBLIC
            slamtec_lidar_sdk
            Threads::Threads
    )
endif()

# ============================================================
# JETSON NANO EXECUTABLE (LINUX ONLY)
# ============================================================

if(ROBOTCORE_BUILD_NANO_EXE AND CMAKE_SYSTEM_NAME STREQUAL "Linux")

    find_package(Threads REQUIRED)

    add_executable(MPCController
            src/main.cpp
            src/link/serial.cpp
            src/link/serialProtocol.cpp
            ../shared/src/protocol/serialProtocol.cpp
    )

    target_include_directories(MPCController PRIVATE
            "${INCLUDE_DIR}"
            "${SHARED_INCLUDE_DIR}"
    )

    target_compile_definitions(MPCController PRIVATE
            __jetson_nano__
    )

    target_link_libraries(MPCController PRIVATE
            robotcore_mpc
            robotcore_lidar
            Threads::Threads
            m
    )

    if(TARGET robotcore_lidar_driver)
        target_link_libraries(MPCController PRIVATE
                robotcore_lidar_driver
        )
    endif()

elseif(ROBOTCORE_BUILD_NANO_EXE)
    message(STATUS
            "MPCController is Linux-only; skipping on ${CMAKE_SYSTEM_NAME}"
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
