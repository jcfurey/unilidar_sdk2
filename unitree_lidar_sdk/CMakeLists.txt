# **********************************************************************
#  Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
# **********************************************************************
#
# NOTE: the minimum required version used to be 3.0. CMake 4.0 removed
# compatibility with CMake < 3.5 entirely, so the old value makes this project
# fail to configure with the CMake shipped by recent distributions (and by
# Ubuntu 26.04 / ROS 2 Lyrical). 3.16 is available on every distribution we
# support (Ubuntu 20.04 and newer).
cmake_minimum_required(VERSION 3.16)

project(unitree_lidar_sdk
  VERSION 2.0.11
  DESCRIPTION "Unitree LiDAR L2 SDK"
  LANGUAGES CXX
)

# ---------------------------------------------------------------------------
# Build configuration
# ---------------------------------------------------------------------------

# Only force a build type for a standalone build, and never override a choice
# made by the user or by a multi-config generator.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    Debug Release RelWithDebInfo MinSizeRel)
endif()

# Use the standard CMake mechanism instead of hard-coding -std=c++17 into
# CMAKE_CXX_FLAGS, which used to clobber flags coming from the environment or
# from a parent project.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(UNILIDAR_BUILD_EXAMPLES "Build the unilidar_sdk2 example programs" ON)

# ---------------------------------------------------------------------------
# Pre-built library
#
# The SDK ships as a pre-built static archive, one per architecture. Map the
# CMake processor name onto the directory layout in lib/ so that cross builds
# and the various names Linux reports for arm64 all work.
# ---------------------------------------------------------------------------

if(NOT UNILIDAR_SDK_ARCH)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(UNILIDAR_SDK_ARCH "x86_64")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(UNILIDAR_SDK_ARCH "aarch64")
  else()
    message(FATAL_ERROR
      "unilidar_sdk2 ships pre-built libraries for x86_64 and aarch64 only, but "
      "CMAKE_SYSTEM_PROCESSOR is '${CMAKE_SYSTEM_PROCESSOR}'. Set "
      "UNILIDAR_SDK_ARCH manually if you know which archive to use.")
  endif()
endif()

set(UNILIDAR_SDK_LIBRARY "${PROJECT_SOURCE_DIR}/lib/${UNILIDAR_SDK_ARCH}/libunilidar_sdk2.a")
if(NOT EXISTS "${UNILIDAR_SDK_LIBRARY}")
  message(FATAL_ERROR "Cannot find the unilidar_sdk2 library at ${UNILIDAR_SDK_LIBRARY}")
endif()

find_package(Threads REQUIRED)

# An imported target carries its include directories and its transitive
# dependency on pthread, so consumers only have to link `unilidar::sdk2`.
add_library(unilidar::sdk2 STATIC IMPORTED GLOBAL)
set_target_properties(unilidar::sdk2 PROPERTIES
  IMPORTED_LOCATION "${UNILIDAR_SDK_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/include"
  INTERFACE_LINK_LIBRARIES "Threads::Threads"
)

# Where a locally rebuilt archive would be written.
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib/${UNILIDAR_SDK_ARCH}")

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------

if(UNILIDAR_BUILD_EXAMPLES)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")

  foreach(example
      example_lidar_udp
      example_lidar_serial
      set_ip_address
      set_to_serial_mode
      set_to_udp_mode)
    add_executable(${example} examples/${example}.cpp)
    target_link_libraries(${example} PRIVATE unilidar::sdk2)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(${example} PRIVATE -Wall -Wextra)
    endif()
  endforeach()
endif()
