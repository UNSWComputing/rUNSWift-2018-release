cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

project(runswift)

set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")

OPTION(FULL_DEB_INFO "Include all debug info." OFF)

IF(FULL_DEB_INFO)
    # Compile includes gdb info
    add_definitions(-Wall -O0 -g)
ELSE(FULL_DEB_INFO)
    # Basic Compile
    add_definitions(-Wall)
ENDIF(FULL_DEB_INFO)


OPTION(SIMULATION "Make simulation build." OFF)

IF(SIMULATION)
    add_definitions(-DSIMULATION)
ENDIF(SIMULATION)

# Keep build process generic for v[45]
message("Toolchain File: ${CMAKE_TOOLCHAIN_FILE}")
message("CTC_DIR: ${CTC_DIR}")
add_definitions(-DCROSS_COMPILING_FOR_ROBOT)
if(${CTC_DIR} MATCHES atom)
    message("Nao v4/5 atom build detected")
    add_definitions(-march=atom -mtune=atom)
else()
    message(FATAL_ERROR "Unknown CTC_DIR")
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap-eigen.cmake")

add_subdirectory(robot)

if(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Darwin")
    message("Skipping offnao, vatnao and vatnao-legacy builds on macOS")
else()
    add_subdirectory(utils)
endif()
