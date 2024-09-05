
if(NOT DEFINED GEOGRAM_SOURCE_DIR)
   set(GEOGRAM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
endif()

if(EXISTS ${GEOGRAM_SOURCE_DIR}/CMakeOptions.txt)
   message(STATUS "Using local options file: ${GEOGRAM_SOURCE_DIR}/CMakeOptions.txt")
   include(${GEOGRAM_SOURCE_DIR}/CMakeOptions.txt)
endif()

# Make sure that VORPALINE_PLATFORM is defined
if(NOT VORPALINE_PLATFORM)
     if(WIN32)
        message(
           STATUS
           " Using Win-vs-generic (default),\n"
           " (if need be, use CMake variable VORPALINE_PLATFORM to override)."
        )
        set(VORPALINE_PLATFORM Win-vs-generic)
     else()
        message(FATAL_ERROR
           " CMake variable VORPALINE_PLATFORM is not defined.\n"
           " Please run configure.{sh,bat} to setup the build tree."
        )
     endif()
endif()

# Determine whether Geogram is built with Vorpaline
if(IS_DIRECTORY ${GEOGRAM_SOURCE_DIR}/src/lib/vorpalib)
   set(GEOGRAM_WITH_VORPALINE ON)
else()
   set(GEOGRAM_WITH_VORPALINE OFF)
endif()

# Determine whether Geogram is built with Vorpaline
if(IS_DIRECTORY ${GEOGRAM_SOURCE_DIR}/src/lib/geogram/geogramplus)
   set(GEOGRAM_WITH_GEOGRAMPLUS ON)
else()
   set(GEOGRAM_WITH_GEOGRAMPLUS OFF)
endif()

# Determine whether Geogram is built with Exploragram
if(NOT DEFINED GEOGRAM_WITH_EXPLORAGRAM)
   if(IS_DIRECTORY ${GEOGRAM_SOURCE_DIR}/src/lib/exploragram)
      set(GEOGRAM_WITH_EXPLORAGRAM ON)
   else()
      set(GEOGRAM_WITH_EXPLORAGRAM OFF)
   endif()
endif()

if (GEOGRAM_WITH_GEOGRAMPLUS)
   message(STATUS "addon: geogramplus")
   add_definitions(-DGEOGRAM_WITH_GEOGRAMPLUS)
endif()

if (GEOGRAM_WITH_VORPALINE)
   message(STATUS "addon: Vorpaline")
   add_definitions(-DGEOGRAM_WITH_VORPALINE)
endif()

if(GEOGRAM_WITH_HLBFGS)
   add_definitions(-DGEOGRAM_WITH_HLBFGS)
endif()

if(GEOGRAM_WITH_TETGEN)
   add_definitions(-DGEOGRAM_WITH_TETGEN)
endif()

if(GEOGRAM_WITH_TRIANGLE)
   add_definitions(-DGEOGRAM_WITH_TRIANGLE)
endif()

if(GEOGRAM_WITH_LUA)
   add_definitions(-DGEOGRAM_WITH_LUA)
endif()

# This test is there to keep CMake happy about unused variable CMAKE_BUILD_TYPE
if(CMAKE_BUILD_TYPE STREQUAL "")
endif()

##############################################################################

include(${GEOGRAM_SOURCE_DIR}/cmake/utilities.cmake)
include(${GEOGRAM_SOURCE_DIR}/cmake/platforms/${VORPALINE_PLATFORM}/config.cmake)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Static versus dynamic builds
if(VORPALINE_BUILD_DYNAMIC)
    set(BUILD_SHARED_LIBS TRUE)
    # Object files in OBJECT libraries are compiled in static mode, even if
    # BUILD_SHARED_LIBS is true! We must set CMAKE_POSITION_INDEPENDENT_CODE
    # to force compilation in dynamic mode.
    set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
    add_definitions(-DGEO_DYNAMIC_LIBS)
else()
    set(BUILD_SHARED_LIBS FALSE)
endif()

##############################################################################

# RELATIVE_OUTPUT_DIR: Binary build directory relative to source directory
# Since the build tree is a subdirectory of the source tree, it is
#  found by replacing the source dir with an empty string in the bin dir.

string(REPLACE ${CMAKE_SOURCE_DIR} "" RELATIVE_OUTPUT_DIR ${CMAKE_BINARY_DIR})

# RELATIVE_BIN_DIR
# RELATIVE_LIB_DIR
#  Directories where libraries and executables are generated, relative to
# source directory. These variables are to be used as an input (e.g., link
# path, or path to find GOMGEN executable)
#
# RELATIVE_BIN_OUTPUT_DIR
# RELATIVE_LIB_OUTPUT_DIR
#  Directory where libraries and executables are generated as specified
# as an output (does not include configuration name under MSVC, because
# MSVC adds it automatically, thus it would be there twice if we add it).
# This is where plugins are supposed to copy their generated DLL's/so's.
#
# Note: Ideally one should use the generator expressions TARGET_FILE_DIR
# and TARGET_RUNTIME_DLLS to copy runtime dependencies as a post-build
# action. See for example:
# https://cmake.org/cmake/help/latest/manual/cmake-generator-expressions.7.html#genex:TARGET_RUNTIME_DLLS
#
# 07/07/2022 replaced "if(WIN32 AND GENERATOR_IS_MULTI_CONFIG)" with
# "if(WIN32)" in test above (was breaking compilation of Graphite).

if(WIN32 AND NOT(CMAKE_GENERATOR STREQUAL "Ninja"))
    set(MSVC_CONFIG \$\(Configuration\))
    set(RELATIVE_BIN_DIR ${RELATIVE_OUTPUT_DIR}/bin/${MSVC_CONFIG}/)
    set(RELATIVE_LIB_DIR ${RELATIVE_OUTPUT_DIR}/lib/${MSVC_CONFIG}/)
else()
    set(RELATIVE_BIN_DIR ${RELATIVE_OUTPUT_DIR}/bin/)
    set(RELATIVE_LIB_DIR ${RELATIVE_OUTPUT_DIR}/lib/)
endif()

set(RELATIVE_BIN_OUTPUT_DIR ${RELATIVE_OUTPUT_DIR}/bin/)
set(RELATIVE_LIB_OUTPUT_DIR ${RELATIVE_OUTPUT_DIR}/lib/)

##############################################################################

include_directories(${GEOGRAM_SOURCE_DIR}/src/lib)
include_directories(${GEOGRAM_SOURCE_DIR}/src/lib/geogram_gfx/third_party/)
link_directories(${GEOGRAM_SOURCE_DIR}/${RELATIVE_LIB_DIR})


if(GEOGRAM_WITH_VORPALINE)
    include_directories(${GEOGRAM_SOURCE_DIR}/src/lib/vorpalib)
endif()

##############################################################################
