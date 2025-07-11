##############################################################################
# Geogram/Vorpaline root CMakeList
##############################################################################

cmake_minimum_required(VERSION 3.5...3.30)
cmake_policy(SET CMP0048 NEW)


##############################################################################
set(VORPALINE_VERSION_RC TRUE)
set(VORPALINE_VERSION_MAJOR 1)
set(VORPALINE_VERSION_MINOR 9)
set(VORPALINE_VERSION_PATCH 7)

set(VORPALINE_VERSION_PLAIN ${VORPALINE_VERSION_MAJOR}.${VORPALINE_VERSION_MINOR}.${VORPALINE_VERSION_PATCH})
if(VORPALINE_VERSION_RC)
set(VORPALINE_VERSION ${VORPALINE_VERSION_PLAIN}-rc)
else()
set(VORPALINE_VERSION ${VORPALINE_VERSION_PLAIN})
endif()
##############################################################################

# Note: geogram.cmake defines GEOGRAM_WITH_VORPALINE
# that we could have used instead,
# but geogram.cmake needs to be included after the project()
# command, since project() resets CFLAGS and CXXFLAGS.

if(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/src/lib/vorpalib)
project(Vorpaline VERSION "${VORPALINE_VERSION_PLAIN}")
else()
project(Geogram VERSION "${VORPALINE_VERSION_PLAIN}")
endif()

# Optional modules
# (can be overriden in CMakeOptions.txt)

# Set GEOGRAM_SUB_BUILD if Geogram sources included in buildtree, then
# VORPALINE_PLATFORM can be set directly in parent CMakeLists.txt
if(NOT GEOGRAM_SUB_BUILD)
option(GEOGRAM_WITH_GRAPHICS "Viewers and geogram_gfx library" ON)
option(GEOGRAM_WITH_LEGACY_NUMERICS "Legacy numerical libraries" ON)
option(GEOGRAM_WITH_HLBFGS "Non-linear solver (Yang Liu's HLBFGS)" ON)
option(GEOGRAM_WITH_TETGEN "Tetrahedral mesher (Hang Si's TetGen)" ON)
option(GEOGRAM_WITH_TRIANGLE "Triangle mesher (Jonathan Shewchuk's triangle)" ON)
option(GEOGRAM_WITH_LUA "Built-in LUA interpreter" ON)
option(GEOGRAM_LIB_ONLY "Libraries only (no example programs/no viewer)" OFF)
option(GEOGRAM_WITH_FPG "Predicate generator (Sylvain Pion's FPG)" OFF)
option(GEOGRAM_USE_SYSTEM_GLFW3 "Use the version of GLFW3 installed in the system if found" OFF)
option(GEOGRAM_WITH_GARGANTUA "64-bit indices" OFF)
option(GEOGRAM_WITH_TBB "Use TBB for multi-threading" OFF)
include(cmake/geo_detect_platform.cmake)
endif()

if(GEOGRAM_WITH_TBB AND NOT TARGET TBB::tbb)
include(cmake/onetbb.cmake)
endif()

include(cmake/geogram.cmake)

# Override default CMAKE_INSTALL_DOCDIR: Use doc instead of CMake default DATAROOTDIR/doc/PROJECT_NAME
set(CMAKE_INSTALL_DOCDIR doc CACHE PATH "Documentation root")

# Set remaining CMAKE_INSTALL_...DIR variables
include(GNUInstallDirs)

# Install destinations for pkg-config and CMake files
set(GEOGRAM_INSTALL_PKGCONFIG_DIR ${CMAKE_INSTALL_LIBDIR}/pkgconfig CACHE PATH "pkg-config file install destination")
set(GEOGRAM_INSTALL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/modules CACHE PATH "CMake file install destination")


set(VORPALINE_INCLUDE_SUBPATH geogram${VORPALINE_VERSION_MAJOR})

# Determine the current Build-OS (Build-platform without the compiler info)
string(REGEX REPLACE "-[^-]+$" "" VORPALINE_OS ${VORPALINE_PLATFORM})

# Determine the current build date
string(TIMESTAMP VORPALINE_BUILD_DATE "%Y-%m-%d %H:%M:%S")
string(TIMESTAMP YEAR "%Y")

# Determine the current build number
# This is set by Jenkins in environment variable BUILD_NUMBER
set(VORPALINE_BUILD_NUMBER $ENV{BUILD_NUMBER})

if(GEOGRAM_WITH_GARGANTUA)
add_definitions(-DGARGANTUA)
endif()


##############################################################################
# Get SVN revision info

if(GEOGRAM_WITH_VORPALINE)
##  set(VORPALINE_SVN_REVISION ${Vorpaline_WC_REVISION})
endif()

##############################################################################
# RPATH (where executables find the .so / DLLs)
# - Enables RPATH support for MACOSX
# - Makes RPATH of dynamic libraries and executable point to the directory
#   where libraries are installed.

if(VORPALINE_BUILD_DYNAMIC)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif()

set(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")

# uninstall target
# Needs to be created before add_subdirectory() because GLFw has
# also an uninstall target that will be inhibited if there is
# already one (Geogram's one needs to be first)

if(PROJECT_IS_TOP_LEVEL)
configure_file(
"${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
"${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
IMMEDIATE @ONLY)

add_custom_target(uninstall
COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()

##############################################################################
# Geogram/Vorpaline sources

add_subdirectory(src/lib/geogram)
if(GEOGRAM_WITH_VORPALINE)
add_subdirectory(src/lib/vorpalib)
endif()

add_subdirectory(src/lib/third_party)

if(GEOGRAM_WITH_GRAPHICS)
add_subdirectory(src/lib/geogram_gfx)
endif()

if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/exploragram)
if(GEOGRAM_WITH_EXPLORAGRAM)
add_subdirectory(src/lib/exploragram)
endif()
endif()

if(NOT GEOGRAM_LIB_ONLY)
add_subdirectory(src/bin)
add_subdirectory(src/tests)
add_subdirectory(src/examples)
add_subdirectory(tests)
endif()

add_subdirectory(doc)

##############################################################################
# Cleanup from previous builds

file(REMOVE ${CMAKE_BINARY_DIR}/doc/LICENSE.txt)


##############################################################################
# Geogram installation


# FindGeogram.cmake

install(FILES cmake/FindGeogram.cmake DESTINATION ${GEOGRAM_INSTALL_CMAKE_DIR} COMPONENT devkit)

# Configure CPack

set(CPACK_PACKAGE_NAME ${CMAKE_PROJECT_NAME})
set(CPACK_SYSTEM_NAME ${VORPALINE_OS})
set(CPACK_PACKAGE_VENDOR "INRIA - ALICE")


if(${GEOGRAM_WITH_VORPALINE})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "A flexible mesh generator")
else()
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "fast, simple and easy-to-use primitives for geometric programming")
endif()

set(CPACK_PACKAGE_VERSION_MAJOR ${VORPALINE_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VORPALINE_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VORPALINE_VERSION_PATCH})
set(CPACK_PACKAGE_VERSION ${VORPALINE_VERSION})
set(CPACK_PACKAGE_FILE_NAME ${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION})
set(CPACK_COMPONENT_INCLUDE_TOPLEVEL_DIRECTORY true)

if(CPACK_GENERATOR STREQUAL "DEB")
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CPACK_PACKAGE_CONTACT Bruno.Levy@inria.fr)
# set(CPACK_PACKAGE_DEPENDS "libglfw3 (>= 3.2-1), libc6 (>= 2.22-11), libstdc++ (>= 6.1.1-4)")
#   TODO: use objdump -p | grep NEEDED to automate...
#   or GET_PROPERTY(result GLOBAL ENABLED_FEATURES)  (successful FIND_PACKAGE())
endif()

if(NOT DEFINED CPACK_GENERATOR)
if(WIN32)
set(CPACK_GENERATOR ZIP)
else()
set(CPACK_GENERATOR TGZ)
endif()
endif()

# Enable component-based packaging for archive generators (TGZ, ZIP)
set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)

set(CPACK_COMPONENTS_ALL runtime devkit devkit-full doc-devkit doc-devkit-full)
set(CPACK_COMPONENTS_GROUPING "IGNORE")

set(CPACK_COMPONENT_RUNTIME_DISPLAY_NAME "Vorpaline Application")
set(CPACK_COMPONENT_RUNTIME_GROUP "Runtime")

set(CPACK_COMPONENT_DEVKIT_DISPLAY_NAME "Vorpaline Developer Kit")
set(CPACK_COMPONENT_DEVKIT_GROUP "Development")

set(CPACK_COMPONENT_DEVKIT-FULL_DISPLAY_NAME "Vorpaline Full Developer Kit")
set(CPACK_COMPONENT_DEVKIT-FULL_GROUP "Development")

set(CPACK_COMPONENT_DOC-DEVKIT_DISPLAY_NAME "Vorpaline API Developer Kit Documentation")
set(CPACK_COMPONENT_DOC-DEVKIT_GROUP "Documentation")

set(CPACK_COMPONENT_DOC-DEVKIT-FULL_DISPLAY_NAME "Vorpaline Full Developer Kit Documentation")
set(CPACK_COMPONENT_DOC-DEVKIT-FULL_GROUP "Documentation")

set(CPACK_COMPONENT_DOC-DEVKIT-INTERNAL_DISPLAY_NAME "Vorpaline Internal Developer Kit Documentation")
set(CPACK_COMPONENT_DOC-DEVKIT-INTERNAL_GROUP "Documentation")

# Copy the helper script to build individual packages to the binary directory
configure_file(
tools/make_package.pl.in
make_package.pl
@ONLY
)

# This must always be last!
include(CPack)
