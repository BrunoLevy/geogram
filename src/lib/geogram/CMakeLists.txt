# Tetgen (by Hang Si) is currently included by default.
# Triangle (by Jonathan Shewchuk) is currently included by default.

configure_file(geogram.pc.in ${PROJECT_BINARY_DIR}/geogram${VORPALINE_VERSION_MAJOR}.pc @ONLY)

if(GEOGRAM_WITH_GEOGRAMPLUS)
   add_subdirectory(geogramplus)
   list(APPEND SOURCES $<TARGET_OBJECTS:geogramplus>)
   if(NOT GEOGRAMPLUS_USE_GMP)
       list(APPEND SOURCES $<TARGET_OBJECTS:geogramplus_third_party>)
   endif()
   target_include_directories(
      geogramplus PRIVATE
      ${PROJECT_SOURCE_DIR}/src/lib/geogram/
      ${PROJECT_SOURCE_DIR}/src/lib/geogram/geogramplus/third_party
   )
endif()

add_subdirectory(third_party)

aux_source_directories(SOURCES "Source Files"           .)
aux_source_directories(SOURCES "Source Files\\basic"    basic)
aux_source_directories(SOURCES "Source Files\\numerics" numerics)
aux_source_directories(SOURCES "Source Files\\mesh"     mesh)
aux_source_directories(SOURCES "Source Files\\delaunay" delaunay)
aux_source_directories(SOURCES "Source Files\\voronoi"  voronoi)
aux_source_directories(SOURCES "Source Files\\points"   points)
aux_source_directories(SOURCES "Source Files\\api"      api)
aux_source_directories(SOURCES "Source Files\\NL"       NL)
aux_source_directories(SOURCES "Source Files\\image"    image)
aux_source_directories(SOURCES "Source Files\\parameterization" parameterization)
aux_source_directories(SOURCES "Source Files\\bibliography" bibliography)

if(GEOGRAM_WITH_LUA)
aux_source_directories(SOURCES "Source Files\\lua"      lua)
endif()

# Parallel Delaunay now works on both Windows and Unix !
add_definitions(-DGEOGRAM_WITH_PDEL)

# Activate AMGCL support in OpenNL (algebraic multigrid solver)
add_definitions(-DNL_WITH_AMGCL)

# Copy the default version info to the build directory
configure_file(basic/version.h.in version.h @ONLY)
list(APPEND SOURCES version.h)
set_source_files_properties(version.h PROPERTIES GENERATED true)

include_directories(${PROJECT_BINARY_DIR}/src/lib)

add_library(geogram ${SOURCES} $<TARGET_OBJECTS:geogram_third_party>)

target_compile_features(geogram PUBLIC cxx_std_17)

target_include_directories(geogram PRIVATE
${PROJECT_SOURCE_DIR}/src/lib/geogram/third_party/amgcl
)

if(GEOGRAM_WITH_GEOGRAMPLUS)
target_include_directories(geogram PUBLIC
${PROJECT_SOURCE_DIR}/src/lib/geogram/
${PROJECT_SOURCE_DIR}/src/lib/geogram/geogramplus/third_party
)
endif()

if(GEOGRAM_WITH_TBB)
target_link_libraries(geogram TBB::tbb) # TODO: Use PRIVATE keyword here
target_compile_definitions(geogram PRIVATE -DGEO_TBB)
endif()

# Jeremie/Maxence: avoids the need to redeclare geogram include
# path for targets that depend on geogram.
# See: https://cmake.org/cmake/help/v3.3/command/target_include_directories.html
# https://stackoverflow.com/questions/26243169/cmake-target-include-directories-meaning-of-scope
target_include_directories(geogram PUBLIC ${PROJECT_SOURCE_DIR}/src/lib)

if(ANDROID)
target_include_directories(geogram PRIVATE
${ANDROID_NDK}/sources/android/native_app_glue
)
message(STATUS "building for Android")
endif()


set_target_properties(geogram PROPERTIES
VERSION ${VORPALINE_VERSION}
SOVERSION ${VORPALINE_VERSION_MAJOR}
FOLDER "GEOGRAM")

if(UNIX AND VORPALINE_BUILD_DYNAMIC)
if (ANDROID)
target_link_libraries(geogram dl)
else()
target_link_libraries(geogram pthread dl)
endif()
endif()

if(WIN32)
target_link_libraries(geogram psapi)
endif()

if(GEOGRAM_WITH_GEOGRAMPLUS)
target_link_libraries(geogram ${GEOGRAMPLUS_LIBS})
endif()

# Install the library
install_devkit_targets(geogram)

# Install include files for the standard devkit
install(
DIRECTORY api
DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${VORPALINE_INCLUDE_SUBPATH}/geogram
COMPONENT devkit
FILES_MATCHING PATTERN *.h
)

# Install include files for the full devkit
install(
DIRECTORY .
DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${VORPALINE_INCLUDE_SUBPATH}/geogram
COMPONENT devkit-full
FILES_MATCHING PATTERN *.h
# Exclude all files related to licensing
REGEX /license/ EXCLUDE
)

install(
FILES "${PROJECT_BINARY_DIR}/geogram${VORPALINE_VERSION_MAJOR}.pc"
DESTINATION ${GEOGRAM_INSTALL_PKGCONFIG_DIR}
)
