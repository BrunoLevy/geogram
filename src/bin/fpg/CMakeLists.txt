vor_reset_warning_level()

# -frounding-math: disable optimizations that suppose
# that rounding mode is round to nearest (FPG uses 
# other rounding modes)
if(
  "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
  "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR
  "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang" 
)
add_definitions(-frounding-math)
endif()

set(APP_NAME mcc)
aux_source_directories(SOURCES "" .)
include_directories(.)
vor_add_executable(${APP_NAME} ${SOURCES})
target_link_libraries(${APP_NAME} geogram)

set_target_properties(${APP_NAME} PROPERTIES FOLDER "GEOGRAM/Programs")
