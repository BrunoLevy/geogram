set(VORPALINE_ARCH_64 true)
set(VORPALINE_BUILD_DYNAMIC true)
include(${GEOGRAM_SOURCE_DIR}/cmake/platforms/Darwin-clang.cmake)

add_flags(CMAKE_CXX_FLAGS -m64)
add_flags(CMAKE_C_FLAGS -m64)

# Enable SSE3 instruction set on Intel architecture
if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64")
  add_flags(CMAKE_CXX_FLAGS -msse3)
  add_flags(CMAKE_C_FLAGS -msse3)
endif()


