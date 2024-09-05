set(VORPALINE_ARCH_64 true)
include(${GEOGRAM_SOURCE_DIR}/cmake/platforms/Linux-gcc-aarch64.cmake)
add_flags(CMAKE_CXX_FLAGS -DGEO_OS_LINUX_AARCH64)
add_flags(CMAKE_C_FLAGS -DGEO_OS_LINUX_AARCH64)

