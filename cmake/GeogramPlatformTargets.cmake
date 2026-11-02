include_guard(GLOBAL)

#-------------------------------------------------------------------
# geogram::warnings
#   was: FULL_WARNINGS / NORMAL_WARNINGS + vor_reset_warning_level()
#-------------------------------------------------------------------
add_library(geogram_warnings INTERFACE)
add_library(geogram::warnings ALIAS geogram_warnings)

target_compile_options(geogram_warnings INTERFACE
    # ---- GCC (Linux-gcc.cmake / Linux-gcc-aarch64.cmake) ----
    $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wall;-Wextra;-pedantic;-Wno-long-long;-Wconversion;-Wsign-conversion;-Wdouble-promotion;-Wnon-virtual-dtor>
    $<$<COMPILE_LANG_AND_ID:C,GNU>:-Wall;-Wextra;-pedantic;-Wno-long-long;-Wconversion;-Wsign-conversion;-Wdouble-promotion>
    # ---- Clang on Linux (Linux-clang.cmake) ----
    $<$<AND:$<PLATFORM_ID:Linux>,$<COMPILE_LANG_AND_ID:CXX,Clang>>:-Weverything;-Wno-padded;-Wno-float-equal;-Wno-global-constructors;-Wno-exit-time-destructors;-Wno-old-style-cast;-Wno-format-nonliteral;-Wno-disabled-macro-expansion;-Wno-unknown-warning-option;-Qunused-arguments;-Wno-c++98-compat>
    $<$<AND:$<PLATFORM_ID:Linux>,$<COMPILE_LANG_AND_ID:C,Clang>>:-Wall;-Wextra;-Wno-unknown-warning-option>
    # ---- Clang on Darwin (Darwin-clang.cmake) ----
    $<$<AND:$<PLATFORM_ID:Darwin>,$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>>:-Weverything;-Wno-padded;-Wno-float-equal;-Wno-global-constructors;-Wno-exit-time-destructors;-Wno-old-style-cast;-Wno-format-nonliteral;-Wno-poison-system-directories;-Wno-switch-default;-Qunused-arguments;-stdlib=libc++;-Wno-c++98-compat>
    $<$<AND:$<PLATFORM_ID:Darwin>,$<COMPILE_LANG_AND_ID:C,Clang,AppleClang>>:-Wall;-Wextra>
    # ---- MSVC (Windows-vs.cmake) ----
    $<$<CXX_COMPILER_ID:MSVC>:/W4;/wd4275;/wd4996;/wd4512;/bigobj>
    $<$<C_COMPILER_ID:MSVC>:/W4;/bigobj>
)
target_compile_definitions(geogram_warnings INTERFACE
    # Clang/libc++ C++23 header split, added in Linux-clang.cmake / Darwin-clang.cmake
    $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:_LIBCPP_REMOVE_TRANSITIVE_INCLUDES>
)
if(VORPALINE_WITH_CLANGSA)
    target_compile_options(geogram_warnings INTERFACE
        $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:--analyze>)
endif()

#-------------------------------------------------------------------
#  geogram::warnings_relaxed
#-------------------------------------------------------------------
add_library(geogram_warnings_relaxed INTERFACE)
add_library(geogram::warnings_relaxed ALIAS geogram_warnings_relaxed)

target_compile_options(geogram_warnings_relaxed INTERFACE
    $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wall;-Wextra>
    $<$<COMPILE_LANG_AND_ID:C,GNU>:-Wall;-Wextra>
    $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wall;-Wextra>
    $<$<COMPILE_LANG_AND_ID:C,Clang,AppleClang>:-Wall;-Wextra>
    $<$<CXX_COMPILER_ID:MSVC>:/W3;/wd4245;/wd4389>
)

#-------------------------------------------------------------------
# geogram::numeric
#   was: -frounding-math -ffp-contract=off (all Linux/Darwin compiler
#   files) and -D_FORTIFY_SOURCE=2 in Release, which is present in
#   Linux-gcc.cmake and Darwin-clang.cmake but NOT in Linux-clang.cmake
#-------------------------------------------------------------------
add_library(geogram_numeric INTERFACE)
add_library(geogram::numeric ALIAS geogram_numeric)

target_compile_options(geogram_numeric INTERFACE
    $<$<AND:$<NOT:$<PLATFORM_ID:Windows>>,$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>>:-frounding-math;-ffp-contract=off>
    $<$<AND:$<NOT:$<PLATFORM_ID:Windows>>,$<COMPILE_LANG_AND_ID:C,GNU,Clang,AppleClang>>:-frounding-math;-ffp-contract=off>
    $<$<AND:$<COMPILE_LANG_AND_ID:CXX,GNU>,$<CONFIG:Release>>:-D_FORTIFY_SOURCE=2>
    $<$<AND:$<COMPILE_LANG_AND_ID:C,GNU>,$<CONFIG:Release>>:-D_FORTIFY_SOURCE=2>
    $<$<AND:$<PLATFORM_ID:Darwin>,$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>,$<CONFIG:Release>>:-D_FORTIFY_SOURCE=2>
    $<$<AND:$<PLATFORM_ID:Darwin>,$<COMPILE_LANG_AND_ID:C,Clang,AppleClang>,$<CONFIG:Release>>:-D_FORTIFY_SOURCE=2>
)

#-------------------------------------------------------------------
# geogram::visibility -
#   was -fPIC -fvisibility=hidden (all Linux/Darwin compiler files)
#-------------------------------------------------------------------
add_library(geogram_visibility INTERFACE)
add_library(geogram::visibility ALIAS geogram_visibility)

target_compile_options(geogram_visibility INTERFACE
    $<$<AND:$<NOT:$<PLATFORM_ID:Windows>>,$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>>:-fPIC;-fvisibility=hidden>
    $<$<AND:$<NOT:$<PLATFORM_ID:Windows>>,$<COMPILE_LANG_AND_ID:C,GNU,Clang,AppleClang>>:-fPIC;-fvisibility=hidden>
)

#-------------------------------------------------------------------
# geogram::linux_linker 
#   was -Wl,--no-undefined (Linux.cmake).
#-------------------------------------------------------------------
add_library(geogram_linux_linker INTERFACE)
add_library(geogram::linux_linker ALIAS geogram_linux_linker)

target_link_options(geogram_linux_linker INTERFACE
    $<$<PLATFORM_ID:Linux>:-Wl,--no-undefined>
)

#-------------------------------------------------------------------
# geogram::openmp
#   was: -fopenmp in Linux-gcc.cmake / Linux-gcc-aarch64.cmake.
#   Explicitly NOT enabled under Clang: both Linux-clang.cmake and
#   Darwin-clang.cmake disable it ("NOT YET SUPPORTED in clang 3",
#   SET(USE_OPENMP FALSE))
#-------------------------------------------------------------------
find_package(OpenMP QUIET)
add_library(geogram_openmp INTERFACE)
add_library(geogram::openmp ALIAS geogram_openmp)
if(OpenMP_CXX_FOUND AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_libraries(geogram_openmp INTERFACE OpenMP::OpenMP_CXX)
endif()

#-------------------------------------------------------------------
# geogram::sanitizers
#   was: VORPALINE_WITH_ASAN / VORPALINE_WITH_TSAN. TSan is a hard
#   FATAL_ERROR upstream in every compiler file
#-------------------------------------------------------------------
add_library(geogram_sanitizers INTERFACE)
add_library(geogram::sanitizers ALIAS geogram_sanitizers)

if(VORPALINE_WITH_ASAN)
    if(NOT BUILD_SHARED_LIBS)
        message(WARNING "AddressSanitizer can be used with dynamic builds only")
        set(VORPALINE_WITH_ASAN false)
    else()
        target_compile_options(geogram_sanitizers INTERFACE
            $<$<CONFIG:Debug>:-fsanitize=address;-fno-omit-frame-pointer>)
        target_link_options(geogram_sanitizers INTERFACE
            $<$<CONFIG:Debug>:-fsanitize=address>)
    endif()
endif()

if(NOT VORPALINE_WITH_ASAN)
    target_compile_options(geogram_sanitizers INTERFACE
        $<$<AND:$<CONFIG:Debug>,$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>>:-fstack-protector-all>
        $<$<AND:$<CONFIG:Debug>,$<COMPILE_LANG_AND_ID:C,GNU,Clang,AppleClang>>:-fstack-protector-all>
    )
endif()

if(VORPALINE_WITH_TSAN)
    message(FATAL_ERROR "ThreadSanitizer is not available: cannot figure out how to link with library libtsan")
endif()

#-------------------------------------------------------------------
# geogram::profiling
#   was: VORPALINE_WITH_GPROF / VORPALINE_WITH_GCOV.
#-------------------------------------------------------------------
add_library(geogram_profiling INTERFACE)
add_library(geogram::profiling ALIAS geogram_profiling)

if(VORPALINE_WITH_GPROF)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "Profiling is not (yet) available with clang")
    endif()
    target_compile_options(geogram_profiling INTERFACE -pg)
    target_compile_definitions(geogram_profiling INTERFACE PROFILER)
    target_link_options(geogram_profiling INTERFACE -pg)
endif()

if(VORPALINE_WITH_GCOV)
    target_compile_options(geogram_profiling INTERFACE --coverage)
    target_link_options(geogram_profiling INTERFACE --coverage)
endif()

#-------------------------------------------------------------------
# geogram::windows
#   was: Windows.cmake / Windows-vs.cmake defines, release-mode
#   /D_SECURE_SCL=0 /GS- /Ox, and /OPT:REF,ICF linker flags.
#-------------------------------------------------------------------
add_library(geogram_windows INTERFACE)
add_library(geogram::windows ALIAS geogram_windows)

target_compile_options(geogram_windows INTERFACE
    $<$<CXX_COMPILER_ID:MSVC>:/MP;/bigobj>
    $<$<C_COMPILER_ID:MSVC>:/MP;/bigobj>
    $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:Release,RelWithDebInfo,MinSizeRel>>:/D_SECURE_SCL=0;/GS-;/Ox>
    $<$<AND:$<C_COMPILER_ID:MSVC>,$<CONFIG:Release,RelWithDebInfo,MinSizeRel>>:/D_SECURE_SCL=0;/GS-;/Ox>
)
target_compile_definitions(geogram_windows INTERFACE
    $<$<PLATFORM_ID:Windows>:WIN32_LEAN_AND_MEAN;VC_EXTRALEAN;NOMINMAX;_CRT_SECURE_NO_WARNINGS;_USE_MATH_DEFINES>
)
target_link_options(geogram_windows INTERFACE
    $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:Release,RelWithDebInfo,MinSizeRel>>:/OPT:REF,ICF>
    $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:Debug>>:/OPT:NOREF,NOICF>
)

# TODO 
# Set per target static vs shared selection instead of global flags
#   set_property(TARGET <tgt> PROPERTY
#       MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>$<$<BOOL:${BUILD_SHARED_LIBS}>:DLL>")

#-------------------------------------------------------------------
# geogram::platform -- convenience bundle for geogram's own targets.
# Third-party targets should link geogram::warnings_relaxed instead
# of geogram::warnings, and compose the remaining pieces directly.
#-------------------------------------------------------------------
add_library(geogram_platform INTERFACE)
add_library(geogram::platform ALIAS geogram_platform)
target_link_libraries(geogram_platform INTERFACE
    geogram::warnings
    geogram::numeric
    geogram::visibility
    geogram::linux_linker
    geogram::openmp
    geogram::sanitizers
    geogram::profiling
    geogram::windows
)

#-------------------------------------------------------------------
# geogram_add_executable() -- replacement for vor_add_executable().
# Static-link flags differ by compiler/OS in the real sources:
#   - GCC/Linux:   -static-libstdc++ -static-libgcc -static (CXX)
#                  -static-libgcc -static (C)
#   - Clang/Linux: -static (CXX and C)
#   - Darwin (gcc or clang): no static-link flags at all (the
#     vor_add_executable() body is empty on Darwin -- macOS does not
#     support static linking well)
# `m pthread` is only linked on Linux in the real sources: Linux-*.cmake
# do it, Darwin-clang.cmake's vor_add_executable() does not
#-------------------------------------------------------------------
function(geogram_add_executable)
    add_executable(${ARGN})
    list(GET ARGN 0 _tgt)
    if(NOT BUILD_SHARED_LIBS AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
        target_link_options(${_tgt} PRIVATE
            $<$<CXX_COMPILER_ID:GNU>:-static-libstdc++;-static-libgcc;-static>
            $<$<CXX_COMPILER_ID:Clang>:-static>
        )
        target_link_libraries(${_tgt} PRIVATE m pthread)
    endif()
endfunction()
