if(TARGET TBB::tbb)
    return()
endif()

message(STATUS "Third-party: creating target 'TBB::tbb' (OneTBB)")

# Emscripten sets CMAKE_SYSTEM_PROCESSOR to "x86". Change it to "WASM" to prevent TBB from
# adding machine-specific "-mrtm" and "-mwaitpkg" compile options.
if(EMSCRIPTEN)
    set(CMAKE_SYSTEM_PROCESSOR "WASM")
endif()

option(TBB_TEST "Enable testing" OFF)
option(TBB_EXAMPLES "Enable examples" OFF)
option(TBB_STRICT "Treat compiler warnings as errors" OFF)
option(TBB_PREFER_STATIC "Use the static version of TBB for the alias target" ON)
option(TBB_ENABLE_WASM_THREADS "Use wasm threads" ON)
unset(TBB_DIR CACHE)

function(onetbb_import_target)
    macro(push_variable var value)
        if(DEFINED CACHE{${var}})
            set(ONETBB_OLD_${var}_VALUE "${${var}}")
            set(ONETBB_OLD_${var}_TYPE CACHE_TYPE)
        elseif(DEFINED ${var})
            set(ONETBB_OLD_${var}_VALUE "${${var}}")
            set(ONETBB_OLD_${var}_TYPE NORMAL_TYPE)
        else()
            set(ONETBB_OLD_${var}_TYPE NONE_TYPE)
        endif()
        set(${var} "${value}")
    endmacro()

    macro(pop_variable var)
        if(ONETBB_OLD_${var}_TYPE STREQUAL CACHE_TYPE)
            set(${var} "${ONETBB_OLD_${var}_VALUE}" CACHE PATH "" FORCE)
        elseif(ONETBB_OLD_${var}_TYPE STREQUAL NORMAL_TYPE)
            unset(${var} CACHE)
            set(${var} "${ONETBB_OLD_${var}_VALUE}")
        elseif(ONETBB_OLD_${var}_TYPE STREQUAL NONE_TYPE)
            unset(${var} CACHE)
        else()
            message(FATAL_ERROR "Trying to pop a variable that has not been pushed: ${var}")
        endif()
    endmacro()

    if(TBB_PREFER_STATIC)
        push_variable(BUILD_SHARED_LIBS OFF)
    else()
        push_variable(BUILD_SHARED_LIBS ON)
    endif()

    set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME tbb)

    include(FetchContent)
    FetchContent_Declare(
        tbb
        GIT_REPOSITORY https://github.com/oneapi-src/oneTBB.git
        GIT_TAG v2021.11.0
    )
    FetchContent_MakeAvailable(tbb)

    # TODO: This might break with future versions of onetbb. Onetbb should eventually add a proper cmake option to turn wasm threads on/off.
    if (EMSCRIPTEN AND NOT TBB_ENABLE_WASM_THREADS)
        set_property(TARGET Threads::Threads PROPERTY INTERFACE_LINK_LIBRARIES "")
    endif()

    pop_variable(BUILD_SHARED_LIBS)
endfunction()

onetbb_import_target()

if(NOT TARGET TBB::tbb)
    message(FATAL_ERROR "TBB::tbb is still not defined!")
endif()

foreach(name IN ITEMS tbb tbbmalloc tbbmalloc_proxy)
    if(TARGET ${name})
        # Folder name for IDE
        set_target_properties(${name} PROPERTIES FOLDER "third_party//tbb")

        # Force debug postfix for library name. Our pre-compiled MKL library expects "tbb12.dll" (without postfix).
        set_target_properties(${name} PROPERTIES DEBUG_POSTFIX "")

        # Without this macro, TBB will explicitly link against "tbb12_debug.lib" in Debug configs.
        # This is undesirable, since our pre-compiled version of MKL is linked against "tbb12.dll".
        target_compile_definitions(${name} PUBLIC -D__TBB_NO_IMPLICIT_LINKAGE=1)

        # Disable some features and avoid processor-specific code paths when compiling with
        # Emscripten for WebAssembly.
        if(EMSCRIPTEN)
            target_compile_definitions(${name} PRIVATE
                ITT_ARCH=-1
                __TBB_RESUMABLE_TASKS_USE_THREADS=1
                __TBB_DYNAMIC_LOAD_ENABLED=0
                __TBB_WEAK_SYMBOLS_PRESENT=0
            )
        endif()
    endif()
endforeach()
