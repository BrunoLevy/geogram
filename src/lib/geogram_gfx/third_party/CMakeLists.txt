vor_reset_warning_level()

if(
    ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR
    ${CMAKE_CXX_COMPILER_ID} STREQUAL "AppleClang"
)
  add_definitions(-Wno-unknown-warning-option)
  add_definitions(-Wno-zero-as-null-pointer-constant)
  add_definitions(-Wno-unused-parameter)
endif()

if(ANDROID)
  add_definitions(-Wno-unused-variable)
endif()

# Since we are in a CMake "object library" (i.e.,
# a set of object files compiled with same options),
# the automatically generated geogram_gfx_EXPORTS
# pre-processor definition of the parent library
# is not automatically inherited (this is why
# I set it manually)
if(BUILD_SHARED_LIBS)
   add_definitions(-Dgeogram_gfx_EXPORTS)
endif()


aux_source_directories(SOURCES "Source Files"            .)
aux_source_directories(SOURCES "Source Files\\glad"      glad)
if(NOT GEOGRAM_USE_SYSTEM_TEXT_EDITOR)
  aux_source_directories(SOURCES "Source Files\\ImGuiColorTextEdit" ImGuiColorTextEdit_geogram )
endif()

if(GEOGRAM_WITH_LUA)
 if(GEOGRAM_USE_SYSTEM_TEXT_EDITOR)
   # The binding runtime couples builtin Lua and builtin ImGui under one
   # switch. Keep its Lua choice, but adapt ImGui in a private build-tree copy.
   # Build both generated translation units against that same runtime header.
   set(_lua_bindings_source ${CMAKE_CURRENT_SOURCE_DIR}/imgui_lua_bindings_gomgen)
   set(_lua_bindings_adapter ${PROJECT_BINARY_DIR}/src/lib/geogram_gfx/lua)
   file(MAKE_DIRECTORY ${_lua_bindings_adapter})
   configure_file(${_lua_bindings_source}/luawrap_runtime.h
     ${_lua_bindings_adapter}/luawrap_runtime.h COPYONLY)
   file(READ ${_lua_bindings_adapter}/luawrap_runtime.h _lua_runtime)
   if(NOT _lua_runtime MATCHES "#include <geogram_gfx/third_party/imgui/imgui.h>")
     message(FATAL_ERROR "The ImGui Lua runtime changed; review the system-ImGui adapter")
   endif()
   string(REPLACE "#include <geogram_gfx/third_party/imgui/imgui.h>"
     "#include <geogram_gfx/lua/imgui_lua_compat.h>" _lua_runtime "${_lua_runtime}")
   file(WRITE ${_lua_bindings_adapter}/luawrap_runtime.h "${_lua_runtime}")
   foreach(_binding imgui_lua_bindings.cpp imdrawlist_lua_bindings.cpp)
     configure_file(${_lua_bindings_source}/${_binding}
       ${_lua_bindings_adapter}/${_binding} COPYONLY)
     list(APPEND SOURCES ${_lua_bindings_adapter}/${_binding})
   endforeach()
 else()
   aux_source_directories(SOURCES "Source Files\\imgui_lua_bindings_gomgen" imgui_lua_bindings_gomgen)
 endif()
endif()

add_library(geogram_gfx_third_party OBJECT ${SOURCES})

if(NOT GEOGRAM_USE_SYSTEM_TEXT_EDITOR)
target_sources(
  geogram_gfx_third_party PRIVATE
    imgui/imgui.cpp
    imgui/imgui_demo.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
)
endif()

# The std::string adapter and platform backends are not part of ImGui's core.
# Keep building them, but link the external core once in system-editor mode.
target_sources(geogram_gfx_third_party PRIVATE imgui/misc/cpp/imgui_stdlib.cpp)
if(GEOGRAM_USE_SYSTEM_TEXT_EDITOR)
  target_link_libraries(geogram_gfx_third_party PRIVATE ImGuiColorTextEdit::ImGuiColorTextEdit)
  target_compile_features(geogram_gfx_third_party PRIVATE cxx_std_17)
endif()

target_sources(
  geogram_gfx_third_party PRIVATE
    imgui/backends/imgui_impl_opengl3.cpp
)

if(NOT GEOGRAM_USE_SYSTEM_TEXT_EDITOR)
target_include_directories(
  geogram_gfx_third_party PRIVATE
  ${PROJECT_SOURCE_DIR}/src/lib/geogram_gfx/third_party/imgui
)
endif()

if(ANDROID)
#  target_sources(
#    geogram_gfx_third_party PRIVATE
#       imgui/backends/imgui_impl_android.cpp
#  )
  target_include_directories(geogram_gfx_third_party PRIVATE
    ${ANDROID_NDK}/sources/android/native_app_glue
  )
else()
  target_sources(
    geogram_gfx_third_party PRIVATE
       imgui/backends/imgui_impl_glfw.cpp
  )
  target_include_directories(
    geogram_gfx_third_party PRIVATE
    ${PROJECT_SOURCE_DIR}/src/lib/third_party/glfw/include
  )
endif()



set_target_properties(
    geogram_gfx_third_party PROPERTIES
    FOLDER "GEOGRAM"
)
