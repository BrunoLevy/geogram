include(${PROJECT_SOURCE_DIR}/cmake/opengl.cmake)

configure_file(basic/config.h.in basic/config.h)
include_directories(${PROJECT_BINARY_DIR}/src/lib)
foreach(component devkit devkit-full)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/basic/config.h"
    DESTINATION include/${VORPALINE_INCLUDE_SUBPATH}/geogram_gfx/basic
    COMPONENT ${component})
endforeach()

configure_file(geogram_gfx.pc.in ${PROJECT_BINARY_DIR}/geogram_gfx${VORPALINE_VERSION_MAJOR}.pc @ONLY)

if(GEOGRAM_USE_SYSTEM_TEXT_EDITOR)
  find_package(ImGuiColorTextEdit CONFIG REQUIRED)
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_LIBRARIES ImGuiColorTextEdit::ImGuiColorTextEdit)
  check_cxx_source_compiles("#include <imgui.h>
    #ifndef IMGUI_HAS_DOCK
    #error Geogram requires ImGui docking
    #endif
    int main() { return 0; }" GEOGRAM_SYSTEM_IMGUI_HAS_DOCKING)
  unset(CMAKE_REQUIRED_LIBRARIES)
  if(NOT GEOGRAM_SYSTEM_IMGUI_HAS_DOCKING)
    message(FATAL_ERROR "Geogram requires an ImGui docking build compatible with its backends")
  endif()
  add_definitions(-DGEOGRAM_USE_SYSTEM_TEXT_EDITOR)
else()
  add_definitions(-DIMGUI_USER_CONFIG="geogram_imconfig.h")
endif()
add_subdirectory(third_party)

aux_source_directories(SOURCES "Source Files"                .)
aux_source_directories(SOURCES "Source Files\\imgui_ext"     imgui_ext)
aux_source_directories(SOURCES "Source Files\\api"           api)
aux_source_directories(SOURCES "Source Files\\basic"         basic)
aux_source_directories(SOURCES "Source Files\\mesh"          mesh)
aux_source_directories(SOURCES "Source Files\\GLUP"          GLUP)
aux_source_directories(SOURCES "Source Files\\GLUP\\shaders" GLUP/shaders)
aux_source_directories(SOURCES "Source Files\\full_screen_effects" full_screen_effects)
aux_source_directories(SOURCES "Source Files\\gui"           gui)

if(GEOGRAM_WITH_LUA)
aux_source_directories(SOURCES "Source Files\\lua"           lua)
endif()

if(GEOGRAM_USE_SYSTEM_GLFW3)
add_definitions(-DGEO_USE_SYSTEM_GLFW3)
endif()

add_library(geogram_gfx ${SOURCES} $<TARGET_OBJECTS:geogram_gfx_third_party>)

if(GEOGRAM_USE_SYSTEM_TEXT_EDITOR)
  target_link_libraries(geogram_gfx ImGuiColorTextEdit::ImGuiColorTextEdit)
  target_compile_features(geogram_gfx PUBLIC cxx_std_17)
  target_compile_definitions(geogram_gfx PUBLIC GEOGRAM_USE_SYSTEM_TEXT_EDITOR)
endif()

target_include_directories(geogram_gfx PUBLIC ${PROJECT_SOURCE_DIR}/src/lib/geogram_gfx/third_party)
if(NOT GEOGRAM_USE_SYSTEM_TEXT_EDITOR)
  target_include_directories(geogram_gfx PRIVATE ${PROJECT_SOURCE_DIR}/src/lib/geogram_gfx/third_party/imgui)
endif()

if(ANDROID)
target_include_directories(geogram_gfx PRIVATE
${ANDROID_NDK}/sources/android/native_app_glue
)
message(STATUS "building for Android")
endif()

set_target_properties(geogram_gfx PROPERTIES
VERSION ${VORPALINE_VERSION}
SOVERSION ${VORPALINE_VERSION_MAJOR}
FOLDER "GEOGRAM")

if(NOT ANDROID)
target_link_libraries(geogram_gfx glfw)
endif()

if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
   find_package(X11 REQUIRED)
   target_link_libraries(geogram_gfx ${X11_LIBRARIES})
endif()

if(VORPALINE_BUILD_DYNAMIC)
# Must provide all symbols at link time
target_link_libraries(geogram_gfx geogram)
target_link_libraries(geogram_gfx ${OPENGL_LIBRARIES})
endif()

# Install the library
install_devkit_targets(geogram_gfx)

# Install include files for the standard devkit
install(
DIRECTORY .
DESTINATION include/${VORPALINE_INCLUDE_SUBPATH}/geogram_gfx
COMPONENT devkit
FILES_MATCHING PATTERN *.h
)

# Install include files for the full devkit
install(
DIRECTORY .
DESTINATION include/${VORPALINE_INCLUDE_SUBPATH}/geogram_gfx
COMPONENT devkit-full
FILES_MATCHING PATTERN *.h
)

if(GEOGRAM_USE_SYSTEM_TEXT_EDITOR AND GEOGRAM_WITH_LUA)
  foreach(component devkit devkit-full)
    install(FILES "${PROJECT_BINARY_DIR}/src/lib/geogram_gfx/lua/luawrap_runtime.h"
      DESTINATION include/${VORPALINE_INCLUDE_SUBPATH}/geogram_gfx/lua
      COMPONENT ${component})
  endforeach()
endif()

install(
FILES "${PROJECT_BINARY_DIR}/geogram_gfx${VORPALINE_VERSION_MAJOR}.pc"
DESTINATION lib${LIB_SUFFIX}/pkgconfig
)
