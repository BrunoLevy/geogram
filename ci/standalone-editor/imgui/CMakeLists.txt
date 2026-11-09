cmake_minimum_required(VERSION 3.14)
project(ImGuiIntegrationDependency LANGUAGES CXX)
include(GNUInstallDirs)
if(NOT EXISTS "${IMGUI_SOURCE_DIR}/imgui.cpp")
  message(FATAL_ERROR "Set IMGUI_SOURCE_DIR to the unmodified maintainer checkout")
endif()
# Test adapter only: do not alter the maintainer's CMake, packaging or sources.
add_library(imgui SHARED
  "${IMGUI_SOURCE_DIR}/imgui.cpp" "${IMGUI_SOURCE_DIR}/imgui_draw.cpp"
  "${IMGUI_SOURCE_DIR}/imgui_tables.cpp" "${IMGUI_SOURCE_DIR}/imgui_widgets.cpp"
  "${IMGUI_SOURCE_DIR}/imgui_demo.cpp")
target_compile_features(imgui PUBLIC cxx_std_11)
target_include_directories(imgui PUBLIC
  $<BUILD_INTERFACE:${IMGUI_SOURCE_DIR}> $<INSTALL_INTERFACE:include/imgui>)
install(TARGETS imgui EXPORT imgui-targets LIBRARY DESTINATION lib)
install(FILES
  "${IMGUI_SOURCE_DIR}/imgui.h" "${IMGUI_SOURCE_DIR}/imgui_internal.h"
  "${IMGUI_SOURCE_DIR}/imconfig.h" "${IMGUI_SOURCE_DIR}/imstb_rectpack.h"
  "${IMGUI_SOURCE_DIR}/imstb_textedit.h" "${IMGUI_SOURCE_DIR}/imstb_truetype.h"
  DESTINATION include/imgui)
install(EXPORT imgui-targets NAMESPACE imgui:: DESTINATION lib/cmake/imgui)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/imguiConfig.cmake"
  "include(\"\${CMAKE_CURRENT_LIST_DIR}/imgui-targets.cmake\")\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/imguiConfig.cmake" DESTINATION lib/cmake/imgui)
