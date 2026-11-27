cmake_minimum_required(VERSION 3.14)
project(GeogramStandaloneEditorCheck LANGUAGES CXX)

# Point CMAKE_MODULE_PATH at the installed Geogram lib/cmake/modules directory,
# and GEOGRAM_INSTALL_PREFIX at the matching installation.
find_package(Geogram REQUIRED)
enable_testing()
add_executable(standalone-editor main.cpp)
target_link_libraries(standalone-editor PRIVATE Geogram::geogram_gfx Geogram::geogram)
add_test(NAME standalone-editor COMMAND standalone-editor)

option(GEOGRAM_CHECK_LUA "Check the installed Geogram builtin Lua bindings" OFF)
if(GEOGRAM_CHECK_LUA)
  add_executable(standalone-editor-lua lua.cpp)
  target_compile_definitions(standalone-editor-lua PRIVATE GEOGRAM_USE_BUILTIN_DEPS)
  target_link_libraries(standalone-editor-lua PRIVATE Geogram::geogram_gfx Geogram::geogram)
  add_test(NAME standalone-editor-lua COMMAND standalone-editor-lua)
endif()
