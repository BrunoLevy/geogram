cmake_minimum_required(VERSION 3.14)
project(GeogramStandaloneEditorCheck LANGUAGES CXX)

# Point CMAKE_MODULE_PATH at the installed Geogram lib/cmake/modules directory,
# and GEOGRAM_INSTALL_PREFIX at the matching installation.
find_package(Geogram REQUIRED)
enable_testing()
add_executable(standalone-editor main.cpp)
target_link_libraries(standalone-editor PRIVATE Geogram::geogram_gfx Geogram::geogram)
add_test(NAME standalone-editor COMMAND standalone-editor)
