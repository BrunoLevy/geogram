# Standalone text editor

The optional `GEOGRAM_USE_SYSTEM_TEXT_EDITOR` mode uses the independently
packaged ImGuiColorTextEdit library and its ImGui dependency. It defaults to OFF,
preserving the existing submodule build.

Build and install ImGuiColorTextEdit with `ENABLE_GEOGRAM=ON`, then configure
Geogram with `-DGEOGRAM_USE_SYSTEM_TEXT_EDITOR=ON` and the corresponding
`CMAKE_PREFIX_PATH`. A library built without the compatibility interface is
rejected at configure time.

In this mode the bundled editor and ImGui core are excluded, avoiding duplicate
symbols and independent ImGui contexts. ImGui's core headers and configuration
come from the external package. The bundled platform backends and std::string
adapter remain application code. Use an ImGui package compatible with these
backends and Geogram's docking API (tested with 1.93.0-docking). The standalone
editor itself also supports ordinary, non-docking ImGui.

The Lua language definition uses the new language registry. Application actions
still reach Geogram through the callback; no Geogram header or symbol is needed
by the standalone editor. Fonts and icons can be supplied by the application
through `SetGeogramFindFont` and `SetGeogramCloseButtonLabel`.

The installed `geogram_gfx/basic/config.h` records the editor mode. The installed
`FindGeogram.cmake` propagates the standalone dependency and its compile options
to CMake consumers; consumers must not recreate the editor's layout themselves.

An installed-consumer check is provided in `tests/standalone-text-editor`.
Configure it with `GEOGRAM_INSTALL_PREFIX`, the installed
`lib/cmake/modules` directory in `CMAKE_MODULE_PATH`, and the editor/ImGui
installations in `CMAKE_PREFIX_PATH`. Build and run CTest. Install the
`devkit-full` component when the check needs the C++ headers (the smaller
`devkit` contains only Geogram's core C API headers).
