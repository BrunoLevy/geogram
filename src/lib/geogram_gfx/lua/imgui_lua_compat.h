// Compatibility names used by the generated Lua bindings. Preserve their Lua
// API while compiling against the external ImGui headers used by the editor.
#ifndef GEOGRAM_GFX_LUA_IMGUI_COMPAT
#define GEOGRAM_GFX_LUA_IMGUI_COMPAT

#include <imgui.h>

#if IMGUI_VERSION_NUM >= 19298
// These spellings are local to generated Lua binding translation units.
// Map references after including imgui.h rather than declaring globals: some
// 1.93 snapshots retain enum members with the same legacy names.
#define ImGuiPopupFlags_MouseButtonDefault_ ImGuiPopupFlags_MouseButtonRight
#define ImDrawFlags_RoundCornersDefault_ ImDrawFlags_RoundAll
#define ImDrawFlags_RoundCornersMask_ (ImDrawFlags_RoundAll | ImDrawFlags_RoundNone)
#endif

#endif
