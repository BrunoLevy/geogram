// Compatibility names used by the generated Lua bindings. Preserve their Lua
// API while compiling against the external ImGui headers used by the editor.
#ifndef GEOGRAM_GFX_LUA_IMGUI_COMPAT
#define GEOGRAM_GFX_LUA_IMGUI_COMPAT

#include <imgui.h>

#if IMGUI_VERSION_NUM >= 19298
static constexpr ImGuiPopupFlags ImGuiPopupFlags_MouseButtonDefault_ = ImGuiPopupFlags_MouseButtonRight;
static constexpr ImDrawFlags ImDrawFlags_RoundCornersDefault_ = ImDrawFlags_RoundAll;
static constexpr ImDrawFlags ImDrawFlags_RoundCornersMask_ = ImDrawFlags_RoundAll | ImDrawFlags_RoundNone;
#endif

#endif
