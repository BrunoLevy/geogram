#include <geogram_gfx/lua/lua_imgui.h>
#include <imgui.h>
#include <iostream>

int main() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    lua_State* state = luaL_newstate();
    if(state == nullptr) { ImGui::DestroyContext(); return 1; }
    luaL_openlibs(state);
    init_lua_imgui(state);
    const int status = luaL_dostring(state,
        "assert(ImGuiPopupFlags_MouseButtonDefault_ == ImGuiPopupFlags_MouseButtonRight)\n"
        "assert(ImDrawFlags_RoundCornersDefault_ == ImDrawFlags_RoundCornersAll)\n"
        "assert(ImDrawFlags_RoundCornersMask_ == (ImDrawFlags_RoundCornersAll | ImDrawFlags_RoundCornersNone))\n"
        "imgui.PushStyleVar_2(ImGuiStyleVar_WindowPadding, 8, 12)\n"
        "imgui.PopStyleVar(1)\n");
    if(status != LUA_OK) { std::cerr << lua_tostring(state, -1) << '\n'; }
    lua_close(state);
    ImGui::DestroyContext();
    return status == LUA_OK ? 0 : 2;
}
