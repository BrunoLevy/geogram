#include "luawrap_runtime.h"

// GOMGEN automatically generated code
// Do not edit.

// Command line:
//  gomgen
//  -oluawrap_imgui.cpp
//  -iimgui.h
//  -sImGui
//  -DIMGUI_DISABLE_OBSOLETE_FUNCTIONS
//  -lua


// Include path:
// Input path:
//   /home/blevy/Programming/imgui_gomgen/imgui.h
// Output file:
//   /home/blevy/Programming/imgui_gomgen/luawrap_imgui.cpp


namespace ImGui_lua_wrappers {
   using namespace LuaWrap;

   static int NewFrame(lua_State*) {
      ImGui::NewFrame();
      return 0;
   }

   static int EndFrame(lua_State*) {
      ImGui::EndFrame();
      return 0;
   }

   static int Render(lua_State*) {
      ImGui::Render();
      return 0;
   }

   static int GetDrawData(lua_State* L) {
      Arg<ImDrawData*> retval = ImGui::GetDrawData();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int ShowDemoWindow(lua_State* L) {
      static const char* proto = "void ImGui::ShowDemoWindow(bool* p_open=NULL)";
      Arg<bool> p_open(L,1,NullPointer());
      LUAWRAP_CHECK_ARGS(p_open);
      ImGui::ShowDemoWindow(p_open.pointer());
      int prevtop = lua_gettop(L);
      p_open.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int ShowMetricsWindow(lua_State* L) {
      static const char* proto = "void ImGui::ShowMetricsWindow(bool* p_open=NULL)";
      Arg<bool> p_open(L,1,NullPointer());
      LUAWRAP_CHECK_ARGS(p_open);
      ImGui::ShowMetricsWindow(p_open.pointer());
      int prevtop = lua_gettop(L);
      p_open.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int ShowDebugLogWindow(lua_State* L) {
      static const char* proto = "void ImGui::ShowDebugLogWindow(bool* p_open=NULL)";
      Arg<bool> p_open(L,1,NullPointer());
      LUAWRAP_CHECK_ARGS(p_open);
      ImGui::ShowDebugLogWindow(p_open.pointer());
      int prevtop = lua_gettop(L);
      p_open.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int ShowIDStackToolWindow(lua_State* L) {
      static const char* proto = "void ImGui::ShowIDStackToolWindow(bool* p_open=NULL)";
      Arg<bool> p_open(L,1,NullPointer());
      LUAWRAP_CHECK_ARGS(p_open);
      ImGui::ShowIDStackToolWindow(p_open.pointer());
      int prevtop = lua_gettop(L);
      p_open.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int ShowAboutWindow(lua_State* L) {
      static const char* proto = "void ImGui::ShowAboutWindow(bool* p_open=NULL)";
      Arg<bool> p_open(L,1,NullPointer());
      LUAWRAP_CHECK_ARGS(p_open);
      ImGui::ShowAboutWindow(p_open.pointer());
      int prevtop = lua_gettop(L);
      p_open.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int ShowStyleEditor(lua_State* L) {
      static const char* proto = "void ImGui::ShowStyleEditor(ImGuiStyle* ref=NULL)";
      Arg<ImGuiStyle*> ref(L,1,NULL);
      LUAWRAP_CHECK_ARGS(ref);
      ImGui::ShowStyleEditor(ref.value);
      return 0;
   }

   static int ShowStyleSelector(lua_State* L) {
      static const char* proto = "bool ImGui::ShowStyleSelector(const char* label)";
      Arg<const char*> label(L,1);
      LUAWRAP_CHECK_ARGS(label);
      Arg<bool> retval = ImGui::ShowStyleSelector(label.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int ShowFontSelector(lua_State* L) {
      static const char* proto = "void ImGui::ShowFontSelector(const char* label)";
      Arg<const char*> label(L,1);
      LUAWRAP_CHECK_ARGS(label);
      ImGui::ShowFontSelector(label.value);
      return 0;
   }

   static int ShowUserGuide(lua_State*) {
      ImGui::ShowUserGuide();
      return 0;
   }

   static int GetVersion(lua_State* L) {
      Arg<const char*> retval = ImGui::GetVersion();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int StyleColorsDark(lua_State* L) {
      static const char* proto = "void ImGui::StyleColorsDark(ImGuiStyle* dst=NULL)";
      Arg<ImGuiStyle*> dst(L,1,NULL);
      LUAWRAP_CHECK_ARGS(dst);
      ImGui::StyleColorsDark(dst.value);
      return 0;
   }

   static int StyleColorsLight(lua_State* L) {
      static const char* proto = "void ImGui::StyleColorsLight(ImGuiStyle* dst=NULL)";
      Arg<ImGuiStyle*> dst(L,1,NULL);
      LUAWRAP_CHECK_ARGS(dst);
      ImGui::StyleColorsLight(dst.value);
      return 0;
   }

   static int StyleColorsClassic(lua_State* L) {
      static const char* proto = "void ImGui::StyleColorsClassic(ImGuiStyle* dst=NULL)";
      Arg<ImGuiStyle*> dst(L,1,NULL);
      LUAWRAP_CHECK_ARGS(dst);
      ImGui::StyleColorsClassic(dst.value);
      return 0;
   }

   static int Begin(lua_State* L) {
      static const char* proto = "bool ImGui::Begin(const char* name, bool* p_open=NULL, ImGuiWindowFlags flags=0)";
      Arg<const char*> name(L,1);
      Arg<bool> p_open(L,2,NullPointer());
      Arg<int,lua_Integer> flags(L,3,0);
      LUAWRAP_CHECK_ARGS(name, p_open, flags);
      Arg<bool> retval = ImGui::Begin(name.value, p_open.pointer(), flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      p_open.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int End(lua_State*) {
      ImGui::End();
      return 0;
   }

   static int BeginChild(lua_State* L) {
      static const char* proto = "bool ImGui::BeginChild(const char* str_id, ImVec2 size=ImVec2(0, 0), ImGuiChildFlags child_flags=0, ImGuiWindowFlags window_flags=0)";
      Arg<const char*> str_id(L,1);
      Arg<ImVec2> size(L,2,ImVec2(0, 0));
      Arg<int,lua_Integer> child_flags(L,4,0);
      Arg<int,lua_Integer> window_flags(L,5,0);
      LUAWRAP_CHECK_ARGS(str_id, size, child_flags, window_flags);
      Arg<bool> retval = ImGui::BeginChild(str_id.value, size.value, child_flags.value, window_flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndChild(lua_State*) {
      ImGui::EndChild();
      return 0;
   }

   static int IsWindowAppearing(lua_State* L) {
      Arg<bool> retval = ImGui::IsWindowAppearing();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsWindowCollapsed(lua_State* L) {
      Arg<bool> retval = ImGui::IsWindowCollapsed();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsWindowFocused(lua_State* L) {
      static const char* proto = "bool ImGui::IsWindowFocused(ImGuiFocusedFlags flags=0)";
      Arg<int,lua_Integer> flags(L,1,0);
      LUAWRAP_CHECK_ARGS(flags);
      Arg<bool> retval = ImGui::IsWindowFocused(flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsWindowHovered(lua_State* L) {
      static const char* proto = "bool ImGui::IsWindowHovered(ImGuiHoveredFlags flags=0)";
      Arg<int,lua_Integer> flags(L,1,0);
      LUAWRAP_CHECK_ARGS(flags);
      Arg<bool> retval = ImGui::IsWindowHovered(flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetWindowDrawList(lua_State* L) {
      Arg<ImDrawList*> retval = ImGui::GetWindowDrawList();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetWindowDpiScale(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetWindowDpiScale();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetWindowPos(lua_State* L) {
      Arg<ImVec2> retval = ImGui::GetWindowPos();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetWindowSize(lua_State* L) {
      Arg<ImVec2> retval = ImGui::GetWindowSize();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetWindowWidth(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetWindowWidth();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetWindowHeight(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetWindowHeight();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetWindowViewport(lua_State* L) {
      Arg<ImGuiViewport*> retval = ImGui::GetWindowViewport();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetNextWindowPos(lua_State* L) {
      static const char* proto = "void ImGui::SetNextWindowPos(ImVec2 pos, ImGuiCond cond=0, ImVec2 pivot=ImVec2(0, 0))";
      Arg<ImVec2> pos(L,1);
      Arg<int,lua_Integer> cond(L,3,0);
      Arg<ImVec2> pivot(L,4,ImVec2(0, 0));
      LUAWRAP_CHECK_ARGS(pos, cond, pivot);
      ImGui::SetNextWindowPos(pos.value, cond.value, pivot.value);
      return 0;
   }

   static int SetNextWindowSize(lua_State* L) {
      static const char* proto = "void ImGui::SetNextWindowSize(ImVec2 size, ImGuiCond cond=0)";
      Arg<ImVec2> size(L,1);
      Arg<int,lua_Integer> cond(L,3,0);
      LUAWRAP_CHECK_ARGS(size, cond);
      ImGui::SetNextWindowSize(size.value, cond.value);
      return 0;
   }

   static int SetNextWindowContentSize(lua_State* L) {
      static const char* proto = "void ImGui::SetNextWindowContentSize(ImVec2 size)";
      Arg<ImVec2> size(L,1);
      LUAWRAP_CHECK_ARGS(size);
      ImGui::SetNextWindowContentSize(size.value);
      return 0;
   }

   static int SetNextWindowCollapsed(lua_State* L) {
      static const char* proto = "void ImGui::SetNextWindowCollapsed(bool collapsed, ImGuiCond cond=0)";
      Arg<bool> collapsed(L,1);
      Arg<int,lua_Integer> cond(L,2,0);
      LUAWRAP_CHECK_ARGS(collapsed, cond);
      ImGui::SetNextWindowCollapsed(collapsed.value, cond.value);
      return 0;
   }

   static int SetNextWindowFocus(lua_State*) {
      ImGui::SetNextWindowFocus();
      return 0;
   }

   static int SetNextWindowScroll(lua_State* L) {
      static const char* proto = "void ImGui::SetNextWindowScroll(ImVec2 scroll)";
      Arg<ImVec2> scroll(L,1);
      LUAWRAP_CHECK_ARGS(scroll);
      ImGui::SetNextWindowScroll(scroll.value);
      return 0;
   }

   static int SetNextWindowBgAlpha(lua_State* L) {
      static const char* proto = "void ImGui::SetNextWindowBgAlpha(float alpha)";
      Arg<float,lua_Number> alpha(L,1);
      LUAWRAP_CHECK_ARGS(alpha);
      ImGui::SetNextWindowBgAlpha(alpha.value);
      return 0;
   }

   static int SetNextWindowViewport(lua_State* L) {
      static const char* proto = "void ImGui::SetNextWindowViewport(ImGuiID viewport_id)";
      Arg<unsigned int,lua_Integer> viewport_id(L,1);
      LUAWRAP_CHECK_ARGS(viewport_id);
      ImGui::SetNextWindowViewport(viewport_id.value);
      return 0;
   }

   static int SetWindowPos(lua_State* L) {
      static const char* proto = "void ImGui::SetWindowPos(ImVec2 pos, ImGuiCond cond=0)";
      Arg<ImVec2> pos(L,1);
      Arg<int,lua_Integer> cond(L,3,0);
      LUAWRAP_CHECK_ARGS(pos, cond);
      ImGui::SetWindowPos(pos.value, cond.value);
      return 0;
   }

   static int SetWindowSize(lua_State* L) {
      static const char* proto = "void ImGui::SetWindowSize(ImVec2 size, ImGuiCond cond=0)";
      Arg<ImVec2> size(L,1);
      Arg<int,lua_Integer> cond(L,3,0);
      LUAWRAP_CHECK_ARGS(size, cond);
      ImGui::SetWindowSize(size.value, cond.value);
      return 0;
   }

   static int SetWindowCollapsed(lua_State* L) {
      static const char* proto = "void ImGui::SetWindowCollapsed(bool collapsed, ImGuiCond cond=0)";
      Arg<bool> collapsed(L,1);
      Arg<int,lua_Integer> cond(L,2,0);
      LUAWRAP_CHECK_ARGS(collapsed, cond);
      ImGui::SetWindowCollapsed(collapsed.value, cond.value);
      return 0;
   }

   static int SetWindowFocus(lua_State*) {
      ImGui::SetWindowFocus();
      return 0;
   }

   static int GetScrollX(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetScrollX();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetScrollY(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetScrollY();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetScrollX(lua_State* L) {
      static const char* proto = "void ImGui::SetScrollX(float scroll_x)";
      Arg<float,lua_Number> scroll_x(L,1);
      LUAWRAP_CHECK_ARGS(scroll_x);
      ImGui::SetScrollX(scroll_x.value);
      return 0;
   }

   static int SetScrollY(lua_State* L) {
      static const char* proto = "void ImGui::SetScrollY(float scroll_y)";
      Arg<float,lua_Number> scroll_y(L,1);
      LUAWRAP_CHECK_ARGS(scroll_y);
      ImGui::SetScrollY(scroll_y.value);
      return 0;
   }

   static int GetScrollMaxX(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetScrollMaxX();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetScrollMaxY(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetScrollMaxY();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetScrollHereX(lua_State* L) {
      static const char* proto = "void ImGui::SetScrollHereX(float center_x_ratio=0.5)";
      Arg<float,lua_Number> center_x_ratio(L,1,0.5);
      LUAWRAP_CHECK_ARGS(center_x_ratio);
      ImGui::SetScrollHereX(center_x_ratio.value);
      return 0;
   }

   static int SetScrollHereY(lua_State* L) {
      static const char* proto = "void ImGui::SetScrollHereY(float center_y_ratio=0.5)";
      Arg<float,lua_Number> center_y_ratio(L,1,0.5);
      LUAWRAP_CHECK_ARGS(center_y_ratio);
      ImGui::SetScrollHereY(center_y_ratio.value);
      return 0;
   }

   static int SetScrollFromPosX(lua_State* L) {
      static const char* proto = "void ImGui::SetScrollFromPosX(float local_x, float center_x_ratio=0.5)";
      Arg<float,lua_Number> local_x(L,1);
      Arg<float,lua_Number> center_x_ratio(L,2,0.5);
      LUAWRAP_CHECK_ARGS(local_x, center_x_ratio);
      ImGui::SetScrollFromPosX(local_x.value, center_x_ratio.value);
      return 0;
   }

   static int SetScrollFromPosY(lua_State* L) {
      static const char* proto = "void ImGui::SetScrollFromPosY(float local_y, float center_y_ratio=0.5)";
      Arg<float,lua_Number> local_y(L,1);
      Arg<float,lua_Number> center_y_ratio(L,2,0.5);
      LUAWRAP_CHECK_ARGS(local_y, center_y_ratio);
      ImGui::SetScrollFromPosY(local_y.value, center_y_ratio.value);
      return 0;
   }

   static int PushFont(lua_State* L) {
      static const char* proto = "void ImGui::PushFont(ImFont* font, float font_size_base_unscaled)";
      Arg<ImFont*> font(L,1);
      Arg<float,lua_Number> font_size_base_unscaled(L,2);
      LUAWRAP_CHECK_ARGS(font, font_size_base_unscaled);
      ImGui::PushFont(font.value, font_size_base_unscaled.value);
      return 0;
   }

   static int PopFont(lua_State*) {
      ImGui::PopFont();
      return 0;
   }

   static int GetFont(lua_State* L) {
      Arg<ImFont*> retval = ImGui::GetFont();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetFontSize(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetFontSize();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetFontBaked(lua_State* L) {
      Arg<ImFontBaked*> retval = ImGui::GetFontBaked();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int PushStyleColor(lua_State* L) {
      static const char* proto = "void ImGui::PushStyleColor(ImGuiCol idx, ImU32 col)";
      Arg<int,lua_Integer> idx(L,1);
      Arg<unsigned int,lua_Integer> col(L,2);
      LUAWRAP_CHECK_ARGS(idx, col);
      ImGui::PushStyleColor(idx.value, col.value);
      return 0;
   }

   static int PopStyleColor(lua_State* L) {
      static const char* proto = "void ImGui::PopStyleColor(int count=1)";
      Arg<int,lua_Integer> count(L,1,1);
      LUAWRAP_CHECK_ARGS(count);
      ImGui::PopStyleColor(count.value);
      return 0;
   }

   static int PushStyleVar(lua_State* L) {
      static const char* proto = "void ImGui::PushStyleVar(ImGuiStyleVar idx, float val)";
      Arg<int,lua_Integer> idx(L,1);
      Arg<float,lua_Number> val(L,2);
      LUAWRAP_CHECK_ARGS(idx, val);
      ImGui::PushStyleVar(idx.value, val.value);
      return 0;
   }

   static int PushStyleVarX(lua_State* L) {
      static const char* proto = "void ImGui::PushStyleVarX(ImGuiStyleVar idx, float val_x)";
      Arg<int,lua_Integer> idx(L,1);
      Arg<float,lua_Number> val_x(L,2);
      LUAWRAP_CHECK_ARGS(idx, val_x);
      ImGui::PushStyleVarX(idx.value, val_x.value);
      return 0;
   }

   static int PushStyleVarY(lua_State* L) {
      static const char* proto = "void ImGui::PushStyleVarY(ImGuiStyleVar idx, float val_y)";
      Arg<int,lua_Integer> idx(L,1);
      Arg<float,lua_Number> val_y(L,2);
      LUAWRAP_CHECK_ARGS(idx, val_y);
      ImGui::PushStyleVarY(idx.value, val_y.value);
      return 0;
   }

   static int PopStyleVar(lua_State* L) {
      static const char* proto = "void ImGui::PopStyleVar(int count=1)";
      Arg<int,lua_Integer> count(L,1,1);
      LUAWRAP_CHECK_ARGS(count);
      ImGui::PopStyleVar(count.value);
      return 0;
   }

   static int PushItemFlag(lua_State* L) {
      static const char* proto = "void ImGui::PushItemFlag(ImGuiItemFlags option, bool enabled)";
      Arg<int,lua_Integer> option(L,1);
      Arg<bool> enabled(L,2);
      LUAWRAP_CHECK_ARGS(option, enabled);
      ImGui::PushItemFlag(option.value, enabled.value);
      return 0;
   }

   static int PopItemFlag(lua_State*) {
      ImGui::PopItemFlag();
      return 0;
   }

   static int PushItemWidth(lua_State* L) {
      static const char* proto = "void ImGui::PushItemWidth(float item_width)";
      Arg<float,lua_Number> item_width(L,1);
      LUAWRAP_CHECK_ARGS(item_width);
      ImGui::PushItemWidth(item_width.value);
      return 0;
   }

   static int PopItemWidth(lua_State*) {
      ImGui::PopItemWidth();
      return 0;
   }

   static int SetNextItemWidth(lua_State* L) {
      static const char* proto = "void ImGui::SetNextItemWidth(float item_width)";
      Arg<float,lua_Number> item_width(L,1);
      LUAWRAP_CHECK_ARGS(item_width);
      ImGui::SetNextItemWidth(item_width.value);
      return 0;
   }

   static int CalcItemWidth(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::CalcItemWidth();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int PushTextWrapPos(lua_State* L) {
      static const char* proto = "void ImGui::PushTextWrapPos(float wrap_local_pos_x=0.0)";
      Arg<float,lua_Number> wrap_local_pos_x(L,1,0.0);
      LUAWRAP_CHECK_ARGS(wrap_local_pos_x);
      ImGui::PushTextWrapPos(wrap_local_pos_x.value);
      return 0;
   }

   static int PopTextWrapPos(lua_State*) {
      ImGui::PopTextWrapPos();
      return 0;
   }

   static int GetFontTexUvWhitePixel(lua_State* L) {
      Arg<ImVec2> retval = ImGui::GetFontTexUvWhitePixel();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetColorU32(lua_State* L) {
      static const char* proto = "ImU32 ImGui::GetColorU32(ImGuiCol idx, float alpha_mul=1.0)";
      Arg<int,lua_Integer> idx(L,1);
      Arg<float,lua_Number> alpha_mul(L,2,1.0);
      LUAWRAP_CHECK_ARGS(idx, alpha_mul);
      Arg<ImU32,lua_Integer> retval = ImGui::GetColorU32(idx.value, alpha_mul.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetStyleColorVec4(lua_State* L) {
      static const char* proto = "ImVec4 ImGui::GetStyleColorVec4(ImGuiCol idx)";
      Arg<int,lua_Integer> idx(L,1);
      LUAWRAP_CHECK_ARGS(idx);
      Arg<ImVec4> retval = ImGui::GetStyleColorVec4(idx.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetCursorScreenPos(lua_State* L) {
      Arg<ImVec2> retval = ImGui::GetCursorScreenPos();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetCursorScreenPos(lua_State* L) {
      static const char* proto = "void ImGui::SetCursorScreenPos(ImVec2 pos)";
      Arg<ImVec2> pos(L,1);
      LUAWRAP_CHECK_ARGS(pos);
      ImGui::SetCursorScreenPos(pos.value);
      return 0;
   }

   static int GetContentRegionAvail(lua_State* L) {
      Arg<ImVec2> retval = ImGui::GetContentRegionAvail();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetCursorPos(lua_State* L) {
      Arg<ImVec2> retval = ImGui::GetCursorPos();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetCursorPosX(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetCursorPosX();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetCursorPosY(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetCursorPosY();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetCursorPos(lua_State* L) {
      static const char* proto = "void ImGui::SetCursorPos(ImVec2 local_pos)";
      Arg<ImVec2> local_pos(L,1);
      LUAWRAP_CHECK_ARGS(local_pos);
      ImGui::SetCursorPos(local_pos.value);
      return 0;
   }

   static int SetCursorPosX(lua_State* L) {
      static const char* proto = "void ImGui::SetCursorPosX(float local_x)";
      Arg<float,lua_Number> local_x(L,1);
      LUAWRAP_CHECK_ARGS(local_x);
      ImGui::SetCursorPosX(local_x.value);
      return 0;
   }

   static int SetCursorPosY(lua_State* L) {
      static const char* proto = "void ImGui::SetCursorPosY(float local_y)";
      Arg<float,lua_Number> local_y(L,1);
      LUAWRAP_CHECK_ARGS(local_y);
      ImGui::SetCursorPosY(local_y.value);
      return 0;
   }

   static int GetCursorStartPos(lua_State* L) {
      Arg<ImVec2> retval = ImGui::GetCursorStartPos();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int Separator(lua_State*) {
      ImGui::Separator();
      return 0;
   }

   static int SameLine(lua_State* L) {
      static const char* proto = "void ImGui::SameLine(float offset_from_start_x=0.0, float spacing=-1.0)";
      Arg<float,lua_Number> offset_from_start_x(L,1,0.0);
      Arg<float,lua_Number> spacing(L,2,-1.0);
      LUAWRAP_CHECK_ARGS(offset_from_start_x, spacing);
      ImGui::SameLine(offset_from_start_x.value, spacing.value);
      return 0;
   }

   static int NewLine(lua_State*) {
      ImGui::NewLine();
      return 0;
   }

   static int Spacing(lua_State*) {
      ImGui::Spacing();
      return 0;
   }

   static int Dummy(lua_State* L) {
      static const char* proto = "void ImGui::Dummy(ImVec2 size)";
      Arg<ImVec2> size(L,1);
      LUAWRAP_CHECK_ARGS(size);
      ImGui::Dummy(size.value);
      return 0;
   }

   static int Indent(lua_State* L) {
      static const char* proto = "void ImGui::Indent(float indent_w=0.0)";
      Arg<float,lua_Number> indent_w(L,1,0.0);
      LUAWRAP_CHECK_ARGS(indent_w);
      ImGui::Indent(indent_w.value);
      return 0;
   }

   static int Unindent(lua_State* L) {
      static const char* proto = "void ImGui::Unindent(float indent_w=0.0)";
      Arg<float,lua_Number> indent_w(L,1,0.0);
      LUAWRAP_CHECK_ARGS(indent_w);
      ImGui::Unindent(indent_w.value);
      return 0;
   }

   static int BeginGroup(lua_State*) {
      ImGui::BeginGroup();
      return 0;
   }

   static int EndGroup(lua_State*) {
      ImGui::EndGroup();
      return 0;
   }

   static int AlignTextToFramePadding(lua_State*) {
      ImGui::AlignTextToFramePadding();
      return 0;
   }

   static int GetTextLineHeight(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetTextLineHeight();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetTextLineHeightWithSpacing(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetTextLineHeightWithSpacing();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetFrameHeight(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetFrameHeight();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetFrameHeightWithSpacing(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetFrameHeightWithSpacing();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int PushID(lua_State* L) {
      static const char* proto = "void ImGui::PushID(const char* str_id)";
      Arg<const char*> str_id(L,1);
      LUAWRAP_CHECK_ARGS(str_id);
      ImGui::PushID(str_id.value);
      return 0;
   }

   static int PopID(lua_State*) {
      ImGui::PopID();
      return 0;
   }

   static int GetID(lua_State* L) {
      static const char* proto = "ImGuiID ImGui::GetID(const char* str_id)";
      Arg<const char*> str_id(L,1);
      LUAWRAP_CHECK_ARGS(str_id);
      Arg<ImGuiID,lua_Integer> retval = ImGui::GetID(str_id.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TextUnformatted(lua_State* L) {
      static const char* proto = "void ImGui::TextUnformatted(const char* text, const char* text_end=NULL)";
      Arg<const char*> text(L,1);
      Arg<const char*> text_end(L,2,NULL);
      LUAWRAP_CHECK_ARGS(text, text_end);
      ImGui::TextUnformatted(text.value, text_end.value);
      return 0;
   }

   static int SeparatorText(lua_State* L) {
      static const char* proto = "void ImGui::SeparatorText(const char* label)";
      Arg<const char*> label(L,1);
      LUAWRAP_CHECK_ARGS(label);
      ImGui::SeparatorText(label.value);
      return 0;
   }

   static int Button(lua_State* L) {
      static const char* proto = "bool ImGui::Button(const char* label, ImVec2 size=ImVec2(0, 0))";
      Arg<const char*> label(L,1);
      Arg<ImVec2> size(L,2,ImVec2(0, 0));
      LUAWRAP_CHECK_ARGS(label, size);
      Arg<bool> retval = ImGui::Button(label.value, size.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SmallButton(lua_State* L) {
      static const char* proto = "bool ImGui::SmallButton(const char* label)";
      Arg<const char*> label(L,1);
      LUAWRAP_CHECK_ARGS(label);
      Arg<bool> retval = ImGui::SmallButton(label.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int InvisibleButton(lua_State* L) {
      static const char* proto = "bool ImGui::InvisibleButton(const char* str_id, ImVec2 size, ImGuiButtonFlags flags=0)";
      Arg<const char*> str_id(L,1);
      Arg<ImVec2> size(L,2);
      Arg<int,lua_Integer> flags(L,4,0);
      LUAWRAP_CHECK_ARGS(str_id, size, flags);
      Arg<bool> retval = ImGui::InvisibleButton(str_id.value, size.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int ArrowButton(lua_State* L) {
      static const char* proto = "bool ImGui::ArrowButton(const char* str_id, ImGuiDir dir)";
      Arg<const char*> str_id(L,1);
      Arg<ImGuiDir,lua_Integer> dir(L,2);
      LUAWRAP_CHECK_ARGS(str_id, dir);
      Arg<bool> retval = ImGui::ArrowButton(str_id.value, dir.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int Checkbox(lua_State* L) {
      static const char* proto = "bool ImGui::Checkbox(const char* label, bool* v)";
      Arg<const char*> label(L,1);
      Arg<bool> v(L,2,UninitializedPointer());
      LUAWRAP_CHECK_ARGS(label, v);
      Arg<bool> retval = ImGui::Checkbox(label.value, v.pointer());
      int prevtop = lua_gettop(L);
      retval.push(L);
      v.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int CheckboxFlags(lua_State* L) {
      static const char* proto = "bool ImGui::CheckboxFlags(const char* label, int* flags, int flags_value)";
      Arg<const char*> label(L,1);
      Arg<int,lua_Integer> flags(L,2,UninitializedPointer());
      Arg<int,lua_Integer> flags_value(L,3);
      LUAWRAP_CHECK_ARGS(label, flags, flags_value);
      Arg<bool> retval = ImGui::CheckboxFlags(label.value, flags.pointer(), flags_value.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      flags.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int RadioButton(lua_State* L) {
      static const char* proto = "bool ImGui::RadioButton(const char* label, bool active)";
      Arg<const char*> label(L,1);
      Arg<bool> active(L,2);
      LUAWRAP_CHECK_ARGS(label, active);
      Arg<bool> retval = ImGui::RadioButton(label.value, active.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int ProgressBar(lua_State* L) {
      static const char* proto = "void ImGui::ProgressBar(float fraction, ImVec2 size_arg=ImVec2(-FLT_MIN, 0), const char* overlay=NULL)";
      Arg<float,lua_Number> fraction(L,1);
      Arg<ImVec2> size_arg(L,2,ImVec2(-FLT_MIN, 0));
      Arg<const char*> overlay(L,4,NULL);
      LUAWRAP_CHECK_ARGS(fraction, size_arg, overlay);
      ImGui::ProgressBar(fraction.value, size_arg.value, overlay.value);
      return 0;
   }

   static int Bullet(lua_State*) {
      ImGui::Bullet();
      return 0;
   }

   static int TextLink(lua_State* L) {
      static const char* proto = "bool ImGui::TextLink(const char* label)";
      Arg<const char*> label(L,1);
      LUAWRAP_CHECK_ARGS(label);
      Arg<bool> retval = ImGui::TextLink(label.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TextLinkOpenURL(lua_State* L) {
      static const char* proto = "bool ImGui::TextLinkOpenURL(const char* label, const char* url=NULL)";
      Arg<const char*> label(L,1);
      Arg<const char*> url(L,2,NULL);
      LUAWRAP_CHECK_ARGS(label, url);
      Arg<bool> retval = ImGui::TextLinkOpenURL(label.value, url.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int Image(lua_State* L) {
      static const char* proto = "void ImGui::Image(ImTextureRef tex_ref, ImVec2 image_size, ImVec2 uv0=ImVec2(0, 0), ImVec2 uv1=ImVec2(1, 1))";
      Arg<ImTextureRef> tex_ref(L,1);
      Arg<ImVec2> image_size(L,2);
      Arg<ImVec2> uv0(L,4,ImVec2(0, 0));
      Arg<ImVec2> uv1(L,6,ImVec2(1, 1));
      LUAWRAP_CHECK_ARGS(tex_ref, image_size, uv0, uv1);
      ImGui::Image(tex_ref.value, image_size.value, uv0.value, uv1.value);
      return 0;
   }

   static int ImageWithBg(lua_State* L) {
      static const char* proto = "void ImGui::ImageWithBg(ImTextureRef tex_ref, ImVec2 image_size, ImVec2 uv0=ImVec2(0, 0), ImVec2 uv1=ImVec2(1, 1), ImVec4 bg_col=ImVec4(0, 0, 0, 0), ImVec4 tint_col=ImVec4(1, 1, 1, 1))";
      Arg<ImTextureRef> tex_ref(L,1);
      Arg<ImVec2> image_size(L,2);
      Arg<ImVec2> uv0(L,4,ImVec2(0, 0));
      Arg<ImVec2> uv1(L,6,ImVec2(1, 1));
      Arg<ImVec4> bg_col(L,8,ImVec4(0, 0, 0, 0));
      Arg<ImVec4> tint_col(L,12,ImVec4(1, 1, 1, 1));
      LUAWRAP_CHECK_ARGS(tex_ref, image_size, uv0, uv1, bg_col, tint_col);
      ImGui::ImageWithBg(tex_ref.value, image_size.value, uv0.value, uv1.value, bg_col.value, tint_col.value);
      return 0;
   }

   static int ImageButton(lua_State* L) {
      static const char* proto = "bool ImGui::ImageButton(const char* str_id, ImTextureRef tex_ref, ImVec2 image_size, ImVec2 uv0=ImVec2(0, 0), ImVec2 uv1=ImVec2(1, 1), ImVec4 bg_col=ImVec4(0, 0, 0, 0), ImVec4 tint_col=ImVec4(1, 1, 1, 1))";
      Arg<const char*> str_id(L,1);
      Arg<ImTextureRef> tex_ref(L,2);
      Arg<ImVec2> image_size(L,3);
      Arg<ImVec2> uv0(L,5,ImVec2(0, 0));
      Arg<ImVec2> uv1(L,7,ImVec2(1, 1));
      Arg<ImVec4> bg_col(L,9,ImVec4(0, 0, 0, 0));
      Arg<ImVec4> tint_col(L,13,ImVec4(1, 1, 1, 1));
      LUAWRAP_CHECK_ARGS(str_id, tex_ref, image_size, uv0, uv1, bg_col, tint_col);
      Arg<bool> retval = ImGui::ImageButton(str_id.value, tex_ref.value, image_size.value, uv0.value, uv1.value, bg_col.value, tint_col.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int BeginCombo(lua_State* L) {
      static const char* proto = "bool ImGui::BeginCombo(const char* label, const char* preview_value, ImGuiComboFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<const char*> preview_value(L,2);
      Arg<int,lua_Integer> flags(L,3,0);
      LUAWRAP_CHECK_ARGS(label, preview_value, flags);
      Arg<bool> retval = ImGui::BeginCombo(label.value, preview_value.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndCombo(lua_State*) {
      ImGui::EndCombo();
      return 0;
   }

   static int DragFloat(lua_State* L) {
      static const char* proto = "bool ImGui::DragFloat(const char* label, float* v, float v_speed=1.0, float v_min=0.0, float v_max=0.0, const char* format=%.3f, ImGuiSliderFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<float,lua_Number> v(L,2,UninitializedPointer());
      Arg<float,lua_Number> v_speed(L,3,1.0);
      Arg<float,lua_Number> v_min(L,4,0.0);
      Arg<float,lua_Number> v_max(L,5,0.0);
      Arg<const char*> format(L,6,"%.3f");
      Arg<int,lua_Integer> flags(L,7,0);
      LUAWRAP_CHECK_ARGS(label, v, v_speed, v_min, v_max, format, flags);
      Arg<bool> retval = ImGui::DragFloat(label.value, v.pointer(), v_speed.value, v_min.value, v_max.value, format.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      v.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int DragFloatRange2(lua_State* L) {
      static const char* proto = "bool ImGui::DragFloatRange2(const char* label, float* v_current_min, float* v_current_max, float v_speed=1.0, float v_min=0.0, float v_max=0.0, const char* format=%.3f, const char* format_max=NULL, ImGuiSliderFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<float,lua_Number> v_current_min(L,2,UninitializedPointer());
      Arg<float,lua_Number> v_current_max(L,3,UninitializedPointer());
      Arg<float,lua_Number> v_speed(L,4,1.0);
      Arg<float,lua_Number> v_min(L,5,0.0);
      Arg<float,lua_Number> v_max(L,6,0.0);
      Arg<const char*> format(L,7,"%.3f");
      Arg<const char*> format_max(L,8,NULL);
      Arg<int,lua_Integer> flags(L,9,0);
      LUAWRAP_CHECK_ARGS(label, v_current_min, v_current_max, v_speed, v_min, v_max, format, format_max, flags);
      Arg<bool> retval = ImGui::DragFloatRange2(label.value, v_current_min.pointer(), v_current_max.pointer(), v_speed.value, v_min.value, v_max.value, format.value, format_max.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      v_current_min.push_if_set(L);
      v_current_max.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int DragInt(lua_State* L) {
      static const char* proto = "bool ImGui::DragInt(const char* label, int* v, float v_speed=1.0, int v_min=0, int v_max=0, const char* format=%d, ImGuiSliderFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<int,lua_Integer> v(L,2,UninitializedPointer());
      Arg<float,lua_Number> v_speed(L,3,1.0);
      Arg<int,lua_Integer> v_min(L,4,0);
      Arg<int,lua_Integer> v_max(L,5,0);
      Arg<const char*> format(L,6,"%d");
      Arg<int,lua_Integer> flags(L,7,0);
      LUAWRAP_CHECK_ARGS(label, v, v_speed, v_min, v_max, format, flags);
      Arg<bool> retval = ImGui::DragInt(label.value, v.pointer(), v_speed.value, v_min.value, v_max.value, format.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      v.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int DragIntRange2(lua_State* L) {
      static const char* proto = "bool ImGui::DragIntRange2(const char* label, int* v_current_min, int* v_current_max, float v_speed=1.0, int v_min=0, int v_max=0, const char* format=%d, const char* format_max=NULL, ImGuiSliderFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<int,lua_Integer> v_current_min(L,2,UninitializedPointer());
      Arg<int,lua_Integer> v_current_max(L,3,UninitializedPointer());
      Arg<float,lua_Number> v_speed(L,4,1.0);
      Arg<int,lua_Integer> v_min(L,5,0);
      Arg<int,lua_Integer> v_max(L,6,0);
      Arg<const char*> format(L,7,"%d");
      Arg<const char*> format_max(L,8,NULL);
      Arg<int,lua_Integer> flags(L,9,0);
      LUAWRAP_CHECK_ARGS(label, v_current_min, v_current_max, v_speed, v_min, v_max, format, format_max, flags);
      Arg<bool> retval = ImGui::DragIntRange2(label.value, v_current_min.pointer(), v_current_max.pointer(), v_speed.value, v_min.value, v_max.value, format.value, format_max.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      v_current_min.push_if_set(L);
      v_current_max.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int SliderFloat(lua_State* L) {
      static const char* proto = "bool ImGui::SliderFloat(const char* label, float* v, float v_min, float v_max, const char* format=%.3f, ImGuiSliderFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<float,lua_Number> v(L,2,UninitializedPointer());
      Arg<float,lua_Number> v_min(L,3);
      Arg<float,lua_Number> v_max(L,4);
      Arg<const char*> format(L,5,"%.3f");
      Arg<int,lua_Integer> flags(L,6,0);
      LUAWRAP_CHECK_ARGS(label, v, v_min, v_max, format, flags);
      Arg<bool> retval = ImGui::SliderFloat(label.value, v.pointer(), v_min.value, v_max.value, format.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      v.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int SliderAngle(lua_State* L) {
      static const char* proto = "bool ImGui::SliderAngle(const char* label, float* v_rad, float v_degrees_min=-360.0, float v_degrees_max=+360.0, const char* format=%.0f deg, ImGuiSliderFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<float,lua_Number> v_rad(L,2,UninitializedPointer());
      Arg<float,lua_Number> v_degrees_min(L,3,-360.0);
      Arg<float,lua_Number> v_degrees_max(L,4,+360.0);
      Arg<const char*> format(L,5,"%.0f deg");
      Arg<int,lua_Integer> flags(L,6,0);
      LUAWRAP_CHECK_ARGS(label, v_rad, v_degrees_min, v_degrees_max, format, flags);
      Arg<bool> retval = ImGui::SliderAngle(label.value, v_rad.pointer(), v_degrees_min.value, v_degrees_max.value, format.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      v_rad.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int SliderInt(lua_State* L) {
      static const char* proto = "bool ImGui::SliderInt(const char* label, int* v, int v_min, int v_max, const char* format=%d, ImGuiSliderFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<int,lua_Integer> v(L,2,UninitializedPointer());
      Arg<int,lua_Integer> v_min(L,3);
      Arg<int,lua_Integer> v_max(L,4);
      Arg<const char*> format(L,5,"%d");
      Arg<int,lua_Integer> flags(L,6,0);
      LUAWRAP_CHECK_ARGS(label, v, v_min, v_max, format, flags);
      Arg<bool> retval = ImGui::SliderInt(label.value, v.pointer(), v_min.value, v_max.value, format.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      v.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int VSliderFloat(lua_State* L) {
      static const char* proto = "bool ImGui::VSliderFloat(const char* label, ImVec2 size, float* v, float v_min, float v_max, const char* format=%.3f, ImGuiSliderFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<ImVec2> size(L,2);
      Arg<float,lua_Number> v(L,4,UninitializedPointer());
      Arg<float,lua_Number> v_min(L,5);
      Arg<float,lua_Number> v_max(L,6);
      Arg<const char*> format(L,7,"%.3f");
      Arg<int,lua_Integer> flags(L,8,0);
      LUAWRAP_CHECK_ARGS(label, size, v, v_min, v_max, format, flags);
      Arg<bool> retval = ImGui::VSliderFloat(label.value, size.value, v.pointer(), v_min.value, v_max.value, format.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      v.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int VSliderInt(lua_State* L) {
      static const char* proto = "bool ImGui::VSliderInt(const char* label, ImVec2 size, int* v, int v_min, int v_max, const char* format=%d, ImGuiSliderFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<ImVec2> size(L,2);
      Arg<int,lua_Integer> v(L,4,UninitializedPointer());
      Arg<int,lua_Integer> v_min(L,5);
      Arg<int,lua_Integer> v_max(L,6);
      Arg<const char*> format(L,7,"%d");
      Arg<int,lua_Integer> flags(L,8,0);
      LUAWRAP_CHECK_ARGS(label, size, v, v_min, v_max, format, flags);
      Arg<bool> retval = ImGui::VSliderInt(label.value, size.value, v.pointer(), v_min.value, v_max.value, format.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      v.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int InputFloat(lua_State* L) {
      static const char* proto = "bool ImGui::InputFloat(const char* label, float* v, float step=0.0, float step_fast=0.0, const char* format=%.3f, ImGuiInputTextFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<float,lua_Number> v(L,2,UninitializedPointer());
      Arg<float,lua_Number> step(L,3,0.0);
      Arg<float,lua_Number> step_fast(L,4,0.0);
      Arg<const char*> format(L,5,"%.3f");
      Arg<int,lua_Integer> flags(L,6,0);
      LUAWRAP_CHECK_ARGS(label, v, step, step_fast, format, flags);
      Arg<bool> retval = ImGui::InputFloat(label.value, v.pointer(), step.value, step_fast.value, format.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      v.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int InputInt(lua_State* L) {
      static const char* proto = "bool ImGui::InputInt(const char* label, int* v, int step=1, int step_fast=100, ImGuiInputTextFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<int,lua_Integer> v(L,2,UninitializedPointer());
      Arg<int,lua_Integer> step(L,3,1);
      Arg<int,lua_Integer> step_fast(L,4,100);
      Arg<int,lua_Integer> flags(L,5,0);
      LUAWRAP_CHECK_ARGS(label, v, step, step_fast, flags);
      Arg<bool> retval = ImGui::InputInt(label.value, v.pointer(), step.value, step_fast.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      v.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int InputDouble(lua_State* L) {
      static const char* proto = "bool ImGui::InputDouble(const char* label, double* v, double step=0.0, double step_fast=0.0, const char* format=%.6f, ImGuiInputTextFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<double,lua_Number> v(L,2,UninitializedPointer());
      Arg<double,lua_Number> step(L,3,0.0);
      Arg<double,lua_Number> step_fast(L,4,0.0);
      Arg<const char*> format(L,5,"%.6f");
      Arg<int,lua_Integer> flags(L,6,0);
      LUAWRAP_CHECK_ARGS(label, v, step, step_fast, format, flags);
      Arg<bool> retval = ImGui::InputDouble(label.value, v.pointer(), step.value, step_fast.value, format.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      v.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int ColorButton(lua_State* L) {
      static const char* proto = "bool ImGui::ColorButton(const char* desc_id, ImVec4 col, ImGuiColorEditFlags flags=0, ImVec2 size=ImVec2(0, 0))";
      Arg<const char*> desc_id(L,1);
      Arg<ImVec4> col(L,2);
      Arg<int,lua_Integer> flags(L,6,0);
      Arg<ImVec2> size(L,7,ImVec2(0, 0));
      LUAWRAP_CHECK_ARGS(desc_id, col, flags, size);
      Arg<bool> retval = ImGui::ColorButton(desc_id.value, col.value, flags.value, size.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetColorEditOptions(lua_State* L) {
      static const char* proto = "void ImGui::SetColorEditOptions(ImGuiColorEditFlags flags)";
      Arg<int,lua_Integer> flags(L,1);
      LUAWRAP_CHECK_ARGS(flags);
      ImGui::SetColorEditOptions(flags.value);
      return 0;
   }

   static int TreeNode(lua_State* L) {
      static const char* proto = "bool ImGui::TreeNode(const char* label)";
      Arg<const char*> label(L,1);
      LUAWRAP_CHECK_ARGS(label);
      Arg<bool> retval = ImGui::TreeNode(label.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TreeNodeEx(lua_State* L) {
      static const char* proto = "bool ImGui::TreeNodeEx(const char* label, ImGuiTreeNodeFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<int,lua_Integer> flags(L,2,0);
      LUAWRAP_CHECK_ARGS(label, flags);
      Arg<bool> retval = ImGui::TreeNodeEx(label.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TreePush(lua_State* L) {
      static const char* proto = "void ImGui::TreePush(const char* str_id)";
      Arg<const char*> str_id(L,1);
      LUAWRAP_CHECK_ARGS(str_id);
      ImGui::TreePush(str_id.value);
      return 0;
   }

   static int TreePop(lua_State*) {
      ImGui::TreePop();
      return 0;
   }

   static int GetTreeNodeToLabelSpacing(lua_State* L) {
      Arg<float,lua_Number> retval = ImGui::GetTreeNodeToLabelSpacing();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int CollapsingHeader(lua_State* L) {
      static const char* proto = "bool ImGui::CollapsingHeader(const char* label, ImGuiTreeNodeFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<int,lua_Integer> flags(L,2,0);
      LUAWRAP_CHECK_ARGS(label, flags);
      Arg<bool> retval = ImGui::CollapsingHeader(label.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetNextItemOpen(lua_State* L) {
      static const char* proto = "void ImGui::SetNextItemOpen(bool is_open, ImGuiCond cond=0)";
      Arg<bool> is_open(L,1);
      Arg<int,lua_Integer> cond(L,2,0);
      LUAWRAP_CHECK_ARGS(is_open, cond);
      ImGui::SetNextItemOpen(is_open.value, cond.value);
      return 0;
   }

   static int SetNextItemStorageID(lua_State* L) {
      static const char* proto = "void ImGui::SetNextItemStorageID(ImGuiID storage_id)";
      Arg<unsigned int,lua_Integer> storage_id(L,1);
      LUAWRAP_CHECK_ARGS(storage_id);
      ImGui::SetNextItemStorageID(storage_id.value);
      return 0;
   }

   static int Selectable(lua_State* L) {
      static const char* proto = "bool ImGui::Selectable(const char* label, bool selected=false, ImGuiSelectableFlags flags=0, ImVec2 size=ImVec2(0, 0))";
      Arg<const char*> label(L,1);
      Arg<bool> selected(L,2,false);
      Arg<int,lua_Integer> flags(L,3,0);
      Arg<ImVec2> size(L,4,ImVec2(0, 0));
      LUAWRAP_CHECK_ARGS(label, selected, flags, size);
      Arg<bool> retval = ImGui::Selectable(label.value, selected.value, flags.value, size.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int BeginMultiSelect(lua_State* L) {
      static const char* proto = "ImGuiMultiSelectIO* ImGui::BeginMultiSelect(ImGuiMultiSelectFlags flags, int selection_size=-1, int items_count=-1)";
      Arg<int,lua_Integer> flags(L,1);
      Arg<int,lua_Integer> selection_size(L,2,-1);
      Arg<int,lua_Integer> items_count(L,3,-1);
      LUAWRAP_CHECK_ARGS(flags, selection_size, items_count);
      Arg<ImGuiMultiSelectIO*> retval = ImGui::BeginMultiSelect(flags.value, selection_size.value, items_count.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndMultiSelect(lua_State* L) {
      Arg<ImGuiMultiSelectIO*> retval = ImGui::EndMultiSelect();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetNextItemSelectionUserData(lua_State* L) {
      static const char* proto = "void ImGui::SetNextItemSelectionUserData(ImGuiSelectionUserData selection_user_data)";
      Arg<long long,lua_Integer> selection_user_data(L,1);
      LUAWRAP_CHECK_ARGS(selection_user_data);
      ImGui::SetNextItemSelectionUserData(selection_user_data.value);
      return 0;
   }

   static int IsItemToggledSelection(lua_State* L) {
      Arg<bool> retval = ImGui::IsItemToggledSelection();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int BeginListBox(lua_State* L) {
      static const char* proto = "bool ImGui::BeginListBox(const char* label, ImVec2 size=ImVec2(0, 0))";
      Arg<const char*> label(L,1);
      Arg<ImVec2> size(L,2,ImVec2(0, 0));
      LUAWRAP_CHECK_ARGS(label, size);
      Arg<bool> retval = ImGui::BeginListBox(label.value, size.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndListBox(lua_State*) {
      ImGui::EndListBox();
      return 0;
   }

   static int PlotLines(lua_State* L) {
      static const char* proto = "void ImGui::PlotLines(const char* label, const float* values, int values_count, int values_offset=0, const char* overlay_text=NULL, float scale_min=FLT_MAX, float scale_max=FLT_MAX, ImVec2 graph_size=ImVec2(0, 0), int stride=sizeof(float))";
      Arg<const char*> label(L,1);
      Arg<const float*> values(L,2);
      Arg<int,lua_Integer> values_count(L,3);
      Arg<int,lua_Integer> values_offset(L,4,0);
      Arg<const char*> overlay_text(L,5,NULL);
      Arg<float,lua_Number> scale_min(L,6,FLT_MAX);
      Arg<float,lua_Number> scale_max(L,7,FLT_MAX);
      Arg<ImVec2> graph_size(L,8,ImVec2(0, 0));
      Arg<int,lua_Integer> stride(L,10,sizeof(float));
      LUAWRAP_CHECK_ARGS(label, values, values_count, values_offset, overlay_text, scale_min, scale_max, graph_size, stride);
      ImGui::PlotLines(label.value, values.value, values_count.value, values_offset.value, overlay_text.value, scale_min.value, scale_max.value, graph_size.value, stride.value);
      return 0;
   }

   static int PlotHistogram(lua_State* L) {
      static const char* proto = "void ImGui::PlotHistogram(const char* label, const float* values, int values_count, int values_offset=0, const char* overlay_text=NULL, float scale_min=FLT_MAX, float scale_max=FLT_MAX, ImVec2 graph_size=ImVec2(0, 0), int stride=sizeof(float))";
      Arg<const char*> label(L,1);
      Arg<const float*> values(L,2);
      Arg<int,lua_Integer> values_count(L,3);
      Arg<int,lua_Integer> values_offset(L,4,0);
      Arg<const char*> overlay_text(L,5,NULL);
      Arg<float,lua_Number> scale_min(L,6,FLT_MAX);
      Arg<float,lua_Number> scale_max(L,7,FLT_MAX);
      Arg<ImVec2> graph_size(L,8,ImVec2(0, 0));
      Arg<int,lua_Integer> stride(L,10,sizeof(float));
      LUAWRAP_CHECK_ARGS(label, values, values_count, values_offset, overlay_text, scale_min, scale_max, graph_size, stride);
      ImGui::PlotHistogram(label.value, values.value, values_count.value, values_offset.value, overlay_text.value, scale_min.value, scale_max.value, graph_size.value, stride.value);
      return 0;
   }

   static int Value(lua_State* L) {
      static const char* proto = "void ImGui::Value(const char* prefix, bool b)";
      Arg<const char*> prefix(L,1);
      Arg<bool> b(L,2);
      LUAWRAP_CHECK_ARGS(prefix, b);
      ImGui::Value(prefix.value, b.value);
      return 0;
   }

   static int BeginMenuBar(lua_State* L) {
      Arg<bool> retval = ImGui::BeginMenuBar();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndMenuBar(lua_State*) {
      ImGui::EndMenuBar();
      return 0;
   }

   static int BeginMainMenuBar(lua_State* L) {
      Arg<bool> retval = ImGui::BeginMainMenuBar();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndMainMenuBar(lua_State*) {
      ImGui::EndMainMenuBar();
      return 0;
   }

   static int BeginMenu(lua_State* L) {
      static const char* proto = "bool ImGui::BeginMenu(const char* label, bool enabled=true)";
      Arg<const char*> label(L,1);
      Arg<bool> enabled(L,2,true);
      LUAWRAP_CHECK_ARGS(label, enabled);
      Arg<bool> retval = ImGui::BeginMenu(label.value, enabled.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndMenu(lua_State*) {
      ImGui::EndMenu();
      return 0;
   }

   static int MenuItem(lua_State* L) {
      static const char* proto = "bool ImGui::MenuItem(const char* label, const char* shortcut=NULL, bool selected=false, bool enabled=true)";
      Arg<const char*> label(L,1);
      Arg<const char*> shortcut(L,2,NULL);
      Arg<bool> selected(L,3,false);
      Arg<bool> enabled(L,4,true);
      LUAWRAP_CHECK_ARGS(label, shortcut, selected, enabled);
      Arg<bool> retval = ImGui::MenuItem(label.value, shortcut.value, selected.value, enabled.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int BeginTooltip(lua_State* L) {
      Arg<bool> retval = ImGui::BeginTooltip();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndTooltip(lua_State*) {
      ImGui::EndTooltip();
      return 0;
   }

   static int BeginItemTooltip(lua_State* L) {
      Arg<bool> retval = ImGui::BeginItemTooltip();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int BeginPopup(lua_State* L) {
      static const char* proto = "bool ImGui::BeginPopup(const char* str_id, ImGuiWindowFlags flags=0)";
      Arg<const char*> str_id(L,1);
      Arg<int,lua_Integer> flags(L,2,0);
      LUAWRAP_CHECK_ARGS(str_id, flags);
      Arg<bool> retval = ImGui::BeginPopup(str_id.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int BeginPopupModal(lua_State* L) {
      static const char* proto = "bool ImGui::BeginPopupModal(const char* name, bool* p_open=NULL, ImGuiWindowFlags flags=0)";
      Arg<const char*> name(L,1);
      Arg<bool> p_open(L,2,NullPointer());
      Arg<int,lua_Integer> flags(L,3,0);
      LUAWRAP_CHECK_ARGS(name, p_open, flags);
      Arg<bool> retval = ImGui::BeginPopupModal(name.value, p_open.pointer(), flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      p_open.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndPopup(lua_State*) {
      ImGui::EndPopup();
      return 0;
   }

   static int OpenPopup(lua_State* L) {
      static const char* proto = "void ImGui::OpenPopup(const char* str_id, ImGuiPopupFlags popup_flags=0)";
      Arg<const char*> str_id(L,1);
      Arg<int,lua_Integer> popup_flags(L,2,0);
      LUAWRAP_CHECK_ARGS(str_id, popup_flags);
      ImGui::OpenPopup(str_id.value, popup_flags.value);
      return 0;
   }

   static int OpenPopupOnItemClick(lua_State* L) {
      static const char* proto = "void ImGui::OpenPopupOnItemClick(const char* str_id=NULL, ImGuiPopupFlags popup_flags=1)";
      Arg<const char*> str_id(L,1,NULL);
      Arg<int,lua_Integer> popup_flags(L,2,1);
      LUAWRAP_CHECK_ARGS(str_id, popup_flags);
      ImGui::OpenPopupOnItemClick(str_id.value, popup_flags.value);
      return 0;
   }

   static int CloseCurrentPopup(lua_State*) {
      ImGui::CloseCurrentPopup();
      return 0;
   }

   static int BeginPopupContextItem(lua_State* L) {
      static const char* proto = "bool ImGui::BeginPopupContextItem(const char* str_id=NULL, ImGuiPopupFlags popup_flags=1)";
      Arg<const char*> str_id(L,1,NULL);
      Arg<int,lua_Integer> popup_flags(L,2,1);
      LUAWRAP_CHECK_ARGS(str_id, popup_flags);
      Arg<bool> retval = ImGui::BeginPopupContextItem(str_id.value, popup_flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int BeginPopupContextWindow(lua_State* L) {
      static const char* proto = "bool ImGui::BeginPopupContextWindow(const char* str_id=NULL, ImGuiPopupFlags popup_flags=1)";
      Arg<const char*> str_id(L,1,NULL);
      Arg<int,lua_Integer> popup_flags(L,2,1);
      LUAWRAP_CHECK_ARGS(str_id, popup_flags);
      Arg<bool> retval = ImGui::BeginPopupContextWindow(str_id.value, popup_flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int BeginPopupContextVoid(lua_State* L) {
      static const char* proto = "bool ImGui::BeginPopupContextVoid(const char* str_id=NULL, ImGuiPopupFlags popup_flags=1)";
      Arg<const char*> str_id(L,1,NULL);
      Arg<int,lua_Integer> popup_flags(L,2,1);
      LUAWRAP_CHECK_ARGS(str_id, popup_flags);
      Arg<bool> retval = ImGui::BeginPopupContextVoid(str_id.value, popup_flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsPopupOpen(lua_State* L) {
      static const char* proto = "bool ImGui::IsPopupOpen(const char* str_id, ImGuiPopupFlags flags=0)";
      Arg<const char*> str_id(L,1);
      Arg<int,lua_Integer> flags(L,2,0);
      LUAWRAP_CHECK_ARGS(str_id, flags);
      Arg<bool> retval = ImGui::IsPopupOpen(str_id.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int BeginTable(lua_State* L) {
      static const char* proto = "bool ImGui::BeginTable(const char* str_id, int columns, ImGuiTableFlags flags=0, ImVec2 outer_size=ImVec2(0.0f, 0.0f), float inner_width=0.0)";
      Arg<const char*> str_id(L,1);
      Arg<int,lua_Integer> columns(L,2);
      Arg<int,lua_Integer> flags(L,3,0);
      Arg<ImVec2> outer_size(L,4,ImVec2(0.0f, 0.0f));
      Arg<float,lua_Number> inner_width(L,6,0.0);
      LUAWRAP_CHECK_ARGS(str_id, columns, flags, outer_size, inner_width);
      Arg<bool> retval = ImGui::BeginTable(str_id.value, columns.value, flags.value, outer_size.value, inner_width.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndTable(lua_State*) {
      ImGui::EndTable();
      return 0;
   }

   static int TableNextRow(lua_State* L) {
      static const char* proto = "void ImGui::TableNextRow(ImGuiTableRowFlags row_flags=0, float min_row_height=0.0)";
      Arg<int,lua_Integer> row_flags(L,1,0);
      Arg<float,lua_Number> min_row_height(L,2,0.0);
      LUAWRAP_CHECK_ARGS(row_flags, min_row_height);
      ImGui::TableNextRow(row_flags.value, min_row_height.value);
      return 0;
   }

   static int TableNextColumn(lua_State* L) {
      Arg<bool> retval = ImGui::TableNextColumn();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TableSetColumnIndex(lua_State* L) {
      static const char* proto = "bool ImGui::TableSetColumnIndex(int column_n)";
      Arg<int,lua_Integer> column_n(L,1);
      LUAWRAP_CHECK_ARGS(column_n);
      Arg<bool> retval = ImGui::TableSetColumnIndex(column_n.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TableSetupColumn(lua_State* L) {
      static const char* proto = "void ImGui::TableSetupColumn(const char* label, ImGuiTableColumnFlags flags=0, float init_width_or_weight=0.0, ImGuiID user_id=0)";
      Arg<const char*> label(L,1);
      Arg<int,lua_Integer> flags(L,2,0);
      Arg<float,lua_Number> init_width_or_weight(L,3,0.0);
      Arg<unsigned int,lua_Integer> user_id(L,4,0);
      LUAWRAP_CHECK_ARGS(label, flags, init_width_or_weight, user_id);
      ImGui::TableSetupColumn(label.value, flags.value, init_width_or_weight.value, user_id.value);
      return 0;
   }

   static int TableSetupScrollFreeze(lua_State* L) {
      static const char* proto = "void ImGui::TableSetupScrollFreeze(int cols, int rows)";
      Arg<int,lua_Integer> cols(L,1);
      Arg<int,lua_Integer> rows(L,2);
      LUAWRAP_CHECK_ARGS(cols, rows);
      ImGui::TableSetupScrollFreeze(cols.value, rows.value);
      return 0;
   }

   static int TableHeader(lua_State* L) {
      static const char* proto = "void ImGui::TableHeader(const char* label)";
      Arg<const char*> label(L,1);
      LUAWRAP_CHECK_ARGS(label);
      ImGui::TableHeader(label.value);
      return 0;
   }

   static int TableHeadersRow(lua_State*) {
      ImGui::TableHeadersRow();
      return 0;
   }

   static int TableAngledHeadersRow(lua_State*) {
      ImGui::TableAngledHeadersRow();
      return 0;
   }

   static int TableGetSortSpecs(lua_State* L) {
      Arg<ImGuiTableSortSpecs*> retval = ImGui::TableGetSortSpecs();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TableGetColumnCount(lua_State* L) {
      Arg<int,lua_Integer> retval = ImGui::TableGetColumnCount();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TableGetColumnIndex(lua_State* L) {
      Arg<int,lua_Integer> retval = ImGui::TableGetColumnIndex();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TableGetRowIndex(lua_State* L) {
      Arg<int,lua_Integer> retval = ImGui::TableGetRowIndex();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TableGetColumnName(lua_State* L) {
      static const char* proto = "const char* ImGui::TableGetColumnName(int column_n=-1)";
      Arg<int,lua_Integer> column_n(L,1,-1);
      LUAWRAP_CHECK_ARGS(column_n);
      Arg<const char*> retval = ImGui::TableGetColumnName(column_n.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TableGetColumnFlags(lua_State* L) {
      static const char* proto = "ImGuiTableColumnFlags ImGui::TableGetColumnFlags(int column_n=-1)";
      Arg<int,lua_Integer> column_n(L,1,-1);
      LUAWRAP_CHECK_ARGS(column_n);
      Arg<ImGuiTableColumnFlags,lua_Integer> retval = ImGui::TableGetColumnFlags(column_n.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TableSetColumnEnabled(lua_State* L) {
      static const char* proto = "void ImGui::TableSetColumnEnabled(int column_n, bool v)";
      Arg<int,lua_Integer> column_n(L,1);
      Arg<bool> v(L,2);
      LUAWRAP_CHECK_ARGS(column_n, v);
      ImGui::TableSetColumnEnabled(column_n.value, v.value);
      return 0;
   }

   static int TableGetHoveredColumn(lua_State* L) {
      Arg<int,lua_Integer> retval = ImGui::TableGetHoveredColumn();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int TableSetBgColor(lua_State* L) {
      static const char* proto = "void ImGui::TableSetBgColor(ImGuiTableBgTarget target, ImU32 color, int column_n=-1)";
      Arg<int,lua_Integer> target(L,1);
      Arg<unsigned int,lua_Integer> color(L,2);
      Arg<int,lua_Integer> column_n(L,3,-1);
      LUAWRAP_CHECK_ARGS(target, color, column_n);
      ImGui::TableSetBgColor(target.value, color.value, column_n.value);
      return 0;
   }

   static int Columns(lua_State* L) {
      static const char* proto = "void ImGui::Columns(int count=1, const char* id=NULL, bool borders=true)";
      Arg<int,lua_Integer> count(L,1,1);
      Arg<const char*> id(L,2,NULL);
      Arg<bool> borders(L,3,true);
      LUAWRAP_CHECK_ARGS(count, id, borders);
      ImGui::Columns(count.value, id.value, borders.value);
      return 0;
   }

   static int NextColumn(lua_State*) {
      ImGui::NextColumn();
      return 0;
   }

   static int GetColumnIndex(lua_State* L) {
      Arg<int,lua_Integer> retval = ImGui::GetColumnIndex();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetColumnWidth(lua_State* L) {
      static const char* proto = "float ImGui::GetColumnWidth(int column_index=-1)";
      Arg<int,lua_Integer> column_index(L,1,-1);
      LUAWRAP_CHECK_ARGS(column_index);
      Arg<float,lua_Number> retval = ImGui::GetColumnWidth(column_index.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetColumnWidth(lua_State* L) {
      static const char* proto = "void ImGui::SetColumnWidth(int column_index, float width)";
      Arg<int,lua_Integer> column_index(L,1);
      Arg<float,lua_Number> width(L,2);
      LUAWRAP_CHECK_ARGS(column_index, width);
      ImGui::SetColumnWidth(column_index.value, width.value);
      return 0;
   }

   static int GetColumnOffset(lua_State* L) {
      static const char* proto = "float ImGui::GetColumnOffset(int column_index=-1)";
      Arg<int,lua_Integer> column_index(L,1,-1);
      LUAWRAP_CHECK_ARGS(column_index);
      Arg<float,lua_Number> retval = ImGui::GetColumnOffset(column_index.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetColumnOffset(lua_State* L) {
      static const char* proto = "void ImGui::SetColumnOffset(int column_index, float offset_x)";
      Arg<int,lua_Integer> column_index(L,1);
      Arg<float,lua_Number> offset_x(L,2);
      LUAWRAP_CHECK_ARGS(column_index, offset_x);
      ImGui::SetColumnOffset(column_index.value, offset_x.value);
      return 0;
   }

   static int GetColumnsCount(lua_State* L) {
      Arg<int,lua_Integer> retval = ImGui::GetColumnsCount();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int BeginTabBar(lua_State* L) {
      static const char* proto = "bool ImGui::BeginTabBar(const char* str_id, ImGuiTabBarFlags flags=0)";
      Arg<const char*> str_id(L,1);
      Arg<int,lua_Integer> flags(L,2,0);
      LUAWRAP_CHECK_ARGS(str_id, flags);
      Arg<bool> retval = ImGui::BeginTabBar(str_id.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndTabBar(lua_State*) {
      ImGui::EndTabBar();
      return 0;
   }

   static int BeginTabItem(lua_State* L) {
      static const char* proto = "bool ImGui::BeginTabItem(const char* label, bool* p_open=NULL, ImGuiTabItemFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<bool> p_open(L,2,NullPointer());
      Arg<int,lua_Integer> flags(L,3,0);
      LUAWRAP_CHECK_ARGS(label, p_open, flags);
      Arg<bool> retval = ImGui::BeginTabItem(label.value, p_open.pointer(), flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      p_open.push_if_set(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndTabItem(lua_State*) {
      ImGui::EndTabItem();
      return 0;
   }

   static int TabItemButton(lua_State* L) {
      static const char* proto = "bool ImGui::TabItemButton(const char* label, ImGuiTabItemFlags flags=0)";
      Arg<const char*> label(L,1);
      Arg<int,lua_Integer> flags(L,2,0);
      LUAWRAP_CHECK_ARGS(label, flags);
      Arg<bool> retval = ImGui::TabItemButton(label.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetTabItemClosed(lua_State* L) {
      static const char* proto = "void ImGui::SetTabItemClosed(const char* tab_or_docked_window_label)";
      Arg<const char*> tab_or_docked_window_label(L,1);
      LUAWRAP_CHECK_ARGS(tab_or_docked_window_label);
      ImGui::SetTabItemClosed(tab_or_docked_window_label.value);
      return 0;
   }

   static int DockSpace(lua_State* L) {
      static const char* proto = "ImGuiID ImGui::DockSpace(ImGuiID dockspace_id, ImVec2 size=ImVec2(0, 0), ImGuiDockNodeFlags flags=0, const ImGuiWindowClass* window_class=NULL)";
      Arg<unsigned int,lua_Integer> dockspace_id(L,1);
      Arg<ImVec2> size(L,2,ImVec2(0, 0));
      Arg<int,lua_Integer> flags(L,4,0);
      Arg<const ImGuiWindowClass*> window_class(L,5,NULL);
      LUAWRAP_CHECK_ARGS(dockspace_id, size, flags, window_class);
      Arg<ImGuiID,lua_Integer> retval = ImGui::DockSpace(dockspace_id.value, size.value, flags.value, window_class.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int DockSpaceOverViewport(lua_State* L) {
      static const char* proto = "ImGuiID ImGui::DockSpaceOverViewport(ImGuiID dockspace_id=0, const ImGuiViewport* viewport=NULL, ImGuiDockNodeFlags flags=0, const ImGuiWindowClass* window_class=NULL)";
      Arg<unsigned int,lua_Integer> dockspace_id(L,1,0);
      Arg<const ImGuiViewport*> viewport(L,2,NULL);
      Arg<int,lua_Integer> flags(L,3,0);
      Arg<const ImGuiWindowClass*> window_class(L,4,NULL);
      LUAWRAP_CHECK_ARGS(dockspace_id, viewport, flags, window_class);
      Arg<ImGuiID,lua_Integer> retval = ImGui::DockSpaceOverViewport(dockspace_id.value, viewport.value, flags.value, window_class.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetNextWindowDockID(lua_State* L) {
      static const char* proto = "void ImGui::SetNextWindowDockID(ImGuiID dock_id, ImGuiCond cond=0)";
      Arg<unsigned int,lua_Integer> dock_id(L,1);
      Arg<int,lua_Integer> cond(L,2,0);
      LUAWRAP_CHECK_ARGS(dock_id, cond);
      ImGui::SetNextWindowDockID(dock_id.value, cond.value);
      return 0;
   }

   static int SetNextWindowClass(lua_State* L) {
      static const char* proto = "void ImGui::SetNextWindowClass(const ImGuiWindowClass* window_class)";
      Arg<const ImGuiWindowClass*> window_class(L,1);
      LUAWRAP_CHECK_ARGS(window_class);
      ImGui::SetNextWindowClass(window_class.value);
      return 0;
   }

   static int GetWindowDockID(lua_State* L) {
      Arg<ImGuiID,lua_Integer> retval = ImGui::GetWindowDockID();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsWindowDocked(lua_State* L) {
      Arg<bool> retval = ImGui::IsWindowDocked();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int LogToTTY(lua_State* L) {
      static const char* proto = "void ImGui::LogToTTY(int auto_open_depth=-1)";
      Arg<int,lua_Integer> auto_open_depth(L,1,-1);
      LUAWRAP_CHECK_ARGS(auto_open_depth);
      ImGui::LogToTTY(auto_open_depth.value);
      return 0;
   }

   static int LogToFile(lua_State* L) {
      static const char* proto = "void ImGui::LogToFile(int auto_open_depth=-1, const char* filename=NULL)";
      Arg<int,lua_Integer> auto_open_depth(L,1,-1);
      Arg<const char*> filename(L,2,NULL);
      LUAWRAP_CHECK_ARGS(auto_open_depth, filename);
      ImGui::LogToFile(auto_open_depth.value, filename.value);
      return 0;
   }

   static int LogToClipboard(lua_State* L) {
      static const char* proto = "void ImGui::LogToClipboard(int auto_open_depth=-1)";
      Arg<int,lua_Integer> auto_open_depth(L,1,-1);
      LUAWRAP_CHECK_ARGS(auto_open_depth);
      ImGui::LogToClipboard(auto_open_depth.value);
      return 0;
   }

   static int LogFinish(lua_State*) {
      ImGui::LogFinish();
      return 0;
   }

   static int LogButtons(lua_State*) {
      ImGui::LogButtons();
      return 0;
   }

   static int BeginDragDropSource(lua_State* L) {
      static const char* proto = "bool ImGui::BeginDragDropSource(ImGuiDragDropFlags flags=0)";
      Arg<int,lua_Integer> flags(L,1,0);
      LUAWRAP_CHECK_ARGS(flags);
      Arg<bool> retval = ImGui::BeginDragDropSource(flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndDragDropSource(lua_State*) {
      ImGui::EndDragDropSource();
      return 0;
   }

   static int BeginDragDropTarget(lua_State* L) {
      Arg<bool> retval = ImGui::BeginDragDropTarget();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int AcceptDragDropPayload(lua_State* L) {
      static const char* proto = "const ImGuiPayload* ImGui::AcceptDragDropPayload(const char* type, ImGuiDragDropFlags flags=0)";
      Arg<const char*> type(L,1);
      Arg<int,lua_Integer> flags(L,2,0);
      LUAWRAP_CHECK_ARGS(type, flags);
      Arg<const ImGuiPayload*> retval = ImGui::AcceptDragDropPayload(type.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int EndDragDropTarget(lua_State*) {
      ImGui::EndDragDropTarget();
      return 0;
   }

   static int GetDragDropPayload(lua_State* L) {
      Arg<const ImGuiPayload*> retval = ImGui::GetDragDropPayload();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int BeginDisabled(lua_State* L) {
      static const char* proto = "void ImGui::BeginDisabled(bool disabled=true)";
      Arg<bool> disabled(L,1,true);
      LUAWRAP_CHECK_ARGS(disabled);
      ImGui::BeginDisabled(disabled.value);
      return 0;
   }

   static int EndDisabled(lua_State*) {
      ImGui::EndDisabled();
      return 0;
   }

   static int PushClipRect(lua_State* L) {
      static const char* proto = "void ImGui::PushClipRect(ImVec2 clip_rect_min, ImVec2 clip_rect_max, bool intersect_with_current_clip_rect)";
      Arg<ImVec2> clip_rect_min(L,1);
      Arg<ImVec2> clip_rect_max(L,3);
      Arg<bool> intersect_with_current_clip_rect(L,5);
      LUAWRAP_CHECK_ARGS(clip_rect_min, clip_rect_max, intersect_with_current_clip_rect);
      ImGui::PushClipRect(clip_rect_min.value, clip_rect_max.value, intersect_with_current_clip_rect.value);
      return 0;
   }

   static int PopClipRect(lua_State*) {
      ImGui::PopClipRect();
      return 0;
   }

   static int SetItemDefaultFocus(lua_State*) {
      ImGui::SetItemDefaultFocus();
      return 0;
   }

   static int SetKeyboardFocusHere(lua_State* L) {
      static const char* proto = "void ImGui::SetKeyboardFocusHere(int offset=0)";
      Arg<int,lua_Integer> offset(L,1,0);
      LUAWRAP_CHECK_ARGS(offset);
      ImGui::SetKeyboardFocusHere(offset.value);
      return 0;
   }

   static int SetNavCursorVisible(lua_State* L) {
      static const char* proto = "void ImGui::SetNavCursorVisible(bool visible)";
      Arg<bool> visible(L,1);
      LUAWRAP_CHECK_ARGS(visible);
      ImGui::SetNavCursorVisible(visible.value);
      return 0;
   }

   static int SetNextItemAllowOverlap(lua_State*) {
      ImGui::SetNextItemAllowOverlap();
      return 0;
   }

   static int IsItemHovered(lua_State* L) {
      static const char* proto = "bool ImGui::IsItemHovered(ImGuiHoveredFlags flags=0)";
      Arg<int,lua_Integer> flags(L,1,0);
      LUAWRAP_CHECK_ARGS(flags);
      Arg<bool> retval = ImGui::IsItemHovered(flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsItemActive(lua_State* L) {
      Arg<bool> retval = ImGui::IsItemActive();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsItemFocused(lua_State* L) {
      Arg<bool> retval = ImGui::IsItemFocused();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsItemClicked(lua_State* L) {
      static const char* proto = "bool ImGui::IsItemClicked(ImGuiMouseButton mouse_button=0)";
      Arg<int,lua_Integer> mouse_button(L,1,0);
      LUAWRAP_CHECK_ARGS(mouse_button);
      Arg<bool> retval = ImGui::IsItemClicked(mouse_button.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsItemVisible(lua_State* L) {
      Arg<bool> retval = ImGui::IsItemVisible();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsItemEdited(lua_State* L) {
      Arg<bool> retval = ImGui::IsItemEdited();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsItemActivated(lua_State* L) {
      Arg<bool> retval = ImGui::IsItemActivated();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsItemDeactivated(lua_State* L) {
      Arg<bool> retval = ImGui::IsItemDeactivated();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsItemDeactivatedAfterEdit(lua_State* L) {
      Arg<bool> retval = ImGui::IsItemDeactivatedAfterEdit();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsItemToggledOpen(lua_State* L) {
      Arg<bool> retval = ImGui::IsItemToggledOpen();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsAnyItemHovered(lua_State* L) {
      Arg<bool> retval = ImGui::IsAnyItemHovered();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsAnyItemActive(lua_State* L) {
      Arg<bool> retval = ImGui::IsAnyItemActive();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsAnyItemFocused(lua_State* L) {
      Arg<bool> retval = ImGui::IsAnyItemFocused();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetItemID(lua_State* L) {
      Arg<ImGuiID,lua_Integer> retval = ImGui::GetItemID();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetItemRectMin(lua_State* L) {
      Arg<ImVec2> retval = ImGui::GetItemRectMin();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetItemRectMax(lua_State* L) {
      Arg<ImVec2> retval = ImGui::GetItemRectMax();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetItemRectSize(lua_State* L) {
      Arg<ImVec2> retval = ImGui::GetItemRectSize();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetItemFlags(lua_State* L) {
      Arg<ImGuiItemFlags,lua_Integer> retval = ImGui::GetItemFlags();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetMainViewport(lua_State* L) {
      Arg<ImGuiViewport*> retval = ImGui::GetMainViewport();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetBackgroundDrawList(lua_State* L) {
      static const char* proto = "ImDrawList* ImGui::GetBackgroundDrawList(ImGuiViewport* viewport=NULL)";
      Arg<ImGuiViewport*> viewport(L,1,NULL);
      LUAWRAP_CHECK_ARGS(viewport);
      Arg<ImDrawList*> retval = ImGui::GetBackgroundDrawList(viewport.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetForegroundDrawList(lua_State* L) {
      static const char* proto = "ImDrawList* ImGui::GetForegroundDrawList(ImGuiViewport* viewport=NULL)";
      Arg<ImGuiViewport*> viewport(L,1,NULL);
      LUAWRAP_CHECK_ARGS(viewport);
      Arg<ImDrawList*> retval = ImGui::GetForegroundDrawList(viewport.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsRectVisible(lua_State* L) {
      static const char* proto = "bool ImGui::IsRectVisible(ImVec2 size)";
      Arg<ImVec2> size(L,1);
      LUAWRAP_CHECK_ARGS(size);
      Arg<bool> retval = ImGui::IsRectVisible(size.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetTime(lua_State* L) {
      Arg<double,lua_Number> retval = ImGui::GetTime();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetFrameCount(lua_State* L) {
      Arg<int,lua_Integer> retval = ImGui::GetFrameCount();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetStyleColorName(lua_State* L) {
      static const char* proto = "const char* ImGui::GetStyleColorName(ImGuiCol idx)";
      Arg<int,lua_Integer> idx(L,1);
      LUAWRAP_CHECK_ARGS(idx);
      Arg<const char*> retval = ImGui::GetStyleColorName(idx.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetStateStorage(lua_State* L) {
      static const char* proto = "void ImGui::SetStateStorage(ImGuiStorage* storage)";
      Arg<ImGuiStorage*> storage(L,1);
      LUAWRAP_CHECK_ARGS(storage);
      ImGui::SetStateStorage(storage.value);
      return 0;
   }

   static int GetStateStorage(lua_State* L) {
      Arg<ImGuiStorage*> retval = ImGui::GetStateStorage();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int CalcTextSize(lua_State* L) {
      static const char* proto = "ImVec2 ImGui::CalcTextSize(const char* text, const char* text_end=NULL, bool hide_text_after_double_hash=false, float wrap_width=-1.0)";
      Arg<const char*> text(L,1);
      Arg<const char*> text_end(L,2,NULL);
      Arg<bool> hide_text_after_double_hash(L,3,false);
      Arg<float,lua_Number> wrap_width(L,4,-1.0);
      LUAWRAP_CHECK_ARGS(text, text_end, hide_text_after_double_hash, wrap_width);
      Arg<ImVec2> retval = ImGui::CalcTextSize(text.value, text_end.value, hide_text_after_double_hash.value, wrap_width.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int ColorConvertU32ToFloat4(lua_State* L) {
      static const char* proto = "ImVec4 ImGui::ColorConvertU32ToFloat4(ImU32 in)";
      Arg<unsigned int,lua_Integer> in(L,1);
      LUAWRAP_CHECK_ARGS(in);
      Arg<ImVec4> retval = ImGui::ColorConvertU32ToFloat4(in.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int ColorConvertFloat4ToU32(lua_State* L) {
      static const char* proto = "ImU32 ImGui::ColorConvertFloat4ToU32(ImVec4 in)";
      Arg<ImVec4> in(L,1);
      LUAWRAP_CHECK_ARGS(in);
      Arg<ImU32,lua_Integer> retval = ImGui::ColorConvertFloat4ToU32(in.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsKeyDown(lua_State* L) {
      static const char* proto = "bool ImGui::IsKeyDown(ImGuiKey key)";
      Arg<ImGuiKey,lua_Integer> key(L,1);
      LUAWRAP_CHECK_ARGS(key);
      Arg<bool> retval = ImGui::IsKeyDown(key.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsKeyPressed(lua_State* L) {
      static const char* proto = "bool ImGui::IsKeyPressed(ImGuiKey key, bool repeat=true)";
      Arg<ImGuiKey,lua_Integer> key(L,1);
      Arg<bool> repeat(L,2,true);
      LUAWRAP_CHECK_ARGS(key, repeat);
      Arg<bool> retval = ImGui::IsKeyPressed(key.value, repeat.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsKeyReleased(lua_State* L) {
      static const char* proto = "bool ImGui::IsKeyReleased(ImGuiKey key)";
      Arg<ImGuiKey,lua_Integer> key(L,1);
      LUAWRAP_CHECK_ARGS(key);
      Arg<bool> retval = ImGui::IsKeyReleased(key.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsKeyChordPressed(lua_State* L) {
      static const char* proto = "bool ImGui::IsKeyChordPressed(ImGuiKeyChord key_chord)";
      Arg<int,lua_Integer> key_chord(L,1);
      LUAWRAP_CHECK_ARGS(key_chord);
      Arg<bool> retval = ImGui::IsKeyChordPressed(key_chord.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetKeyPressedAmount(lua_State* L) {
      static const char* proto = "int ImGui::GetKeyPressedAmount(ImGuiKey key, float repeat_delay, float rate)";
      Arg<ImGuiKey,lua_Integer> key(L,1);
      Arg<float,lua_Number> repeat_delay(L,2);
      Arg<float,lua_Number> rate(L,3);
      LUAWRAP_CHECK_ARGS(key, repeat_delay, rate);
      Arg<int,lua_Integer> retval = ImGui::GetKeyPressedAmount(key.value, repeat_delay.value, rate.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetKeyName(lua_State* L) {
      static const char* proto = "const char* ImGui::GetKeyName(ImGuiKey key)";
      Arg<ImGuiKey,lua_Integer> key(L,1);
      LUAWRAP_CHECK_ARGS(key);
      Arg<const char*> retval = ImGui::GetKeyName(key.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetNextFrameWantCaptureKeyboard(lua_State* L) {
      static const char* proto = "void ImGui::SetNextFrameWantCaptureKeyboard(bool want_capture_keyboard)";
      Arg<bool> want_capture_keyboard(L,1);
      LUAWRAP_CHECK_ARGS(want_capture_keyboard);
      ImGui::SetNextFrameWantCaptureKeyboard(want_capture_keyboard.value);
      return 0;
   }

   static int Shortcut(lua_State* L) {
      static const char* proto = "bool ImGui::Shortcut(ImGuiKeyChord key_chord, ImGuiInputFlags flags=0)";
      Arg<int,lua_Integer> key_chord(L,1);
      Arg<int,lua_Integer> flags(L,2,0);
      LUAWRAP_CHECK_ARGS(key_chord, flags);
      Arg<bool> retval = ImGui::Shortcut(key_chord.value, flags.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetNextItemShortcut(lua_State* L) {
      static const char* proto = "void ImGui::SetNextItemShortcut(ImGuiKeyChord key_chord, ImGuiInputFlags flags=0)";
      Arg<int,lua_Integer> key_chord(L,1);
      Arg<int,lua_Integer> flags(L,2,0);
      LUAWRAP_CHECK_ARGS(key_chord, flags);
      ImGui::SetNextItemShortcut(key_chord.value, flags.value);
      return 0;
   }

   static int SetItemKeyOwner(lua_State* L) {
      static const char* proto = "void ImGui::SetItemKeyOwner(ImGuiKey key)";
      Arg<ImGuiKey,lua_Integer> key(L,1);
      LUAWRAP_CHECK_ARGS(key);
      ImGui::SetItemKeyOwner(key.value);
      return 0;
   }

   static int IsMouseDown(lua_State* L) {
      static const char* proto = "bool ImGui::IsMouseDown(ImGuiMouseButton button)";
      Arg<int,lua_Integer> button(L,1);
      LUAWRAP_CHECK_ARGS(button);
      Arg<bool> retval = ImGui::IsMouseDown(button.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsMouseClicked(lua_State* L) {
      static const char* proto = "bool ImGui::IsMouseClicked(ImGuiMouseButton button, bool repeat=false)";
      Arg<int,lua_Integer> button(L,1);
      Arg<bool> repeat(L,2,false);
      LUAWRAP_CHECK_ARGS(button, repeat);
      Arg<bool> retval = ImGui::IsMouseClicked(button.value, repeat.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsMouseReleased(lua_State* L) {
      static const char* proto = "bool ImGui::IsMouseReleased(ImGuiMouseButton button)";
      Arg<int,lua_Integer> button(L,1);
      LUAWRAP_CHECK_ARGS(button);
      Arg<bool> retval = ImGui::IsMouseReleased(button.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsMouseDoubleClicked(lua_State* L) {
      static const char* proto = "bool ImGui::IsMouseDoubleClicked(ImGuiMouseButton button)";
      Arg<int,lua_Integer> button(L,1);
      LUAWRAP_CHECK_ARGS(button);
      Arg<bool> retval = ImGui::IsMouseDoubleClicked(button.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsMouseReleasedWithDelay(lua_State* L) {
      static const char* proto = "bool ImGui::IsMouseReleasedWithDelay(ImGuiMouseButton button, float delay)";
      Arg<int,lua_Integer> button(L,1);
      Arg<float,lua_Number> delay(L,2);
      LUAWRAP_CHECK_ARGS(button, delay);
      Arg<bool> retval = ImGui::IsMouseReleasedWithDelay(button.value, delay.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetMouseClickedCount(lua_State* L) {
      static const char* proto = "int ImGui::GetMouseClickedCount(ImGuiMouseButton button)";
      Arg<int,lua_Integer> button(L,1);
      LUAWRAP_CHECK_ARGS(button);
      Arg<int,lua_Integer> retval = ImGui::GetMouseClickedCount(button.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsMouseHoveringRect(lua_State* L) {
      static const char* proto = "bool ImGui::IsMouseHoveringRect(ImVec2 r_min, ImVec2 r_max, bool clip=true)";
      Arg<ImVec2> r_min(L,1);
      Arg<ImVec2> r_max(L,3);
      Arg<bool> clip(L,5,true);
      LUAWRAP_CHECK_ARGS(r_min, r_max, clip);
      Arg<bool> retval = ImGui::IsMouseHoveringRect(r_min.value, r_max.value, clip.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsAnyMouseDown(lua_State* L) {
      Arg<bool> retval = ImGui::IsAnyMouseDown();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetMousePos(lua_State* L) {
      Arg<ImVec2> retval = ImGui::GetMousePos();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetMousePosOnOpeningCurrentPopup(lua_State* L) {
      Arg<ImVec2> retval = ImGui::GetMousePosOnOpeningCurrentPopup();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int IsMouseDragging(lua_State* L) {
      static const char* proto = "bool ImGui::IsMouseDragging(ImGuiMouseButton button, float lock_threshold=-1.0)";
      Arg<int,lua_Integer> button(L,1);
      Arg<float,lua_Number> lock_threshold(L,2,-1.0);
      LUAWRAP_CHECK_ARGS(button, lock_threshold);
      Arg<bool> retval = ImGui::IsMouseDragging(button.value, lock_threshold.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetMouseDragDelta(lua_State* L) {
      static const char* proto = "ImVec2 ImGui::GetMouseDragDelta(ImGuiMouseButton button=0, float lock_threshold=-1.0)";
      Arg<int,lua_Integer> button(L,1,0);
      Arg<float,lua_Number> lock_threshold(L,2,-1.0);
      LUAWRAP_CHECK_ARGS(button, lock_threshold);
      Arg<ImVec2> retval = ImGui::GetMouseDragDelta(button.value, lock_threshold.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int ResetMouseDragDelta(lua_State* L) {
      static const char* proto = "void ImGui::ResetMouseDragDelta(ImGuiMouseButton button=0)";
      Arg<int,lua_Integer> button(L,1,0);
      LUAWRAP_CHECK_ARGS(button);
      ImGui::ResetMouseDragDelta(button.value);
      return 0;
   }

   static int GetMouseCursor(lua_State* L) {
      Arg<ImGuiMouseCursor,lua_Integer> retval = ImGui::GetMouseCursor();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetMouseCursor(lua_State* L) {
      static const char* proto = "void ImGui::SetMouseCursor(ImGuiMouseCursor cursor_type)";
      Arg<int,lua_Integer> cursor_type(L,1);
      LUAWRAP_CHECK_ARGS(cursor_type);
      ImGui::SetMouseCursor(cursor_type.value);
      return 0;
   }

   static int SetNextFrameWantCaptureMouse(lua_State* L) {
      static const char* proto = "void ImGui::SetNextFrameWantCaptureMouse(bool want_capture_mouse)";
      Arg<bool> want_capture_mouse(L,1);
      LUAWRAP_CHECK_ARGS(want_capture_mouse);
      ImGui::SetNextFrameWantCaptureMouse(want_capture_mouse.value);
      return 0;
   }

   static int GetClipboardText(lua_State* L) {
      Arg<const char*> retval = ImGui::GetClipboardText();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int SetClipboardText(lua_State* L) {
      static const char* proto = "void ImGui::SetClipboardText(const char* text)";
      Arg<const char*> text(L,1);
      LUAWRAP_CHECK_ARGS(text);
      ImGui::SetClipboardText(text.value);
      return 0;
   }

   static int LoadIniSettingsFromDisk(lua_State* L) {
      static const char* proto = "void ImGui::LoadIniSettingsFromDisk(const char* ini_filename)";
      Arg<const char*> ini_filename(L,1);
      LUAWRAP_CHECK_ARGS(ini_filename);
      ImGui::LoadIniSettingsFromDisk(ini_filename.value);
      return 0;
   }

   static int LoadIniSettingsFromMemory(lua_State* L) {
      static const char* proto = "void ImGui::LoadIniSettingsFromMemory(const char* ini_data, size_t ini_size=0)";
      Arg<const char*> ini_data(L,1);
      Arg<unsigned long,lua_Integer> ini_size(L,2,0);
      LUAWRAP_CHECK_ARGS(ini_data, ini_size);
      ImGui::LoadIniSettingsFromMemory(ini_data.value, ini_size.value);
      return 0;
   }

   static int SaveIniSettingsToDisk(lua_State* L) {
      static const char* proto = "void ImGui::SaveIniSettingsToDisk(const char* ini_filename)";
      Arg<const char*> ini_filename(L,1);
      LUAWRAP_CHECK_ARGS(ini_filename);
      ImGui::SaveIniSettingsToDisk(ini_filename.value);
      return 0;
   }

   static int DebugTextEncoding(lua_State* L) {
      static const char* proto = "void ImGui::DebugTextEncoding(const char* text)";
      Arg<const char*> text(L,1);
      LUAWRAP_CHECK_ARGS(text);
      ImGui::DebugTextEncoding(text.value);
      return 0;
   }

   static int DebugFlashStyleColor(lua_State* L) {
      static const char* proto = "void ImGui::DebugFlashStyleColor(ImGuiCol idx)";
      Arg<int,lua_Integer> idx(L,1);
      LUAWRAP_CHECK_ARGS(idx);
      ImGui::DebugFlashStyleColor(idx.value);
      return 0;
   }

   static int DebugStartItemPicker(lua_State*) {
      ImGui::DebugStartItemPicker();
      return 0;
   }

   static int DebugCheckVersionAndDataLayout(lua_State* L) {
      static const char* proto = "bool ImGui::DebugCheckVersionAndDataLayout(const char* version_str, size_t sz_io, size_t sz_style, size_t sz_vec2, size_t sz_vec4, size_t sz_drawvert, size_t sz_drawidx)";
      Arg<const char*> version_str(L,1);
      Arg<unsigned long,lua_Integer> sz_io(L,2);
      Arg<unsigned long,lua_Integer> sz_style(L,3);
      Arg<unsigned long,lua_Integer> sz_vec2(L,4);
      Arg<unsigned long,lua_Integer> sz_vec4(L,5);
      Arg<unsigned long,lua_Integer> sz_drawvert(L,6);
      Arg<unsigned long,lua_Integer> sz_drawidx(L,7);
      LUAWRAP_CHECK_ARGS(version_str, sz_io, sz_style, sz_vec2, sz_vec4, sz_drawvert, sz_drawidx);
      Arg<bool> retval = ImGui::DebugCheckVersionAndDataLayout(version_str.value, sz_io.value, sz_style.value, sz_vec2.value, sz_vec4.value, sz_drawvert.value, sz_drawidx.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int UpdatePlatformWindows(lua_State*) {
      ImGui::UpdatePlatformWindows();
      return 0;
   }

   static int DestroyPlatformWindows(lua_State*) {
      ImGui::DestroyPlatformWindows();
      return 0;
   }

   static int FindViewportByID(lua_State* L) {
      static const char* proto = "ImGuiViewport* ImGui::FindViewportByID(ImGuiID viewport_id)";
      Arg<unsigned int,lua_Integer> viewport_id(L,1);
      LUAWRAP_CHECK_ARGS(viewport_id);
      Arg<ImGuiViewport*> retval = ImGui::FindViewportByID(viewport_id.value);
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

} // namespace ImGui_lua_wrappers_wrappers


void ImGui_lua_wrappers_register(lua_State* L) {
   lua_getglobal(L, "imgui");
   if(lua_isnil(L,-1)) {
      lua_pop(L,1); 
      lua_newtable(L); 
      lua_pushvalue(L, -1);
      lua_setglobal(L, "imgui");
   } 

   using namespace ImGui_lua_wrappers;
   LUAWRAP_DECLARE_FUNCTION(L,NewFrame);
   LUAWRAP_DECLARE_FUNCTION(L,EndFrame);
   LUAWRAP_DECLARE_FUNCTION(L,Render);
   LUAWRAP_DECLARE_FUNCTION(L,GetDrawData);
   LUAWRAP_DECLARE_FUNCTION(L,ShowDemoWindow);
   LUAWRAP_DECLARE_FUNCTION(L,ShowMetricsWindow);
   LUAWRAP_DECLARE_FUNCTION(L,ShowDebugLogWindow);
   LUAWRAP_DECLARE_FUNCTION(L,ShowIDStackToolWindow);
   LUAWRAP_DECLARE_FUNCTION(L,ShowAboutWindow);
   LUAWRAP_DECLARE_FUNCTION(L,ShowStyleEditor);
   LUAWRAP_DECLARE_FUNCTION(L,ShowStyleSelector);
   LUAWRAP_DECLARE_FUNCTION(L,ShowFontSelector);
   LUAWRAP_DECLARE_FUNCTION(L,ShowUserGuide);
   LUAWRAP_DECLARE_FUNCTION(L,GetVersion);
   LUAWRAP_DECLARE_FUNCTION(L,StyleColorsDark);
   LUAWRAP_DECLARE_FUNCTION(L,StyleColorsLight);
   LUAWRAP_DECLARE_FUNCTION(L,StyleColorsClassic);
   LUAWRAP_DECLARE_FUNCTION(L,Begin);
   LUAWRAP_DECLARE_FUNCTION(L,End);
   LUAWRAP_DECLARE_FUNCTION(L,BeginChild);
   LUAWRAP_DECLARE_FUNCTION(L,EndChild);
   LUAWRAP_DECLARE_FUNCTION(L,IsWindowAppearing);
   LUAWRAP_DECLARE_FUNCTION(L,IsWindowCollapsed);
   LUAWRAP_DECLARE_FUNCTION(L,IsWindowFocused);
   LUAWRAP_DECLARE_FUNCTION(L,IsWindowHovered);
   LUAWRAP_DECLARE_FUNCTION(L,GetWindowDrawList);
   LUAWRAP_DECLARE_FUNCTION(L,GetWindowDpiScale);
   LUAWRAP_DECLARE_FUNCTION(L,GetWindowPos);
   LUAWRAP_DECLARE_FUNCTION(L,GetWindowSize);
   LUAWRAP_DECLARE_FUNCTION(L,GetWindowWidth);
   LUAWRAP_DECLARE_FUNCTION(L,GetWindowHeight);
   LUAWRAP_DECLARE_FUNCTION(L,GetWindowViewport);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextWindowPos);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextWindowSize);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextWindowContentSize);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextWindowCollapsed);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextWindowFocus);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextWindowScroll);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextWindowBgAlpha);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextWindowViewport);
   LUAWRAP_DECLARE_FUNCTION(L,SetWindowPos);
   LUAWRAP_DECLARE_FUNCTION(L,SetWindowSize);
   LUAWRAP_DECLARE_FUNCTION(L,SetWindowCollapsed);
   LUAWRAP_DECLARE_FUNCTION(L,SetWindowFocus);
   LUAWRAP_DECLARE_FUNCTION(L,GetScrollX);
   LUAWRAP_DECLARE_FUNCTION(L,GetScrollY);
   LUAWRAP_DECLARE_FUNCTION(L,SetScrollX);
   LUAWRAP_DECLARE_FUNCTION(L,SetScrollY);
   LUAWRAP_DECLARE_FUNCTION(L,GetScrollMaxX);
   LUAWRAP_DECLARE_FUNCTION(L,GetScrollMaxY);
   LUAWRAP_DECLARE_FUNCTION(L,SetScrollHereX);
   LUAWRAP_DECLARE_FUNCTION(L,SetScrollHereY);
   LUAWRAP_DECLARE_FUNCTION(L,SetScrollFromPosX);
   LUAWRAP_DECLARE_FUNCTION(L,SetScrollFromPosY);
   LUAWRAP_DECLARE_FUNCTION(L,PushFont);
   LUAWRAP_DECLARE_FUNCTION(L,PopFont);
   LUAWRAP_DECLARE_FUNCTION(L,GetFont);
   LUAWRAP_DECLARE_FUNCTION(L,GetFontSize);
   LUAWRAP_DECLARE_FUNCTION(L,GetFontBaked);
   LUAWRAP_DECLARE_FUNCTION(L,PushStyleColor);
   LUAWRAP_DECLARE_FUNCTION(L,PopStyleColor);
   LUAWRAP_DECLARE_FUNCTION(L,PushStyleVar);
   LUAWRAP_DECLARE_FUNCTION(L,PushStyleVarX);
   LUAWRAP_DECLARE_FUNCTION(L,PushStyleVarY);
   LUAWRAP_DECLARE_FUNCTION(L,PopStyleVar);
   LUAWRAP_DECLARE_FUNCTION(L,PushItemFlag);
   LUAWRAP_DECLARE_FUNCTION(L,PopItemFlag);
   LUAWRAP_DECLARE_FUNCTION(L,PushItemWidth);
   LUAWRAP_DECLARE_FUNCTION(L,PopItemWidth);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextItemWidth);
   LUAWRAP_DECLARE_FUNCTION(L,CalcItemWidth);
   LUAWRAP_DECLARE_FUNCTION(L,PushTextWrapPos);
   LUAWRAP_DECLARE_FUNCTION(L,PopTextWrapPos);
   LUAWRAP_DECLARE_FUNCTION(L,GetFontTexUvWhitePixel);
   LUAWRAP_DECLARE_FUNCTION(L,GetColorU32);
   LUAWRAP_DECLARE_FUNCTION(L,GetStyleColorVec4);
   LUAWRAP_DECLARE_FUNCTION(L,GetCursorScreenPos);
   LUAWRAP_DECLARE_FUNCTION(L,SetCursorScreenPos);
   LUAWRAP_DECLARE_FUNCTION(L,GetContentRegionAvail);
   LUAWRAP_DECLARE_FUNCTION(L,GetCursorPos);
   LUAWRAP_DECLARE_FUNCTION(L,GetCursorPosX);
   LUAWRAP_DECLARE_FUNCTION(L,GetCursorPosY);
   LUAWRAP_DECLARE_FUNCTION(L,SetCursorPos);
   LUAWRAP_DECLARE_FUNCTION(L,SetCursorPosX);
   LUAWRAP_DECLARE_FUNCTION(L,SetCursorPosY);
   LUAWRAP_DECLARE_FUNCTION(L,GetCursorStartPos);
   LUAWRAP_DECLARE_FUNCTION(L,Separator);
   LUAWRAP_DECLARE_FUNCTION(L,SameLine);
   LUAWRAP_DECLARE_FUNCTION(L,NewLine);
   LUAWRAP_DECLARE_FUNCTION(L,Spacing);
   LUAWRAP_DECLARE_FUNCTION(L,Dummy);
   LUAWRAP_DECLARE_FUNCTION(L,Indent);
   LUAWRAP_DECLARE_FUNCTION(L,Unindent);
   LUAWRAP_DECLARE_FUNCTION(L,BeginGroup);
   LUAWRAP_DECLARE_FUNCTION(L,EndGroup);
   LUAWRAP_DECLARE_FUNCTION(L,AlignTextToFramePadding);
   LUAWRAP_DECLARE_FUNCTION(L,GetTextLineHeight);
   LUAWRAP_DECLARE_FUNCTION(L,GetTextLineHeightWithSpacing);
   LUAWRAP_DECLARE_FUNCTION(L,GetFrameHeight);
   LUAWRAP_DECLARE_FUNCTION(L,GetFrameHeightWithSpacing);
   LUAWRAP_DECLARE_FUNCTION(L,PushID);
   LUAWRAP_DECLARE_FUNCTION(L,PopID);
   LUAWRAP_DECLARE_FUNCTION(L,GetID);
   LUAWRAP_DECLARE_FUNCTION(L,TextUnformatted);
   LUAWRAP_DECLARE_FUNCTION(L,SeparatorText);
   LUAWRAP_DECLARE_FUNCTION(L,Button);
   LUAWRAP_DECLARE_FUNCTION(L,SmallButton);
   LUAWRAP_DECLARE_FUNCTION(L,InvisibleButton);
   LUAWRAP_DECLARE_FUNCTION(L,ArrowButton);
   LUAWRAP_DECLARE_FUNCTION(L,Checkbox);
   LUAWRAP_DECLARE_FUNCTION(L,CheckboxFlags);
   LUAWRAP_DECLARE_FUNCTION(L,RadioButton);
   LUAWRAP_DECLARE_FUNCTION(L,ProgressBar);
   LUAWRAP_DECLARE_FUNCTION(L,Bullet);
   LUAWRAP_DECLARE_FUNCTION(L,TextLink);
   LUAWRAP_DECLARE_FUNCTION(L,TextLinkOpenURL);
   LUAWRAP_DECLARE_FUNCTION(L,Image);
   LUAWRAP_DECLARE_FUNCTION(L,ImageWithBg);
   LUAWRAP_DECLARE_FUNCTION(L,ImageButton);
   LUAWRAP_DECLARE_FUNCTION(L,BeginCombo);
   LUAWRAP_DECLARE_FUNCTION(L,EndCombo);
   LUAWRAP_DECLARE_FUNCTION(L,DragFloat);
   LUAWRAP_DECLARE_FUNCTION(L,DragFloatRange2);
   LUAWRAP_DECLARE_FUNCTION(L,DragInt);
   LUAWRAP_DECLARE_FUNCTION(L,DragIntRange2);
   LUAWRAP_DECLARE_FUNCTION(L,SliderFloat);
   LUAWRAP_DECLARE_FUNCTION(L,SliderAngle);
   LUAWRAP_DECLARE_FUNCTION(L,SliderInt);
   LUAWRAP_DECLARE_FUNCTION(L,VSliderFloat);
   LUAWRAP_DECLARE_FUNCTION(L,VSliderInt);
   LUAWRAP_DECLARE_FUNCTION(L,InputFloat);
   LUAWRAP_DECLARE_FUNCTION(L,InputInt);
   LUAWRAP_DECLARE_FUNCTION(L,InputDouble);
   LUAWRAP_DECLARE_FUNCTION(L,ColorButton);
   LUAWRAP_DECLARE_FUNCTION(L,SetColorEditOptions);
   LUAWRAP_DECLARE_FUNCTION(L,TreeNode);
   LUAWRAP_DECLARE_FUNCTION(L,TreeNodeEx);
   LUAWRAP_DECLARE_FUNCTION(L,TreePush);
   LUAWRAP_DECLARE_FUNCTION(L,TreePop);
   LUAWRAP_DECLARE_FUNCTION(L,GetTreeNodeToLabelSpacing);
   LUAWRAP_DECLARE_FUNCTION(L,CollapsingHeader);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextItemOpen);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextItemStorageID);
   LUAWRAP_DECLARE_FUNCTION(L,Selectable);
   LUAWRAP_DECLARE_FUNCTION(L,BeginMultiSelect);
   LUAWRAP_DECLARE_FUNCTION(L,EndMultiSelect);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextItemSelectionUserData);
   LUAWRAP_DECLARE_FUNCTION(L,IsItemToggledSelection);
   LUAWRAP_DECLARE_FUNCTION(L,BeginListBox);
   LUAWRAP_DECLARE_FUNCTION(L,EndListBox);
   LUAWRAP_DECLARE_FUNCTION(L,PlotLines);
   LUAWRAP_DECLARE_FUNCTION(L,PlotHistogram);
   LUAWRAP_DECLARE_FUNCTION(L,Value);
   LUAWRAP_DECLARE_FUNCTION(L,BeginMenuBar);
   LUAWRAP_DECLARE_FUNCTION(L,EndMenuBar);
   LUAWRAP_DECLARE_FUNCTION(L,BeginMainMenuBar);
   LUAWRAP_DECLARE_FUNCTION(L,EndMainMenuBar);
   LUAWRAP_DECLARE_FUNCTION(L,BeginMenu);
   LUAWRAP_DECLARE_FUNCTION(L,EndMenu);
   LUAWRAP_DECLARE_FUNCTION(L,MenuItem);
   LUAWRAP_DECLARE_FUNCTION(L,BeginTooltip);
   LUAWRAP_DECLARE_FUNCTION(L,EndTooltip);
   LUAWRAP_DECLARE_FUNCTION(L,BeginItemTooltip);
   LUAWRAP_DECLARE_FUNCTION(L,BeginPopup);
   LUAWRAP_DECLARE_FUNCTION(L,BeginPopupModal);
   LUAWRAP_DECLARE_FUNCTION(L,EndPopup);
   LUAWRAP_DECLARE_FUNCTION(L,OpenPopup);
   LUAWRAP_DECLARE_FUNCTION(L,OpenPopupOnItemClick);
   LUAWRAP_DECLARE_FUNCTION(L,CloseCurrentPopup);
   LUAWRAP_DECLARE_FUNCTION(L,BeginPopupContextItem);
   LUAWRAP_DECLARE_FUNCTION(L,BeginPopupContextWindow);
   LUAWRAP_DECLARE_FUNCTION(L,BeginPopupContextVoid);
   LUAWRAP_DECLARE_FUNCTION(L,IsPopupOpen);
   LUAWRAP_DECLARE_FUNCTION(L,BeginTable);
   LUAWRAP_DECLARE_FUNCTION(L,EndTable);
   LUAWRAP_DECLARE_FUNCTION(L,TableNextRow);
   LUAWRAP_DECLARE_FUNCTION(L,TableNextColumn);
   LUAWRAP_DECLARE_FUNCTION(L,TableSetColumnIndex);
   LUAWRAP_DECLARE_FUNCTION(L,TableSetupColumn);
   LUAWRAP_DECLARE_FUNCTION(L,TableSetupScrollFreeze);
   LUAWRAP_DECLARE_FUNCTION(L,TableHeader);
   LUAWRAP_DECLARE_FUNCTION(L,TableHeadersRow);
   LUAWRAP_DECLARE_FUNCTION(L,TableAngledHeadersRow);
   LUAWRAP_DECLARE_FUNCTION(L,TableGetSortSpecs);
   LUAWRAP_DECLARE_FUNCTION(L,TableGetColumnCount);
   LUAWRAP_DECLARE_FUNCTION(L,TableGetColumnIndex);
   LUAWRAP_DECLARE_FUNCTION(L,TableGetRowIndex);
   LUAWRAP_DECLARE_FUNCTION(L,TableGetColumnName);
   LUAWRAP_DECLARE_FUNCTION(L,TableGetColumnFlags);
   LUAWRAP_DECLARE_FUNCTION(L,TableSetColumnEnabled);
   LUAWRAP_DECLARE_FUNCTION(L,TableGetHoveredColumn);
   LUAWRAP_DECLARE_FUNCTION(L,TableSetBgColor);
   LUAWRAP_DECLARE_FUNCTION(L,Columns);
   LUAWRAP_DECLARE_FUNCTION(L,NextColumn);
   LUAWRAP_DECLARE_FUNCTION(L,GetColumnIndex);
   LUAWRAP_DECLARE_FUNCTION(L,GetColumnWidth);
   LUAWRAP_DECLARE_FUNCTION(L,SetColumnWidth);
   LUAWRAP_DECLARE_FUNCTION(L,GetColumnOffset);
   LUAWRAP_DECLARE_FUNCTION(L,SetColumnOffset);
   LUAWRAP_DECLARE_FUNCTION(L,GetColumnsCount);
   LUAWRAP_DECLARE_FUNCTION(L,BeginTabBar);
   LUAWRAP_DECLARE_FUNCTION(L,EndTabBar);
   LUAWRAP_DECLARE_FUNCTION(L,BeginTabItem);
   LUAWRAP_DECLARE_FUNCTION(L,EndTabItem);
   LUAWRAP_DECLARE_FUNCTION(L,TabItemButton);
   LUAWRAP_DECLARE_FUNCTION(L,SetTabItemClosed);
   LUAWRAP_DECLARE_FUNCTION(L,DockSpace);
   LUAWRAP_DECLARE_FUNCTION(L,DockSpaceOverViewport);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextWindowDockID);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextWindowClass);
   LUAWRAP_DECLARE_FUNCTION(L,GetWindowDockID);
   LUAWRAP_DECLARE_FUNCTION(L,IsWindowDocked);
   LUAWRAP_DECLARE_FUNCTION(L,LogToTTY);
   LUAWRAP_DECLARE_FUNCTION(L,LogToFile);
   LUAWRAP_DECLARE_FUNCTION(L,LogToClipboard);
   LUAWRAP_DECLARE_FUNCTION(L,LogFinish);
   LUAWRAP_DECLARE_FUNCTION(L,LogButtons);
   LUAWRAP_DECLARE_FUNCTION(L,BeginDragDropSource);
   LUAWRAP_DECLARE_FUNCTION(L,EndDragDropSource);
   LUAWRAP_DECLARE_FUNCTION(L,BeginDragDropTarget);
   LUAWRAP_DECLARE_FUNCTION(L,AcceptDragDropPayload);
   LUAWRAP_DECLARE_FUNCTION(L,EndDragDropTarget);
   LUAWRAP_DECLARE_FUNCTION(L,GetDragDropPayload);
   LUAWRAP_DECLARE_FUNCTION(L,BeginDisabled);
   LUAWRAP_DECLARE_FUNCTION(L,EndDisabled);
   LUAWRAP_DECLARE_FUNCTION(L,PushClipRect);
   LUAWRAP_DECLARE_FUNCTION(L,PopClipRect);
   LUAWRAP_DECLARE_FUNCTION(L,SetItemDefaultFocus);
   LUAWRAP_DECLARE_FUNCTION(L,SetKeyboardFocusHere);
   LUAWRAP_DECLARE_FUNCTION(L,SetNavCursorVisible);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextItemAllowOverlap);
   LUAWRAP_DECLARE_FUNCTION(L,IsItemHovered);
   LUAWRAP_DECLARE_FUNCTION(L,IsItemActive);
   LUAWRAP_DECLARE_FUNCTION(L,IsItemFocused);
   LUAWRAP_DECLARE_FUNCTION(L,IsItemClicked);
   LUAWRAP_DECLARE_FUNCTION(L,IsItemVisible);
   LUAWRAP_DECLARE_FUNCTION(L,IsItemEdited);
   LUAWRAP_DECLARE_FUNCTION(L,IsItemActivated);
   LUAWRAP_DECLARE_FUNCTION(L,IsItemDeactivated);
   LUAWRAP_DECLARE_FUNCTION(L,IsItemDeactivatedAfterEdit);
   LUAWRAP_DECLARE_FUNCTION(L,IsItemToggledOpen);
   LUAWRAP_DECLARE_FUNCTION(L,IsAnyItemHovered);
   LUAWRAP_DECLARE_FUNCTION(L,IsAnyItemActive);
   LUAWRAP_DECLARE_FUNCTION(L,IsAnyItemFocused);
   LUAWRAP_DECLARE_FUNCTION(L,GetItemID);
   LUAWRAP_DECLARE_FUNCTION(L,GetItemRectMin);
   LUAWRAP_DECLARE_FUNCTION(L,GetItemRectMax);
   LUAWRAP_DECLARE_FUNCTION(L,GetItemRectSize);
   LUAWRAP_DECLARE_FUNCTION(L,GetItemFlags);
   LUAWRAP_DECLARE_FUNCTION(L,GetMainViewport);
   LUAWRAP_DECLARE_FUNCTION(L,GetBackgroundDrawList);
   LUAWRAP_DECLARE_FUNCTION(L,GetForegroundDrawList);
   LUAWRAP_DECLARE_FUNCTION(L,IsRectVisible);
   LUAWRAP_DECLARE_FUNCTION(L,GetTime);
   LUAWRAP_DECLARE_FUNCTION(L,GetFrameCount);
   LUAWRAP_DECLARE_FUNCTION(L,GetStyleColorName);
   LUAWRAP_DECLARE_FUNCTION(L,SetStateStorage);
   LUAWRAP_DECLARE_FUNCTION(L,GetStateStorage);
   LUAWRAP_DECLARE_FUNCTION(L,CalcTextSize);
   LUAWRAP_DECLARE_FUNCTION(L,ColorConvertU32ToFloat4);
   LUAWRAP_DECLARE_FUNCTION(L,ColorConvertFloat4ToU32);
   LUAWRAP_DECLARE_FUNCTION(L,IsKeyDown);
   LUAWRAP_DECLARE_FUNCTION(L,IsKeyPressed);
   LUAWRAP_DECLARE_FUNCTION(L,IsKeyReleased);
   LUAWRAP_DECLARE_FUNCTION(L,IsKeyChordPressed);
   LUAWRAP_DECLARE_FUNCTION(L,GetKeyPressedAmount);
   LUAWRAP_DECLARE_FUNCTION(L,GetKeyName);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextFrameWantCaptureKeyboard);
   LUAWRAP_DECLARE_FUNCTION(L,Shortcut);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextItemShortcut);
   LUAWRAP_DECLARE_FUNCTION(L,SetItemKeyOwner);
   LUAWRAP_DECLARE_FUNCTION(L,IsMouseDown);
   LUAWRAP_DECLARE_FUNCTION(L,IsMouseClicked);
   LUAWRAP_DECLARE_FUNCTION(L,IsMouseReleased);
   LUAWRAP_DECLARE_FUNCTION(L,IsMouseDoubleClicked);
   LUAWRAP_DECLARE_FUNCTION(L,IsMouseReleasedWithDelay);
   LUAWRAP_DECLARE_FUNCTION(L,GetMouseClickedCount);
   LUAWRAP_DECLARE_FUNCTION(L,IsMouseHoveringRect);
   LUAWRAP_DECLARE_FUNCTION(L,IsAnyMouseDown);
   LUAWRAP_DECLARE_FUNCTION(L,GetMousePos);
   LUAWRAP_DECLARE_FUNCTION(L,GetMousePosOnOpeningCurrentPopup);
   LUAWRAP_DECLARE_FUNCTION(L,IsMouseDragging);
   LUAWRAP_DECLARE_FUNCTION(L,GetMouseDragDelta);
   LUAWRAP_DECLARE_FUNCTION(L,ResetMouseDragDelta);
   LUAWRAP_DECLARE_FUNCTION(L,GetMouseCursor);
   LUAWRAP_DECLARE_FUNCTION(L,SetMouseCursor);
   LUAWRAP_DECLARE_FUNCTION(L,SetNextFrameWantCaptureMouse);
   LUAWRAP_DECLARE_FUNCTION(L,GetClipboardText);
   LUAWRAP_DECLARE_FUNCTION(L,SetClipboardText);
   LUAWRAP_DECLARE_FUNCTION(L,LoadIniSettingsFromDisk);
   LUAWRAP_DECLARE_FUNCTION(L,LoadIniSettingsFromMemory);
   LUAWRAP_DECLARE_FUNCTION(L,SaveIniSettingsToDisk);
   LUAWRAP_DECLARE_FUNCTION(L,DebugTextEncoding);
   LUAWRAP_DECLARE_FUNCTION(L,DebugFlashStyleColor);
   LUAWRAP_DECLARE_FUNCTION(L,DebugStartItemPicker);
   LUAWRAP_DECLARE_FUNCTION(L,DebugCheckVersionAndDataLayout);
   LUAWRAP_DECLARE_FUNCTION(L,UpdatePlatformWindows);
   LUAWRAP_DECLARE_FUNCTION(L,DestroyPlatformWindows);
   LUAWRAP_DECLARE_FUNCTION(L,FindViewportByID);
   lua_pop(L,1);

   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoTitleBar);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoResize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoMove);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoScrollbar);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoScrollWithMouse);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoCollapse);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_AlwaysAutoResize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoBackground);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoSavedSettings);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoMouseInputs);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_MenuBar);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_HorizontalScrollbar);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoFocusOnAppearing);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoBringToFrontOnFocus);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_AlwaysVerticalScrollbar);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_AlwaysHorizontalScrollbar);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoNavInputs);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoNavFocus);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_UnsavedDocument);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoDocking);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoNav);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoDecoration);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_NoInputs);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_DockNodeHost);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_ChildWindow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_Tooltip);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_Popup);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_Modal);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiWindowFlags_ChildMenu);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiChildFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiChildFlags_Borders);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiChildFlags_AlwaysUseWindowPadding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiChildFlags_ResizeX);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiChildFlags_ResizeY);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiChildFlags_AutoResizeX);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiChildFlags_AutoResizeY);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiChildFlags_AlwaysAutoResize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiChildFlags_FrameStyle);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiChildFlags_NavFlattened);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiItemFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiItemFlags_NoTabStop);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiItemFlags_NoNav);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiItemFlags_NoNavDefaultFocus);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiItemFlags_ButtonRepeat);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiItemFlags_AutoClosePopups);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiItemFlags_AllowDuplicateId);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiItemFlags_Disabled);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_CharsDecimal);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_CharsHexadecimal);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_CharsScientific);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_CharsUppercase);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_CharsNoBlank);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_AllowTabInput);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_EnterReturnsTrue);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_EscapeClearsAll);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_CtrlEnterForNewLine);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_ReadOnly);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_Password);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_AlwaysOverwrite);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_AutoSelectAll);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_ParseEmptyRefVal);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_DisplayEmptyRefVal);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_NoHorizontalScroll);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_NoUndoRedo);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_ElideLeft);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_CallbackCompletion);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_CallbackHistory);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_CallbackAlways);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_CallbackCharFilter);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_CallbackResize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_CallbackEdit);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputTextFlags_WordWrap);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_Selected);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_Framed);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_AllowOverlap);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_NoTreePushOnOpen);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_NoAutoOpenOnLog);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_DefaultOpen);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_OpenOnDoubleClick);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_OpenOnArrow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_Leaf);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_Bullet);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_FramePadding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_SpanAvailWidth);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_SpanFullWidth);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_SpanLabelWidth);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_SpanAllColumns);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_LabelSpanAllColumns);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_NavLeftJumpsToParent);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_CollapsingHeader);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_DrawLinesNone);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_DrawLinesFull);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTreeNodeFlags_DrawLinesToNodes);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiPopupFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiPopupFlags_MouseButtonLeft);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiPopupFlags_MouseButtonRight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiPopupFlags_MouseButtonMiddle);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiPopupFlags_MouseButtonMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiPopupFlags_MouseButtonDefault_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiPopupFlags_NoReopen);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiPopupFlags_NoOpenOverExistingPopup);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiPopupFlags_NoOpenOverItems);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiPopupFlags_AnyPopupId);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiPopupFlags_AnyPopupLevel);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiPopupFlags_AnyPopup);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSelectableFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSelectableFlags_NoAutoClosePopups);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSelectableFlags_SpanAllColumns);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSelectableFlags_AllowDoubleClick);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSelectableFlags_Disabled);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSelectableFlags_AllowOverlap);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSelectableFlags_Highlight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSelectableFlags_SelectOnNav);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiComboFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiComboFlags_PopupAlignLeft);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiComboFlags_HeightSmall);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiComboFlags_HeightRegular);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiComboFlags_HeightLarge);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiComboFlags_HeightLargest);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiComboFlags_NoArrowButton);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiComboFlags_NoPreview);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiComboFlags_WidthFitPreview);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiComboFlags_HeightMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_Reorderable);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_AutoSelectNewTabs);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_TabListPopupButton);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_NoCloseWithMiddleMouseButton);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_NoTabListScrollingButtons);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_NoTooltip);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_DrawSelectedOverline);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_FittingPolicyMixed);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_FittingPolicyShrink);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_FittingPolicyScroll);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_FittingPolicyMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabBarFlags_FittingPolicyDefault_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabItemFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabItemFlags_UnsavedDocument);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabItemFlags_SetSelected);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabItemFlags_NoCloseWithMiddleMouseButton);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabItemFlags_NoPushId);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabItemFlags_NoTooltip);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabItemFlags_NoReorder);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabItemFlags_Leading);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabItemFlags_Trailing);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTabItemFlags_NoAssumedClosure);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiFocusedFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiFocusedFlags_ChildWindows);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiFocusedFlags_RootWindow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiFocusedFlags_AnyWindow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiFocusedFlags_NoPopupHierarchy);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiFocusedFlags_DockHierarchy);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiFocusedFlags_RootAndChildWindows);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_ChildWindows);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_RootWindow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_AnyWindow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_NoPopupHierarchy);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_DockHierarchy);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_AllowWhenBlockedByPopup);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_AllowWhenOverlappedByItem);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_AllowWhenOverlappedByWindow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_AllowWhenDisabled);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_NoNavOverride);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_AllowWhenOverlapped);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_RectOnly);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_RootAndChildWindows);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_ForTooltip);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_Stationary);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_DelayNone);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_DelayShort);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_DelayNormal);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiHoveredFlags_NoSharedDelay);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDockNodeFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDockNodeFlags_KeepAliveOnly);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDockNodeFlags_NoDockingOverCentralNode);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDockNodeFlags_PassthruCentralNode);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDockNodeFlags_NoDockingSplit);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDockNodeFlags_NoResize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDockNodeFlags_AutoHideTabBar);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDockNodeFlags_NoUndocking);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_SourceNoPreviewTooltip);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_SourceNoDisableHover);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_SourceNoHoldToOpenOthers);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_SourceAllowNullID);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_SourceExtern);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_PayloadAutoExpire);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_PayloadNoCrossContext);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_PayloadNoCrossProcess);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_AcceptBeforeDelivery);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_AcceptNoDrawDefaultRect);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_AcceptNoPreviewTooltip);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_AcceptDrawAsHovered);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDragDropFlags_AcceptPeekOnly);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDir_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDir_Left);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDir_Right);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDir_Up);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDir_Down);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiDir_COUNT);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_NamedKey_BEGIN);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Tab);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_LeftArrow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_RightArrow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_UpArrow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_DownArrow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_PageUp);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_PageDown);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Home);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_End);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Insert);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Delete);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Backspace);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Space);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Enter);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Escape);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_LeftCtrl);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_LeftShift);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_LeftAlt);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_LeftSuper);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_RightCtrl);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_RightShift);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_RightAlt);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_RightSuper);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Menu);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_0);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_1);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_2);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_3);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_4);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_5);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_6);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_7);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_8);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_9);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_A);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_B);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_C);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_D);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_E);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_G);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_H);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_I);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_J);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_K);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_L);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_M);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_N);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_O);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_P);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Q);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_R);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_S);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_T);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_U);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_V);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_W);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_X);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Y);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Z);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F1);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F2);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F3);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F4);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F5);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F6);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F7);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F8);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F9);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F10);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F11);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F12);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F13);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F14);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F15);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F16);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F17);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F18);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F19);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F20);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F21);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F22);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F23);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_F24);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Apostrophe);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Comma);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Minus);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Period);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Slash);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Semicolon);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Equal);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_LeftBracket);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Backslash);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_RightBracket);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GraveAccent);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_CapsLock);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_ScrollLock);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_NumLock);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_PrintScreen);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Pause);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Keypad0);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Keypad1);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Keypad2);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Keypad3);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Keypad4);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Keypad5);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Keypad6);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Keypad7);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Keypad8);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Keypad9);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_KeypadDecimal);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_KeypadDivide);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_KeypadMultiply);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_KeypadSubtract);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_KeypadAdd);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_KeypadEnter);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_KeypadEqual);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_AppBack);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_AppForward);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_Oem102);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadStart);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadBack);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadFaceLeft);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadFaceRight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadFaceUp);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadFaceDown);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadDpadLeft);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadDpadRight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadDpadUp);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadDpadDown);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadL1);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadR1);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadL2);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadR2);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadL3);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadR3);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadLStickLeft);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadLStickRight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadLStickUp);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadLStickDown);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadRStickLeft);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadRStickRight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadRStickUp);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_GamepadRStickDown);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_MouseLeft);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_MouseRight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_MouseMiddle);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_MouseX1);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_MouseX2);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_MouseWheelX);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_MouseWheelY);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_ReservedForModCtrl);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_ReservedForModShift);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_ReservedForModAlt);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_ReservedForModSuper);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_NamedKey_END);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiKey_NamedKey_COUNT);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMod_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMod_Ctrl);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMod_Shift);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMod_Alt);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMod_Super);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMod_Mask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputFlags_Repeat);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputFlags_RouteActive);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputFlags_RouteFocused);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputFlags_RouteGlobal);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputFlags_RouteAlways);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputFlags_RouteOverFocused);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputFlags_RouteOverActive);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputFlags_RouteUnlessBgFocused);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputFlags_RouteFromRootWindow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiInputFlags_Tooltip);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_Text);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TextDisabled);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_WindowBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_ChildBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_PopupBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_Border);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_BorderShadow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_FrameBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_FrameBgHovered);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_FrameBgActive);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TitleBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TitleBgActive);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TitleBgCollapsed);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_MenuBarBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_ScrollbarBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_ScrollbarGrab);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_ScrollbarGrabHovered);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_ScrollbarGrabActive);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_CheckMark);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_SliderGrab);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_SliderGrabActive);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_Button);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_ButtonHovered);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_ButtonActive);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_Header);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_HeaderHovered);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_HeaderActive);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_Separator);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_SeparatorHovered);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_SeparatorActive);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_ResizeGrip);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_ResizeGripHovered);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_ResizeGripActive);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_InputTextCursor);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TabHovered);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_Tab);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TabSelected);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TabSelectedOverline);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TabDimmed);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TabDimmedSelected);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TabDimmedSelectedOverline);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_DockingPreview);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_DockingEmptyBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_PlotLines);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_PlotLinesHovered);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_PlotHistogram);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_PlotHistogramHovered);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TableHeaderBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TableBorderStrong);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TableBorderLight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TableRowBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TableRowBgAlt);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TextLink);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TextSelectedBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_TreeLines);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_DragDropTarget);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_DragDropTargetBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_UnsavedMarker);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_NavCursor);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_NavWindowingHighlight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_NavWindowingDimBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_ModalWindowDimBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCol_COUNT);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_Alpha);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_DisabledAlpha);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_WindowPadding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_WindowRounding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_WindowBorderSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_WindowMinSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_WindowTitleAlign);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_ChildRounding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_ChildBorderSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_PopupRounding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_PopupBorderSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_FramePadding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_FrameRounding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_FrameBorderSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_ItemSpacing);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_ItemInnerSpacing);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_IndentSpacing);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_CellPadding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_ScrollbarSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_ScrollbarRounding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_ScrollbarPadding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_GrabMinSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_GrabRounding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_ImageBorderSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_TabRounding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_TabBorderSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_TabMinWidthBase);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_TabMinWidthShrink);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_TabBarBorderSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_TabBarOverlineSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_TableAngledHeadersAngle);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_TableAngledHeadersTextAlign);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_TreeLinesSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_TreeLinesRounding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_ButtonTextAlign);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_SelectableTextAlign);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_SeparatorTextBorderSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_SeparatorTextAlign);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_SeparatorTextPadding);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_DockingSeparatorSize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiStyleVar_COUNT);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiButtonFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiButtonFlags_MouseButtonLeft);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiButtonFlags_MouseButtonRight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiButtonFlags_MouseButtonMiddle);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiButtonFlags_MouseButtonMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiButtonFlags_EnableNav);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_NoAlpha);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_NoPicker);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_NoOptions);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_NoSmallPreview);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_NoInputs);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_NoTooltip);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_NoLabel);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_NoSidePreview);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_NoDragDrop);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_NoBorder);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_NoColorMarkers);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_AlphaOpaque);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_AlphaNoBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_AlphaPreviewHalf);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_AlphaBar);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_HDR);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_DisplayRGB);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_DisplayHSV);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_DisplayHex);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_Uint8);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_Float);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_PickerHueBar);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_PickerHueWheel);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_InputRGB);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_InputHSV);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_DefaultOptions_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_AlphaMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_DisplayMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_DataTypeMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_PickerMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiColorEditFlags_InputMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSliderFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSliderFlags_Logarithmic);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSliderFlags_NoRoundToFormat);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSliderFlags_NoInput);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSliderFlags_WrapAround);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSliderFlags_ClampOnInput);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSliderFlags_ClampZeroRange);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSliderFlags_NoSpeedTweaks);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSliderFlags_ColorMarkers);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSliderFlags_AlwaysClamp);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiSliderFlags_InvalidMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseButton_Left);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseButton_Right);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseButton_Middle);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseButton_COUNT);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_Arrow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_TextInput);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_ResizeAll);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_ResizeNS);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_ResizeEW);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_ResizeNESW);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_ResizeNWSE);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_Hand);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_Wait);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_Progress);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_NotAllowed);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMouseCursor_COUNT);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCond_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCond_Always);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCond_Once);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCond_FirstUseEver);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiCond_Appearing);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_Resizable);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_Reorderable);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_Hideable);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_Sortable);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_NoSavedSettings);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_ContextMenuInBody);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_RowBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_BordersInnerH);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_BordersOuterH);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_BordersInnerV);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_BordersOuterV);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_BordersH);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_BordersV);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_BordersInner);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_BordersOuter);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_Borders);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_NoBordersInBody);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_NoBordersInBodyUntilResize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_SizingFixedFit);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_SizingFixedSame);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_SizingStretchProp);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_SizingStretchSame);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_NoHostExtendX);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_NoHostExtendY);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_NoKeepColumnsVisible);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_PreciseWidths);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_NoClip);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_PadOuterX);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_NoPadOuterX);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_NoPadInnerX);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_ScrollX);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_ScrollY);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_SortMulti);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_SortTristate);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_HighlightHoveredColumn);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableFlags_SizingMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_Disabled);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_DefaultHide);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_DefaultSort);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_WidthStretch);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_WidthFixed);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_NoResize);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_NoReorder);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_NoHide);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_NoClip);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_NoSort);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_NoSortAscending);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_NoSortDescending);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_NoHeaderLabel);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_NoHeaderWidth);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_PreferSortAscending);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_PreferSortDescending);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_IndentEnable);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_IndentDisable);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_AngledHeader);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_IsEnabled);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_IsVisible);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_IsSorted);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_IsHovered);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_WidthMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_IndentMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_StatusMask_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableColumnFlags_NoDirectResize_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableRowFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableRowFlags_Headers);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableBgTarget_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableBgTarget_RowBg0);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableBgTarget_RowBg1);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiTableBgTarget_CellBg);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_SingleSelect);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_NoSelectAll);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_NoRangeSelect);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_NoAutoSelect);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_NoAutoClear);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_NoAutoClearOnReselect);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_BoxSelect1d);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_BoxSelect2d);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_BoxSelectNoScroll);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_ClearOnEscape);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_ClearOnClickVoid);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_ScopeWindow);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_ScopeRect);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_SelectOnClick);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_SelectOnClickRelease);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_NavWrapX);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImGuiMultiSelectFlags_NoSelectOnRightClick);
}
