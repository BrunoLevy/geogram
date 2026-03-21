#include "luawrap_runtime.h"

// GOMGEN automatically generated code
// Do not edit.

// Command line:
//  /home/blevy/Programming/GraphiteThree/build/Linux64-gcc-dynamic-Debug/bin/gomgen
//  -oluawrap_imdrawlist.cpp
//  -iimgui.h
//  -sImDrawList
//  -DIMGUI_DISABLE_OBSOLETE_FUNCTIONS
//  -lua


// Include path:
// Input path:
//   /home/blevy/Programming/imgui_gomgen/imgui.h
// Output file:
//   /home/blevy/Programming/imgui_gomgen/luawrap_imdrawlist.cpp


namespace ImDrawList_lua_wrappers {
   using namespace LuaWrap;

   static int PushClipRect(lua_State* L) {
      static const char* proto = "void ImDrawList::PushClipRect(ImVec2 clip_rect_min, ImVec2 clip_rect_max, bool intersect_with_current_clip_rect=false)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> clip_rect_min(L,2);
      Arg<ImVec2> clip_rect_max(L,4);
      Arg<bool> intersect_with_current_clip_rect(L,6,false);
      LUAWRAP_CHECK_ARGS(self, clip_rect_min, clip_rect_max, intersect_with_current_clip_rect);
      self.value->PushClipRect(clip_rect_min.value, clip_rect_max.value, intersect_with_current_clip_rect.value);
      return 0;
   }

   static int PushClipRectFullScreen(lua_State* L) {
      static const char* proto = "void ImDrawList::PushClipRectFullScreen()";
      Arg<ImDrawList*> self(L,1);
      LUAWRAP_CHECK_ARGS(self);
      self.value->PushClipRectFullScreen();
      return 0;
   }

   static int PopClipRect(lua_State* L) {
      static const char* proto = "void ImDrawList::PopClipRect()";
      Arg<ImDrawList*> self(L,1);
      LUAWRAP_CHECK_ARGS(self);
      self.value->PopClipRect();
      return 0;
   }

   static int PushTexture(lua_State* L) {
      static const char* proto = "void ImDrawList::PushTexture(ImTextureRef tex_ref)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImTextureRef> tex_ref(L,2);
      LUAWRAP_CHECK_ARGS(self, tex_ref);
      self.value->PushTexture(tex_ref.value);
      return 0;
   }

   static int PopTexture(lua_State* L) {
      static const char* proto = "void ImDrawList::PopTexture()";
      Arg<ImDrawList*> self(L,1);
      LUAWRAP_CHECK_ARGS(self);
      self.value->PopTexture();
      return 0;
   }

   static int GetClipRectMin(lua_State* L) {
      static const char* proto = "ImVec2 ImDrawList::GetClipRectMin()";
      Arg<ImDrawList*> self(L,1);
      LUAWRAP_CHECK_ARGS(self);
      Arg<ImVec2> retval = self.value->GetClipRectMin();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int GetClipRectMax(lua_State* L) {
      static const char* proto = "ImVec2 ImDrawList::GetClipRectMax()";
      Arg<ImDrawList*> self(L,1);
      LUAWRAP_CHECK_ARGS(self);
      Arg<ImVec2> retval = self.value->GetClipRectMax();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int AddLine(lua_State* L) {
      static const char* proto = "void ImDrawList::AddLine(ImVec2 p1, ImVec2 p2, ImU32 col, float thickness=1.0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> p1(L,2);
      Arg<ImVec2> p2(L,4);
      Arg<unsigned int,lua_Integer> col(L,6);
      Arg<float,lua_Number> thickness(L,7,1.0);
      LUAWRAP_CHECK_ARGS(self, p1, p2, col, thickness);
      self.value->AddLine(p1.value, p2.value, col.value, thickness.value);
      return 0;
   }

   static int AddRect(lua_State* L) {
      static const char* proto = "void ImDrawList::AddRect(ImVec2 p_min, ImVec2 p_max, ImU32 col, float rounding=0.0, ImDrawFlags flags=0, float thickness=1.0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> p_min(L,2);
      Arg<ImVec2> p_max(L,4);
      Arg<unsigned int,lua_Integer> col(L,6);
      Arg<float,lua_Number> rounding(L,7,0.0);
      Arg<int,lua_Integer> flags(L,8,0);
      Arg<float,lua_Number> thickness(L,9,1.0);
      LUAWRAP_CHECK_ARGS(self, p_min, p_max, col, rounding, flags, thickness);
      self.value->AddRect(p_min.value, p_max.value, col.value, rounding.value, flags.value, thickness.value);
      return 0;
   }

   static int AddRectFilled(lua_State* L) {
      static const char* proto = "void ImDrawList::AddRectFilled(ImVec2 p_min, ImVec2 p_max, ImU32 col, float rounding=0.0, ImDrawFlags flags=0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> p_min(L,2);
      Arg<ImVec2> p_max(L,4);
      Arg<unsigned int,lua_Integer> col(L,6);
      Arg<float,lua_Number> rounding(L,7,0.0);
      Arg<int,lua_Integer> flags(L,8,0);
      LUAWRAP_CHECK_ARGS(self, p_min, p_max, col, rounding, flags);
      self.value->AddRectFilled(p_min.value, p_max.value, col.value, rounding.value, flags.value);
      return 0;
   }

   static int AddRectFilledMultiColor(lua_State* L) {
      static const char* proto = "void ImDrawList::AddRectFilledMultiColor(ImVec2 p_min, ImVec2 p_max, ImU32 col_upr_left, ImU32 col_upr_right, ImU32 col_bot_right, ImU32 col_bot_left)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> p_min(L,2);
      Arg<ImVec2> p_max(L,4);
      Arg<unsigned int,lua_Integer> col_upr_left(L,6);
      Arg<unsigned int,lua_Integer> col_upr_right(L,7);
      Arg<unsigned int,lua_Integer> col_bot_right(L,8);
      Arg<unsigned int,lua_Integer> col_bot_left(L,9);
      LUAWRAP_CHECK_ARGS(self, p_min, p_max, col_upr_left, col_upr_right, col_bot_right, col_bot_left);
      self.value->AddRectFilledMultiColor(p_min.value, p_max.value, col_upr_left.value, col_upr_right.value, col_bot_right.value, col_bot_left.value);
      return 0;
   }

   static int AddQuad(lua_State* L) {
      static const char* proto = "void ImDrawList::AddQuad(ImVec2 p1, ImVec2 p2, ImVec2 p3, ImVec2 p4, ImU32 col, float thickness=1.0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> p1(L,2);
      Arg<ImVec2> p2(L,4);
      Arg<ImVec2> p3(L,6);
      Arg<ImVec2> p4(L,8);
      Arg<unsigned int,lua_Integer> col(L,10);
      Arg<float,lua_Number> thickness(L,11,1.0);
      LUAWRAP_CHECK_ARGS(self, p1, p2, p3, p4, col, thickness);
      self.value->AddQuad(p1.value, p2.value, p3.value, p4.value, col.value, thickness.value);
      return 0;
   }

   static int AddQuadFilled(lua_State* L) {
      static const char* proto = "void ImDrawList::AddQuadFilled(ImVec2 p1, ImVec2 p2, ImVec2 p3, ImVec2 p4, ImU32 col)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> p1(L,2);
      Arg<ImVec2> p2(L,4);
      Arg<ImVec2> p3(L,6);
      Arg<ImVec2> p4(L,8);
      Arg<unsigned int,lua_Integer> col(L,10);
      LUAWRAP_CHECK_ARGS(self, p1, p2, p3, p4, col);
      self.value->AddQuadFilled(p1.value, p2.value, p3.value, p4.value, col.value);
      return 0;
   }

   static int AddTriangle(lua_State* L) {
      static const char* proto = "void ImDrawList::AddTriangle(ImVec2 p1, ImVec2 p2, ImVec2 p3, ImU32 col, float thickness=1.0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> p1(L,2);
      Arg<ImVec2> p2(L,4);
      Arg<ImVec2> p3(L,6);
      Arg<unsigned int,lua_Integer> col(L,8);
      Arg<float,lua_Number> thickness(L,9,1.0);
      LUAWRAP_CHECK_ARGS(self, p1, p2, p3, col, thickness);
      self.value->AddTriangle(p1.value, p2.value, p3.value, col.value, thickness.value);
      return 0;
   }

   static int AddTriangleFilled(lua_State* L) {
      static const char* proto = "void ImDrawList::AddTriangleFilled(ImVec2 p1, ImVec2 p2, ImVec2 p3, ImU32 col)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> p1(L,2);
      Arg<ImVec2> p2(L,4);
      Arg<ImVec2> p3(L,6);
      Arg<unsigned int,lua_Integer> col(L,8);
      LUAWRAP_CHECK_ARGS(self, p1, p2, p3, col);
      self.value->AddTriangleFilled(p1.value, p2.value, p3.value, col.value);
      return 0;
   }

   static int AddCircle(lua_State* L) {
      static const char* proto = "void ImDrawList::AddCircle(ImVec2 center, float radius, ImU32 col, int num_segments=0, float thickness=1.0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> center(L,2);
      Arg<float,lua_Number> radius(L,4);
      Arg<unsigned int,lua_Integer> col(L,5);
      Arg<int,lua_Integer> num_segments(L,6,0);
      Arg<float,lua_Number> thickness(L,7,1.0);
      LUAWRAP_CHECK_ARGS(self, center, radius, col, num_segments, thickness);
      self.value->AddCircle(center.value, radius.value, col.value, num_segments.value, thickness.value);
      return 0;
   }

   static int AddCircleFilled(lua_State* L) {
      static const char* proto = "void ImDrawList::AddCircleFilled(ImVec2 center, float radius, ImU32 col, int num_segments=0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> center(L,2);
      Arg<float,lua_Number> radius(L,4);
      Arg<unsigned int,lua_Integer> col(L,5);
      Arg<int,lua_Integer> num_segments(L,6,0);
      LUAWRAP_CHECK_ARGS(self, center, radius, col, num_segments);
      self.value->AddCircleFilled(center.value, radius.value, col.value, num_segments.value);
      return 0;
   }

   static int AddNgon(lua_State* L) {
      static const char* proto = "void ImDrawList::AddNgon(ImVec2 center, float radius, ImU32 col, int num_segments, float thickness=1.0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> center(L,2);
      Arg<float,lua_Number> radius(L,4);
      Arg<unsigned int,lua_Integer> col(L,5);
      Arg<int,lua_Integer> num_segments(L,6);
      Arg<float,lua_Number> thickness(L,7,1.0);
      LUAWRAP_CHECK_ARGS(self, center, radius, col, num_segments, thickness);
      self.value->AddNgon(center.value, radius.value, col.value, num_segments.value, thickness.value);
      return 0;
   }

   static int AddNgonFilled(lua_State* L) {
      static const char* proto = "void ImDrawList::AddNgonFilled(ImVec2 center, float radius, ImU32 col, int num_segments)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> center(L,2);
      Arg<float,lua_Number> radius(L,4);
      Arg<unsigned int,lua_Integer> col(L,5);
      Arg<int,lua_Integer> num_segments(L,6);
      LUAWRAP_CHECK_ARGS(self, center, radius, col, num_segments);
      self.value->AddNgonFilled(center.value, radius.value, col.value, num_segments.value);
      return 0;
   }

   static int AddEllipse(lua_State* L) {
      static const char* proto = "void ImDrawList::AddEllipse(ImVec2 center, ImVec2 radius, ImU32 col, float rot=0.0, int num_segments=0, float thickness=1.0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> center(L,2);
      Arg<ImVec2> radius(L,4);
      Arg<unsigned int,lua_Integer> col(L,6);
      Arg<float,lua_Number> rot(L,7,0.0);
      Arg<int,lua_Integer> num_segments(L,8,0);
      Arg<float,lua_Number> thickness(L,9,1.0);
      LUAWRAP_CHECK_ARGS(self, center, radius, col, rot, num_segments, thickness);
      self.value->AddEllipse(center.value, radius.value, col.value, rot.value, num_segments.value, thickness.value);
      return 0;
   }

   static int AddEllipseFilled(lua_State* L) {
      static const char* proto = "void ImDrawList::AddEllipseFilled(ImVec2 center, ImVec2 radius, ImU32 col, float rot=0.0, int num_segments=0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> center(L,2);
      Arg<ImVec2> radius(L,4);
      Arg<unsigned int,lua_Integer> col(L,6);
      Arg<float,lua_Number> rot(L,7,0.0);
      Arg<int,lua_Integer> num_segments(L,8,0);
      LUAWRAP_CHECK_ARGS(self, center, radius, col, rot, num_segments);
      self.value->AddEllipseFilled(center.value, radius.value, col.value, rot.value, num_segments.value);
      return 0;
   }

   static int AddText(lua_State* L) {
      static const char* proto = "void ImDrawList::AddText(ImVec2 pos, ImU32 col, const char* text_begin, const char* text_end=NULL)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> pos(L,2);
      Arg<unsigned int,lua_Integer> col(L,4);
      Arg<const char*> text_begin(L,5);
      Arg<const char*> text_end(L,6,NULL);
      LUAWRAP_CHECK_ARGS(self, pos, col, text_begin, text_end);
      self.value->AddText(pos.value, col.value, text_begin.value, text_end.value);
      return 0;
   }

   static int AddBezierCubic(lua_State* L) {
      static const char* proto = "void ImDrawList::AddBezierCubic(ImVec2 p1, ImVec2 p2, ImVec2 p3, ImVec2 p4, ImU32 col, float thickness, int num_segments=0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> p1(L,2);
      Arg<ImVec2> p2(L,4);
      Arg<ImVec2> p3(L,6);
      Arg<ImVec2> p4(L,8);
      Arg<unsigned int,lua_Integer> col(L,10);
      Arg<float,lua_Number> thickness(L,11);
      Arg<int,lua_Integer> num_segments(L,12,0);
      LUAWRAP_CHECK_ARGS(self, p1, p2, p3, p4, col, thickness, num_segments);
      self.value->AddBezierCubic(p1.value, p2.value, p3.value, p4.value, col.value, thickness.value, num_segments.value);
      return 0;
   }

   static int AddBezierQuadratic(lua_State* L) {
      static const char* proto = "void ImDrawList::AddBezierQuadratic(ImVec2 p1, ImVec2 p2, ImVec2 p3, ImU32 col, float thickness, int num_segments=0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> p1(L,2);
      Arg<ImVec2> p2(L,4);
      Arg<ImVec2> p3(L,6);
      Arg<unsigned int,lua_Integer> col(L,8);
      Arg<float,lua_Number> thickness(L,9);
      Arg<int,lua_Integer> num_segments(L,10,0);
      LUAWRAP_CHECK_ARGS(self, p1, p2, p3, col, thickness, num_segments);
      self.value->AddBezierQuadratic(p1.value, p2.value, p3.value, col.value, thickness.value, num_segments.value);
      return 0;
   }

   static int AddImage(lua_State* L) {
      static const char* proto = "void ImDrawList::AddImage(ImTextureRef tex_ref, ImVec2 p_min, ImVec2 p_max, ImVec2 uv_min=ImVec2(0, 0), ImVec2 uv_max=ImVec2(1, 1), ImU32 col=(((ImU32) (255) << 24)|((ImU32) (255) << 16)|((ImU32) (255) << 8)|((ImU32) (255) << 0)))";
      Arg<ImDrawList*> self(L,1);
      Arg<ImTextureRef> tex_ref(L,2);
      Arg<ImVec2> p_min(L,3);
      Arg<ImVec2> p_max(L,5);
      Arg<ImVec2> uv_min(L,7,ImVec2(0, 0));
      Arg<ImVec2> uv_max(L,9,ImVec2(1, 1));
      Arg<unsigned int,lua_Integer> col(L,11,(((ImU32) (255) << 24)|((ImU32) (255) << 16)|((ImU32) (255) << 8)|((ImU32) (255) << 0)));
      LUAWRAP_CHECK_ARGS(self, tex_ref, p_min, p_max, uv_min, uv_max, col);
      self.value->AddImage(tex_ref.value, p_min.value, p_max.value, uv_min.value, uv_max.value, col.value);
      return 0;
   }

   static int AddImageQuad(lua_State* L) {
      static const char* proto = "void ImDrawList::AddImageQuad(ImTextureRef tex_ref, ImVec2 p1, ImVec2 p2, ImVec2 p3, ImVec2 p4, ImVec2 uv1=ImVec2(0, 0), ImVec2 uv2=ImVec2(1, 0), ImVec2 uv3=ImVec2(1, 1), ImVec2 uv4=ImVec2(0, 1), ImU32 col=(((ImU32) (255) << 24)|((ImU32) (255) << 16)|((ImU32) (255) << 8)|((ImU32) (255) << 0)))";
      Arg<ImDrawList*> self(L,1);
      Arg<ImTextureRef> tex_ref(L,2);
      Arg<ImVec2> p1(L,3);
      Arg<ImVec2> p2(L,5);
      Arg<ImVec2> p3(L,7);
      Arg<ImVec2> p4(L,9);
      Arg<ImVec2> uv1(L,11,ImVec2(0, 0));
      Arg<ImVec2> uv2(L,13,ImVec2(1, 0));
      Arg<ImVec2> uv3(L,15,ImVec2(1, 1));
      Arg<ImVec2> uv4(L,17,ImVec2(0, 1));
      Arg<unsigned int,lua_Integer> col(L,19,(((ImU32) (255) << 24)|((ImU32) (255) << 16)|((ImU32) (255) << 8)|((ImU32) (255) << 0)));
      LUAWRAP_CHECK_ARGS(self, tex_ref, p1, p2, p3, p4, uv1, uv2, uv3, uv4, col);
      self.value->AddImageQuad(tex_ref.value, p1.value, p2.value, p3.value, p4.value, uv1.value, uv2.value, uv3.value, uv4.value, col.value);
      return 0;
   }

   static int AddImageRounded(lua_State* L) {
      static const char* proto = "void ImDrawList::AddImageRounded(ImTextureRef tex_ref, ImVec2 p_min, ImVec2 p_max, ImVec2 uv_min, ImVec2 uv_max, ImU32 col, float rounding, ImDrawFlags flags=0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImTextureRef> tex_ref(L,2);
      Arg<ImVec2> p_min(L,3);
      Arg<ImVec2> p_max(L,5);
      Arg<ImVec2> uv_min(L,7);
      Arg<ImVec2> uv_max(L,9);
      Arg<unsigned int,lua_Integer> col(L,11);
      Arg<float,lua_Number> rounding(L,12);
      Arg<int,lua_Integer> flags(L,13,0);
      LUAWRAP_CHECK_ARGS(self, tex_ref, p_min, p_max, uv_min, uv_max, col, rounding, flags);
      self.value->AddImageRounded(tex_ref.value, p_min.value, p_max.value, uv_min.value, uv_max.value, col.value, rounding.value, flags.value);
      return 0;
   }

   static int PathClear(lua_State* L) {
      static const char* proto = "void ImDrawList::PathClear()";
      Arg<ImDrawList*> self(L,1);
      LUAWRAP_CHECK_ARGS(self);
      self.value->PathClear();
      return 0;
   }

   static int PathLineTo(lua_State* L) {
      static const char* proto = "void ImDrawList::PathLineTo(ImVec2 pos)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> pos(L,2);
      LUAWRAP_CHECK_ARGS(self, pos);
      self.value->PathLineTo(pos.value);
      return 0;
   }

   static int PathLineToMergeDuplicate(lua_State* L) {
      static const char* proto = "void ImDrawList::PathLineToMergeDuplicate(ImVec2 pos)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> pos(L,2);
      LUAWRAP_CHECK_ARGS(self, pos);
      self.value->PathLineToMergeDuplicate(pos.value);
      return 0;
   }

   static int PathFillConvex(lua_State* L) {
      static const char* proto = "void ImDrawList::PathFillConvex(ImU32 col)";
      Arg<ImDrawList*> self(L,1);
      Arg<unsigned int,lua_Integer> col(L,2);
      LUAWRAP_CHECK_ARGS(self, col);
      self.value->PathFillConvex(col.value);
      return 0;
   }

   static int PathFillConcave(lua_State* L) {
      static const char* proto = "void ImDrawList::PathFillConcave(ImU32 col)";
      Arg<ImDrawList*> self(L,1);
      Arg<unsigned int,lua_Integer> col(L,2);
      LUAWRAP_CHECK_ARGS(self, col);
      self.value->PathFillConcave(col.value);
      return 0;
   }

   static int PathStroke(lua_State* L) {
      static const char* proto = "void ImDrawList::PathStroke(ImU32 col, ImDrawFlags flags=0, float thickness=1.0)";
      Arg<ImDrawList*> self(L,1);
      Arg<unsigned int,lua_Integer> col(L,2);
      Arg<int,lua_Integer> flags(L,3,0);
      Arg<float,lua_Number> thickness(L,4,1.0);
      LUAWRAP_CHECK_ARGS(self, col, flags, thickness);
      self.value->PathStroke(col.value, flags.value, thickness.value);
      return 0;
   }

   static int PathArcTo(lua_State* L) {
      static const char* proto = "void ImDrawList::PathArcTo(ImVec2 center, float radius, float a_min, float a_max, int num_segments=0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> center(L,2);
      Arg<float,lua_Number> radius(L,4);
      Arg<float,lua_Number> a_min(L,5);
      Arg<float,lua_Number> a_max(L,6);
      Arg<int,lua_Integer> num_segments(L,7,0);
      LUAWRAP_CHECK_ARGS(self, center, radius, a_min, a_max, num_segments);
      self.value->PathArcTo(center.value, radius.value, a_min.value, a_max.value, num_segments.value);
      return 0;
   }

   static int PathArcToFast(lua_State* L) {
      static const char* proto = "void ImDrawList::PathArcToFast(ImVec2 center, float radius, int a_min_of_12, int a_max_of_12)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> center(L,2);
      Arg<float,lua_Number> radius(L,4);
      Arg<int,lua_Integer> a_min_of_12(L,5);
      Arg<int,lua_Integer> a_max_of_12(L,6);
      LUAWRAP_CHECK_ARGS(self, center, radius, a_min_of_12, a_max_of_12);
      self.value->PathArcToFast(center.value, radius.value, a_min_of_12.value, a_max_of_12.value);
      return 0;
   }

   static int PathEllipticalArcTo(lua_State* L) {
      static const char* proto = "void ImDrawList::PathEllipticalArcTo(ImVec2 center, ImVec2 radius, float rot, float a_min, float a_max, int num_segments=0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> center(L,2);
      Arg<ImVec2> radius(L,4);
      Arg<float,lua_Number> rot(L,6);
      Arg<float,lua_Number> a_min(L,7);
      Arg<float,lua_Number> a_max(L,8);
      Arg<int,lua_Integer> num_segments(L,9,0);
      LUAWRAP_CHECK_ARGS(self, center, radius, rot, a_min, a_max, num_segments);
      self.value->PathEllipticalArcTo(center.value, radius.value, rot.value, a_min.value, a_max.value, num_segments.value);
      return 0;
   }

   static int PathBezierCubicCurveTo(lua_State* L) {
      static const char* proto = "void ImDrawList::PathBezierCubicCurveTo(ImVec2 p2, ImVec2 p3, ImVec2 p4, int num_segments=0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> p2(L,2);
      Arg<ImVec2> p3(L,4);
      Arg<ImVec2> p4(L,6);
      Arg<int,lua_Integer> num_segments(L,8,0);
      LUAWRAP_CHECK_ARGS(self, p2, p3, p4, num_segments);
      self.value->PathBezierCubicCurveTo(p2.value, p3.value, p4.value, num_segments.value);
      return 0;
   }

   static int PathBezierQuadraticCurveTo(lua_State* L) {
      static const char* proto = "void ImDrawList::PathBezierQuadraticCurveTo(ImVec2 p2, ImVec2 p3, int num_segments=0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> p2(L,2);
      Arg<ImVec2> p3(L,4);
      Arg<int,lua_Integer> num_segments(L,6,0);
      LUAWRAP_CHECK_ARGS(self, p2, p3, num_segments);
      self.value->PathBezierQuadraticCurveTo(p2.value, p3.value, num_segments.value);
      return 0;
   }

   static int PathRect(lua_State* L) {
      static const char* proto = "void ImDrawList::PathRect(ImVec2 rect_min, ImVec2 rect_max, float rounding=0.0, ImDrawFlags flags=0)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> rect_min(L,2);
      Arg<ImVec2> rect_max(L,4);
      Arg<float,lua_Number> rounding(L,6,0.0);
      Arg<int,lua_Integer> flags(L,7,0);
      LUAWRAP_CHECK_ARGS(self, rect_min, rect_max, rounding, flags);
      self.value->PathRect(rect_min.value, rect_max.value, rounding.value, flags.value);
      return 0;
   }

   static int AddDrawCmd(lua_State* L) {
      static const char* proto = "void ImDrawList::AddDrawCmd()";
      Arg<ImDrawList*> self(L,1);
      LUAWRAP_CHECK_ARGS(self);
      self.value->AddDrawCmd();
      return 0;
   }

   static int CloneOutput(lua_State* L) {
      static const char* proto = "ImDrawList* ImDrawList::CloneOutput()";
      Arg<ImDrawList*> self(L,1);
      LUAWRAP_CHECK_ARGS(self);
      Arg<ImDrawList*> retval = self.value->CloneOutput();
      int prevtop = lua_gettop(L);
      retval.push(L);
      return lua_gettop(L)-prevtop;
   }

   static int ChannelsSplit(lua_State* L) {
      static const char* proto = "void ImDrawList::ChannelsSplit(int count)";
      Arg<ImDrawList*> self(L,1);
      Arg<int,lua_Integer> count(L,2);
      LUAWRAP_CHECK_ARGS(self, count);
      self.value->ChannelsSplit(count.value);
      return 0;
   }

   static int ChannelsMerge(lua_State* L) {
      static const char* proto = "void ImDrawList::ChannelsMerge()";
      Arg<ImDrawList*> self(L,1);
      LUAWRAP_CHECK_ARGS(self);
      self.value->ChannelsMerge();
      return 0;
   }

   static int ChannelsSetCurrent(lua_State* L) {
      static const char* proto = "void ImDrawList::ChannelsSetCurrent(int n)";
      Arg<ImDrawList*> self(L,1);
      Arg<int,lua_Integer> n(L,2);
      LUAWRAP_CHECK_ARGS(self, n);
      self.value->ChannelsSetCurrent(n.value);
      return 0;
   }

   static int PrimReserve(lua_State* L) {
      static const char* proto = "void ImDrawList::PrimReserve(int idx_count, int vtx_count)";
      Arg<ImDrawList*> self(L,1);
      Arg<int,lua_Integer> idx_count(L,2);
      Arg<int,lua_Integer> vtx_count(L,3);
      LUAWRAP_CHECK_ARGS(self, idx_count, vtx_count);
      self.value->PrimReserve(idx_count.value, vtx_count.value);
      return 0;
   }

   static int PrimUnreserve(lua_State* L) {
      static const char* proto = "void ImDrawList::PrimUnreserve(int idx_count, int vtx_count)";
      Arg<ImDrawList*> self(L,1);
      Arg<int,lua_Integer> idx_count(L,2);
      Arg<int,lua_Integer> vtx_count(L,3);
      LUAWRAP_CHECK_ARGS(self, idx_count, vtx_count);
      self.value->PrimUnreserve(idx_count.value, vtx_count.value);
      return 0;
   }

   static int PrimRect(lua_State* L) {
      static const char* proto = "void ImDrawList::PrimRect(ImVec2 a, ImVec2 b, ImU32 col)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> a(L,2);
      Arg<ImVec2> b(L,4);
      Arg<unsigned int,lua_Integer> col(L,6);
      LUAWRAP_CHECK_ARGS(self, a, b, col);
      self.value->PrimRect(a.value, b.value, col.value);
      return 0;
   }

   static int PrimRectUV(lua_State* L) {
      static const char* proto = "void ImDrawList::PrimRectUV(ImVec2 a, ImVec2 b, ImVec2 uv_a, ImVec2 uv_b, ImU32 col)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> a(L,2);
      Arg<ImVec2> b(L,4);
      Arg<ImVec2> uv_a(L,6);
      Arg<ImVec2> uv_b(L,8);
      Arg<unsigned int,lua_Integer> col(L,10);
      LUAWRAP_CHECK_ARGS(self, a, b, uv_a, uv_b, col);
      self.value->PrimRectUV(a.value, b.value, uv_a.value, uv_b.value, col.value);
      return 0;
   }

   static int PrimQuadUV(lua_State* L) {
      static const char* proto = "void ImDrawList::PrimQuadUV(ImVec2 a, ImVec2 b, ImVec2 c, ImVec2 d, ImVec2 uv_a, ImVec2 uv_b, ImVec2 uv_c, ImVec2 uv_d, ImU32 col)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> a(L,2);
      Arg<ImVec2> b(L,4);
      Arg<ImVec2> c(L,6);
      Arg<ImVec2> d(L,8);
      Arg<ImVec2> uv_a(L,10);
      Arg<ImVec2> uv_b(L,12);
      Arg<ImVec2> uv_c(L,14);
      Arg<ImVec2> uv_d(L,16);
      Arg<unsigned int,lua_Integer> col(L,18);
      LUAWRAP_CHECK_ARGS(self, a, b, c, d, uv_a, uv_b, uv_c, uv_d, col);
      self.value->PrimQuadUV(a.value, b.value, c.value, d.value, uv_a.value, uv_b.value, uv_c.value, uv_d.value, col.value);
      return 0;
   }

   static int PrimWriteVtx(lua_State* L) {
      static const char* proto = "void ImDrawList::PrimWriteVtx(ImVec2 pos, ImVec2 uv, ImU32 col)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> pos(L,2);
      Arg<ImVec2> uv(L,4);
      Arg<unsigned int,lua_Integer> col(L,6);
      LUAWRAP_CHECK_ARGS(self, pos, uv, col);
      self.value->PrimWriteVtx(pos.value, uv.value, col.value);
      return 0;
   }

   static int PrimWriteIdx(lua_State* L) {
      static const char* proto = "void ImDrawList::PrimWriteIdx(ImDrawIdx idx)";
      Arg<ImDrawList*> self(L,1);
      Arg<unsigned short,lua_Integer> idx(L,2);
      LUAWRAP_CHECK_ARGS(self, idx);
      self.value->PrimWriteIdx(idx.value);
      return 0;
   }

   static int PrimVtx(lua_State* L) {
      static const char* proto = "void ImDrawList::PrimVtx(ImVec2 pos, ImVec2 uv, ImU32 col)";
      Arg<ImDrawList*> self(L,1);
      Arg<ImVec2> pos(L,2);
      Arg<ImVec2> uv(L,4);
      Arg<unsigned int,lua_Integer> col(L,6);
      LUAWRAP_CHECK_ARGS(self, pos, uv, col);
      self.value->PrimVtx(pos.value, uv.value, col.value);
      return 0;
   }

} // namespace ImDrawList_lua_wrappers_wrappers


void ImDrawList_lua_wrappers_register(lua_State* L) {
   lua_getglobal(L, "imgui");
   if(lua_isnil(L,-1)) {
      lua_pop(L,1); 
      lua_newtable(L); 
      lua_pushvalue(L, -1);
      lua_setglobal(L, "imgui");
   } 

   using namespace ImDrawList_lua_wrappers;
   LUAWRAP_DECLARE_FUNCTION(L,PushClipRect);
   LUAWRAP_DECLARE_FUNCTION(L,PushClipRectFullScreen);
   LUAWRAP_DECLARE_FUNCTION(L,PopClipRect);
   LUAWRAP_DECLARE_FUNCTION(L,PushTexture);
   LUAWRAP_DECLARE_FUNCTION(L,PopTexture);
   LUAWRAP_DECLARE_FUNCTION(L,GetClipRectMin);
   LUAWRAP_DECLARE_FUNCTION(L,GetClipRectMax);
   LUAWRAP_DECLARE_FUNCTION(L,AddLine);
   LUAWRAP_DECLARE_FUNCTION(L,AddRect);
   LUAWRAP_DECLARE_FUNCTION(L,AddRectFilled);
   LUAWRAP_DECLARE_FUNCTION(L,AddRectFilledMultiColor);
   LUAWRAP_DECLARE_FUNCTION(L,AddQuad);
   LUAWRAP_DECLARE_FUNCTION(L,AddQuadFilled);
   LUAWRAP_DECLARE_FUNCTION(L,AddTriangle);
   LUAWRAP_DECLARE_FUNCTION(L,AddTriangleFilled);
   LUAWRAP_DECLARE_FUNCTION(L,AddCircle);
   LUAWRAP_DECLARE_FUNCTION(L,AddCircleFilled);
   LUAWRAP_DECLARE_FUNCTION(L,AddNgon);
   LUAWRAP_DECLARE_FUNCTION(L,AddNgonFilled);
   LUAWRAP_DECLARE_FUNCTION(L,AddEllipse);
   LUAWRAP_DECLARE_FUNCTION(L,AddEllipseFilled);
   LUAWRAP_DECLARE_FUNCTION(L,AddText);
   LUAWRAP_DECLARE_FUNCTION(L,AddBezierCubic);
   LUAWRAP_DECLARE_FUNCTION(L,AddBezierQuadratic);
   LUAWRAP_DECLARE_FUNCTION(L,AddImage);
   LUAWRAP_DECLARE_FUNCTION(L,AddImageQuad);
   LUAWRAP_DECLARE_FUNCTION(L,AddImageRounded);
   LUAWRAP_DECLARE_FUNCTION(L,PathClear);
   LUAWRAP_DECLARE_FUNCTION(L,PathLineTo);
   LUAWRAP_DECLARE_FUNCTION(L,PathLineToMergeDuplicate);
   LUAWRAP_DECLARE_FUNCTION(L,PathFillConvex);
   LUAWRAP_DECLARE_FUNCTION(L,PathFillConcave);
   LUAWRAP_DECLARE_FUNCTION(L,PathStroke);
   LUAWRAP_DECLARE_FUNCTION(L,PathArcTo);
   LUAWRAP_DECLARE_FUNCTION(L,PathArcToFast);
   LUAWRAP_DECLARE_FUNCTION(L,PathEllipticalArcTo);
   LUAWRAP_DECLARE_FUNCTION(L,PathBezierCubicCurveTo);
   LUAWRAP_DECLARE_FUNCTION(L,PathBezierQuadraticCurveTo);
   LUAWRAP_DECLARE_FUNCTION(L,PathRect);
   LUAWRAP_DECLARE_FUNCTION(L,AddDrawCmd);
   LUAWRAP_DECLARE_FUNCTION(L,CloneOutput);
   LUAWRAP_DECLARE_FUNCTION(L,ChannelsSplit);
   LUAWRAP_DECLARE_FUNCTION(L,ChannelsMerge);
   LUAWRAP_DECLARE_FUNCTION(L,ChannelsSetCurrent);
   LUAWRAP_DECLARE_FUNCTION(L,PrimReserve);
   LUAWRAP_DECLARE_FUNCTION(L,PrimUnreserve);
   LUAWRAP_DECLARE_FUNCTION(L,PrimRect);
   LUAWRAP_DECLARE_FUNCTION(L,PrimRectUV);
   LUAWRAP_DECLARE_FUNCTION(L,PrimQuadUV);
   LUAWRAP_DECLARE_FUNCTION(L,PrimWriteVtx);
   LUAWRAP_DECLARE_FUNCTION(L,PrimWriteIdx);
   LUAWRAP_DECLARE_FUNCTION(L,PrimVtx);
   lua_pop(L,1);

   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_None);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_Closed);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_RoundCornersTopLeft);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_RoundCornersTopRight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_RoundCornersBottomLeft);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_RoundCornersBottomRight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_RoundCornersNone);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_RoundCornersTop);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_RoundCornersBottom);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_RoundCornersLeft);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_RoundCornersRight);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_RoundCornersAll);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_RoundCornersDefault_);
   LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,ImDrawFlags_RoundCornersMask_);
}
