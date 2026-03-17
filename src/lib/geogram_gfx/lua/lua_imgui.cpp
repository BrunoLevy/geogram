/*
 *  Copyright (c) 2000-2022 Inria
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *  * Neither the name of the ALICE Project-Team nor the names of its
 *  contributors may be used to endorse or promote products derived from this
 *  software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Contact: Bruno Levy
 *
 *     https://www.inria.fr/fr/bruno-levy
 *
 *     Inria,
 *     Domaine de Voluceau,
 *     78150 Le Chesnay - Rocquencourt
 *     FRANCE
 *
 */

#include <geogram_gfx/lua/lua_imgui.h>
#include <geogram_gfx/imgui_ext/imgui_ext.h>
#include <geogram_gfx/imgui_ext/icon_font.h>


#ifdef GEO_COMPILER_GCC_FAMILY
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#endif

#include <geogram_gfx/third_party/imoguizmo/imoguizmo.hpp>

#ifdef GEO_COMPILER_GCC_FAMILY
#pragma GCC diagnostic pop
#endif

#include <geogram/lua/lua_wrap.h>
#include <geogram/basic/string.h>
#include <geogram/basic/logger.h>
#include <map>

#include "luawrap_runtime.h"

extern void ImGui_lua_wrappers_register(lua_State* L);
extern void ImDrawList_lua_wrappers_register(lua_State* L);


namespace {
    using namespace GEO;


    int wrapper_TextInput(lua_State* L) {

        if(
            lua_gettop(L) != 2 &&
            lua_gettop(L) != 3
        ) {
            return luaL_error(
                L, "'imgui.TextInput()' invalid number of arguments"
            );
        }

        if(!lua_isstring(L,1)) {
            return luaL_error(
                L, "'imgui.TextInput()' argument 1 should be a string"
            );
        }

        if(!lua_isstring(L,2)) {
            return luaL_error(
                L, "'imgui.TextInput()' argument 2 should be a string"
            );
        }

        ImGuiInputTextFlags flags = 0;

        if(lua_gettop(L) == 3) {
            if(!lua_isnumber(L,3)) {
                return luaL_error(
                    L, "'imgui.TextInput()' argument 3 should be a number"
                );
            }
            flags = ImGuiInputTextFlags(lua_tonumber(L,3));
        }

        const char* label  = lua_tostring(L,1);
        const char* str = lua_tostring(L,2);
        static char buff[geo_imgui_string_length];
        strcpy(buff,str);
        bool result = ImGui::InputText(
            label, buff, geo_imgui_string_length, flags
        );
        lua_pushboolean(L,result);
        lua_pushstring(L,buff);


        return 2;
    }


    int wrapper_Combo(lua_State* L) {
        if(lua_gettop(L) != 3) {
            return luaL_error(
                L, "'imgui.Combo()' invalid number of arguments"
            );
        }

        if(!lua_isstring(L,1)) {
            return luaL_error(
                L, "'imgui.Combo()' argument should be a string"
            );
        }

        if(!lua_isstring(L,2)) {
            return luaL_error(
                L, "'imgui.Combo()' argument should be a string"
            );
        }

        if(!lua_isstring(L,3)) {
            return luaL_error(
                L, "'imgui.Combo()' argument should be a string"
            );
        }

        const char* label = lua_tostring(L,1);
        const char* current_item = lua_tostring(L,2);
        const char* items = lua_tostring(L,3);

        char* lua_items = (char*)alloca(strlen(items)+2);
        strcpy(lua_items,items);
        {
            size_t n = strlen(lua_items);
            lua_items[n] = ';';
            lua_items[n+1] = '\0';
        }

        int lua_current_item=0;

        const char* prev_item = lua_items;
        int nb_items = 0;

        char* p = lua_items;
        while(*p != '\0') {
            if(*p == ';') {
                *p = '\0';
                if(!strcmp(prev_item, current_item)) {
                    lua_current_item = nb_items;
                }
                prev_item = p+1;
                ++nb_items;
            }
            ++p;
        }
        *p = '\0'; // Double '\0' to indicate end of item list to lua.

        bool result = ImGui::Combo(label, &lua_current_item, lua_items);

        current_item = lua_items;
        while(lua_current_item > 0) {
            while(*current_item) {
                ++current_item;
            }
            ++current_item;
            --lua_current_item;
        }

        lua_pushboolean(L, result);
        lua_pushstring(L, current_item);

        return 2;
    }

    int wrapper_ColorEdit3WithPalette(
        lua_State* L
    ) {
        if(lua_gettop(L) != 4) {
            return luaL_error(
                L,
		"'imgui.ColorEdit3WithPalette()' invalid number of arguments"
            );
        }

        if(!lua_isstring(L,1)) {
            return luaL_error(
                L,
		"'imgui.ColorEdit3WithPalette()' argument 1 should be a string"
            );
        }

        if(!lua_isnumber(L,2)) {
            return luaL_error(
                L,
		"'imgui.ColorEdit3WithPalette()' argument 2 should be a number"
            );
        }

        if(!lua_isnumber(L,3)) {
            return luaL_error(
                L,
		"'imgui.ColorEdit3WithPalette()' argument 3 should be a number"
            );
        }

        if(!lua_isnumber(L,4)) {
            return luaL_error(
                L,
		"'imgui.ColorEdit3WithPalette()' argument 4 should be a number"
            );
        }

        const char* label = lua_tostring(L,1);

        float rgb[3];
        rgb[0] = float(lua_tonumber(L,2));
        rgb[1] = float(lua_tonumber(L,3));
        rgb[2] = float(lua_tonumber(L,4));

        bool sel = ImGui::ColorEdit3WithPalette(
            label, rgb
        );

        lua_pushboolean(L,sel);
        lua_pushnumber(L,double(rgb[0]));
        lua_pushnumber(L,double(rgb[1]));
        lua_pushnumber(L,double(rgb[2]));

        return 4;
    }

    int wrapper_ColorEdit4WithPalette(
        lua_State* L
    ) {
        if(lua_gettop(L) != 5) {
            return luaL_error(
                L,
		"'imgui.ColorEdit3WithPalette()' invalid number of arguments"
            );
        }

        if(!lua_isstring(L,1)) {
            return luaL_error(
                L,
		"'imgui.ColorEdit3WithPalette()' argument 1 should be a string"
            );
        }

        if(!lua_isnumber(L,2)) {
            return luaL_error(
                L,
		"'imgui.ColorEdit3WithPalette()' argument 2 should be a number"
            );
        }

        if(!lua_isnumber(L,3)) {
            return luaL_error(
                L,
		"'imgui.ColorEdit3WithPalette()' argument 3 should be a number"
            );
        }

        if(!lua_isnumber(L,4)) {
            return luaL_error(
                L,
		"'imgui.ColorEdit3WithPalette()' argument 4 should be a number"
            );
        }

        if(!lua_isnumber(L,5)) {
            return luaL_error(
                L,
		"'imgui.ColorEdit3WithPalette()' argument 5 should be a number"
            );
        }

        const char* label = lua_tostring(L,1);

        float rgb[4];
        rgb[0] = float(lua_tonumber(L,2));
        rgb[1] = float(lua_tonumber(L,3));
        rgb[2] = float(lua_tonumber(L,4));
        rgb[3] = float(lua_tonumber(L,5));

        bool sel = ImGui::ColorEdit4WithPalette(
            label, rgb
        );

        lua_pushboolean(L,sel);
        lua_pushnumber(L,double(rgb[0]));
        lua_pushnumber(L,double(rgb[1]));
        lua_pushnumber(L,double(rgb[2]));
        lua_pushnumber(L,double(rgb[3]));

        return 5;
    }


    int wrapper_OpenFileDialog(
        lua_State* L
    ) {
        if(lua_gettop(L) != 4) {
            return luaL_error(
                L, "'imgui.OpenFileDialog()' invalid number of arguments"
            );
        }

        if(!lua_isstring(L,1)) {
            return luaL_error(
                L, "'imgui.OpenFileDialog()' argument 1 should be a string"
            );
        }

        if(!lua_isstring(L,2)) {
            return luaL_error(
                L, "'imgui.OpenFileDialog()' argument 2 should be a string"
            );
        }

        if(!lua_isstring(L,3)) {
            return luaL_error(
                L, "'imgui.OpenFileDialog()' argument 3 should be a string"
            );
        }

        if(!lua_isnumber(L,4)) {
            return luaL_error(
                L, "'imgui.OpenFileDialog()' argument 4 should be a number"
            );
        }

        const char* label      = lua_tostring(L,1);
        const char* extensions = lua_tostring(L,2);
        const char* filename   = lua_tostring(L,3);
        ImGuiExtFileDialogFlags flags =
            ImGuiExtFileDialogFlags(lua_tonumber(L,4));

        ImGui::OpenFileDialog(label, extensions, filename, flags);

        return 0;
    }

    int wrapper_FileDialog(
        lua_State* L
    ) {
        if(lua_gettop(L) != 2) {
            return luaL_error(
                L, "'imgui.FileDialog()' invalid number of arguments"
            );
        }

        if(!lua_isstring(L,1)) {
            return luaL_error(
                L, "'imgui.OpenFileDialog()' argument 1 should be a string"
            );
        }

        if(!lua_isstring(L,2)) {
            return luaL_error(
                L, "'imgui.OpenFileDialog()' argument 2 should be a string"
            );
        }

        const char* label      = lua_tostring(L,1);
        char filename[geo_imgui_string_length];

        const char* filename_in = lua_tostring(L,2);
        if(filename_in != nullptr) {
            if(strlen(filename_in) > geo_imgui_string_length + 1) {
                Logger::err("ImGui") << "Max file name length exceeded"
                                     << std::endl;
                return false;
            }
            strcpy(filename, filename_in);
        } else {
            filename[0] = '\0';
        }

        bool result =
            ImGui::FileDialog(label, filename, geo_imgui_string_length);

        lua_pushboolean(L,result);
        lua_pushstring(L, result ? filename : filename_in);

        return 2;
    }

    int wrapper_PushFont(lua_State* L) {
        if(lua_gettop(L) != 1) {
            return luaL_error(
                L, "'imgui.PushFont()' invalid number of arguments"
            );
        }
        if(!lua_isinteger(L,1)) {
            return luaL_error(
                L, "'imgui.PushFont()' argument is not an integer"
            );
        }
        int idx = int(lua_tointeger(L,1));
        if(idx < 0 || idx >= ImGui::GetIO().Fonts->Fonts.size()) {
            return luaL_error(
                L, "'imgui.PushFont()' invalid font index"
            );
        }
        ImGui::PushFont(ImGui::GetIO().Fonts->Fonts[idx]);
        return 0;
    }

    int wrapper_font_icon(lua_State* L) {
        if(lua_gettop(L) != 1) {
            return luaL_error(
                L, "'imgui.font_icon()' invalid number of arguments"
            );
        }
        if(!lua_isstring(L,1)) {
            return luaL_error(
                L, "'imgui.font_icon()' argument is not a string"
            );
        }
        const char* K = lua_tostring(L,1);
        wchar_t result[2];
        result[0] = icon_wchar(K);
        result[1] = '\0';
        std::string result_str = String::wchar_to_UTF8(result);
        lua_pushstring(L, result_str.c_str());
        return 1;
    }

    int wrapper_SimpleButton(lua_State* L) {
        if(lua_gettop(L) != 1) {
            return luaL_error(
                L, "'imgui.SimpleButton()' invalid number of arguments"
            );
        }
        if(!lua_isstring(L,1)) {
            return luaL_error(
                L, "'imgui.SimpleButton()' argument is not a string"
            );
        }
        const char* K = lua_tostring(L,1);
        lua_pushboolean(L,ImGui::SimpleButton(K));
        return 1;
    }

    int wrapper_SimpleButton2(lua_State* L) {
        if(lua_gettop(L) != 3) {
            return luaL_error(
                L, "'imgui.SimpleButton2()' invalid number of arguments"
            );
        }
        if(!lua_isstring(L,1)) {
            return luaL_error(
                L, "'imgui.SimpleButton2()' argument is not a string"
            );
        }
        if(!lua_isnumber(L,2)) {
            return luaL_error(
                L, "'imgui.SimpleButton2()' argument is not a number"
            );
        }
        if(!lua_isnumber(L,3)) {
            return luaL_error(
                L, "'imgui.SimpleButton2()' argument is not a number"
            );
        }
        const char* K = lua_tostring(L,1);
        lua_pushboolean(
	    L,
	    ImGui::SimpleButton(
		K,
		ImVec2(float(lua_tonumber(L,2)), float(lua_tonumber(L,3))))
	);
        return 1;
    }

    int wrapper_DrawGizmo(lua_State* L) {
	if(lua_gettop(L) != 3) {
            return luaL_error(
                L, "'ImOGuizmo.DrawGizmo()' invalid number of arguments"
            );
	}
	Matrix<4,float> vm;
	Matrix<4,float> pm;
	if(!lua_tomat(L,1,vm)) {
            return luaL_error(
                L, "'ImOGuizmo.DrawGizmo()' arg 1 is not a matrix"
            );
	}
	if(!lua_tomat(L,2,pm)) {
            return luaL_error(
                L, "'ImOGuizmo.DrawGizmo()' arg 2 is not a matrix"
	    );
	}
	if(!lua_isnumber(L,3)) {
	    return luaL_error(
		L, "'imOGuizmo.DrawGizmo()' arg 3 is not a number"
	    );
	}
	float distance = float(lua_tonumber(L,3));
	bool changed = ImOGuizmo::DrawGizmo(vm.data(), pm.data(), distance);
	lua_push(L,changed);
	lua_push(L,vm);
	return 2;
    }

    int wrapper_IO_DisplayFramebufferScale(lua_State* L) {
	if(lua_gettop(L) != 0) {
	    return luaL_error(
		L, "'IO_DisplayFramebufferScale' invalid number of arguments"
	    );
	}
	lua_pushnumber(L,lua_Number(ImGui::GetIO().DisplayFramebufferScale.x));
	lua_pushnumber(L,lua_Number(ImGui::GetIO().DisplayFramebufferScale.y));
	return 2;
    }
}

#define DECLARE_IMGUI_CONSTANT(C)               \
    lua_pushinteger(L,C);                       \
    lua_setglobal(L,#C)


// Overloads (that gomgen cannot generate yet)
namespace ImGui_lua_wrappers {
    using namespace LuaWrap;

    static int PushStyleVar_2(lua_State* L) {
      static const char* proto =
	  "void ImGui::PushStyleVar_2(ImGuiStyleVar idx, ImVec2 val)";
      Arg<int,lua_Integer> idx(L,1);
      Arg<ImVec2> val(L,2);
      LUAWRAP_CHECK_ARGS(idx, val);
      ImGui::PushStyleVar(idx.value, val.value);
      return 0;
   }

    static int PushStyleColor_2(lua_State* L) {
      static const char* proto =
	  "void ImGui::PushStyleColor_2(ImGuiCol idx, ImVec4 col)";
      Arg<int,lua_Integer> idx(L,1);
      Arg<ImVec4> col(L,2);
      LUAWRAP_CHECK_ARGS(idx, col);
      ImGui::PushStyleColor(idx.value, col.value);
      return 0;
   }
}

void init_lua_imgui(lua_State* L) {
    ImGui_lua_wrappers_register(L);
    ImDrawList_lua_wrappers_register(L);

    DECLARE_IMGUI_CONSTANT(ImGuiExtFileDialogFlags_Load);
    DECLARE_IMGUI_CONSTANT(ImGuiExtFileDialogFlags_Save);

    lua_getglobal(L, "imgui");

    lua_pushliteral(L,"TextInput");
    lua_pushcfunction(L,wrapper_TextInput);
    lua_settable(L,-3);

    lua_pushliteral(L,"Combo");
    lua_pushcfunction(L,wrapper_Combo);
    lua_settable(L,-3);

    lua_pushliteral(L,"ColorEdit3WithPalette");
    lua_pushcfunction(L,wrapper_ColorEdit3WithPalette);
    lua_settable(L,-3);

    lua_pushliteral(L,"ColorEdit4WithPalette");
    lua_pushcfunction(L,wrapper_ColorEdit4WithPalette);
    lua_settable(L,-3);

    lua_pushliteral(L,"OpenFileDialog");
    lua_pushcfunction(L,wrapper_OpenFileDialog);
    lua_settable(L,-3);

    lua_pushliteral(L,"FileDialog");
    lua_pushcfunction(L,wrapper_FileDialog);
    lua_settable(L,-3);

    lua_pushliteral(L,"PushFont");
    lua_pushcfunction(L,wrapper_PushFont);
    lua_settable(L,-3);

    lua_pushliteral(L,"font_icon");
    lua_pushcfunction(L,wrapper_font_icon);
    lua_settable(L,-3);

    lua_pushliteral(L,"SimpleButton");
    lua_pushcfunction(L,wrapper_SimpleButton);
    lua_settable(L,-3);

    lua_pushliteral(L,"SimpleButton2");
    lua_pushcfunction(L,wrapper_SimpleButton2);
    lua_settable(L,-3);

    /*****************************************************************/

    lua_pushliteral(L,"IO_DisplayFramebufferScale");
    lua_pushcfunction(L,wrapper_IO_DisplayFramebufferScale);
    lua_settable(L,-3);

    /*****************************************************************/

    // Overloads that could not be generated by gomgen
    using namespace ImGui_lua_wrappers;
    LUAWRAP_DECLARE_FUNCTION(L,PushStyleVar_2);
    LUAWRAP_DECLARE_FUNCTION(L,PushStyleColor_2);

    /*****************************************************************/

    lua_pop(L,1);

    lua_newtable(L);
    lua_bindwrapper(L,ImOGuizmo::BeginFrame);
    lua_bindwrapper(L,ImOGuizmo::SetRect);

    lua_pushliteral(L,"DrawGizmo");
    lua_pushcfunction(L,wrapper_DrawGizmo);
    lua_settable(L,-3);
    lua_setglobal(L,"ImOGuizmo");
}
