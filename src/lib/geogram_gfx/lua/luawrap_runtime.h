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
 */

#ifndef LUAWRAP_RUNTIME
#define LUAWRAP_RUNTIME

#ifdef __cplusplus
extern "C" {
#endif

#ifdef GEOGRAM_USE_BUILTIN_DEPS
#include <geogram/third_party/lua/lua.h>
#include <geogram/third_party/lua/lauxlib.h>
#include <geogram/third_party/lua/lualib.h>
#else
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
#endif

#ifdef __cplusplus
}
#endif

#ifdef GEOGRAM_USE_BUILTIN_DEPS
#include <geogram_gfx/third_party/imgui/imgui.h>
#else
#include <imgui.h>
#endif

#include <vector>
#include <string>


namespace LuaWrap {

    template <class T> struct LuaType {
    };

    template <> struct LuaType<lua_Integer> {
	static constexpr lua_Integer default_value = 0;
	static bool check(lua_State* L, int idx) {
	    return
		lua_isinteger(L,idx) ||
		lua_isboolean(L,idx) ; // we tolerate false->0 true->1 conversion
	}
	static lua_Integer get(lua_State* L, int idx) {
	    return lua_tointeger(L, idx);
	}
	template <class T> static void push(lua_State* L, T val) {
	    lua_pushinteger(L,lua_Integer(val));
	}
    };

    template <> struct LuaType<lua_Number> {
	static constexpr lua_Integer default_value = 0.0;
	static bool check(lua_State* L, int idx) {
	    return lua_isnumber(L,idx);
	}
	static lua_Number get(lua_State* L, int idx) {
	    return lua_tonumber(L, idx);
	}
	template <class T> static void push(lua_State* L, T val) {
	    lua_pushnumber(L,lua_Number(val));
	}
    };

    template <class T> struct LuaType<T*> {
	static constexpr T* default_value = nullptr;
	static bool check(lua_State* L, int idx) {
	    return lua_islightuserdata(L,idx);
	}
	static T* get(lua_State* L, int idx) {
	    return reinterpret_cast<T*>(lua_touserdata(L, idx));
	}
	static void push(lua_State* L, T* val) {
	    lua_pushlightuserdata(L,(void*)(val));
	}
    };

    template <> struct LuaType<const char*> {
	static constexpr const char* default_value = nullptr;
	static bool check(lua_State* L, int idx) {
	    return lua_isstring(L,idx);
	}
	static const char* get(lua_State* L, int idx) {
	    return lua_tostring(L, idx);
	}
	template <class T> static void push(lua_State* L, T val) {
	    lua_pushstring(L,(const char*)(val));
	}
    };

    template <> struct LuaType<bool> {
	static constexpr bool default_value = false;
	static bool check(lua_State* L, int idx) {
	    return lua_isboolean(L,idx);
	}
	static bool get(lua_State* L, int idx) {
	    return lua_toboolean(L, idx);
	}
	template <class T> static void push(lua_State* L, T val) {
	    lua_pushboolean(L,bool(val));
	}
    };

    template <> struct LuaType<ImVec2> {
	static constexpr ImVec2 default_value = ImVec2(0.0f, 0.0f);
	static bool check(lua_State* L, int idx) {
	    return (lua_gettop(L) >= idx+1) &&
		lua_isnumber(L,idx) && lua_isnumber(L,idx+1);
	}
	static ImVec2 get(lua_State* L, int idx) {
	    return ImVec2(
		float(lua_tonumber(L,idx)),
		float(lua_tonumber(L,idx+1))
	    );
	}
	static void push(lua_State* L, ImVec2 val) {
	    lua_pushnumber(L,lua_Number(val.x));
	    lua_pushnumber(L,lua_Number(val.y));
	}
    };

    template <> struct LuaType<ImVec4> {
	static constexpr ImVec4 default_value = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
	static bool check(lua_State* L, int idx) {
	    return (lua_gettop(L) >= idx+3) &&
		lua_isnumber(L,idx  ) && lua_isnumber(L,idx+1) &&
		lua_isnumber(L,idx+2) && lua_isnumber(L,idx+3) ;
	}
	static ImVec4 get(lua_State* L, int idx) {
	    return ImVec4(
		float(lua_tonumber(L,idx  )),
		float(lua_tonumber(L,idx+1)),
		float(lua_tonumber(L,idx+2)),
		float(lua_tonumber(L,idx+3))
	    );
	}
	static void push(lua_State* L, ImVec4 val) {
	    lua_pushnumber(L,lua_Number(val.x));
	    lua_pushnumber(L,lua_Number(val.y));
	    lua_pushnumber(L,lua_Number(val.z));
	    lua_pushnumber(L,lua_Number(val.w));
	}
    };

    template <> struct LuaType<ImTextureRef> {
	static constexpr ImTextureID default_value = 0;
	static bool check(lua_State* L, int idx) {
	    return lua_isinteger(L,idx);
	}
	static ImTextureRef get(lua_State* L, int idx) {
	    return ImTextureRef(ImTextureID(lua_tointeger(L, idx)));
	}

	template <class T> static void push(lua_State* L, T val) {
	    lua_pushinteger(L,lua_Integer(val.GetTexID()));
	}
    };

    enum UninitializedPointer { };
    enum NullPointer { };

    struct ArgBase {
	enum State { UNINITIALIZED, INITIALIZED, SET, INVALID };
	ArgBase() : state(UNINITIALIZED), is_pointer(false) { }
	bool OK() const {
	    return (state != UNINITIALIZED) && (state != INVALID);
	}
	State state;
	bool is_pointer;
    };

    template <class CTYPE, class LUATYPE=CTYPE> struct Arg : public ArgBase {
	Arg(lua_State* L, int idx) {
	    get(L, idx);
	}

	Arg(lua_State* L, int idx, CTYPE val) {
	    value = val;
	    state = INITIALIZED;
	    get(L, idx);
	}

	Arg(lua_State* L, int idx, NullPointer) {
	    state = INITIALIZED;
	    is_pointer = true;
	    get(L, idx);
	}

	Arg(lua_State* L, int idx, UninitializedPointer) {
	    state = UNINITIALIZED;
	    is_pointer = true;
	    get(L, idx);
	}

	Arg(CTYPE val) {
	    value = val;
	    state = INITIALIZED;
	}

	void push(lua_State* L) {
	    LuaType<LUATYPE>::push(L,value);
	}

	void push_if_set(lua_State* L) {
	    if(state == SET) {
		LuaType<LUATYPE>::push(L,value);
	    }
	}

	CTYPE* pointer() {
	    assert(is_pointer);
	    assert(state == INITIALIZED || state == SET);
	    return (state == SET) ? &value : nullptr;
	}

	CTYPE value = CTYPE(LuaType<LUATYPE>::default_value);

    protected:
	void get(lua_State* L, int idx) {
	    if(lua_isnoneornil(L,idx)) {
		return;
	    }
	    if(LuaType<LUATYPE>::check(L,idx)) {
		value = CTYPE(LuaType<LUATYPE>::get(L,idx));
		state = SET;
	    } else {
		state = INVALID;
	    }
	}
    };

    inline bool arglist_OK() { return true; }

    template <typename T, typename... Types> inline
    bool arglist_OK(const T& arg1, Types... args) {
	return arg1.OK() && arglist_OK(args...);
    }

    inline std::string arglist_missing(const std::vector<ArgBase>& args) {
	std::string missing;
	int i = 0;
	for(const ArgBase& arg : args) {
	    if(arg.state == ArgBase::UNINITIALIZED) {
		if(missing == "") {
		    missing = std::to_string(i+1);
		} else {
		    missing += ("," + std::to_string(i+1));
		}
	    }
	    ++i;
	}
	if(missing == "") {
	    missing = "<none>";
	}
	return "missing:" + missing;
    }

    inline std::string arglist_invalid(const std::vector<ArgBase>& args) {
	std::string invalid;
	int i = 0;
	for(const ArgBase& arg : args) {
	    if(arg.state == ArgBase::INVALID) {
		if(invalid == "") {
		    invalid = std::to_string(i+1);
		} else {
		    invalid += ("," + std::to_string(i+1));
		}
	    }
	    ++i;
	}
	if(invalid == "") {
	    invalid = "<none>";
	}
	return "invalid:" + invalid;
    }

    inline std::string arglist_dump(lua_State* L) {
	std::string result = "Lua stack:";
	for(int i=1; i<=lua_gettop(L); ++i) {
	    result += std::to_string(i);
	    result += ":";
	    int type = lua_type(L,i);
	    if(type == LUA_TNUMBER) {
		result += (lua_isinteger(L,i) ? "int" : "float");
	    } else {
		result += lua_typename(L, type);
	    }
	    result += " ";
	}
	return result;
    }
}

#define LUAWRAP_DECLARE_GLOBAL_CONSTANT(L,C) \
    lua_pushinteger(L,C);                    \
    lua_setglobal(L,#C)

#define LUAWRAP_DECLARE_FUNCTION(L,F) \
    lua_pushliteral(L,#F);  \
    lua_pushcfunction(L,F); \
    lua_settable(L,-3);

#define LUAWRAP_CHECK_ARGS(...) \
   if(!arglist_OK(__VA_ARGS__)) {	  	             \
       std::string missing = arglist_missing({__VA_ARGS__}); \
       std::string invalid = arglist_invalid({__VA_ARGS__}); \
       std::string stack   = arglist_dump(L);                \
       return luaL_error(L, "%s\n%s %s\n%s", proto, missing.c_str(), invalid.c_str(), stack.c_str()); \
   }

#endif
