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

#ifndef GEOGRAM_LUA_LUA_VEC_MAT
#define GEOGRAM_LUA_LUA_VEC_MAT

#include <geogram/basic/common.h>
#include <geogram/lua/lua.h>
#include <geogram/basic/assert.h>
#include <geogram/basic/argused.h>
#include <geogram/basic/numeric.h>
#include <geogram/basic/vecg.h>
#include <geogram/basic/matrix.h>

/**
 * \file geogram/lua/lua_vec_mat.h
 * \brief Functions to exchange vec2,vec3,vec4 and mat4 objects
 *  between Lua and Geogram
 */

namespace GEO {

    /**
     * \brief Converts a lua object into a value
     * \tparam T type of the value
     * \param[in] L a pointer to the Lua state
     * \param[in] index the index of the object in thestack
     * \param[out] result a reference to the read value
     * \retval true if the conversion was successful
     * \retval false otherwise (LUA type did not match)
     */
    template <class T> inline bool lua_toveccomp(
	lua_State* L, int index, T& result
    ) {
	geo_argused(L);
	geo_argused(index);
	geo_argused(result);
	geo_assert_not_reached;
    }

    template<> inline bool lua_toveccomp<double>(
	lua_State* L, int index, double& result
    ) {
	if(lua_type(L,index) == LUA_TNUMBER) {
	    result = lua_tonumber(L,index);
	    return true;
	}
	return false;
    }

    template<> inline bool lua_toveccomp<float>(
	lua_State* L, int index, float& result
    ) {
	if(lua_type(L,index) == LUA_TNUMBER) {
	    result = float(lua_tonumber(L,index));
	    return true;
	}
	return false;
    }

    template<> inline bool lua_toveccomp<Numeric::int32>(
	lua_State* L, int index, Numeric::int32& result
    ) {
	if(lua_type(L,index) == LUA_TNUMBER && lua_isinteger(L,index)) {
	    result = GEO::Numeric::int32(lua_tointeger(L,index));
	    return true;
	}
	return false;
    }


    /**
     * \brief Converts a lua object into a Geogram vec2,vec3 or vec4
     * \tparam N dimension of the vector
     * \tparam T type of the components
     * \param[in] L a pointer to the Lua state
     * \param[in] index the index of the object in thestack
     * \param[out] result a reference to the converted vector
     * \retval true if the conversion was successful
     * \retval false otherwise (mtype does not match, or object on
     *  the stack is not an integer-indexed table of numbers of the
     *  correct size).
     */
    template<unsigned int N, class T> inline bool lua_tovec(
	lua_State* L, int index, ::GEO::vecng<N,T>& result
    ) {
	if(!lua_istable(L,index)) {
	    return false;
	}

	index_t cur = 0;
	bool ok = true;

	for(lua_Integer i=1; lua_geti(L,index,i) != LUA_TNIL; ++i) {
	    if(cur < N) {
		ok = ok && lua_toveccomp(L,-1,result[cur]);
	    }
	    ++cur;
	    lua_pop(L,1);
	}
	lua_pop(L,1); // lua_geti() pushes smthg on the stack
	// even for the last round of the loop !

	return(ok && cur == index_t(N));
    }

    /**
     * \brief Converts a Lua object into a Geogram mat2, mat3 or mat4
     * \tparam N dimension of the vector
     * \tparam T type of the coefficients
     * \param[in] L a pointer to the Lua state
     * \param[in] index the index of the object in thestack
     * \param[out] result a reference to the converted matrix
     * \retval true if the conversion was successful
     * \retval false otherwise (mtype does not match, or object
     *  is not a table of the correct size).
     */
    template<unsigned int N, class T> inline bool lua_tomat(
	lua_State* L, int index, ::GEO::Matrix<N,T>& result
    ) {

	if(!lua_istable(L,index)) {
	    return false;
	}

	index_t cur = 0;
	bool ok = true;

	for(lua_Integer i=1; lua_geti(L,index,i) != LUA_TNIL; ++i) {
	    ::GEO::vecng<N,T> row;
	    if(cur < N) {
		if(lua_tovec(L,-1,row)) {
		    for(index_t j=0; j<index_t(N); ++j) {
			result(cur,j) = row[j];
		    }
		} else {
		    ok = false;
		}
	    }
	    ++cur;
	    lua_pop(L,1);
	}
	lua_pop(L,1); // lua_geti() pushes smthg on the stack
	// even for the last round of the loop !

	return(ok && cur == index_t(N));
    }

    /*******************************************************************/

    /**
     * \brief Pushes a vector component onto the Lua stack
     * \details Specializations need to be declared, this generic version
     *  throws an assertion failure
     * \param[in] L the Lua stack
     * \param[in] T val the value to be pushed
     */
    template <class T> inline void lua_pushveccomp(lua_State* L, T val) {
	geo_argused(L);
	geo_argused(val);
	geo_assert_not_reached;
    }

    template<> inline void lua_pushveccomp(lua_State* L, double val) {
	lua_pushnumber(L, val);
    }

    template<> inline void lua_pushveccomp(lua_State* L, float val) {
	lua_pushnumber(L, double(val));
    }

    template<> inline void lua_pushveccomp(
	lua_State* L, Numeric::int32 val
    ) {
	lua_pushinteger(L, lua_Integer(val));
    }

    /**
     * \brief Pushes a vector onto the Lua stack
     * \param[in] L the Lua stack
     * \param[in] V a vec2,vec3,vec4,vec2i,vec3i or vec4i
     */
    template <unsigned int N, class T> inline void lua_push(
	lua_State* L, const ::GEO::vecng<N,T>& V
    ) {
	lua_createtable(L, int(N), 0);
	for(index_t i=0; i<index_t(N); ++i) {
	    lua_pushveccomp(L,V[i]);
	    lua_seti(L,-2,lua_Integer(i+1)); // indices start from 1 in Lua
	}
    }

    /**
     * \brief Pushes a matrix onto the Lua stack
     * \param[in] L the Lua stack
     * \param[in] M a mat2,mat3,mat4,mat2i,mat3i or mat4i
     */
    template <unsigned int N, class T> inline void lua_push(
	lua_State* L, const ::GEO::Matrix<N,T>& M
    ) {
	lua_createtable(L, int(N), 0);
	for(index_t i=0; i<index_t(N); ++i) {
	    lua_createtable(L, int(N), 0);
	    for(index_t j=0; j<index_t(N); ++j) {
		lua_pushveccomp(L,M(i,j));
		lua_seti(L,-2,lua_Integer(j+1)); // Idces start from 1 in Lua
	    }
	    lua_seti(L,-2,lua_Integer(i+1)); // Idem...
	}
    }

}


#endif
