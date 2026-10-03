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

#ifndef TBO_H
#define TBO_H

#include <geogram/basic/geometry.h>
#include <geogram_gfx/basic/common.h>
#include <geogram_gfx/GLUP/GLUP.h>

namespace GEO {

    /**
     * \brief Easy to use wrapper around TextureBufferObject
     * \details TextureBufferObject is a portable way of sending large arrays
     *  of datas to shaders. It can be used when OpenGL 4.3's SSBOs (Shared
     *  Storage Buffer Objects) are not available, as for instance in
     *  OpenGL 4.1 which is the highest version of OpenGL supported by Macs.
     *  A Texture Buffer Object corresponds to an OpenGL texture that refers
     *  to some data stored in a Vertex Buffer Object.
     *  The Texture Buffer Object is manipulated through an OpenGL texture ID,
     *  returned by TBO(), that can be bound to a texture unit and accessed
     *  through texelFetch() or itexelTetch() in shaders.
     *  The TextureBufferObject class manages a Vertex Buffer Object / Texture
     *  Buffer Object pair.
     *
     *  Creation:
     *  \code
     *    std::vector<vec3f> V = ...;
     *    glActiveTexture(texture_unit);
     *    tbo.create_or_update(V);
     *  \endcode
     *
     *  Usage (CPU side):
     *  \code
     *    glActiveTexture(texture_unit);
     *    glBindTexture(GL_TEXTURE_BUFFER, tbo.TBO());
     *    GLSL::set_program_uniform_by_name(
     *	      program_, "my_TBO", texture_unit // note: texture_unit, not TBO!
     *	  );
     *  \endcode
     *
     *  Usage (GPU shader size):
     *  \code
     *    uniform samplerBuffer  my_TBO;  // float,vec2,vec3,vec4
     *    uniform isamplerBuffer my_iTBO; // int, uint, vec2i, vec3i, vec4i
     *    ...
     *    int index = ...;
     *    vec3 p = texelFetch(my_TBO, index).xyz;
     *    int i =  itexelFetch(my_iTBO, index).x;
     *  \endcode
     *
     */
    class TextureBufferObject {
    public:
        TextureBufferObject() : VBO_(0), TBO_(0) {
	}

	~TextureBufferObject() {
	    reset();
	}

	GLuint VBO() const {
	    return VBO_;
	}

	GLuint TBO() const {
	    return TBO_;
	}

	void reset() {
	    if(TBO_ != 0) {
		glDeleteTextures(1, &TBO_);
		TBO_ = 0;
	    }
	    if(VBO_ != 0) {
		glDeleteBuffers(1,&VBO_);
		VBO_ = 0;
	    }
	}

	// Note: there should be an active texture unit when calling this
	// function the 1st time.
	template <class T> void create_or_update(
	    index_t nb, const T* data
	) {
	    if(max_TBO_size_ == 0) {
		glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &max_TBO_size_);
	    }
	    geo_assert(nb < max_TBO_size_);
	    update_or_check_buffer_object(
		VBO_, GL_ARRAY_BUFFER,
		nb * sizeof(T), data,
		true
	    );
	    if(TBO_ == 0) {
		glGenTextures(1, &TBO_);
		glBindTexture(GL_TEXTURE_BUFFER, TBO_);
		glTexBuffer(GL_TEXTURE_BUFFER, format(T()), VBO_);
	    }
	}

	// Note: there should be an active texture unit when calling this
	// function the 1st time.
	template <class T> void create_or_update(const std::vector<T>& V) {
	    create_or_update(V.size(), V.data());
	}

	void bind(GLuint texture_unit) {
	    geo_debug_assert(VBO_ != 0 && TBO_ != 0);
	    glActiveTexture(texture_unit);
	    glBindTexture(GL_TEXTURE_BUFFER, TBO_);
	}

    protected:
	// Just to map geogram types to OpenGL storage
	static GLuint format(uint32_t) { return GL_R32UI;   }
	static GLuint format(int32_t)  { return GL_R32I;    }
	static GLuint format(vec2i)    { return GL_RG32I;   }
	static GLuint format(vec3i)    { return GL_RGB32I;  }
	static GLuint format(vec4i)    { return GL_RGBA32I; }
	static GLuint format(float)    { return GL_R32F;    }
	static GLuint format(vec2f)    { return GL_RG32F;   }
	static GLuint format(vec3f)    { return GL_RGB32F;  }
	static GLuint format(vec4f)    { return GL_RGBA32F; }

    private:
	GLuint VBO_;
	GLuint TBO_;
	static GLint max_TBO_size_; // in texels.
    };
}

#endif
