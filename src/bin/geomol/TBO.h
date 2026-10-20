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
#include <geogram/basic/memory.h>
#include <geogram/basic/logger.h>
#include <vector>
#include <typeinfo>

//#define GEO_TBO_SUPPORTED

#ifdef GEO_OS_EMSCRIPTEN
#include <GLES3/gl3.h>
   #ifdef GEO_TBO_SUPPORTED
   #undef GEO_TBO_SUPPORTED
   #endif
#endif

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
     *    int  i = texelFetch(my_iTBO, index).x;
     *  \endcode
     *
     */
    class TextureBufferObject {
    public:
        TextureBufferObject() {
	}

	~TextureBufferObject() {
	    reset();
	}

	bool initialized() const {
	    return TBO_ != 0;
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
#ifndef GEO_TBO_SUPPORTED
	    width_ = 0;
	    height_ = 0;
#endif
	}

	// Note: there should be an active texture unit when calling this
	// function the 1st time.
	template <class T> void create_or_update(index_t nb, const T* data) {
	    initialize(); // gets max TBO size or max texture size
#ifdef GEO_TBO_SUPPORTED
	    geo_assert(GLint(nb) < max_TBO_size_);
	    update_or_check_buffer_object(
		VBO_, GL_ARRAY_BUFFER,
		nb * sizeof(T), data,
		true
	    );
	    if(TBO_ == 0) {
		glGenTextures(1, &TBO_);
		glBindTexture(GL_TEXTURE_BUFFER, TBO_);
		glTexBuffer(GL_TEXTURE_BUFFER, internal_format(T()), VBO_);
	    }
#else
	    if(TBO_ == 0) {
		glGenTextures(1, &TBO_);
	    }
	    glBindTexture(GL_TEXTURE_2D, TBO_);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	    GLsizei width = GLsizei(nb);
	    GLsizei height = 1;
	    if(nb > 1024) {
		width = GLsizei(::sqrt(nb));
		height = GLsizei(nb) / width;
		while(width*height < GLsizei(nb)) {
		    ++height;
		}
	    }
	    glPixelStorei(GL_PACK_ALIGNMENT, 1);
	    if(width_ != width || height_ != height) {
		glTexImage2D(
		    GL_TEXTURE_2D,
		    0, // level
		    GLint(internal_format(T())),
		    width, height,
		    0, // border
		    format(T()), type(T()),
		    nullptr
		);
		width_ = width;
		height_ = height;
	    }
	    if(GLsizei(nb) == width*height) {
		glTexSubImage2D(
		    GL_TEXTURE_2D,
		    0,    // level
		    0, 0, // xoffset, yoffset
		    width, height,
		    format(T()), type(T()),
		    data
		);
	    } else {
		// If texture's last row is slightly larger than input
		// data, avoid reading past the end of data by transfering
		// the (partial) last row:
		glTexSubImage2D(
		    GL_TEXTURE_2D,
		    0,                      // level
		    0, 0,                   // xoffset, yoffset
		    width, height-1,        // width, height
		    format(T()), type(T()),
		    data
		);
		glTexSubImage2D(
		    GL_TEXTURE_2D,
		    0,                        // level
		    0, height-1,              // xoffset, yoffset
		    (GLsizei(nb) % width), 1, // width, height
		    format(T()), type(T()),
		    data + (height-1)*width
		);
	    }
#endif
	}

	template <class T> void create_or_update(const std::vector<T>& v) {
	    create_or_update(index_t(v.size()), v.data());
	}

	template <class T> void create_or_update(const GEO::vector<T>& v) {
	    create_or_update(v.size(), v.data());
	}

	void bind(GLuint texture_unit) {
#ifdef GEO_TBO_SUPPORTED
	    geo_debug_assert(VBO_ != 0 && TBO_ != 0);
	    glActiveTexture(texture_unit);
	    glBindTexture(GL_TEXTURE_BUFFER, TBO_);
#else
	    geo_debug_assert(TBO_ != 0);
	    glActiveTexture(texture_unit);
	    glBindTexture(GL_TEXTURE_2D, TBO_);
#endif
	}

    protected:
	/**
	 * \brief Queries max tbo size and max texture size
	 */
	static void initialize();

	// Just to map geogram types to OpenGL storage
	static GLuint internal_format(uint32_t) { return GL_R32UI;   }
	static GLuint internal_format(int32_t)  { return GL_R32I;    }
	static GLuint internal_format(vec2i)    { return GL_RG32I;   }
	static GLuint internal_format(vec3i)    { return GL_RGB32I;  }
	static GLuint internal_format(vec4i)    { return GL_RGBA32I; }
	static GLuint internal_format(float)    { return GL_R32F;    }
	static GLuint internal_format(vec2f)    { return GL_RG32F;   }
	static GLuint internal_format(vec3f)    { return GL_RGB32F;  }
	static GLuint internal_format(vec4f)    { return GL_RGBA32F; }

	static GLuint format(uint32_t) { return GL_RED_INTEGER;  }
	static GLuint format(int32_t)  { return GL_RED_INTEGER;  }
	static GLuint format(vec2i)    { return GL_RG_INTEGER;   }
	static GLuint format(vec3i)    { return GL_RGB_INTEGER;  }
	static GLuint format(vec4i)    { return GL_RGBA_INTEGER; }
	static GLuint format(float)    { return GL_RED;          }
	static GLuint format(vec2f)    { return GL_RG;           }
	static GLuint format(vec3f)    { return GL_RGB;          }
	static GLuint format(vec4f)    { return GL_RGBA;         }

	static GLuint type(uint32_t)   { return GL_UNSIGNED_INT; }
	static GLuint type(int32_t)    { return GL_INT;          }
	static GLuint type(vec2i)      { return GL_INT;          }
	static GLuint type(vec3i)      { return GL_INT;          }
	static GLuint type(vec4i)      { return GL_INT;          }
	static GLuint type(float)      { return GL_FLOAT;        }
	static GLuint type(vec2f)      { return GL_FLOAT;        }
	static GLuint type(vec3f)      { return GL_FLOAT;        }
	static GLuint type(vec4f)      { return GL_FLOAT;        }


    private:
	GLuint TBO_ = 0;
	GLuint VBO_ = 0;
#ifdef GEO_TBO_SUPPORTED
	static GLint max_TBO_size_; // in texels.
#else
	static GLint max_texture_size_; // in texels.
	GLsizei width_ = 0;
	GLsizei height_ = 0;
#endif
    };
}

#endif
