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

#ifndef GLUP_BUFFER_OBJECT_H
#define GLUP_BUFFER_OBJECT_H

#include <geogram/basic/geometry.h>
#include <geogram_gfx/basic/common.h>
#include <geogram_gfx/GLUP/GLUP.h>
#include <geogram/basic/memory.h>
#include <geogram/basic/logger.h>
#include <vector>
#include <typeinfo>

#define GEO_TBO_SUPPORTED
#define GEO_SSBO_SUPPORTED

// Note: for molecules with 2.5M atoms, it is FASTER
// using standard texture than TBOs (with GEO_TBO_SUPPORTED *deactivated*
// It seems that with TBOs it swaps between main memory and GPU
// ... but for smaller molecules, TBOs seems worth it:
// 1HTQ (900K atoms)
// textures: 15   fps
// TBOS:     25.5 fps
//
// TODO: test with SSBOs

#ifdef GEO_OS_EMSCRIPTEN
   #ifdef GEO_TBO_SUPPORTED
   #undef GEO_TBO_SUPPORTED
   #endif
   #ifdef GEO_SSBO_SUPPORTED
   #undef GEO_SSBO_SUPPORTED
   #endif
#endif

#ifdef GEO_OS_APPLE
   #ifdef GEO_SSBO_SUPPORTED
   #undef GEO_SSBO_SUPPORTED
   #endif
#endif

namespace GLUP {
    using namespace GEO;

    /**
     * \brief Wrapper class around the different type of buffers used by the GPU.
     * \details Wraps either a 2D texture, a TBO (Texture Buffer Object) or an
     *  SSBO, depending on what is supported and on user-specified gfx:buffers
     *  command line argument.
     */
    class BufferObject {
    public:
	enum Type {UNINITIALIZED, TEXTURE_2D, TBO, SSBO};

	/**
	 * \brief BufferObject constructor
	 * \details Constructs an uninitialized BufferObject
	 */
        BufferObject() {
	}

	/**
	 * \brief BufferObject destructor
	 */
	~BufferObject() {
	    reset();
	}

	/**
	 * \brief Gets the type of a BufferObject
	 * \return one of UNINITIALIZED, TEXTURE_2D, TBO, SSBO
	 */
	Type type() const {
	    return type_;
	}

	/**
	 * \brief Tests whether a BufferObject is initialized
	 * \retval true if this BufferObject is initialized, that is, has
	 *  associated data in GPU memory
	 * \retval false otherwise
	 */
	bool initialized() const {
	    return type_ != UNINITIALIZED;
	}

	/**
	 * \brief Registers GLSL sources for using portable wrappers around
	 *  buffers.
	 * \TODOC
	 */
	static void register_GLSL_sources();

	/**
	 * \brief Initializes static class variables
	 * \details Needs to be called after OpenGL is initialized and before
	 *  any BufferObject is created.
	 */
	static void initialize_class();

	/**
	 * \brief Gets the default type for BufferObjects
	 * \return one of TEXTURE_2D, TBO, SSBO
	 */
	static Type default_type();

	/**
	 * \brief Resets a BufferObject
	 * \details Releases all memory and objects allocated on the GPU by this
	 *  BufferObject.
	 */
	void reset();

	/**
	 * \brief Creates an uninitialized BufferObject or updates an initialized
	 *  one with new data.
	 * \param[in] nb number of items
	 * \param[in] data a pointer to a contiguous array of \p nb items
	 * \param[in] unit a unit this buffer will be bound to, or 0 if currently
	 *  active unit should be used instead
	 * \see active_unit()
	 */
	template <class T> void create_or_update(
	    index_t nb, const T* data, GLuint unit=0
	) {
	    if(unit != 0) {
		active_unit(unit);
	    }
	    Type type = default_type();
	    switch(type) {
	    case UNINITIALIZED:
		geo_assert_not_reached;
		break;
	    case TEXTURE_2D:
		create_or_update_TEXTURE_2D(nb, data);
		break;
	    case TBO:
		create_or_update_TBO(nb,data);
		break;
	    case SSBO:
		create_or_update_SSBO(nb,data);
		break;
	    }
	    if(unit != 0) {
		active_unit(0);
	    }
	}

	/**
	 * \brief Creates an uninitialized BufferObject or updates an initialized
	 *  one with new data.
	 * \param[in] v a STL vector with the data
	 * \param[in] unit a unit this buffer will be bound to, or 0 if currently
	 *  active unit should be used instead
	 * \see active_unit()
	 */
	template <class T> void create_or_update(
	    const std::vector<T>& v, index_t unit=0
	) {
	    create_or_update(index_t(v.size()), v.data(), unit);
	}

	/**
	 * \brief Creates an uninitialized BufferObject or updates an initialized
	 *  one with new data.
	 * \param[in] v a geogram vector with the data
	 * \param[in] unit a unit this buffer will be bound to, or 0 if currently
	 *  active unit should be used instead
	 * \see active_unit()
	 */
	template <class T> void create_or_update(
	    const GEO::vector<T>& v, index_t unit=0
	) {
	    create_or_update(v.size(), v.data(), unit);
	}

	/**
	 * \brief Active_Units a unit
	 * \param[in] unit a unit id (either a texture unit or a buffer unit,
	 *  depending on default buffer type).
	 */
	static void active_unit(GLuint unit);

	/**
	 * \brief Binds this buffer to a unit.
	 * \details Calls active_unit(unit).
	 * \param[in] unit a unit id (either a texture unit or a buffer unit,
	 *  depending on default buffer type). If 0, use currently active unit.
	 */
	void bind(GLuint unit=0);

	/**
	 * \brief Sets a program uniform buffer unit
	 * \details \p unit is interpreted either as a texture unit or a SSBO
	 *  binding point, depending on default_type().
	 * \param[in] program the OpenGL program id
	 * \param[in] name a string with the name of the uniform variable
	 * \param[in] unit the unit
	 */
	static void set_program_buffer_unit_by_name(
	    GLuint program, const char* name, GLuint unit
	);

    protected:
	static void active_unit_TEXTURE_2D(GLuint unit);
	static void active_unit_TBO(GLuint unit);
	static void active_unit_SSBO(GLuint unit);

	void reset_TEXTURE_2D();
	void reset_TBO();
	void reset_SSBO();

	void bind_TEXTURE_2D(GLuint unit);
	void bind_TBO(GLuint unit);
	void bind_SSBO(GLuint unit);

	template <class T> void create_or_update_TEXTURE_2D(
	    index_t nb, const T* data
	) {
	    geo_debug_assert(type_ == TEXTURE_2D || type_ == UNINITIALIZED);
	    type_ = TEXTURE_2D;
	    if(texture_ == 0) {
		glGenTextures(1, &texture_);
	    }
	    glBindTexture(GL_TEXTURE_2D, texture_);
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
	}

	template <class T> void create_or_update_TBO(
	    index_t nb, const T* data
	) {
#ifdef GEO_TBO_SUPPORTED
	    geo_debug_assert(type_ == TBO || type_ == UNINITIALIZED);
	    type_ = TBO;
	    geo_assert(GLint(nb) < max_TBO_size_);
	    update_or_check_buffer_object(
		BO_, GL_ARRAY_BUFFER,
		nb * sizeof(T), data,
		true
	    );
	    if(texture_ == 0) {
		glGenTextures(1, &texture_);
		glBindTexture(GL_TEXTURE_BUFFER, texture_);
		glTexBuffer(GL_TEXTURE_BUFFER, internal_format(T()), BO_);
	    }
#else
	    geo_argused(nb);
	    geo_argused(data);
	    geo_assert_not_reached;
#endif
	}

	template <class T> void create_or_update_SSBO(
	    index_t nb, const T* data
	) {
#ifdef GEO_SSBO_SUPPORTED
	    geo_debug_assert(type_ == TBO || type_ == UNINITIALIZED);
	    type_ = TBO;
	    geo_assert(GLint(nb) < max_TBO_size_);
	    update_or_check_buffer_object(
		BO_, GL_SHADER_STORAGE_BUFFER,
		nb * sizeof(T), data,
		true
	    );
#else
	    geo_argused(nb);
	    geo_argused(data);
	    geo_assert_not_reached;
#endif
	}

    protected:
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
	Type type_ = UNINITIALIZED;
	GLuint texture_ = 0;
	GLuint BO_ = 0;
	GLsizei width_ = 0;
	GLsizei height_ = 0;

	static bool class_initialized_;
	static GLint max_TBO_size_;     // in texels.
	static GLint max_texture_size_; // in texels.
	static Type default_type_;
	static GLuint active_unit_;
    };
}

#endif
