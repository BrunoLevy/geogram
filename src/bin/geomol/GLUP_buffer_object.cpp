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

#include "GLUP_buffer_object.h"
#include <geogram_gfx/basic/GLSL.h>
#include <geogram/basic/command_line.h>

namespace GLUP {

    bool  BufferObject::class_initialized_ = false;
    GLint BufferObject::max_TBO_size_ = 0;
    GLint BufferObject::max_texture_size_ = 0;
    BufferObject::Type BufferObject::default_type_ = UNINITIALIZED;
    GLuint BufferObject::active_unit_ = 0;

    /**************** reset() ****************************/

    void BufferObject::reset() {
	switch(type_) {
	case UNINITIALIZED:
	    break;
	case TEXTURE_2D:
	    reset_TEXTURE_2D();
	    break;
	case TBO:
	    reset_TBO();
	    break;
	case SSBO:
	    reset_SSBO();
	    break;
	}
	type_ = UNINITIALIZED;
    }


    void BufferObject::reset_TEXTURE_2D() {
	if(texture_ != 0) {
	    glDeleteTextures(1, &texture_);
	    texture_ = 0;
	}
	width_ = 0;
	height_ = 0;
    }

    void BufferObject::reset_TBO() {
	if(texture_ != 0) {
	    glDeleteTextures(1, &texture_);
	    texture_ = 0;
	}
	if(BO_ != 0) {
	    glDeleteBuffers(1,&BO_);
	    BO_ = 0;
	}
    }

    void BufferObject::reset_SSBO() {
#ifdef GEO_SSBO_SUPPORTED
	if(BO_ != 0) {
	    glDeleteBuffers(1,&BO_);
	    BO_ = 0;
	}
#else
	geo_assert_not_reached;
#endif
    }

    /**************** active_unit() ****************************/

    void BufferObject::active_unit(GLuint unit) {
	active_unit_ = unit;
	switch(default_type()) {
	case UNINITIALIZED:
	    geo_assert_not_reached;
	    break;
	case TEXTURE_2D:
	    active_unit_TEXTURE_2D(unit);
	    break;
	case TBO:
	    active_unit_TBO(unit);
	    break;
	case SSBO:
	    active_unit_SSBO(unit);
	    break;
	}
    }

    void BufferObject::active_unit_TEXTURE_2D(GLuint unit) {
	glActiveTexture(GL_TEXTURE0 + unit);
    }

    void BufferObject::active_unit_TBO(GLuint unit) {
	glActiveTexture(GL_TEXTURE0 + unit);
    }

    void BufferObject::active_unit_SSBO(GLuint unit) {
	geo_argused(unit);
	// Does nothing, unit is memorized as active_unit_,
	// then bind() does the job.
    }

    /**************** bind() ****************************/

    void BufferObject::bind(GLuint unit) {
	if(unit != 0) {
	    active_unit(unit);
	} else {
	    unit = active_unit_;
	}
	switch(type_) {
	case UNINITIALIZED:
	    geo_assert_not_reached;
	    break;
	case TEXTURE_2D:
	    bind_TEXTURE_2D(unit);
	    break;
	case TBO:
	    bind_TBO(unit);
	    break;
	case SSBO:
	    bind_SSBO(unit);
	    break;
	}
    }

    void BufferObject::bind_TEXTURE_2D(GLuint unit) {
	geo_debug_assert(texture_ != 0);
	geo_argused(unit);
	glBindTexture(GL_TEXTURE_2D, texture_);
    }

    void BufferObject::bind_TBO(GLuint unit) {
	geo_argused(unit);
#ifdef GEO_TBO_SUPPORTED
	geo_debug_assert(BO_ != 0 && texture_ != 0);
	glBindTexture(GL_TEXTURE_BUFFER, texture_);
#else
	geo_assert_not_reached;
#endif
    }

    void BufferObject::bind_SSBO(GLuint unit) {
#ifdef GEO_SSBO_SUPPORTED
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER,unit,BO_);
#else
	geo_argused(unit);
	geo_assert_not_reached;
#endif
    }

    /**************************************************************************/

    void BufferObject::initialize_class() {
	if(class_initialized_) {
	    return;
	}

	class_initialized_ = true;

#ifdef GEO_TBO_SUPPORTED
	glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &max_TBO_size_);
	GEO::Logger::out("GLUP") << "Max TBO size: " << max_TBO_size_
				 << " pixels" << std::endl;
#endif
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_texture_size_);
	GEO::Logger::out("GLUP") << "Max texture size: "
				 << max_texture_size_
				 << " pixels" << std::endl;


	std::string default_type_str = CmdLine::get_arg("gfx:buffers");
	if(default_type_str == "default") {
	    default_type_ = UNINITIALIZED;
	} else if(default_type_str == "TEXTURE_2D") {
	    default_type_ = TEXTURE_2D;
	} else if(default_type_str == "TBO") {
	    default_type_ = TBO;
	} else if(default_type_str == "SSBO") {
	    default_type_ = SSBO;
	} else {
	    Logger::err("GLUP") << "Invalid buffers type:" << default_type_str
				<< std::endl;
	    exit(-1);
	}

	#ifndef GEO_SSBO_SUPPORTED
	if(default_type_ == SSBO) {
	    Logger::err("GLUP") << "SSBO unsupported, switching to TBO"
				<< std::endl;
	    default_type_ = TBO;
	}
	#endif

	#ifndef GEO_TBO_SUPPORTED
	if(default_type_ == TBO) {
	    Logger::err("GLUP") << "TBO unsupported, switching to TEXTURE_2D"
				<< std::endl;
	    default_type_ = TEXTURE_2D;
	}
	#endif

	if(default_type_ == UNINITIALIZED) {
	    default_type_ = TEXTURE_2D;
#ifdef GEO_TBO_SUPPORTED
	    default_type_ = TBO;
#endif
#ifdef GEO_SSBO_SUPPORTED
	    default_type_ = SSBO;
#endif
	}

	switch(default_type_) {
	case UNINITIALIZED:
	    geo_assert_not_reached;
	    break;
	case TEXTURE_2D:
	    default_type_str = "TEXTURE_2D";
	    break;
	case TBO:
	    default_type_str = "TBO";
	    break;
	case SSBO:
	    default_type_str = "SSBO";
	    break;
	}
	Logger::out("GLUP") << "Buffers: using " << default_type_str
			    << std::endl;
    }

    BufferObject::Type BufferObject::default_type() {
	return default_type_;
    }

    void BufferObject::set_program_buffer_unit_by_name(
	GLuint program, const char* name, GLuint unit
    ) {
	switch(default_type()) {
	case UNINITIALIZED:
	    geo_assert_not_reached;
	    break;
	case TEXTURE_2D:
	case TBO:
	    // Note: needs to be a GLint, doew not work with GLuint (why?)
	    GLSL::set_program_uniform_by_name(program, name, GLint(unit));
	    break;
	case SSBO: {
#ifdef GEO_SSBO_SUPPORTED
	    GLuint block_index = glGetProgramResourceIndex(
		program, GL_SHADER_STORAGE_BLOCK, name
	    );
	    std::cerr << name << ":" << block_index << std::endl;
	    if(block_index == GLuint(-1)) {
		Logger::err("GLUP") << name << ": no such SSBO block"
				    << std::endl;
		return;
	    }
	    glShaderStorageBlockBinding(program, block_index, unit);
#else
	    geo_assert_not_reached;
#endif
	} break;
	}
    }

    /***************************************************************************/

    void BufferObject::register_GLSL_sources() {
	if(default_type() == TEXTURE_2D) {
	    GEO::GLSL::register_GLSL_include_file("GLUP/buffers.h",
            R"(
            #define glup_Buffer  sampler2D
            #define glup_iBuffer isampler2D
            #define glup_uBuffer usampler2D

            vec4 glup_fetch(in glup_Buffer B, in int idx) {
               int W = textureSize(B,0).x;
               return texelFetch(B, ivec2(idx%W,idx/W),0);
            }

            ivec4 glup_fetch(in glup_iBuffer B, in int idx) {
               int W = textureSize(B,0).x;
               return texelFetch(B, ivec2(idx%W,idx/W),0);
            }

            uvec4 glup_fetch(in glup_uBuffer B, in int idx) {
               int W = textureSize(B,0).x;
               return texelFetch(B, ivec2(idx%W,idx/W),0);
            }
            )"
            );
	} else if(default_type() == TBO) {
	  GEO::GLSL::register_GLSL_include_file("GLUP/buffers.h",
          R"(
          #define glup_Buffer  samplerBuffer
          #define glup_iBuffer isamplerBuffer
          #define glup_uBuffer usamplerBuffer

          vec4 glup_fetch(in samplerBuffer B, in int idx) {
             return texelFetch(B,idx);
          }

          ivec4 glup_fetch(in isamplerBuffer B, in int idx) {
             return texelFetch(B,idx);
          }

          uvec4 glup_fetch(in usamplerBuffer B, in int idx) {
             return texelFetch(B,idx);
          }

          )"
          );
	} else if(default_type() == SSBO) {
	  GEO::GLSL::register_GLSL_include_file("GLUP/buffers.h",
          R"(
          #define GLUP_SSBO
          #define glup_Buffer  float
          #define glup_iBuffer int
          #define glup_uBuffer uint
          vec4 glup_fetch(in glup_Buffer B, in int idx) {
             return vec4(0.0,0.0,0.0,0.0);
          }

          ivec4 glup_fetch(in glup_iBuffer B, in int idx) {
             return ivec4(0,0,0,0);
          }

          uvec4 glup_fetch(in glup_uBuffer B, in int idx) {
             return uvec4(0u,0u,0u,0u);
          }
          )"
	  );
	}

    }

    /***************************************************************************/

}
