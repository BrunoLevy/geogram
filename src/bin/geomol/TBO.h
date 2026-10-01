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

    template <class T> class TextureBufferObject {
    public:
        TextureBufferObject(index_t size=0) : data_(size), VBO_(0), TBO_(0) {
	}

	~TextureBufferObject() {
	    if(VBO_ != 0) {
		glDeleteBuffers(1,&VBO_);
		VBO_ = 0;
	    }
	    if(TBO_ != 0) {
		glDeleteTextures(1, &TBO_);
		TBO_ = 0;
	    }
	}

	GLuint VBO() const {
	    return VBO_;
	}

	GLuint TBO() const {
	    return TBO_;
	}

	const vector<T>& data() const {
	    return data_;
	}

	vector<T>& data() {
	    return data_;
	}

	// Note: there should be an active texture unit (which is the
	// case when called from bind())
	void update() {
	    update_or_check_buffer_object(
		VBO_, GL_ARRAY_BUFFER,
		data_.size() * sizeof(T), data_.data(),
		true
	    );
	    if(TBO_ == 0) {
		glGenTextures(1, &TBO_);
		glBindTexture(GL_TEXTURE_BUFFER, TBO_);
		glTexBuffer(GL_TEXTURE_BUFFER, format(T()), VBO_);
	    }
	}

	void bind(GLuint texture_unit) {
	    glActiveTexture(texture_unit);
	    if(VBO_ == 0 || TBO_ == 0) {
		update();
	    }
	    glBindTexture(GL_TEXTURE_BUFFER, TBO_);
	}

    protected:
	// Just to map geogram types to OpenGL storage
	static GLuint format(uint32_t) { return GL_R32I;    }
	static GLuint format(int32_t)  { return GL_R32I;    }
	static GLuint format(vec2i)    { return GL_RG32I;   }
	static GLuint format(vec3i)    { return GL_RGB32I;  }
	static GLuint format(vec4i)    { return GL_RGBA32I; }
	static GLuint format(float)    { return GL_R32F;    }
	static GLuint format(vec2f)    { return GL_RG32F;   }
	static GLuint format(vec3f)    { return GL_RGB32F;  }
	static GLuint format(vec4f)    { return GL_RGBA32F; }

    private:
	vector<T> data_;
	GLuint VBO_;
	GLuint TBO_;
    };
}

#endif
