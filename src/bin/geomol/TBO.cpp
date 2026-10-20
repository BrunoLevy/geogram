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

#include "TBO.h"

namespace GEO {

#ifdef GEO_TBO_SUPPORTED
    GLint TextureBufferObject::max_TBO_size_ = 0;
#else
    GLint TextureBufferObject::max_texture_size_ = 0;
#endif

    void TextureBufferObject::initialize() {
#ifdef GEO_TBO_SUPPORTED
	    if(max_TBO_size_ == 0) {
		GEO::Logger::out("TBO") << "Using true TBO" << std::endl;
		glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &max_TBO_size_);
		GEO::Logger::out("TBO") << "Max TBO size: " << max_TBO_size_
					<< " pixels" << std::endl;
	    }
#else
	    if(max_texture_size_ == 0) {
		GEO::Logger::out("TBO") << "Emulating TBO with texture"
					<< std::endl;
		glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &max_texture_size_);
		GEO::Logger::out("TBO") << "Max texture size: "
					<< max_texture_size_
					<< " pixels" << std::endl;
	    }
#endif
    }
}
