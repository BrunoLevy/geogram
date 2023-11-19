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

#include <geogram/basic/debug_stream.h>
#include <geogram/basic/string.h>
#include <geogram/basic/logger.h>

namespace GEO {
    
    DebugStream::DebugStream(
        const std::string& name, index_t id
    ) :
        out_(filename(name,id).c_str()),
        filename_(filename(name,id).c_str()),
        nv_(0) {
        Logger::out("Debug") << "Opening debug stream:"
                             << filename_ << std::endl;
    }

    DebugStream::~DebugStream() {
        Logger::out("Debug") << "Closing debug stream:"
                             << filename_ << std::endl;
    }
    
    std::string DebugStream::filename(const std::string& name, index_t id) {
        if(id == index_t(-1)) {
            return name + ".obj";
        }
        return name + String::format("_%05d.obj",int(id));
    }

    void DebugStream::add_point(const vec3& p) {
        out_ << "v " << p << std::endl;
        ++nv_;
    }

    void DebugStream::add_point(const vec2& p) {
        out_ << "v " << p << " " << 0.0 << std::endl;
        ++nv_;
    }
    
    void DebugStream::add_segment(const vec3& p1, const vec3& p2) {
        add_point(p1);
        add_point(p2);
        out_ << "l " << nv_- 1 << " " << nv_ << std::endl;
    }

    void DebugStream::add_segment(const vec2& p1, const vec2& p2) {
        add_point(p1);
        add_point(p2);
        out_ << "l " << nv_- 1 << " " << nv_ << std::endl;
    }
    
    void DebugStream::add_triangle(
        const vec3& p1, const vec3& p2, const vec3& p3
    ) {
        add_point(p1);
        add_point(p2);
        add_point(p3);
        out_ << "f " << nv_ - 2 << " " << nv_ - 1 << " " << nv_ << std::endl;
    }

    void DebugStream::add_triangle(
        const vec2& p1, const vec2& p2, const vec2& p3
    ) {
        add_point(p1);
        add_point(p2);
        add_point(p3);
        out_ << "f " << nv_ - 2 << " " << nv_ - 1 << " " << nv_ << std::endl;
    }
    
}

