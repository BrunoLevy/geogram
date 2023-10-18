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

#ifndef GEOGRAM_BASIC_DEBUG_STREAM
#define GEOGRAM_BASIC_DEBUG_STREAM

#include <geogram/basic/common.h>
#include <geogram/basic/geometry.h>

#include <fstream>

namespace GEO {

    /**
     * \brief Easy to use functions to save geometry to a Alias Wavefront
     *  .obj file for debugging.
     */
    class GEOGRAM_API DebugStream {
    public:
        /**
         * \brief DebugStream constructor
         * \param[in] name base name of the file where geometry will be saved
         * \param[in] id optional index, if specified, appended to the name,
         *  padded with zeros (5 digits)
         */
        DebugStream(const std::string& name, index_t id = index_t(-1));
        /**
         * \brief DebugStream destructor
         */
        ~DebugStream();
        void add_point(const vec3& p);
        void add_point(const vec2& p);
        void add_segment(const vec3& p1, const vec3& p2);
        void add_segment(const vec2& p1, const vec2& p2);
        void add_triangle(
            const vec3& p1, const vec3& p2, const vec3& p3
        );
        void add_triangle(
            const vec2& p1, const vec2& p2, const vec2& p3
        );
    protected:
        static std::string filename(
            const std::string& name, index_t id = index_t(-1)
        );
    private:
        std::ofstream out_;
        std::string filename_;
        index_t nv_;
    };
}

#endif

