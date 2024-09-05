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

#include <geogram/basic/numeric.h>
#include <stdlib.h>

#include <random>

#ifdef GEO_COMPILER_EMSCRIPTEN
#pragma GCC diagnostic ignored "-Wc++11-long-long"
#endif

namespace GEO {

    namespace Numeric {

        static std::mt19937_64 random_engine;

        bool is_nan(float32 x) {
#ifdef GEO_COMPILER_MSVC
            return _isnan(x) || !_finite(x);
#else
            return std::isnan(x) || !std::isfinite(x);
#endif
        }

        bool is_nan(float64 x) {
#ifdef GEO_COMPILER_MSVC
            return _isnan(x) || !_finite(x);
#else
            return std::isnan(x) || !std::isfinite(x);
#endif
        }

        void random_reset() {
            random_engine = std::mt19937_64();
        }

        int32 random_int32() {
            return std::uniform_int_distribution<int32>(0, RAND_MAX)(random_engine);
        }

        float32 random_float32() {
            return std::uniform_real_distribution<float32>(0, 1)(random_engine);
        }

        float64 random_float64() {
            return std::uniform_real_distribution<float64>(0, 1)(random_engine);
        }
    }
}
