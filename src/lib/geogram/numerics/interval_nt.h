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

#ifndef GEOGRAM_NUMERICS_INTERVAL_NT
#define GEOGRAM_NUMERICS_INTERVAL_NT

#include <geogram/basic/common.h>
#include <geogram/basic/memory.h>


/**
 * \file geogram/numerics/expansion_nt.h
 * \brief Interval number type
 * \details This file provides a "number-type" that implements interval 
 *   arithmetics.
 * Inspiration:
 *   - https://www.codeproject.com/Articles/1040839/ 
 *      Interval-arithmetic-using-round-to-nearest-mode-pa
 * References (thank you Guillaume Moroz):
 *   - https://github.com/goualard-f/GAOL
 *   - Interval Arithmetic: from Principles to Implementation
 *     Timothy J. Hickey, Qun Ju, and Maarten H. van Emden
 *      https://doi.org/10.1145/502102.502106
 *   - Ball arithmetics: https://arblib.org/
 */

// For now, only supported with AVX2 (needs FMA)
#ifdef __AVX2__
#include <immintrin.h> 

namespace GEO {

    class GEOGRAM_API interval_nt {
    public:
        interval_nt() : value_(zero()) {
        }
        
        interval_nt(double rhs) : value_(
            (rhs == 0.0) ? zero() : _mm_set_pd(rhs,rhs)
        ) {
        }

        interval_nt(const interval_nt& rhs) = default;

        interval_nt& operator=(const interval_nt& rhs) = default;
        
        interval_nt& operator=(double rhs) {
            value_=((rhs==0.0) ? zero() : _mm_set_pd(rhs,rhs));
            return *this;
        }

	enum Sign2 {
            SIGN2_ERROR = -1,
            SIGN2_ZERO,
            SIGN2_NP,
            SIGN2_PP,
            SIGN2_ZP,
            SIGN2_NN,
            SIGN2_NZ,
            SIGN2_COUNT
	};

        Sign2 sign() const {
            return sign(value_);
        }
        
        interval_nt operator-() const {
            // Swap both components
            __m128d swapped = _mm_shuffle_pd(value_, value_, 1);
            // Flip signs
            __m128d sign_mask = _mm_set_pd(-0.0, -0.0);
            return interval_nt(_mm_xor_pd(swapped,sign_mask));
        }
        
        interval_nt& operator+=(const interval_nt& rhs) {
            __m128d err;
            value_ = TwoSum(value_, rhs.value_, err);
            adjust(err);
            return *this;
        }
        
        interval_nt& operator-=(const interval_nt& rhs) {
            __m128d err;
            value_ = TwoDiff(value_, rhs.value_, err);
            adjust(err);
            return *this;
        }
        
        interval_nt& operator*=(const interval_nt& rhs) {
            __m128d err;
            value_ = TwoProd(value_, rhs.value_, err);
            adjust(err);
            return *this;
        }

    protected:
        interval_nt(__m128d rhs) : value_(rhs) {
        }
        
        static Sign2 sign(__m128d value);
        
        static inline __m128d zero() {
            // Note: _mm_set_pd() arguments are inversed,
            // it is _mm_set_pd(e1,e0)
            // (why on earth did they pick this order ?)
            return _mm_set_pd(0.0,-0.0);
        }

        static __m128d TwoSum(__m128d a, __m128d b, __m128d& err) {
            __m128d result = _mm_add_pd(a, b);
            __m128d bb = _mm_sub_pd(result, a);
            err = _mm_add_pd(
                _mm_sub_pd(a, _mm_sub_pd(result, bb)), _mm_sub_pd(b, bb)
            );
            return adjust_zero(result);
        }

        static __m128d TwoDiff(__m128d a, __m128d b, __m128d& err) {
            __m128d result = _mm_sub_pd(a, b);
            __m128d bb = _mm_sub_pd(result, a);
            err = _mm_sub_pd(
                _mm_sub_pd(a, _mm_sub_pd(result, bb)), _mm_add_pd(b, bb)
            );
            return adjust_zero(result);
        }

        static __m128d TwoProd(__m128d a, __m128d b, __m128d& err);

        static __m128d adjust_zero(__m128d x) {
            // Can we do something smarter here ?
            //  (in SSE registers, branchless) ?
            geo_decl_aligned(double unpacked[2]);
            _mm_store_pd(unpacked,x);
            
            if(unpacked[0] == 0.0 && unpacked[1] == 0.0) {
                return zero();
            }
            
            //	[0, b]	- return [+0, b]                
            if(unpacked[0] == 0.0) {
                return _mm_andnot_pd(_mm_set1_pd(-0.0),x);
            }

            //	[a, 0]	- return [a, -0]
            if(unpacked[1] == 0.0) {
                return _mm_or_pd(_mm_set_pd(-0.0,-0.0),x);
            }
            
            return x;
        }

        static double inf(__m128d x) {
            geo_decl_aligned(double retval[2]);
            _mm_store_pd(retval, x);
            return retval[0];
        }

        static double sup(__m128d x) {
            geo_decl_aligned(double retval[2]);
            _mm_store_pd(retval, x);
            return retval[1];
        }
        
        void adjust(__m128d err);
        
    private:
        __m128d value_;
    };


    inline interval_nt operator+(const interval_nt& a, const interval_nt& b) {
        interval_nt result = a;
        return result += b;
    }

    inline interval_nt operator-(const interval_nt& a, const interval_nt& b) {
        interval_nt result = a;
        return result -= b;
    }

    inline interval_nt operator*(const interval_nt& a, const interval_nt& b) {
        interval_nt result = a;
        return result *= b;
    }
    
}

#endif
#endif
