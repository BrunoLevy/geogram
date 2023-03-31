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

#include <geogram/numerics/interval_nt.h>
#include <limits>

#ifdef __AVX2__

namespace {
    using namespace GEO;

    inline __m128d TwoProd_impl(__m128d a, __m128d b, __m128d& err) {
        __m128d result = _mm_mul_pd(a, b);
        err = _mm_fmsub_pd(a, b, result);
        return result;
    }
    
    inline __m128d abs_pd(__m128d x) {
        return _mm_andnot_pd(_mm_set1_pd(-0.0), x);
    }

    inline __m128d nextafter_pd(__m128d x) {
        __m128d phi_pd = _mm_set1_pd(
            0.5*std::numeric_limits<double>::epsilon()*
            (1.0 + std::numeric_limits<double>::epsilon())
        );
        __m128d eta_pd = _mm_set1_pd(
            std::numeric_limits<double>::denorm_min()
        );
        __m128d e = _mm_add_pd(_mm_mul_pd(phi_pd, abs_pd(x)), eta_pd);
        return _mm_add_pd(x, e);
    }

    // Performs an if/then/else statement as follows:
    // For each component:
    // if the mask is FFFFFFFFFFFFFFFF, set the value to trueval
    // if the mask is 0000000000000000, set the value to falseval
    inline __m128d if_pd(__m128d mask, __m128d trueval, __m128d falseval) {
        __m128d true_ = _mm_and_pd(mask, trueval);
        __m128d false_ = _mm_andnot_pd(mask, falseval);
        return _mm_or_pd(true_, false_);
    }
}

namespace GEO {

    void interval_nt::adjust(__m128d err) {
        __m128d pn_one = _mm_set_pd(1.0, -1.0);
        __m128d pn_x = _mm_mul_pd(pn_one, value_);
        __m128d mask = _mm_cmpgt_pd(
            _mm_mul_pd(pn_one, err),
            _mm_setzero_pd()
        );
        __m128d next = nextafter_pd(pn_x);
        pn_x = if_pd(mask, next, pn_x);
        value_ = _mm_mul_pd(pn_one, pn_x);
    }
    
    interval_nt::Sign2 interval_nt::sign(__m128d value) {
        static const Sign2 results[] = { //    sup inf  inf sup
            SIGN2_PP,			 //	00  00	[P, P]
            SIGN2_ZP,			 //	00  01	[Z, P]
            SIGN2_NP,			 //	00  10	[N, P]
            SIGN2_ZP,			 //	00  11	[Z, P]
            
            SIGN2_ERROR,		 //	01  00	[P, Z]	error
            SIGN2_ZERO,			 //	01  01	[Z, Z]
            SIGN2_NZ,			 //	01  10	[N, Z]
            SIGN2_ZERO,			 //	01  11	[Z, Z]
            
            SIGN2_ERROR,		 //	10  00	[P, N]	error
            SIGN2_ERROR,		 //	10  01	[Z, N]	error
            SIGN2_NN,			 //	10  10	[N, N]
            SIGN2_ERROR,		 //	10  11	[Z, N]	error
            
            SIGN2_ERROR,		 //	11  00	[P, Z]	error
            SIGN2_ZERO,			 //	11  01	[Z, Z]
            SIGN2_NZ,			 //	11  10	[N, Z]
            SIGN2_ZERO,			 //	11  11	[Z, Z]
        };

        Numeric::uint32 mask1 = Numeric::uint32(_mm_movemask_pd(value));
        mask1 += 0x02u;			// move b1 to b2, leaving b0 unchanged
        mask1 &= ~0x02u;
        __m128d z = _mm_cmpeq_pd(value, _mm_setzero_pd());
        Numeric::uint32 mask2 = Numeric::uint32(_mm_movemask_pd(z));
        mask2 += 0x02u;			// move b1 to b2, leaving b0 unchanged
        mask2 &= ~0x02u;
        mask1 <<= 1;			// move b0, b2 to b3, b1
        mask1 |= mask2;			// b1..b0 == 00 - positive nonzero
        // 01 - positive zero
        // 10 - negative nonzero
        // 11 - negative zero
        Sign2 retval = results[mask1];
        geo_debug_assert(retval != SIGN2_ERROR);
        return retval;
    }

    __m128d interval_nt::TwoProd(__m128d a, __m128d b, __m128d& err) {
        Sign2 a_sign = sign(a);
        Sign2 b_sign = sign(b);

        switch (SIGN2_COUNT * a_sign + b_sign) {
        case SIGN2_COUNT * SIGN2_ZERO + SIGN2_ZERO:
        case SIGN2_COUNT * SIGN2_ZERO + SIGN2_PP:
        case SIGN2_COUNT * SIGN2_ZERO + SIGN2_ZP:
        case SIGN2_COUNT * SIGN2_ZERO + SIGN2_NP:
        case SIGN2_COUNT * SIGN2_ZERO + SIGN2_NZ:
        case SIGN2_COUNT * SIGN2_ZERO + SIGN2_NN:
        case SIGN2_COUNT * SIGN2_PP + SIGN2_ZERO:
        case SIGN2_COUNT * SIGN2_ZP + SIGN2_ZERO:
        case SIGN2_COUNT * SIGN2_NP + SIGN2_ZERO:
        case SIGN2_COUNT * SIGN2_NZ + SIGN2_ZERO:
        case SIGN2_COUNT * SIGN2_NN + SIGN2_ZERO:
            err = zero();
            return zero();

        case SIGN2_COUNT * SIGN2_PP + SIGN2_PP:
        case SIGN2_COUNT * SIGN2_PP + SIGN2_ZP:
        case SIGN2_COUNT * SIGN2_ZP + SIGN2_PP:
        case SIGN2_COUNT * SIGN2_ZP + SIGN2_ZP:
            return TwoProd_impl(a, b, err);

        case SIGN2_COUNT * SIGN2_NP + SIGN2_PP:
        case SIGN2_COUNT * SIGN2_NP + SIGN2_ZP:
            return TwoProd_impl(a, _mm_set1_pd(sup(b)), err);
            
        case SIGN2_COUNT * SIGN2_NN + SIGN2_PP:
        case SIGN2_COUNT * SIGN2_NN + SIGN2_ZP:
        case SIGN2_COUNT * SIGN2_NZ + SIGN2_PP:
        case SIGN2_COUNT * SIGN2_NZ + SIGN2_ZP:
            return TwoProd_impl(a, _mm_shuffle_pd(b, b, 1), err);

        case SIGN2_COUNT * SIGN2_PP + SIGN2_NP:
        case SIGN2_COUNT * SIGN2_ZP + SIGN2_NP:
            return TwoProd_impl(_mm_set1_pd(sup(a)), b, err);
            
        case SIGN2_COUNT * SIGN2_NP + SIGN2_NP: {
            geo_decl_aligned(double r_[2]);
            geo_decl_aligned(double e_[2]);
            geo_decl_aligned(double p_[2]);
            geo_decl_aligned(double q_[2]);
            __m128d e;
            __m128d p = TwoProd_impl(a, _mm_shuffle_pd(b, b, 1), e);
            _mm_store_pd(p_, p);
            _mm_store_pd(q_, e);
            if ((p_[0] < p_[1]) || ((p_[0] == p_[1]) && (q_[0] < q_[1]))) {
                r_[0] = p_[0];
                e_[0] = q_[0];
            } else {
                r_[0] = p_[1];
                e_[0] = q_[1];
            }
            p = TwoProd_impl(a, b, e);
            _mm_store_pd(p_, p);
            _mm_store_pd(q_, e);
            if ((p_[0] > p_[1]) || ((p_[0] == p_[1]) && (q_[0] > q_[1]))) {
                r_[1] = p_[0];
                e_[1] = q_[0];
            } else {
                r_[1] = p_[1];
                e_[1] = q_[1];
            }
            err = _mm_load_pd(e_);
            return _mm_load_pd(r_);
        } 
            
        case SIGN2_COUNT * SIGN2_NN + SIGN2_NP:
        case SIGN2_COUNT * SIGN2_NZ + SIGN2_NP:
            return TwoProd_impl(
                _mm_set1_pd(inf(a)), _mm_shuffle_pd(b, b, 1), err
            );

        case SIGN2_COUNT * SIGN2_PP + SIGN2_NN:
        case SIGN2_COUNT * SIGN2_PP + SIGN2_NZ:
        case SIGN2_COUNT * SIGN2_ZP + SIGN2_NN:
        case SIGN2_COUNT * SIGN2_ZP + SIGN2_NZ:
            return TwoProd_impl(_mm_shuffle_pd(a, a, 1), b, err);

        case SIGN2_COUNT * SIGN2_NP + SIGN2_NN:
        case SIGN2_COUNT * SIGN2_NP + SIGN2_NZ:
            return TwoProd_impl(
                _mm_shuffle_pd(a, a, 1), _mm_set1_pd(inf(b)), err
            );
            
        case SIGN2_COUNT * SIGN2_NN + SIGN2_NN:
        case SIGN2_COUNT * SIGN2_NN + SIGN2_NZ:
        case SIGN2_COUNT * SIGN2_NZ + SIGN2_NN:
        case SIGN2_COUNT * SIGN2_NZ + SIGN2_NZ:
            return TwoProd_impl(
                _mm_shuffle_pd(a, a, 1), _mm_shuffle_pd(b, b, 1), err
            );
        }
        geo_assert_not_reached;
    }
    
}

#endif
