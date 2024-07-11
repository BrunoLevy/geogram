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

#include <geogram/numerics/expansion_nt.h>

namespace GEO {

    expansion_nt& expansion_nt::operator+= (const expansion_nt& rhs) {
        index_t e_capa = expansion::sum_capacity(rep(), rhs.rep());
        expansion* e = expansion::new_expansion_on_heap(e_capa);
        e->assign_sum(rep(), rhs.rep());
        cleanup();
        rep_ = e;
        return *this;
    }

    expansion_nt& expansion_nt::operator+= (double rhs) {
        index_t e_capa = expansion::sum_capacity(rep(), rhs);

        // TODO: optimized in-place version to be used
        //   if(!shared() && e_capa < rep().capacity())

        expansion* e = expansion::new_expansion_on_heap(e_capa);
        e->assign_sum(rep(), rhs);
        cleanup();
        rep_ = e;
        return *this;
    }

    expansion_nt& expansion_nt::operator-= (const expansion_nt& rhs) {
        index_t e_capa = expansion::diff_capacity(rep(), rhs.rep());
        expansion* e = expansion::new_expansion_on_heap(e_capa);
        e->assign_diff(rep(), rhs.rep());
        cleanup();
        rep_ = e;
        return *this;
    }

    expansion_nt& expansion_nt::operator-= (double rhs) {
        index_t e_capa = expansion::diff_capacity(rep(), rhs);

        // TODO: optimized in-place version to be used
        //   if(!shared() && e_capa < rep().capacity())

        expansion* e = expansion::new_expansion_on_heap(e_capa);
        e->assign_diff(rep(), rhs);
        cleanup();
        rep_ = e;
        return *this;
    }

    expansion_nt& expansion_nt::operator*= (const expansion_nt& rhs) {
        index_t e_capa = expansion::product_capacity(rep(), rhs.rep());
        expansion* e = expansion::new_expansion_on_heap(e_capa);
        e->assign_product(rep(), rhs.rep());
        cleanup();
        rep_ = e;
        return *this;
    }

    expansion_nt& expansion_nt::operator*= (double rhs) {
        index_t e_capa = expansion::product_capacity(rep(), rhs);

        // TODO: optimized in-place version to be used
        //   if(!shared() && e_capa < rep().capacity())

        expansion* e = expansion::new_expansion_on_heap(e_capa);
        e->assign_product(rep(), rhs);
        cleanup();
        rep_ = e;
        return *this;
    }

    /************************************************************************/

    expansion_nt expansion_nt::operator+ (const expansion_nt& rhs) const {
        expansion* e = expansion::new_expansion_on_heap(
            expansion::sum_capacity(rep(), rhs.rep())
        );
        e->assign_sum(rep(), rhs.rep());
        return expansion_nt(e);
    }

    expansion_nt expansion_nt::operator- (const expansion_nt& rhs) const {
        expansion* e = expansion::new_expansion_on_heap(
            expansion::diff_capacity(rep(), rhs.rep())
        );
        e->assign_diff(rep(), rhs.rep());
        return expansion_nt(e);
    }

    expansion_nt expansion_nt::operator* (const expansion_nt& rhs) const {
        expansion* e = expansion::new_expansion_on_heap(
            expansion::product_capacity(rep(), rhs.rep())
        );
        e->assign_product(rep(), rhs.rep());
        return expansion_nt(e);
    }

    expansion_nt expansion_nt::operator+ (double rhs) const {
        expansion* e = expansion::new_expansion_on_heap(
            expansion::sum_capacity(rep(), rhs)
        );
        e->assign_sum(rep(), rhs);
        return expansion_nt(e);
    }

    expansion_nt expansion_nt::operator- (double rhs) const {
        expansion* e = expansion::new_expansion_on_heap(
            expansion::diff_capacity(rep(), rhs)
        );
        e->assign_diff(rep(), rhs);
        return expansion_nt(e);
    }

    expansion_nt expansion_nt::operator* (double rhs) const {
        expansion* e = expansion::new_expansion_on_heap(
            expansion::product_capacity(rep(), rhs)
        );
        e->assign_product(rep(), rhs);
        return expansion_nt(e);
    }

    /************************************************************************/

    expansion_nt expansion_nt::operator- () const {
        expansion_nt result(*this);
        result.rep().negate();
        return result;
    }

    /************************************************************************/

    expansion_nt expansion_nt_determinant(
        const expansion_nt& a00,const expansion_nt& a01,
        const expansion_nt& a10,const expansion_nt& a11
    ) {
        expansion* result = expansion::new_expansion_on_heap(
            expansion::det2x2_capacity(a00.rep(),a01.rep(),a10.rep(),a11.rep())
        );
        result->assign_det2x2(a00.rep(),a01.rep(),a10.rep(),a11.rep());
        return expansion_nt(result);
    }


    expansion_nt expansion_nt_determinant(
        const expansion_nt& a00,const expansion_nt& a01,const expansion_nt& a02,
        const expansion_nt& a10,const expansion_nt& a11,const expansion_nt& a12,
        const expansion_nt& a20,const expansion_nt& a21,const expansion_nt& a22
    ) {
        // First compute the det2x2
        const expansion& m01 =
            expansion_det2x2(a00.rep(), a10.rep(), a01.rep(), a11.rep());
        const expansion& m02 =
            expansion_det2x2(a00.rep(), a20.rep(), a01.rep(), a21.rep());
        const expansion& m12 =
            expansion_det2x2(a10.rep(), a20.rep(), a11.rep(), a21.rep());

        // Now compute the minors of rank 3
        const expansion& z1 = expansion_product(m01,a22.rep());
        const expansion& z2 = expansion_product(m02,a12.rep()).negate();
        const expansion& z3 = expansion_product(m12,a02.rep());

        return expansion_nt(expansion_nt::SUM, z1, z2, z3);
    }

    expansion_nt expansion_nt_determinant(
        const expansion_nt& a00,const expansion_nt& a01,
        const expansion_nt& a02,const expansion_nt& a03,
        const expansion_nt& a10,const expansion_nt& a11,
        const expansion_nt& a12,const expansion_nt& a13,
        const expansion_nt& a20,const expansion_nt& a21,
        const expansion_nt& a22,const expansion_nt& a23,
        const expansion_nt& a30,const expansion_nt& a31,
        const expansion_nt& a32,const expansion_nt& a33
    ) {

        // First compute the det2x2
        const expansion& m01 =
            expansion_det2x2(a10.rep(),a00.rep(),a11.rep(),a01.rep());
        const expansion& m02 =
            expansion_det2x2(a20.rep(),a00.rep(),a21.rep(),a01.rep());
        const expansion& m03 =
            expansion_det2x2(a30.rep(),a00.rep(),a31.rep(),a01.rep());
        const expansion& m12 =
            expansion_det2x2(a20.rep(),a10.rep(),a21.rep(),a11.rep());
        const expansion& m13 =
            expansion_det2x2(a30.rep(),a10.rep(),a31.rep(),a11.rep());
        const expansion& m23 =
            expansion_det2x2(a30.rep(),a20.rep(),a31.rep(),a21.rep());

        // Now compute the minors of rank 3
        const expansion& m012_1 = expansion_product(m12,a02.rep());
        expansion& m012_2 = expansion_product(m02,a12.rep()); m012_2.negate();
        const expansion& m012_3 = expansion_product(m01,a22.rep());
        const expansion& m012 = expansion_sum3(m012_1, m012_2, m012_3);

        const expansion& m013_1 = expansion_product(m13,a02.rep());
        expansion& m013_2 = expansion_product(m03,a12.rep()); m013_2.negate();

        const expansion& m013_3 = expansion_product(m01,a32.rep());
        const expansion& m013 = expansion_sum3(m013_1, m013_2, m013_3);

        const expansion& m023_1 = expansion_product(m23,a02.rep());
        expansion& m023_2 = expansion_product(m03,a22.rep()); m023_2.negate();
        const expansion& m023_3 = expansion_product(m02,a32.rep());
        const expansion& m023 = expansion_sum3(m023_1, m023_2, m023_3);

        const expansion& m123_1 = expansion_product(m23,a12.rep());
        expansion& m123_2 = expansion_product(m13,a22.rep()); m123_2.negate();
        const expansion& m123_3 = expansion_product(m12,a32.rep());
        const expansion& m123 = expansion_sum3(m123_1, m123_2, m123_3);

        // Now compute the minors of rank 4
        const expansion& m0123_1 = expansion_product(m123,a03.rep());
        const expansion& m0123_2 = expansion_product(m023,a13.rep());
        const expansion& m0123_3 = expansion_product(m013,a23.rep());
        const expansion& m0123_4 = expansion_product(m012,a33.rep());

        const expansion& z1 = expansion_sum(m0123_1, m0123_3);
        const expansion& z2 = expansion_sum(m0123_2, m0123_4);

        return expansion_nt(expansion_nt::DIFF,z1,z2);
    }

    /***********************************************************************/

    namespace Numeric {

        template<> Sign ratio_compare(
            const expansion_nt& a_num, const expansion_nt& a_denom,
            const expansion_nt& b_num, const expansion_nt& b_denom
        ) {
            // TODO HERE: CHECK THAT THIS FITS ON STACK

            Sign s1 = Sign(a_num.sign()*a_denom.sign());
            Sign s2 = Sign(b_num.sign()*b_denom.sign());
            if(s1 == ZERO && s2 == ZERO) {
                return ZERO;
            }
            if(s1 != s2) {
                return (int(s1) > int(s2) ? POSITIVE : NEGATIVE);
            }
            if(a_denom == b_denom) {
                const expansion& diff_num = expansion_diff(
                    a_num.rep(), b_num.rep()
                );
                return Sign(diff_num.sign() * a_denom.sign());
            }
            const expansion& num_a = expansion_product(
                a_num.rep(), b_denom.rep()
            );
            const expansion& num_b = expansion_product(
                b_num.rep(), a_denom.rep()
            );
            const expansion& diff_num = expansion_diff(num_a, num_b);
            return Sign(
                diff_num.sign() * a_denom.sign() * b_denom.sign()
            );
        }

    }

}
