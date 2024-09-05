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

#include <geogram/basic/common.h>

// This makes sure the compiler will not optimize y = a*x+b
// with fused multiply-add, this would break the exact
// predicates.
#ifdef GEO_COMPILER_MSVC
#pragma fp_contract(off)
#endif

#include <geogram/numerics/predicates.h>
#include <geogram/numerics/multi_precision.h>
#include <geogram/basic/assert.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/matrix.h>
#include <algorithm>

#define FPG_UNCERTAIN_VALUE 0

#include <geogram/numerics/predicates/side1.h>
#include <geogram/numerics/predicates/side2.h>
#include <geogram/numerics/predicates/side3.h>
#include <geogram/numerics/predicates/side3h.h>
#include <geogram/numerics/predicates/side3_2dlifted.h>
#include <geogram/numerics/predicates/side4.h>
#include <geogram/numerics/predicates/side4h.h>
#include <geogram/numerics/predicates/orient2d.h>
#include <geogram/numerics/predicates/orient3d.h>
#include <geogram/numerics/predicates/det3d.h>
#include <geogram/numerics/predicates/det4d.h>
#include <geogram/numerics/predicates/dot3d.h>
#include <geogram/numerics/predicates/dot_compare_3d.h>
#include <geogram/numerics/predicates/det_compare_4d.h>
#include <geogram/numerics/predicates/aligned3d.h>

#ifdef __SSE2__
#include <emmintrin.h>
#endif


namespace {

    using namespace GEO;

    GEO::PCK::SOSMode SOS_mode_ = GEO::PCK::SOS_ADDRESS;

    /**
     * \brief Comparator class for nD points using lexicographic order.
     * \details Used by symbolic perturbations.
     */
    class LexicoCompare {
    public:

        /**
         * \brief LexicoCompare constructor.
         * \param[in] dim dimension of the points to compare.
         */
        LexicoCompare(index_t dim) : dim_(dim) {
        }

        /**
         * \brief Compares two points with respect to the lexicographic
         *  order.
         * \param[in] x , y pointers to the coordinates of the two points.
         * \retval true if x is strictly before y in the lexicographic order.
         * \retval false otherwise.
         */
        bool operator()(const double* x, const double* y) const {
            for(index_t i=0; i<dim_-1; ++i) {
                if(x[i] < y[i]) {
                    return true;
                }
                if(x[i] > y[i]) {
                    return false;
                }
            }
            return (x[dim_-1] < y[dim_-1]);
        }
    private:
        index_t dim_;
    };

    /**
     * \brief Compares two 3D points with respect to the lexicographic
     *  order.
     * \param[in] x , y pointers to the coordinates of the two 3D points.
     * \retval true if x is strictly before y in the lexicographic order.
     * \retval false otherwise.
     */
    bool lexico_compare_3d(const double* x, const double* y) {
        if(x[0] < y[0]) {
            return true;
        }
        if(x[0] > y[0]) {
            return false;
        }
        if(x[1] < y[1]) {
            return true;
        }
        if(x[1] > y[1]) {
            return false;
        }
        return x[2] < y[2];
    }

    /**
     * \brief Sorts an array of pointers to points.
     * \details set_SOS_mode() alters the behavior of this function.
     *  If set to PCK::SOS_ADDRESS, then just the addresses of the points
     *  are sorted. If set to PCK::SOS_LEXICO, then the points are sorted
     *  in function of the lexicographic order of their coordinates.
     * \param[in] begin a pointer to the first point.
     * \param[in] end one position past the pointer to the last point.
     * \param[in] dim the dimension of the points.
     */
    void GEOGRAM_API SOS_sort(
        const double** begin, const double** end, index_t dim
    ) {
        if(SOS_mode_ == PCK::SOS_ADDRESS) {
            std::sort(begin, end);
        } else {
            if(dim == 3) {
                std::sort(begin, end, lexico_compare_3d);
            } else {
                std::sort(begin, end, LexicoCompare(dim));
            }
        }
    }

    /**
     * \brief Gets the maximum of 4 double precision numbers.
     * \param[in] x1 , x2 , x3 , x4 the four numbers.
     * \return the maximum.
     */
    inline double max4(double x1, double x2, double x3, double x4) {
#ifdef __SSE2__
        double result;
        __m128d X1 =_mm_load_sd(&x1);
        __m128d X2 =_mm_load_sd(&x2);
        __m128d X3 =_mm_load_sd(&x3);
        __m128d X4 =_mm_load_sd(&x4);
        X1 = _mm_max_sd(X1,X2);
        X3 = _mm_max_sd(X3,X4);
        X1 = _mm_max_sd(X1,X3);
        _mm_store_sd(&result, X1);
        return result;
#else
        return std::max(std::max(x1,x2),std::max(x3,x4));
#endif
    }


    /**
     * \brief Gets the minimum and maximum of 3 double precision numbers.
     * \param[in] x1 , x2 , x3 the three numbers.
     * \param[out] m the minimum
     * \param[out] M the maximum
     */
    inline void get_minmax3(
        double& m, double& M, double x1, double x2, double x3
    ) {
#ifdef __SSE2__
        __m128d X1 =_mm_load_sd(&x1);
        __m128d X2 =_mm_load_sd(&x2);
        __m128d X3 =_mm_load_sd(&x3);
        __m128d MIN12 = _mm_min_sd(X1,X2);
        __m128d MAX12 = _mm_max_sd(X1,X2);
        X1 = _mm_min_sd(MIN12, X3);
        X3 = _mm_max_sd(MAX12, X3);
        _mm_store_sd(&m, X1);
        _mm_store_sd(&M, X3);
#else
        m = std::min(std::min(x1,x2), x3);
        M = std::max(std::max(x1,x2), x3);
#endif
    }

    /**
     * \brief Arithmetic filter for the in_sphere_3d_SOS() predicate.
     * \details This filter was optimized by hand by Sylvain Pion
     *  (may be faster than FPG/PCK-generated filter).
     * Since it is used massively by Delaunay_3d, using the
     *   optimized version may be worth it.
     * \param[in] p first vertex of the tetrahedron
     * \param[in] q second vertex of the tetrahedron
     * \param[in] r third vertex of the tetrahedron
     * \param[in] s fourth vertex of the tetrahedron
     * \param[in] t point to be tested
     * \retval +1 if \p t was determined to be outside
     *   the circumsphere of \p p,\p q,\p r,\p s
     * \retval -1 if \p t was determined to be inside
     *   the circumsphere of  \p p,\p q,\p r,\p s
     * \retval 0 if the position of \p t could be be determined
     */
    inline int in_sphere_3d_filter_optim(
        const double* p, const double* q,
        const double* r, const double* s, const double* t
    ) {
        double ptx = p[0] - t[0];
        double pty = p[1] - t[1];
        double ptz = p[2] - t[2];
        double pt2 = geo_sqr(ptx) + geo_sqr(pty) + geo_sqr(ptz);

        double qtx = q[0] - t[0];
        double qty = q[1] - t[1];
        double qtz = q[2] - t[2];
        double qt2 = geo_sqr(qtx) + geo_sqr(qty) + geo_sqr(qtz);

        double rtx = r[0] - t[0];
        double rty = r[1] - t[1];
        double rtz = r[2] - t[2];
        double rt2 = geo_sqr(rtx) + geo_sqr(rty) + geo_sqr(rtz);

        double stx = s[0] - t[0];
        double sty = s[1] - t[1];
        double stz = s[2] - t[2];
        double st2 = geo_sqr(stx) + geo_sqr(sty) + geo_sqr(stz);

        // Compute the semi-static bound.
        double maxx = ::fabs(ptx);
        double maxy = ::fabs(pty);
        double maxz = ::fabs(ptz);

        double aqtx = ::fabs(qtx);
        double artx = ::fabs(rtx);
        double astx = ::fabs(stx);

        double aqty = ::fabs(qty);
        double arty = ::fabs(rty);
        double asty = ::fabs(sty);

        double aqtz = ::fabs(qtz);
        double artz = ::fabs(rtz);
        double astz = ::fabs(stz);

        maxx = max4(maxx, aqtx, artx, astx);
        maxy = max4(maxy, aqty, arty, asty);
        maxz = max4(maxz, aqtz, artz, astz);

        double eps = 1.2466136531027298e-13 * maxx * maxy * maxz;

        double min_max;
        double max_max;
        get_minmax3(min_max, max_max, maxx, maxy, maxz);

        double det = det4x4(
            ptx,pty,ptz,pt2,
            rtx,rty,rtz,rt2,
            qtx,qty,qtz,qt2,
            stx,sty,stz,st2
        );

        if (min_max < 1e-58)  { /* sqrt^5(min_double/eps) */
            // Protect against underflow in the computation of eps.
            return FPG_UNCERTAIN_VALUE;
        } else if (max_max < 1e61)  { /* sqrt^5(max_double/4 [hadamard]) */
            // Protect against overflow in the computation of det.
            eps *= (max_max * max_max);
            // Note: inverted as compared to CGAL
            //   CGAL: in_sphere_3d (called side_of_oriented_sphere())
            //      positive side is outside the sphere.
            //   PCK: in_sphere_3d : positive side is inside the sphere
            if (det > eps)  return -1;
            if (det < -eps) return  1;
        }

        return FPG_UNCERTAIN_VALUE;
    }

    using namespace GEO;

    PCK::PredicateStats stats_side1("side1");
    PCK::PredicateStats stats_side2("side2");
    PCK::PredicateStats stats_side3("side3");
    PCK::PredicateStats stats_side3h("side3h");
    PCK::PredicateStats stats_side4("side4/insphere");
    PCK::PredicateStats stats_orient2d("orient2d");
    PCK::PredicateStats stats_orient3d("orient3d");
    PCK::PredicateStats stats_orient3dh("orient3dh");
    PCK::PredicateStats stats_det3d("det3d");
    PCK::PredicateStats stats_det4d("det4d");

    // ================= side1 =========================================

    /**
     * \brief Exact implementation of the side1() predicate using low-level
     *  exact arithmetics API (expansion class).
     */
    Sign side1_exact_SOS(
        const double* p0, const double* p1,
        const double* q0,
        coord_index_t dim
    ) {
        stats_side1.log_exact();
        expansion& l = expansion_sq_dist(p0, p1, dim);
        expansion& a = expansion_dot_at(p1, q0, p0, dim).scale_fast(2.0);
        expansion& r = expansion_diff(l, a);
        Sign r_sign = r.sign();
        // Symbolic perturbation, Simulation of Simplicity
        if(r_sign == ZERO) {
            stats_side1.log_SOS();
            return (p0 < p1) ? POSITIVE : NEGATIVE;
        }
        return r_sign;
    }

    /**
     * \brief Implements side1() in 3d.
     */
    Sign side1_3d_SOS(
        const double* p0, const double* p1, const double* q0
    ) {
        Sign result = Sign(side1_3d_filter(p0, p1, q0));
        if(result == ZERO) {
            result = side1_exact_SOS(p0, p1, q0, 3);
        }
        return result;
    }

    /**
     * \brief Implements side1() in 4d.
     */
    Sign side1_4d_SOS(
        const double* p0, const double* p1, const double* q0
    ) {
        Sign result = Sign(side1_4d_filter(p0, p1, q0));
        if(result == ZERO) {
            result = side1_exact_SOS(p0, p1, q0, 4);
        }
        return result;
    }

    /**
     * \brief Implements side1() in 6d.
     */
    Sign side1_6d_SOS(
        const double* p0, const double* p1, const double* q0
    ) {
        Sign result = Sign(side1_6d_filter(p0, p1, q0));
        if(result == ZERO) {
            result = side1_exact_SOS(p0, p1, q0, 6);
        }
        return result;
    }

    /**
     * \brief Implements side1() in 7d.
     */
    Sign side1_7d_SOS(
        const double* p0, const double* p1, const double* q0
    ) {
        Sign result = Sign(side1_7d_filter(p0, p1, q0));
        if(result == ZERO) {
            result = side1_exact_SOS(p0, p1, q0, 7);
        }
        return result;
    }

    /**
     * \brief Implements side1() in 8d.
     */
    Sign side1_8d_SOS(
        const double* p0, const double* p1, const double* q0
    ) {
        Sign result = Sign(side1_8d_filter(p0, p1, q0));
        if(result == ZERO) {
            result = side1_exact_SOS(p0, p1, q0, 8);
        }
        return result;
    }

    // ================= side2 =========================================

    /**
     * \brief Exact implementation of the side2() predicate using low-level
     *  exact arithmetics API (expansion class).
     */
    Sign side2_exact_SOS(
        const double* p0, const double* p1, const double* p2,
        const double* q0, const double* q1,
        coord_index_t dim
    ) {
        stats_side2.log_exact();

        const expansion& l1 = expansion_sq_dist(p1, p0, dim);
        const expansion& l2 = expansion_sq_dist(p2, p0, dim);

        const expansion& a10 = expansion_dot_at(p1,q0,p0, dim).scale_fast(2.0);
        const expansion& a11 = expansion_dot_at(p1,q1,p0, dim).scale_fast(2.0);
        const expansion& a20 = expansion_dot_at(p2,q0,p0, dim).scale_fast(2.0);
        const expansion& a21 = expansion_dot_at(p2,q1,p0, dim).scale_fast(2.0);

        const expansion& Delta = expansion_diff(a11, a10);

        Sign Delta_sign = Delta.sign();
        // Should not occur with symbolic
        // perturbation done at previous steps.
        geo_assert(Delta_sign != ZERO);

        //       [ Lambda0 ]   [ -1 ]        [  a11 ]
        // Delta [         ] = [    ] * l1 + [      ]
        //       [ Lambda1 ]   [  1 ]        [ -a10 ]

        const expansion& DeltaLambda0 = expansion_diff(a11, l1);
        const expansion& DeltaLambda1 = expansion_diff(l1, a10);

        // r = Delta*l2 - ( a20*DeltaLambda0 + a21*DeltaLambda1 )

        const expansion& r0 = expansion_product(Delta, l2);
        const expansion& r1 = expansion_product(a20, DeltaLambda0).negate();
        const expansion& r2 = expansion_product(a21, DeltaLambda1).negate();
        const expansion& r = expansion_sum3(r0, r1, r2);

        Sign r_sign = r.sign();

        // Simulation of Simplicity (symbolic perturbation)
        if(r_sign == ZERO) {
            stats_side2.log_SOS();

            const double* p_sort[3];
            p_sort[0] = p0;
            p_sort[1] = p1;
            p_sort[2] = p2;

            SOS_sort(p_sort, p_sort + 3, dim);

            for(index_t i = 0; i < 3; ++i) {
                if(p_sort[i] == p0) {
                    const expansion& z1 = expansion_diff(Delta, a21);
                    const expansion& z = expansion_sum(z1, a20);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta_sign * z_sign);
                    }
                }
                if(p_sort[i] == p1) {
                    const expansion& z = expansion_diff(a21, a20);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta_sign * z_sign);
                    }
                }
                if(p_sort[i] == p2) {
                    return NEGATIVE;
                }
            }
            geo_assert_not_reached;
        }

        return Sign(Delta_sign * r_sign);
    }

    /**
     * \brief Implements side2() in 3d.
     */
    Sign side2_3d_SOS(
        const double* p0, const double* p1, const double* p2,
        const double* q0, const double* q1
    ) {
        Sign result = Sign(side2_3d_filter(p0, p1, p2, q0, q1));
        if(result == ZERO) {
            result = side2_exact_SOS(p0, p1, p2, q0, q1, 3);
        }
        return result;
    }

    /**
     * \brief Implements side2() in 4d.
     */
    Sign side2_4d_SOS(
        const double* p0, const double* p1, const double* p2,
        const double* q0, const double* q1
    ) {
        Sign result = Sign(side2_4d_filter(p0, p1, p2, q0, q1));
        if(result == ZERO) {
            result = side2_exact_SOS(p0, p1, p2, q0, q1, 4);
        }
        return result;
    }

    /**
     * \brief Implements side2() in 6d.
     */
    Sign side2_6d_SOS(
        const double* p0, const double* p1, const double* p2,
        const double* q0, const double* q1
    ) {
        Sign result = Sign(side2_6d_filter(p0, p1, p2, q0, q1));
        if(result == ZERO) {
            result = side2_exact_SOS(p0, p1, p2, q0, q1, 6);
        }
        return result;
    }

    /**
     * \brief Implements side2() in 7d.
     */
    Sign side2_7d_SOS(
        const double* p0, const double* p1, const double* p2,
        const double* q0, const double* q1
    ) {
        Sign result = Sign(side2_7d_filter(p0, p1, p2, q0, q1));
        if(result == ZERO) {
            result = side2_exact_SOS(p0, p1, p2, q0, q1, 7);
        }
        return result;
    }

    /**
     * \brief Implements side2() in 8d.
     */
    Sign side2_8d_SOS(
        const double* p0, const double* p1, const double* p2,
        const double* q0, const double* q1
    ) {
        Sign result = Sign(side2_8d_filter(p0, p1, p2, q0, q1));
        if(result == ZERO) {
            result = side2_exact_SOS(p0, p1, p2, q0, q1, 8);
        }
        return result;
    }

    // ================= side3 =========================================

    /**
     * \brief Exact implementation of the side3() predicate using low-level
     *  exact arithmetics API (expansion class).
     */
    Sign side3_exact_SOS(
        const double* p0, const double* p1, const double* p2, const double* p3,
        const double* q0, const double* q1, const double* q2,
        coord_index_t dim
    ) {
        stats_side3.log_exact();

        const expansion& l1 = expansion_sq_dist(p1, p0, dim);
        const expansion& l2 = expansion_sq_dist(p2, p0, dim);
        const expansion& l3 = expansion_sq_dist(p3, p0, dim);

        const expansion& a10 = expansion_dot_at(p1,q0,p0, dim).scale_fast(2.0);
        const expansion& a11 = expansion_dot_at(p1,q1,p0, dim).scale_fast(2.0);
        const expansion& a12 = expansion_dot_at(p1,q2,p0, dim).scale_fast(2.0);
        const expansion& a20 = expansion_dot_at(p2,q0,p0, dim).scale_fast(2.0);
        const expansion& a21 = expansion_dot_at(p2,q1,p0, dim).scale_fast(2.0);
        const expansion& a22 = expansion_dot_at(p2,q2,p0, dim).scale_fast(2.0);

        const expansion& a30 = expansion_dot_at(p3,q0,p0, dim).scale_fast(2.0);
        const expansion& a31 = expansion_dot_at(p3,q1,p0, dim).scale_fast(2.0);
        const expansion& a32 = expansion_dot_at(p3,q2,p0, dim).scale_fast(2.0);

        // [ b00 b01 b02 ]           [  1   1   1  ]-1
        // [ b10 b11 b12 ] = Delta * [ a10 a11 a12 ]
        // [ b20 b21 b22 ]           [ a20 a21 a22 ]

        const expansion& b00 = expansion_det2x2(a11, a12, a21, a22);
        const expansion& b01 = expansion_diff(a21, a22);
        const expansion& b02 = expansion_diff(a12, a11);
        const expansion& b10 = expansion_det2x2(a12, a10, a22, a20);
        const expansion& b11 = expansion_diff(a22, a20);
        const expansion& b12 = expansion_diff(a10, a12);
        const expansion& b20 = expansion_det2x2(a10, a11, a20, a21);
        const expansion& b21 = expansion_diff(a20, a21);
        const expansion& b22 = expansion_diff(a11, a10);

        const expansion& Delta = expansion_sum3(b00, b10, b20);
        Sign Delta_sign = Delta.sign();
        // Should not occur with symbolic
        // perturbation done at previous steps.
        geo_assert(Delta_sign != ZERO);

        //       [ Lambda0 ]   [ b01 b02 ]   [ l1 ]   [ b00 ]
        // Delta [ Lambda1 ] = [ b11 b12 ] * [    ] + [ b10 ]
        //       [ Lambda2 ]   [ b21 b22 ]   [ l2 ]   [ b20 ]

        const expansion& b01_l1 = expansion_product(b01, l1);
        const expansion& b02_l2 = expansion_product(b02, l2);
        const expansion& DeltaLambda0 = expansion_sum3(b01_l1, b02_l2, b00);

        const expansion& b11_l1 = expansion_product(b11, l1);
        const expansion& b12_l2 = expansion_product(b12, l2);
        const expansion& DeltaLambda1 = expansion_sum3(b11_l1, b12_l2, b10);

        const expansion& b21_l1 = expansion_product(b21, l1);
        const expansion& b22_l2 = expansion_product(b22, l2);
        const expansion& DeltaLambda2 = expansion_sum3(b21_l1, b22_l2, b20);

        // r = Delta*l3-(a30*DeltaLambda0+a31*DeltaLambda1+a32*DeltaLambda2)

        const expansion& r0 = expansion_product(Delta, l3);
        const expansion& r1 = expansion_product(a30, DeltaLambda0).negate();
        const expansion& r2 = expansion_product(a31, DeltaLambda1).negate();
        const expansion& r3 = expansion_product(a32, DeltaLambda2).negate();
        const expansion& r = expansion_sum4(r0, r1, r2, r3);
        Sign r_sign = r.sign();

        // Simulation of Simplicity (symbolic perturbation)
        if(r_sign == ZERO) {
            stats_side3.log_SOS();

            const double* p_sort[4];
            p_sort[0] = p0;
            p_sort[1] = p1;
            p_sort[2] = p2;
            p_sort[3] = p3;
            SOS_sort(p_sort, p_sort + 4, dim);
            for(index_t i = 0; i < 4; ++i) {
                if(p_sort[i] == p0) {
                    const expansion& z1_0 = expansion_sum(b01, b02);
                    const expansion& z1 = expansion_product(a30, z1_0).negate();
                    const expansion& z2_0 = expansion_sum(b11, b12);
                    const expansion& z2 = expansion_product(a31, z2_0).negate();
                    const expansion& z3_0 = expansion_sum(b21, b22);
                    const expansion& z3 = expansion_product(a32, z3_0).negate();
                    const expansion& z = expansion_sum4(Delta, z1, z2, z3);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta_sign * z_sign);
                    }
                } else if(p_sort[i] == p1) {
                    const expansion& z1 = expansion_product(a30, b01);
                    const expansion& z2 = expansion_product(a31, b11);
                    const expansion& z3 = expansion_product(a32, b21);
                    const expansion& z = expansion_sum3(z1, z2, z3);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta_sign * z_sign);
                    }
                } else if(p_sort[i] == p2) {
                    const expansion& z1 = expansion_product(a30, b02);
                    const expansion& z2 = expansion_product(a31, b12);
                    const expansion& z3 = expansion_product(a32, b22);
                    const expansion& z = expansion_sum3(z1, z2, z3);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta_sign * z_sign);
                    }
                } else if(p_sort[i] == p3) {
                    return NEGATIVE;
                }
            }
            geo_assert_not_reached;
        }
        return Sign(Delta_sign * r_sign);
    }


    /**
     * \brief Exact implementation of the side3_3dlifted() predicate
     *  using low-level exact arithmetics API (expansion class).
     */
    Sign side3h_exact_SOS(
        const double* p0, const double* p1, const double* p2, const double* p3,
        double h0, double h1, double h2, double h3,
        const double* q0, const double* q1, const double* q2
    ) {
        stats_side3h.log_exact();

        const expansion& l1 = expansion_diff(h1,h0);
        const expansion& l2 = expansion_diff(h2,h0);
        const expansion& l3 = expansion_diff(h3,h0);

        const expansion& a10 = expansion_dot_at(p1, q0, p0, 3).scale_fast(2.0);
        const expansion& a11 = expansion_dot_at(p1, q1, p0, 3).scale_fast(2.0);
        const expansion& a12 = expansion_dot_at(p1, q2, p0, 3).scale_fast(2.0);
        const expansion& a20 = expansion_dot_at(p2, q0, p0, 3).scale_fast(2.0);
        const expansion& a21 = expansion_dot_at(p2, q1, p0, 3).scale_fast(2.0);
        const expansion& a22 = expansion_dot_at(p2, q2, p0, 3).scale_fast(2.0);

        const expansion& a30 = expansion_dot_at(p3, q0, p0, 3).scale_fast(2.0);
        const expansion& a31 = expansion_dot_at(p3, q1, p0, 3).scale_fast(2.0);
        const expansion& a32 = expansion_dot_at(p3, q2, p0, 3).scale_fast(2.0);

        // [ b00 b01 b02 ]           [  1   1   1  ]-1
        // [ b10 b11 b12 ] = Delta * [ a10 a11 a12 ]
        // [ b20 b21 b22 ]           [ a20 a21 a22 ]

        const expansion& b00 = expansion_det2x2(a11, a12, a21, a22);
        const expansion& b01 = expansion_diff(a21, a22);
        const expansion& b02 = expansion_diff(a12, a11);
        const expansion& b10 = expansion_det2x2(a12, a10, a22, a20);
        const expansion& b11 = expansion_diff(a22, a20);
        const expansion& b12 = expansion_diff(a10, a12);
        const expansion& b20 = expansion_det2x2(a10, a11, a20, a21);
        const expansion& b21 = expansion_diff(a20, a21);
        const expansion& b22 = expansion_diff(a11, a10);

        const expansion& Delta = expansion_sum3(b00, b10, b20);
        Sign Delta_sign = Delta.sign();
        // Should not occur with symbolic
        // perturbation done at previous steps.
        geo_assert(Delta_sign != ZERO);

        //       [ Lambda0 ]   [ b01 b02 ]   [ l1 ]   [ b00 ]
        // Delta [ Lambda1 ] = [ b11 b12 ] * [    ] + [ b10 ]
        //       [ Lambda2 ]   [ b21 b22 ]   [ l2 ]   [ b20 ]

        const expansion& b01_l1 = expansion_product(b01, l1);
        const expansion& b02_l2 = expansion_product(b02, l2);
        const expansion& DeltaLambda0 = expansion_sum3(b01_l1, b02_l2, b00);

        const expansion& b11_l1 = expansion_product(b11, l1);
        const expansion& b12_l2 = expansion_product(b12, l2);
        const expansion& DeltaLambda1 = expansion_sum3(b11_l1, b12_l2, b10);

        const expansion& b21_l1 = expansion_product(b21, l1);
        const expansion& b22_l2 = expansion_product(b22, l2);
        const expansion& DeltaLambda2 = expansion_sum3(b21_l1, b22_l2, b20);

        // r = Delta*l3-(a30*DeltaLambda0+a31*DeltaLambda1+a32*DeltaLambda2)

        const expansion& r0 = expansion_product(Delta, l3);
        const expansion& r1 = expansion_product(a30, DeltaLambda0).negate();
        const expansion& r2 = expansion_product(a31, DeltaLambda1).negate();
        const expansion& r3 = expansion_product(a32, DeltaLambda2).negate();
        const expansion& r = expansion_sum4(r0, r1, r2, r3);
        Sign r_sign = r.sign();

        // Simulation of Simplicity (symbolic perturbation)
        if(r_sign == ZERO) {
            stats_side3h.log_SOS();

            const double* p_sort[4];
            p_sort[0] = p0;
            p_sort[1] = p1;
            p_sort[2] = p2;
            p_sort[3] = p3;

            SOS_sort(p_sort, p_sort + 4, 3);
            for(index_t i = 0; i < 4; ++i) {
                if(p_sort[i] == p0) {
                    const expansion& z1_0 = expansion_sum(b01, b02);
                    const expansion& z1 = expansion_product(a30, z1_0).negate();
                    const expansion& z2_0 = expansion_sum(b11, b12);
                    const expansion& z2 = expansion_product(a31, z2_0).negate();
                    const expansion& z3_0 = expansion_sum(b21, b22);
                    const expansion& z3 = expansion_product(a32, z3_0).negate();
                    const expansion& z = expansion_sum4(Delta, z1, z2, z3);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta_sign * z_sign);
                    }
                } else if(p_sort[i] == p1) {
                    const expansion& z1 = expansion_product(a30, b01);
                    const expansion& z2 = expansion_product(a31, b11);
                    const expansion& z3 = expansion_product(a32, b21);
                    const expansion& z = expansion_sum3(z1, z2, z3);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta_sign * z_sign);
                    }
                } else if(p_sort[i] == p2) {
                    const expansion& z1 = expansion_product(a30, b02);
                    const expansion& z2 = expansion_product(a31, b12);
                    const expansion& z3 = expansion_product(a32, b22);
                    const expansion& z = expansion_sum3(z1, z2, z3);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta_sign * z_sign);
                    }
                } else if(p_sort[i] == p3) {
                    return NEGATIVE;
                }
            }
            geo_assert_not_reached;
        }
        return Sign(Delta_sign * r_sign);
    }


    /**
     * \brief Implements side3() in 3d.
     */
    Sign side3_3d_SOS(
        const double* p0, const double* p1, const double* p2, const double* p3,
        const double* q0, const double* q1, const double* q2
    ) {
        Sign result = Sign(side3_3d_filter(p0, p1, p2, p3, q0, q1, q2));
        if(result == ZERO) {
            result = side3_exact_SOS(p0, p1, p2, p3, q0, q1, q2, 3);
        }
        return result;
    }


    /**
     * \brief Implements side3() in 4d.
     */
    Sign side3_4d_SOS(
        const double* p0, const double* p1, const double* p2, const double* p3,
        const double* q0, const double* q1, const double* q2
    ) {
        Sign result = Sign(side3_4d_filter(p0, p1, p2, p3, q0, q1, q2));
        if(result == ZERO) {
            result = side3_exact_SOS(p0, p1, p2, p3, q0, q1, q2, 4);
        }
        return result;
    }

    /**
     * \brief Implements side3() in 6d.
     */
    Sign side3_6d_SOS(
        const double* p0, const double* p1, const double* p2, const double* p3,
        const double* q0, const double* q1, const double* q2
    ) {
        Sign result = Sign(side3_6d_filter(p0, p1, p2, p3, q0, q1, q2));
        if(result == ZERO) {
            result = side3_exact_SOS(p0, p1, p2, p3, q0, q1, q2, 6);
        }
        return result;
    }

    /**
     * \brief Implements side3() in 7d.
     */
    Sign side3_7d_SOS(
        const double* p0, const double* p1, const double* p2, const double* p3,
        const double* q0, const double* q1, const double* q2
    ) {
        Sign result = Sign(side3_7d_filter(p0, p1, p2, p3, q0, q1, q2));
        if(result == ZERO) {
            result = side3_exact_SOS(p0, p1, p2, p3, q0, q1, q2, 7);
        }
        return result;
    }

    /**
     * \brief Implements side3() in 7d.
     */
    Sign side3_8d_SOS(
        const double* p0, const double* p1, const double* p2, const double* p3,
        const double* q0, const double* q1, const double* q2
    ) {
        Sign result = Sign(side3_8d_filter(p0, p1, p2, p3, q0, q1, q2));
        if(result == ZERO) {
            result = side3_exact_SOS(p0, p1, p2, p3, q0, q1, q2, 8);
        }
        return result;
    }

    // ================= side4 =========================================

    /**
     * \brief Exact implementation of the side4_3d_SOS() predicate
     *  using low-level exact arithmetics API (expansion class).
     * \param[in] sos if true, applies symbolic perturbation when
     *  result is zero, else returns zero
     */
    Sign side4_3d_exact_SOS(
        const double* p0, const double* p1, const double* p2, const double* p3,
        const double* p4, bool sos = true
    ) {
        stats_side4.log_exact();

        const expansion& a11 = expansion_diff(p1[0], p0[0]);
        const expansion& a12 = expansion_diff(p1[1], p0[1]);
        const expansion& a13 = expansion_diff(p1[2], p0[2]);
        const expansion& a14 = expansion_sq_dist(p1, p0, 3).negate();

        const expansion& a21 = expansion_diff(p2[0], p0[0]);
        const expansion& a22 = expansion_diff(p2[1], p0[1]);
        const expansion& a23 = expansion_diff(p2[2], p0[2]);
        const expansion& a24 = expansion_sq_dist(p2, p0, 3).negate();

        const expansion& a31 = expansion_diff(p3[0], p0[0]);
        const expansion& a32 = expansion_diff(p3[1], p0[1]);
        const expansion& a33 = expansion_diff(p3[2], p0[2]);
        const expansion& a34 = expansion_sq_dist(p3, p0, 3).negate();

        const expansion& a41 = expansion_diff(p4[0], p0[0]);
        const expansion& a42 = expansion_diff(p4[1], p0[1]);
        const expansion& a43 = expansion_diff(p4[2], p0[2]);
        const expansion& a44 = expansion_sq_dist(p4, p0, 3).negate();

        // This commented-out version does not reuse
        // the 2x2 minors.
/*
  const expansion& Delta1 = expansion_det3x3(
  a21, a22, a23,
  a31, a32, a33,
  a41, a42, a43
  );
  const expansion& Delta2 = expansion_det3x3(
  a11, a12, a13,
  a31, a32, a33,
  a41, a42, a43
  );
  const expansion& Delta3 = expansion_det3x3(
  a11, a12, a13,
  a21, a22, a23,
  a41, a42, a43
  );
  const expansion& Delta4 = expansion_det3x3(
  a11, a12, a13,
  a21, a22, a23,
  a31, a32, a33
  );
*/

        // Optimized version that reuses the 2x2 minors

        const expansion& m12 = expansion_det2x2(a12,a13,a22,a23);
        const expansion& m13 = expansion_det2x2(a12,a13,a32,a33);
        const expansion& m14 = expansion_det2x2(a12,a13,a42,a43);
        const expansion& m23 = expansion_det2x2(a22,a23,a32,a33);
        const expansion& m24 = expansion_det2x2(a22,a23,a42,a43);
        const expansion& m34 = expansion_det2x2(a32,a33,a42,a43);


        const expansion& z11 = expansion_product(a21,m34);
        const expansion& z12 = expansion_product(a31,m24).negate();
        const expansion& z13 = expansion_product(a41,m23);
        const expansion& Delta1 = expansion_sum3(z11,z12,z13);

        const expansion& z21 = expansion_product(a11,m34);
        const expansion& z22 = expansion_product(a31,m14).negate();
        const expansion& z23 = expansion_product(a41,m13);
        const expansion& Delta2 = expansion_sum3(z21,z22,z23);

        const expansion& z31 = expansion_product(a11,m24);
        const expansion& z32 = expansion_product(a21,m14).negate();
        const expansion& z33 = expansion_product(a41,m12);
        const expansion& Delta3 = expansion_sum3(z31,z32,z33);

        const expansion& z41 = expansion_product(a11,m23);
        const expansion& z42 = expansion_product(a21,m13).negate();
        const expansion& z43 = expansion_product(a31,m12);
        const expansion& Delta4 = expansion_sum3(z41,z42,z43);


        Sign Delta4_sign = Delta4.sign();
        geo_assert(Delta4_sign != ZERO);

        const expansion& r_1 = expansion_product(Delta1, a14);
        const expansion& r_2 = expansion_product(Delta2, a24).negate();
        const expansion& r_3 = expansion_product(Delta3, a34);
        const expansion& r_4 = expansion_product(Delta4, a44).negate();
        const expansion& r = expansion_sum4(r_1, r_2, r_3, r_4);
        Sign r_sign = r.sign();

        // Simulation of Simplicity (symbolic perturbation)
        if(sos && r_sign == ZERO) {
            stats_side4.log_SOS();

            const double* p_sort[5];
            p_sort[0] = p0;
            p_sort[1] = p1;
            p_sort[2] = p2;
            p_sort[3] = p3;
            p_sort[4] = p4;
            SOS_sort(p_sort, p_sort + 5, 3);
            for(index_t i = 0; i < 5; ++i) {
                if(p_sort[i] == p0) {
                    const expansion& z1 = expansion_diff(Delta2, Delta1);
                    const expansion& z2 = expansion_diff(Delta4, Delta3);
                    const expansion& z = expansion_sum(z1, z2);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta4_sign * z_sign);
                    }
                } else if(p_sort[i] == p1) {
                    Sign Delta1_sign = Delta1.sign();
                    if(Delta1_sign != ZERO) {
                        return Sign(Delta4_sign * Delta1_sign);
                    }
                } else if(p_sort[i] == p2) {
                    Sign Delta2_sign = Delta2.sign();
                    if(Delta2_sign != ZERO) {
                        return Sign(-Delta4_sign * Delta2_sign);
                    }
                } else if(p_sort[i] == p3) {
                    Sign Delta3_sign = Delta3.sign();
                    if(Delta3_sign != ZERO) {
                        return Sign(Delta4_sign * Delta3_sign);
                    }
                } else if(p_sort[i] == p4) {
                    return NEGATIVE;
                }
            }
        }
        return Sign(Delta4_sign * r_sign);
    }

    /**
     * \brief Exact implementation of the side4() predicate using low-level
     *  exact arithmetics API (expansion class).
     */
    Sign side4_exact_SOS(
        const double* p0, const double* p1, const double* p2, const double* p3,
        const double* p4,
        const double* q0, const double* q1, const double* q2, const double* q3,
        coord_index_t dim
    ) {
        stats_side4.log_exact();

        const expansion& l1 = expansion_sq_dist(p1, p0, dim);
        const expansion& l2 = expansion_sq_dist(p2, p0, dim);
        const expansion& l3 = expansion_sq_dist(p3, p0, dim);
        const expansion& l4 = expansion_sq_dist(p4, p0, dim);

        const expansion& a10 = expansion_dot_at(p1,q0,p0, dim).scale_fast(2.0);
        const expansion& a11 = expansion_dot_at(p1,q1,p0, dim).scale_fast(2.0);
        const expansion& a12 = expansion_dot_at(p1,q2,p0, dim).scale_fast(2.0);
        const expansion& a13 = expansion_dot_at(p1,q3,p0, dim).scale_fast(2.0);

        const expansion& a20 = expansion_dot_at(p2,q0,p0, dim).scale_fast(2.0);
        const expansion& a21 = expansion_dot_at(p2,q1,p0, dim).scale_fast(2.0);
        const expansion& a22 = expansion_dot_at(p2,q2,p0, dim).scale_fast(2.0);
        const expansion& a23 = expansion_dot_at(p2,q3,p0, dim).scale_fast(2.0);

        const expansion& a30 = expansion_dot_at(p3,q0,p0, dim).scale_fast(2.0);
        const expansion& a31 = expansion_dot_at(p3,q1,p0, dim).scale_fast(2.0);
        const expansion& a32 = expansion_dot_at(p3,q2,p0, dim).scale_fast(2.0);
        const expansion& a33 = expansion_dot_at(p3,q3,p0, dim).scale_fast(2.0);

        const expansion& a40 = expansion_dot_at(p4,q0,p0, dim).scale_fast(2.0);
        const expansion& a41 = expansion_dot_at(p4,q1,p0, dim).scale_fast(2.0);
        const expansion& a42 = expansion_dot_at(p4,q2,p0, dim).scale_fast(2.0);
        const expansion& a43 = expansion_dot_at(p4,q3,p0, dim).scale_fast(2.0);

        // [ b00 b01 b02 b03 ]           [  1   1   1   1  ]-1
        // [ b10 b11 b12 b13 ]           [ a10 a11 a12 a13 ]
        // [ b20 b21 b22 b23 ] = Delta * [ a20 a21 a22 a23 ]
        // [ b30 b31 b32 b33 ]           [ a30 a31 a32 a33 ]

        // Note: we could probably reuse some of the co-factors
        // (but for now I'd rather keep this form that is easier to
        //  read ... and to debug if need be !)

        const expansion& b00 = expansion_det3x3(a11, a12, a13, a21, a22, a23, a31, a32, a33);
        const expansion& b01 = expansion_det_111_2x3(a21, a22, a23, a31, a32, a33).negate();
        const expansion& b02 = expansion_det_111_2x3(a11, a12, a13, a31, a32, a33);
        const expansion& b03 = expansion_det_111_2x3(a11, a12, a13, a21, a22, a23).negate();

        const expansion& b10 = expansion_det3x3(a10, a12, a13, a20, a22, a23, a30, a32, a33).negate();
        const expansion& b11 = expansion_det_111_2x3(a20, a22, a23, a30, a32, a33);
        const expansion& b12 = expansion_det_111_2x3(a10, a12, a13, a30, a32, a33).negate();
        const expansion& b13 = expansion_det_111_2x3(a10, a12, a13, a20, a22, a23);

        const expansion& b20 = expansion_det3x3(a10, a11, a13, a20, a21, a23, a30, a31, a33);
        const expansion& b21 = expansion_det_111_2x3(a20, a21, a23, a30, a31, a33).negate();
        const expansion& b22 = expansion_det_111_2x3(a10, a11, a13, a30, a31, a33);
        const expansion& b23 = expansion_det_111_2x3(a10, a11, a13, a20, a21, a23).negate();

        const expansion& b30 = expansion_det3x3(a10, a11, a12, a20, a21, a22, a30, a31, a32).negate();
        const expansion& b31 = expansion_det_111_2x3(a20, a21, a22, a30, a31, a32);
        const expansion& b32 = expansion_det_111_2x3(a10, a11, a12, a30, a31, a32).negate();
        const expansion& b33 = expansion_det_111_2x3(a10, a11, a12, a20, a21, a22);

        const expansion& Delta = expansion_sum4(b00, b10, b20, b30);
        Sign Delta_sign = Delta.sign();
        geo_assert(Delta_sign != ZERO);

        //       [ Lambda0 ]   [ b01 b02 b03 ]   [ l1 ]   [ b00 ]
        //       [ Lambda1 ]   [ b11 b12 b13 ]   [ l2 ]   [ b10 ]
        // Delta [ Lambda2 ] = [ b21 b22 b23 ] * [ l3 ] + [ b20 ]
        //       [ Lambda3 ]   [ b31 b32 b33 ]   [ l4 ]   [ b30 ]

        const expansion& b01_l1 = expansion_product(b01, l1);
        const expansion& b02_l2 = expansion_product(b02, l2);
        const expansion& b03_l3 = expansion_product(b03, l3);
        const expansion& DeltaLambda0 = expansion_sum4(b01_l1, b02_l2, b03_l3, b00);

        const expansion& b11_l1 = expansion_product(b11, l1);
        const expansion& b12_l2 = expansion_product(b12, l2);
        const expansion& b13_l3 = expansion_product(b13, l3);
        const expansion& DeltaLambda1 = expansion_sum4(b11_l1, b12_l2, b13_l3, b10);

        const expansion& b21_l1 = expansion_product(b21, l1);
        const expansion& b22_l2 = expansion_product(b22, l2);
        const expansion& b23_l3 = expansion_product(b23, l3);
        const expansion& DeltaLambda2 = expansion_sum4(b21_l1, b22_l2, b23_l3, b20);

        const expansion& b31_l1 = expansion_product(b31, l1);
        const expansion& b32_l2 = expansion_product(b32, l2);
        const expansion& b33_l3 = expansion_product(b33, l3);
        const expansion& DeltaLambda3 = expansion_sum4(b31_l1, b32_l2, b33_l3, b30);

        // r = Delta*l4 - (
        //    a40*DeltaLambda0+
        //    a41*DeltaLambda1+
        //    a42*DeltaLambda2+
        //    a43*DeltaLambda3
        // )

        const expansion& r0 = expansion_product(Delta, l4);
        const expansion& r1 = expansion_product(a40, DeltaLambda0);
        const expansion& r2 = expansion_product(a41, DeltaLambda1);
        const expansion& r3 = expansion_product(a42, DeltaLambda2);
        const expansion& r4 = expansion_product(a43, DeltaLambda3);
        const expansion& r1234 = expansion_sum4(r1, r2, r3, r4);
        const expansion& r = expansion_diff(r0, r1234);
        Sign r_sign = r.sign();

        // Simulation of Simplicity (symbolic perturbation)
        if(r_sign == ZERO) {
            stats_side4.log_SOS();

            const double* p_sort[5];
            p_sort[0] = p0;
            p_sort[1] = p1;
            p_sort[2] = p2;
            p_sort[3] = p3;
            p_sort[4] = p4;
            SOS_sort(p_sort, p_sort + 5, dim);
            for(index_t i = 0; i < 5; ++i) {
                if(p_sort[i] == p0) {
                    const expansion& z1_0 = expansion_sum3(b01, b02, b03);
                    const expansion& z1 = expansion_product(a30, z1_0);
                    const expansion& z2_0 = expansion_sum3(b11, b12, b13);
                    const expansion& z2 = expansion_product(a31, z2_0);
                    const expansion& z3_0 = expansion_sum3(b21, b22, b23);
                    const expansion& z3 = expansion_product(a32, z3_0);
                    const expansion& z4_0 = expansion_sum3(b31, b32, b33);
                    const expansion& z4 = expansion_product(a33, z4_0);
                    const expansion& z1234 = expansion_sum4(z1, z2, z3, z4);
                    const expansion& z = expansion_diff(Delta, z1234);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta_sign * z_sign);
                    }
                } else if(p_sort[i] == p1) {
                    const expansion& z1 = expansion_product(a30, b01);
                    const expansion& z2 = expansion_product(a31, b11);
                    const expansion& z3 = expansion_product(a32, b21);
                    const expansion& z4 = expansion_product(a33, b31);
                    const expansion& z = expansion_sum4(z1, z2, z3, z4);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta_sign * z_sign);
                    }
                } else if(p_sort[i] == p2) {
                    const expansion& z1 = expansion_product(a30, b02);
                    const expansion& z2 = expansion_product(a31, b12);
                    const expansion& z3 = expansion_product(a32, b22);
                    const expansion& z4 = expansion_product(a33, b32);
                    const expansion& z = expansion_sum4(z1, z2, z3, z4);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta_sign * z_sign);
                    }
                } else if(p_sort[i] == p3) {
                    const expansion& z1 = expansion_product(a30, b03);
                    const expansion& z2 = expansion_product(a31, b13);
                    const expansion& z3 = expansion_product(a32, b23);
                    const expansion& z4 = expansion_product(a33, b33);
                    const expansion& z = expansion_sum4(z1, z2, z3, z4);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta_sign * z_sign);
                    }
                } else if(p_sort[i] == p4) {
                    return NEGATIVE;
                }
            }
            geo_assert_not_reached;
        }
        return Sign(r_sign * Delta_sign);
    }

    /**
     * \brief Implements side4() in 4d.
     */
    Sign side4_4d_SOS(
        const double* p0,
        const double* p1, const double* p2, const double* p3, const double* p4,
        const double* q0, const double* q1, const double* q2, const double* q3
    ) {
        Sign result = Sign(side4_4d_filter(p0, p1, p2, p3, p4, q0, q1, q2, q3));
        if(result == ZERO) {
            result = side4_exact_SOS(p0, p1, p2, p3, p4, q0, q1, q2, q3, 4);
        }
        return result;
    }

    /**
     * \brief Implements side4() in 6d.
     */
    Sign side4_6d_SOS(
        const double* p0,
        const double* p1, const double* p2, const double* p3, const double* p4,
        const double* q0, const double* q1, const double* q2, const double* q3
    ) {
        Sign result = Sign(side4_6d_filter(p0, p1, p2, p3, p4, q0, q1, q2, q3));
        if(result == ZERO) {
            result = side4_exact_SOS(p0, p1, p2, p3, p4, q0, q1, q2, q3, 6);
        }
        return result;
    }

    /**
     * \brief Implements side4() in 7d.
     */
    Sign side4_7d_SOS(
        const double* p0,
        const double* p1, const double* p2, const double* p3, const double* p4,
        const double* q0, const double* q1, const double* q2, const double* q3
    ) {
        Sign result = Sign(side4_7d_filter(p0, p1, p2, p3, p4, q0, q1, q2, q3));
        if(result == ZERO) {
            result = side4_exact_SOS(p0, p1, p2, p3, p4, q0, q1, q2, q3, 7);
        }
        return result;
    }

    /**
     * \brief Implements side4() in 7d.
     */
    Sign side4_8d_SOS(
        const double* p0,
        const double* p1, const double* p2, const double* p3, const double* p4,
        const double* q0, const double* q1, const double* q2, const double* q3
    ) {
        Sign result = Sign(side4_8d_filter(p0, p1, p2, p3, p4, q0, q1, q2, q3));
        if(result == ZERO) {
            result = side4_exact_SOS(p0, p1, p2, p3, p4, q0, q1, q2, q3, 8);
        }
        return result;
    }

    // ============ orient2d ==============================================

    Sign orient_2d_exact(
        const double* p0, const double* p1, const double* p2
    ) {
        stats_orient2d.log_exact();

        const expansion& a11 = expansion_diff(p1[0], p0[0]);
        const expansion& a12 = expansion_diff(p1[1], p0[1]);

        const expansion& a21 = expansion_diff(p2[0], p0[0]);
        const expansion& a22 = expansion_diff(p2[1], p0[1]);

        const expansion& Delta = expansion_det2x2(
            a11, a12, a21, a22
        );

        return Delta.sign();
    }


    // ============ orient3d ==============================================

    Sign orient_3d_exact(
        const double* p0, const double* p1,
        const double* p2, const double* p3
    ) {
        stats_orient3d.log_exact();

        const expansion& a11 = expansion_diff(p1[0], p0[0]);
        const expansion& a12 = expansion_diff(p1[1], p0[1]);
        const expansion& a13 = expansion_diff(p1[2], p0[2]);

        const expansion& a21 = expansion_diff(p2[0], p0[0]);
        const expansion& a22 = expansion_diff(p2[1], p0[1]);
        const expansion& a23 = expansion_diff(p2[2], p0[2]);

        const expansion& a31 = expansion_diff(p3[0], p0[0]);
        const expansion& a32 = expansion_diff(p3[1], p0[1]);
        const expansion& a33 = expansion_diff(p3[2], p0[2]);

        const expansion& Delta = expansion_det3x3(
            a11, a12, a13, a21, a22, a23, a31, a32, a33
        );

        return Delta.sign();
    }

    Sign side4h_3d_exact_SOS(
        const double* p0, const double* p1,
        const double* p2, const double* p3, const double* p4,
        double h0, double h1, double h2, double h3, double h4,
        bool sos = true
    ) {
        stats_orient3dh.log_exact();

        const expansion& a11 = expansion_diff(p1[0], p0[0]);
        const expansion& a12 = expansion_diff(p1[1], p0[1]);
        const expansion& a13 = expansion_diff(p1[2], p0[2]);
        const expansion& a14 = expansion_diff(h0,h1);

        const expansion& a21 = expansion_diff(p2[0], p0[0]);
        const expansion& a22 = expansion_diff(p2[1], p0[1]);
        const expansion& a23 = expansion_diff(p2[2], p0[2]);
        const expansion& a24 = expansion_diff(h0,h2);

        const expansion& a31 = expansion_diff(p3[0], p0[0]);
        const expansion& a32 = expansion_diff(p3[1], p0[1]);
        const expansion& a33 = expansion_diff(p3[2], p0[2]);
        const expansion& a34 = expansion_diff(h0,h3);

        const expansion& a41 = expansion_diff(p4[0], p0[0]);
        const expansion& a42 = expansion_diff(p4[1], p0[1]);
        const expansion& a43 = expansion_diff(p4[2], p0[2]);
        const expansion& a44 = expansion_diff(h0,h4);

        // Note: we could probably reuse some of the 2x2 co-factors
        // (but for now I'd rather keep this form that is easier to
        //  read ... and to debug if need be !)
        const expansion& Delta1 = expansion_det3x3(
            a21, a22, a23,
            a31, a32, a33,
            a41, a42, a43
        );
        const expansion& Delta2 = expansion_det3x3(
            a11, a12, a13,
            a31, a32, a33,
            a41, a42, a43
        );
        const expansion& Delta3 = expansion_det3x3(
            a11, a12, a13,
            a21, a22, a23,
            a41, a42, a43
        );
        const expansion& Delta4 = expansion_det3x3(
            a11, a12, a13,
            a21, a22, a23,
            a31, a32, a33
        );

        Sign Delta4_sign = Delta4.sign();
        geo_assert(Delta4_sign != ZERO);

        const expansion& r_1 = expansion_product(Delta1, a14);
        const expansion& r_2 = expansion_product(Delta2, a24).negate();
        const expansion& r_3 = expansion_product(Delta3, a34);
        const expansion& r_4 = expansion_product(Delta4, a44).negate();
        const expansion& r = expansion_sum4(r_1, r_2, r_3, r_4);

        Sign r_sign = r.sign();

        // Simulation of Simplicity (symbolic perturbation)
        if(sos && r_sign == ZERO) {
            stats_orient3dh.log_SOS();
            const double* p_sort[5];
            p_sort[0] = p0;
            p_sort[1] = p1;
            p_sort[2] = p2;
            p_sort[3] = p3;
            p_sort[4] = p4;

            SOS_sort(p_sort, p_sort + 5, 3);
            for(index_t i = 0; i < 5; ++i) {
                if(p_sort[i] == p0) {
                    const expansion& z1 = expansion_diff(Delta2, Delta1);
                    const expansion& z2 = expansion_diff(Delta4, Delta3);
                    const expansion& z = expansion_sum(z1, z2);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta4_sign * z_sign);
                    }
                } else if(p_sort[i] == p1) {
                    Sign Delta1_sign = Delta1.sign();
                    if(Delta1_sign != ZERO) {
                        return Sign(Delta4_sign * Delta1_sign);
                    }
                } else if(p_sort[i] == p2) {
                    Sign Delta2_sign = Delta2.sign();
                    if(Delta2_sign != ZERO) {
                        return Sign(-Delta4_sign * Delta2_sign);
                    }
                } else if(p_sort[i] == p3) {
                    Sign Delta3_sign = Delta3.sign();
                    if(Delta3_sign != ZERO) {
                        return Sign(Delta4_sign * Delta3_sign);
                    }
                } else if(p_sort[i] == p4) {
                    return NEGATIVE;
                }
            }
        }
        return Sign(Delta4_sign * r_sign);
    }


    Sign side3h_2d_exact_SOS(
        const double* p0, const double* p1,
        const double* p2, const double* p3,
        double h0, double h1, double h2, double h3,
        bool sos = true
    ) {

        const expansion& a11 = expansion_diff(p1[0], p0[0]);
        const expansion& a12 = expansion_diff(p1[1], p0[1]);
        const expansion& a13 = expansion_diff(h0,h1);

        const expansion& a21 = expansion_diff(p2[0], p0[0]);
        const expansion& a22 = expansion_diff(p2[1], p0[1]);
        const expansion& a23 = expansion_diff(h0,h2);

        const expansion& a31 = expansion_diff(p3[0], p0[0]);
        const expansion& a32 = expansion_diff(p3[1], p0[1]);
        const expansion& a33 = expansion_diff(h0,h3);

        const expansion& Delta1 = expansion_det2x2(
            a21, a22,
            a31, a32
        );
        const expansion& Delta2 = expansion_det2x2(
            a11, a12,
            a31, a32
        );
        const expansion& Delta3 = expansion_det2x2(
            a11, a12,
            a21, a22
        );

        Sign Delta3_sign = Delta3.sign();
        geo_assert(Delta3_sign != ZERO);

        const expansion& r_1 = expansion_product(Delta1, a13);
        const expansion& r_2 = expansion_product(Delta2, a23).negate();
        const expansion& r_3 = expansion_product(Delta3, a33);
        const expansion& r = expansion_sum3(r_1, r_2, r_3);

        Sign r_sign = r.sign();

        // Simulation of Simplicity (symbolic perturbation)
        if(sos && r_sign == ZERO) {
            const double* p_sort[4];
            p_sort[0] = p0;
            p_sort[1] = p1;
            p_sort[2] = p2;
            p_sort[3] = p3;
            SOS_sort(p_sort, p_sort + 4, 2);
            for(index_t i = 0; i < 4; ++i) {
                if(p_sort[i] == p0) {
                    const expansion& z1 = expansion_diff(Delta2, Delta1);
                    const expansion& z = expansion_sum(z1, Delta3);
                    Sign z_sign = z.sign();
                    if(z_sign != ZERO) {
                        return Sign(Delta3_sign * z_sign);
                    }
                } else if(p_sort[i] == p1) {
                    Sign Delta1_sign = Delta1.sign();
                    if(Delta1_sign != ZERO) {
                        return Sign(Delta3_sign * Delta1_sign);
                    }
                } else if(p_sort[i] == p2) {
                    Sign Delta2_sign = Delta2.sign();
                    if(Delta2_sign != ZERO) {
                        return Sign(-Delta3_sign * Delta2_sign);
                    }
                } else if(p_sort[i] == p3) {
                    return NEGATIVE;
                }
            }
        }
        return Sign(Delta3_sign * r_sign);
    }


    // ================================ det and dot =======================

    /**
     * \brief Computes the sign of the determinant of a 3x3
     *  matrix formed by three 3d points using exact arithmetics.
     * \param[in] p0 , p1 , p2 the three points
     * \return the sign of the determinant of the matrix.
     */
    Sign det_3d_exact(
        const double* p0, const double* p1, const double* p2
    ) {
        stats_det3d.log_exact();

        const expansion& p0_0 = expansion_create(p0[0]);
        const expansion& p0_1 = expansion_create(p0[1]);
        const expansion& p0_2 = expansion_create(p0[2]);

        const expansion& p1_0 = expansion_create(p1[0]);
        const expansion& p1_1 = expansion_create(p1[1]);
        const expansion& p1_2 = expansion_create(p1[2]);

        const expansion& p2_0 = expansion_create(p2[0]);
        const expansion& p2_1 = expansion_create(p2[1]);
        const expansion& p2_2 = expansion_create(p2[2]);

        const expansion& Delta = expansion_det3x3(
            p0_0, p0_1, p0_2,
            p1_0, p1_1, p1_2,
            p2_0, p2_1, p2_2
        );

        return Delta.sign();
    }


    /**
     * \brief Tests whether three points are aligned using
     *  exact arithmetics.
     * \param[in] p0 , p1 , p2 the three points
     * \retval true if the three points are aligned.
     * \retval false otherwise.
     */
    bool aligned_3d_exact(
        const double* p0, const double* p1, const double* p2
    ) {
        const expansion& U_0 = expansion_diff(p1[0],p0[0]);
        const expansion& U_1 = expansion_diff(p1[1],p0[1]);
        const expansion& U_2 = expansion_diff(p1[2],p0[2]);

        const expansion& V_0 = expansion_diff(p2[0],p0[0]);
        const expansion& V_1 = expansion_diff(p2[1],p0[1]);
        const expansion& V_2 = expansion_diff(p2[2],p0[2]);

        const expansion& N_0 = expansion_det2x2(U_1, V_1, U_2, V_2);
        const expansion& N_1 = expansion_det2x2(U_2, V_2, U_0, V_0);
        const expansion& N_2 = expansion_det2x2(U_0, V_0, U_1, V_1);

        return(
            N_0.sign() == 0 &&
            N_1.sign() == 0 &&
            N_2.sign() == 0
        );
    }

    /**
     * \brief Computes the sign of the dot product between two
     *  vectors using exact arithmetics.
     * \param[in] p0 , p1 , p2 three 3d points.
     * \return the sign of the dot product between the vectors
     *  p0p1 and p0p2.
     */
    Sign dot_3d_exact(
        const double* p0, const double* p1, const double* p2
    ) {
        const expansion& U_0 = expansion_diff(p1[0],p0[0]);
        const expansion& U_1 = expansion_diff(p1[1],p0[1]);
        const expansion& U_2 = expansion_diff(p1[2],p0[2]);

        const expansion& V_0 = expansion_diff(p2[0],p0[0]);
        const expansion& V_1 = expansion_diff(p2[1],p0[1]);
        const expansion& V_2 = expansion_diff(p2[2],p0[2]);

        const expansion& UV_0 = expansion_product(U_0, V_0);
        const expansion& UV_1 = expansion_product(U_1, V_1);
        const expansion& UV_2 = expansion_product(U_2, V_2);

        const expansion& Delta = expansion_sum3(UV_0, UV_1, UV_2);

        return Delta.sign();
    }

    /**
     * \brief Compares two dot products using exact arithmetics.
     * \param[in] v0 , v1 , v2 three vectors
     * \return the sign of v0.v1 - v0.v2
     */
    Sign dot_compare_3d_exact(
        const double* v0, const double* v1, const double* v2
    ) {
        const expansion& d01_0 = expansion_product(v0[0], v1[0]);
        const expansion& d01_1 = expansion_product(v0[1], v1[1]);
        const expansion& d01_2 = expansion_product(v0[2], v1[2]);
        const expansion& d01_12 = expansion_sum(d01_1, d01_2);
        const expansion& d01 = expansion_sum(d01_0, d01_12);

        const expansion& d02_0 = expansion_product(v0[0], v2[0]);
        const expansion& d02_1 = expansion_product(v0[1], v2[1]);
        const expansion& d02_2 = expansion_product(v0[2], v2[2]);
        const expansion& d02_12 = expansion_sum(d02_1, d02_2);
        const expansion& d02 = expansion_sum(d02_0, d02_12);

        const expansion& result = expansion_diff(d01, d02);

        return result.sign();
    }
}

/****************************************************************************/

namespace GEO {

    namespace PCK {

        void set_SOS_mode(SOSMode m) {
            SOS_mode_ = m;
        }

        SOSMode get_SOS_mode() {
            return SOS_mode_;
        }


        Sign side1_SOS(
            const double* p0, const double* p1,
            const double* q0,
            coord_index_t DIM
        ) {
            stats_side1.log_invoke();
            switch(DIM) {
            case 3:
                return side1_3d_SOS(p0, p1, q0);
            case 4:
                return side1_4d_SOS(p0, p1, q0);
            case 6:
                return side1_6d_SOS(p0, p1, q0);
            case 7:
                return side1_7d_SOS(p0, p1, q0);
            case 8:
                return side1_8d_SOS(p0, p1, q0);
            }
            geo_assert_not_reached;
        }

        Sign side2_SOS(
            const double* p0, const double* p1, const double* p2,
            const double* q0, const double* q1,
            coord_index_t DIM
        ) {
            stats_side2.log_invoke();
            switch(DIM) {
            case 3:
                return side2_3d_SOS(p0, p1, p2, q0, q1);
            case 4:
                return side2_4d_SOS(p0, p1, p2, q0, q1);
            case 6:
                return side2_6d_SOS(p0, p1, p2, q0, q1);
            case 7:
                return side2_7d_SOS(p0, p1, p2, q0, q1);
            case 8:
                return side2_8d_SOS(p0, p1, p2, q0, q1);
            }
            geo_assert_not_reached;
        }

        Sign side3_SOS(
            const double* p0, const double* p1,
            const double* p2, const double* p3,
            const double* q0, const double* q1, const double* q2,
            coord_index_t DIM
        ) {
            stats_side3.log_invoke();
            switch(DIM) {
            case 3:
                return side3_3d_SOS(p0, p1, p2, p3, q0, q1, q2);
            case 4:
                return side3_4d_SOS(p0, p1, p2, p3, q0, q1, q2);
            case 6:
                return side3_6d_SOS(p0, p1, p2, p3, q0, q1, q2);
            case 7:
                return side3_7d_SOS(p0, p1, p2, p3, q0, q1, q2);
            case 8:
                return side3_8d_SOS(p0, p1, p2, p3, q0, q1, q2);
            }
            geo_assert_not_reached;
        }


        Sign side3_3dlifted_SOS(
            const double* p0, const double* p1,
            const double* p2, const double* p3,
            double h0, double h1, double h2, double h3,
            const double* q0, const double* q1, const double* q2,
            bool SOS
        ) {
            Sign result = Sign(
                side3h_3d_filter(p0, p1, p2, p3, h0, h1, h2, h3, q0, q1, q2)
            );
            if(SOS && result == ZERO) {
                result = side3h_exact_SOS(
                    p0, p1, p2, p3, h0, h1, h2, h3, q0, q1, q2
                );
            }
            return result;
        }

        Sign side4_SOS(
            const double* p0,
            const double* p1, const double* p2,
            const double* p3, const double* p4,
            const double* q0, const double* q1,
            const double* q2, const double* q3,
            coord_index_t DIM
        ) {
            switch(DIM) {
            case 3:
                // 3d is a special case for side4()
                //   (intrinsic dim == ambient dim)
                // therefore embedding tet q0,q1,q2,q3 is not needed.
                // WARNING: cnt_side4_total is not incremented here,
                // because it is
                // incremented in side4_3d_SOS().
                return side4_3d_SOS(p0, p1, p2, p3, p4);
            case 4:
                stats_side4.log_invoke();
                return side4_4d_SOS(p0, p1, p2, p3, p4, q0, q1, q2, q3);
            case 6:
                stats_side4.log_invoke();
                return side4_6d_SOS(p0, p1, p2, p3, p4, q0, q1, q2, q3);
            case 7:
                stats_side4.log_invoke();
                return side4_7d_SOS(p0, p1, p2, p3, p4, q0, q1, q2, q3);
            case 8:
                stats_side4.log_invoke();
                return side4_8d_SOS(p0, p1, p2, p3, p4, q0, q1, q2, q3);
            }
            geo_assert_not_reached;
        }


        Sign side4_3d(
            const double* p0, const double* p1, const double* p2,
            const double* p3, const double* p4
        ) {
            stats_side4.log_invoke();
            Sign result = Sign(side4_3d_filter(p0, p1, p2, p3, p4));
            if(result == 0) {
                // last argument is false: do not apply symbolic perturbation
                result = side4_3d_exact_SOS(p0, p1, p2, p3, p4, false);
            }
            return result;
        }

        Sign side4_3d_SOS(
            const double* p0, const double* p1,
            const double* p2, const double* p3,
            const double* p4
        ) {
            stats_side4.log_invoke();
            Sign result = Sign(side4_3d_filter(p0, p1, p2, p3, p4));
            if(result == 0) {
                result = side4_3d_exact_SOS(p0, p1, p2, p3, p4);
            }
            return result;
        }


        Sign in_sphere_3d_SOS(
            const double* p0, const double* p1,
            const double* p2, const double* p3,
            const double* p4
        ) {
            // in_sphere_3d is simply implemented using side4_3d.
            // Both predicates are equivalent through duality as can
            // be easily seen:
            // side4_3d(p0,p1,p2,p3,p4) returns POSITIVE if
            //    d(q,p0) < d(q,p4)
            //    where q denotes the circumcenter of (p0,p1,p2,p3)
            // Note that d(q,p0) = R  (radius of circumscribed sphere)
            // In other words, side4_3d(p0,p1,p2,p3,p4) returns POSITIVE if
            //   d(q,p4) > R which means whenever p4 is not in the
            //   circumscribed sphere of (p0,p1,p2,p3).
            // Therefore:
            // in_sphere_3d(p0,p1,p2,p3,p4) = -side4_3d(p0,p1,p2,p3,p4)

            stats_side4.log_invoke();

            // This specialized filter supposes that orient_3d(p0,p1,p2,p3) > 0

            Sign result = Sign(in_sphere_3d_filter_optim(p0, p1, p2, p3, p4));

            if(result == 0) {
                result = side4_3d_exact_SOS(p0, p1, p2, p3, p4);
            }
            return Sign(-result);
        }

        Sign GEOGRAM_API in_circle_2d_SOS(
            const double* p0, const double* p1, const double* p2,
            const double* p3
        ) {
            // in_circle_2d is simply implemented using side3_2d.
            // Both predicates are equivalent through duality as can
            // be easily seen:
            // side3_2d(p0,p1,p2,p3,p0,p1,p2) returns POSITIVE if
            //    d(q,p0) < d(q,p3)
            //    where q denotes the circumcenter of (p0,p1,p2)
            // Note that d(q,p0) = R  (radius of circumscribed circle)
            // In other words, side3_2d(p0,p1,p2,p3,p4) returns POSITIVE if
            //   d(q,p3) > R which means whenever p3 is not in the
            //   circumscribed circle of (p0,p1,p2).
            // Therefore:
            // in_circle_2d(p0,p1,p2,p3) = -side3_2d(p0,p1,p2,p3)

            // TODO: implement specialized filter like the one used
            // by "in-sphere".
            Sign s = Sign(-side3_2d_filter(p0, p1, p2, p3, p0, p1, p2));
            if(s != ZERO) {
                return s;
            }
            return Sign(-side3_exact_SOS(p0, p1, p2, p3, p0, p1, p2, 2));
        }

        Sign GEOGRAM_API in_circle_3d_SOS(
            const double* p0, const double* p1, const double* p2,
            const double* p3
        ) {
            // in_circle_3d is simply implemented using side3_3d.
            // Both predicates are equivalent through duality as can
            // be easily seen:
            // side3_3d(p0,p1,p2,p3,p0,p1,p2) returns POSITIVE if
            //    d(q,p0) < d(q,p3)
            //    where q denotes the circumcenter of (p0,p1,p2)
            // Note that d(q,p0) = R  (radius of circumscribed circle)
            // In other words, side3_3d(p0,p1,p2,p3,p4) returns POSITIVE if
            //   d(q,p3) > R which means whenever p3 is not in the
            //   circumscribed circle of (p0,p1,p2).
            // Therefore:
            // in_circle_3d(p0,p1,p2,p3) = -side3_3d(p0,p1,p2,p3)
            return Sign(-side3_3d_SOS(p0,p1,p2,p3,p0,p1,p2));
        }

        Sign GEOGRAM_API in_circle_3dlifted_SOS(
            const double* p0, const double* p1, const double* p2,
            const double* p3,
            double h0, double h1, double h2, double h3,
            bool SOS
        ) {
            // in_circle_3dlifted is simply implemented using side3_3dlifted.
            // Both predicates are equivalent through duality
            // (see comment in in_circle_3d_SOS(), the same
            //  remark applies).
            return Sign(
                -side3_3dlifted_SOS(p0,p1,p2,p3,h0,h1,h2,h3,p0,p1,p2,SOS)
            );
        }


        Sign orient_2d(
            const double* p0, const double* p1, const double* p2
        ) {
            stats_orient2d.log_invoke();
            Sign result = Sign(orient_2d_filter(p0, p1, p2));
            if(result == 0) {
                result = orient_2d_exact(p0, p1, p2);
            }
            return result;
        }

        Sign orient_2dlifted_SOS(
            const double* p0, const double* p1,
            const double* p2, const double* p3,
            double h0, double h1, double h2, double h3
        ) {
            Sign result = Sign(
                side3_2dlifted_2d_filter(
                    p0, p1, p2, p3, h0, h1, h2, h3
                )
            );
            if(result == 0) {
                result = side3h_2d_exact_SOS(
                    p0, p1, p2, p3, h0, h1, h2, h3
                );
            }
            // orient_3d() is opposite to side3h()
            // (like in_sphere() that is opposite to side3())
            return result;
        }


        Sign orient_3d(
            const double* p0, const double* p1,
            const double* p2, const double* p3
        ) {
            stats_orient3d.log_invoke();
            Sign result = Sign(orient_3d_filter(p0, p1, p2, p3));
            if(result == 0) {
                result = orient_3d_exact(p0, p1, p2, p3);
            }
            return result;
        }


        Sign orient_3dlifted(
            const double* p0, const double* p1,
            const double* p2, const double* p3, const double* p4,
            double h0, double h1, double h2, double h3, double h4
        ) {
            stats_orient3dh.log_invoke();
            Sign result = Sign(
                side4h_3d_filter(
                    p0, p1, p2, p3, p4, h0, h1, h2, h3, h4
                )
            );
            if(result == 0) {
                // last argument is false -> do not perturb.
                result = side4h_3d_exact_SOS(
                    p0, p1, p2, p3, p4, h0, h1, h2, h3, h4, false
                );
            }
            // orient_4d() is opposite to side4h()
            // (like in_sphere() that is opposite to side4())
            return Sign(-result);
        }

        Sign orient_3dlifted_SOS(
            const double* p0, const double* p1,
            const double* p2, const double* p3, const double* p4,
            double h0, double h1, double h2, double h3, double h4
        ) {
            stats_orient3dh.log_invoke();
            Sign result = Sign(
                side4h_3d_filter(
                    p0, p1, p2, p3, p4, h0, h1, h2, h3, h4
                )
            );
            if(result == 0) {
                result = side4h_3d_exact_SOS(
                    p0, p1, p2, p3, p4, h0, h1, h2, h3, h4
                );
            }
            // orient_4d() is opposite to side4h()
            // (like in_sphere() that is opposite to side4())
            return Sign(-result);
        }

        Sign det_3d(
            const double* p0, const double* p1, const double* p2
        ) {
            stats_det3d.log_invoke();
            Sign result = Sign(
                det_3d_filter(p0, p1, p2)
            );
            if(result == 0) {
                result = det_3d_exact(p0, p1, p2);
            }
            return result;
        }


        Sign det_4d(
            const double* p0, const double* p1,
            const double* p2, const double* p3
        ) {
            stats_det4d.log_invoke();
            Sign result = Sign(
                det_4d_filter(p0, p1, p2, p3)
            );

            if(result == 0) {
                stats_det4d.log_exact();

                const expansion& p0_0 = expansion_create(p0[0]);
                const expansion& p0_1 = expansion_create(p0[1]);
                const expansion& p0_2 = expansion_create(p0[2]);
                const expansion& p0_3 = expansion_create(p0[3]);

                const expansion& p1_0 = expansion_create(p1[0]);
                const expansion& p1_1 = expansion_create(p1[1]);
                const expansion& p1_2 = expansion_create(p1[2]);
                const expansion& p1_3 = expansion_create(p1[3]);

                const expansion& p2_0 = expansion_create(p2[0]);
                const expansion& p2_1 = expansion_create(p2[1]);
                const expansion& p2_2 = expansion_create(p2[2]);
                const expansion& p2_3 = expansion_create(p2[3]);

                const expansion& p3_0 = expansion_create(p3[0]);
                const expansion& p3_1 = expansion_create(p3[1]);
                const expansion& p3_2 = expansion_create(p3[2]);
                const expansion& p3_3 = expansion_create(p3[3]);

                result = sign_of_expansion_determinant(
                    p0_0, p0_1, p0_2, p0_3,
                    p1_0, p1_1, p1_2, p1_3,
                    p2_0, p2_1, p2_2, p2_3,
                    p3_0, p3_1, p3_2, p3_3
                );
            }
            return result;
        }


        Sign det_compare_4d(
            const double* p0, const double* p1,
            const double* p2, const double* p3,
            const double* p4
        ) {
            Sign result = Sign(
                det_compare_4d_filter(p0, p1, p2, p3, p4)
            );
            if(result == 0) {
                const expansion& p0_0 = expansion_create(p0[0]);
                const expansion& p0_1 = expansion_create(p0[1]);
                const expansion& p0_2 = expansion_create(p0[2]);
                const expansion& p0_3 = expansion_create(p0[3]);

                const expansion& p1_0 = expansion_create(p1[0]);
                const expansion& p1_1 = expansion_create(p1[1]);
                const expansion& p1_2 = expansion_create(p1[2]);
                const expansion& p1_3 = expansion_create(p1[3]);

                const expansion& p2_0 = expansion_create(p2[0]);
                const expansion& p2_1 = expansion_create(p2[1]);
                const expansion& p2_2 = expansion_create(p2[2]);
                const expansion& p2_3 = expansion_create(p2[3]);

                const expansion& a3_0 = expansion_diff(p4[0],p3[0]);
                const expansion& a3_1 = expansion_diff(p4[1],p3[1]);
                const expansion& a3_2 = expansion_diff(p4[2],p3[2]);
                const expansion& a3_3 = expansion_diff(p4[3],p3[3]);

                result = sign_of_expansion_determinant(
                    p0_0, p0_1, p0_2, p0_3,
                    p1_0, p1_1, p1_2, p1_3,
                    p2_0, p2_1, p2_2, p2_3,
                    a3_0, a3_1, a3_2, a3_3
                );
            }
            return result;
        }


        bool aligned_3d(
            const double* p0, const double* p1, const double* p2
        ) {
            /*
              Sign result = Sign(
              aligned_3d_filter(p0,p1,p2)
              );
              if(result != 0) {
              return false;
              }
            */
            return aligned_3d_exact(p0, p1, p2);
        }

        Sign dot_3d(
            const double* p0, const double* p1, const double* p2
        ) {
            Sign result = Sign(det_3d_filter(p0, p1, p2));
            if(result == 0) {
                result = dot_3d_exact(p0, p1, p2);
            }
            return result;
        }

        Sign dot_compare_3d(
            const double* v0, const double* v1, const double* v2
        ) {
            Sign result = Sign(dot_compare_3d_filter(v0, v1, v2));
            if(result == 0) {
                result = dot_compare_3d_exact(v0, v1, v2);
            }
            return result;
        }


        bool points_are_identical_2d(
            const double* p1,
            const double* p2
        ) {
            return
                (p1[0] == p2[0]) &&
                (p1[1] == p2[1])
                ;
        }

        bool points_are_identical_3d(
            const double* p1,
            const double* p2
        ) {
            return
                (p1[0] == p2[0]) &&
                (p1[1] == p2[1]) &&
                (p1[2] == p2[2])
                ;
        }

        bool points_are_colinear_3d(
            const double* p1,
            const double* p2,
            const double* p3
        ) {
            // Colinearity is tested by using four coplanarity
            // tests with four points that are not coplanar.
            // TODO: use PCK::aligned_3d() instead (to be tested)
            static const double q000[3] = {0.0, 0.0, 0.0};
            static const double q001[3] = {0.0, 0.0, 1.0};
            static const double q010[3] = {0.0, 1.0, 0.0};
            static const double q100[3] = {1.0, 0.0, 0.0};
            return
                PCK::orient_3d(p1, p2, p3, q000) == ZERO &&
                PCK::orient_3d(p1, p2, p3, q001) == ZERO &&
                PCK::orient_3d(p1, p2, p3, q010) == ZERO &&
                PCK::orient_3d(p1, p2, p3, q100) == ZERO
                ;
        }

        void initialize() {
            expansion::initialize();
        }

        void terminate() {
            // Nothing to do.
        }

        void show_stats() {
            PredicateStats::show_all_stats();
            expansion::show_all_stats();
        }
    }
}
