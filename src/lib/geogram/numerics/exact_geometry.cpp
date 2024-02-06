/*
 *  Copyright (c) 2000-2023 Inria
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

#include <geogram/numerics/exact_geometry.h>
#include <geogram/numerics/interval_nt.h>
#include <geogram/numerics/predicates.h>
#include <geogram/numerics/PCK.h>
#include <geogram/basic/logger.h>

namespace GEO {

    
    namespace PCK {

        Sign orient_2d(
            const vec2HE& p0, const vec2HE& p1, const vec2HE& p2
        ) {
            static PredicateStats stats("orient_2d(vec2HE)");
            stats.log_invoke();
            // Filter, using interval arithmetics
            {
                interval_nt::Rounding rounding;
                interval_nt Delta = det3x3(
                    interval_nt(p0.x),interval_nt(p0.y),interval_nt(p0.w),
                    interval_nt(p1.x),interval_nt(p1.y),interval_nt(p1.w),
                    interval_nt(p2.x),interval_nt(p2.y),interval_nt(p2.w)
                );
                interval_nt::Sign2 s = Delta.sign();
                if(interval_nt::sign_is_determined(s)) {
                    return Sign(
                        interval_nt::convert_sign(s)*
                        p0.w.sign()*p1.w.sign()*p2.w.sign()
                    );
                }
            }
            stats.log_exact();
#ifdef GEO_HAS_BIG_STACK            
            const expansion& Delta = expansion_det3x3(
                p0.x.rep(), p0.y.rep(), p0.w.rep(),
                p1.x.rep(), p1.y.rep(), p1.w.rep(),
                p2.x.rep(), p2.y.rep(), p2.w.rep()
            );
#else
            expansion_nt Delta = det3x3(
                p0.x, p0.y, p0.w,
                p1.x, p1.y, p1.w,
                p2.x, p2.y, p2.w
            );
#endif            
            return Sign(
                Delta.sign()*
                p0.w.rep().sign()*
                p1.w.rep().sign()*
                p2.w.rep().sign()
            );
        }

        Sign orient_3d(
            const vec3HE& p0, const vec3HE& p1,
            const vec3HE& p2, const vec3HE& p3
        ) {
            static PredicateStats stats("orient_3d(vec3HE)");
            stats.log_invoke();
            // Filter
            {
                interval_nt::Rounding rounding;                
                vec3HI p0I(p0);
                vec3HI U = vec3HI(p1)-p0I;
                vec3HI V = vec3HI(p2)-p0I;
                vec3HI W = vec3HI(p3)-p0I;
                interval_nt::Sign2 s1 = U.w.sign();
                interval_nt::Sign2 s2 = V.w.sign();
                interval_nt::Sign2 s3 = W.w.sign();
                if(
                    interval_nt::sign_is_non_zero(s1) &&
                    interval_nt::sign_is_non_zero(s2) &&
                    interval_nt::sign_is_non_zero(s3) 
                ) {
                    interval_nt Delta = det3x3(
                        U.x, U.y, U.z,
                        V.x, V.y, V.z,
                        W.x, W.y, W.z                
                    );
                    interval_nt::Sign2 s = Delta.sign();
                    if(interval_nt::sign_is_non_zero(s)) {
                        return Sign(
                            interval_nt::convert_sign(s)*
                            interval_nt::convert_sign(s1)*
                            interval_nt::convert_sign(s2)*
                            interval_nt::convert_sign(s3)
                        );
                    }
                }
            }

            stats.log_exact();
            
            vec3HE U = p1-p0;
            vec3HE V = p2-p0;
            vec3HE W = p3-p0;

            // Here we do not use expansion_det3x3() (that
            // allocates on the stack), because
            // RadialSort uses generted points that
            // can have very looonng expansions that
            // can cause stack overflow.
            expansion_nt Delta = det3x3(
                U.x, U.y, U.z,
                V.x, V.y, V.z,
                W.x, W.y, W.z                
            );
                
            Sign result = Sign(
                Delta.sign()*
                U.w.rep().sign()*
                V.w.rep().sign()*
                W.w.rep().sign()
            );
            
            return result;
        }

        Sign orient_2d_projected(
            const vec3HE& p0, const vec3HE& p1, const vec3HE& p2,
            coord_index_t axis
        ) {
            static PredicateStats stats("orient_2d_projected(vec3HE)");
            stats.log_invoke();
            
            coord_index_t u = coord_index_t((axis+1)%3);
            coord_index_t v = coord_index_t((axis+2)%3);

            // Filter, using interval arithmetics
            { 
                interval_nt::Rounding rounding;

                interval_nt Delta = det3x3(
                    interval_nt(p0[u]),interval_nt(p0[v]),interval_nt(p0.w),
                    interval_nt(p1[u]),interval_nt(p1[v]),interval_nt(p1.w),
                    interval_nt(p2[u]),interval_nt(p2[v]),interval_nt(p2.w)
                );
                interval_nt::Sign2 s = Delta.sign();
                if(interval_nt::sign_is_determined(s)) {
                    return Sign(
                        interval_nt::convert_sign(s)*
                        p0.w.sign()*p1.w.sign()*p2.w.sign()
                    );
                }
            }

            stats.log_exact();
            
            Sign result = ZERO;
            {
#ifdef GEO_HAS_BIG_STACK                
                const expansion& Delta = expansion_det3x3(
                    p0[u].rep(), p0[v].rep(), p0.w.rep(),
                    p1[u].rep(), p1[v].rep(), p1.w.rep(),
                    p2[u].rep(), p2[v].rep(), p2.w.rep()
                );
#else
                expansion_nt Delta = det3x3(
                    p0[u], p0[v], p0.w,
                    p1[u], p1[v], p1.w,
                    p2[u], p2[v], p2.w
                );
#endif                
                result = Sign(
                    Delta.sign()*
                    p0.w.rep().sign()*
                    p1.w.rep().sign()*
                    p2.w.rep().sign()
                );
            }
            return result;
        }

        Sign dot_2d(const vec2HE& p0, const vec2HE& p1, const vec2HE& p2) {
            static PredicateStats stats("dot_2d(vec2HE)");
            stats.log_invoke();
            
            // TODO: filter
            
            vec2HE U = p1 - p0;
            vec2HE V = p2 - p0;
#ifdef GEO_HAS_BIG_STACK            
            const expansion& x1x2 = expansion_product(U.x.rep(), V.x.rep());
            const expansion& y1y2 = expansion_product(U.y.rep(), V.y.rep());
            const expansion& S = expansion_sum(x1x2, y1y2);
#else
            expansion_nt S = U.x*V.x+U.y*V.y;
#endif            
            return Sign(S.sign()*U.w.sign()*V.w.sign());
        }

/******************************************************************************/

        /**
         * \brief Computes the sign of 
         *   det3x3(x1,y1,1,x2,y2,1,x3,y3,1)
         * \param[in] p1 , p2 , p3 the three points in 
         *   homogeneous exact coordiates
         * \return the sign of the determinant
         */
        static inline Sign det3_111_sign(
            const vec2HE& p1,
            const vec2HE& p2,
            const vec2HE& p3
        ) {
            expansion_nt m1 = det2x2(p2.x, p2.y, p3.x, p3.y);
            expansion_nt m2 = det2x2(p1.x, p1.y, p3.x, p3.y);
            expansion_nt m3 = det2x2(p1.x, p1.y, p2.x, p2.y);
            m1.optimize(); m2.optimize(); m3.optimize();
            expansion_nt D = p1.w*m1-p2.w*m2+p3.w*m3 ;
            return Sign(p1.w.sign()*p2.w.sign()*p3.w.sign()*D.sign());
        }

        Sign incircle_2d_SOS_with_lengths(
            const vec2HE& p0, const vec2HE& p1,
            const vec2HE& p2, const vec2HE& p3,
            double l0, double l1, double l2, double l3
        ) {
            static PredicateStats stats("incircle_2d_SOS_with_lengths(vec2HE)");
            stats.log_invoke();
            
            Sign result = ZERO;

            // "Documentation is a love letter that you write to your
            //  future self." - Damian Conway  (or that you write to the
            //  poor guy who will have one day to dive again in this stuff,
            //  but it's probably me anyway)
            //
            // Determinant to compute:
            // | x0 y0 l0 1 |
            // | x1 y1 l1 1 |
            // | x2 y2 l2 1 |
            // | x3 y3 l3 1 |
            // where li = xi^2 + yi^2
            // (positive if (p0,p1,p2) counterclockwise and p3 in circumcircle
            //  of (p0,p1,p2)). Sign changes if (p0,p1,p2) is clockwise). 
            // We suppose that the li's are *given* numbers (it is like
            // perturbating a regular (weighted) triangulation instead of a
            // Delaunay triangulation).
            // It allows to use arithmetic expansions without
            // overflowing/underflowing too soon.
            //
            // Subtract last row to first three rows
            // (does not change determinant):
            //
            // | x0-x3 y0-y3 l0-l3 0 |
            // | x1-x3 y1-y3 l1-l3 0 |
            // | x2-x3 y2-y3 l2-l3 0 |            
            // | x3    y3    l3    1 |
            //
            // Develop along last column:
            // | x0-x3 y0-y3 l0-l3 |
            // | x1-x3 y1-y3 l1-l3 |
            // | x2-x3 y2-y3 l2-l3 |            
            //
            // let (Xi+1,Yi+1,Wi+1) = (xi,yi)-(x3,y3) in homogeneous coordinates
            // let Li+1 = li-l3:
            // | X1/W1 Y1/W1 L1 |
            // | X2/W2 Y2/W2 L2 |
            // | X3/W3 Y3/W3 L3 |
            //
            // Develop along last column, factor-out the Wi's
            //
            //           | X2 Y2 |             | X1 Y1 |             | X1 Y1 |
            // (L1/W2W3) | X3 Y3 | - (L2/W1W3) | X3 Y3 | + (L3/W1W2) | X2 Y2 |
            //
            // Multiply everything by W1W2W3:
            //
            //            | X2 Y2 |        | X1 Y1 |        | X1 Y1 |
            // sign( L1W1 | X3 Y3 | - L2W2 | X3 Y3 | + L3W3 | X2 Y2 | ) *
            // sign(W1) * sign(W2) * sign(W3)
            
            // The four approximated li's. It is OK since they will
            // always have the same value for the same vertex.
            // We do it like that because computing them exactly (and
            // properly propagating the wi^2's) makes expansions
            // overflow/underflow.
            // It is like perturbating a regular (weighted) triangulation
            // instead of a Delaunay triangulation.
            // However, if incircle(p1,p2,p3,p4) is lower than 0, it does
            // not imply that (p1,p2,p3,p4) forms a convex quadrilateral
            // (needs to be tested in addition, it is what CDT2d does
            // when exact_incircle_ is set to false).

            // We could also compute them each time, as in incircle_2d_SOS()
            // (but we are caching them in MeshSurfaceIntersection's temporary
            // Vertex objects), this gains 20-25% performance so it is
            // worth it.
            
            // Filter
            {
                interval_nt::Rounding rounding;                
                interval_nt l3I(l3);
                interval_nt L1 = interval_nt(l0) - l3I;
                interval_nt L2 = interval_nt(l1) - l3I;
                interval_nt L3 = interval_nt(l2) - l3I;

                vec2HI p3I(p3);
                vec2HI P1 = vec2HI(p0) - p3I;
                vec2HI P2 = vec2HI(p1) - p3I;
                vec2HI P3 = vec2HI(p2) - p3I;

                interval_nt::Sign2 s1 = P1.w.sign();
                interval_nt::Sign2 s2 = P2.w.sign();
                interval_nt::Sign2 s3 = P3.w.sign();

                if(
                    interval_nt::sign_is_non_zero(s1) &&
                    interval_nt::sign_is_non_zero(s2) &&
                    interval_nt::sign_is_non_zero(s3)
                ) {
                
                    interval_nt M1 = det2x2(P2.x, P2.y, P3.x, P3.y);
                    interval_nt M2 = det2x2(P1.x, P1.y, P3.x, P3.y);
                    interval_nt M3 = det2x2(P1.x, P1.y, P2.x, P2.y);

                    interval_nt D = L1*P1.w*M1
                                  - L2*P2.w*M2
                                  + L3*P3.w*M3 ;

                    interval_nt::Sign2 s = D.sign();
                    if(interval_nt::sign_is_non_zero(s)) {
                        return Sign(
                            interval_nt::convert_sign(s) *
                            interval_nt::convert_sign(s1) *
                            interval_nt::convert_sign(s2) *
                            interval_nt::convert_sign(s3) 
                        );
                    }
                }
            }

            // Exact
            stats.log_exact();
            {
                // These ones can be computed on the stack even
                // under MacOSX since they are at most of length 2
                expansion_nt L1(expansion_nt::DIFF, l0, l3);
                expansion_nt L2(expansion_nt::DIFF, l1, l3);
                expansion_nt L3(expansion_nt::DIFF, l2, l3);
                L1.optimize(); L2.optimize(); L3.optimize();
                
                vec2HE P1 = p0 - p3;
                vec2HE P2 = p1 - p3;
                vec2HE P3 = p2 - p3;
                P1.optimize(); P2.optimize(); P3.optimize();
                

                expansion_nt M1 = det2x2(P2.x, P2.y, P3.x, P3.y);
                expansion_nt M2 = det2x2(P1.x, P1.y, P3.x, P3.y);
                expansion_nt M3 = det2x2(P1.x, P1.y, P2.x, P2.y);
                M1.optimize(); M2.optimize(); M3.optimize();
                
                expansion_nt D = L1*P1.w*M1
                               - L2*P2.w*M2
                               + L3*P3.w*M3 ;
                
                result = Sign(D.sign()*P1.w.sign()*P2.w.sign()*P3.w.sign());
            }
            
            if(result != ZERO) {
                return result;
            }

            stats.log_SOS();

            // Symbolic perturbation.
            //
            // We use the simple form of the predicate:
            // | x0 y0 (x0^2+y0^2+eps^i0) 1 |
            // | x1 y1 (x1^2+y1^2+eps^i1) 1 |
            // | x2 y2 (x2^2+y2^2+eps^i2) 1 |
            // | x3 y3 (x3^2+y3^2+eps^i3) 1 |
            // where i0,i1,i2,i3 denote the indices of the points (here, they
            // are local indices, coming from geometric sorting, lexico order)
            // Develop along the third row (keeping only the terms in epsilon):
            //          | x1 y1 1 |          | x0 y0 1 |          
            //   eps^i0 | x2 y2 1 | - eps^i1 | x2 y2 1 |
            //          | x3 y3 1 |          | x3 y3 1 |          
            //
            //          | x0 y0 1 |          | x0 y0 1 |
            // + eps^i2 | x1 y1 1 | - eps^i3 | x1 y1 1 |
            //          | x3 y3 1 |          | x2 y2 1 |
            return SOS(
                vec2HgLexicoCompare<expansion_nt>(),
                p0, SOS_result( det3_111_sign(p1,p2,p3)),
                p1, SOS_result(-det3_111_sign(p0,p2,p3)),
                p2, SOS_result( det3_111_sign(p0,p1,p3)),
                p3, SOS_result(-det3_111_sign(p0,p1,p2))
            );
        }

        coord_index_t triangle_normal_axis(
            const vec3& p1, const vec3& p2, const vec3& p3
        ) {
            static PredicateStats stats("triangle_normal_axis");
            stats.log_invoke();
            
            // Filter using interval arithmetics
            {
                interval_nt::Rounding rounding;                                
                vec3I p1I(p1);
                vec3I U = vec3I(p2) - p1I;
                vec3I V = vec3I(p3) - p1I;
                vec3I N = cross(U,V);
                interval_nt::Sign2 sx = N.x.sign();
                interval_nt::Sign2 sy = N.y.sign();
                interval_nt::Sign2 sz = N.z.sign();
                if(
                   !interval_nt::sign_is_determined(sx) ||
                   !interval_nt::sign_is_determined(sy) ||
                   !interval_nt::sign_is_determined(sz)
                ) {
                    goto exact; // Yes, goto, why not ?
                }

                if(interval_nt::convert_sign(sx) != POSITIVE) {
                    N.x.negate();
                }
                if(interval_nt::convert_sign(sy) != POSITIVE) {
                    N.y.negate();
                }
                if(interval_nt::convert_sign(sz) != POSITIVE) {
                    N.z.negate();
                }
                interval_nt::Sign2 sxy = (N.x - N.y).sign();
                interval_nt::Sign2 sxz = (N.x - N.z).sign();
                if(
                    !interval_nt::sign_is_determined(sxy) ||
                    !interval_nt::sign_is_determined(sxz) 
                ) {
                    goto exact; // Ahaha, another one !!!
                }
                if(
                    interval_nt::convert_sign(sxy) >= 0 &&
                    interval_nt::convert_sign(sxz) >= 0 
                ) {
                    return 0;
                }
                interval_nt::Sign2 syz = (N.y - N.z).sign();
                if(!interval_nt::sign_is_determined(syz)) {
                    goto exact; // The last one (for now !)
                }
                if(interval_nt::convert_sign(syz) >=0 ) {
                    return 1;
                }
                return 2;
            }

            // Exact computation, using low-level expansion API
            // (expansions allocated on the stack, better for
            // multithreading)
        exact:
            stats.log_exact();

            // These ones can be computed on the stack even
            // under MacOSX since they are at most of length 2
            
            const expansion& Ux = expansion_diff(p2.x, p1.x);
            const expansion& Uy = expansion_diff(p2.y, p1.y);
            const expansion& Uz = expansion_diff(p2.z, p1.z);

            const expansion& Vx = expansion_diff(p3.x, p1.x);
            const expansion& Vy = expansion_diff(p3.y, p1.y);
            const expansion& Vz = expansion_diff(p3.z, p1.z);

            expansion& Nx = expansion_det2x2(Uy,Vy,Uz,Vz);
            expansion& Ny = expansion_det2x2(Uz,Vz,Ux,Vx);
            expansion& Nz = expansion_det2x2(Ux,Vx,Uy,Vy);

            if(Nx.sign() != POSITIVE) {
                Nx.negate();
            }

            if(Ny.sign() != POSITIVE) {
                Ny.negate();
            }

            if(Nz.sign() != POSITIVE) {
                Nz.negate();
            }
            
            if(Nx.compare(Ny) >= 0 && Nx.compare(Nz) >= 0) {
                geo_debug_assert(Nx.sign() != ZERO);
                return 0;
            }
            if(Ny.compare(Nz) >= 0) {
                geo_debug_assert(Ny.sign() != ZERO);            
                return 1;
            }
            geo_assert(Nz.sign() != ZERO);        
            return 2;
        }

        bool aligned_3d(
            const vec3HE& p0, const vec3HE& p1, const vec3HE& p2
        ) {
            // TODO: filter if need be
            vec3HE U = p1-p0;
            vec3HE V = p2-p0;
            return (
                det2x2(U.x,V.x,U.y,V.y).sign() == ZERO &&
                det2x2(U.y,V.y,U.z,V.z).sign() == ZERO &&
                det2x2(U.z,V.z,U.x,V.x).sign() == ZERO 
            );
        }

        bool on_segment_3d(
            const vec3HE& p, const vec3HE& q1, const vec3HE& q2
        ) {
            // TODO: filter if need be
            vec3HE U = p-q1;
            vec3HE V = p-q2;
            if (
                det2x2(U.x,V.x,U.y,V.y).sign() != ZERO ||
                det2x2(U.y,V.y,U.z,V.z).sign() != ZERO ||
                det2x2(U.z,V.z,U.x,V.x).sign() != ZERO 
            ) {
                return false;
            }

            return (
                (
                    (U.x*V.x + U.y*V.y + U.z*V.z).sign() *
                    U.w.sign() * V.w.sign()
                ) <= ZERO
            );
        }
        

        vec3 approximate(const vec3HE& p) {
            // TODO: find a way of computing the round to nearest approxomation.
            // see division operation for expansions,
            // here:
            // https://www.jucs.org/jucs_5_6/division_of_floating_point/Daumas_M.pdf
            double w = p.w.estimate();
            return vec3(p.x.estimate()/w, p.y.estimate()/w, p.z.estimate()/w);
        }

        vec2 approximate(const vec2HE& p) {
            // TODO: find a way of computing the round to nearest approxomation.
            // see division operation for expansions,
            // here:
            // https://www.jucs.org/jucs_5_6/division_of_floating_point/Daumas_M.pdf
            double w = p.w.estimate();
            return vec2(p.x.estimate()/w, p.y.estimate()/w);
        }
        
    }

/*****************************************************************/

// Under Linux we got 10 Mb of stack (!) Then some operations can be
// made faster by using the low-level expansion API (that allocates
// intermediary multiprecision values on stack rather than in the heap).
// These optimized functions are written as template specializations
// (used automatically).    

#ifdef GEO_HAS_BIG_STACK
    
    template<> expansion_nt det(const vec2E& v1, const vec2E& v2) {
        expansion* result = expansion::new_expansion_on_heap(
            expansion::det2x2_capacity(
                v1.x.rep(), v1.y.rep(),
                v2.x.rep(), v2.y.rep()
            )
        );
        result->assign_det2x2(
            v1.x.rep(), v1.y.rep(),
            v2.x.rep(), v2.y.rep()
        );
        return expansion_nt(result);
    }

    template<> expansion_nt dot(const vec2E& v1, const vec2E& v2) {
        const expansion& m1 = expansion_product(v1.x.rep(), v2.x.rep());
        const expansion& m2 = expansion_product(v1.y.rep(), v2.y.rep());
        return expansion_nt(expansion_nt::SUM, m1, m2);
    }

    template<> expansion_nt dot(const vec3E& v1, const vec3E& v2) {
        const expansion& m1 = expansion_product(v1.x.rep(), v2.x.rep());
        const expansion& m2 = expansion_product(v1.y.rep(), v2.y.rep());
        const expansion& m3 = expansion_product(v1.z.rep(), v2.z.rep());
        return expansion_nt(expansion_nt::SUM,m1,m2,m3);
    }

    /*********************************************/
    
    template<> vec3Hg<expansion_nt> mix(
        const rationalg<expansion_nt>& t, const vec3& p1, const vec3& p2
    ) {
        expansion& st_d = const_cast<expansion&>(t.denom().rep());
        st_d.optimize();
        expansion& t_n  = const_cast<expansion&>(t.num().rep());
        t_n.optimize();
        const expansion& s_n  = expansion_diff(st_d, t_n);
        const expansion& sx   = expansion_product(s_n, p1.x);
        const expansion& tx   = expansion_product(t_n, p2.x);
        const expansion& sy   = expansion_product(s_n, p1.y);
        const expansion& ty   = expansion_product(t_n, p2.y);
        const expansion& sz   = expansion_product(s_n, p1.z);
        const expansion& tz   = expansion_product(t_n, p2.z);
        return vec3HE(
            expansion_nt(expansion_nt::SUM, sx,tx),
            expansion_nt(expansion_nt::SUM, sy,ty),
            expansion_nt(expansion_nt::SUM, sz,tz),
            expansion_nt(st_d)
        );
    }

    template<> vec2Hg<expansion_nt> mix(
        const rationalg<expansion_nt>& t, const vec2& p1, const vec2& p2
    ) {
        expansion& st_d = const_cast<expansion&>(t.denom().rep());
        st_d.optimize();
        expansion& t_n  = const_cast<expansion&>(t.num().rep());
        t_n.optimize();
        const expansion& s_n  = expansion_diff(st_d, t_n);
        const expansion& sx   = expansion_product(s_n, p1.x);
        const expansion& tx   = expansion_product(t_n, p2.x);
        const expansion& sy   = expansion_product(s_n, p1.y);
        const expansion& ty   = expansion_product(t_n, p2.y);
        return vec2HE(
            expansion_nt(expansion_nt::SUM, sx,tx),
            expansion_nt(expansion_nt::SUM, sy,ty),
            expansion_nt(st_d)
        );
    }

    /*********************************************/    

    template <> vec3E triangle_normal<vec3E>(
        const vec3& p1, const vec3& p2, const vec3& p3
    ) {
        const expansion& Ux = expansion_diff(p2.x,p1.x);
        const expansion& Uy = expansion_diff(p2.y,p1.y);
        const expansion& Uz = expansion_diff(p2.z,p1.z);
        const expansion& Vx = expansion_diff(p3.x,p1.x);
        const expansion& Vy = expansion_diff(p3.y,p1.y);
        const expansion& Vz = expansion_diff(p3.z,p1.z);
        expansion* Nx = expansion::new_expansion_on_heap(
            expansion::det2x2_capacity(Uy,Uz,Vy,Vz)
        );
        Nx->assign_det2x2(Uy,Uz,Vy,Vz);
        expansion* Ny = expansion::new_expansion_on_heap(
            expansion::det2x2_capacity(Uz,Ux,Vz,Vx)
        );
        Ny->assign_det2x2(Uz,Ux,Vz,Vx);
        expansion* Nz = expansion::new_expansion_on_heap(
            expansion::det2x2_capacity(Ux,Uy,Vx,Vy)
        );
        Nz->assign_det2x2(Ux,Uy,Vx,Vy);
        return vec3E(expansion_nt(Nx), expansion_nt(Ny), expansion_nt(Nz));
    }
    
#endif
    
}


