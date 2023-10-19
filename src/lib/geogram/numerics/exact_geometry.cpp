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
#include <geogram/basic/logger.h>

namespace {
    using namespace GEO;
    
    Sign orient_3d_filter(
        const vec3HE& p0, const vec3HE& p1,
        const vec3HE& p2, const vec3HE& p3
    ) {
        interval_nt::Rounding rounding;
            
        interval_nt w0(p0.w);
        interval_nt::Sign2 s0 = w0.sign();
        if(!interval_nt::sign_is_non_zero(s0)) {
            return ZERO;
        }

        interval_nt w1(p1.w);
        interval_nt::Sign2 s1 = w1.sign();
        if(!interval_nt::sign_is_non_zero(s1)) {
            return ZERO;
        }

        interval_nt w2(p2.w);
        interval_nt::Sign2 s2 = w2.sign();
        if(!interval_nt::sign_is_non_zero(s2)) {
            return ZERO;
        }

        interval_nt w3(p3.w);
        interval_nt::Sign2 s3 = w3.sign();
        if(!interval_nt::sign_is_non_zero(s3)) {
            return ZERO;
        }
            
        interval_nt x0(p0.x);
        interval_nt y0(p0.y);
        interval_nt z0(p0.z);

        interval_nt x1(p1.x);
        interval_nt y1(p1.y);
        interval_nt z1(p1.z);

        interval_nt x2(p2.x);
        interval_nt y2(p2.y);
        interval_nt z2(p2.z);

        interval_nt x3(p3.x);
        interval_nt y3(p3.y);
        interval_nt z3(p3.z);

        interval_nt Ux = det2x2(x1,w1,x0,w0);
        interval_nt Uy = det2x2(y1,w1,y0,w0);
        interval_nt Uz = det2x2(z1,w1,z0,w0);

        interval_nt Vx = det2x2(x2,w2,x0,w0);
        interval_nt Vy = det2x2(y2,w2,y0,w0);
        interval_nt Vz = det2x2(z2,w2,z0,w0);

        interval_nt Wx = det2x2(x3,w3,x0,w0);
        interval_nt Wy = det2x2(y3,w3,y0,w0);
        interval_nt Wz = det2x2(z3,w3,z0,w0);

        interval_nt Delta = det3x3(
            Ux, Uy, Uz,
            Vx, Vy, Vz,
            Wx, Wy, Wz                
        );

        interval_nt::Sign2 s = Delta.sign();
        if(!interval_nt::sign_is_non_zero(s)) {
            return ZERO;
        }

        return Sign(
            interval_nt::convert_sign(s)  *
            interval_nt::convert_sign(s0) *
            interval_nt::convert_sign(s1) *
            interval_nt::convert_sign(s2) *
            interval_nt::convert_sign(s3)                 
        );
    }
}

namespace GEO {

    vec3HE mix(const rational_nt& t, const vec3& p1, const vec3& p2) {
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

    vec2HE mix(const rational_nt& t, const vec2& p1, const vec2& p2) {
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
    
    vec2HE mix(const rational_nt& t, const vec2HE& p1, const vec2HE& p2) {
        expansion& st_d = expansion_product(t.denom().rep(),p1.w.rep());
        st_d.optimize();
        expansion& t_n  = const_cast<expansion&>(t.num().rep());
        t_n.optimize();
        const expansion& s_n  = expansion_diff(st_d, t_n);
        if(p1.w == p2.w) {
            const expansion& sx   = expansion_product(s_n, p1.x.rep());
            const expansion& tx   = expansion_product(t_n, p2.x.rep());
            const expansion& sy   = expansion_product(s_n, p1.y.rep());
            const expansion& ty   = expansion_product(t_n, p2.y.rep());
            return vec2HE(
                expansion_nt(expansion_nt::SUM, sx,tx),
                expansion_nt(expansion_nt::SUM, sy,ty),
                expansion_nt(st_d)
            );
        }
        expansion& st_d_2 = expansion_product(st_d, p2.w.rep());
        st_d_2.optimize();
        const expansion& t_n_2  = expansion_product(t_n, p1.w.rep());
        const expansion& s_n_2  = expansion_product(s_n, p2.w.rep());
        const expansion& sx   = expansion_product(s_n_2, p1.x.rep());
        const expansion& tx   = expansion_product(t_n_2, p2.x.rep());
        const expansion& sy   = expansion_product(s_n_2, p1.y.rep());
        const expansion& ty   = expansion_product(t_n_2, p2.y.rep());
        return vec2HE(
            expansion_nt(expansion_nt::SUM, sx,tx),
            expansion_nt(expansion_nt::SUM, sy,ty),
            expansion_nt(st_d_2)
        );
    }

    vec3HE mix(const rational_nt& t, const vec3HE& p1, const vec3HE& p2) {
        expansion& st_d = expansion_product(t.denom().rep(),p1.w.rep());
        st_d.optimize();
        expansion& t_n  = const_cast<expansion&>(t.num().rep());
        t_n.optimize();
        const expansion& s_n  = expansion_diff(st_d, t_n);
        if(p1.w == p2.w) {
            const expansion& sx   = expansion_product(s_n, p1.x.rep());
            const expansion& tx   = expansion_product(t_n, p2.x.rep());
            const expansion& sy   = expansion_product(s_n, p1.y.rep());
            const expansion& ty   = expansion_product(t_n, p2.y.rep());
            const expansion& sz   = expansion_product(s_n, p1.z.rep());
            const expansion& tz   = expansion_product(t_n, p2.z.rep());
            return vec3HE(
                expansion_nt(expansion_nt::SUM, sx,tx),
                expansion_nt(expansion_nt::SUM, sy,ty),
                expansion_nt(expansion_nt::SUM, sz,tz),
                expansion_nt(st_d)
            );
        }
        expansion& st_d_2 = expansion_product(st_d, p2.w.rep());
        st_d_2.optimize();
        const expansion& t_n_2  = expansion_product(t_n, p1.w.rep());
        const expansion& s_n_2  = expansion_product(s_n, p2.w.rep());
        const expansion& sx   = expansion_product(s_n_2, p1.x.rep());
        const expansion& tx   = expansion_product(t_n_2, p2.x.rep());
        const expansion& sy   = expansion_product(s_n_2, p1.y.rep());
        const expansion& ty   = expansion_product(t_n_2, p2.y.rep());
        const expansion& sz   = expansion_product(s_n_2, p1.z.rep());
        const expansion& tz   = expansion_product(t_n_2, p2.z.rep());
        return vec3HE(
            expansion_nt(expansion_nt::SUM, sx,tx),
            expansion_nt(expansion_nt::SUM, sy,ty),
            expansion_nt(expansion_nt::SUM, sz,tz),
            expansion_nt(st_d_2)
        );
    }

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
    
    namespace PCK {

        Sign orient_2d(
            const vec2HE& p0, const vec2HE& p1, const vec2HE& p2
        ) {
            const expansion& Delta = expansion_det3x3(
                p0.x.rep(), p0.y.rep(), p0.w.rep(),
                p1.x.rep(), p1.y.rep(), p1.w.rep(),
                p2.x.rep(), p2.y.rep(), p2.w.rep()
            );
            return Sign(
                Delta.sign()*
                p0.w.rep().sign()*
                p1.w.rep().sign()*
                p2.w.rep().sign()
            );
        }



        PCK_STAT(Numeric::uint64 orient3dHE_calls = 0;)
        PCK_STAT(Numeric::uint64 orient3dHE_filter_success = 0;)
        
        Sign orient_3d(
            const vec3HE& p0, const vec3HE& p1,
            const vec3HE& p2, const vec3HE& p3
        ) {
            PCK_STAT(++orient3dHE_calls;)

            // Filter
            { 
                Sign filter_result = orient_3d_filter(p0,p1,p2,p3);
                if(filter_result != ZERO) {
                    PCK_STAT(++orient3dHE_filter_success;)
                    return filter_result;
                }
            }
            
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


        PCK_STAT(Numeric::uint64 proj_orient2d_calls = 0;)
        PCK_STAT(Numeric::uint64 proj_orient2d_filter_success = 0;)

        
        Sign orient_2d_projected(
            const vec3HE& p0, const vec3HE& p1, const vec3HE& p2,
            coord_index_t axis
        ) {
            coord_index_t u = coord_index_t((axis+1)%3);
            coord_index_t v = coord_index_t((axis+2)%3);


            PCK_STAT(++proj_orient2d_calls;)

            // Filter, using interval arithmetics
            { 
                interval_nt::Rounding rounding;
                
                interval_nt a13(p0.w);
                interval_nt a23(p1.w);
                interval_nt a33(p2.w);
                interval_nt::Sign2 s13 = a13.sign();
                interval_nt::Sign2 s23 = a23.sign();
                interval_nt::Sign2 s33 = a33.sign();                
                if(
                    interval_nt::sign_is_determined(s13) &&
                    interval_nt::sign_is_determined(s23) &&
                    interval_nt::sign_is_determined(s33)
                ) {
                    interval_nt a11(p0[u]);
                    interval_nt a12(p0[v]);
                    interval_nt a21(p1[u]);
                    interval_nt a22(p1[v]);
                    interval_nt a31(p2[u]);
                    interval_nt a32(p2[v]);
                    interval_nt DeltaI= det3x3(
                        a11,a12,a13,
                        a21,a22,a23,
                        a31,a32,a33
                    );
                    interval_nt::Sign2 sDeltaI = DeltaI.sign();
                    if(interval_nt::sign_is_determined(sDeltaI)) {
                        PCK_STAT(++proj_orient2d_filter_success;)
                        return Sign(
                            interval_nt::convert_sign(sDeltaI)*
                            interval_nt::convert_sign(s13)*
                            interval_nt::convert_sign(s23)*
                            interval_nt::convert_sign(s33)
                        );
                    }
                }
            }

            Sign result = ZERO;
            {
                const expansion& Delta = expansion_det3x3(
                    p0[u].rep(), p0[v].rep(), p0.w.rep(),
                    p1[u].rep(), p1[v].rep(), p1.w.rep(),
                    p2[u].rep(), p2[v].rep(), p2.w.rep()
                );
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
            vec2HE U = p1 - p0;
            vec2HE V = p2 - p0;
            const expansion& x1x2 = expansion_product(U.x.rep(), V.x.rep());
            const expansion& y1y2 = expansion_product(U.y.rep(), V.y.rep());
            const expansion& S = expansion_sum(x1x2, y1y2);
            return Sign(S.sign()*U.w.sign()*V.w.sign());
        }

/******************************************************************************/

        void orient_2d_projected_stats() {
#ifdef PCK_STATS
            Logger::out("PCK") << "Plain orient2d:" << std::endl;            
            Logger::out("PCK")
                << proj_orient2d_calls << " proj orient2d calls" << std::endl;
            Logger::out("PCK")
                << proj_orient2d_filter_success
                << " proj orient2d filter success" << std::endl;
            Logger::out("PCK")
                << 100.0 * double(proj_orient2d_filter_success) /
                           double(proj_orient2d_calls)
                << "% filter success"  << std::endl;
            Logger::out("PCK") << "orient3dHE:" << std::endl;
            Logger::out("PCK") << orient3dHE_calls
                               << " orient3dHE calls" << std::endl;
            Logger::out("PCK") << orient3dHE_filter_success
                               << " orient3dHE filter success" << std::endl;
            Logger::out("PCK") << 100.0 *
                                  double(orient3dHE_filter_success) /
                                  double(orient3dHE_calls)
                               << "% filter success" << std::endl;
#endif    
        }

/*****************************************************************************/

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

        Sign incircle_2d_SOS(
            const vec2HE& p0, const vec2HE& p1,
            const vec2HE& p2, const vec2HE& p3
        ) {
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
            
            {
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
                
                double l0 = (
                    geo_sqr(p0.x) + geo_sqr(p0.y)
                ).estimate() / geo_sqr(p0.w).estimate();

                double l1 = (
                    geo_sqr(p1.x) + geo_sqr(p1.y)
                ).estimate() / geo_sqr(p1.w).estimate();

                double l2 = (
                    geo_sqr(p2.x) + geo_sqr(p2.y)
                ).estimate() / geo_sqr(p2.w).estimate();
                
                double l3 = (
                    geo_sqr(p3.x) + geo_sqr(p3.y)
                ).estimate() / geo_sqr(p3.w).estimate();

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
            
            
            const vec2HE* p_sort[4] = {
                &p0, &p1, &p2, &p3
            };
            std::sort(
                p_sort, p_sort+4,
                [](const vec2HE* A, const vec2HE* B)->bool{
                    vec2HgLexicoCompare<expansion_nt> cmp;
                    return cmp(*A,*B);
                }
            );
            for(index_t i = 0; i < 4; ++i) {
                if(p_sort[i] == &p0) {
                    result = det3_111_sign(p1,p2,p3);
                    if(result != ZERO) {
                        return result;
                    }
                }
                if(p_sort[i] == &p1) {
                    result = Sign(-det3_111_sign(p0,p2,p3));
                    if(result != ZERO) {
                        return result;
                    }
                }
                if(p_sort[i] == &p2) {
                    result = det3_111_sign(p0,p1,p3);
                    if(result != ZERO) {
                        return result;
                    }
                }
                if(p_sort[i] == &p3) {
                    result = Sign(-det3_111_sign(p0,p1,p2));
                    if(result != ZERO) {
                        return result;
                    }
                }
            }
            geo_assert_not_reached;
            return result;
        }

        Sign incircle_2d_SOS_projected(
            const vec3HE& pp0, const vec3HE& pp1,
            const vec3HE& pp2, const vec3HE& pp3,
            coord_index_t axis
        ) {
            
            coord_index_t u = coord_index_t((axis+1)%3);
            coord_index_t v = coord_index_t((axis+2)%3);
            
            vec2HE p0(pp0[u], pp0[v], pp0.w);
            vec2HE p1(pp1[u], pp1[v], pp1.w);
            vec2HE p2(pp2[u], pp2[v], pp2.w);
            vec2HE p3(pp3[u], pp3[v], pp3.w);
            
            Sign result = incircle_2d_SOS(p0, p1, p2, p3);
            
            return result;
        }
        
/*****************************************************************************/        
        
    }

    bool get_three_planes_intersection(
        vec3HE& result,
        const vec3& p1, const vec3& p2, const vec3& p3,
        const vec3& q1, const vec3& q2, const vec3& q3,
        const vec3& r1, const vec3& r2, const vec3& r3
    ) {

        vec3E N1 = triangle_normal<vec3E>(p1,p2,p3);
        vec3E N2 = triangle_normal<vec3E>(q1,q2,q3);
        vec3E N3 = triangle_normal<vec3E>(r1,r2,r3);

        vec3E B(
            dot(N1,make_vec3<vec3E>(p1)),
            dot(N2,make_vec3<vec3E>(q1)),
            dot(N3,make_vec3<vec3E>(r1))
        );
        
        result.w = det3x3(
            N1.x, N1.y, N1.z,
            N2.x, N2.y, N2.z,
            N3.x, N3.y, N3.z
        );

        if(result.w.sign() == ZERO) {
            return false;
        }
        
        result.x = det3x3(
            B.x, N1.y, N1.z,
            B.y, N2.y, N2.z,
            B.z, N3.y, N3.z
        );

        result.y = det3x3(
            N1.x, B.x, N1.z,
            N2.x, B.y, N2.z,
            N3.x, B.z, N3.z
        );

        result.z = det3x3(
            N1.x, N1.y, B.x,
            N2.x, N2.y, B.y,
            N3.x, N3.y, B.z
        );

        return true;
    }

    /**********************************************************/
    
    template<> vec3HE plane_line_intersection<vec3HE>(
        const vec3& p1, const vec3& p2, const vec3& p3,
        const vec3& q1, const vec3& q2
    ) {
        // Moller & Trumbore's algorithm
        // see: https://stackoverflow.com/questions/42740765/
        //  intersection-between-line-and-triangle-in-3d
        vec3E D   = make_vec3<vec3E>(q1,q2);
        vec3E E1  = make_vec3<vec3E>(p1,p2);
        vec3E E2  = make_vec3<vec3E>(p1,p3);
        vec3E AO  = make_vec3<vec3E>(p1,q1);
        vec3E N   = cross(E1,E2);
        expansion_nt d = -dot(D,N);
        geo_debug_assert(d.sign() != ZERO);
        rational_nt t(dot(AO,N),d);
        return mix(t,q1,q2);
    }
    
    /**********************************************************/

    coord_index_t triangle_normal_axis_exact(
        const vec3& p1, const vec3& p2, const vec3& p3
    ) {
        const expansion& Ux = expansion_diff(p2.x,p1.x);
        const expansion& Uy = expansion_diff(p2.y,p1.y);
        const expansion& Uz = expansion_diff(p2.z,p1.z);
        const expansion& Vx = expansion_diff(p3.x,p1.x);
        const expansion& Vy = expansion_diff(p3.y,p1.y);
        const expansion& Vz = expansion_diff(p3.z,p1.z);
        
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
            return 0;
        }

        if(Ny.compare(Nz) >= 0) {
            return 1;
        }

        return 2;
    }
}


