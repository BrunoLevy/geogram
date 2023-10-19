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

#ifndef GEOGRAM_NUMERICS_EXACT_GEOMETRY
#define GEOGRAM_NUMERICS_EXACT_GEOMETRY

#include <geogram/basic/common.h>
#include <geogram/basic/geometry.h>
#include <geogram/basic/vechg.h>
#include <geogram/numerics/expansion_nt.h>
#include <geogram/numerics/interval_nt.h>

/**
 * \file geogram/numerics/exact_geometry.h
 * \brief Exact predicates and constructs
 * \details Implements vector types with expansion
 *  coordinates (vec2E, vec3E), vector types with
 *  homogeneous expansion coordinates (vec2HE, vec3HE),
 *  2d orientation predicate, incircle predicate
 *  and constructions for intersections.
 */

namespace GEO {

    /**
     * \brief vec2 with coordinates as expansions
     * \details Coordinates support +,-,*
     */
    typedef vecng<2,expansion_nt> vec2E;

    /**
     * \brief vec3 with coordinates as expansions
     * \details Coordinates support +,-,*
     */
    typedef vecng<3,expansion_nt> vec3E;    

    /**
     * \brief vec2 with coordinates as interval_nt
     * \details Used to write arithmetic filters
     *  for geometric predicates.
     */
    typedef vecng<2,interval_nt> vec2I;    
    
    /**
     * \brief vec3 with coordinates as interval_nt
     * \details Used to write arithmetic filters
     *  for geometric predicates.
     */
    typedef vecng<3,interval_nt> vec3I;    

    /**
     * \brief 2D vector in homogeneous coordinates
     *  with coordinates as expansions
     * \details Coordinates support +,-,* and / by
     *  multiplying w.
     */
    typedef vec2Hg<expansion_nt> vec2HE;

    /**
     * \brief 3D vector in homogeneous coordinates
     *  with coordinates as expansions
     * \details Coordinates support +,-,* and / by
     *  multiplying w.
     */
    typedef vec3Hg<expansion_nt> vec3HE;

    /**
     * \brief 2D vector in homogeneous coordinates
     *  with coordinates as intervals.
     * \details Used to write arithmetic filters
     *  for geometric predicates.
     */
    typedef vec2Hg<interval_nt> vec2HI;

    /**
     * \brief 3D vector in homogeneous coordinates
     *  with coordinates as intervals.
     * \details Used to write arithmetic filters
     *  for geometric predicates.
     */
    typedef vec2Hg<interval_nt> vec3HI;

    /***********************************************************************/

    /**
     * \brief Specialization optimized using low-level API
     */
    template<> vec2Hg<expansion_nt> GEOGRAM_API mix(
        const rationalg<expansion_nt>& t,
        const vecng<2,double>& p1, const vecng<2,double>& p2
    );

    /**
     * \brief Specialization optimized using low-level API
     */
    template<> vec3Hg<expansion_nt> GEOGRAM_API mix(
        const rationalg<expansion_nt>& t,
        const vecng<3,double>& p1, const vecng<3,double>& p2
    );

    /**
     * \brief Specialization optimized using low-level API
     */
    template<> vec2Hg<expansion_nt> GEOGRAM_API mix(
        const rationalg<expansion_nt>& t,
        const vec2Hg<expansion_nt>& p1, const vec2Hg<expansion_nt>& p2
    );

    /**
     * \brief Specialization optimized using low-level API
     */
    template<> vec3Hg<expansion_nt> GEOGRAM_API mix(
        const rationalg<expansion_nt>& t,
        const vec3Hg<expansion_nt>& p1, const vec3Hg<expansion_nt>& p2
    );

    /***********************************************************************/
    
    /**
     * \brief Specialization optimized using low-level API
     */
    template<> expansion_nt GEOGRAM_API det(const vec2E& v1, const vec2E& v2);

    /**
     * \brief Specialization optimized using low-level API
     */
    template<> expansion_nt GEOGRAM_API dot(const vec2E& v1, const vec2E& v2);

    /**
     * \brief Specialization optimized using low-level API
     */
    template<> expansion_nt GEOGRAM_API dot(const vec3E& v1, const vec3E& v2);
    
    namespace PCK {

        Sign GEOGRAM_API orient_2d(
            const vec2HE& p0, const vec2HE& p1, const vec2HE& p2
        );

        Sign GEOGRAM_API orient_2d_projected(
            const vec3HE& p0, const vec3HE& p1, const vec3HE& p2,
            coord_index_t axis
        );

        void GEOGRAM_API orient_2d_projected_stats(); 
        
        // TODO: check, it seems that orient_3d(vec3,...) and
        // orient_3d(vec3HE,...) have opposite orientations !
        Sign GEOGRAM_API orient_3d(
            const vec3HE& p0, const vec3HE& p1,
            const vec3HE& p2, const vec3HE& p3
        );

        Sign GEOGRAM_API dot_2d(
            const vec2HE& p0, const vec2HE& p1, const vec2HE& p2
        );

        /**
         * \brief Tests whether a point is in the circumscribed circle of 
         *  three other points.
         * \details If the triangle \p p0 , \p p1 , \p p2 is oriented 
         *  clockwise instead of counter-clockwise, then the result is inversed.
         * \param[in] p0 , p1 , p2 , p3 the four points, 
         *  in homogeneous coordinates,represented in exact form. 
         * \retval POSITIVE if p3 is inside 
         *   the circumscribed circle of p0, p1, p2
         * \retval NEGATIVE if p3 is outside 
         *  the circumscribed circle of p0, p1, p2
         * \retval a coherent perturbation otherwise
         */
        Sign GEOGRAM_API incircle_2d_SOS(
            const vec2HE& p0, const vec2HE& p1,
            const vec2HE& p2, const vec2HE& p3
        );
        
        /**
         * \brief Tests whether a point is in the circumscribed circle 
         *  of three other points, by projection.
         * \details If the (projected) triangle \p p0 , \p p1 , \p p2 
         *  is oriented clockwise instead of counter-clockwise, 
         *  then the result is inversed.
         * \param[in] p0 , p1 , p2 , p3 the four points, 
         *  in homogeneous coordinates, represented in exact form.
         * \param[in] axis the axis of projection
         * \retval POSITIVE if p3 is inside 
         *  the circumscribed circle of p0, p1, p2
         * \retval NEGATIVE if p3 is outside 
         *  the circumscribed circle of p0, p1, p2
         * \retval a coherent perturbation otherwise
         */
        Sign GEOGRAM_API incircle_2d_SOS_projected(
            const vec3HE& p0, const vec3HE& p1,
            const vec3HE& p2, const vec3HE& p3,
            coord_index_t axis
        );
    }

    /**
     * \brief Converts a 3d vector with double coordinates
     *  into a 3d vector with coordinates of arbitrary type
     * \param[in] p the vector to be converted
     * \return the converted vector
     * \tparam VEC3 the type of the returned vector
     */
    template <class VEC3>
    inline VEC3 make_vec3(const vec3& p) {
        typedef typename VEC3::value_type value_type;
        return VEC3(value_type(p.x),value_type(p.y),value_type(p.z));
    }

    /**
     * \brief Creates a vector with coordinates of arbitrary type
     *  from two points with double coordinates
     * \param[in] p1 , p2 the two vectors
     * \return The vector \p p2 - \p p1 
     * \tparam VEC3 the type of the returned vector
     */
    template <class VEC3 = vec3>
    inline VEC3 make_vec3(const vec3& p1, const vec3& p2) {
        typedef typename VEC3::value_type value_type;
        return VEC3(
            value_type(p2.x) - value_type(p1.x),
            value_type(p2.y) - value_type(p1.y),
            value_type(p2.z) - value_type(p1.z)
        );
    }

    /**
     * \brief Specialization for vec3E
     */
    template <>
    inline vec3E make_vec3<vec3E>(const vec3& p1, const vec3& p2) {
        return vec3E(
            expansion_nt(expansion_nt::DIFF, p2.x, p1.x),
            expansion_nt(expansion_nt::DIFF, p2.y, p1.y),
            expansion_nt(expansion_nt::DIFF, p2.z, p1.z)            
        );
    }
    
    /**
     * \brief Creates a vector with coordinates of arbitrary type
     *  from two points with double coordinates
     * \param[in] p1 , p2 the two vectors
     * \return The vector \p p2 - \p p1 
     * \tparam VEC2 the type of the returned vector
     */
    template <class VEC2>
    inline VEC2 make_vec2(
        const vec2& p1, const vec2& p2
    ) {
        typedef typename VEC2::value_type value_type;        
        return VEC2(
            value_type(p2.x) - value_type(p1.x),
            value_type(p2.y) - value_type(p1.y)
        );
    }

    /**
     * \brief Specialization for vec2E
     */
    template <>
    inline vec2E make_vec2<vec2E>(const vec2& p1, const vec2& p2) {
        return vec2E(
            expansion_nt(expansion_nt::DIFF, p2.x, p1.x),
            expansion_nt(expansion_nt::DIFF, p2.y, p1.y)
        );
    }
    
    /**
     * \brief Computes the normal to a triangle from its three
     *  vertices 
     * \param[in] p1 , p2 , p3 the three vertices of the triangle
     * \return the normal to the triangle with coordinates of 
     *  arbitrary type
     * \tparam VEC3 the type of the returned vector
     */
    template <class VEC3>
    inline VEC3 triangle_normal(
        const vec3& p1, const vec3& p2, const vec3& p3
    ) {
        return cross(
            make_vec3<VEC3>(p1,p2),
            make_vec3<VEC3>(p1,p3)
        );
    }

    /**
     * \brief Specialization for vec3E
     */
    template <> GEOGRAM_API vec3E triangle_normal<vec3E>(
        const vec3& p1, const vec3& p2, const vec3& p3
    );

    /**
     * \brief Computes the exact intersection between the support
     *  planes of three triangles
     * \param[in] p1 , p2 , p3 the three vertices of the first triangle
     * \param[in] q1 , q2 , q3 the three vertices of the second triangle
     * \param[in] r1 , r2 , r3 the three vertices of the third triangle
     * \param[out] result the exact intersection between the three planes
     *  if it exsists
     * \retval true if the planes have an intersection
     * \retval false otherwise
     */
    bool GEOGRAM_API get_three_planes_intersection(
        vec3HE& result,
        const vec3& p1, const vec3& p2, const vec3& p3,
        const vec3& q1, const vec3& q2, const vec3& q3,
        const vec3& r1, const vec3& r2, const vec3& r3
    );

    template <class VEC3> inline VEC3 plane_line_intersection(
        const vec3& p1, const vec3& p2, const vec3& p3,
        const vec3& q1, const vec3& q2
    ) {
        geo_argused(p1);
        geo_argused(p2);
        geo_argused(p3);
        geo_argused(q1);
        geo_argused(q2);        
        geo_assert_not_reached;
        return VEC3();
    }
    
    /**
     * \brief Computes the exact intersection between the support plane
     *  of a triangle and the support line of a segment
     * \pre The intersection exists
     * \param[in] p1 , p2 , p3 the three vertices of the triangle
     * \param[in] q1 , q2 the two vertices of the segment
     * \return the exact intersection between the plane and the line
     */
    template<>
    vec3HE GEOGRAM_API plane_line_intersection<vec3HE>(
        const vec3& p1, const vec3& p2, const vec3& p3,
        const vec3& q1, const vec3& q2
    );

    /************************************************************************/

    /**
     * \brief Finds an axis along which a triangle can be projected without
     *  degeneracy
     * \details All computations are done with exact arithmetics
     * \param[in] p1 , p2 , p3 the three vertices of the triangle
     * \return the coordinate of largest magnitude of the normal vector 
     *  (one of 0,1,2)
     */
    coord_index_t GEOGRAM_API triangle_normal_axis_exact(
        const vec3& p1, const vec3& p2, const vec3& p3
    );

    /************************************************************************/
}

#endif
