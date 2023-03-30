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
#include <geogram/numerics/expansion_nt.h>

/**
 * \file geogram/numerics/exact_geometry.h
 * \brief Exact predicates and constructs
 * \details Implements vector types with expansion
 *  coordinates (vec2E, vec3E), vector types with
 *  homogeneous expansion coordinates (vec2HE, vec3HE),
 *  2d orientation predicate, 3d-lifted orientation
 *  predicate (can be used to implement incircle), and
 *  constructions for intersections.
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
     * \brief 2D vector in homogeneous coordinates
     *  with coordinates as arithmetic expansions
     * \details Coordinates support +,-,* and / by
     *  multiplying w.
     */
    struct vec2HE {

        /**
         * \brief Creates an uninitialized vec2HE
         */
        vec2HE() :
            x(expansion_nt::UNINITIALIZED),
            y(expansion_nt::UNINITIALIZED),
            w(expansion_nt::UNINITIALIZED)            
        {
        }

        vec2HE(
            const expansion_nt& x_in,
            const expansion_nt& y_in,
            const expansion_nt& w_in
        ) : x(x_in), y(y_in), w(w_in) {
        }

        vec2HE(
            expansion_nt&& x_in,
            expansion_nt&& y_in,
            expansion_nt&& w_in
        ) : x(x_in), y(y_in), w(w_in) {
        }
        
        vec2HE(const vec2HE& rhs) :
            x(rhs.x), y(rhs.y), w(rhs.w) {
        }

        vec2HE(vec2HE&& rhs) :
            x(rhs.x), y(rhs.y), w(rhs.w) {
        }

        explicit vec2HE(const vec2& rhs) : 
            x(rhs.x), y(rhs.y), w(1.0) {
        }
        
        vec2HE& operator=(const vec2HE& rhs) {
            if(&rhs != this) {
                x=rhs.x;
                y=rhs.y;
                w=rhs.w;
            }
            return *this;
        }

        vec2HE& operator=(vec2HE&& rhs) {
            if(&rhs != this) {
                x=rhs.x;
                y=rhs.y;
                w=rhs.w;
            }
            return *this;
        }

        expansion_nt* data() {
            return &x;
        }

        const expansion_nt* data() const {
            return &x;
        }
        
        expansion_nt& operator[](coord_index_t i) {
            geo_debug_assert(i < 2);
            return data()[i];
        }

        const expansion_nt& operator[](coord_index_t i) const {
            geo_debug_assert(i < 2);
            return data()[i];
        }

        /**
         * \brief Optimizes the internal storage of the 
         *  expansions used to store the coordinates.
         */
        void optimize() {
            x.optimize();
            y.optimize();
            w.optimize();
        }
        
        expansion_nt x;
        expansion_nt y;
        expansion_nt w;
    };

    /**
     * \brief 3D vector in homogeneous coordinates
     *  with coordinates as arithmetic expansions.
     * \details Coordinates support +,-,* and / by
     *  multiplying w.
     */
    struct vec3HE {

        /**
         * \brief Creates an uninitialized vec3HE
         */
        vec3HE() :
            x(expansion_nt::UNINITIALIZED),
            y(expansion_nt::UNINITIALIZED),
            z(expansion_nt::UNINITIALIZED),
            w(expansion_nt::UNINITIALIZED)            
        {
        }

        vec3HE(
            const expansion_nt& x_in,
            const expansion_nt& y_in,
            const expansion_nt& z_in,
            const expansion_nt& w_in            
        ) : x(x_in), y(y_in), z(z_in), w(w_in) {
        }

        vec3HE(
            expansion_nt&& x_in,
            expansion_nt&& y_in,
            expansion_nt&& z_in,
            expansion_nt&& w_in
        ) : x(x_in), y(y_in), z(z_in), w(w_in) {
        }

        vec3HE(
            double x_in,
            double y_in,
            double z_in,
            double w_in
        ) : x(x_in), y(y_in), z(z_in), w(w_in) {
        }
        
        vec3HE(const vec3HE& rhs) :
            x(rhs.x), y(rhs.y), z(rhs.z), w(rhs.w) {
        }

        vec3HE(vec3HE&& rhs) :
            x(rhs.x), y(rhs.y), z(rhs.z), w(rhs.w) {
        }

        explicit vec3HE(const vec3& rhs) : 
            x(rhs.x), y(rhs.y), z(rhs.z), w(1.0) {
        }
        
        vec3HE& operator=(const vec3HE& rhs) {
            if(&rhs != this) {
                x=rhs.x;
                y=rhs.y;
                z=rhs.z;                
                w=rhs.w;
            }
            return *this;
        }

        vec3HE& operator=(vec3HE&& rhs) {
            if(&rhs != this) {
                x=rhs.x;
                y=rhs.y;
                z=rhs.z;                
                w=rhs.w;
            }
            return *this;
        }

        expansion_nt* data() {
            return &x;
        }

        const expansion_nt* data() const {
            return &x;
        }
        
        expansion_nt& operator[](coord_index_t i) {
            geo_debug_assert(i < 3);
            return data()[i];
        }

        const expansion_nt& operator[](coord_index_t i) const {
            geo_debug_assert(i < 3);
            return data()[i];
        }

        /**
         * \brief Optimizes the internal storage of the 
         *  expansions used to store the coordinates.
         */
        void optimize() {
            x.optimize();
            y.optimize();
            z.optimize();            
            w.optimize();
        }
        
        expansion_nt x;
        expansion_nt y;
        expansion_nt z;        
        expansion_nt w;
    };

    vec2HE GEOGRAM_API operator-(const vec2HE& p1, const vec2HE& p2);
    
    vec3HE GEOGRAM_API operator-(const vec3HE& p1, const vec3HE& p2);

    /**
     * \brief Comparator class for vec3HE
     * \detail Used to create maps indexed by vec3HE
     */
    class GEOGRAM_API vec2HELexicoCompare {
    public:
       /**
        * \brief Compares two vec3HE
        * \retval true if \p v1 is before \p v2 in the lexicographic
        *  order
        * \retval false otherwise
        */
       bool operator()(const vec2HE& v1, const vec2HE& v2) const; 
    };

    
    /**
     * \brief Comparator class for vec3HE
     * \detail Used to create maps indexed by vec3HE
     */
    class GEOGRAM_API vec3HELexicoCompare {
    public:
       /**
        * \brief Compares two vec3HE
        * \retval true if \p v1 is before \p v2 in the lexicographic
        *  order
        * \retval false otherwise
        */
       bool operator()(const vec3HE& v1, const vec3HE& v2) const; 
    };

   /**
     * \brief Comparator class for projected vec3HE
     */
    class GEOGRAM_API vec3HEProjectedLexicoCompare {
    public:
       vec3HEProjectedLexicoCompare(coord_index_t coord) {
           u = coord_index_t((coord+1)%3);
           v = coord_index_t((coord+2)%3);
       }
       /**
        * \brief Compares two vec3HE
        * \retval true if \p v1 is before \p v2 in the lexicographic
        *  order
        * \retval false otherwise
        */
       bool operator()(const vec3HE& v1, const vec3HE& v2) const;
       coord_index_t u;
       coord_index_t v;
    };
    
    vec3HE GEOGRAM_API mix(
        const rational_nt& t, const vec3& p1, const vec3& p2
    );

    vec2HE GEOGRAM_API mix(
        const rational_nt& t, const vec2& p1, const vec2& p2
    );

    vec2HE GEOGRAM_API mix(
        const rational_nt& t, const vec2HE& p1, const vec2HE& p2
    );

    vec3HE GEOGRAM_API mix(
        const rational_nt& t, const vec3HE& p1, const vec3HE& p2
    );

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

        template <class T> inline bool same_point(
            const vecng<3,T>& v1, const vecng<3,T>& v2
        ) {
            // operator== is well optimized for expansion_nt and rational_nt
            return (v1.x == v2.x) && (v1.y == v2.y) && (v2.z == v1.z);
        }

        template <class T> inline bool same_point(
            const vecng<2,T>& v1, const vecng<2,T>& v2
        ) {
            // operator== is well optimized for expansion_nt and rational_nt
            return (v1.x == v2.x) && (v1.y == v2.y);
        }

        bool GEOGRAM_API same_point(const vec2HE& v1, const vec2HE& v2);
        bool GEOGRAM_API same_point(const vec3HE& v1, const vec3HE& v2);  
        
        Sign GEOGRAM_API orient_2d(
            const vec2HE& p0, const vec2HE& p1, const vec2HE& p2
        );

        Sign GEOGRAM_API orient_2d_projected(
            const vec3HE& p0, const vec3HE& p1, const vec3HE& p2,
            coord_index_t axis
        );

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
         * \brief Computes the 3d orientation test with lifted points.
         * \details Given three lifted points p0', p1', p2' in 
         *  R^2, tests if the lifted point p3' in R^3 lies below or above 
         *  the plane passing through the three points 
         *  p0', p1', p2'.
         *  The first two coordinates and the third one are specified in 
         *  separate arguments for each vertex.
         * \param[in] p0 , p1 , p2 , p3 first 2 coordinates 
         *   of the vertices of the 3-simplex
         * \param[in] h0 , h1 , h2 , h3 heights of the vertices of 
         *  the 3-simplex
         * \retval POSITIVE if p3' lies below the plane
         * \retval NEGATIVE if p3' lies above the plane
         * \retval perturb() if p3' lies exactly on the hyperplane
         *  where \c perturb() denotes a globally
         *  consistent perturbation, that returns either POSITIVE or NEGATIVE
         */
        Sign GEOGRAM_API orient_2dlifted_SOS(
            const vec2HE& p0, const vec2HE& p1,
            const vec2HE& p2, const vec2HE& p3,
            double h0, double h1, double h2, double h3
        );

        Sign GEOGRAM_API orient_2dlifted_SOS_projected(
            const vec3HE& p0, const vec3HE& p1,
            const vec3HE& p2, const vec3HE& p3,
            double h0, double h1, double h2, double h3,
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

    /**
     * \brief Computes the exact intersection between the support plane
     *  of a triangle and the support line of a segment
     * \pre The intersection exists
     * \param[in] p1 , p2 , p3 the three vertices of the triangle
     * \param[in] q1 , q2 the two vertices of the segment
     * \return the exact intersection between the plane and the line
     */
    vec3HE GEOGRAM_API plane_line_intersection(
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
}

#endif
