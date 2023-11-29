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
 
#ifndef GEOGRAM_MESH_MESH_SURFACE_INTERSECTION
#define GEOGRAM_MESH_MESH_SURFACE_INTERSECTION

#include <geogram/basic/common.h>
#include <geogram/mesh/mesh.h>
#include <geogram/numerics/exact_geometry.h>
#include <geogram/basic/process.h>
#include <geogram/basic/attributes.h>
#include <functional>

#ifdef GEOGRAM_WITH_GEOGRAMPLUS
#include <geogramplus/numerics/exact_geometry.h>
#endif 

/**
 * \file geogram/mesh/mesh_surface_intersection.h
 * \brief Functions for computing intersections between surfacic meshes and
 *        for boolean operations.
 */


// If Tessael's geogramplus is available, use exact_nt coordinates,
// else use expansion_nt coordinates.
// exact_nt coordinates makes the algorithm  10x to 20x faster
// and have no risk of underflow / overflow.
#ifdef GEOGRAM_WITH_GEOGRAMPLUS
#define INTERSECTIONS_USE_EXACT_NT
#endif

namespace GEO {

    struct IsectInfo;

    /********************************************************************/

    /**
     * \brief Computes surface intersections
     * \details New vertices are stored with exact coordinates
     */
    class GEOGRAM_API MeshSurfaceIntersection {
    public:

#ifdef INTERSECTIONS_USE_EXACT_NT
        typedef vec3HEx ExactPoint;
        // Exact points are canonicalized
        // (by Numeric::optimize_number_representation(vec3HEx)) so
        // we can use this comparator that makes the global vertex map
        // much much faster.
        typedef vec3HExLexicoCompareCanonical ExactPointLexicoCompare;
#else    
        typedef vec3HE  ExactPoint;
        // Generic comparator for global vertex map.
        typedef vec3HgLexicoCompare<ExactPoint::value_type>
             ExactPointLexicoCompare;        
#endif
        typedef ExactPoint::value_type ExactCoord;
        typedef vecng<3,ExactCoord> ExactVec3;
        typedef vecng<2,ExactCoord> ExactVec2;
        typedef vec2Hg<ExactCoord> ExactVec2H;
        typedef rationalg<ExactCoord> ExactRational;
        
        MeshSurfaceIntersection(Mesh& M);
        ~MeshSurfaceIntersection();

        /**
         * \details A facet attribute of type index_t named "operand_bit" can 
         *  indicate for each facet to which operand of a n-ary boolean 
         *  operation it corresponds to (the same facet might belong to 
         *  several operands). 
         *  It is taken into account by the two variants of 
         *  mesh_classify_intersections()
         */
        void intersect();


        /**
         * \brief Removes all the facets that are on the outer boundary
         * \pre set_radial_sort(true) was set before calling intersect()
         */
        void remove_external_shell();

        /**
         * \brief Removes all the facets that are not on the outer boundary
         * \pre set_radial_sort(true) was set before calling intersect()
         */
        void remove_internal_shells();

        /**
         * \brief Classifies the facets and keep only 
         *   the ones on the boundary of a combination of regions defined
         *   by a boolean expression.
         * \details A facet attribute of type index_t named "operand_bit" 
         *  indicates for each facet to which operand of a n-ary boolean 
         *  operation it corresponds to (the same facet might belong to 
         *  several operands). 
         * \pre set_radial_sort(true) was set before calling intersect()
         * \param[in] expr the boolean function in ASCII. 
         *  One can use the following elements, and parentheses:
         *  - Variables: A..Z or x0..x31, correspond to the bits of the 
         *    "operand_bit" attribute
         *  - the special variable '*' corresponds to the union of everything
         *  - and:        '&' or '*'
         *  - or:         '|' or '+'
         *  - xor:        '^'
         *  - difference: '-'
         *  - not:        '!' or '~'
         *  Special values for expr: 
         *  - "union" (union of everything), synonym of '*'
         *  - "intersection" (intersection of everything).
         */
        void classify(const std::string& expr);

        /**
         * \brief Merge coplanar facets and retriangulate them using a 
         *  Constrained Delaunay triangulation
         */
        void simplify_coplanar_facets();
        
        /**
         * \brief Display information while computing the intersection.
         *  Default is unset.
         */
        void set_verbose(bool x) {
            verbose_ = x;
        }

        /**
         * \brief Sets the threshold from which triangle is considered 
         *  to be a monster.
         * \details Monster triangles are saved to a file for the zoo.
         * \param[in] nb if a triangle has more than \p nb intersections
         *  in it, then it is considered to be a monster. 
         */
        void set_monster_threshold(index_t nb) {
            monster_threshold_ = nb;
        }

        /**
         * \brief In dry run mode, the computed local triangulations
         *  are not inserted in the global mesh. This is for benchmarking.
         *  Default is off.
         */
        void set_dry_run(bool x) {
            dry_run_ = x;
        }

        /**
         * \brief If set, compute constrained Delaunay triangulation
         *  in the intersected triangles. If there are intersections
         *  in coplanar facets, it guarantees uniqueness of their
         *  triangulation. Default is set.
         */
        void set_delaunay(bool x) {
            delaunay_ = x;
        }
        
        /** 
         * \brief detect and compute intersections between facets that share 
         *  a facet or an edge. Set to false if input is a set of conformal
         *  meshes. Default is set.
         */
        void set_detect_intersecting_neighbors(bool x) {
            detect_intersecting_neighbors_ = x;
        }

        /**
         * \brief Specifies whether surfaces should be duplicated and
         *  radial edges sorted in order to create the volumetric
         *  partition yielded by the intersection
         * \param[in] x true if radial edges should be sorted. Default is
         *  set
         */
        void set_radial_sort(bool x) {
            use_radial_sort_ = x;
        }

        /**
         * \brief Specifies whether coordinates should be normalized during
         *  computation. If set, original coordinates are restored at the
         *  end of intersect().
         * \param[in] x true if coordinates should be normalized. Default is
         *  set.
         */
        void set_normalize(bool x) {
            normalize_ = x;
        }


        /**
         * \brief Optionally save the skeleton (that is, the collection of 
         *  non-manifold edges) to a given mesh.
         * \param[in] skeleton a pointer to the mesh that will receive the
         *  skeleton.
         */
        void set_build_skeleton(Mesh* skeleton) {
            skeleton_ = skeleton;
        }
        
    protected:
        /**
         * \brief substep of intersect(), prepares the mesh
         * \details Tesselates the facets if they are not triangulated,
         *  creates the operand bit for boolean op classification, removes
         *  the exactly degenerate triangles, colocate the points,
         *  optionally scales the coordinates and sets symbolic perturbation
         *  mode to lexicographic.
         */
        void intersect_prologue();

        /**
         * \brief substep of intersect(), finds all the intersection points
         *   and segments.
         * \param[out] intersections the vector of IsectInfo. Each IsectInfo
         *   is either an intersection vertex or a pair of intersection
         *   vertices. Intersection vertices are represented in symbolic
         *   form, as a couple of triangle indices plus a couple of triangle
         *   subregion id (TriangleRegion).
         * \details First uses a MeshFacetsAABB to detect candidate pairs
         *   of intersecting triangles, then calls triangles_intersection()
         *   in parallel. Finally, mesh facets are shuffled randomly, to
         *   ensure balanced multithreading for the subsequent steps.
         */
        void intersect_get_intersections(vector<IsectInfo>& intersections);

        /**
         * \brief substep of intersect(), inserts the intersection points
         *   and segments into the triangles.
         * \param[in,out] intersections the vector of IsectInfo. Each IsectInfo
         *   is either an intersection vertex or a pair of intersection
         *   vertices. Intersection vertices are represented in symbolic
         *   form, as a couple of triangle indices plus a couple of triangle
         *   subregion id (TriangleRegion).
         * \details Uses MeshInTriangle, a class derived from CDTBase2d, 
         *   that computes a constrained Delaunay triangulation with 
         *   intersection points represented with exact coordinates. 
         *   Operates in parallel. Each thread computes constrained
         *   Delaunay triangulations independently, and commits them in the
         *   resulting mesh (with a lock to protect concurrent accesses).
         *   The initial mesh is copied (and kept in the mesh_copy_ member), 
         *   so that concurrent read access do not need a lock. 
         */
        void intersect_remesh_intersections(vector<IsectInfo>& intersections);

        /**
         * \brief subset of intersect(), cleans the resulting mesh and
         *   undoes optional geometric normalization.
         * \param[in] intersections the vector of IsectInfo. Each IsectInfo
         *   is either an intersection vertex or a pair of intersection
         *   vertices. Intersection vertices are represented in symbolic
         *   form, as a couple of triangle indices plus a couple of triangle
         *   subregion id (TriangleRegion).
         * \details find the intersection that landed exactly onto an
         *   existing mesh vertex and merges them. Removes the initial 
         *   triangles that had intersections (they are replaced with new
         *   triangles). Merges duplicated triangles that come from 
         *   coplanar regions. Undoes geometric normalizations. Restores
         *   initial symbolic perturbation mode.
         */
        void intersect_epilogue(const vector<IsectInfo>& intersections);

    
        /**
         * \brief Acquires a lock on this mesh
         * \details A single thread can have the lock. When multiple threads 
         *  want the lock, the ones that do not have it keep waiting until 
         *  the one that owns the lock calls unlock(). All threads that modify 
         *  the target mesh should call this function
         * \see unlock()
         */
        void lock() {
            Process::acquire_spinlock(lock_);
        }

        /**
         * \brief Releases the lock associated with this mesh
         */
        void unlock() {
            Process::release_spinlock(lock_);
        }

        /**
         * \brief Gets the exact point associated with a vertex
         * \details If the vertex has explicit exact coordinates associated
         *  with it, they are returned, else an exact ExactPoint is constructed
         *  from the double-precision coordinates stored in the mesh
         * \param[in] v a vertex of the mesh
         * \return the exact coordinates of this vertex, as a vector in
         *  homogeneous coordinates stored as expansions
         */
        ExactPoint exact_vertex(index_t v) const;

        /**
         * \brief Finds or creates a vertex in the mesh, by exact coordinates
         * \details If there is already a vertex with coordinates \p p, then
         *  the existing vertex is returned, else a new vertex is constructed.
         *  Note that only the vertices created by find_or_create_vertex() can
         *  be returned as existing vertices. Mesh vertices stored as double-
         *  precision coordinates are not retreived by this function.
         * \param[in] p the exact coordinates of a point
         * \return the index of a mesh vertex with \p p as coordinates
         */
        index_t find_or_create_exact_vertex(const ExactPoint& p);

        /**
         * \brief Gets the target mesh
         * \return a modifiable reference to the mesh that was passed to
         *  the constructor
         */
        Mesh& target_mesh() {
            return mesh_;
        }

        /**
         * \brief Gets the target mesh
         * \return a const reference to the mesh that was passed to
         *  the constructor
         */
        const Mesh& target_mesh() const {
            return mesh_;
        }
        
        /**
         * \brief Gets a copy of the initial mesh passed to the constructor
         * \details It is used by the multithreaded mesh intersection algorithm.
         *   Each thread needs to both access the initial geometry and create
         *   new vertices and triangles in the target mesh. Creating new mesh
         *   elements can reallocate the internal vectors of the mesh, and 
         *   change the address of the elements. This should not occur while 
         *   another thread is reading the mesh. Copying the initial geometry 
         *   in another mesh prevents this type of problems.
         * \return a const reference to the mesh that was copied from the one
         *   passed to the constructor
         */
        const Mesh& readonly_mesh() const {
            return mesh_copy_;
        }

        class RadialSort;
        
        /**
         * \brief Sorts a range of halfedges in radial order
         * \param[in] RS a reference to a RadialSort
         * \param[in] b , e iterators in a vector of halfedge indices
         * \retval true if everything went well
         * \retval false if two triangles are coplanar with same normal
         *  orientation
         */
        bool radial_sort(
            RadialSort& RS,
            vector<index_t>::iterator b, vector<index_t>::iterator e
        );

        /**
         * \brief Builds the Weiler model
         * \details The Weiler model is a volumetric representation, where each
         *  facet is on the boundary of a closed region. Facets are duplicated,
         *  so that when two regions touch each other, each region has its own 
         *  facet on the boundary. Two facets that touch in this way are 
         *  connected by alpha3 links. Facets on the boundary of the same 
         *  region are connected by alpha2 links. 
         */
        void build_Weiler_model();

        /**
         * \brief Marks all the facets that are on the external shell
         */
        void mark_external_shell(vector<index_t>& on_external_shell);

        index_t halfedge_vertex(index_t h, index_t dlv) const {
            index_t f  = h/3;
            index_t lv = (h+dlv)%3;
            return mesh_.facets.vertex(f,lv);
        }

        index_t alpha2(index_t h) const {
            index_t t1 = h/3;
            index_t t2 = mesh_.facet_corners.adjacent_facet(h);
            if(t2 == index_t(-1)) {
                return index_t(-1);
            }
            for(index_t h2: mesh_.facets.corners(t2)) {
                if(mesh_.facet_corners.adjacent_facet(h2) == t1) {
                    return h2;
                }
            }
            geo_assert_not_reached;
        }

        void sew2(index_t h1, index_t h2) {
            geo_debug_assert(
                halfedge_vertex(h1,0) == halfedge_vertex(h2,1)
            );
            geo_debug_assert(
                halfedge_vertex(h2,0) == halfedge_vertex(h1,1)
            );            
            index_t t1 = h1/3;
            index_t t2 = h2/3;
            mesh_.facet_corners.set_adjacent_facet(h1,t2);
            mesh_.facet_corners.set_adjacent_facet(h2,t1);
        }
        
        index_t alpha3(index_t h) const {
            return facet_corner_alpha3_[h];
        }

        index_t alpha3_facet(index_t f) const {
            return alpha3(3*f)/3;
        }
        
        void sew3(index_t h1, index_t h2) {
            geo_debug_assert(
                halfedge_vertex(h1,0) == halfedge_vertex(h2,1)
            );
            geo_debug_assert(
                halfedge_vertex(h2,0) == halfedge_vertex(h1,1)
            );            
            facet_corner_alpha3_[h1] = h2;
            facet_corner_alpha3_[h2] = h1;
        }

        void save_triangle(const std::string& name, index_t h) {
            std::ofstream out(name + ".obj");
            index_t v1 = halfedge_vertex(h,0);
            index_t v2 = halfedge_vertex(h,1);
            index_t v3 = halfedge_vertex(h,2);
            out << "v " << vec3(mesh_.vertices.point_ptr(v1)) << std::endl;
            out << "v " << vec3(mesh_.vertices.point_ptr(v2)) << std::endl;
            out << "v " << vec3(mesh_.vertices.point_ptr(v3)) << std::endl;
            out << "f 1 2 3" << std::endl;
        }

        void save_radial(
            const std::string& name,
            vector<index_t>::iterator b, vector<index_t>::iterator e
        ) {
            std::ofstream out(name + ".obj");
            index_t v_ofs = 0;
            for(vector<index_t>::iterator i=b; i!=e; ++i) {
                index_t t = (*i/3);
                index_t v1 = mesh_.facets.vertex(t,0);
                index_t v2 = mesh_.facets.vertex(t,1);
                index_t v3 = mesh_.facets.vertex(t,2);
                out << "v "
                    << vec3(mesh_.vertices.point_ptr(v1)) << std::endl;
                out << "v "
                    << vec3(mesh_.vertices.point_ptr(v2)) << std::endl;
                out << "v "
                    << vec3(mesh_.vertices.point_ptr(v3)) << std::endl;
                out << "f "
                    << v_ofs+1 << " " << v_ofs+2 << " " << v_ofs+3
                    << std::endl;
                v_ofs += 3;
            }
        }

        /**
         * \brief Tests whether a segment intersects a triangle
         * \details All points are given with exact homogeneous
         *  coordinates (MeshSurfaceIntersection::ExactPoint)
         * \param[in] P1 , P2 the two extremities of the segment
         * \param[in] p1 , p2 , p3 the three verties of the triangle
         * \param[out] degenerate if set, the segment passes exactly
         *  through one of the vertices, one of the edges or through
         *  the supporting plane of the triangle.
         * \retval true if the segment has an intersection with the
         *  interior of the triangle and is not contained in the
         *  supporting plane of the triangle
         * \retval false otherwise
         */
        static bool segment_triangle_intersection(
            const ExactPoint& P1, const ExactPoint& P2, 
            const ExactPoint& p1,
            const ExactPoint& p2,
            const ExactPoint& p3,
            bool& degenerate
        );

    protected:

        /**
         * A class for sorting triangles around their common radial edge.
         */
        class GEOGRAM_API RadialSort {
        public:
            /**
             * \brief RadialSort constructor
             * \param[in] mesh the MeshSurfaceIntersection
             */
            RadialSort(
                const MeshSurfaceIntersection& mesh
            ) : mesh_(mesh),
                h_ref_(index_t(-1)),
                degenerate_(false)
            {
            }

            /**
             * \brief Initializes radial sorting around a given halfedge
             * \param[in] h_ref the reference halfedge 
             */
            void init(index_t h_ref);

            /**
             * \brief Compares two halfedges
             * \param[in] h1 , h2 the two halfedges
             * \retval true if \p h1 should be before \p h2 in radial order
             * \retval false otherwise
             */
            bool operator()(index_t h1, index_t h2) const;

            /**
             * \brief Tests if a degeneracy was encountered
             * \retval true if there were two coplanar triangles on the same
             *  side relative to h_ref
             * \retval false otherwise
             */
            bool degenerate() const {
                return degenerate_;
            }

            /**
             * \brief Computes a vector of arbitrary length with its 
             *  direction given by two points 
             * \param[in] p1 , p2 the two points in homogeneous coordinates
             * \return a vector in cartesian coordinates with the same 
             *  direction and orientation as \p p2 - \p p1
             */
            static ExactVec3 exact_direction(
                const ExactPoint& p1, const ExactPoint& p2
            );

            /**
             * \brief Computes an interval vector of arbitrary length with its 
             *  direction given by two points 
             * \param[in] p1 , p2 the two points in homogeneous coordinates
             * \return an interval vector in cartesian coordinates 
             *  with the same direction and orientation as \p p2 - \p p1
             */
            static vec3I exact_direction_I(
                const ExactPoint& p1, const ExactPoint& p2
            );
            
        protected:

            /**
             * \brief Computes the relative orientations of two halfedges
             * \param[in] h1 , h2 the two halfedges
             * \retval POSITIVE if going from \p h1's triangle to
             *  \p h2's triangle is a left turn (with h_ref facing to you)
             * \retval ZERO if \p h1 and \p h2 have co-linear normals
             * \retval NEGATIVE otherwise
             */
            Sign h_orient(index_t h1, index_t h2) const;

            /**
             * \brief Computes the normal orientation of a halfedge 
             *  relative to h_ref
             * \return the sign of the dot product between h_ref's triangle 
             *  normal and \p h2's triangle normal.
             */
            Sign h_refNorient(index_t h2) const;

        public:
            void test(index_t h1, index_t h2) {
                (*this)(h1,h2);
                Sign o_ref1 = h_orient(h_ref_, h1);
                Sign o_ref2 = h_orient(h_ref_, h2);
                Sign oN_ref1 = h_refNorient(h1);
                Sign oN_ref2 = h_refNorient(h2);
                Sign o_12 = h_orient(h1,h2);
                std::cerr
                   << " o_ref1=" << int(o_ref1) << " o_ref2=" << int(o_ref2)
                   << " oN_ref1=" << int(oN_ref1) << " oN_ref2=" << int(oN_ref2)
                   << " o_12=" << int(o_12)
                   << std::endl;
            }

            
        private:
            const MeshSurfaceIntersection& mesh_;
            index_t h_ref_; // ---reference halfedge
            ExactVec3 U_ref_;   // -.
            ExactVec3 V_ref_;   //  +-reference basis
            ExactVec3 N_ref_;   // -'
            vec3I U_ref_I_; // -.
            vec3I V_ref_I_; //  +-reference basis (interval arithmetics)
            vec3I N_ref_I_; // _'
            mutable vector< std::pair<index_t, Sign> > refNorient_cache_;
            mutable bool degenerate_;
        };

        
    protected:
        Process::spinlock lock_;
        Mesh& mesh_;
        Mesh mesh_copy_;
        Attribute<const ExactPoint*> vertex_to_exact_point_;
        Attribute<index_t> facet_corner_alpha3_;
        Attribute<bool> facet_corner_degenerate_;
        
        
        std::map<ExactPoint,index_t,ExactPointLexicoCompare>
            exact_point_to_vertex_;
        
        bool verbose_;
        bool delaunay_;
        bool detect_intersecting_neighbors_;
        bool use_radial_sort_;

        PCK::SOSMode SOS_bkp_;
        bool rescale_; 
        bool normalize_;
        vec3 normalize_center_;
        double normalize_radius_;
        
        index_t monster_threshold_;
        bool dry_run_;
        friend class MeshInTriangle;
        friend class CoplanarFacets;

        Mesh* skeleton_;
    };
    
    /********************************************************************/    

    /**
     * \brief Computes a boolean operation with two surface meshes.
     * \details A and B need to be two closed surface
     *  mesh without intersections.
     * \param[in] A , B the two operands.
     * \param[out] result the computed mesh.
     * \param[in] operation one of "A+B", "A*B", "A-B", "B-A"
     * \param[in] verbose if set, display additional information 
     *   during computation
     */
    void GEOGRAM_API mesh_boolean_operation(
        Mesh& result, Mesh& A, Mesh& B, const std::string& operation,
        bool verbose=false
    );
    
    /**
     * \brief Computes the union of two surface meshes.
     * \details A and B need to be two closed surface
     *  mesh without intersections.
     * \param[in] A , B the two operands.
     * \param[out] result the computed mesh.
     * \param[in] verbose if set, display additional 
     *  information during computation
     */
    void GEOGRAM_API mesh_union(
        Mesh& result, Mesh& A, Mesh& B, bool verbose=false
    );

    /**
     * \brief Computes the intersection of two surface meshes.
     * \details A and B need to be two closed surface
     *  mesh without intersections.
     * \param[in] A , B the two operands.
     * \param[out] result the computed mesh.
     * \param[in] verbose if set, display additional information 
     *  during computation
     */
    void GEOGRAM_API mesh_intersection(
        Mesh& result, Mesh& A, Mesh& B, bool verbose=false
    );

    /**
     * \brief Computes the difference of two surface meshes.
     * \details A and B need to be two closed surface
     *  mesh without intersections.
     * \param[in] A , B the two operands.
     * \param[out] result the computed mesh.
     * \param[in] verbose if set, display additional information 
     *  during computation
     */
    void GEOGRAM_API mesh_difference(
        Mesh& result, Mesh& A, Mesh& B, bool verbose=false
    );
    
    /**
     * \brief Attempts to make a surface mesh conformal by
     *  removing intersecting facets and re-triangulating the holes.
     * \param[in] verbose if set, display additional information 
     *  during computation
     */
    void GEOGRAM_API mesh_remove_intersections(
        Mesh& M, index_t max_iter = 3, bool verbose=false
    );

    /**
     * \brief Tests whether two mesh facets have a non-degenerate intersection.
     * \details If the facets are polygonal, they are triangulated from the
     *  first vertex, and intersections between each pair of triangles is
     *  tested.
     * \retval true if the two facets have an intersection. If they share a
     *  vertex, it does not count as an intersection. 
     * \retval false otherwise.
     */
    bool GEOGRAM_API mesh_facets_have_intersection(
        Mesh& M, index_t f1, index_t f2
    );

    /**************************************************************************/
}

#endif

