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

/**
 * \file geogram/mesh/mesh_surface_intersection.h
 * \brief Functions for computing intersections between surfacic meshes and
 *        boolean operations.
 */

namespace GEO {

    /********************************************************************/    

    /**
     * \brief Computes surface intersections
     * \details New vertices are stored with exact coordinates
     */
    class GEOGRAM_API MeshSurfaceIntersection {
    public:
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


        void remove_external_shell();

        void remove_internal_shells();
        
        /**
         * \brief Display information while computing the intersection.
         *  Default is unset.
         */
        void set_verbose(bool x) {
            verbose_ = x;
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
         * \brief If set, then Delaunay mode uses approximated incircle
         *  predicate (else it uses exact arithmetics). Default is unset.
         */
        void set_approx_incircle(bool x) {
            approx_incircle_ = x;
        }

        /**
         * \brief If set, do not use exact geometry for ordering triangles
         *  around radial edge. Default is unset.
         */
        void set_approx_radial_sort(bool x) {
            radial_sort_.set_approx_predicates(x);
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
        
        void save_exact(const std::string& filename);
        
    protected:

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
         *  with it, they are returned, else an exact vec3HE is constructed
         *  from the double-precision coordinates stored in the mesh
         * \param[in] v a vertex of the mesh
         * \return the exact coordinates of this vertex, as a vector in
         *  homogeneous coordinates stored as expansions
         */
        vec3HE exact_vertex(index_t v) const;

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
        index_t find_or_create_exact_vertex(const vec3HE& p);

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

        /**
         * \brief Sorts a range of halfedges in radial order
         * \param[in] b , e iterators in a vector of halfedge indices
         * \retval true if everything went well
         * \retval false if two triangles are coplanar with same normal
         *  orientation
         */
        bool radial_sort(
            vector<index_t>::iterator b, vector<index_t>::iterator e
        );
        
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
                approx_predicates_(false),
                h_ref_(index_t(-1)),
                degenerate_(false)
            {
            }

            /**
             * \brief Specifies whether approximate predicates shoud be used
             * \param[in] x true if approximate predicates should be used,
             *  default is false (use exact predicates)
             */
            void set_approx_predicates(bool x) {
                approx_predicates_ = x;
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
            static vec3E exact_direction(const vec3HE& p1, const vec3HE& p2);
            
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
            bool approx_predicates_;
            index_t h_ref_; // ---reference halfedge
            vec3E U_ref_;   // -.
            vec3E V_ref_;   //  +-reference basis
            vec3E N_ref_;   // -'
            mutable vector< std::pair<index_t, Sign> > refNorient_cache_;
            mutable bool degenerate_;
        };

        
    protected:
        Process::spinlock lock_;
        Mesh& mesh_;
        Mesh mesh_copy_;
        Attribute<const vec3HE*> vertex_to_exact_point_;
        Attribute<index_t> facet_corner_alpha3_;
        Attribute<bool> facet_corner_degenerate_;
        std::map<vec3HE,index_t,vec3HELexicoCompare> exact_point_to_vertex_;
        RadialSort radial_sort_;
        bool verbose_;
        bool delaunay_;
        bool detect_intersecting_neighbors_;
        bool approx_incircle_;
        bool use_radial_sort_;
        bool normalize_;
        vec3 normalize_center_;
        double normalize_radius_;
        friend class MeshInTriangle;
    };
    
    /********************************************************************/    

    /**
     * \brief Classifies the facets of the result of mesh_intersect_surface()
     *  based on a boolean function
     * \param[in,out] M the surface mesh with the result of 
     *  mesh_intersect_surface()
     * \param[in] eqn the boolean function. Each bit of its parameter 
     *  corresponds to an operand (among 32). 
     * \param[in] attribute if an attribute name is specified, then this 
     *  attribute is set for all facets on the boundary of the computed object,
     *  (besides that the mesh is not modified). If an attribute name is not
     *  specified, then all the facets that are not on the boundary of the
     *  computed object are discarded.
     * \param[in] reorder if the intersection was just computed, one does not
     *  need to reorder the facets and one can set this parameter to false.
     */
    void GEOGRAM_API mesh_classify_intersections(
        Mesh& M, std::function<bool(index_t)> eqn,
        const std::string& attribute="", bool reorder=true
    );

    /**
     * \brief Classifies the facets of the result of mesh_intersect_surface()
     *  based on a boolean function
     * \param[in,out] M the surface mesh with the result of 
     *  mesh_intersect_surface()
     * \param[in] expr the boolean function in ASCII. One can use the following
     *  elements, and parentheses:
     *  - Variables: A..Z or x0..x31 
     *  - and:        '&' or '*'
     *  - or:         '|' or '+'
     *  - xor:        '^'
     *  - difference: '-'
     *  - not:        '!' or '~'
     *  Special values for expr: 
     *  - "union" (union of everything)
     *  - "intersection" (intersection of everything).
     * \param[in] attribute if an attribute name is specified, then this 
     *  attribute is set for all facets on the boundary of the computed object,
     *  (besides that the mesh is not modified). If an attribute name is not
     *  specified, then all the facets that are not on the boundary of the
     *  computed object are discarded.
     * \param[in] reorder if the intersection was just computed, one does not
     *  need to reorder the facets and one can set this parameter to false.
     */
    void GEOGRAM_API mesh_classify_intersections(
        Mesh& M, const std::string& expr,
        const std::string& attribute="", bool reorder=true
    );

    /*************************************************************************/

    /**
     * \brief Computes the union of two surface meshes.
     * \details A and B need to be two closed surface
     *  mesh without intersections.
     * \note This is work in progress, the function is
     *  not robust yet. 
     * \param[in] A , B the two operands.
     * \param[out] result the computed mesh.
     * \param[in] operation one of "A+B", "A*B", "A-B", "B-A"
     */
    void GEOGRAM_API mesh_boolean_operation(
        Mesh& result, Mesh& A, Mesh& B, const std::string& operation
    );
    
    /**
     * \brief Computes the union of two surface meshes.
     * \details A and B need to be two closed surface
     *  mesh without intersections.
     * \note This is work in progress, the function is
     *  not robust yet. 
     * \param[in] A , B the two operands.
     * \param[out] result the computed mesh.
     */
    void GEOGRAM_API mesh_union(Mesh& result, Mesh& A, Mesh& B);

    /**
     * \brief Computes the intersection of two surface meshes.
     * \details A and B need to be two closed surface
     *  mesh without intersections.
     * \note This is work in progress, the function is
     *  not robust yet. 
     * \param[in] A , B the two operands.
     * \param[out] result the computed mesh.
     */
    void GEOGRAM_API mesh_intersection(Mesh& result, Mesh& A, Mesh& B);

    /**
     * \brief Computes the difference of two surface meshes.
     * \details A and B need to be two closed surface
     *  mesh without intersections.
     * \note This is work in progress, the function is
     *  not robust yet. 
     * \param[in] A , B the two operands.
     * \param[out] result the computed mesh.
     */
    void GEOGRAM_API mesh_difference(Mesh& result, Mesh& A, Mesh& B);
    
    /**
     * \brief Attempts to make a surface mesh conformal by
     *  removing intersecting facets and re-triangulating the holes.
     */
    void GEOGRAM_API mesh_remove_intersections(Mesh& M, index_t max_iter = 3);

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

