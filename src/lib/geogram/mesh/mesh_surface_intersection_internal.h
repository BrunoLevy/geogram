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

#ifndef GEOGRAM_MESH_MESH_SURFACE_INTERSECTION_INTERNAL
#define GEOGRAM_MESH_MESH_SURFACE_INTERSECTION_INTERNAL

/**
 * \file mesh_surface_intersection_internal.h
 * \brief Classes used by MeshSurfaceIntersection
 */

#include <geogram/basic/common.h>
#include <geogram/mesh/mesh_surface_intersection.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/index.h>
#include <geogram/mesh/triangle_intersection.h>
#include <geogram/delaunay/CDT_2d.h>
#include <geogram/numerics/exact_geometry.h>

#include <map>

namespace GEO {

    /**
     * \brief Meshes a single triangle with the constraints that come from
     *  the intersections with the other triangles.
     * \details Inherits CDTBase2d (constrained Delaunay triangulation), and
     *  redefines orient2d(), incircle2d() and create_intersection() using
     *  vectors with homogeneous coordinates stored as arithmetic expansions
     *  (vec2HE) or arbitrary-precision floating point numbers (vec2HEx) if
     *  compiled with Tessael's geogramplus extension package.
     */
    class MeshInTriangle : public CDTBase2d {
    public:

        typedef exact::vec3h ExactPoint;
        
        /***************************************************************/

        /**
         * \brief An edge of the mesh.
         * \details It represents the constraints to be used by the
         *  constrained triangulation to remesh the facet. Makes a maximum 
         *  use of the combinatorial information to reduce the complexity 
         *  (degree) of the constructed coordinates as much as possible.
         */
        class Edge {
        public:
            Edge(
                index_t v1_in = index_t(-1),
                index_t v2_in = index_t(-1),
                index_t f2    = index_t(-1),
                TriangleRegion R2 = T2_RGN_T
            ) : v1(v1_in),
                v2(v2_in) {
                sym.f2 = f2;
                sym.R2 = R2;
            }
            index_t v1,v2; // The two extremities of the edge
            struct {       // Symbolic information: this edge = f1 /\ f2.R2
                index_t        f2;
                TriangleRegion R2;
            } sym;
        };

        /***************************************************************/

        /**
         * \brief A vertex of the triangulation
         * \details Stores geometric information in exact precision, both
         *  in 3D and in local 2D coordinates. It also stores symbolic 
         *  information, that is, facet indices and regions that generated 
         *  the vertex.
         */
        class Vertex {
        public:
            
            enum Type {
                UNINITIALIZED, MESH_VERTEX, PRIMARY_ISECT, SECONDARY_ISECT
            };

            /**
             * \brief Constructor for macro-triangle vertices.
             * \param[in] f facet index, supposed to correspond to
             *  MeshInTriangle's current facet
             * \param[in] lv local vertex index in \p f
             */
            Vertex(MeshInTriangle* M, index_t f, index_t lv) {
                geo_assert(f == M->f1_);
                type = MESH_VERTEX;
                mit = M;
                init_sym(f, index_t(-1), TriangleRegion(lv), T2_RGN_T);
                init_geometry(compute_geometry());
            }

            /**
             * \brief Constructor for intersections with other facets.
             * \param[in] f1 , f2 the two facets. \p f1 is suposed to
             *  correspond to MeshInTriangle's current facet
             * \param[in] R1 , R2 the two facet regions.
             */
            Vertex(
                MeshInTriangle* M,
                index_t f1, index_t f2,
                TriangleRegion R1, TriangleRegion R2
            ) {
                geo_assert(f1 == M->f1_);                
                type = PRIMARY_ISECT;
                mit = M;
                init_sym(f1,f2,R1,R2);
                init_geometry(compute_geometry());
            }

            /**
             * \brief Constructor for intersections between constraints.
             * \param[in] point_exact_in exact 3D coordinates 
             *   of the intersection
             */
            Vertex(
                MeshInTriangle* M, const ExactPoint& point_exact_in
            ) {
                type = SECONDARY_ISECT;                
                mit = M;
                init_sym(index_t(-1), index_t(-1), T1_RGN_T, T2_RGN_T);
                init_geometry(point_exact_in);
            }

            /**
             * \brief Default constructor
             */
            Vertex() {
                type = UNINITIALIZED;                
                mit = nullptr;
                init_sym(index_t(-1), index_t(-1), T1_RGN_T, T2_RGN_T);
                mesh_vertex_index = index_t(-1);
            }

            /**
             * \brief Gets the mesh
             * \return a reference to the mesh
             */
            const Mesh& mesh() const {
                return mit->mesh();
            }

            /**
             * \brief Prints this vertex
             * \details Displays the combinatorial information
             * \param[out] out an optional stream where to print
             */
            void print(std::ostream& out=std::cerr) const;

            /**
             * \brief Gets a string representation of this Vertex
             * \return a string with the combinatorial information 
             *  of this Vertex
             */
            std::string to_string() const {
                std::ostringstream out;
                print(out);
                return out.str();
            }

            vec2 get_UV_approx() const {
                double u = point_exact[mit->u_].estimate();
                double v = point_exact[mit->v_].estimate();
                double w = point_exact.w.estimate();
                return vec2(u/w,v/w);
            }
            
        protected:

            /**
             * \brief Initializes the symbolic information of this Vertex
             * \param[in] f1 , f2 the two facets. \p f1 is suposed to
             *  correspond to MeshInTriangle's current facet
             * \param[in] R1 , R2 the two facet regions.
             */
            void init_sym(
                index_t f1, index_t f2, TriangleRegion R1, TriangleRegion R2
            ) {
                sym.f1 = f1;
                sym.f2 = f2;
                sym.R1 = R1;
                sym.R2 = R2;
                mesh_vertex_index = index_t(-1);
            }

            /**
             * \brief Gets the geometry of this vertex
             * \details Computes the exact 3D position of this vertex
             *  based on the mesh and the combinatorial information
             */
            ExactPoint compute_geometry();

            /**
             * \brief Optimizes exact numbers in generated
             *  points and computes approximate coordinates.
             */
            void init_geometry(const ExactPoint& P);

        public:
            MeshInTriangle* mit;
            ExactPoint point_exact; // Exact homogeneous coords using expansions
            Type type;          // MESH_VERTEX, PRIMARY_ISECT or SECONDARY_ISECT
            index_t mesh_vertex_index; // Global mesh vertex index once created
            struct {                   // Symbolic information - tri-tri isect
                index_t f1,f2;         //   global facet indices in mesh
                TriangleRegion R1,R2;  //   triangle regions
            } sym;
#ifndef GEOGRAM_USE_EXACT_NT            
            double l; // precomputed approximated (p[u]^2 + p[v]^2) / p.w^2
#endif            
        };

        /***************************************************************/
        
        MeshInTriangle(MeshSurfaceIntersection& EM);

        /**
         * \brief Gets the readonly initial mesh
         * \return a const reference to a copy of the initial mesh
         */
        const Mesh& mesh() const {
            return mesh_;
        }

        /**
         * \brief Gets the target mesh
         * \return a reference to the target mesh
         */
        Mesh& target_mesh() {
            return exact_mesh_.target_mesh();            
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
         * \brief For debugging, save constraints to a file
         * \param[in] filename a mesh filename where to solve the constraints
         *  (.obj or .geogram)
         */
        void save_constraints(const std::string& filename) {
            Mesh M;
            get_constraints(M);
            mesh_save(M,filename);
        }
        
        void begin_facet(index_t f);
        
        index_t add_vertex(index_t f2, TriangleRegion R1, TriangleRegion R2);

        void add_edge(
            index_t f2,
            TriangleRegion AR1, TriangleRegion AR2,
            TriangleRegion BR1, TriangleRegion BR2
        );

        /**
         * \brief Creates new vertices and new triangles in target mesh
         */
        void commit();


        /**
         * \see CDT2d::clear()
         */
        void clear() override;

    protected:
        /**
         * \brief For debugging, copies the constraints to a mesh
         */
        void get_constraints(Mesh& M, bool with_edges=true) const;
        
        vec3 mesh_vertex(index_t v) const {
            return vec3(mesh().vertices.point_ptr(v));
        }
        
        vec3 mesh_facet_vertex(index_t f, index_t lv) const {
            index_t v = mesh().facets.vertex(f,lv);
            return mesh_vertex(v);
        }

        vec2 mesh_vertex_UV(index_t v) const {
            const double* p = mesh().vertices.point_ptr(v);
            return vec2(p[u_], p[v_]);
        }
            
        vec2 mesh_facet_vertex_UV(index_t f, index_t lv) const {
            index_t v = mesh().facets.vertex(f,lv);
            return mesh_vertex_UV(v);
        }
        

        void log_err() const {
            std::cerr << "Houston, we got a problem (while remeshing facet "
                      << f1_ << "):" << std::endl;
        }
        
    protected:

        /********************** CDTBase2d overrides ***********************/

        /**
         * \brief Tests the orientation of three vertices
         * \param[in] v1 , v2 , v3 the three vertices
         * \retval POSITIVE if they are in the trigonometric order
         * \retval ZERO if they are aligned
         * \retval NEGATIVE if they are in the anti-trigonometric order
         */
        Sign orient2d(index_t v1,index_t v2,index_t v3) const override;

        /**
         * \brief Tests the relative position of a point with respect
         *  to the circumscribed circle of a triangle
         * \param[in] v1 , v2 , v3 the three vertices of the triangle
         *  oriented anticlockwise
         * \param[in] v4 the point to be tested
         * \retval POSITIVE if the point is inside the circle
         * \retval ZERO if the point is on the circle
         * \retval NEGATIVE if the point is outside the circle
         */
        Sign incircle(
            index_t v1,index_t v2,index_t v3,index_t v4
        ) const override;

        /**
         * \brief Given two segments that have an intersection, create the
         *  intersection
         * \details The intersection is given both as the indices of segment
         *  extremities (i,j) and (k,l), that one can use to retreive the 
         *  points in derived classes, and constraint indices E1 and E2, that
         *  derived classes may use to retreive symbolic information attached
         *  to the constraint
         * \param[in] e1 the index of the first edge, corresponding to the
         *  value of ncnstr() when insert_constraint() was called for
         *  that edge
         * \param[in] i , j the vertices of the first segment
         * \param[in] e2 the index of the second edge, corresponding to the
         *  value of ncnstr() when insert_constraint() was called for
         *  that edge
         * \param[in] k , l the vertices of the second segment
         * \return the index of a newly created vertex that corresponds to
         *  the intersection between [\p i , \p j] and [\p k , \p l]
         */
        index_t create_intersection(
            index_t e1, index_t i, index_t j,
            index_t e2, index_t k, index_t l
        ) override;

        /**
         * \brief Computes the intersection between two edges
         * \param[in] e1 , e2 the two edges
         * \param[out] I the intersection
         */
        void get_edge_edge_intersection(
            index_t e1, index_t e2, ExactPoint& I
        ) const;

        /**
         * \brief Auxilliary function used by get_edge_edge_intersection()
         *  for the special case when the two edges are coplanar
         * \param[in] e1 , e2 the two edges
         * \param[out] I the intersection
         */
        void get_edge_edge_intersection_2D(
            index_t e1, index_t e2, ExactPoint& I
        ) const;

    public:
        void save(const std::string& filename) const override;

    protected:
        void begin_insert_transaction() override;
        void commit_insert_transaction() override;
        void rollback_insert_transaction() override;
        
    private:
        MeshSurfaceIntersection& exact_mesh_;
        const Mesh& mesh_;
        index_t f1_;
        index_t latest_f2_;
        index_t latest_f2_count_;
        coord_index_t f1_normal_axis_;
        coord_index_t u_; // = (f1_normal_axis_ + 1)%3
        coord_index_t v_; // = (f1_normal_axis_ + 2)%3
        vector<Vertex> vertex_;
        vector<Edge> edges_;
        bool has_planar_isect_;
        bool dry_run_;
        mutable std::map<trindex, Sign> pred_cache_;
        bool use_pred_cache_insert_buffer_;
        mutable std::vector< std::pair<trindex, Sign> >
             pred_cache_insert_buffer_;
    };

    /*************************************************************************/

    /**
     * \brief Stores information about a triangle-triangle intersection.
     * \details The intersection is a segment. Its extremities are indicated
     *  by the regions in f1 and f2 that created the intersection. If the
     *  intersection is just a point, then A and B regions are the same.
     */
    struct IsectInfo {
    public:

        /**
         * Swaps the two facets and updates the combinatorial
         * information accordingly.
         */
        void flip() {
            std::swap(f1,f2);
            A_rgn_f1 = swap_T1_T2(A_rgn_f1);
            A_rgn_f2 = swap_T1_T2(A_rgn_f2);
            std::swap(A_rgn_f1, A_rgn_f2);
            B_rgn_f1 = swap_T1_T2(B_rgn_f1);
            B_rgn_f2 = swap_T1_T2(B_rgn_f2);
            std::swap(B_rgn_f1, B_rgn_f2);
        }

        /**
         * \brief Tests whether intersection is just a point.
         * \details Points are encoded as segments with the
         *  same symbolic information for both vertices.
         */
        bool is_point() const {
            return
                A_rgn_f1 == B_rgn_f1 &&
                A_rgn_f2 == B_rgn_f2 ;
        }
        
        index_t f1; 
        index_t f2;
        TriangleRegion A_rgn_f1;
        TriangleRegion A_rgn_f2;
        TriangleRegion B_rgn_f1;
        TriangleRegion B_rgn_f2;
    };

    /**********************************************************************/

    /**
     * \brief Detects and retriangulates a set of coplanar facets for
     *  MeshSurfaceIntersection.
     */
    class CoplanarFacets {
    public:
        static constexpr index_t NON_MANIFOLD = index_t(-2);
        typedef MeshSurfaceIntersection::ExactPoint ExactPoint;
        
        /**
         * \brief Constructs a CoplanarFacets object associated with a
         *   MeshSurfaceIntersection
         * \details No set of facets is identified. One needs to call get().
         * \param[in] I a reference to the MeshSurfaceIntersection
         * \param[in] clear_attributes if set, resets facet_chart and 
         *  keep_vertex
         * \param[in] angle_tolerance angle tolerance for detecting coplanar
         *  facets and colinear edges (in degrees)
         */
        CoplanarFacets(
            MeshSurfaceIntersection& I, bool clear_attributes,
            double angle_tolerance = 0.0
        );

        /**
         * \brief Gets the set of coplanar facets from a given facet and
         *   group id.
         * \details Uses the "group" facets attribute. If \p f's group is
         *  uninitialized (NO_INDEX), determines the facets of the group
         *  geometrically and initializes the attribute, else gets the
         *  facets based on the attribute.
         * \param[in] f the facet
         * \param[in] group_id the facet group id
         */
        void get(index_t f, index_t group_id);

        /**
         * \brief Marks the vertices that need to be kept in the 
         *  simplified facets.
         * \details A vertex is kept if it is incident to at least 
         *  two non-colinear
         *  edges on the border. The status of the vertices is stored in the
         *  "keep" vertex attribute.
         */
        void mark_vertices_to_keep();

        /**
         * \brief For debugging purposes, saves border edges to a file.
         * \param[in] filename the file where to store the borders.
         */
        void save_borders(const std::string& filename);

        /**
         * \brief For debugging purposes, saves all the facets of the group 
         *  to a file.
         * \param[in] filename the file where to store the facets of the group.
         */
        void save_facet_group(const std::string& filename);

        /**
         * \brief Triangulates the kept vertices.
         * \details One can get the triangle through the (public) CDT member
         *  (ExactCDT2d).
         */
        void triangulate();

    protected:

        /**
         * \brief Finds all the pairs of coplanar facets
         * \details Initializes c_is_coplanar_[], a vector of booleans indexed
         *   by facet corners.
         */
        void find_coplanar_facets();
        
        /**
         * \brief Gets the coordinate along which one can project a triangle
         *  without creating degeneracies.
         * \param[in] p1 , p2 , p3 the three vertices of the triangle, with
         *   exact homogeneous coordinates.
         * \return one of {0,1,2}
         */
        static coord_index_t triangle_normal_axis(
            const ExactPoint& p1, const ExactPoint& p2, const ExactPoint& p3
        );

        /**
         * \brief Tests whether two adjacent triangles are coplanar
         * \details This is used to determine the facets that can be
         *  merged
         * \param[in] P1 , P2 , P3 , P4 the vertices of the triangles,
         *  as points with exact homogeneous coordinates. The two triangles
         *  are \p P1, \p P2, \p P3 and \p P2, \p P1, \p P4
         * \retval true if the two triangles are coplanar
         * \retval false otherwise
         * \details uses angle_tolerance specified to the constructor (if set
         *  to zero, uses exact computation)
         */
        bool triangles_are_coplanar(
            const ExactPoint& P1, const ExactPoint& P2,
            const ExactPoint& P3, const ExactPoint& P4
        ) const;

        /**
         * \brief Tests whether two edges are co-linear
         * \param[in] P1 , P2 , P3 the vertices of the two edges
         * \retval true if [P1,P2] and [P2,P3] are co-linear, and P2 is between
         *  P1 and p3
         * \retval false otherwise
         * \details uses angle_tolerance specified to the constructor (if set
         *  to zero, uses exact computation)
         */
        bool edges_are_colinear(
            const ExactPoint& P1, const ExactPoint& P2, const ExactPoint& P3
        ) const;
        
        
        
    public:
        ExactCDT2d CDT;

        /**
         * \brief Gets the number of coplanar facets
         * \return the number of coplanar facets present in the mesh
         */
        index_t nb_facets() {
            return facets_.size();
        }

        /**
         * \brief Marks the facets
         * \param[out] facet_is_marked on exit, set to 1 for facets present
         *  in the list of coplanar facets. Needs to be of size 
         *  mesh_.facets.nb().
         */
        void mark_facets(vector<index_t>& facet_is_marked) {
            for(index_t f: facets_) {
                facet_is_marked[f] = 1;
            }
        }

    private:
        MeshSurfaceIntersection& intersection_;
        Mesh& mesh_;
        double angle_tolerance_;
        index_t group_id_;
        Attribute<index_t> facet_group_;
        Attribute<bool> keep_vertex_;
        Attribute<bool> c_is_coplanar_;
        vector<bool>    f_visited_;
        vector<bool>    h_visited_;
        vector<bool>    v_visited_;
        vector<index_t> v_idx_;
        coord_index_t   u_;
        coord_index_t   v_;

        /***********************************************************/

        vector<index_t> vertices_;
        vector<index_t> facets_;        

        /**
         * \brief A 2d Incident Edge Lists data structure
         */
        class Halfedges {
        public:

            /**
             * \brief Halfedges constructor
             * \param[in] coplanar_facets a reference to the CoplanarFacets
             */
            Halfedges(
                CoplanarFacets& coplanar_facets
            ) : mesh_(coplanar_facets.mesh_) {
            }

            /**
             * \brief Initializes this Halfedges
             * \details Clears the list of halfedges and incident edge lists
             */
            void initialize() {
                // Use resize rather than assign so that we do not traverse
                // all the halfedges of the mesh_
                v_first_halfedge_.resize(mesh_.vertices.nb(), NO_INDEX);
                h_next_around_v_.resize(mesh_.facet_corners.nb(), NO_INDEX);
                // We only need to reset the halfedges of this set of coplanar
                // facets.
                for(index_t h: halfedges_) {
                    v_first_halfedge_[vertex(h,0)] = NO_INDEX;
                    h_next_around_v_[h] = NO_INDEX;
                }
                halfedges_.resize(0);
            }

            /**
             * \brief Gets a vertex of a halfedge
             * \param[in] h the halfedge
             * \param[in] dlv 0 for origin, 1 for destination, 2 for opposite
             * \return the vertex
             */
            index_t vertex(index_t h, index_t dlv) const {
                index_t f  = h/3;
                index_t lv = (h+dlv)%3;
                return mesh_.facets.vertex(f,lv);
            }

            /**
             * \brief Gets the incident facet
             * \param[in] h a halfedge
             * \return the facet incident to the halfedge
             */
            index_t facet(index_t h) const {
                return h/3;
            }

            /**
             * \brief Gets the opposite halfedge
             * \param[in] h a halfedge
             * \return the halfedge opposite to \p h, or NO_INDEX if there
             *  is no such halfedge
             */
            index_t alpha2(index_t h) const {
                index_t t1 = facet(h);
                index_t t2 = mesh_.facet_corners.adjacent_facet(h);
                if(t2 == NO_INDEX) {
                    return NO_INDEX;
                }
                for(index_t h2: mesh_.facets.corners(t2)) {
                    if(mesh_.facet_corners.adjacent_facet(h2) == t1) {
                        return h2;
                    }
                }
                geo_assert_not_reached;
            }

            /**
             * \brief Adds a halfedge
             * \param[in] h the halfedge
             */
            void add(index_t h) {
                halfedges_.push_back(h);
                index_t v1 = vertex(h,0);
                h_next_around_v_[h] = v_first_halfedge_[v1];
                v_first_halfedge_[v1] = h;
            }

            /**
             * \brief used by range-based for
             * \return an iterator to the first halfedge
             */
            vector<index_t>::const_iterator begin() const {
                return halfedges_.begin();
            }

            /**
             * \brief used by range-based for
             * \return an iterator to one position past the last halfedge
             */
            vector<index_t>::const_iterator end() const {
                return halfedges_.end();
            }

            /**
             * \brief Gets the first halfedge starting from a vertex
             * \param[in] v the vertex
             * \return the first halfedge starting from \p v
             */
            index_t vertex_first_halfedge(index_t v) const {
                return v_first_halfedge_[v];
            }

            /**
             * \brief Gets the next halfedge in the incident edge list
             * \param[in] h a halfedge
             * \return the next halfedge around the origin of \p h or
             *  NO_INDEX if there is no such halfedge
             */
            index_t next_around_vertex(index_t h) const {
                return h_next_around_v_[h];
            }

            /**
             * \brief Gets the number of halfedges around a vertex
             * \partam[in] v a vertex
             * \return the number of halfedges starting from \p v
             */
            index_t nb_halfedges_around_vertex(index_t v) const {
                index_t result = 0;
                for(
                    index_t h = vertex_first_halfedge(v);
                    h != NO_INDEX;
                    h = next_around_vertex(h)
                ) {
                    ++result;
                }
                return result;
            }

            /**
             * \brief Gets the next halfedge along a polyline
             * \param[in] h a halfedge
             * \return the halfedge on the same polyline as \p h starting
             *  from \p h destination or NO_INDEX if there is no such 
             *  halfedge. Polyline stops where it encounters a vertex that does 
             *  not have exactly 1 incident halfedge, that is, 
             *  where the halfedges graph is non-manifold.
             */
            index_t next_along_polyline(index_t h) const {
                index_t v2 = vertex(h,1);
                if(nb_halfedges_around_vertex(v2) != 1) {
                    return NO_INDEX;
                }
                return vertex_first_halfedge(v2);
            }
            
        private:
            Mesh& mesh_;
            vector<index_t> halfedges_;
            vector<index_t> v_first_halfedge_;
            vector<index_t> h_next_around_v_;
        } halfedges_;


        /**
         * \brief Organizes halfedges as a set of chains starting and ending
         *  at non-manifold vertices
         */
        class Polylines {
        public:

            /**
             * \brief Polylines constructor
             * \param[in] CF a reference to the CoplanarFacets
             */
            Polylines(CoplanarFacets& CF) : CF_(CF) {
            }

            /**
             * \brief Initializes this Polylines
             * \details Resets all the stored polylines
             */
            void initialize() {
                H_.resize(0);
                polyline_start_.resize(0);
                polyline_start_.push_back(0);
            }

            /**
             * \brief Gets the number of polylines
             * \return the number of polylines
             */
            index_t nb() const {
                return polyline_start_.size() - 1;
            }

            /**
             * \brief used by range-based for
             * \return a non-iterator corresponding to the first polyline index.
             */
            index_as_iterator begin() const {
                return index_as_iterator(0);
            }

            /**
             * \brief used by range-based for
             * \return a non-iterator to one position past the 
             *  last polyline index.
             */
            index_as_iterator end() const {
                return index_as_iterator(nb());
            }

            /**
             * \brief Gets the halfedges in a polyline
             * \param[in] polyline the polyline index
             * \return an iteratable sequence of halfedges
             */
            const_index_ptr_range halfedges(index_t polyline) const {
                geo_debug_assert(polyline < nb());
                return const_index_ptr_range(
                    H_, polyline_start_[polyline], polyline_start_[polyline+1]
                );
            }            

            /**
             * \brief Creates a new polyline
             */
            void begin_polyline() {
            }

            /**
             * \brief Finishes a polyline creation
             */
            void end_polyline() {
                polyline_start_.push_back(H_.size());
            }

            /**
             * \brief Adds a halfedge to the current polyline
             * \details Needs to be called between begin_polyline() and 
             *   end_polyline()
             * \param[in] h the halfedge to be added to the current polyline
             */
            void add_halfedge(index_t h) {
                H_.push_back(h);
            }

            /**
             * \brief Gets the first vertex of a polyline
             * \param[in] polyline a polyline index
             * \return the index of the first vertex of \p polyline
             */
            index_t first_vertex(index_t polyline) const {
                index_t h = H_[polyline_start_[polyline]];
                return CF_.halfedges_.vertex(h,0);
            }

            /**
             * \brief Gets the last vertex of a polyline
             * \details if the polyline is closed, last_vertex() is the same as
             *  first_vertex()
             * \param[in] polyline a polyline index
             * \return the index of the first vertex of \p polyline
             */
            index_t last_vertex(index_t polyline) const {
                index_t h = H_[polyline_start_[polyline+1]-1];
                return CF_.halfedges_.vertex(h,1);
            }

            /**
             * \brief Gets the predecessor of the first vertex
             * \param[in] polyline a polyline
             * \return if the polyline is closed, the predecessor 
             *  of the first vertex, otherwise NO_INDEX
             */
            index_t prev_first_vertex(index_t polyline) const {
                if(first_vertex(polyline) != last_vertex(polyline)) {
                    return NO_INDEX;
                }
                index_t h = H_[polyline_start_[polyline+1]-1];
                return CF_.halfedges_.vertex(h,0);
            }
            
        private:
            CoplanarFacets& CF_;
            vector<index_t> H_;
            vector<index_t> polyline_start_;
        } polylines_;
        
    };

    /**********************************************************************/    
    
}

#endif
