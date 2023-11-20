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

        typedef MeshSurfaceIntersection::ExactPoint ExactPoint;
        typedef MeshSurfaceIntersection::ExactCoord ExactCoord;
        typedef MeshSurfaceIntersection::ExactVec2 ExactVec2;
        typedef MeshSurfaceIntersection::ExactVec2H ExactVec2H;
        typedef MeshSurfaceIntersection::ExactRational ExactRational;
        
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
#ifndef INTERSECTIONS_USE_EXACT_NT            
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
     * \brief Constrained Delaunay Triangulation with vertices that are
     *  exact points.
     * \details Inherits CDTBase2d (constrained Delaunay triangulation), and
     *  redefines orient2d(), incircle2d() and create_intersection() using
     *  vectors with homogeneous coordinates stored as arithmetic expansions
     *  (vec2HE) or arbitrary-precision floating point numbers (vec2HEx) if
     *  compiled with Tessael's geogramplus extension package.
     */
    class ExactCDT2d : public CDTBase2d {
    public:
        typedef MeshSurfaceIntersection::ExactVec2H ExactPoint;
        typedef MeshSurfaceIntersection::ExactCoord ExactCoord;
        typedef MeshSurfaceIntersection::ExactVec2 ExactVec2;
        typedef MeshSurfaceIntersection::ExactVec2H ExactVec2H;
        typedef MeshSurfaceIntersection::ExactRational ExactRational;

        ExactCDT2d();
        ~ExactCDT2d() override;
        
        /**
         * \copydoc CDTBase2d::clear()
         */
        void clear() override;

        /**
         * \brief Inserts a point
         * \param[in] p the point to be inserted
         * \param[in] hint a triangle not too far away from the point to
         *  be inserted
         * \return the index of the created point. Duplicated points are
         *  detected (and then the index of the existing point is returned)
         */
        index_t insert(
            const ExactPoint& p, index_t id, index_t hint = index_t(-1)
        );

        void insert_constraint(index_t v1, index_t v2, index_t operand_bits) {
            constraints_.push_back(bindex(v1,v2,bindex::KEEP_ORDER));
            cnstr_operand_bits_.push_back(operand_bits);
            CDTBase2d::insert_constraint(v1,v2);
        }
        
        /**
         * \brief Creates a first large enclosing quad
         * \param[in] p1 , p2 , p3 , p4 the four vertices of the quad
         * \details The quad needs to be convex. 
         *  create_enclosing_rectangle() or create_enclosing_quad()  
         *  need to be called before anything else
         */
        void create_enclosing_quad(
            const ExactPoint& p1, const ExactPoint& p2,
            const ExactPoint& p3, const ExactPoint& p4
        );


        /**
         * \brief Creates a first large enclosing rectangle
         * \param[in] x1 , y1 , x2 , y2 rectangle bounds
         * \details create_enclosing_triangle(), create_enclosing_rectangle() 
         *  or create_enclosing_quad() need to be called before anything else
         */
        void create_enclosing_rectangle(
            double x1, double y1, double x2, double y2
        ) {
            create_enclosing_quad(
                ExactPoint(vec2(x1,y1)),
                ExactPoint(vec2(x2,y1)),
                ExactPoint(vec2(x2,y2)),
                ExactPoint(vec2(x1,y2))
            );
        }

        /**
         * \brief Gets a point by vertex index
         * \param[in] v vertex index
         * \return the point at index \p v
         */
        const ExactPoint& vertex_point(index_t v) const {
            geo_debug_assert(v < nv());
            return point_[v];
        }

        /**
         * \brief Gets a vertex id by index
         * \param[in] v vertex index
         * \return the point at index \p v
         */
        index_t vertex_id(index_t v) const {
            geo_debug_assert(v < nv());
            return id_[v];
        }

        /**
         * \brief Sets a vertex id by index
         * \param[in] v vertex index
         * \param[in] id vertex id
         */
        void set_vertex_id(index_t v, index_t id) {
            geo_debug_assert(v < nv());
            id_[v] = id;
        }

        void classify_triangles(const std::string& boolean_expression);
        
        void save(const std::string& filename) const override;
        
    protected:
        void add_point(const ExactPoint& p, index_t id = index_t(-1));
        void begin_insert_transaction() override;
        void commit_insert_transaction() override;
        void rollback_insert_transaction() override;
        
        /**
         * \copydoc CDTBase2d::orient_2d()
         */
        Sign orient2d(index_t i, index_t j, index_t k) const override;

        /**
         * \copydoc CDTBase2d::incircle()
         */
        Sign incircle(index_t i,index_t j,index_t k,index_t l) const override;

        /**
         * \copydoc CDTBase2d::create_intersection()
         */
        index_t create_intersection(
            index_t E1, index_t i, index_t j,
            index_t E2, index_t k, index_t l
        ) override;
        
    protected:
        vector<ExactPoint> point_;
#ifndef INTERSECTIONS_USE_EXACT_NT            
        vector<double> length_;
#endif        
        vector<index_t> id_;
        vector<index_t> cnstr_operand_bits_;
        vector<index_t> facet_inclusion_bits_;
        mutable std::map<trindex, Sign> pred_cache_;
        bool use_pred_cache_insert_buffer_;
        mutable std::vector< std::pair<trindex, Sign> > pred_cache_insert_buffer_;
        vector<bindex> constraints_;
    };
    
    /**********************************************************************/

    /**
     * \brief Detects and retriangulates a set of coplanar facets for
     *  MeshSurfaceIntersection.
     */
    class CoplanarFacets {
    public:
        static constexpr index_t NO_INDEX = index_t(-1);
        static constexpr index_t NON_MANIFOLD = index_t(-2);
        typedef MeshSurfaceIntersection::ExactPoint ExactPoint;
        typedef MeshSurfaceIntersection::ExactVec3 ExactVec3;
        typedef MeshSurfaceIntersection::ExactVec2H ExactVec2H;

        /**
         * \brief Constructs a CoplanarFacets object associated with a
         *   MeshSurfaceIntersection
         * \details No set of facets is identified. One needs to call get().
         * \param[in] I a reference to the MeshSurfaceIntersection
         */
        CoplanarFacets(MeshSurfaceIntersection& I);

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
         * \brief Marks the vertices that need to be kept in the simplified facets.
         * \details A vertex is kept if it is incident to at least two non-colinear
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
         */
        static bool triangles_are_coplanar(
            const ExactPoint& P1, const ExactPoint& P2,
            const ExactPoint& P3, const ExactPoint& P4
        );

    public:
        vector<index_t> facets;
        vector<index_t> vertices;
        ExactCDT2d      CDT;
        
    private:
        MeshSurfaceIntersection& intersection_;
        Mesh& mesh_;
        index_t group_id_;
        Attribute<index_t> facet_group_;
        Attribute<bool> keep_vertex_;
        vector<index_t> v_prev_;
        vector<index_t> v_next_;
        vector<bool>    f_visited_;
        vector<bool>    v_visited_;
        vector<index_t> v_idx_;
    };

    /**********************************************************************/    
    
}

#endif
