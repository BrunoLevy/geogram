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

#include <geogram/mesh/mesh_surface_intersection.h>
#include <geogram/mesh/triangle_intersection.h>
#include <geogram/mesh/mesh_AABB.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_fill_holes.h>
#include <geogram/mesh/index.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/delaunay/CDT_2d.h>
#include <geogram/numerics/predicates.h>
#include <geogram/numerics/expansion_nt.h>
#include <geogram/numerics/exact_geometry.h>
#include <geogram/basic/stopwatch.h>

#include <sstream>
#include <stack>

#ifdef GEO_COMPILER_CLANG
// I'm using long long 
#pragma GCC diagnostic ignored "-Wc++98-compat-pedantic"
// Keeping some debugging functions here
#pragma GCC diagnostic ignored "-Wunused-member-function"
#endif

namespace {
    using namespace GEO;

    /***********************************************************************/

    /**
     * \brief Removes all the triangles with their three vertices aligned
     */
    void remove_degenerate_triangles(Mesh& M) {
        vector<index_t> remove_f(M.facets.nb());
        for(index_t f: M.facets) {
            index_t v1 = M.facets.vertex(f,0);
            index_t v2 = M.facets.vertex(f,1);
            index_t v3 = M.facets.vertex(f,2);            
            const double* p1 = M.vertices.point_ptr(v1);
            const double* p2 = M.vertices.point_ptr(v2);
            const double* p3 = M.vertices.point_ptr(v3);
            remove_f[f] = PCK::aligned_3d(p1,p2,p3);
        }
        M.facets.delete_elements(remove_f);
    }

    /***********************************************************************/

    /**
     * \brief A mesh with some of its vertices stored with exact coordinates
     */
    class ExactMesh {
    public:

        ExactMesh(Mesh& M) :
            lock_(GEOGRAM_SPINLOCK_INIT),
            mesh_(M),
            vertex_to_exact_point_(M.vertices.attributes(), "exact_point") {
            for(index_t v: mesh_.vertices) {
                vertex_to_exact_point_[v] = nullptr;
            }
            // We need to copy the initial mesh, because MeshInTriangle needs
            // to access it in parallel thread, and without a copy, the internal
            // arrays of the mesh can be modified whenever there is a reallocation.
            // Without copying, we would need to insert many locks (each time the
            // mesh is accessed). 
            mesh_copy_.copy(M);
        }

        ~ExactMesh() {
            vertex_to_exact_point_.destroy();
        }

        void lock() {
            Process::acquire_spinlock(lock_);
        }

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
        vec3HE exact_vertex(index_t v) const {
            geo_debug_assert(v < mesh_.vertices.nb());
            const vec3HE* p = vertex_to_exact_point_[v];
            if(p != nullptr) {
                return *p;
            }
            const double* xyz = mesh_.vertices.point_ptr(v);
            return vec3HE(xyz[0], xyz[1], xyz[2], 1.0);
        }

        /**
         * \brief Finds or creates a vertex in the mesh, by exact coordinates
         * \details If there is already a vertex with coordinates \p p, then
         *  the existing vertex is returned, else a new vertex is constructed.
         *  Note that only the vertices created by find_or_create_vertex() can
         *  be returned as existing vertices. Mesh vertices stored as double-
         *  precision coordinates are not retrieved by this function.
         * \param[in] p the exact coordinates of a point
         * \return the index of a mesh vertex with \p p as coordinates
         */
        index_t find_or_create_exact_vertex(const vec3HE& p) {
            std::map<vec3HE,index_t,vec3HELexicoCompare>::iterator it;
            bool inserted;
            std::tie(it, inserted) = exact_point_to_vertex_.insert(
                std::make_pair(p,index_t(-1))
            );
            if(!inserted) {
                return it->second;
            }
            double w = p.w.estimate();
            vec3 p_inexact(
                p.x.estimate()/w,
                p.y.estimate()/w,
                p.z.estimate()/w
            );
            index_t v = mesh_.vertices.create_vertex(p_inexact.data());
            it->second = v;
            vertex_to_exact_point_[v] = &(it->first);
            return v;
        }

        Mesh& mesh() {
            return mesh_;
        }

        const Mesh& mesh() const {
            return mesh_;
        }

        const Mesh& mesh_copy() const {
            return mesh_copy_;
        }
        
    private:
        Process::spinlock lock_;
        Mesh& mesh_;
        Mesh mesh_copy_;
        Attribute<const vec3HE*> vertex_to_exact_point_;
        std::map<vec3HE,index_t,vec3HELexicoCompare> exact_point_to_vertex_;
    };

    /***********************************************************************/
    
    /**
     * \brief Meshes a single triangle with the constraints that come from
     *  the intersections with the other triangles.
     * \details Inherits CDTBase2d (constrained Delaunay triangulation), and
     *  redefines orient2d(), incircle2d() and create_intersection() using
     *  vectors with homogeneous coordinates stored as arithmetic expansions
     *  (vec2HE).
     */
    class MeshInTriangle : public CDTBase2d {
    public:

        /***************************************************************/

        /**
         * \brief An edge of the mesh.
         * \details It represents the constraints to be used by the
         *  constrained triangulation to remesh the facet. It contains
         *  a list of vertices coming from the intersection with other
         *  constrained edge. Makes a maximum use of the combinatorial
         *  information to reduce the complexity (degree) of the constructed
         *  coordinates as much as possible.
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
            index_t v1,v2; // The two extremities of the mesh
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
            Vertex(MeshInTriangle* M, const vec3HE& point_exact_in) {
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
            void print(std::ostream& out=std::cerr) const {
                if(sym.f1 != index_t(-1)) {
                    out << " ( ";
                    out << sym.f1;
                    out << region_to_string(sym.R1).substr(2);
                }
                if(sym.f2 != index_t(-1)) {
                    out << " /\\ ";
                    out << sym.f2;
                    out << region_to_string(sym.R2).substr(2);
                }
                if(sym.f1 != index_t(-1)) {
                    out << " ) ";
                }
            }

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
            vec3HE compute_geometry() {
                
                // Case 1: f1 vertex
                if(region_dim(sym.R1) == 0) {
                    index_t lv = index_t(sym.R1) - index_t(T1_RGN_P0);
                    geo_assert(lv < 3);
                    mesh_vertex_index = mesh().facets.vertex(sym.f1,lv);
                    vec3 p = mit->mesh_vertex(mesh_vertex_index);
                    return vec3HE(p);
                }

                geo_assert(sym.f1 != index_t(-1) && sym.f2 != index_t(-1));

                // Case 2: f2 vertex
                if(region_dim(sym.R2) == 0) {
                    index_t lv = index_t(sym.R2) - index_t(T2_RGN_P0);
                    geo_assert(lv < 3);
                    mesh_vertex_index = mesh().facets.vertex(sym.f2, lv);
                    vec3 p = mit->mesh_vertex(mesh_vertex_index);
                    return vec3HE(p);
                }

                // case 3: f1 /\ f2 edge in 3D or f1 edge /\ f2 edge in 3D
                if(
                    (region_dim(sym.R1) == 2 || region_dim(sym.R1) == 1) &&
                     region_dim(sym.R2) == 1
                ) {
                    vec3 p1 = mit->mesh_facet_vertex(sym.f1, 0);
                    vec3 p2 = mit->mesh_facet_vertex(sym.f1, 1);
                    vec3 p3 = mit->mesh_facet_vertex(sym.f1, 2);
                    index_t e = index_t(sym.R2)-index_t(T2_RGN_E0);
                    geo_debug_assert(e<3);
                    vec3 q1 = mit->mesh_facet_vertex(sym.f2, (e+1)%3);
                    vec3 q2 = mit->mesh_facet_vertex(sym.f2, (e+2)%3);
                    
                    bool seg_seg_two_D = (
                        region_dim(sym.R1) == 1 &&
                        PCK::orient_3d(p1,p2,p3,q1) == ZERO &&
                        PCK::orient_3d(p1,p2,p3,q2) == ZERO) ;

                    if(!seg_seg_two_D) {
                        return plane_line_intersection(p1,p2,p3,q1,q2);
                    }
                }

                // case 4: f1 edge /\ f2
                if(region_dim(sym.R1) == 1 && region_dim(sym.R2) == 2) {
                    index_t e = index_t(sym.R1)-index_t(T1_RGN_E0);
                    geo_debug_assert(e<3);
                    vec3 p1 = mit->mesh_facet_vertex(sym.f2,0);
                    vec3 p2 = mit->mesh_facet_vertex(sym.f2,1);
                    vec3 p3 = mit->mesh_facet_vertex(sym.f2,2);
                    vec3 q1 = mit->mesh_facet_vertex(sym.f1, (e+1)%3);
                    vec3 q2 = mit->mesh_facet_vertex(sym.f1, (e+2)%3);
                    return plane_line_intersection(p1,p2,p3,q1,q2);
                }

                // case 5: f1 edge /\ f2 edge in 2D
                if(region_dim(sym.R1) == 1 && region_dim(sym.R2) == 1) {
                    index_t e1 = index_t(sym.R1) - index_t(T1_RGN_E0);
                    geo_debug_assert(e1 < 3);
                    index_t e2 = index_t(sym.R2) - index_t(T2_RGN_E0);
                    geo_debug_assert(e2 < 3);
                    vec2 p1 = mit->mesh_facet_vertex_UV(sym.f1, (e1+1)%3);
                    vec2 p2 = mit->mesh_facet_vertex_UV(sym.f1, (e1+2)%3);
                    vec2 q1 = mit->mesh_facet_vertex_UV(sym.f2, (e2+1)%3);
                    vec2 q2 = mit->mesh_facet_vertex_UV(sym.f2, (e2+2)%3);
                    vec3 P1 = mit->mesh_facet_vertex(sym.f1, (e1+1)%3);
                    vec3 P2 = mit->mesh_facet_vertex(sym.f1, (e1+2)%3);
                    vec2E D1 = make_vec2<vec2E>(p1,p2);
                    vec2E D2 = make_vec2<vec2E>(q1,q2);
                    expansion_nt d = det(D1,D2);
                    geo_debug_assert(d.sign() != ZERO);
                    vec2E AO = make_vec2<vec2E>(p1,q1);
                    rational_nt t(det(AO,D2),d);
                    return mix(t,P1,P2);
                }

                // Normally we enumerated all possible cases
                geo_assert_not_reached;
            }

            /**
             * \brief Optimizes exact numbers in generated
             *  points and computes approximate coordinates.
             */
            void init_geometry(const vec3HE& P) {
                point_exact = P;
                point_exact.optimize();

                // Compute the lifting coordinate h = (u2+v2)/w2
                // Keep exact computation as long as possible and convert
                // to double only in the end.
                // Use low-level API (expansions allocated on stack)
                const expansion& u2 =
                    expansion_square(point_exact[mit->u_].rep());
                const expansion& v2 =
                    expansion_square(point_exact[mit->v_].rep());
                const expansion& l2 = expansion_sum(u2,v2);
                const expansion& w  = expansion_square(point_exact.w.rep());
                h_approx = l2.estimate() / w.estimate();
            }

        public:
            MeshInTriangle* mit;
            vec3HE point_exact; // Exact homogeneous coords using expansions
            double h_approx;    // Lifting coordinate for incircle
            Type type;          // MESH_VERTEX, PRIMARY_ISECT or SECONDARY_ISECT
            index_t mesh_vertex_index; // Global mesh vertex index once created
            struct {                   // Symbolic information - tri-tri isect
                index_t f1,f2;         //   global facet indices in mesh
                TriangleRegion R1,R2;  //   triangle regions
            } sym;
        };

        /***************************************************************/
        
        MeshInTriangle(ExactMesh& EM) :
            exact_mesh_(EM),
            mesh_(EM.mesh_copy()),
            f1_(index_t(-1)),
            approx_incircle_(false) {
            // Since we use lifted coordinates stored in doubles,
            // we need to activate additional checks for Delaunayization.
            CDTBase2d::exact_incircle_ = false;
        }

        const Mesh& mesh() const {
            return mesh_;
        }

        /**
         * \brief If Delaunay is set, use approximated incircle
         *  predicate (default: use exact incircle)
         */
        void set_approx_incircle(bool x) {
            approx_incircle_ = x;
        }
        
        void save_constraints(const std::string& filename) {
            Mesh M;
            get_constraints(M);
            mesh_save(M,filename);
        }
        
        void begin_facet(index_t f) {
            f1_ = f;
            latest_f2_ = index_t(-1);
            latest_f2_count_ = 0;
            
            vec3 p1 = mesh_facet_vertex(f,0);
            vec3 p2 = mesh_facet_vertex(f,1);
            vec3 p3 = mesh_facet_vertex(f,2);
            
            f1_normal_axis_ = ::GEO::Geom::triangle_normal_axis(
                p1,p2,p3
            );
            
            u_ = coord_index_t((f1_normal_axis_ + 1) % 3);
            v_ = coord_index_t((f1_normal_axis_ + 2) % 3);
            
            for(index_t lv=0; lv<3; ++lv) {
                vertex_.push_back(Vertex(this, f, lv));
            }

            CDTBase2d::create_enclosing_triangle(0,1,2);

            edges_.push_back(Edge(1,2));
            edges_.push_back(Edge(2,0));
            edges_.push_back(Edge(0,1));
            
            has_planar_isect_ = false;
        }
        
        index_t add_vertex(index_t f2, TriangleRegion R1, TriangleRegion R2) {
            geo_debug_assert(f1_ != index_t(-1));

            // If the same f2 comes more than twice, then
            // we got a planar facet /\ facet intersection
            // (and it is good to know it, see get_constraints())
            if(f2 != index_t(-1) && f2 == latest_f2_) {
                ++latest_f2_count_;
                if(latest_f2_count_ > 2) {
                    has_planar_isect_ = true;
                }
            } else {
                latest_f2_ = f2;
                latest_f2_count_ = 0;
            }

            // If vertex is a macro-vertex, return it directly.
            if(region_dim(R1) == 0) {
                return index_t(R1);
            }

            // Create the vertex
            vertex_.push_back(Vertex(this, f1_, f2, R1, R2));
            
            // Insert it into the triangulation
            index_t v = CDTBase2d::insert(vertex_.size()-1);
            
            // If it was an existing vertex, return the existing vertex
            if(vertex_.size() > CDTBase2d::nv()) {
                vertex_.pop_back();
            }
            return v;
        }

        void add_edge(
            index_t f2,
            TriangleRegion AR1, TriangleRegion AR2,
            TriangleRegion BR1, TriangleRegion BR2
        ) {
            index_t v1 = add_vertex(f2, AR1, AR2);
            index_t v2 = add_vertex(f2, BR1, BR2);

            // If both extremities are on the same edge of f1,
            // we do not add the edge, because it will be generated
            // when remeshing the edge of f1
            if(region_dim(regions_convex_hull(AR1,BR1)) == 1) {
                return;
            }

            // Generate also the combinatorial information of the edge,
            // that indicates whether both extremities are on the same
            // edge of f2 (useful later to compute the intersections)
            edges_.push_back(Edge(v1,v2,f2,regions_convex_hull(AR2,BR2)));

            // Constraints will be added to the triangulation during commit()
        }

        void end_facet() {
            commit();
            clear();
        }

    protected:

        void commit() {
            
            for(const Edge& E: edges_) {
                CDTBase2d::insert_constraint(E.v1, E.v2);
            }

            // Protect global mesh from concurrent accesses
            exact_mesh_.lock();
            
            // Create vertices and facets in target mesh
            for(index_t i=0; i<vertex_.size(); ++i) {
                // Vertex already exists in this MeshInTriangle
                if(vertex_[i].mesh_vertex_index != index_t(-1)) {
                    continue;
                }
                vertex_[i].mesh_vertex_index =
                    exact_mesh_.find_or_create_exact_vertex(
                        vertex_[i].point_exact
                    );
            }
            
            // Create facets in target mesh
            for(index_t t=0; t<CDTBase2d::nT(); ++t) {
                index_t i = CDTBase2d::Tv(t,0);
                index_t j = CDTBase2d::Tv(t,1);
                index_t k = CDTBase2d::Tv(t,2);
                i = vertex_[i].mesh_vertex_index;
                j = vertex_[j].mesh_vertex_index;
                k = vertex_[k].mesh_vertex_index;                    
                index_t new_t = exact_mesh_.mesh().facets.create_triangle(i,j,k);
                // Copy all attributes from initial facet
                exact_mesh_.mesh().facets.attributes().copy_item(new_t, f1_);
            }

            // We are done with modification in the mesh
            exact_mesh_.unlock();
        }
        
        void get_constraints(Mesh& M, bool with_edges=true) const {
            if(M.vertices.nb() == 0) {
                M.vertices.set_dimension(2);
                for(index_t v=0; v<vertex_.size(); ++v) {
                    vec2 p = vertex_[v].get_UV_approx();
                    M.vertices.create_vertex(p.data());
                }
            }
            if(with_edges && M.edges.nb() == 0) {
                index_t i=0;
                for(const Edge& E: edges_) {
                    M.edges.create_edge(E.v1, E.v2); 
                    ++i;
                }
            }
        }
        
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
        
        void clear() override {
            vertex_.resize(0);
            edges_.resize(0);
            f1_ = index_t(-1);
            CDTBase2d::clear();
        }

        void log_err() const {
            std::cerr << "Houston, we got a problem (while remeshing facet "
                      << f1_ << "):" << std::endl;
        }
        
    protected:

        /********************** CDTBase2d overrides ***********************/
        
        Sign orient2d(index_t vx1,index_t vx2,index_t vx3) const override {
            return PCK::orient_2d_projected(
                vertex_[vx1].point_exact,
                vertex_[vx2].point_exact,
                vertex_[vx3].point_exact,
                f1_normal_axis_
            );
        }

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
        ) const override {

            if(approx_incircle_) {
                return PCK::orient_2dlifted_SOS(
                    vertex_[v1].get_UV_approx().data(),
                    vertex_[v2].get_UV_approx().data(),
                    vertex_[v3].get_UV_approx().data(),
                    vertex_[v4].get_UV_approx().data(),
                    vertex_[v1].h_approx,
                    vertex_[v2].h_approx,
                    vertex_[v3].h_approx,
                    vertex_[v4].h_approx
                );
            }

            // Exact version (using approximate lifted coordinates,
            // but its OK as soon as it always the same for the same vertex).
            return PCK::orient_2dlifted_SOS_projected(
                vertex_[v1].point_exact,
                vertex_[v2].point_exact,
                vertex_[v3].point_exact,
                vertex_[v4].point_exact,
                vertex_[v1].h_approx,
                vertex_[v2].h_approx,
                vertex_[v3].h_approx,
                vertex_[v4].h_approx,
                f1_normal_axis_
            );
        }

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
        ) override {
            geo_argused(i);
            geo_argused(j);
            geo_argused(k);
            geo_argused(l);
            vec3HE I;
            get_edge_edge_intersection(e1,e2,I);
            vertex_.push_back(Vertex(this,I));
            index_t x = vertex_.size()-1;
            CDTBase2d::v2T_.push_back(index_t(-1));
            geo_debug_assert(x == CDTBase2d::nv_);
            ++CDTBase2d::nv_;
            return x;
        }

        void get_edge_edge_intersection(
            index_t e1, index_t e2, vec3HE& I
        ) const {
            index_t f1 = f1_;
            index_t f2 = edges_[e1].sym.f2; 
            index_t f3 = edges_[e2].sym.f2; 
            
            geo_assert(f1 != index_t(-1));
            geo_assert(f2 != index_t(-1));
            geo_assert(f3 != index_t(-1));                        
            
            vec3 P[9] = {
                mesh_facet_vertex(f1,0), mesh_facet_vertex(f1,1),
                mesh_facet_vertex(f1,2),
                mesh_facet_vertex(f2,0), mesh_facet_vertex(f2,1),
                mesh_facet_vertex(f2,2),
                mesh_facet_vertex(f3,0), mesh_facet_vertex(f3,1),
                mesh_facet_vertex(f3,2)
            };

            if(!get_three_planes_intersection(
                    I,
                    P[0], P[1], P[2],
                    P[3], P[4], P[5],
                    P[6], P[7], P[8]
            )) {
                get_edge_edge_intersection_2D(e1,e2,I);
                return;
            }
        }             

        void get_edge_edge_intersection_2D(
            index_t e1, index_t e2, vec3HE& I
        ) const {
            const Edge& E1 = edges_[e1];
            const Edge& E2 = edges_[e2];

            if(
                region_dim(E1.sym.R2) == 1 &&
                region_dim(E2.sym.R2) == 1
            ) {
                index_t le1 = index_t(E1.sym.R2)-index_t(T2_RGN_E0);
                index_t le2 = index_t(E2.sym.R2)-index_t(T2_RGN_E0);
                geo_assert(le1 < 3);
                geo_assert(le2 < 3);

                vec2 p1_uv = mesh_facet_vertex_UV(E1.sym.f2, (le1+1)%3);
                vec2 p2_uv = mesh_facet_vertex_UV(E1.sym.f2, (le1+2)%3);
                vec2 q1_uv = mesh_facet_vertex_UV(E2.sym.f2, (le2+1)%3);
                vec2 q2_uv = mesh_facet_vertex_UV(E2.sym.f2, (le2+2)%3);
                vec2E C1 = make_vec2<vec2E>(p1_uv, p2_uv);
                vec2E C2 = make_vec2<vec2E>(q2_uv, q1_uv);
                vec2E B  = make_vec2<vec2E>(p1_uv, q1_uv);
                
                expansion_nt d = det(C1,C2);
                geo_debug_assert(d.sign() != ZERO);
                rational_nt t(det(B,C2),d);
                I = mix(
                    t,
                    mesh_facet_vertex(E1.sym.f2,(le1+1)%3),
                    mesh_facet_vertex(E1.sym.f2,(le1+2)%3)
                );
            } else {
                geo_assert(
                    region_dim(E1.sym.R2) == 1 || region_dim(E2.sym.R2) == 1
                );
                index_t f1 = E1.sym.f2;
                TriangleRegion R1 = E1.sym.R2;
                index_t f2 = E2.sym.f2;
                TriangleRegion R2 = E2.sym.R2;
                if(region_dim(R1) == 1) {
                    std::swap(f1,f2);
                    std::swap(R1,R2);
                }

                index_t e = index_t(R2) - index_t(T2_RGN_E0);
                geo_assert(e < 3);

                I = plane_line_intersection(
                    mesh_facet_vertex(f1,0),
                    mesh_facet_vertex(f1,1),
                    mesh_facet_vertex(f1,2),
                    mesh_facet_vertex(f2,(e+1)%3),
                    mesh_facet_vertex(f2,(e+2)%3)
                );
            }
        }

    public:
        void save(const std::string& filename) const override {
            Mesh M;
            M.vertices.set_dimension(2);
            for(index_t v=0; v<CDTBase2d::nv(); ++v) {
                vec2 p = vertex_[v].get_UV_approx();
                M.vertices.create_vertex(p.data());
            }
            for(index_t t=0; t<CDTBase2d::nT(); ++t) {
                M.facets.create_triangle(
                    CDTBase2d::Tv(t,0),
                    CDTBase2d::Tv(t,1),
                    CDTBase2d::Tv(t,2)
                );
            }

            Attribute<double> tex_coord;
            tex_coord.create_vector_attribute(
                M.facet_corners.attributes(), "tex_coord", 2
            );
            static double triangle_tex[3][2] = {
                {0.0, 0.0},
                {1.0, 0.0},
                {0.0, 1.0}
            };
            for(index_t c: M.facet_corners) {
                tex_coord[2*c]   = triangle_tex[c%3][0];
                tex_coord[2*c+1] = triangle_tex[c%3][1];
            }
            
            mesh_save(M, filename);
        }
        
    private:
        ExactMesh& exact_mesh_;
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
        bool approx_incircle_;
    };

    /***********************************************************************/

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

    /**
     * \brief Computes the intersection between two mesh triangular facets
     * \details This function is just a wrapper around triangles_intersections()
     *  for Mesh facets.
     * \param[in] M the mesh
     * \param[in] f1 , f2 the two facets
     * \param[out] I a vector of triangle intersections
     * \retval true if there was an intersection
     * \retval false otherwise
     */
    bool mesh_facets_intersect(
        Mesh& M, index_t f1, index_t f2, vector<TriangleIsect>& I
    ) {
        geo_debug_assert(M.facets.nb_vertices(f1) == 3);
        geo_debug_assert(M.facets.nb_vertices(f2) == 3);        
        vec3 p1(M.vertices.point_ptr(M.facets.vertex(f1,0)));
        vec3 p2(M.vertices.point_ptr(M.facets.vertex(f1,1)));
        vec3 p3(M.vertices.point_ptr(M.facets.vertex(f1,2)));
        vec3 q1(M.vertices.point_ptr(M.facets.vertex(f2,0)));
        vec3 q2(M.vertices.point_ptr(M.facets.vertex(f2,1)));
        vec3 q3(M.vertices.point_ptr(M.facets.vertex(f2,2)));
        return triangles_intersections(p1,p2,p3,q1,q2,q3,I);
    }

    void mesh_intersect_surface_compute_arrangement(
        Mesh& M, const MeshSurfaceIntersectionParams& params
    ) {

        // Step 1: Preparation
        // -------------------
        
        // Set symbolic perturbation mode to lexicographic order
        // on point coordinates instead of point indices only,
        // Needed to get compatible triangulations on coplanar faces
        // (example, cubes that touch on a facet).
        PCK::SOSMode SOS_bkp = PCK::get_SOS_mode();
        PCK::set_SOS_mode(PCK::SOS_LEXICO);

        // Exact arithmetics is exact ... until we encounter
        // underflows/overflows (and underflows can happen quite
        // often !!) -> I want to detect them.
        bool FPE_bkp = Process::FPE_enabled();
        Process::enable_FPE(params.debug_enable_FPE);

        // Step 2: Get intersections
        // -------------------------
        
        vector<IsectInfo> intersections;
        {
            Stopwatch W("Detect isect");
            MeshFacetsAABB AABB(M,!params.debug_do_not_order_facets);
            vector<std::pair<index_t, index_t> > FF;

            // Get candidate pairs of intersecting facets
            AABB.compute_facet_bbox_intersections(
                [&](index_t f1, index_t f2) {
                    // Needed (maybe I should change that in AABB class)
                    if(f1 == f2) {
                        return;
                    }
                    geo_assert(f1 < f2);

                    // Optionally skip facet pairs that
                    // share a vertex or an edge
                    if(
                        !params.detect_intersecting_neighbors && (
                            (M.facets.find_adjacent(f1,f2)  != index_t(-1)) ||
                            (M.facets.find_common_vertex(f1,f2) != index_t(-1))
                        )
                    ) {
                        return;
                    }

                    FF.push_back(std::make_pair(f1,f2));
                }
            );

            // Compute facet-facet intersections in parallel
            Process::spinlock lock = GEOGRAM_SPINLOCK_INIT;
            parallel_for_slice(
                0,FF.size(), [&](index_t b, index_t e) {
                    vector<TriangleIsect> I;
                    for(index_t i=b; i<e; ++i){
                        index_t f1 = FF[i].first;
                        index_t f2 = FF[i].second;
                        
                        if(mesh_facets_intersect(M, f1, f2, I)) {

                            Process::acquire_spinlock(lock);
                            
                            if(I.size() > 2) {
                                // Coplanar intersection: to generate the edges,
                                // test validity of all possible
                                // pairs of vertices.
                                for(index_t i1=0; i1< I.size(); ++i1) {
                                    for(index_t i2=0; i2<i1; ++i2) {
                                        IsectInfo II = {
                                            f1, f2,
                                            I[i1].first, I[i1].second,
                                            I[i2].first, I[i2].second
                                        };

                                        // Valid edges are the ones where both
                                        // extremities are on the same edge
                                        // of f1 or on the same edge of f2
                                        // (note: it is a *combinatorial*
                                        // convex hull).
                                    
                                        TriangleRegion AB1=regions_convex_hull(
                                            II.A_rgn_f1,II.B_rgn_f1
                                        );
                                        
                                        TriangleRegion AB2=regions_convex_hull(
                                            II.A_rgn_f2, II.B_rgn_f2
                                        );
                                        
                                        if(
                                            region_dim(AB1) == 1 ||
                                            region_dim(AB2) == 1
                                        ) {

                                            intersections.push_back(II);
                                            II.flip();
                                            intersections.push_back(II);
                                        } 
                                    }
                                }
                            } else {
                                // Intersection is either a segment
                                // or a vertex of f2.
                                TriangleRegion A_rgn_f1 = I[0].first;
                                TriangleRegion A_rgn_f2 = I[0].second;
                                
                                TriangleRegion B_rgn_f1 = A_rgn_f1;
                                TriangleRegion B_rgn_f2 = A_rgn_f2;
                                
                                if(I.size() == 2) {
                                    B_rgn_f1 = I[1].first;
                                    B_rgn_f2 = I[1].second;
                                }
                                
                                IsectInfo II = {
                                    f1, f2,
                                    A_rgn_f1, A_rgn_f2,
                                    B_rgn_f1, B_rgn_f2
                                };
                                intersections.push_back(II);
                                II.flip();
                                intersections.push_back(II);
                            }
                            Process::release_spinlock(lock);
                        }
                    }
                }
            );
        }

        
        // Step 3: Remesh intersected triangles
        // ------------------------------------
        {
            Stopwatch W("Remesh isect");

            // Sort intersections by f1, so that all intersections between f1
            // and another facet appear as a contiguous sequence.
            std::sort(
                intersections.begin(), intersections.end(),
                [](const IsectInfo& a, const IsectInfo& b) -> bool {
                    return (a.f1 < b.f1) ? true  :
                        (a.f1 > b.f1) ? false :
                        (a.f2 < b.f2) ;
                }
            );

            // Now iterate on all intersections, and identify
            // the [b,e[ intervals that correspond to the same f1 facet.
            // Get starting indices of intersections in same facet.
            vector<index_t> start;
            {
                index_t b=0;            
                while(b < intersections.size()) {
                    start.push_back(b);
                    index_t e = b;
                    while(
                        e < intersections.size() &&
                        intersections[e].f1 == intersections[b].f1
                    ) {
                        ++e;
                    }
                    b = e;
                }
                start.push_back(intersections.size());
            }

            
            // The exact mesh, that keeps the exact intersection
            // coordinates
            ExactMesh EM(M);

            // Slower if activated. Probably comes from the lock
            // on new_expansion_on_heap(), massively used when
            // creating the points in exact precision...
            //   First thing will be to rewrite the predicates by
            // directly accessing the coordinates in the computed points
            // rather than copying to a vec2HE...
            #define TRIANGULATE_IN_PARALLEL
            
            #ifdef TRIANGULATE_IN_PARALLEL
               parallel_for_slice( 0,start.size()-1, [&](index_t k1, index_t k2) {
            #else
               index_t k1 = 0;
               index_t k2 = start.size()-1;
            #endif                   
            
            MeshInTriangle MIT(EM);
            MIT.set_delaunay(params.delaunay);
            MIT.set_approx_incircle(params.approx_incircle);

            index_t nf = M.facets.nb();
            
            for(index_t k=k1; k<k2; ++k) {
                index_t b = start[k];
                index_t e = start[k+1];
//#ifndef TRIANGULATE_IN_PARALLEL                
                if(params.verbose) {
                    std::cerr << "Isects in " << intersections[b].f1
                              << " / " << nf                    
                              << "    : " << (e-b)
                              << std::endl;
                }
//#endif                
                MIT.begin_facet(intersections[b].f1);
                for(index_t i=b; i<e; ++i) {
                    const IsectInfo& II = intersections[i];
                    
                    // Each IsectIfo is either an individual vertex
                    // or a segment with two vertices.
                    // Each vertex is represented combinatorially.
                    // The MeshInTriangle knows how to compute the
                    // geometry from the combinatorial information.
                    
                    if(II.is_point()) {
                        MIT.add_vertex(
                            II.f2,
                            II.A_rgn_f1, II.A_rgn_f2
                        );
                    } else {
                        MIT.add_edge(
                            II.f2, 
                            II.A_rgn_f1, II.A_rgn_f2,
                            II.B_rgn_f1, II.B_rgn_f2
                        );
                    }
                }
                MIT.end_facet();
            }
        #ifdef TRIANGULATE_IN_PARALLEL
           });
        #endif
        }
        
        // Step 4: Epilogue
        // ----------------
        
        vector<index_t> has_intersections(M.facets.nb(), 0);
        for(const IsectInfo& II: intersections) {
            has_intersections[II.f1] = 1;
            has_intersections[II.f2] = 1;
        }
        
        M.facets.delete_elements(has_intersections);
        M.facets.connect();

        if(!FPE_bkp) {
            Process::enable_FPE(false);
        }
        PCK::set_SOS_mode(SOS_bkp);
    }

    /*****************************************************************/

    /**
     * \brief Enumerates the connected components in a facet attribute
     * \param[in] M a reference to the mesh
     * \param[in] attribute the name of the facet attribute
     * \return the number of found connected components
     */
    index_t get_surface_connected_components(
        Mesh& M, const std::string& attribute = "chart"
    ) {
        Attribute<index_t> chart(M.facets.attributes(), attribute);
        for(index_t f: M.facets) {
            chart[f] = index_t(-1);
        }
        std::stack<index_t> S;
        index_t cur_chart = 0;
        for(index_t f: M.facets) {
            if(chart[f] == index_t(-1)) {
                chart[f] = cur_chart;
                S.push(f);
                while(!S.empty()) {
                    index_t g = S.top();
                    S.pop();
                    for(
                        index_t le=0;
                        le<M.facets.nb_vertices(g); ++le
                    ) {
                        index_t h = M.facets.adjacent(g,le);
                        if(h != index_t(-1) && chart[h] == index_t(-1)) {
                            chart[h] = cur_chart;
                            S.push(h);
                        }
                    }
                }
                ++cur_chart;
            }
        }
        return cur_chart;
    }
}




namespace GEO {
    
    void mesh_intersect_surface(
        Mesh& M, const MeshSurfaceIntersectionParams& params
    ) {
        if(!M.facets.are_simplices()) {
            tessellate_facets(M,3);
        }

        Attribute<index_t> operand_bit;
        operand_bit.bind_if_is_defined(M.facets.attributes(), "operand_bit");
        if(!operand_bit.is_bound()) {
            get_surface_connected_components(M,"operand_bit");
            operand_bit.bind(M.facets.attributes(), "operand_bit");
            for(index_t f: M.facets) {
                operand_bit[f] =
                    params.per_component_ids ? (1u << operand_bit[f]) : 1;
            }
        }
        
        if(params.pre_detect_duplicated_vertices) {
            remove_degenerate_triangles(M);
            mesh_colocate_vertices_no_check(M);
        }
        
        if(params.pre_detect_duplicated_facets) {        
            mesh_remove_bad_facets_no_check(M);
        }

        
        const double SCALING = double(1ull << 20);
        const double INV_SCALING = 1.0/SCALING;

        // Pre-scale everything by 2^20 to avoid underflows
        // (note: this just adds 20 to the exponents of all
        //  coordinates).
        {
            double* p = M.vertices.point_ptr(0);
            index_t N = M.vertices.nb() *
                        M.vertices.dimension();
            for(index_t i=0; i<N; ++i) {
                p[i] *= SCALING;
            }
        }
        
        mesh_intersect_surface_compute_arrangement(M, params);

        if(params.post_connect_facets) {
            /*
            mesh_colocate_vertices_no_check(M);
            mesh_remove_bad_facets_no_check(M);
            mesh_connect_and_reorient_facets_no_check(M);
            */
            mesh_repair(
                M,
                GEO::MeshRepairMode(
                    GEO::MESH_REPAIR_COLOCATE | GEO::MESH_REPAIR_DUP_F
                ),
                0.0
            );
        }

        // Scale-back everything
        {
            double* p = M.vertices.point_ptr(0);
            index_t N = M.vertices.nb() *
                        M.vertices.dimension();
            for(index_t i=0; i<N; ++i) {
                p[i] *= INV_SCALING;
            }
        }
    }
}

namespace {
    using namespace GEO;

    /**
     * \brief A simple parser for boolean expressions
     * \details 
     *  - Variables: A..Z or x0..x31 
     *  - and:        '&' or '*'
     *  - or:         '|' or '+'
     *  - xor:        '^'
     *  - difference: '-'
     */
    class BooleanExprParser {
    public:
        BooleanExprParser(
            const std::string& expr
        ) : expr_(expr) {
        }

        bool eval(index_t x) {
            x_   = x;
            ptr_ = expr_.begin();
            return parse_or();
        }

    protected:

        bool parse_or() {
            bool left = parse_and();
            while(
                cur_char() == '|' ||
                cur_char() == '^' ||
                cur_char() == '+' ||
                cur_char() == '-'
            ) {
                char op = cur_char();
                next_char();
                bool right = parse_and();
                left = (op == '-') ? (left && !right) :
                       (op == '^') ? (left ^   right) :
                                     (left ||  right) ;
            }
            return left;
        }

        bool parse_and() {
            bool left = parse_factor();
            while(cur_char() == '&' || cur_char() == '*') {
                next_char();
                bool right = parse_factor();
                left = left && right;
            }
            return left;
        }

        bool parse_factor() {
            if(cur_char() == '!' || cur_char() == '~' || cur_char() == '-') {
                next_char();
                return !parse_factor();
            }
            if(cur_char() == '(') {
                next_char();
                bool result = parse_or();
                if(cur_char() != ')') {
                    throw std::logic_error(
                        std::string("Unmatched parenthesis: ")+cur_char()
                    );
                }
                next_char();
                return result;
            }
            if((cur_char() >= 'A' && cur_char() <= 'Z') || cur_char() == 'x') {
                return parse_variable();
            }
            throw std::logic_error("Syntax error");
        }

        bool parse_variable() {
            int bit = 0;
            if(cur_char() >= 'A' && cur_char() <= 'Z') {
                bit = int(cur_char()) - int('A');
                next_char();
            } else {
                if(cur_char() != 'x') {
                    throw std::logic_error("Syntax error in variable");
                }
                next_char();
                while(cur_char() >= '0' && cur_char() <= '9') {
                    bit = bit * 10 + (int(cur_char()) - '0');
                    next_char();
                }
            }
            if(bit > 31) {
                throw std::logic_error("Bit larger than 31");
            }
            return ((x_ & (index_t(1u) << bit)) != 0);
        }

        char cur_char() const {
            return *ptr_;
        }
        
        void next_char() {
            if(ptr_ == expr_.end()) {
                throw std::logic_error("Unexpected end of string");
            }
            ptr_++;
        }
        
    private:
        std::string expr_;
        std::string::iterator ptr_;
        index_t x_;
    };

    /**
     * \brief Gets the number of bits set in
     *  a 32 bits integer
     * \param[in] x the integer
     * \return the number of "ones" in the 
     *  binary form of x
     */
    inline index_t nb_bits_set(index_t x) {
        index_t result = 0;
        for(index_t i=0; i<32; ++i) {
            result += (x&1);
            x = x >> 1;
        }
        return result;
    }
}

namespace {
    using namespace GEO;

    // TODO: something less hacky...
    bool facet_on_border(MeshFacetsAABB& AABB, index_t f) {
        vec3 g = Geom::mesh_facet_center(*(AABB.mesh()),f);
        for(index_t i=0; i<1000; ++i) {
            vec3 D(
                Numeric::random_float64(),
                Numeric::random_float64(),
                Numeric::random_float64()
            );
            
            if(!AABB.ray_intersection(
                   Ray(g,D), Numeric::max_float64(), f
            )) {
                return true;
            }
            if(!AABB.ray_intersection(
                   Ray(g,-D), Numeric::max_float64(), f
            )) {
                return true;
            }
        }
        return false;
    }
    
    void mesh_classify_union(
        Mesh& M, 
        const std::string& attribute,
        bool reorder
    ) {
        MeshFacetsAABB AABB(M,reorder);
        index_t nb_charts = get_surface_connected_components(M);        
        Attribute<index_t> chart(M.facets.attributes(), "chart");
        Attribute<bool> selection;
        vector<index_t> delete_f;
        if(attribute != "") {
            selection.bind(M.facets.attributes(), attribute);            
        } else {
            delete_f.assign(M.facets.nb(), 0);
        }

        vector<index_t> chart_facet(nb_charts, index_t(-1));
        for(index_t f: M.facets) {
            index_t c = chart[f];
            if(chart_facet[c] == index_t(-1)) {
                bool f_is_selected = facet_on_border(AABB,f);
                chart_facet[c] = f;
                if(selection.is_bound()) {
                    selection[f] = f_is_selected;
                } else {
                    delete_f[f] = !f_is_selected;
                }
            } else {
                if(selection.is_bound()) {
                    selection[f] = selection[chart_facet[c]];
                } else {
                    delete_f[f] = delete_f[chart_facet[c]];
                }
            }
        } 
        if(!selection.is_bound()) {
            M.facets.delete_elements(delete_f);
        }
        mesh_repair(
            M,
            GEO::MeshRepairMode(
                GEO::MESH_REPAIR_COLOCATE | GEO::MESH_REPAIR_DUP_F
            ),
            0.0
        );
    }
    
}

namespace GEO {

    void mesh_classify_intersections(
        Mesh& M, std::function<bool(index_t)> eqn,
        const std::string& attribute,
        bool reorder
    ) {
        MeshFacetsAABB AABB(M,reorder);
        index_t nb_charts = get_surface_connected_components(M);
        Attribute<index_t> chart(M.facets.attributes(), "chart");
        Attribute<index_t> operand_bit(M.facets.attributes(), "operand_bit");
        Attribute<bool> selection;
        vector<index_t> delete_f;
        if(attribute != "") {
            selection.bind(M.facets.attributes(), attribute);            
        } else {
            delete_f.assign(M.facets.nb(), 0);
        }

        vector<index_t> chart_facet(nb_charts, index_t(-1));
        try {
            for(index_t f: M.facets) {
                index_t c = chart[f];
                if(chart_facet[c] == index_t(-1)) {
                    bool f_is_selected = false;
                    chart_facet[c] = f;
                    vec3 g = Geom::mesh_facet_center(M,f);
                    // Picking f's normal is not a good idea,
                    // because for industrial parts it will
                    // encounter many degenerate ray/triangle
                    // intersections.
                    // TODO: we need to detect them and launch
                    // another ray whenever the ray intersects
                    // the surface on a vertex or on an edge.
                    vec3 D(
                        Numeric::random_float64(),
                        Numeric::random_float64(),
                        Numeric::random_float64()
                    );
                    index_t parity = 0;
                    AABB.ray_all_intersections(
                        Ray(g,D),
                        [&](const MeshFacetsAABB::Intersection & I) {
                            if(I.f != f) {
                                parity = parity ^ operand_bit[I.f];
                            }
                        }
                    );
                    if(nb_bits_set(operand_bit[f]) == 1) {
                        // Facet f is on the boundary of the result if
                        // crossing f changes the result of eqn,
                        // in other words, if eqn gives a different
                        // result with and without f's object bit set
                        f_is_selected =
                            eqn(parity |  operand_bit[f]) !=
                            eqn(parity & ~operand_bit[f]) ;
                    } else {
                        // Now if f is on several objects (that is, has
                        // several bit sets), then we determine whether
                        // it is on the boundary of the result by raytracing
                        // in two different directions, and seeing if eqn
                        // gives a different result. 
                        index_t parity2 = 0;
                        AABB.ray_all_intersections(
                            Ray(g,-D),
                            [&](const MeshFacetsAABB::Intersection & I) {
                                if(I.f != f) {
                                    parity2 = parity2 ^ operand_bit[I.f];
                                }
                            }
                        );
                        f_is_selected = (eqn(parity) != eqn(parity2));
                    }
                    if(selection.is_bound()) {
                        selection[f] = f_is_selected;
                    } else {
                        delete_f[f] = !f_is_selected;
                    }
                } else {
                    if(selection.is_bound()) {
                        selection[f] = selection[chart_facet[c]];
                    } else {
                        delete_f[f] = delete_f[chart_facet[c]];
                    }
                }
            }
        } catch(const std::logic_error& e) {
            Logger::err("Classify") << "Error while parsing expression:"
                                    << e.what()
                                    << std::endl;
            return;
        }
        if(!selection.is_bound()) {
            M.facets.delete_elements(delete_f);
        }
        mesh_repair(
            M,
            GEO::MeshRepairMode(
                GEO::MESH_REPAIR_COLOCATE | GEO::MESH_REPAIR_DUP_F
            ),
            0.0
        );
    }

    void mesh_classify_intersections(
        Mesh& M, const std::string& expr,
        const std::string& attribute, bool reorder
    ) {
        BooleanExprParser eqn(expr);
        index_t operand_all_bits;
        {
            index_t max_operand_bit = 0;
            Attribute<index_t> operand_bit;
            operand_bit.bind_if_is_defined(M.facets.attributes(),"operand_bit");
            if(!operand_bit.is_bound()) {
                Logger::err("Classify")
                    << "operand_bit: no such facet attribute"
                    << std::endl;
                return;
            }
            for(index_t f: M.facets) {
                max_operand_bit = std::max(max_operand_bit, operand_bit[f]);
            }
            operand_all_bits = (max_operand_bit << 1)-1;
        }
        
        if(expr == "union") {
            mesh_classify_union(M, attribute, reorder);
            return;
        }
        
        try {
            mesh_classify_intersections(
                M,
                [&](index_t x)->bool {
                    return
                        (expr == "union")        ? (x != 0)                :
                        (expr == "intersection") ? (x == operand_all_bits) : 
                        eqn.eval(x);
                },
                attribute,
                reorder
            );
        } catch(const std::logic_error& e) {
            Logger::err("Classify") << "Error while parsing expression:"
                                    << e.what()
                                    << std::endl;
            return;
        }
    }    
}
