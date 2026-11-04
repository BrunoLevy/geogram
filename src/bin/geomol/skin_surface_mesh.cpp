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

#include "skin_surface_mesh.h"
#include <geogram/basic/geometry.h>
#include <geogram/basic/memory.h>
#include <geogram/basic/attributes.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/stopwatch.h>
#include <geogram/basic/assert.h>

#include <unordered_map>
#include <algorithm>
#include <cmath>

/*
 * Notations (see the paper):
 *
 *  P      : the set of weighted points (atoms), p_hat = (p, P) is the ball
 *           of center p and squared radius P.
 *  s      : the shrink factor.
 *  delta_X: the Delaunay simplex of a subset X of P (|X| in 1..4).
 *  nu_X   : the dual power (Voronoi) cell of delta_X.
 *  c(X)   : the center of X, aff(delta_X) inter aff(nu_X). It is the
 *           radical point of X (same power distance to all balls of X).
 *  mu_X   : the mixed cell (1-s) delta_X (+) s nu_X. It is of type l=|X|-1.
 *           Inside mu_X the skin surface is the quadric I_X = 0, with
 *             I_X(x) = -|u|^2/(1-s) + |w|^2/s - W_c
 *           where x - c(X) = u + w, u parallel to delta_X, w orthogonal
 *           to delta_X, and W_c = P_i - |c(X)-p_i|^2 is the weight of the
 *           weighted point of aff(X) centered at c(X). The body of the
 *           skin surface is { I_X <= 0 }.
 *  mu_{X,X'} with X subset of X': the polyhedral cell
 *           (1-s) delta_X (+) s nu_X', a face of both mu_X and mu_X'.
 *           Its dimension is 3 - |X'| + |X|.
 *  ad(X)  : the anchor point of delta_X, i.e. the point of delta_X closest
 *           to c(X). sd(X) is the face of delta_X that contains ad(X) in its
 *           relative interior (Lemma 15: ad(X) = ad(sd(X))).
 *  av(nu) : the anchor point of a face nu of the power diagram, i.e. the
 *           point of nu closest to c(G), G being the set of points whose
 *           cells contain nu. sv(nu) is the face of nu that contains av(nu)
 *           in its relative interior (Lemma 16: av(nu) = av(sv(nu))).
 *
 *  The anchor point of the polyhedral cell mu_{X,nu} = (1-s)delta_X (+) s nu
 *  is (Lemma 14)
 *     (1-s) ad(X) + s av(nu)
 *  and it is identified by the pair (sd(X), sv(nu)): if the anchor point
 *  lies on the boundary of mu_{X,nu} then it coincides with the anchor
 *  point of the face mu_{sd(X),sv(nu)} and the vertex is "collapsed" as
 *  described in Section 3.2 of the paper. The collapsed vertices generate
 *  degenerate tetrahedra (with repeated vertices) that are simply skipped.
 *
 *  Infinite power cells. Rather than inserting additional points, the
 *  power diagram is clipped by a bounding box B, and the mixed complex of
 *  the clipped diagram, mu_X = (1-s) delta_X (+) s (nu_X inter B), is
 *  used. This preserves the structure needed by the algorithm: the box
 *  cuts of a type 0 cell are planar (circular) facets; the box cuts of a
 *  type 1 cell mu_e are (1-s) delta_e (+) s sigma for segments sigma of
 *  the bisector plane of e, that is, rectangles with sides parallel and
 *  orthogonal to the symmetry axis (hyperbolic facets); the clipped nu_f
 *  of a type 2 cell is still a segment along the normal of f. Lemmas
 *  13-16 generalize verbatim (the anchor of a clipped face is the point
 *  closest to the projection of c(G) onto its affine hull). B is chosen
 *  large enough so that the Voronoi component z of every point
 *  x = (1-s) y + s z of the skin surface lies inside B: the body is
 *  contained in conv(P) (+) ball(sqrt(Pmax)), hence
 *  |z - y| = |x - y|/s <= (diam(P) + sqrt(Pmax))/s. Hence no facet of the
 *  clipped complex that lies on the boundary of B meets the surface, and
 *  the mixed cells that are clipped away contain no part of the surface.
 */

namespace GEO {
  namespace skin_internal {

    /**
     * \brief Local vertex indices of the facets of a tetrahedron.
     * \details Facet lf is opposite to vertex lf, and the tetrahedron
     *  (lf, f[0], f[1], f[2]) has the same orientation as the original
     *  one (same convention as geomol's PowerDiagram::tet_facet_lv()).
     */
    constexpr index_t tet_facet_vertex[4][3] = {
        {1, 2, 3},
        {0, 3, 2},
        {3, 0, 1},
        {1, 0, 2}
    };

    /**
     * \brief Local vertex indices of the edges of a tetrahedron.
     */
    constexpr index_t tet_edge_vertex[6][2] = {
        {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}
    };

    /**
     * \brief A Delaunay simplex (vertex, edge, triangle or tetrahedron).
     */
    struct Simplex {
        index_t dim; // 0: vertex, 1: edge, 2: triangle, 3: tetrahedron
        index_t id;
        Numeric::uint64 key() const {
            return (Numeric::uint64(dim) << 32) | Numeric::uint64(id);
        }
    };

    inline Simplex SV(index_t v) { return Simplex{0, v}; }
    inline Simplex SE(index_t e) { return Simplex{1, e}; }
    inline Simplex SF(index_t f) { return Simplex{2, f}; }
    inline Simplex ST(index_t t) { return Simplex{3, t}; }

    /**
     * \brief Key that identifies a vertex of the tetrahedral complex
     *  (a pair of Delaunay simplices (sd(X), sv(X')), see above).
     */
    struct VertexKey {
        Numeric::uint64 a;
        Numeric::uint64 b;
        bool operator==(const VertexKey& rhs) const {
            return a == rhs.a && b == rhs.b;
        }
    };

    struct VertexKeyHash {
        size_t operator()(const VertexKey& k) const noexcept {
            Numeric::uint64 h = k.a * 0x9E3779B97F4A7C15ull;
            h ^= (k.b + 0x7F4A7C159E3779B9ull) + (h << 6) + (h >> 2);
            return size_t(h);
        }
    };

    inline Numeric::uint64 edge_key(index_t v1, index_t v2) {
        if(v1 > v2) {
            std::swap(v1, v2);
        }
        return (Numeric::uint64(v1) << 32) | Numeric::uint64(v2);
    }

    /**
     * \brief Solves the 3x3 linear system [c0 c1 c2] x = rhs, where
     *  c0, c1, c2 are the columns of the matrix.
     * \retval true if the system could be solved.
     * \retval false if the matrix is singular.
     */
    inline bool solve3_columns(
        const vec3& c0, const vec3& c1, const vec3& c2,
        const vec3& rhs, vec3& x
    ) {
        double det = dot(c0, cross(c1, c2));
        if(det == 0.0 || !std::isfinite(det)) {
            return false;
        }
        x.x = dot(rhs, cross(c1, c2)) / det;
        x.y = dot(c0, cross(rhs, c2)) / det;
        x.z = dot(c0, cross(c1, rhs)) / det;
        return std::isfinite(x.x) && std::isfinite(x.y) && std::isfinite(x.z);
    }

    /**
     * \brief Solves the 3x3 linear system M x = rhs, where r0, r1, r2
     *  are the rows of M.
     */
    inline bool solve3_rows(
        const vec3& r0, const vec3& r1, const vec3& r2,
        const vec3& rhs, vec3& x
    ) {
        return solve3_columns(
            vec3(r0.x, r1.x, r2.x),
            vec3(r0.y, r1.y, r2.y),
            vec3(r0.z, r1.z, r2.z),
            rhs, x
        );
    }

    /**
     * \brief The quadric I_X that defines the skin surface inside a
     *  mixed cell mu_X (Equation (4) of the paper).
     */
    struct Quadric {
        index_t ell;  // type of the mixed cell (= dimension of delta_X)
        vec3 c;       // center c(X)
        vec3 axis;    // ell=1: unit direction of the edge (symmetry axis)
                      // ell=2: unit normal of the triangle
        double W;     // W_c (the R^2 of Equation (4))
        double s;     // shrink factor

        /**
         * \brief Evaluates the quadratic form of I_X at a vector y = x-c.
         */
        double form(const vec3& y) const {
            switch(ell) {
            case 0: {
                return length2(y) / s;
            }
            case 1: {
                double a = dot(y, axis);
                return -(a*a) / (1.0-s) + (length2(y) - a*a) / s;
            }
            case 2: {
                double a = dot(y, axis);
                return -(length2(y) - a*a) / (1.0-s) + (a*a) / s;
            }
            default: {
                return -length2(y) / (1.0-s);
            }
            }
        }

        /**
         * \brief Evaluates I_X at a point x.
         * \details The body of the skin surface is { I_X <= 0 }.
         */
        double eval(const vec3& x) const {
            return form(x - c) - W;
        }

        /**
         * \brief Evaluates the gradient of I_X at a point x.
         */
        vec3 gradient(const vec3& x) const {
            vec3 y = x - c;
            switch(ell) {
            case 0: {
                return (2.0/s)*y;
            }
            case 1: {
                double a = dot(y, axis);
                return (2.0/s)*(y - a*axis) - (2.0/(1.0-s))*(a*axis);
            }
            case 2: {
                double a = dot(y, axis);
                return (2.0/s)*(a*axis) - (2.0/(1.0-s))*(y - a*axis);
            }
            default: {
                return (-2.0/(1.0-s))*y;
            }
            }
        }

        /**
         * \brief Projects a point onto the quadric, along its gradient
         *  if the gradient line meets the quadric (choosing the root
         *  closest to a reference point), else by Newton iterations
         *  started from the reference point.
         * \retval true on success
         */
        bool project(vec3& x, const vec3& ref) const {
            vec3 g = gradient(x);
            double gl2 = length2(g);
            if(gl2 < 1e-300) {
                return false;
            }
            // I(x + lambda g) = A lambda^2 + B lambda + C
            double A = form(g);
            double C = eval(x);
            double B = form((x - c) + g) - form(x - c) - A; // 2 bilinear
            double lambda;
            bool ok = false;
            if(std::fabs(A) < 1e-14*std::fabs(B)) {
                if(B != 0.0) {
                    lambda = -C/B;
                    ok = true;
                }
            } else {
                double disc = B*B - 4.0*A*C;
                if(disc >= 0.0) {
                    double sq = std::sqrt(disc);
                    double l1 = (-B + sq)/(2.0*A);
                    double l2 = (-B - sq)/(2.0*A);
                    // choose the root closest to the reference point
                    lambda = (
                        distance2(x + l1*g, ref) < distance2(x + l2*g, ref)
                    ) ? l1 : l2;
                    ok = true;
                }
            }
            if(ok) {
                x = x + lambda*g;
                return true;
            }
            // Newton iterations (the gradient line misses the quadric,
            // this happens with hyperboloids)
            vec3 y = ref;
            double scale = std::fabs(W) + length2(x - c);
            for(index_t iter=0; iter<20; ++iter) {
                double f = eval(y);
                if(std::fabs(f) < 1e-14*scale) {
                    x = y;
                    return true;
                }
                vec3 gg = gradient(y);
                double gg2 = length2(gg);
                if(gg2 < 1e-300) {
                    return false;
                }
                y = y - (f/gg2)*gg;
            }
            if(std::fabs(eval(y)) < 1e-9*scale) {
                x = y;
                return true;
            }
            return false;
        }
    };

  }
}

/*************************************************************************/

namespace GEO {

    using namespace skin_internal;

    /**
     * \brief Internal class that does all the work.
     */
    class SkinSurfaceMesherImpl {
    public:

        SkinSurfaceMesherImpl(
            const Delaunay& D,
            const double* points,
            const double* weights,
            index_t nb_atoms,
            double shrink,
            bool verbose
        ) : D_(D),
            points_(points),
            weights_(weights),
            nb_atoms_(nb_atoms),
            s_(shrink),
            verbose_(verbose) {
            nb_degenerate_ = 0;
        }

        /**
         * \brief Runs the whole pipeline.
         */
        bool run(Mesh& skin, Mesh* tet_complex) {
            {
                Stopwatch W("skin:topo", verbose_);
                build_tets();
                build_triangles();
                if(!build_edges()) {
                    return false;
                }
                build_vertex_edges();
                compute_delaunay_anchors();
            }
            {
                Stopwatch W("skin:clip", verbose_);
                // The clipped power diagram is built with a bounding box
                // large enough to contain all the surface. In the (rare)
                // near-degenerate cases where the combinatorial decisions
                // are inconsistent, the box is enlarged and the
                // construction is restarted.
                bool ok = false;
                double scale = 1.0;
                for(index_t trial=0; trial<8 && !ok; ++trial) {
                    compute_box(scale);
                    ok = build_clipped_power_diagram();
                    if(!ok) {
                        if(verbose_) {
                            Logger::warn("skin")
                                << "Inconsistent clipped power diagram, "
                                << "enlarging the bounding box"
                                << std::endl;
                        }
                        scale *= 1.7;
                    }
                }
                if(!ok) {
                    Logger::err("skin")
                        << "Could not build the clipped power diagram"
                        << std::endl;
                    return false;
                }
                compute_voronoi_anchors();
            }
            {
                Stopwatch W("skin:complex", verbose_);
                build_tetrahedral_complex();
            }
            {
                Stopwatch W("skin:extract", verbose_);
                extract_surface();
                output_mesh(skin);
            }
            if(tet_complex != nullptr) {
                get_tetrahedral_complex(*tet_complex);
            }
            if(verbose_) {
                Logger::out("skin")
                    << "Clipped power diagram: " << vf_.size() << " faces"
                    << std::endl;
                Logger::out("skin")
                    << "Tetrahedral complex: "
                    << cv_pos_.size() << " vertices, "
                    << nb_complex_tets() << " tets, "
                    << nb_degenerate_ << " degenerate tets skipped"
                    << std::endl;
                Logger::out("skin")
                    << "Skin mesh: "
                    << skin.vertices.nb() << " vertices, "
                    << skin.facets.nb() << " triangles"
                    << std::endl;
            }
            return true;
        }

        index_t nb_complex_vertices() const {
            return cv_pos_.size();
        }

        /**
         * \brief Counts the triangles of the current mesh that violate
         *  the isotopy condition of Lemma 23.
         */
        index_t count_isotopy_violations() {
            return check_isotopy();
        }

        /**
         * \brief Subdivides the current mesh (each triangle into n^2
         *  triangles) and copies it into \p skin.
         */
        bool subdivide(Mesh& skin, index_t n) {
            if(rt_v_.size() == 0) {
                return false;
            }
            Stopwatch W("skin:subdivide", verbose_);
            if(!subdivide(n)) {
                return false;
            }
            output_mesh(skin);
            return true;
        }

        /**
         * \brief Refines the current mesh and copies it into \p skin.
         */
        bool refine(Mesh& skin, const SkinRefineParams& params) {
            if(rt_v_.size() == 0) {
                return false;
            }
            Stopwatch W("skin:refine", verbose_);
            refine_params_ = params;
            if(!refine()) {
                return false;
            }
            output_mesh(skin);
            return true;
        }

        index_t nb_complex_tets() const {
            return ct_v_.size() / 4;
        }

        index_t nb_degenerate_tets() const {
            return nb_degenerate_;
        }

    private:

        /*****************************************************************/
        /** Access to the Delaunay triangulation and to the points       */
        /*****************************************************************/

        index_t nb_tets() const {
            return D_.nb_cells();
        }

        index_t nb_vertices() const {
            return D_.nb_vertices();
        }

        index_t tet_vertex(index_t t, index_t lv) const {
            return index_t(D_.cell_vertex(t, lv));
        }

        index_t tet_adjacent(index_t t, index_t lf) const {
            return index_t(D_.cell_adjacent(t, lf));
        }

        index_t find_tet_vertex(index_t t, index_t v) const {
            for(index_t lv=0; lv<4; ++lv) {
                if(tet_vertex(t, lv) == v) {
                    return lv;
                }
            }
            geo_assert_not_reached;
        }

        index_t find_tet_adjacent(index_t t1, index_t t2) const {
            for(index_t lf=0; lf<4; ++lf) {
                if(tet_adjacent(t1, lf) == t2) {
                    return lf;
                }
            }
            geo_assert_not_reached;
        }

        vec3 point(index_t v) const {
            geo_debug_assert(v < nb_vertices());
            return vec3(points_[3*v], points_[3*v+1], points_[3*v+2]);
        }

        double weight(index_t v) const {
            geo_debug_assert(v < nb_vertices());
            return weights_[v];
        }

        bool is_atom(index_t v) const {
            return v < nb_atoms_;
        }

        /*****************************************************************/
        /** Combinatorics: tets, triangles, edges                        */
        /*****************************************************************/

        /**
         * \brief Computes for each tetrahedron whether it is finite and
         *  its radical center (the vertex of the power diagram).
         */
        void build_tets() {
            index_t nt = nb_tets();
            tet_finite_.assign(nt, false);
            tet_c_.assign(nt, vec3(0.0, 0.0, 0.0));
            for(index_t t=0; t<nt; ++t) {
                index_t v0 = tet_vertex(t,0);
                index_t v1 = tet_vertex(t,1);
                index_t v2 = tet_vertex(t,2);
                index_t v3 = tet_vertex(t,3);
                if(
                    v0 == NO_INDEX || v1 == NO_INDEX ||
                    v2 == NO_INDEX || v3 == NO_INDEX
                ) {
                    continue;
                }
                tet_finite_[t] = true;
                tet_c_[t] = radical_point(v0, v1, v2, v3);
            }
        }

        /**
         * \brief Creates the triangles (one per pair of adjacent
         *  tetrahedra, at least one of them being finite).
         */
        void build_triangles() {
            index_t nt = nb_tets();
            tri_of_tf_.assign(4*nt, NO_INDEX);
            for(index_t t=0; t<nt; ++t) {
                if(!tet_finite_[t]) {
                    continue;
                }
                for(index_t lf=0; lf<4; ++lf) {
                    if(tri_of_tf_[4*t+lf] != NO_INDEX) {
                        continue;
                    }
                    index_t t2 = tet_adjacent(t, lf);
                    index_t f = tri_v_.size()/3;
                    for(index_t k=0; k<3; ++k) {
                        tri_v_.push_back(
                            tet_vertex(t, tet_facet_vertex[lf][k])
                        );
                    }
                    tri_t_.push_back(t);
                    tri_t_.push_back(t2);
                    tri_of_tf_[4*t+lf] = f;
                    if(t2 != NO_INDEX) {
                        index_t lf2 = find_tet_adjacent(t2, t);
                        tri_of_tf_[4*t2+lf2] = f;
                    }
                }
            }
            tri_e_.assign(tri_v_.size(), NO_INDEX);
        }

        /**
         * \brief Creates the edges and the circular lists of tetrahedra
         *  (and triangles) around each edge.
         * \retval true on success.
         * \retval false if the triangulation is inconsistent.
         */
        bool build_edges() {
            std::unordered_map<Numeric::uint64, index_t> edge_map;
            index_t nt = nb_tets();
            edge_tets_ptr_.push_back(0);
            for(index_t t=0; t<nt; ++t) {
                if(!tet_finite_[t]) {
                    continue;
                }
                for(index_t le=0; le<6; ++le) {
                    index_t v0 = tet_vertex(t, tet_edge_vertex[le][0]);
                    index_t v1 = tet_vertex(t, tet_edge_vertex[le][1]);
                    Numeric::uint64 key = edge_key(v0, v1);
                    if(edge_map.find(key) != edge_map.end()) {
                        continue;
                    }
                    index_t e = edge_v_.size()/2;
                    edge_map[key] = e;
                    edge_v_.push_back(std::min(v0,v1));
                    edge_v_.push_back(std::max(v0,v1));
                    bool open = false;
                    if(!walk_around_edge(t, v0, v1, open)) {
                        Logger::err("skin")
                            << "Inconsistent triangulation (edge walk)"
                            << std::endl;
                        return false;
                    }
                    edge_open_.push_back(open);
                    edge_tets_ptr_.push_back(edge_tets_.size());
                }
            }
            // Edges of the triangles
            for(index_t f=0; f<tri_v_.size()/3; ++f) {
                for(index_t k=0; k<3; ++k) {
                    index_t v0 = tri_v_[3*f+k];
                    index_t v1 = tri_v_[3*f+(k+1)%3];
                    auto it = edge_map.find(edge_key(v0,v1));
                    geo_assert(it != edge_map.end());
                    tri_e_[3*f+k] = it->second;
                }
            }
            return true;
        }

        /**
         * \brief Turns around an edge and appends the tetrahedra and
         *  the triangles incident to the edge to edge_tets_ and edge_tris_.
         * \details The k-th triangle is between the k-th and the
         *  (k+1)-th (modulo m) tetrahedra.
         * \param[in] t0 a finite tetrahedron incident to the edge
         * \param[in] v0 , v1 the extremities of the edge
         * \param[out] open true if one of the tetrahedra around the edge
         *  is infinite (unbounded power facet)
         */
        bool walk_around_edge(
            index_t t0, index_t v0, index_t v1, bool& open
        ) {
            open = false;
            index_t t = t0;
            // exit_v: the vertex of t opposite to the facet through which
            // we leave t.
            index_t exit_v = NO_INDEX;
            {
                for(index_t lv=0; lv<4; ++lv) {
                    index_t v = tet_vertex(t,lv);
                    if(v != v0 && v != v1) {
                        exit_v = v;
                        break;
                    }
                }
            }
            index_t count = 0;
            do {
                if(!tet_finite_[t]) {
                    open = true;
                }
                index_t lx = find_tet_vertex(t, exit_v);
                edge_tets_.push_back(t);
                edge_tris_.push_back(tri_of_tf_[4*t+lx]);
                index_t t_next = tet_adjacent(t, lx);
                // The next exit vertex is the vertex of t that is neither
                // v0, v1 nor exit_v (it is shared with t_next).
                index_t next_exit = NO_INDEX;
                for(index_t lv=0; lv<4; ++lv) {
                    index_t v = tet_vertex(t,lv);
                    if(v != v0 && v != v1 && lv != lx) {
                        next_exit = v;
                        break;
                    }
                }
                exit_v = next_exit;
                t = t_next;
                if(t == NO_INDEX) {
                    open = true;
                    return true;
                }
                ++count;
                if(count > 100000) {
                    return false;
                }
            } while(t != t0);
            return true;
        }

        /**
         * \brief Computes for each vertex the list of incident edges
         *  (compressed row storage).
         */
        void build_vertex_edges() {
            index_t nv = nb_vertices();
            index_t ne = edge_v_.size()/2;
            v_edges_ptr_.assign(nv+1, 0);
            for(index_t e=0; e<ne; ++e) {
                v_edges_ptr_[edge_v_[2*e]+1]++;
                v_edges_ptr_[edge_v_[2*e+1]+1]++;
            }
            for(index_t v=0; v<nv; ++v) {
                v_edges_ptr_[v+1] += v_edges_ptr_[v];
            }
            v_edges_.assign(v_edges_ptr_[nv], NO_INDEX);
            vector<index_t> fill(nv, 0);
            for(index_t e=0; e<ne; ++e) {
                for(index_t k=0; k<2; ++k) {
                    index_t v = edge_v_[2*e+k];
                    v_edges_[v_edges_ptr_[v] + fill[v]] = e;
                    fill[v]++;
                }
            }
        }

        index_t nb_edges() const {
            return edge_v_.size()/2;
        }

        index_t nb_tris() const {
            return tri_v_.size()/3;
        }

        index_t edge_nb_tets(index_t e) const {
            return edge_tets_ptr_[e+1] - edge_tets_ptr_[e];
        }

        index_t edge_tet(index_t e, index_t k) const {
            return edge_tets_[edge_tets_ptr_[e] + k];
        }

        index_t edge_tri(index_t e, index_t k) const {
            return edge_tris_[edge_tets_ptr_[e] + k];
        }

        /*****************************************************************/
        /** Geometry: centers c(X)                                       */
        /*****************************************************************/

        /**
         * \brief Computes the radical point of four weighted points
         *  (center of the tetrahedron)
         */
        vec3 radical_point(
            index_t v0, index_t v1, index_t v2, index_t v3
        ) const {
            vec3 p0 = point(v0);
            vec3 p1 = point(v1);
            vec3 p2 = point(v2);
            vec3 p3 = point(v3);
            double h0 = length2(p0) - weight(v0);
            double h1 = length2(p1) - weight(v1);
            double h2 = length2(p2) - weight(v2);
            double h3 = length2(p3) - weight(v3);
            vec3 c;
            if(!solve3_rows(
                   p1-p0, p2-p0, p3-p0,
                   0.5*vec3(h1-h0, h2-h0, h3-h0), c
               )
            ) {
                // Degenerate (flat) tetrahedron, should not happen
                // in a valid Delaunay triangulation.
                c = 0.25*(p0+p1+p2+p3);
            }
            return c;
        }

        /**
         * \brief Computes the radical point of three weighted points
         *  (center of the triangle), in the supporting plane.
         * \param[out] a , b the coordinates of the center in the basis
         *  (p1-p0, p2-p0)
         */
        vec3 radical_point(
            index_t v0, index_t v1, index_t v2, double& a, double& b
        ) const {
            vec3 p0 = point(v0);
            vec3 p1 = point(v1);
            vec3 p2 = point(v2);
            double w0 = weight(v0);
            double w1 = weight(v1);
            double w2 = weight(v2);
            vec3 U = p1 - p0;
            vec3 V = p2 - p0;
            double UU = dot(U,U);
            double UV = dot(U,V);
            double VV = dot(V,V);
            double r0 = 0.5*(w0 - w1 + UU);
            double r1 = 0.5*(w0 - w2 + VV);
            double det = UU*VV - UV*UV;
            if(det == 0.0) {
                a = 1.0/3.0;
                b = 1.0/3.0;
            } else {
                a = (r0*VV - r1*UV) / det;
                b = (UU*r1 - UV*r0) / det;
            }
            return p0 + a*U + b*V;
        }

        /**
         * \brief Computes the radical point of two weighted points
         *  (center of the edge), on the supporting line.
         * \param[out] u the coordinate of the center in the basis (p1-p0)
         */
        vec3 radical_point(index_t v0, index_t v1, double& u) const {
            vec3 p0 = point(v0);
            vec3 p1 = point(v1);
            vec3 U = p1 - p0;
            double UU = length2(U);
            u = (UU == 0.0) ? 0.5 : (UU + weight(v0) - weight(v1)) / (2.0*UU);
            return p0 + u*U;
        }

        /*****************************************************************/
        /** Anchor points                                                */
        /*****************************************************************/

        /**
         * \brief Computes for each Delaunay simplex delta_X the point
         *  ad(X) of delta_X closest to c(X), and the face sd(X) of delta_X
         *  that contains it (Lemma 15).
         */
        void compute_delaunay_anchors() {

            // Edges
            index_t ne = nb_edges();
            c_e_.resize(ne);
            ad_e_.resize(ne);
            sd_e_.resize(ne);
            for(index_t e=0; e<ne; ++e) {
                index_t v0 = edge_v_[2*e];
                index_t v1 = edge_v_[2*e+1];
                double u;
                c_e_[e] = radical_point(v0, v1, u);
                if(u > 0.0 && u < 1.0) {
                    sd_e_[e] = SE(e);
                    ad_e_[e] = c_e_[e];
                } else if(u <= 0.0) {
                    sd_e_[e] = SV(v0);
                    ad_e_[e] = point(v0);
                } else {
                    sd_e_[e] = SV(v1);
                    ad_e_[e] = point(v1);
                }
            }

            // Triangles
            index_t nf = nb_tris();
            c_f_.resize(nf);
            ad_f_.resize(nf);
            sd_f_.resize(nf);
            for(index_t f=0; f<nf; ++f) {
                index_t v0 = tri_v_[3*f];
                index_t v1 = tri_v_[3*f+1];
                index_t v2 = tri_v_[3*f+2];
                double a,b;
                c_f_[f] = radical_point(v0, v1, v2, a, b);
                if(a > 0.0 && b > 0.0 && (1.0 - a - b) > 0.0) {
                    sd_f_[f] = SF(f);
                    ad_f_[f] = c_f_[f];
                } else {
                    // Closest point is on the boundary. By Lemma 15,
                    // the closest point on edge e is ad(e).
                    double best_d2 = Numeric::max_float64();
                    for(index_t k=0; k<3; ++k) {
                        index_t e = tri_e_[3*f+k];
                        double d2 = distance2(c_f_[f], ad_e_[e]);
                        if(d2 < best_d2) {
                            best_d2 = d2;
                            sd_f_[f] = sd_e_[e];
                            ad_f_[f] = ad_e_[e];
                        }
                    }
                }
            }

            // Tetrahedra
            index_t nt = nb_tets();
            ad_t_.resize(nt);
            sd_t_.resize(nt);
            for(index_t t=0; t<nt; ++t) {
                if(!tet_finite_[t]) {
                    continue;
                }
                vec3 p0 = point(tet_vertex(t,0));
                vec3 p1 = point(tet_vertex(t,1));
                vec3 p2 = point(tet_vertex(t,2));
                vec3 p3 = point(tet_vertex(t,3));
                vec3 lambda;
                bool inside = false;
                if(solve3_columns(p1-p0, p2-p0, p3-p0, tet_c_[t]-p0, lambda)) {
                    inside = (
                        lambda.x > 0.0 && lambda.y > 0.0 && lambda.z > 0.0 &&
                        (1.0 - lambda.x - lambda.y - lambda.z) > 0.0
                    );
                }
                if(inside) {
                    sd_t_[t] = ST(t);
                    ad_t_[t] = tet_c_[t];
                } else {
                    double best_d2 = Numeric::max_float64();
                    for(index_t lf=0; lf<4; ++lf) {
                        index_t f = tri_of_tf_[4*t+lf];
                        double d2 = distance2(tet_c_[t], ad_f_[f]);
                        if(d2 < best_d2) {
                            best_d2 = d2;
                            sd_t_[t] = sd_f_[f];
                            ad_t_[t] = ad_f_[f];
                        }
                    }
                }
            }
        }

        /*****************************************************************/
        /** The bounding box B                                           */
        /*****************************************************************/

        /**
         * \brief Box faces are numbered 0..5: face k is on axis k/2,
         *  on the min side if k is even and on the max side if k is odd.
         */
        static index_t box_axis(index_t k) {
            return k >> 1;
        }

        static bool box_max_side(index_t k) {
            return (k & 1) != 0;
        }

        /**
         * \brief Gets the outward normal of a box face.
         */
        vec3 box_normal(index_t k) const {
            vec3 N(0.0, 0.0, 0.0);
            N[box_axis(k)] = box_max_side(k) ? 1.0 : -1.0;
            return N;
        }

        /**
         * \brief Gets the offset b of a box face, such that the face is
         *  N.x = b and the box is N.x <= b.
         */
        double box_offset(index_t k) const {
            index_t a = box_axis(k);
            return box_max_side(k) ? box_max_[a] : -box_min_[a];
        }

        /**
         * \brief Gets the generator id of a box face (box faces are
         *  numbered after the points).
         */
        index_t box_gen(index_t k) const {
            return nb_vertices() + k;
        }

        bool is_box_gen(index_t g) const {
            return g >= nb_vertices();
        }

        /**
         * \brief Gets the corner of the box at the intersection of three
         *  faces (with three different axes).
         */
        vec3 box_corner(index_t k1, index_t k2, index_t k3) const {
            vec3 c;
            index_t K[3] = {k1, k2, k3};
            for(index_t i=0; i<3; ++i) {
                index_t a = box_axis(K[i]);
                c[a] = box_max_side(K[i]) ? box_max_[a] : box_min_[a];
            }
            return c;
        }

        /**
         * \brief Clips a line or a ray by the box.
         * \param[in] o , d origin and direction
         * \param[in,out] tlo , thi the parameter interval, clipped in place
         * \param[out] klo , khi the box faces that clip the interval, or
         *  NO_INDEX if the bound was not changed.
         * \retval true if the clipped interval is not empty
         */
        bool clip_by_box(
            const vec3& o, const vec3& d,
            double& tlo, double& thi, index_t& klo, index_t& khi
        ) const {
            klo = NO_INDEX;
            khi = NO_INDEX;
            for(index_t a=0; a<3; ++a) {
                if(d[a] == 0.0) {
                    if(o[a] < box_min_[a] || o[a] > box_max_[a]) {
                        return false;
                    }
                    continue;
                }
                double t1 = (box_min_[a] - o[a]) / d[a];
                double t2 = (box_max_[a] - o[a]) / d[a];
                index_t k1 = 2*a;
                index_t k2 = 2*a+1;
                if(t1 > t2) {
                    std::swap(t1, t2);
                    std::swap(k1, k2);
                }
                if(t1 > tlo) {
                    tlo = t1;
                    klo = k1;
                }
                if(t2 < thi) {
                    thi = t2;
                    khi = k2;
                }
            }
            return tlo <= thi;
        }

        /**
         * \brief Computes the bounding box B.
         * \details B needs to contain the Voronoi component z of each point
         *  x = (1-s) y + s z of the skin surface, y in delta_X, z in nu_X.
         *  The body is contained in conv(P) (+) ball(sqrt(Pmax)), hence
         *  |x-y| <= diam + sqrt(Pmax) and |z-y| = |x-y|/s.
         * \param[in] scale an additional scaling factor (used when
         *  retrying after a consistency failure)
         */
        void compute_box(double scale) {
            double Pmax = 0.0;
            for(index_t a=0; a<3; ++a) {
                box_min_[a] = Numeric::max_float64();
                box_max_[a] = -Numeric::max_float64();
            }
            for(index_t v=0; v<nb_vertices(); ++v) {
                vec3 p = point(v);
                for(index_t a=0; a<3; ++a) {
                    box_min_[a] = std::min(box_min_[a], p[a]);
                    box_max_[a] = std::max(box_max_[a], p[a]);
                }
                Pmax = std::max(Pmax, weight(v));
            }
            double diam = 0.0;
            for(index_t a=0; a<3; ++a) {
                diam += geo_sqr(box_max_[a] - box_min_[a]);
            }
            diam = std::sqrt(diam);
            double margin = scale * (1.05 * (diam + std::sqrt(Pmax)) / s_ + 1e-3*diam);
            for(index_t a=0; a<3; ++a) {
                box_min_[a] -= margin;
                box_max_[a] += margin;
            }
            // Make sure no radical center is too close to the box boundary
            // (so that the inside/outside decisions are unambiguous).
            for(index_t iter=0; iter<30; ++iter) {
                double eps = 1e-6 * (box_max_[0] - box_min_[0]);
                bool ok = true;
                for(index_t t=0; t<nb_tets(); ++t) {
                    if(!tet_finite_[t]) {
                        continue;
                    }
                    for(index_t a=0; a<3; ++a) {
                        double c = tet_c_[t][a];
                        if(
                            std::fabs(c - box_min_[a]) < eps ||
                            std::fabs(c - box_max_[a]) < eps
                        ) {
                            ok = false;
                        }
                    }
                }
                if(ok) {
                    break;
                }
                for(index_t a=0; a<3; ++a) {
                    double m = 0.5*(box_max_[a]+box_min_[a]);
                    double h = 0.5*(box_max_[a]-box_min_[a]) * 1.1;
                    box_min_[a] = m - h;
                    box_max_[a] = m + h;
                }
            }
            tet_in_box_.assign(nb_tets(), false);
            for(index_t t=0; t<nb_tets(); ++t) {
                if(!tet_finite_[t]) {
                    continue;
                }
                bool inside = true;
                for(index_t a=0; a<3; ++a) {
                    double c = tet_c_[t][a];
                    if(c <= box_min_[a] || c >= box_max_[a]) {
                        inside = false;
                    }
                }
                tet_in_box_[t] = inside;
            }
        }

        /*****************************************************************/
        /** Faces of the clipped power diagram                           */
        /*****************************************************************/

        /**
         * \brief A face of the clipped power diagram Vor(P) inter B.
         * \details A face is the intersection of some power cells (its
         *  generators G) and some box faces (K). Its dimension is
         *  3 - |G| - |K| + 1. It is identified by the set of generator ids
         *  (points and box faces, that are numbered after the points).
         */
        struct VFace {
            index_t dim;
            vec3 center;             // c(G), center of the generators
            vec3 normal;             // dim 2: normal of the supporting plane
            vec3 pos;                // dim 0: position
            index_t K[3];            // box faces the face lies on
            index_t nK;
            index_t gen[2];          // dim 1,2: the point generators (nG<=2),
                                     // dim 3: gen[0] = the point
            vector<index_t> verts;   // dim 1: 2 extremities,
                                     // dim 2: vertices in cyclic order
            vector<index_t> subs;    // dim 2: edges (edge k goes from
                                     // verts[k] to verts[k+1]),
                                     // dim 3: facets
            vec3 av;                 // anchor point
            index_t sv;              // face that contains av in its interior
        };

        /**
         * \brief Identifies a face of the clipped power diagram by its
         *  (sorted) set of generators.
         */
        struct FaceKey {
            index_t g[4];
            bool operator==(const FaceKey& rhs) const {
                return g[0]==rhs.g[0] && g[1]==rhs.g[1] &&
                       g[2]==rhs.g[2] && g[3]==rhs.g[3];
            }
        };

        struct FaceKeyHash {
            size_t operator()(const FaceKey& k) const noexcept {
                Numeric::uint64 h = 0x9E3779B97F4A7C15ull;
                for(index_t i=0; i<4; ++i) {
                    h ^= Numeric::uint64(k.g[i]) + 0x7F4A7C159E3779B9ull +
                         (h << 6) + (h >> 2);
                    h *= 0xBF58476D1CE4E5B9ull;
                }
                return size_t(h);
            }
        };

        static FaceKey make_key(
            index_t g0, index_t g1=NO_INDEX, index_t g2=NO_INDEX, index_t g3=NO_INDEX
        ) {
            FaceKey k;
            k.g[0] = g0; k.g[1] = g1; k.g[2] = g2; k.g[3] = g3;
            std::sort(k.g, k.g+4); // NO_INDEX sorts last
            return k;
        }

        /**
         * \brief Gets a face from its key, or creates it.
         * \param[out] created true if the face was created
         */
        index_t get_face(const FaceKey& key, index_t dim, bool& created) {
            auto it = vf_map_.find(key);
            if(it != vf_map_.end()) {
                created = false;
                return it->second;
            }
            created = true;
            index_t result = vf_.size();
            vf_map_[key] = result;
            VFace F;
            F.dim = dim;
            F.nK = 0;
            F.gen[0] = NO_INDEX;
            F.gen[1] = NO_INDEX;
            F.sv = NO_INDEX;
            index_t ng = 0;
            for(index_t i=0; i<4; ++i) {
                if(key.g[i] == NO_INDEX) {
                    continue;
                }
                if(is_box_gen(key.g[i])) {
                    geo_debug_assert(F.nK < 3);
                    F.K[F.nK++] = key.g[i] - nb_vertices();
                } else {
                    if(ng < 2) {
                        F.gen[ng] = key.g[i];
                    }
                    ++ng;
                }
            }
            vf_.push_back(F);
            return result;
        }

        /**
         * \brief Gets or creates a vertex of the clipped power diagram.
         */
        index_t get_vfvertex(const FaceKey& key, const vec3& pos) {
            bool created;
            index_t v = get_face(key, 0, created);
            if(created) {
                vf_[v].pos = pos;
                vf_[v].av = pos;
                vf_[v].sv = v;
            }
            return v;
        }

        /**
         * \brief Gets or creates an edge of the clipped power diagram.
         * \param[in] center the center c(G) of its point generators
         */
        index_t get_vfedge(
            const FaceKey& key, index_t v0, index_t v1, const vec3& center
        ) {
            bool created;
            index_t e = get_face(key, 1, created);
            if(created) {
                vf_[e].verts.push_back(v0);
                vf_[e].verts.push_back(v1);
                vf_[e].center = center;
            }
            return e;
        }

        /**
         * \brief Tests whether a point is inside the power cell of a
         *  point v (relative to its Delaunay neighbors), i.e. whether
         *  it has smaller power distance to v than to its neighbors.
         */
        bool in_power_cell(index_t v, const vec3& x) const {
            double dv = distance2(x, point(v)) - weight(v);
            for(index_t k=v_edges_ptr_[v]; k<v_edges_ptr_[v+1]; ++k) {
                index_t e = v_edges_[k];
                index_t q = edge_v_[2*e] + edge_v_[2*e+1] - v;
                if(distance2(x, point(q)) - weight(q) < dv) {
                    return false;
                }
            }
            return true;
        }

        /**
         * \brief Tests whether a point of the bisector plane of an
         *  edge e is inside the power facet nu_e (relative to the
         *  link of e).
         */
        bool in_power_facet(index_t e, const vec3& x) const {
            index_t v = edge_v_[2*e];
            double dv = distance2(x, point(v)) - weight(v);
            index_t m = edge_nb_tets(e);
            for(index_t k=0; k<m; ++k) {
                index_t f = edge_tri(e,k);
                if(f == NO_INDEX) {
                    continue;
                }
                for(index_t i=0; i<3; ++i) {
                    index_t r = tri_v_[3*f+i];
                    if(r == edge_v_[2*e] || r == edge_v_[2*e+1]) {
                        continue;
                    }
                    if(distance2(x, point(r)) - weight(r) < dv) {
                        return false;
                    }
                }
            }
            return true;
        }

        /**
         * \brief Computes the clipped power edges nu_f inter B of all
         *  the triangles (segments between radical centers, or rays for
         *  the triangles on the convex hull).
         * \details vf_tri_[f] receives the face id, or NO_INDEX if the
         *  clipped power edge is empty. The first extremity of the face
         *  is on the side of tri_t_[2*f].
         */
        void build_tri_segments() {
            index_t nf = nb_tris();
            vf_tri_.assign(nf, NO_INDEX);
            for(index_t f=0; f<nf; ++f) {
                index_t v0 = tri_v_[3*f];
                index_t v1 = tri_v_[3*f+1];
                index_t v2 = tri_v_[3*f+2];
                index_t t0 = tri_t_[2*f];
                index_t t1 = tri_t_[2*f+1];
                bool fin0 = (t0 != NO_INDEX && tet_finite_[t0]);
                bool fin1 = (t1 != NO_INDEX && tet_finite_[t1]);
                if(!fin0 && !fin1) {
                    continue;
                }
                vec3 o;
                vec3 d;
                double tlo = 0.0;
                double thi;
                index_t t_from = NO_INDEX; // tet at the origin of the ray
                bool swapped = false;      // true if t_from is t1
                if(fin0 && fin1) {
                    o = tet_c_[t0];
                    d = tet_c_[t1] - tet_c_[t0];
                    thi = 1.0;
                    t_from = t0;
                } else {
                    t_from = fin0 ? t0 : t1;
                    swapped = !fin0;
                    o = tet_c_[t_from];
                    // ray direction: normal to the triangle pointing
                    // away from the opposite vertex of the finite tet
                    vec3 p0 = point(v0);
                    vec3 n = cross(point(v1)-p0, point(v2)-p0);
                    index_t opp = NO_INDEX;
                    for(index_t lv=0; lv<4; ++lv) {
                        index_t w = tet_vertex(t_from, lv);
                        if(w != v0 && w != v1 && w != v2) {
                            opp = w;
                        }
                    }
                    if(dot(n, point(opp)-p0) > 0.0) {
                        n = -1.0*n;
                    }
                    d = normalize(n);
                    thi = Numeric::max_float64();
                }
                index_t klo, khi;
                if(!clip_by_box(o, d, tlo, thi, klo, khi)) {
                    continue;
                }
                // Extremities. Decisions on the radical centers are the
                // global ones (tet_in_box_), consistent for all faces.
                index_t a, b;
                if(tet_in_box_[t_from]) {
                    FaceKey k = make_key(
                        tet_vertex(t_from,0), tet_vertex(t_from,1),
                        tet_vertex(t_from,2), tet_vertex(t_from,3)
                    );
                    a = get_vfvertex(k, tet_c_[t_from]);
                } else {
                    if(klo == NO_INDEX) {
                        continue; // should not happen
                    }
                    a = get_vfvertex(
                        make_key(v0, v1, v2, box_gen(klo)), o + tlo*d
                    );
                }
                index_t t_to = (fin0 && fin1) ? t1 : NO_INDEX;
                if(t_to != NO_INDEX && tet_in_box_[t_to]) {
                    FaceKey k = make_key(
                        tet_vertex(t_to,0), tet_vertex(t_to,1),
                        tet_vertex(t_to,2), tet_vertex(t_to,3)
                    );
                    b = get_vfvertex(k, tet_c_[t_to]);
                } else {
                    if(khi == NO_INDEX) {
                        continue; // should not happen
                    }
                    b = get_vfvertex(
                        make_key(v0, v1, v2, box_gen(khi)), o + thi*d
                    );
                }
                if(a == b) {
                    continue;
                }
                if(swapped) {
                    std::swap(a, b);
                }
                vf_tri_[f] = get_vfedge(make_key(v0, v1, v2), a, b, c_f_[f]);
            }
        }

        /**
         * \brief Walks along the boundary of C = B inter (bisector plane
         *  of e), from a vertex to another one (or all around C).
         * \details Creates the vertices (on the edges of the box) and
         *  the edges (on the faces of the box) of the clipped power
         *  facet nu_e inter B that lie on the boundary of B.
         * \param[in] e the edge
         * \param[in] n the unit normal of the bisector plane
         * \param[in] sigma +1 if the boundary of nu_e inter B is
         *  traversed counterclockwise around n, -1 otherwise
         * \param[in] from the vertex to start from, and k its box face
         * \param[in] to the vertex to reach (on box face kto), or NO_INDEX
         *  to walk all around C (then from needs to be on a box edge)
         * \param[out] cycle_v , cycle_e the generated vertices and edges
         *  are appended to these lists. The last vertex appended is \p to
         *  (or \p from) and is not in the list.
         * \retval true on success
         * \retval false on inconsistency
         */
        bool walk_box_section(
            index_t e, const vec3& n, double sigma,
            index_t from, index_t k, index_t to, index_t kto,
            vector<index_t>& cycle_v, vector<index_t>& cycle_e
        ) {
            index_t v = edge_v_[2*e];
            index_t q = edge_v_[2*e+1];
            index_t cur = from;
            index_t kprev = NO_INDEX;
            for(index_t iter=0; iter<8; ++iter) {
                vec3 P = vf_[cur].pos;
                vec3 dir = sigma * cross(n, box_normal(k));
                double l = length(dir);
                if(l < 1e-12) {
                    return false;
                }
                dir = (1.0/l) * dir;
                // next box face hit
                double tmin = Numeric::max_float64();
                index_t knext = NO_INDEX;
                for(index_t k2=0; k2<6; ++k2) {
                    if(box_axis(k2) == box_axis(k) || k2 == kprev) {
                        continue;
                    }
                    double denom = dot(dir, box_normal(k2));
                    if(denom <= 0.0) {
                        continue;
                    }
                    double t = (box_offset(k2) - dot(box_normal(k2), P)) / denom;
                    if(t < tmin) {
                        tmin = t;
                        knext = k2;
                    }
                }
                if(knext == NO_INDEX) {
                    return false;
                }
                double eps = 1e-9 * (1.0 + tmin);
                if(to != NO_INDEX && kto == k) {
                    double ta = dot(vf_[to].pos - P, dir);
                    if(ta >= -eps && ta <= tmin + eps) {
                        if(to != cur) {
                            cycle_v.push_back(cur);
                            cycle_e.push_back(get_vfedge(
                                make_key(v, q, box_gen(k)), cur, to, c_e_[e]
                            ));
                        }
                        return true;
                    }
                }
                vec3 Q = P + tmin*dir;
                FaceKey kq = make_key(v, q, box_gen(k), box_gen(knext));
                index_t next = get_vfvertex(kq, Q);
                if(to == NO_INDEX && next == from) {
                    cycle_v.push_back(cur);
                    cycle_e.push_back(get_vfedge(
                        make_key(v, q, box_gen(k)), cur, from, c_e_[e]
                    ));
                    return true;
                }
                cycle_v.push_back(cur);
                cycle_e.push_back(get_vfedge(
                    make_key(v, q, box_gen(k)), cur, next, c_e_[e]
                ));
                kprev = k;
                k = knext;
                cur = next;
            }
            return false;
        }

        /**
         * \brief Gets the box face a vertex of the clipped power diagram
         *  lies on (the first one if it lies on several ones).
         */
        index_t vertex_box_face(index_t vv) const {
            return (vf_[vv].nK == 0) ? NO_INDEX : vf_[vv].K[0];
        }

        /**
         * \brief Computes the clipped power facet nu_e inter B of an
         *  edge e (a convex polygon in the bisector plane).
         * \details vf_edge_[e] receives the face id, or NO_INDEX if the
         *  clipped power facet is empty.
         * \retval true on success
         * \retval false on inconsistency
         */
        bool build_edge_polygon(index_t e) {
            index_t v = edge_v_[2*e];
            index_t q = edge_v_[2*e+1];
            vec3 n = normalize(point(q) - point(v));
            index_t m = edge_nb_tets(e);

            // Collect the non-empty clipped power edges around e, in
            // cyclic order, oriented along the traversal.
            vector<index_t> seg_f;   // face id
            vector<index_t> seg_a;   // first vertex
            vector<index_t> seg_b;   // second vertex
            vector<index_t> seg_tri; // triangle
            for(index_t k=0; k<m; ++k) {
                index_t f = edge_tri(e,k);
                if(f == NO_INDEX || vf_tri_[f] == NO_INDEX) {
                    continue;
                }
                index_t S = vf_tri_[f];
                index_t a = vf_[S].verts[0];
                index_t b = vf_[S].verts[1];
                if(tri_t_[2*f] != edge_tet(e,k)) {
                    std::swap(a,b);
                }
                seg_f.push_back(S);
                seg_a.push_back(a);
                seg_b.push_back(b);
                seg_tri.push_back(f);
            }

            vector<index_t> cycle_v;
            vector<index_t> cycle_e;

            if(seg_f.size() == 0) {
                // No power edge crosses B: nu_e inter B is either empty
                // or the whole section C = B inter plane.
                index_t start = NO_INDEX;
                index_t kstart = NO_INDEX;
                for(index_t k1=0; k1<6 && start == NO_INDEX; ++k1) {
                    for(index_t k2=k1+1; k2<6 && start == NO_INDEX; ++k2) {
                        if(box_axis(k1) == box_axis(k2)) {
                            continue;
                        }
                        // the box edge (k1,k2): find its two corners
                        index_t a3 = 3 - box_axis(k1) - box_axis(k2);
                        vec3 c1 = box_corner(k1, k2, 2*a3);
                        vec3 c2 = box_corner(k1, k2, 2*a3+1);
                        double s1 = dot(n, c1 - c_e_[e]);
                        double s2 = dot(n, c2 - c_e_[e]);
                        if(s1 * s2 >= 0.0) {
                            continue;
                        }
                        vec3 x = c1 + (s1/(s1-s2))*(c2-c1);
                        if(!in_power_facet(e, x)) {
                            // C is entirely outside nu_e
                            vf_edge_[e] = NO_INDEX;
                            return true;
                        }
                        start = get_vfvertex(
                            make_key(v, q, box_gen(k1), box_gen(k2)), x
                        );
                        // choose the face to walk on: the one along which
                        // the direction does not immediately hit the other
                        vec3 d1 = cross(n, box_normal(k1));
                        kstart = (dot(d1, box_normal(k2)) > 0.0) ? k2 : k1;
                    }
                }
                if(start == NO_INDEX) {
                    vf_edge_[e] = NO_INDEX; // the plane misses the box
                    return true;
                }
                if(!walk_box_section(
                       e, n, 1.0, start, kstart, NO_INDEX, NO_INDEX,
                       cycle_v, cycle_e
                   )
                ) {
                    return false;
                }
            } else {
                // Orientation of the traversal around n: the interior
                // of nu_e is on the side of v relative to the third vertex
                // r of the first triangle.
                index_t f0 = seg_tri[0];
                index_t r = NO_INDEX;
                for(index_t i=0; i<3; ++i) {
                    index_t w = tri_v_[3*f0+i];
                    if(w != v && w != q) {
                        r = w;
                    }
                }
                vec3 in_dir = point(v) - point(r);
                in_dir = in_dir - dot(in_dir, n)*n;
                vec3 ab = vf_[seg_b[0]].pos - vf_[seg_a[0]].pos;
                double sigma = (dot(n, cross(ab, in_dir)) >= 0.0) ? 1.0 : -1.0;
                index_t ns = seg_f.size();
                for(index_t i=0; i<ns; ++i) {
                    cycle_v.push_back(seg_a[i]);
                    cycle_e.push_back(seg_f[i]);
                    index_t b = seg_b[i];
                    index_t a_next = seg_a[(i+1)%ns];
                    if(b == a_next) {
                        continue;
                    }
                    // gap between b and a_next: along the boundary of B
                    index_t kb = vertex_box_face(b);
                    index_t ka = vertex_box_face(a_next);
                    if(kb == NO_INDEX || ka == NO_INDEX) {
                        return false;
                    }
                    if(!walk_box_section(
                           e, n, sigma, b, kb, a_next, ka, cycle_v, cycle_e
                       )
                    ) {
                        return false;
                    }
                }
            }
            if(cycle_v.size() < 3) {
                return false;
            }
            bool created;
            index_t F = get_face(make_key(v, q), 2, created);
            geo_assert(created);
            vf_[F].verts.swap(cycle_v);
            vf_[F].subs.swap(cycle_e);
            vf_[F].center = c_e_[e];
            vf_[F].normal = n;
            vf_edge_[e] = F;
            return true;
        }

        /**
         * \brief Computes the facet nu_v inter F_k of the clipped power
         *  cell of v on the box face k.
         * \details Its boundary is made of the box edges of the clipped
         *  power facets nu_{vq} inter B that lie on F_k, and of pieces of
         *  the edges of the box face.
         * \return the face id, or NO_INDEX if empty
         * \param[out] ok set to false on inconsistency
         */
        index_t build_box_facet(index_t v, index_t k, bool& ok) {
            ok = true;
            vec3 Nk = box_normal(k);
            vec3 pv = point(v);

            // The four box faces adjacent to k, in counterclockwise
            // order around Nk.
            index_t adj[4];
            {
                index_t a = box_axis(k);
                vec3 u1(0.0,0.0,0.0);
                vec3 u2(0.0,0.0,0.0);
                u1[(a+1)%3] = 1.0;
                u2[(a+2)%3] = 1.0;
                if(!box_max_side(k)) {
                    std::swap(u1,u2);
                }
                double angle[4];
                index_t na = 0;
                for(index_t j=0; j<6; ++j) {
                    if(box_axis(j) == a) {
                        continue;
                    }
                    vec3 Nj = box_normal(j);
                    adj[na] = j;
                    angle[na] = std::atan2(dot(Nj,u2), dot(Nj,u1));
                    ++na;
                }
                for(index_t i=0; i<4; ++i) {
                    for(index_t j=i+1; j<4; ++j) {
                        if(angle[j] < angle[i]) {
                            std::swap(angle[i], angle[j]);
                            std::swap(adj[i], adj[j]);
                        }
                    }
                }
            }

            // Outgoing edge of each node of the boundary (node = vertex
            // of the clipped diagram on F_k).
            std::unordered_map<index_t, std::pair<index_t,index_t>> out;
            // The box edges (sigma) of the power facets on F_k
            index_t nb_sigma = 0;
            vector<index_t> on_edge[4]; // vertices on each box edge of F_k
            for(index_t kk=v_edges_ptr_[v]; kk<v_edges_ptr_[v+1]; ++kk) {
                index_t e = v_edges_[kk];
                index_t P = vf_edge_[e];
                if(P == NO_INDEX) {
                    continue;
                }
                index_t q = edge_v_[2*e] + edge_v_[2*e+1] - v;
                for(index_t s : vf_[P].subs) {
                    const VFace& S = vf_[s];
                    if(S.nK != 1 || S.K[0] != k) {
                        continue;
                    }
                    ++nb_sigma;
                    // orient the segment counterclockwise around Nk,
                    // the interior (side of v) being on the left
                    vec3 in_dir = pv - point(q);
                    in_dir = in_dir - dot(in_dir,Nk)*Nk;
                    vec3 d = cross(in_dir, Nk);
                    index_t x = S.verts[0];
                    index_t y = S.verts[1];
                    if(dot(d, vf_[y].pos - vf_[x].pos) < 0.0) {
                        std::swap(x,y);
                    }
                    if(out.find(x) != out.end()) {
                        ok = false;
                        return NO_INDEX;
                    }
                    out[x] = std::make_pair(y, s);
                    // register the extremities that are on box edges
                    for(index_t w : {x, y}) {
                        const VFace& W = vf_[w];
                        if(W.nK == 2) {
                            index_t j = (W.K[0] == k) ? W.K[1] : W.K[0];
                            for(index_t i=0; i<4; ++i) {
                                if(adj[i] == j) {
                                    on_edge[i].push_back(w);
                                }
                            }
                        }
                    }
                }
            }

            // Corners of F_k: corner i is between box edges i-1 and i.
            index_t corner[4];
            bool corner_in[4];
            for(index_t i=0; i<4; ++i) {
                index_t j1 = adj[(i+3)%4];
                index_t j2 = adj[i];
                vec3 c = box_corner(k, j1, j2);
                corner_in[i] = in_power_cell(v, c);
                corner[i] = corner_in[i] ?
                    get_vfvertex(
                        make_key(v, box_gen(k), box_gen(j1), box_gen(j2)), c
                    ) : NO_INDEX;
            }

            if(nb_sigma == 0) {
                index_t nin = 0;
                for(index_t i=0; i<4; ++i) {
                    nin += index_t(corner_in[i]);
                }
                if(nin == 0) {
                    return NO_INDEX;
                }
                if(nin != 4) {
                    ok = false;
                    return NO_INDEX;
                }
            }

            // Pieces of the box edges of F_k. Box edge i goes from
            // corner i to corner i+1, in direction Nk x Nj.
            vector<index_t> cycle_v;
            vector<index_t> cycle_e;
            for(index_t i=0; i<4; ++i) {
                index_t j = adj[i];
                vec3 dir = cross(Nk, box_normal(j));
                vector<index_t>& pts = on_edge[i];
                // remove duplicates and sort along dir
                std::sort(pts.begin(), pts.end());
                pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
                std::sort(
                    pts.begin(), pts.end(),
                    [&](index_t a, index_t b) {
                        return dot(vf_[a].pos, dir) < dot(vf_[b].pos, dir);
                    }
                );
                vector<index_t> seq;
                if(corner_in[i]) {
                    seq.push_back(corner[i]);
                }
                for(index_t p : pts) {
                    seq.push_back(p);
                }
                if(corner_in[(i+1)%4]) {
                    seq.push_back(corner[(i+1)%4]);
                }
                for(index_t p=0; p+1<seq.size(); ++p) {
                    index_t x = seq[p];
                    index_t y = seq[p+1];
                    // the piece x->y is inside nu_v iff x is a corner or
                    // the end of a sigma (then the boundary continues
                    // along the box edge).
                    bool x_is_corner = (vf_[x].nK == 3);
                    bool x_is_end = false;
                    for(auto& kv : out) {
                        if(kv.second.first == x) {
                            x_is_end = true;
                            break;
                        }
                    }
                    if(!(x_is_corner || x_is_end)) {
                        continue;
                    }
                    if(out.find(x) != out.end()) {
                        ok = false;
                        return NO_INDEX;
                    }
                    index_t piece = get_vfedge(
                        make_key(v, box_gen(k), box_gen(j)), x, y, pv
                    );
                    out[x] = std::make_pair(y, piece);
                }
            }

            if(out.size() == 0) {
                return NO_INDEX;
            }

            // Chain the boundary
            index_t start = out.begin()->first;
            index_t cur = start;
            do {
                auto it = out.find(cur);
                if(it == out.end()) {
                    ok = false;
                    return NO_INDEX;
                }
                cycle_v.push_back(cur);
                cycle_e.push_back(it->second.second);
                cur = it->second.first;
                if(cycle_v.size() > out.size()) {
                    ok = false;
                    return NO_INDEX;
                }
            } while(cur != start);
            if(cycle_v.size() != out.size() || cycle_v.size() < 3) {
                ok = false;
                return NO_INDEX;
            }

            bool created;
            index_t F = get_face(make_key(v, box_gen(k)), 2, created);
            geo_assert(created);
            vf_[F].verts.swap(cycle_v);
            vf_[F].subs.swap(cycle_e);
            vf_[F].center = pv;
            vf_[F].normal = Nk;
            return F;
        }

        /**
         * \brief Computes the clipped power cell nu_v inter B of an atom.
         * \retval true on success
         * \retval false on inconsistency
         */
        bool build_vertex_polyhedron(index_t v) {
            vector<index_t> facets;
            for(index_t kk=v_edges_ptr_[v]; kk<v_edges_ptr_[v+1]; ++kk) {
                index_t e = v_edges_[kk];
                if(vf_edge_[e] != NO_INDEX) {
                    facets.push_back(vf_edge_[e]);
                }
            }
            for(index_t k=0; k<6; ++k) {
                bool ok;
                index_t F = build_box_facet(v, k, ok);
                if(!ok) {
                    return false;
                }
                if(F != NO_INDEX) {
                    facets.push_back(F);
                }
            }
            if(facets.size() == 0) {
                vf_vertex_[v] = NO_INDEX;
                return true;
            }
            if(facets.size() < 4) {
                return false;
            }
            bool created;
            index_t C = get_face(make_key(v), 3, created);
            geo_assert(created);
            vf_[C].subs.swap(facets);
            vf_[C].center = point(v);
            vf_vertex_[v] = C;
            return true;
        }

        /**
         * \brief Builds all the faces of the clipped power diagram that
         *  are needed to mesh the mixed cells of the atoms.
         * \retval true on success
         * \retval false on inconsistency (then the box is enlarged and
         *  the construction is restarted)
         */
        bool build_clipped_power_diagram() {
            vf_.clear();
            vf_map_.clear();
            vf_tri_.clear();
            vf_edge_.assign(nb_edges(), NO_INDEX);
            vf_vertex_.assign(nb_vertices(), NO_INDEX);
            build_tri_segments();
            for(index_t e=0; e<nb_edges(); ++e) {
                if(!build_edge_polygon(e)) {
                    return false;
                }
            }
            for(index_t v=0; v<nb_atoms_; ++v) {
                if(!build_vertex_polyhedron(v)) {
                    return false;
                }
            }
            return true;
        }

        /*****************************************************************/
        /** Anchor points of the clipped power diagram                   */
        /*****************************************************************/

        /**
         * \brief Computes for each face nu of the clipped power diagram
         *  the point av of nu closest to the center c(G) of its point
         *  generators, and the face sv that contains it in its relative
         *  interior (Lemma 16, generalized to the clipped diagram).
         */
        void compute_voronoi_anchors() {
            // Faces are created in order of increasing dimension within
            // each construction step, but a facet of a polyhedron may be
            // created after the polyhedron is... no: faces of dimension d
            // only reference faces of dimension d-1 created before them.
            // We nevertheless process by increasing dimension to be safe.
            for(index_t dim=1; dim<=3; ++dim) {
                for(index_t F=0; F<vf_.size(); ++F) {
                    if(vf_[F].dim != dim) {
                        continue;
                    }
                    compute_face_anchor(F);
                }
            }
        }

        void compute_face_anchor(index_t F) {
            VFace& f = vf_[F];
            if(f.dim == 1) {
                vec3 a = vf_[f.verts[0]].pos;
                vec3 b = vf_[f.verts[1]].pos;
                vec3 D = b - a;
                double DD = length2(D);
                double lambda = (DD == 0.0) ? 0.0 : dot(f.center - a, D) / DD;
                if(lambda > 0.0 && lambda < 1.0) {
                    f.av = a + lambda*D;
                    f.sv = F;
                } else if(lambda <= 0.0) {
                    f.av = a;
                    f.sv = f.verts[0];
                } else {
                    f.av = b;
                    f.sv = f.verts[1];
                }
            } else if(f.dim == 2) {
                vec3 p0 = vf_[f.verts[0]].pos;
                vec3 proj = f.center - dot(f.center - p0, f.normal)*f.normal;
                index_t n = f.verts.size();
                bool has_pos = false;
                bool has_neg = false;
                for(index_t k=0; k<n; ++k) {
                    vec3 pk = vf_[f.verts[k]].pos;
                    vec3 pk1 = vf_[f.verts[(k+1)%n]].pos;
                    double sgn = dot(f.normal, cross(pk1-pk, proj-pk));
                    if(sgn > 0.0) {
                        has_pos = true;
                    } else {
                        has_neg = true;
                    }
                }
                if(has_pos != has_neg) {
                    f.av = proj;
                    f.sv = F;
                } else {
                    double best = Numeric::max_float64();
                    for(index_t s : f.subs) {
                        double d2 = distance2(f.center, vf_[s].av);
                        if(d2 < best) {
                            best = d2;
                            f.av = vf_[s].av;
                            f.sv = vf_[s].sv;
                        }
                    }
                }
            } else {
                vec3 p = f.center;
                bool inside = in_power_cell(f.gen[0], p);
                for(index_t a=0; a<3 && inside; ++a) {
                    if(p[a] <= box_min_[a] || p[a] >= box_max_[a]) {
                        inside = false;
                    }
                }
                if(inside) {
                    f.av = p;
                    f.sv = F;
                } else {
                    double best = Numeric::max_float64();
                    for(index_t s : f.subs) {
                        double d2 = distance2(p, vf_[s].av);
                        if(d2 < best) {
                            best = d2;
                            f.av = vf_[s].av;
                            f.sv = vf_[s].sv;
                        }
                    }
                }
            }
        }

        Simplex support_d(Simplex X) const {
            switch(X.dim) {
            case 0: return X;
            case 1: return sd_e_[X.id];
            case 2: return sd_f_[X.id];
            default: return sd_t_[X.id];
            }
        }

        vec3 anchor_d(Simplex X) const {
            switch(X.dim) {
            case 0: return point(X.id);
            case 1: return ad_e_[X.id];
            case 2: return ad_f_[X.id];
            default: return ad_t_[X.id];
            }
        }

        /*****************************************************************/
        /** Quadrics                                                     */
        /*****************************************************************/

        Quadric vertex_quadric(index_t v) const {
            Quadric Q;
            Q.ell = 0;
            Q.c = point(v);
            Q.axis = vec3(0.0, 0.0, 0.0);
            Q.W = weight(v);
            Q.s = s_;
            return Q;
        }

        Quadric edge_quadric(index_t e) const {
            index_t v0 = edge_v_[2*e];
            index_t v1 = edge_v_[2*e+1];
            Quadric Q;
            Q.ell = 1;
            Q.c = c_e_[e];
            Q.axis = normalize(point(v1) - point(v0));
            Q.W = weight(v0) - distance2(Q.c, point(v0));
            Q.s = s_;
            return Q;
        }

        Quadric tri_quadric(index_t f) const {
            index_t v0 = tri_v_[3*f];
            index_t v1 = tri_v_[3*f+1];
            index_t v2 = tri_v_[3*f+2];
            vec3 p0 = point(v0);
            Quadric Q;
            Q.ell = 2;
            Q.c = c_f_[f];
            Q.axis = normalize(cross(point(v1)-p0, point(v2)-p0));
            Q.W = weight(v0) - distance2(Q.c, p0);
            Q.s = s_;
            return Q;
        }

        Quadric tet_quadric(index_t t) const {
            index_t v0 = tet_vertex(t,0);
            Quadric Q;
            Q.ell = 3;
            Q.c = tet_c_[t];
            Q.axis = vec3(0.0, 0.0, 0.0);
            Q.W = weight(v0) - distance2(Q.c, point(v0));
            Q.s = s_;
            return Q;
        }

        /*****************************************************************/
        /** Tetrahedral complex                                          */
        /*****************************************************************/

        /**
         * \brief Gets (or creates) the vertex of the tetrahedral complex
         *  that corresponds to the anchor point of the polyhedral cell
         *  (1-s) delta_X (+) s nu, where nu is a face of the clipped
         *  power diagram.
         * \param[in] X a Delaunay simplex whose vertices are generators
         *  of \p nu
         * \param[in] nu a face of the clipped power diagram
         * \param[in] q the index of the quadric of the mixed cell being
         *  processed, used to compute the value of the implicit function
         *  at the vertex (if it is created).
         */
        index_t get_vertex(Simplex X, index_t nu, index_t q) {
            VertexKey key{ support_d(X).key(), Numeric::uint64(vf_[nu].sv) };
            auto it = cv_map_.find(key);
            if(it != cv_map_.end()) {
                return it->second;
            }
            vec3 p = (1.0 - s_)*anchor_d(X) + s_*vf_[nu].av;
            index_t result = cv_pos_.size();
            cv_map_[key] = result;
            cv_pos_.push_back(p);
            cv_val_.push_back(quadrics_[q].eval(p));
            return result;
        }

        /**
         * \brief Adds a tetrahedron to the complex if it is not
         *  degenerate (collapsed anchor points generate degenerate
         *  tetrahedra, Section 3.2 of the paper).
         */
        void add_tet(
            index_t a, index_t b, index_t c, index_t d, index_t q
        ) {
            if(a==b || a==c || a==d || b==c || b==d || c==d) {
                ++nb_degenerate_;
                return;
            }
            ct_v_.push_back(a);
            ct_v_.push_back(b);
            ct_v_.push_back(c);
            ct_v_.push_back(d);
            ct_q_.push_back(q);
        }

        /**
         * \brief Joins the triangles of a (triangulated) facet with an
         *  apex vertex.
         */
        void join(
            index_t apex, const vector<index_t>& tris, index_t q
        ) {
            for(index_t k=0; k+2<tris.size(); k+=3) {
                add_tet(apex, tris[k], tris[k+1], tris[k+2], q);
            }
        }

        /**
         * \brief Triangulates a circular facet by connecting its anchor
         *  point to all the anchor points on its boundary (Figure 4(a)).
         * \param[in] F the anchor point of the facet
         * \param[in] cycle the anchor points on the boundary of the facet,
         *  in cyclic order
         * \param[out] tris the triangles (3 vertices per triangle),
         *  appended.
         */
        void fan(
            index_t F, const vector<index_t>& cycle, vector<index_t>& tris
        ) {
            index_t n = cycle.size();
            for(index_t k=0; k<n; ++k) {
                index_t a = cycle[k];
                index_t b = cycle[(k+1)%n];
                if(a == b || a == F || b == F) {
                    continue;
                }
                tris.push_back(F);
                tris.push_back(a);
                tris.push_back(b);
            }
        }

        /**
         * \brief Triangulates a hyperbolic facet (1-s) delta_e (+) s S,
         *  where S is an edge of the clipped power diagram lying in the
         *  bisector plane of e (a rectangle, Figure 4(b)), split into its
         *  four quadrants.
         * \details Corners are (v_i, w_j) where w_j are the extremities
         *  of S. Edge anchors are (v_i, S) on the two sides parallel to
         *  the symmetry axis of the hyperboloid of e, and (sd(e), w_j) on
         *  the two sides parallel to its symmetry plane. The facet anchor
         *  is (sd(e), sv(S)). Quadrant (i,j) is the one that contains
         *  corner (v_i, w_j), it is triangulated with the two triangles
         *  (F, Et_j, Ev_i) and (Et_j, P_ij, Ev_i) : on the edge Et_j-Ev_i,
         *  the distance to one symmetry set increases while the distance
         *  to the other one decreases (monotonicity condition).
         * \param[in] e the edge
         * \param[in] S an edge of the clipped power facet nu_e inter B
         * \param[in] q the quadric of the mixed cell being processed
         * \param[out] quad quad[i][j] receives the two triangles of
         *  quadrant (i,j), i = extremity of e, j = extremity of S.
         */
        void hyperbolic_facet(
            index_t e, index_t S, index_t q, vector<index_t> quad[2][2]
        ) {
            index_t v[2] = { edge_v_[2*e], edge_v_[2*e+1] };
            index_t w[2] = { vf_[S].verts[0], vf_[S].verts[1] };
            index_t F = get_vertex(SE(e), S, q);
            index_t Ev[2];
            index_t Et[2];
            for(index_t i=0; i<2; ++i) {
                Ev[i] = get_vertex(SV(v[i]), S, q);
                Et[i] = get_vertex(SE(e), w[i], q);
            }
            for(index_t i=0; i<2; ++i) {
                for(index_t j=0; j<2; ++j) {
                    index_t P = get_vertex(SV(v[i]), w[j], q);
                    vector<index_t>& Q = quad[i][j];
                    Q.resize(0);
                    Q.push_back(F);     Q.push_back(Et[j]); Q.push_back(Ev[i]);
                    Q.push_back(Et[j]); Q.push_back(P);     Q.push_back(Ev[i]);
                }
            }
        }

        /**
         * \brief Triangulates the mixed cell mu_v of an atom v
         *  (type 0, shrunk clipped power cell, contains a spherical patch).
         * \details The facets are the polygons (1-s) p_v + s F for the
         *  facets F of the clipped power cell (power facets and box
         *  facets). They are circular facets. All the tetrahedra are
         *  joined with the anchor point of the cell (the point of the
         *  cell closest to the center of the sphere).
         */
        void process_vertex_cell(index_t v) {
            index_t C = vf_vertex_[v];
            if(C == NO_INDEX) {
                return;
            }
            index_t q = quadrics_.size();
            quadrics_.push_back(vertex_quadric(v));
            index_t apex = get_vertex(SV(v), C, q);
            vector<index_t> cycle;
            vector<index_t> tris;
            for(index_t Fi : vf_[C].subs) {
                index_t F = get_vertex(SV(v), Fi, q);
                cycle.resize(0);
                tris.resize(0);
                const VFace& P = vf_[Fi];
                for(index_t k=0; k<P.verts.size(); ++k) {
                    cycle.push_back(get_vertex(SV(v), P.verts[k], q));
                    cycle.push_back(get_vertex(SV(v), P.subs[k], q));
                }
                fan(F, cycle, tris);
                join(apex, tris, q);
            }
        }

        /**
         * \brief Triangulates the mixed cell mu_e of an atom edge e
         *  (type 1, prism, contains an hyperboloid patch).
         * \details The two bases (1-s) p_i + s (nu_e inter B) are circular
         *  facets, the side facets (1-s) delta_e (+) s S, for the edges S
         *  of the clipped polygon nu_e inter B (power edges or box
         *  segments, all lying in the bisector plane) are hyperbolic
         *  facets. The prism is split by the plane V through the anchor
         *  point of the cell, parallel to its bases (the symmetry plane
         *  of the hyperboloid). V contains the anchor points (sd(e),w) of
         *  the edges parallel to the axis and the anchor points
         *  (sd(e),sv(S)) of the side facets, and the new facet in V is a
         *  circular facet (a copy of nu_e inter B). Each half-prism is
         *  triangulated by joining the anchor point of its base
         *  (v_i, sv(nu_e)) with its boundary.
         */
        void process_edge_cell(index_t e) {
            index_t P = vf_edge_[e];
            if(P == NO_INDEX) {
                return;
            }
            index_t q = quadrics_.size();
            quadrics_.push_back(edge_quadric(e));
            index_t v[2] = { edge_v_[2*e], edge_v_[2*e+1] };
            index_t C = get_vertex(SE(e), P, q);
            index_t apex[2];
            for(index_t i=0; i<2; ++i) {
                apex[i] = get_vertex(SV(v[i]), P, q);
            }
            const VFace& poly = vf_[P];
            index_t m = poly.verts.size();

            // The splitting facet V (circular facet)
            vector<index_t> cycle;
            vector<index_t> tris;
            for(index_t k=0; k<m; ++k) {
                cycle.push_back(get_vertex(SE(e), poly.verts[k], q));
                cycle.push_back(get_vertex(SE(e), poly.subs[k], q));
            }
            fan(C, cycle, tris);
            join(apex[0], tris, q);
            join(apex[1], tris, q);

            // The side facets (hyperbolic facets), each one split
            // into two halves by V.
            vector<index_t> quad[2][2];
            for(index_t k=0; k<m; ++k) {
                hyperbolic_facet(e, poly.subs[k], q, quad);
                for(index_t i=0; i<2; ++i) {
                    for(index_t j=0; j<2; ++j) {
                        join(apex[i], quad[i][j], q);
                    }
                }
            }
            // Note: the two bases do not need to be processed, since
            // their anchor point is the apex of the half-prism they
            // belong to (joining them would create degenerate tets).
        }

        /**
         * \brief Triangulates the mixed cell mu_f of an atom triangle f
         *  (type 2, prism, contains an hyperboloid patch).
         * \details The clipped power edge S = nu_f inter B is a segment
         *  along the normal of f, with extremities w_0, w_1 (radical
         *  centers or points on the box). The two bases
         *  (1-s) delta_f + s w_j are circular facets, the three side
         *  facets (1-s) delta_e (+) s S are hyperbolic facets. The prism
         *  is split by the plane V through the anchor point of the cell,
         *  parallel to its bases (the plane of f, that is the symmetry
         *  plane of the hyperboloid). V contains the anchor points
         *  (v_i, sv(S)) and (sd(e), sv(S)), the new facet in V is a
         *  circular facet (a copy of delta_f). Each half-prism is
         *  triangulated by joining the anchor point of its base
         *  (sd(f), w_j) with its boundary.
         */
        void process_tri_cell(index_t f) {
            index_t S = vf_tri_[f];
            if(S == NO_INDEX) {
                return;
            }
            index_t q = quadrics_.size();
            quadrics_.push_back(tri_quadric(f));
            index_t w[2] = { vf_[S].verts[0], vf_[S].verts[1] };
            index_t C = get_vertex(SF(f), S, q);
            index_t apex[2];
            for(index_t j=0; j<2; ++j) {
                apex[j] = get_vertex(SF(f), w[j], q);
            }

            // The splitting facet V (circular facet)
            vector<index_t> cycle;
            vector<index_t> tris;
            for(index_t k=0; k<3; ++k) {
                cycle.push_back(get_vertex(SV(tri_v_[3*f+k]), S, q));
                cycle.push_back(get_vertex(SE(tri_e_[3*f+k]), S, q));
            }
            fan(C, cycle, tris);
            join(apex[0], tris, q);
            join(apex[1], tris, q);

            // The side facets (hyperbolic facets), each one split
            // into two halves by V.
            vector<index_t> quad[2][2];
            for(index_t k=0; k<3; ++k) {
                index_t e = tri_e_[3*f+k];
                hyperbolic_facet(e, S, q, quad);
                for(index_t i=0; i<2; ++i) {
                    for(index_t j=0; j<2; ++j) {
                        join(apex[j], quad[i][j], q);
                    }
                }
            }
        }

        /**
         * \brief Triangulates the mixed cell mu_t of an atom tetrahedron
         *  t (type 3, shrunk tetrahedron, contains a spherical patch).
         * \details Nothing to do if the radical center of t is outside B
         *  (then the cell contains no part of the skin surface). The four
         *  facets (1-s) delta_f + s c_t are circular facets. All the
         *  tetrahedra are joined with the anchor point of the cell.
         */
        void process_tet_cell(index_t t) {
            if(!tet_in_box_[t]) {
                return;
            }
            FaceKey key = make_key(
                tet_vertex(t,0), tet_vertex(t,1), tet_vertex(t,2), tet_vertex(t,3)
            );
            auto it = vf_map_.find(key);
            if(it == vf_map_.end()) {
                return; // isolated radical center (should not happen)
            }
            index_t W = it->second;
            index_t q = quadrics_.size();
            quadrics_.push_back(tet_quadric(t));
            index_t apex = get_vertex(ST(t), W, q);
            vector<index_t> cycle;
            vector<index_t> tris;
            for(index_t lf=0; lf<4; ++lf) {
                index_t f = tri_of_tf_[4*t+lf];
                index_t F = get_vertex(SF(f), W, q);
                cycle.resize(0);
                tris.resize(0);
                for(index_t k=0; k<3; ++k) {
                    cycle.push_back(get_vertex(SV(tri_v_[3*f+k]), W, q));
                    cycle.push_back(get_vertex(SE(tri_e_[3*f+k]), W, q));
                }
                fan(F, cycle, tris);
                join(apex, tris, q);
            }
        }

        /**
         * \brief Builds the tetrahedral complex by triangulating all the
         *  mixed cells incident to atoms only.
         */
        void build_tetrahedral_complex() {
            cv_map_.clear();
            cv_pos_.clear();
            cv_val_.clear();
            ct_v_.clear();
            ct_q_.clear();
            quadrics_.clear();
            nb_degenerate_ = 0;
            // Type 0
            for(index_t v=0; v<nb_atoms_; ++v) {
                process_vertex_cell(v);
            }
            // Type 1
            for(index_t e=0; e<nb_edges(); ++e) {
                if(is_atom(edge_v_[2*e]) && is_atom(edge_v_[2*e+1])) {
                    process_edge_cell(e);
                }
            }
            // Type 2
            for(index_t f=0; f<nb_tris(); ++f) {
                if(
                    is_atom(tri_v_[3*f]) &&
                    is_atom(tri_v_[3*f+1]) &&
                    is_atom(tri_v_[3*f+2])
                ) {
                    process_tri_cell(f);
                }
            }
            // Type 3
            for(index_t t=0; t<nb_tets(); ++t) {
                if(
                    tet_finite_[t] &&
                    is_atom(tet_vertex(t,0)) &&
                    is_atom(tet_vertex(t,1)) &&
                    is_atom(tet_vertex(t,2)) &&
                    is_atom(tet_vertex(t,3))
                ) {
                    process_tet_cell(t);
                }
            }
        }

        /**
         * \brief Copies the tetrahedral complex into a mesh.
         */
        void get_tetrahedral_complex(Mesh& M) {
            M.clear();
            M.vertices.set_dimension(3);
            M.vertices.create_vertices(cv_pos_.size());
            Attribute<double> value(M.vertices.attributes(), "value");
            for(index_t v=0; v<cv_pos_.size(); ++v) {
                M.vertices.point(v) = cv_pos_[v];
                value[v] = cv_val_[v];
            }
            Attribute<index_t> type(M.cells.attributes(), "mixed_cell_type");
            for(index_t t=0; t<nb_complex_tets(); ++t) {
                index_t c = M.cells.create_tet(
                    ct_v_[4*t], ct_v_[4*t+1], ct_v_[4*t+2], ct_v_[4*t+3]
                );
                type[c] = quadrics_[ct_q_[t]].ell;
            }
            M.cells.connect();
        }

        /*****************************************************************/
        /** Marching tetrahedra                                          */
        /*****************************************************************/

        /**
         * \brief Finds the root of the quadric of a mixed cell along a
         *  segment, given the values at the extremities.
         * \details I_X(pa + t (pb - pa)) = A t^2 + B t + C, with the
         *  constraints f(0) = fa and f(1) = fb. There is exactly one root
         *  in [0,1] when fa and fb have different signs.
         */
        vec3 segment_root(
            const vec3& pa, const vec3& pb, double fa, double fb, index_t q
        ) const {
            double A = quadrics_[q].form(pb - pa);
            double B = fb - fa - A;
            double C = fa;
            double t = -1.0;
            if(std::fabs(A) > 1e-12 * (std::fabs(B) + std::fabs(C))) {
                double disc = B*B - 4.0*A*C;
                if(disc >= 0.0) {
                    double sq = std::sqrt(disc);
                    double qq = -0.5*(B + (B >= 0.0 ? sq : -sq));
                    double t1 = qq / A;
                    double t2 = (qq != 0.0) ? C / qq : -1.0;
                    if(t1 >= 0.0 && t1 <= 1.0) {
                        t = t1;
                    } else if(t2 >= 0.0 && t2 <= 1.0) {
                        t = t2;
                    }
                }
            } else if(B != 0.0) {
                t = -C / B;
            }
            if(t < 0.0 || t > 1.0) {
                // Should not happen (fa and fb have different signs),
                // fallback to linear interpolation.
                t = fa / (fa - fb);
                t = std::max(0.0, std::min(1.0, t));
            }
            return pa + t*(pb - pa);
        }

        /**
         * \brief Gets (or creates) the mesh vertex at the intersection
         *  between the skin surface and an edge of the tetrahedral
         *  complex.
         * \details The intersection is computed as the root of the
         *  quadric I_X restricted to the edge (there is exactly one by
         *  the monotonicity condition, Observation 18). It is cached, so
         *  that the two cells that share an edge of the complex use the
         *  same vertex (no crack). The support of the created vertex is
         *  the edge (a,b).
         * \param[in] a , b the two extremities of the edge (one inside
         *  and one outside)
         * \param[in] q the quadric of the mixed cell
         */
        index_t edge_intersection(index_t a, index_t b, index_t q) {
            Numeric::uint64 key = edge_key(a, b);
            auto it = edge_vertex_.find(key);
            if(it != edge_vertex_.end()) {
                return it->second;
            }
            vec3 p = segment_root(
                cv_pos_[a], cv_pos_[b], cv_val_[a], cv_val_[b], q
            );
            index_t sup[2] = { std::min(a,b), std::max(a,b) };
            index_t result = new_rvertex(p, sup, 2);
            edge_vertex_[key] = result;
            return result;
        }

        /**
         * \brief Extracts the skin surface from the tetrahedral complex
         *  with the marching tetrahedra algorithm (Figure 5), into the
         *  internal refinable mesh.
         */
        void extract_surface() {
            rv_.clear();
            rt_v_.clear();
            rt_adj_.clear();
            rt_alive_.clear();
            rt_carrier_.clear();
            edge_vertex_.clear();
            build_cv_tets();
            for(index_t t=0; t<nb_complex_tets(); ++t) {
                index_t q = ct_q_[t];
                index_t in[4];
                index_t out[4];
                index_t nb_in = 0;
                index_t nb_out = 0;
                for(index_t lv=0; lv<4; ++lv) {
                    index_t v = ct_v_[4*t+lv];
                    if(cv_val_[v] < 0.0) {
                        in[nb_in++] = v;
                    } else {
                        out[nb_out++] = v;
                    }
                }
                if(nb_in == 0 || nb_out == 0) {
                    continue;
                }
                vec3 g_in(0.0, 0.0, 0.0);
                vec3 g_out(0.0, 0.0, 0.0);
                for(index_t k=0; k<nb_in; ++k) {
                    g_in += cv_pos_[in[k]];
                }
                for(index_t k=0; k<nb_out; ++k) {
                    g_out += cv_pos_[out[k]];
                }
                vec3 dir = (1.0/double(nb_out))*g_out - (1.0/double(nb_in))*g_in;

                if(nb_in == 1) {
                    index_t x0 = edge_intersection(in[0], out[0], q);
                    index_t x1 = edge_intersection(in[0], out[1], q);
                    index_t x2 = edge_intersection(in[0], out[2], q);
                    new_rtriangle(x0, x1, x2, dir, t);
                } else if(nb_in == 3) {
                    index_t x0 = edge_intersection(in[0], out[0], q);
                    index_t x1 = edge_intersection(in[1], out[0], q);
                    index_t x2 = edge_intersection(in[2], out[0], q);
                    new_rtriangle(x0, x1, x2, dir, t);
                } else {
                    // nb_in == 2: quad x00 x01 x11 x10, where xij is
                    // on the edge (in[i], out[j]).
                    index_t x00 = edge_intersection(in[0], out[0], q);
                    index_t x01 = edge_intersection(in[0], out[1], q);
                    index_t x11 = edge_intersection(in[1], out[1], q);
                    index_t x10 = edge_intersection(in[1], out[0], q);
                    new_rtriangle(x00, x01, x11, dir, t);
                    new_rtriangle(x00, x11, x10, dir, t);
                }
            }
            rebuild_adjacency();
        }

        /*****************************************************************/
        /** Refinable mesh                                               */
        /*****************************************************************/

        /*
         * The refinable mesh is a triangle mesh with adjacencies.
         * Each vertex stores its support: the (sorted) vertices of the
         * smallest simplex of the tetrahedral complex that contains it.
         * Each triangle stores its carrier: a tetrahedron of the complex
         * that contains it (the triangle is always contained in a single
         * tetrahedron, this is an invariant maintained by all the
         * operations, that makes the isotopy tests easy).
         *
         * Isotopy is maintained as follows (Section 3.4 of the paper):
         * - new vertices are always inserted on an existing triangle (or
         *   edge) and moved to the skin surface along the transversal
         *   segment through their initial position (Lemma 24);
         * - edge flips and collapses are only done if the new triangles
         *   are each contained in a single tetrahedron of the complex and
         *   pass the test of Lemma 23 (the plane of the triangle separates
         *   the inside and outside vertices of the tetrahedron, with the
         *   inside vertices on the inner side).
         */

        struct RVertex {
            vec3 p;
            index_t sup[4];
            index_t nsup;
            bool alive;
        };

        index_t new_rvertex(const vec3& p, const index_t* sup, index_t nsup) {
            RVertex v;
            v.p = p;
            v.nsup = nsup;
            for(index_t i=0; i<4; ++i) {
                v.sup[i] = (i < nsup) ? sup[i] : NO_INDEX;
            }
            v.alive = true;
            rv_.push_back(v);
            return rv_.size()-1;
        }

        /**
         * \brief Creates a triangle, oriented in such a way that its
         *  normal points towards \p outside.
         */
        index_t new_rtriangle(
            index_t v0, index_t v1, index_t v2, const vec3& outside,
            index_t carrier
        ) {
            vec3 n = cross(rv_[v1].p - rv_[v0].p, rv_[v2].p - rv_[v0].p);
            if(dot(n, outside) < 0.0) {
                std::swap(v1, v2);
            }
            return new_rtriangle(v0, v1, v2, carrier);
        }

        index_t new_rtriangle(
            index_t v0, index_t v1, index_t v2, index_t carrier
        ) {
            index_t t = rt_v_.size()/3;
            rt_v_.push_back(v0);
            rt_v_.push_back(v1);
            rt_v_.push_back(v2);
            rt_adj_.push_back(NO_INDEX);
            rt_adj_.push_back(NO_INDEX);
            rt_adj_.push_back(NO_INDEX);
            rt_alive_.push_back(true);
            rt_carrier_.push_back(carrier);
            return t;
        }

        index_t nb_rtriangles() const {
            return rt_v_.size()/3;
        }

        index_t rt_vertex(index_t t, index_t k) const {
            return rt_v_[3*t + (k%3)];
        }

        index_t& rt_adjacent(index_t t, index_t k) {
            return rt_adj_[3*t + (k%3)];
        }

        /**
         * \brief Finds the side k of triangle t such that the edge
         *  (vertex k, vertex k+1) is (u,v).
         */
        index_t rt_side(index_t t, index_t u, index_t v) const {
            for(index_t k=0; k<3; ++k) {
                if(rt_vertex(t,k) == u && rt_vertex(t,k+1) == v) {
                    return k;
                }
            }
            return NO_INDEX;
        }

        /**
         * \brief Replaces an adjacency pointer in a triangle.
         */
        void rt_replace_adjacent(index_t t, index_t from, index_t to) {
            if(t == NO_INDEX) {
                return;
            }
            for(index_t k=0; k<3; ++k) {
                if(rt_adjacent(t,k) == from) {
                    rt_adjacent(t,k) = to;
                    return;
                }
            }
        }

        /**
         * \brief Recomputes all the adjacencies from scratch.
         */
        void rebuild_adjacency() {
            std::unordered_map<Numeric::uint64, index_t> edge_tri;
            for(index_t t=0; t<nb_rtriangles(); ++t) {
                if(!rt_alive_[t]) {
                    continue;
                }
                for(index_t k=0; k<3; ++k) {
                    rt_adjacent(t,k) = NO_INDEX;
                }
            }
            for(index_t t=0; t<nb_rtriangles(); ++t) {
                if(!rt_alive_[t]) {
                    continue;
                }
                for(index_t k=0; k<3; ++k) {
                    index_t u = rt_vertex(t,k);
                    index_t v = rt_vertex(t,k+1);
                    Numeric::uint64 key = edge_key(u,v);
                    auto it = edge_tri.find(key);
                    if(it == edge_tri.end()) {
                        edge_tri[key] = t;
                    } else {
                        index_t t2 = it->second;
                        index_t k2 = rt_side(t2, v, u);
                        if(k2 != NO_INDEX) {
                            rt_adjacent(t,k) = t2;
                            rt_adjacent(t2,k2) = t;
                        }
                    }
                }
            }
        }

        /**
         * \brief Copies the refinable mesh into a geogram mesh.
         */
        void output_mesh(Mesh& M) {
            M.clear();
            M.vertices.set_dimension(3);
            vector<index_t> new_id(rv_.size(), NO_INDEX);
            index_t nv = 0;
            for(index_t v=0; v<rv_.size(); ++v) {
                if(rv_[v].alive) {
                    new_id[v] = nv++;
                }
            }
            M.vertices.create_vertices(nv);
            for(index_t v=0; v<rv_.size(); ++v) {
                if(rv_[v].alive) {
                    M.vertices.point(new_id[v]) = rv_[v].p;
                }
            }
            Attribute<index_t> type_attr(
                M.facets.attributes(), "mixed_cell_type"
            );
            for(index_t t=0; t<nb_rtriangles(); ++t) {
                if(!rt_alive_[t]) {
                    continue;
                }
                index_t f = M.facets.create_triangle(
                    new_id[rt_vertex(t,0)],
                    new_id[rt_vertex(t,1)],
                    new_id[rt_vertex(t,2)]
                );
                type_attr[f] = quadrics_[ct_q_[rt_carrier_[t]]].ell;
            }
            M.facets.connect();
        }

        /*****************************************************************/
        /** Complex simplices, carriers and transversal segments         */
        /*****************************************************************/

        /**
         * \brief Computes for each vertex of the complex the list of
         *  tetrahedra incident to it.
         */
        void build_cv_tets() {
            index_t nv = cv_pos_.size();
            index_t nt = nb_complex_tets();
            cv_tets_ptr_.assign(nv+1, 0);
            for(index_t i=0; i<4*nt; ++i) {
                cv_tets_ptr_[ct_v_[i]+1]++;
            }
            for(index_t v=0; v<nv; ++v) {
                cv_tets_ptr_[v+1] += cv_tets_ptr_[v];
            }
            cv_tets_.assign(cv_tets_ptr_[nv], NO_INDEX);
            vector<index_t> fill(nv, 0);
            for(index_t t=0; t<nt; ++t) {
                for(index_t lv=0; lv<4; ++lv) {
                    index_t v = ct_v_[4*t+lv];
                    cv_tets_[cv_tets_ptr_[v] + fill[v]] = t;
                    fill[v]++;
                }
            }
        }

        bool ct_contains(index_t t, const index_t* U, index_t nU) const {
            for(index_t i=0; i<nU; ++i) {
                bool found = false;
                for(index_t lv=0; lv<4; ++lv) {
                    if(ct_v_[4*t+lv] == U[i]) {
                        found = true;
                    }
                }
                if(!found) {
                    return false;
                }
            }
            return true;
        }

        /**
         * \brief Finds all the tetrahedra of the complex that contain a
         *  set of complex vertices.
         */
        void tets_containing(
            const index_t* U, index_t nU, vector<index_t>& tets
        ) const {
            tets.resize(0);
            if(nU == 0) {
                return;
            }
            for(index_t k=cv_tets_ptr_[U[0]]; k<cv_tets_ptr_[U[0]+1]; ++k) {
                index_t t = cv_tets_[k];
                if(ct_contains(t, U, nU)) {
                    tets.push_back(t);
                }
            }
        }

        /**
         * \brief Computes the union of the supports of a set of mesh
         *  vertices (the vertices of the smallest complex simplex that
         *  contains them all, if it is a simplex of the complex).
         * \return the number of complex vertices (nU), or 5 if there are
         *  more than 4.
         */
        index_t carrier_set(
            const index_t* verts, index_t n, index_t U[4]
        ) const {
            index_t nU = 0;
            for(index_t i=0; i<n; ++i) {
                const RVertex& V = rv_[verts[i]];
                for(index_t j=0; j<V.nsup; ++j) {
                    index_t w = V.sup[j];
                    bool found = false;
                    for(index_t k=0; k<nU; ++k) {
                        if(U[k] == w) {
                            found = true;
                        }
                    }
                    if(!found) {
                        if(nU == 4) {
                            return 5;
                        }
                        U[nU++] = w;
                    }
                }
            }
            // sort (at most 4 elements)
            for(index_t i=1; i<nU; ++i) {
                for(index_t j=i; j>0 && U[j] < U[j-1]; --j) {
                    std::swap(U[j], U[j-1]);
                }
            }
            return nU;
        }

        /**
         * \brief Finds a tetrahedron of the complex that contains a set
         *  of complex vertices, or NO_INDEX if there is none.
         */
        index_t find_carrier(const index_t* U, index_t nU) const {
            if(nU == 0 || nU > 4) {
                return NO_INDEX;
            }
            for(index_t k=cv_tets_ptr_[U[0]]; k<cv_tets_ptr_[U[0]+1]; ++k) {
                index_t t = cv_tets_[k];
                if(ct_contains(t, U, nU)) {
                    return t;
                }
            }
            return NO_INDEX;
        }

        /**
         * \brief Computes the parameters of the closest points between
         *  two lines p0 + s d0 and p1 + t d1.
         */
        static void closest_params(
            const vec3& p0, const vec3& d0,
            const vec3& p1, const vec3& d1,
            double& s, double& t
        ) {
            vec3 r = p0 - p1;
            double a = dot(d0,d0);
            double b = dot(d0,d1);
            double c = dot(d1,d1);
            double d = dot(d0,r);
            double e = dot(d1,r);
            double den = a*c - b*b;
            if(std::fabs(den) < 1e-20 * (a*c + 1e-300)) {
                s = 0.0;
                t = (c > 0.0) ? e / c : 0.0;
                return;
            }
            s = (b*e - c*d) / den;
            t = (a*e - b*d) / den;
        }

        static double clamp01(double x) {
            return std::max(0.0, std::min(1.0, x));
        }

        /**
         * \brief Computes the transversal segment through a point x of
         *  a simplex of the complex (Theorem 21 and Section 3.4).
         * \details Let V- and V+ be the inside and outside vertices of
         *  the simplex. The transversal segments are the segments ab with
         *  a in conv(V-) and b in conv(V+). The one through x is unique:
         *  if |V-| = 1 (resp. |V+| = 1), it is on the line through a
         *  (resp. b) and x; if |V-| = |V+| = 2, it is the unique line
         *  through x that meets both edges.
         * \param[in] U , nU the vertices of the simplex
         * \param[in] x the point
         * \param[out] a , b the extremities of the segment (a inside,
         *  b outside)
         * \param[in] clamp if true, the extremities are clamped to the
         *  faces of the simplex (x is expected to be inside). If false,
         *  the family of segments is extended outside the simplex (used
         *  to walk on the mesh).
         * \retval true on success
         * \retval false if the simplex does not intersect the surface
         */
        bool transversal_segment(
            const index_t* U, index_t nU, const vec3& x, vec3& a, vec3& b,
            bool clamp = true
        ) const {
            vec3 In[4];
            vec3 Out[4];
            index_t nIn = 0;
            index_t nOut = 0;
            for(index_t i=0; i<nU; ++i) {
                if(cv_val_[U[i]] < 0.0) {
                    In[nIn++] = cv_pos_[U[i]];
                } else {
                    Out[nOut++] = cv_pos_[U[i]];
                }
            }
            if(nIn == 0 || nOut == 0) {
                return false;
            }
            if(nIn == 1 && nOut == 1) {
                a = In[0];
                b = Out[0];
                return true;
            }
            if(nIn == 1 || nOut == 1) {
                // Central projection from the single vertex onto the
                // opposite face.
                const vec3& apex = (nIn == 1) ? In[0] : Out[0];
                const vec3* face = (nIn == 1) ? Out : In;
                index_t nf = (nIn == 1) ? nOut : nIn;
                vec3 hit;
                if(nf == 2) {
                    double s, t;
                    closest_params(apex, x - apex, face[0], face[1]-face[0], s, t);
                    if(clamp) {
                        t = clamp01(t);
                    }
                    hit = face[0] + t*(face[1]-face[0]);
                } else {
                    vec3 sol;
                    bool ok = solve3_columns(
                        face[0]-apex, face[1]-face[0], face[2]-face[0],
                        x - apex, sol
                    );
                    double lambda = ok ? sol.x : 0.0;
                    double alpha = 0.0;
                    double beta = 0.0;
                    if(ok && lambda > 1e-12) {
                        alpha = sol.y / lambda;
                        beta = sol.z / lambda;
                    } else {
                        alpha = 1.0/3.0;
                        beta = 1.0/3.0;
                    }
                    if(clamp) {
                        // clamp to the triangle
                        alpha = std::max(0.0, alpha);
                        beta = std::max(0.0, beta);
                        if(alpha + beta > 1.0) {
                            double sum = alpha + beta;
                            alpha /= sum;
                            beta /= sum;
                        }
                    }
                    hit = face[0] + alpha*(face[1]-face[0]) + beta*(face[2]-face[0]);
                }
                if(nIn == 1) {
                    a = apex;
                    b = hit;
                } else {
                    a = hit;
                    b = apex;
                }
                return true;
            }
            // nIn == 2 and nOut == 2: the line through x that meets both
            // edges. The plane through x and the inside edge meets the
            // outside edge in b.
            vec3 n = cross(In[1]-In[0], x-In[0]);
            double denom = dot(n, Out[1]-Out[0]);
            double beta = (std::fabs(denom) > 1e-300) ?
                -dot(n, Out[0]-In[0]) / denom : 0.5;
            if(clamp) {
                beta = clamp01(beta);
            }
            b = Out[0] + beta*(Out[1]-Out[0]);
            double s, t;
            closest_params(b, x - b, In[0], In[1]-In[0], s, t);
            if(clamp) {
                t = clamp01(t);
            }
            a = In[0] + t*(In[1]-In[0]);
            return true;
        }

        /**
         * \brief Moves a point of a simplex of the complex to the skin
         *  surface, along the transversal segment through it (Lemma 24).
         * \param[in] U , nU the vertices of the simplex
         * \param[in] carrier a tetrahedron that contains the simplex
         *  (used for the quadric)
         * \param[in,out] x the point, moved to the surface
         * \retval true on success
         */
        bool move_to_surface(
            const index_t* U, index_t nU, index_t carrier, vec3& x
        ) const {
            vec3 a, b;
            if(!transversal_segment(U, nU, x, a, b)) {
                return false;
            }
            index_t q = ct_q_[carrier];
            double fa = quadrics_[q].eval(a);
            double fb = quadrics_[q].eval(b);
            if(!(fa < 0.0 && fb > 0.0)) {
                // Numerical issue (a or b too close to the surface),
                // extend the segment slightly along its direction.
                vec3 d = b - a;
                a = a - 1e-6*d;
                b = b + 1e-6*d;
                fa = quadrics_[q].eval(a);
                fb = quadrics_[q].eval(b);
                if(!(fa < 0.0 && fb > 0.0)) {
                    return false;
                }
            }
            x = segment_root(a, b, fa, fb, q);
            return true;
        }

        /**
         * \brief Computes the inward-oriented plane of a facet of a
         *  tetrahedron of the complex.
         */
        void ct_facet_plane(index_t t, index_t lf, vec3& p, vec3& n) const {
            const vec3& a = cv_pos_[ct_v_[4*t + tet_facet_vertex[lf][0]]];
            const vec3& b = cv_pos_[ct_v_[4*t + tet_facet_vertex[lf][1]]];
            const vec3& c = cv_pos_[ct_v_[4*t + tet_facet_vertex[lf][2]]];
            const vec3& d = cv_pos_[ct_v_[4*t + lf]];
            p = a;
            n = cross(b-a, c-a);
            if(dot(n, d-a) < 0.0) {
                n = -1.0*n;
            }
        }

        /**
         * \brief Tests whether a point is inside a tetrahedron of the
         *  complex (with a relative tolerance).
         */
        bool ct_contains_point(index_t t, const vec3& x) const {
            for(index_t lf=0; lf<4; ++lf) {
                vec3 p, n;
                ct_facet_plane(t, lf, p, n);
                if(dot(n, x-p) < -1e-9*length(n)*length(x-p)) {
                    return false;
                }
            }
            return true;
        }

        /**
         * \brief Clips a triangle by a tetrahedron of the complex
         *  (Sutherland-Hodgman).
         * \param[out] poly the clipped polygon (empty if the triangle
         *  does not intersect the tetrahedron)
         */
        void clip_triangle_by_tet(
            index_t t, const vec3& p0, const vec3& p1, const vec3& p2,
            vector<vec3>& poly
        ) {
            poly.resize(0);
            poly.push_back(p0);
            poly.push_back(p1);
            poly.push_back(p2);
            for(index_t lf=0; lf<4 && poly.size() != 0; ++lf) {
                vec3 p, n;
                ct_facet_plane(t, lf, p, n);
                tmp_poly_.resize(0);
                index_t np = poly.size();
                for(index_t i=0; i<np; ++i) {
                    const vec3& A = poly[i];
                    const vec3& B = poly[(i+1)%np];
                    double da = dot(n, A-p);
                    double db = dot(n, B-p);
                    if(da >= 0.0) {
                        tmp_poly_.push_back(A);
                    }
                    if((da >= 0.0) != (db >= 0.0)) {
                        double tt = da / (da - db);
                        tmp_poly_.push_back(A + tt*(B-A));
                    }
                }
                poly.swap(tmp_poly_);
            }
        }

        static double polygon_area(const vector<vec3>& poly, const vec3& n_unit) {
            double a = 0.0;
            for(index_t i=1; i+1<poly.size(); ++i) {
                a += dot(n_unit, cross(poly[i]-poly[0], poly[i+1]-poly[0]));
            }
            return 0.5*a;
        }

        /**
         * \brief Collects the tetrahedra of the complex that may be
         *  crossed by a triangle: the tetrahedra incident to the support
         *  vertices of its vertices (and optionally their neighbors).
         */
        void candidate_tets(
            const index_t* verts, index_t nv, bool two_rings,
            vector<index_t>& tets
        ) {
            tets.resize(0);
            ++tet_stamp_;
            if(tet_mark_.size() != nb_complex_tets()) {
                tet_mark_.assign(nb_complex_tets(), 0);
            }
            for(index_t i=0; i<nv; ++i) {
                const RVertex& V = rv_[verts[i]];
                for(index_t j=0; j<V.nsup; ++j) {
                    index_t sv = V.sup[j];
                    for(index_t k=cv_tets_ptr_[sv]; k<cv_tets_ptr_[sv+1]; ++k) {
                        index_t t = cv_tets_[k];
                        if(tet_mark_[t] != tet_stamp_) {
                            tet_mark_[t] = tet_stamp_;
                            tets.push_back(t);
                        }
                    }
                }
            }
            if(two_rings) {
                index_t n1 = tets.size();
                for(index_t i=0; i<n1; ++i) {
                    for(index_t lv=0; lv<4; ++lv) {
                        index_t sv = ct_v_[4*tets[i]+lv];
                        for(index_t k=cv_tets_ptr_[sv]; k<cv_tets_ptr_[sv+1]; ++k) {
                            index_t t = cv_tets_[k];
                            if(tet_mark_[t] != tet_stamp_) {
                                tet_mark_[t] = tet_stamp_;
                                tets.push_back(t);
                            }
                        }
                    }
                }
            }
        }

        /**
         * \brief Finds the tetrahedron of the complex that contains a
         *  point, among the candidates of some mesh vertices.
         */
        index_t find_tet_containing(
            const index_t* verts, index_t nv, const vec3& x, index_t hint
        ) {
            if(hint != NO_INDEX && ct_contains_point(hint, x)) {
                return hint;
            }
            for(index_t pass=0; pass<2; ++pass) {
                candidate_tets(verts, nv, pass==1, tmp_cand_);
                for(index_t t : tmp_cand_) {
                    if(ct_contains_point(t, x)) {
                        return t;
                    }
                }
            }
            return NO_INDEX;
        }

        /**
         * \brief Tests whether a triangle of the mesh is transversal to
         *  the family of transversal segments of the tetrahedra of the
         *  complex that it crosses, with the right orientation.
         * \details This is the local condition that makes the mesh,
         *  restricted to a tetrahedron, a "graph" over the family of
         *  transversal segments (proof of Theorem 21): at each point x of
         *  the triangle inside tetrahedron c, the direction
         *  d(x) = b(x) - a(x) of the segment of c through x (from inside
         *  to outside) must cross the (outward oriented) triangle from
         *  inside to outside, i.e. n . d(x) > 0. The triangle is clipped
         *  by each tetrahedron it crosses; in the central projection cases
         *  (one inside or one outside vertex) n . d(x) is affine so it is
         *  sufficient to test the vertices of the clipped polygon; in the
         *  2-2 case, the centroid is tested as well. The condition of
         *  Lemma 23 of the paper is a sufficient condition that only works
         *  when the triangles are large with respect to the tetrahedra;
         *  the present condition works at any resolution.
         *  Two adjacent triangles that both satisfy the condition have
         *  disjoint projections: this is the invariant maintained by
         *  refine() (an edge flip is valid iff the two new triangles
         *  satisfy it, which means that the projected quad is convex, i.e.
         *  that the new triangles are crossed by the same transversal
         *  segments as the old ones, Section 3.4).
         * \param[in] v0 , v1 , v2 the vertices of the (oriented) triangle
         * \param[out] carrier the tetrahedron that contains the largest
         *  part of the triangle
         */
        bool isotopy_test(
            index_t v0, index_t v1, index_t v2, index_t& carrier
        ) {
            const vec3& p0 = rv_[v0].p;
            const vec3& p1 = rv_[v1].p;
            const vec3& p2 = rv_[v2].p;
            vec3 n = cross(p1 - p0, p2 - p0);
            double nl2 = length2(n);
            if(nl2 == 0.0) {
                return false;
            }
            vec3 n_unit = (1.0/std::sqrt(nl2))*n;
            double tri_area = 0.5*std::sqrt(nl2);
            index_t verts[3] = {v0, v1, v2};
            carrier = NO_INDEX;
            for(index_t pass=0; pass<2; ++pass) {
                candidate_tets(verts, 3, pass==1, tmp_cand_);
                double area = 0.0;
                double best_area = 0.0;
                for(index_t t : tmp_cand_) {
                    // quick reject
                    bool out = false;
                    for(index_t lf=0; lf<4 && !out; ++lf) {
                        vec3 p, nn;
                        ct_facet_plane(t, lf, p, nn);
                        if(dot(nn,p0-p) < 0.0 && dot(nn,p1-p) < 0.0 && dot(nn,p2-p) < 0.0) {
                            out = true;
                        }
                    }
                    if(out) {
                        continue;
                    }
                    clip_triangle_by_tet(t, p0, p1, p2, tmp_clip_);
                    if(tmp_clip_.size() < 3) {
                        continue;
                    }
                    double a = polygon_area(tmp_clip_, n_unit);
                    if(a <= 1e-12*tri_area) {
                        continue;
                    }
                    area += a;
                    if(a > best_area) {
                        best_area = a;
                        carrier = t;
                    }
                    const index_t* T = &(ct_v_[4*t]);
                    vec3 g(0.0, 0.0, 0.0);
                    for(const vec3& x : tmp_clip_) {
                        g += x;
                    }
                    g = (1.0/double(tmp_clip_.size()))*g;
                    tmp_clip_.push_back(g);
                    for(const vec3& x : tmp_clip_) {
                        vec3 a3, b3;
                        if(!transversal_segment(T, 4, x, a3, b3, false)) {
                            return false;
                        }
                        if(!(dot(n, b3 - a3) > 0.0)) {
                            return false;
                        }
                    }
                }
                if(area >= (1.0 - 1e-6)*tri_area) {
                    return carrier != NO_INDEX;
                }
                // The triangle crosses tetrahedra that are not in the
                // candidates: retry with a larger neighborhood.
            }
            return false;
        }

        /**
         * \brief Counts the triangles of the mesh that do not satisfy
         *  the transversality condition (see isotopy_test()).
         */
        index_t check_isotopy() {
            index_t nb_bad = 0;
            for(index_t t=0; t<nb_rtriangles(); ++t) {
                if(!rt_alive_[t]) {
                    continue;
                }
                index_t carrier;
                if(!isotopy_test(
                       rt_vertex(t,0), rt_vertex(t,1), rt_vertex(t,2), carrier
                   )
                ) {
                    ++nb_bad;
                }
            }
            return nb_bad;
        }

        /*****************************************************************/
        /** Subdivision (Section 3.4, sqrt-3 method generalized)         */
        /*****************************************************************/

        /**
         * \brief Subdivides each triangle into n^2 triangles (n=3 is the
         *  result of two sqrt-3 subdivision steps, as in the paper) and
         *  moves the new vertices to the skin surface along the
         *  transversal segments (Lemma 24).
         * \retval true on success
         */
        bool subdivide(index_t n) {
            if(n < 2) {
                return true;
            }
            index_t nt0 = nb_rtriangles();
            // Vertices on the edges: shared by the two incident triangles.
            // key: (u,v) with u<v, the n-1 vertices from u to v.
            std::unordered_map<Numeric::uint64, index_t> edge_pts;
            vector<index_t> epts;
            index_t nb_failed = 0;
            auto edge_vertices = [&](index_t u, index_t v) -> index_t {
                Numeric::uint64 key = edge_key(u,v);
                auto it = edge_pts.find(key);
                if(it != edge_pts.end()) {
                    return it->second;
                }
                index_t umin = std::min(u,v);
                index_t vmax = std::max(u,v);
                index_t verts[2] = {umin, vmax};
                index_t first = epts.size();
                index_t hint = NO_INDEX;
                for(index_t i=1; i<n; ++i) {
                    double lambda = double(i)/double(n);
                    vec3 x = (1.0-lambda)*rv_[umin].p + lambda*rv_[vmax].p;
                    index_t c = find_tet_containing(verts, 2, x, hint);
                    if(c == NO_INDEX) {
                        ++nb_failed;
                        index_t U[2] = {umin, vmax}; // unused support
                        epts.push_back(new_rvertex(x, U, 0));
                        continue;
                    }
                    hint = c;
                    const index_t* U = &(ct_v_[4*c]);
                    if(!move_to_surface(U, 4, c, x)) {
                        ++nb_failed;
                    }
                    epts.push_back(new_rvertex(x, U, 4));
                }
                edge_pts[key] = first;
                return first;
            };

            vector<index_t> lattice((n+1)*(n+2)/2);
            for(index_t t=0; t<nt0; ++t) {
                if(!rt_alive_[t]) {
                    continue;
                }
                index_t v[3] = { rt_vertex(t,0), rt_vertex(t,1), rt_vertex(t,2) };
                index_t carrier = rt_carrier_[t];
                // lattice point (i,j): barycentric (i/n, j/n, 1-i/n-j/n)
                // on vertices (v1, v2, v0). index = j*(n+1) - j(j-1)/2 + i
                auto idx = [n](index_t i, index_t j) -> index_t {
                    return j*(n+1) - (j*(j-1))/2 + i;
                };
                // corners
                lattice[idx(0,0)] = v[0];
                lattice[idx(n,0)] = v[1];
                lattice[idx(0,n)] = v[2];
                // edges: (v0,v1): i=1..n-1, j=0; (v1,v2): i+j=n;
                // (v2,v0): i=0, j=1..n-1
                {
                    index_t f01 = edge_vertices(v[0], v[1]);
                    index_t f12 = edge_vertices(v[1], v[2]);
                    index_t f20 = edge_vertices(v[2], v[0]);
                    for(index_t i=1; i<n; ++i) {
                        // point i/n from v0 to v1: stored from min to max
                        index_t k01 = (v[0] < v[1]) ? i-1 : n-1-i;
                        lattice[idx(i,0)] = epts[f01 + k01];
                        // point at i/n from v1 to v2: (n-i, i)
                        index_t k12 = (v[1] < v[2]) ? i-1 : n-1-i;
                        lattice[idx(n-i,i)] = epts[f12 + k12];
                        // point at i/n from v2 to v0: (0, n-i)
                        index_t k20 = (v[2] < v[0]) ? i-1 : n-1-i;
                        lattice[idx(0,n-i)] = epts[f20 + k20];
                    }
                }
                // interior points
                for(index_t j=1; j<n; ++j) {
                    for(index_t i=1; i+j<n; ++i) {
                        double a = double(i)/double(n);
                        double b = double(j)/double(n);
                        vec3 x = (1.0-a-b)*rv_[v[0]].p + a*rv_[v[1]].p + b*rv_[v[2]].p;
                        index_t c = find_tet_containing(v, 3, x, carrier);
                        if(c == NO_INDEX) {
                            ++nb_failed;
                            lattice[idx(i,j)] = new_rvertex(x, v, 0);
                            continue;
                        }
                        const index_t* U = &(ct_v_[4*c]);
                        if(!move_to_surface(U, 4, c, x)) {
                            ++nb_failed;
                        }
                        lattice[idx(i,j)] = new_rvertex(x, U, 4);
                    }
                }
                // sub-triangles
                auto add_sub = [&](index_t a, index_t b, index_t cc) {
                    index_t tt = new_rtriangle(a, b, cc, carrier);
                    index_t sv[3] = {a, b, cc};
                    vec3 g = (1.0/3.0)*(rv_[a].p + rv_[b].p + rv_[cc].p);
                    index_t c2 = find_tet_containing(sv, 3, g, carrier);
                    if(c2 != NO_INDEX) {
                        rt_carrier_[tt] = c2;
                    }
                };
                for(index_t j=0; j<n; ++j) {
                    for(index_t i=0; i+j<n; ++i) {
                        add_sub(lattice[idx(i,j)], lattice[idx(i+1,j)], lattice[idx(i,j+1)]);
                        if(i+j+1 < n) {
                            add_sub(lattice[idx(i+1,j)], lattice[idx(i+1,j+1)], lattice[idx(i,j+1)]);
                        }
                    }
                }
                rt_alive_[t] = false;
            }
            rebuild_adjacency();
            if(nb_failed != 0 && verbose_) {
                Logger::warn("skin")
                    << nb_failed << " vertices could not be moved to the surface"
                    << std::endl;
            }
            repair_transversality();
            return true;
        }

        /**
         * \brief Repairs the (rare) triangles that do not satisfy the
         *  transversality condition by flipping their edges.
         * \details Subdivision in a tetrahedron with two inside and two
         *  outside vertices can create sliver triangles that are not
         *  graphs over the (non-linear) family of transversal segments.
         *  Flipping an edge of such a triangle usually repairs it.
         * \return the number of triangles that could not be repaired
         */
        index_t repair_transversality() {
            index_t nb_bad = 0;
            index_t nb_repaired = 0;
            vector<index_t> around;
            for(index_t round=0; round<5; ++round) {
                nb_bad = 0;
                for(index_t t=0; t<nb_rtriangles(); ++t) {
                    if(!rt_alive_[t]) {
                        continue;
                    }
                    index_t c;
                    if(isotopy_test(rt_vertex(t,0), rt_vertex(t,1), rt_vertex(t,2), c)) {
                        continue;
                    }
                    bool repaired = false;
                    // try to flip an edge
                    for(index_t k=0; k<3 && !repaired; ++k) {
                        index_t n = rt_adjacent(t,k);
                        if(n == NO_INDEX) {
                            continue;
                        }
                        index_t u = rt_vertex(t,k);
                        index_t v = rt_vertex(t,k+1);
                        index_t p = rt_vertex(t,k+2);
                        index_t kn = rt_side(n, v, u);
                        if(kn == NO_INDEX) {
                            continue;
                        }
                        index_t q = rt_vertex(n, kn+2);
                        if(q == p || edge_exists(p, q, t)) {
                            continue;
                        }
                        index_t ct, cn;
                        if(isotopy_test(u, q, p, ct) && isotopy_test(v, p, q, cn)) {
                            index_t nt[2];
                            flip_edge(t, k, ct, cn, nt);
                            repaired = true;
                        }
                    }
                    // try to collapse an edge (shortest first)
                    if(!repaired) {
                        index_t order[3] = {0, 1, 2};
                        double len[3];
                        for(index_t k=0; k<3; ++k) {
                            len[k] = length(rv_[rt_vertex(t,k+1)].p - rv_[rt_vertex(t,k)].p);
                        }
                        std::sort(order, order+3, [&](index_t a, index_t b) {
                            return len[a] < len[b];
                        });
                        for(index_t i=0; i<3 && !repaired; ++i) {
                            if(try_collapse(t, order[i], around)) {
                                repaired = true;
                            } else {
                                // try the other direction
                                index_t n = rt_adjacent(t, order[i]);
                                if(n != NO_INDEX) {
                                    index_t kn = rt_side(
                                        n, rt_vertex(t,order[i]+1), rt_vertex(t,order[i])
                                    );
                                    if(kn != NO_INDEX && try_collapse(n, kn, around)) {
                                        repaired = true;
                                    }
                                }
                            }
                        }
                    }
                    if(repaired) {
                        ++nb_repaired;
                    } else {
                        ++nb_bad;
                    }
                }
                if(nb_bad == 0) {
                    break;
                }
            }
            if(verbose_ && (nb_repaired != 0 || nb_bad != 0)) {
                Logger::out("skin")
                    << "Transversality repair: " << nb_repaired
                    << " triangles repaired, " << nb_bad << " remaining"
                    << std::endl;
            }
            return nb_bad;
        }

        /**
         * \brief Tests whether an edge (p,q) exists in the mesh, by
         *  turning around p starting from a triangle incident to p.
         */
        bool edge_exists(index_t p, index_t q, index_t t_start) {
            if(!triangles_around(p, t_start, tmp_tris_)) {
                return true; // be conservative
            }
            for(index_t tp : tmp_tris_) {
                for(index_t i=0; i<3; ++i) {
                    if(rt_vertex(tp,i) == q) {
                        return true;
                    }
                }
            }
            return false;
        }

        /*****************************************************************/
        /** Chew-style refinement (Section 3.4)                           */
        /*****************************************************************/

        /**
         * \brief Computes the maximum absolute principal curvature of
         *  the skin surface at a point, for a given mixed cell.
         * \details The quadric is I(x) = (x-c)^T M (x-c) - W, its
         *  gradient is g = 2 M (x-c) and its Hessian is H = 2 M. The
         *  principal curvatures are the two non-zero eigenvalues of
         *  P H P / |g|, with P = I - n n^T, n = g/|g|.
         */
        double max_curvature(const Quadric& Q, const vec3& x) const {
            double M[3][3];
            double sa = 1.0 / Q.s;
            double sb = -1.0 / (1.0 - Q.s);
            vec3 d = Q.axis;
            for(index_t i=0; i<3; ++i) {
                for(index_t j=0; j<3; ++j) {
                    double I = (i==j) ? 1.0 : 0.0;
                    double D = d[i]*d[j];
                    switch(Q.ell) {
                    case 0:  M[i][j] = sa*I; break;
                    case 1:  M[i][j] = sb*D + sa*(I - D); break;
                    case 2:  M[i][j] = sb*(I - D) + sa*D; break;
                    default: M[i][j] = sb*I; break;
                    }
                }
            }
            vec3 y = x - Q.c;
            vec3 g;
            for(index_t i=0; i<3; ++i) {
                g[i] = 2.0*(M[i][0]*y[0] + M[i][1]*y[1] + M[i][2]*y[2]);
            }
            double gl = length(g);
            if(gl < 1e-300) {
                return Numeric::max_float64();
            }
            vec3 n = (1.0/gl)*g;
            // A = P H P / |g|, H = 2M, P = I - n n^T
            double P[3][3], HP[3][3], A[3][3];
            for(index_t i=0; i<3; ++i) {
                for(index_t j=0; j<3; ++j) {
                    P[i][j] = ((i==j) ? 1.0 : 0.0) - n[i]*n[j];
                }
            }
            for(index_t i=0; i<3; ++i) {
                for(index_t j=0; j<3; ++j) {
                    HP[i][j] = 0.0;
                    for(index_t k=0; k<3; ++k) {
                        HP[i][j] += 2.0*M[i][k]*P[k][j];
                    }
                }
            }
            double tr = 0.0;
            double F2 = 0.0;
            for(index_t i=0; i<3; ++i) {
                for(index_t j=0; j<3; ++j) {
                    A[i][j] = 0.0;
                    for(index_t k=0; k<3; ++k) {
                        A[i][j] += P[i][k]*HP[k][j];
                    }
                    A[i][j] /= gl;
                    F2 += A[i][j]*A[i][j];
                }
                tr += A[i][i];
            }
            // eigenvalues k1,k2 (the third one is 0):
            // k1+k2 = tr, k1^2+k2^2 = F2
            double disc = std::max(0.0, 2.0*F2 - tr*tr);
            double k1 = 0.5*(tr + std::sqrt(disc));
            double k2 = 0.5*(tr - std::sqrt(disc));
            return std::max(std::fabs(k1), std::fabs(k2));
        }

        /**
         * \brief Gets the target size (maximum circumradius) of the
         *  triangles at a point of a triangle.
         */
        double target_size(index_t t, const vec3& x) const {
            double h = (refine_params_.max_edge_length > 0.0) ?
                refine_params_.max_edge_length : Numeric::max_float64();
            if(refine_params_.size_factor > 0.0) {
                double k = max_curvature(quadrics_[ct_q_[rt_carrier_[t]]], x);
                if(k > 0.0) {
                    h = std::min(h, refine_params_.size_factor / k);
                }
            }
            return h;
        }

        static double triangle_min_angle(
            const vec3& p0, const vec3& p1, const vec3& p2
        ) {
            double a = length(p1-p2);
            double b = length(p2-p0);
            double c = length(p0-p1);
            double m = std::min(a, std::min(b,c));
            if(a*b*c == 0.0) {
                return 0.0;
            }
            // angle opposite to the smallest edge
            double cosang;
            if(m == a) {
                cosang = (b*b + c*c - a*a) / (2.0*b*c);
            } else if(m == b) {
                cosang = (a*a + c*c - b*b) / (2.0*a*c);
            } else {
                cosang = (a*a + b*b - c*c) / (2.0*a*b);
            }
            cosang = std::max(-1.0, std::min(1.0, cosang));
            return std::acos(cosang);
        }

        static vec3 circumcenter(
            const vec3& p0, const vec3& p1, const vec3& p2, double& R
        ) {
            vec3 a = p1 - p0;
            vec3 b = p2 - p0;
            vec3 axb = cross(a,b);
            double d = 2.0*length2(axb);
            if(d < 1e-300) {
                R = Numeric::max_float64();
                return (1.0/3.0)*(p0+p1+p2);
            }
            vec3 c = (1.0/d)*cross(length2(a)*b - length2(b)*a, axb);
            R = length(c);
            return p0 + c;
        }

        /**
         * \brief Tests whether a triangle needs to be refined (too
         *  small angle or too large circumradius).
         */
        bool is_bad(index_t t) const {
            const vec3& p0 = rv_[rt_vertex(t,0)].p;
            const vec3& p1 = rv_[rt_vertex(t,1)].p;
            const vec3& p2 = rv_[rt_vertex(t,2)].p;
            double R;
            circumcenter(p0, p1, p2, R);
            vec3 g = (1.0/3.0)*(p0+p1+p2);
            double h = target_size(t, g);
            if(R > h) {
                return true;
            }
            // Do not try to improve very small triangles (relative to
            // the target size), this avoids infinite refinement.
            if(R < refine_params_.min_size_factor * h) {
                return false;
            }
            double min_angle = triangle_min_angle(p0, p1, p2);
            return min_angle < refine_params_.min_angle_deg * 3.14159265358979323846 / 180.0;
        }

        /**
         * \brief Splits a triangle into three by inserting a vertex.
         * \param[in] t the triangle
         * \param[in] x the position of the new vertex (on the surface)
         * \param[in] U , nU the support of the new vertex
         * \param[out] new_tris the three triangles
         */
        index_t split_triangle(
            index_t t, const vec3& x, const index_t* U, index_t nU,
            index_t new_tris[3]
        ) {
            index_t w = new_rvertex(x, U, nU);
            index_t v0 = rt_vertex(t,0);
            index_t v1 = rt_vertex(t,1);
            index_t v2 = rt_vertex(t,2);
            index_t a0 = rt_adjacent(t,0);
            index_t a1 = rt_adjacent(t,1);
            index_t a2 = rt_adjacent(t,2);
            index_t carrier = rt_carrier_[t];
            index_t t1 = new_rtriangle(v1, v2, w, carrier);
            index_t t2 = new_rtriangle(v2, v0, w, carrier);
            rt_v_[3*t+2] = w;
            rt_adjacent(t,0) = a0;
            rt_adjacent(t,1) = t1;
            rt_adjacent(t,2) = t2;
            rt_adjacent(t1,0) = a1;
            rt_adjacent(t1,1) = t2;
            rt_adjacent(t1,2) = t;
            rt_adjacent(t2,0) = a2;
            rt_adjacent(t2,1) = t;
            rt_adjacent(t2,2) = t1;
            rt_replace_adjacent(a1, t, t1);
            rt_replace_adjacent(a2, t, t2);
            new_tris[0] = t;
            new_tris[1] = t1;
            new_tris[2] = t2;
            return w;
        }

        /**
         * \brief Splits an edge (and its two incident triangles) by
         *  inserting a vertex.
         * \param[in] t , k the edge (side k of triangle t)
         * \param[in] x the position of the new vertex (on the surface)
         * \param[in] U , nU the support of the new vertex
         * \param[out] new_tris the four triangles
         */
        index_t split_edge(
            index_t t, index_t k, const vec3& x, const index_t* U, index_t nU,
            index_t new_tris[4]
        ) {
            index_t n = rt_adjacent(t,k);
            index_t u = rt_vertex(t,k);
            index_t v = rt_vertex(t,k+1);
            index_t p = rt_vertex(t,k+2);
            index_t kn = rt_side(n, v, u);
            index_t q = rt_vertex(n, kn+2);
            index_t w = new_rvertex(x, U, nU);
            index_t t_out1 = rt_adjacent(t, k+1);  // across (v,p)
            index_t t_out2 = rt_adjacent(t, k+2);  // across (p,u)
            index_t n_out1 = rt_adjacent(n, kn+1); // across (u,q)
            index_t n_out2 = rt_adjacent(n, kn+2); // across (q,v)
            index_t ct = rt_carrier_[t];
            index_t cn = rt_carrier_[n];
            // t  = (u,w,p), t1 = (w,v,p), n = (v,w,q), n1 = (w,u,q)
            index_t t1 = new_rtriangle(w, v, p, ct);
            index_t n1 = new_rtriangle(w, u, q, cn);
            rt_v_[3*t] = u;   rt_v_[3*t+1] = w;   rt_v_[3*t+2] = p;
            rt_v_[3*n] = v;   rt_v_[3*n+1] = w;   rt_v_[3*n+2] = q;
            rt_adjacent(t,0) = n1;     rt_adjacent(t,1) = t1;     rt_adjacent(t,2) = t_out2;
            rt_adjacent(t1,0) = n;     rt_adjacent(t1,1) = t_out1; rt_adjacent(t1,2) = t;
            rt_adjacent(n,0) = t1;     rt_adjacent(n,1) = n1;     rt_adjacent(n,2) = n_out2;
            rt_adjacent(n1,0) = t;     rt_adjacent(n1,1) = n_out1; rt_adjacent(n1,2) = n;
            rt_replace_adjacent(t_out1, t, t1);
            rt_replace_adjacent(n_out1, n, n1);
            new_tris[0] = t;
            new_tris[1] = t1;
            new_tris[2] = n;
            new_tris[3] = n1;
            return w;
        }

        /**
         * \brief Flips an edge.
         * \param[in] t , k the edge (side k of triangle t)
         * \param[in] carrier_t , carrier_n the carriers of the new
         *  triangles (u,q,p) and (v,p,q)
         * \param[out] new_tris the two new triangles
         */
        void flip_edge(
            index_t t, index_t k, index_t carrier_t, index_t carrier_n,
            index_t new_tris[2]
        ) {
            index_t n = rt_adjacent(t,k);
            index_t u = rt_vertex(t,k);
            index_t v = rt_vertex(t,k+1);
            index_t p = rt_vertex(t,k+2);
            index_t kn = rt_side(n, v, u);
            index_t q = rt_vertex(n, kn+2);
            index_t t_out1 = rt_adjacent(t, k+1);  // across (v,p)
            index_t t_out2 = rt_adjacent(t, k+2);  // across (p,u)
            index_t n_out1 = rt_adjacent(n, kn+1); // across (u,q)
            index_t n_out2 = rt_adjacent(n, kn+2); // across (q,v)
            // t = (u,q,p), n = (v,p,q)
            rt_v_[3*t] = u;   rt_v_[3*t+1] = q;   rt_v_[3*t+2] = p;
            rt_v_[3*n] = v;   rt_v_[3*n+1] = p;   rt_v_[3*n+2] = q;
            rt_adjacent(t,0) = n_out1; rt_adjacent(t,1) = n;      rt_adjacent(t,2) = t_out2;
            rt_adjacent(n,0) = t_out1; rt_adjacent(n,1) = t;      rt_adjacent(n,2) = n_out2;
            rt_replace_adjacent(n_out1, n, t);
            rt_replace_adjacent(t_out1, t, n);
            rt_carrier_[t] = carrier_t;
            rt_carrier_[n] = carrier_n;
            new_tris[0] = t;
            new_tris[1] = n;
        }

        /**
         * \brief Tests whether an edge should be flipped to improve the
         *  Delaunay property (sum of the opposite angles greater than pi)
         *  and whether it can be flipped while maintaining the isotopy.
         * \param[out] carrier_t , carrier_n the carriers of the new
         *  triangles, if the edge can be flipped
         */
        bool flip_wanted_and_safe(
            index_t t, index_t k, index_t& carrier_t, index_t& carrier_n
        ) {
            index_t n = rt_adjacent(t,k);
            if(n == NO_INDEX || n == t) {
                return false;
            }
            index_t u = rt_vertex(t,k);
            index_t v = rt_vertex(t,k+1);
            index_t p = rt_vertex(t,k+2);
            index_t kn = rt_side(n, v, u);
            if(kn == NO_INDEX) {
                return false;
            }
            index_t q = rt_vertex(n, kn+2);
            if(q == p) {
                return false;
            }
            // Delaunay criterion (angles at p and q opposite to (u,v))
            const vec3& pu = rv_[u].p;
            const vec3& pv = rv_[v].p;
            const vec3& pp = rv_[p].p;
            const vec3& pq = rv_[q].p;
            auto angle = [](const vec3& a, const vec3& b) {
                double c = dot(a,b) / std::sqrt(length2(a)*length2(b) + 1e-300);
                return std::acos(std::max(-1.0, std::min(1.0, c)));
            };
            double ap = angle(pu-pp, pv-pp);
            double aq = angle(pu-pq, pv-pq);
            if(ap + aq <= 3.14159265358979323846 + 1e-9) {
                return false;
            }
            ++nb_flips_wanted_;
            // the new edge (p,q) should not already exist
            if(edge_exists(p, q, t)) {
                return false;
            }
            // isotopy test (and fold test) on the new triangles
            // (u,q,p) and (v,p,q)
            if(!isotopy_test(u, q, p, carrier_t)) {
                ++nb_flips_refused_;
                return false;
            }
            if(!isotopy_test(v, p, q, carrier_n)) {
                ++nb_flips_refused_;
                return false;
            }
            ++nb_flips_done_;
            return true;
        }

        /**
         * \brief Restores the Delaunay property around some triangles
         *  by edge flips (only isotopy-safe flips are done).
         * \param[in,out] stack the triangles to process (emptied)
         * \param[in,out] to_check receives the modified triangles
         */
        void lawson_flips(
            vector<index_t>& stack, vector<index_t>& to_check
        ) {
            index_t nb_flips = 0;
            while(stack.size() != 0 && nb_flips < refine_params_.max_flips_per_insertion) {
                index_t t = stack.back();
                stack.pop_back();
                if(!rt_alive_[t]) {
                    continue;
                }
                for(index_t k=0; k<3; ++k) {
                    index_t ct, cn;
                    if(flip_wanted_and_safe(t, k, ct, cn)) {
                        index_t nt[2];
                        flip_edge(t, k, ct, cn, nt);
                        stack.push_back(nt[0]);
                        stack.push_back(nt[1]);
                        to_check.push_back(nt[0]);
                        to_check.push_back(nt[1]);
                        ++nb_flips;
                        break;
                    }
                }
            }
            stack.resize(0);
        }

        /**
         * \brief Locates the triangle crossed by the transversal segment
         *  through a point of the surface, by walking on the mesh from a
         *  starting triangle.
         * \details For each visited triangle T, the transversal segment
         *  through \p xs in the carrier of T is intersected with the
         *  plane of T.
         * \param[in] t the starting triangle
         * \param[in] xs the point (on the surface)
         * \param[out] bary the barycentric coordinates of the
         *  intersection in the found triangle
         * \return the triangle, or NO_INDEX if the walk failed
         */
        index_t locate(index_t t, const vec3& xs, double bary[3]) {
            index_t prev = NO_INDEX;
            for(index_t iter=0; iter<refine_params_.max_walk_steps; ++iter) {
                const vec3& p0 = rv_[rt_vertex(t,0)].p;
                const vec3& p1 = rv_[rt_vertex(t,1)].p;
                const vec3& p2 = rv_[rt_vertex(t,2)].p;
                vec3 n = cross(p1-p0, p2-p0);
                double area2 = length2(n);
                if(area2 < 1e-300) {
                    return NO_INDEX;
                }
                // intersection of the transversal segment through xs
                // with the plane of t
                const index_t* T = &(ct_v_[4*rt_carrier_[t]]);
                vec3 a, b;
                if(!transversal_segment(T, 4, xs, a, b, false)) {
                    return NO_INDEX;
                }
                vec3 d = b - a;
                double denom = dot(n, d);
                if(std::fabs(denom) < 1e-300) {
                    return NO_INDEX;
                }
                double lambda = dot(n, p0 - a) / denom;
                vec3 x = a + lambda*d;
                bary[0] = dot(n, cross(p1-x, p2-x)) / area2;
                bary[1] = dot(n, cross(p2-x, p0-x)) / area2;
                bary[2] = dot(n, cross(p0-x, p1-x)) / area2;
                index_t kmin = 0;
                for(index_t k=1; k<3; ++k) {
                    if(bary[k] < bary[kmin]) {
                        kmin = k;
                    }
                }
                if(bary[kmin] >= -1e-9) {
                    return t;
                }
                if(iter + 1 == refine_params_.max_walk_steps) {
                    return NO_INDEX;
                }
                // cross the edge opposite to vertex kmin, i.e.
                // side (kmin+1, kmin+2)
                index_t next = rt_adjacent(t, kmin+1);
                if(next == NO_INDEX || next == prev) {
                    // try the second most negative
                    index_t k2 = NO_INDEX;
                    for(index_t k=0; k<3; ++k) {
                        if(k != kmin && bary[k] < 0.0 && rt_adjacent(t,k+1) != prev) {
                            k2 = k;
                        }
                    }
                    if(k2 == NO_INDEX) {
                        return NO_INDEX;
                    }
                    next = rt_adjacent(t, k2+1);
                }
                prev = t;
                t = next;
            }
            return NO_INDEX;
        }

        /**
         * \brief Inserts a vertex inside a triangle, at the point of the
         *  surface on the transversal segment through \p x.
         * \retval true on success
         */
        bool insert_in_triangle(
            index_t t, vec3 x, vector<index_t>& to_check
        ) {
            index_t verts[3] = { rt_vertex(t,0), rt_vertex(t,1), rt_vertex(t,2) };
            index_t c = find_tet_containing(verts, 3, x, rt_carrier_[t]);
            if(c == NO_INDEX) {
                return false;
            }
            const index_t* U = &(ct_v_[4*c]);
            index_t nU = 4;
            if(!move_to_surface(U, nU, c, x)) {
                return false;
            }
            // save state for undo
            index_t save_v[3] = { verts[0], verts[1], verts[2] };
            index_t save_adj[3] = { rt_adjacent(t,0), rt_adjacent(t,1), rt_adjacent(t,2) };
            index_t nv0 = rv_.size();
            index_t nt0 = nb_rtriangles();
            index_t nt[3];
            split_triangle(t, x, U, nU, nt);
            // check the new triangles (Lemma 24 guarantees the isotopy,
            // but numerically degenerate configurations may occur)
            index_t carriers[3];
            bool ok = true;
            for(index_t i=0; i<3 && ok; ++i) {
                ok = isotopy_test(
                    rt_vertex(nt[i],0), rt_vertex(nt[i],1), rt_vertex(nt[i],2),
                    carriers[i]
                );
            }
            if(!ok) {
                {
                    // undo
                    for(index_t k=0; k<3; ++k) {
                        rt_v_[3*t+k] = save_v[k];
                        rt_adjacent(t,k) = save_adj[k];
                    }
                    rt_replace_adjacent(save_adj[1], nt[1], t);
                    rt_replace_adjacent(save_adj[2], nt[2], t);
                    rv_.resize(nv0);
                    rt_v_.resize(3*nt0);
                    rt_adj_.resize(3*nt0);
                    rt_alive_.resize(nt0);
                    rt_carrier_.resize(nt0);
                    return false;
                }
            }
            for(index_t i=0; i<3; ++i) {
                rt_carrier_[nt[i]] = carriers[i];
                to_check.push_back(nt[i]);
            }
            return true;
        }

        /**
         * \brief Inserts a vertex on an edge, at the point of the
         *  surface on the transversal segment through the midpoint.
         * \retval true on success
         */
        bool insert_on_edge(
            index_t t, index_t k, vector<index_t>& to_check
        ) {
            index_t n = rt_adjacent(t,k);
            if(n == NO_INDEX) {
                return false;
            }
            index_t u = rt_vertex(t,k);
            index_t v = rt_vertex(t,k+1);
            index_t verts[2] = {u, v};
            vec3 x = 0.5*(rv_[u].p + rv_[v].p);
            index_t c = find_tet_containing(verts, 2, x, rt_carrier_[t]);
            if(c == NO_INDEX) {
                return false;
            }
            const index_t* U = &(ct_v_[4*c]);
            index_t nU = 4;
            if(!move_to_surface(U, nU, c, x)) {
                return false;
            }
            // save state for undo
            index_t kn = rt_side(n, v, u);
            index_t save_tv[3] = { rt_vertex(t,0), rt_vertex(t,1), rt_vertex(t,2) };
            index_t save_nv[3] = { rt_vertex(n,0), rt_vertex(n,1), rt_vertex(n,2) };
            index_t save_ta[3] = { rt_adjacent(t,0), rt_adjacent(t,1), rt_adjacent(t,2) };
            index_t save_na[3] = { rt_adjacent(n,0), rt_adjacent(n,1), rt_adjacent(n,2) };
            index_t nv0 = rv_.size();
            index_t nt0 = nb_rtriangles();
            index_t nt[4];
            split_edge(t, k, x, U, nU, nt);
            index_t carriers[4];
            bool ok = true;
            for(index_t i=0; i<4 && ok; ++i) {
                ok = isotopy_test(
                    rt_vertex(nt[i],0), rt_vertex(nt[i],1), rt_vertex(nt[i],2),
                    carriers[i]
                );
            }
            if(!ok) {
                {
                    // undo
                    for(index_t j=0; j<3; ++j) {
                        rt_v_[3*t+j] = save_tv[j];
                        rt_v_[3*n+j] = save_nv[j];
                        rt_adjacent(t,j) = save_ta[j];
                        rt_adjacent(n,j) = save_na[j];
                    }
                    rt_replace_adjacent(save_ta[(k+1)%3], nt[1], t);
                    rt_replace_adjacent(save_na[(kn+1)%3], nt[3], n);
                    rv_.resize(nv0);
                    rt_v_.resize(3*nt0);
                    rt_adj_.resize(3*nt0);
                    rt_alive_.resize(nt0);
                    rt_carrier_.resize(nt0);
                    return false;
                }
            }
            for(index_t i=0; i<4; ++i) {
                rt_carrier_[nt[i]] = carriers[i];
                to_check.push_back(nt[i]);
            }
            return true;
        }

        /**
         * \brief Collapses the edges shorter than a threshold (relative
         *  to the target size), when the isotopy is maintained.
         * \details As in the paper, this is done before Chew's algorithm
         *  to reduce the size of the final mesh (the coarse mesh has
         *  many very short edges).
         */
        void collapse_short_edges() {
            index_t nb_collapsed = 0;
            vector<index_t> around;
            for(index_t t=0; t<nb_rtriangles(); ++t) {
                if(!rt_alive_[t]) {
                    continue;
                }
                for(index_t k=0; k<3; ++k) {
                    if(!rt_alive_[t]) {
                        break;
                    }
                    index_t u = rt_vertex(t,k);
                    index_t v = rt_vertex(t,k+1);
                    double l = length(rv_[u].p - rv_[v].p);
                    vec3 m = 0.5*(rv_[u].p + rv_[v].p);
                    double h = target_size(t, m);
                    if(l >= refine_params_.collapse_factor * h) {
                        continue;
                    }
                    if(try_collapse(t, k, around)) {
                        ++nb_collapsed;
                        break;
                    }
                }
            }
            if(verbose_) {
                Logger::out("skin") << "Collapsed " << nb_collapsed
                                    << " short edges" << std::endl;
            }
        }

        /**
         * \brief Gets the triangles around a vertex, in order.
         * \retval true on success
         * \retval false if the vertex is not surrounded by a closed fan
         */
        bool triangles_around(index_t u, index_t t0, vector<index_t>& tris) {
            tris.resize(0);
            index_t t = t0;
            for(index_t iter=0; iter<1000; ++iter) {
                tris.push_back(t);
                index_t lu = NO_INDEX;
                for(index_t k=0; k<3; ++k) {
                    if(rt_vertex(t,k) == u) {
                        lu = k;
                    }
                }
                if(lu == NO_INDEX) {
                    return false;
                }
                index_t next = rt_adjacent(t, lu);
                if(next == NO_INDEX) {
                    return false;
                }
                if(next == t0) {
                    return true;
                }
                t = next;
            }
            return false;
        }

        /**
         * \brief Collapses the edge (side k of triangle t) by merging
         *  its first vertex into its second one, if the result is
         *  manifold and the isotopy is maintained.
         */
        bool try_collapse(index_t t, index_t k, vector<index_t>& around) {
            index_t n = rt_adjacent(t,k);
            if(n == NO_INDEX) {
                return false;
            }
            index_t u = rt_vertex(t,k);
            index_t v = rt_vertex(t,k+1);
            index_t p = rt_vertex(t,k+2);
            index_t kn = rt_side(n, v, u);
            index_t q = rt_vertex(n, kn+2);
            if(kn == NO_INDEX || p == q) {
                return false;
            }
            if(!triangles_around(u, t, around)) {
                return false;
            }
            if(around.size() < 4) {
                return false;
            }
            // link condition: the neighbors of u, other than v, p, q,
            // must not be neighbors of v. We check it with the triangles
            // around v.
            vector<index_t> around_v;
            if(!triangles_around(v, n, around_v)) {
                return false;
            }
            for(index_t tu : around) {
                if(tu == t || tu == n) {
                    continue;
                }
                for(index_t i=0; i<3; ++i) {
                    index_t w = rt_vertex(tu,i);
                    if(w == u || w == p || w == q || w == v) {
                        continue;
                    }
                    for(index_t tv : around_v) {
                        for(index_t j=0; j<3; ++j) {
                            if(rt_vertex(tv,j) == w) {
                                return false;
                            }
                        }
                    }
                }
            }
            // isotopy test on the new triangles
            vector<index_t> carriers;
            for(index_t tu : around) {
                if(tu == t || tu == n) {
                    continue;
                }
                index_t w[3];
                for(index_t i=0; i<3; ++i) {
                    w[i] = rt_vertex(tu,i);
                    if(w[i] == u) {
                        w[i] = v;
                    }
                }
                index_t carrier;
                if(!isotopy_test(w[0], w[1], w[2], carrier)) {
                    return false;
                }
                carriers.push_back(carrier);
            }
            // do it
            index_t t_out1 = rt_adjacent(t, k+1);  // across (v,p)
            index_t t_out2 = rt_adjacent(t, k+2);  // across (p,u)
            index_t n_out1 = rt_adjacent(n, kn+1); // across (u,q)
            index_t n_out2 = rt_adjacent(n, kn+2); // across (q,v)
            if(t_out1 == NO_INDEX || t_out2 == NO_INDEX || n_out1 == NO_INDEX || n_out2 == NO_INDEX) {
                return false;
            }
            rt_replace_adjacent(t_out1, t, t_out2);
            rt_replace_adjacent(t_out2, t, t_out1);
            rt_replace_adjacent(n_out1, n, n_out2);
            rt_replace_adjacent(n_out2, n, n_out1);
            rt_alive_[t] = false;
            rt_alive_[n] = false;
            index_t c = 0;
            for(index_t tu : around) {
                if(tu == t || tu == n) {
                    continue;
                }
                for(index_t i=0; i<3; ++i) {
                    if(rt_v_[3*tu+i] == u) {
                        rt_v_[3*tu+i] = v;
                    }
                }
                rt_carrier_[tu] = carriers[c++];
            }
            rv_[u].alive = false;
            return true;
        }


        /**
         * \brief Chew-style Delaunay refinement with isotopy tests.
         * \details Bad triangles (too small angle or too large) are
         *  refined by inserting their circumcenter, located by walking on
         *  the mesh, moved to the skin surface along the transversal
         *  segment (Lemma 24). If the circumcenter cannot be located, the
         *  longest edge is split instead. After each insertion, the
         *  Delaunay property is restored by edge flips, each flip being
         *  tested with Lemma 23.
         */
        bool refine() {
            if(refine_params_.collapse_factor > 0.0) {
                collapse_short_edges();
            }
            vector<index_t> queue;
            vector<index_t> to_check;
            vector<index_t> stack;
            for(index_t t=0; t<nb_rtriangles(); ++t) {
                if(rt_alive_[t]) {
                    queue.push_back(t);
                }
            }
            index_t nv0 = 0;
            for(const RVertex& v : rv_) {
                nv0 += index_t(v.alive);
            }
            index_t max_nv = index_t(
                double(nv0) * refine_params_.max_vertices_factor
            );
            index_t nb_inserted = 0;
            index_t nb_fallback = 0;
            while(queue.size() != 0) {
                index_t t = queue.back();
                queue.pop_back();
                if(!rt_alive_[t]) {
                    continue;
                }
                if(!is_bad(t)) {
                    index_t cc;
                    if(isotopy_test(rt_vertex(t,0), rt_vertex(t,1), rt_vertex(t,2), cc)) {
                        continue;
                    }
                }
                if(nv0 + nb_inserted >= max_nv) {
                    break;
                }
                const vec3& p0 = rv_[rt_vertex(t,0)].p;
                const vec3& p1 = rv_[rt_vertex(t,1)].p;
                const vec3& p2 = rv_[rt_vertex(t,2)].p;
                double R;
                vec3 c = circumcenter(p0, p1, p2, R);
                // target point: closest point of the surface to the
                // circumcenter (along the gradient of the quadric)
                vec3 xs = c;
                to_check.resize(0);
                bool done = false;
                double bary[3];
                index_t T = NO_INDEX;
                vec3 g_t = (1.0/3.0)*(p0+p1+p2);
                if(quadrics_[ct_q_[rt_carrier_[t]]].project(xs, g_t)) {
                    T = locate(t, xs, bary);
                    if(T == NO_INDEX) ++nb_locate_failures_;
                } else {
                    ++nb_project_failures_;
                }
                if(T != NO_INDEX) {
                    // If the circumcenter is close to an edge of T, split
                    // this edge instead (encroachment), else insert it
                    // inside T.
                    const double eps = refine_params_.encroach_factor;
                    index_t kmin = 0;
                    for(index_t i=1; i<3; ++i) {
                        if(bary[i] < bary[kmin]) {
                            kmin = i;
                        }
                    }
                    if(bary[kmin] < eps) {
                        // edge opposite to vertex kmin is side kmin+1
                        done = insert_on_edge(T, kmin+1, to_check);
                        if(done) {
                            ++nb_fallback;
                        }
                    } else {
                        for(index_t i=0; i<3; ++i) {
                            bary[i] = std::max(bary[i], 1e-3);
                        }
                        double sum = bary[0]+bary[1]+bary[2];
                        // Chew: the inserted point is at distance >= R
                        // from the existing vertices. If this is far from
                        // being the case (bad location), do not insert.
                        for(index_t i=0; i<3; ++i) {
                            if(length(xs - rv_[rt_vertex(T,i)].p) < 0.25*R) {
                                T = NO_INDEX;
                                break;
                            }
                        }
                        if(T != NO_INDEX) {
                            vec3 x = (bary[0]/sum)*rv_[rt_vertex(T,0)].p +
                                     (bary[1]/sum)*rv_[rt_vertex(T,1)].p +
                                     (bary[2]/sum)*rv_[rt_vertex(T,2)].p;
                            done = insert_in_triangle(T, x, to_check);
                            if(!done) ++nb_insert_failures_;
                        }
                    }
                }
                // Fallback: split the longest edge, only if the triangle
                // is too large or its longest edge is still long with
                // respect to the target size (so that edge lengths are
                // bounded from below and the process terminates). Other
                // badly shaped triangles whose circumcenter could not be
                // inserted are left as is.
                if(!done) {
                    index_t kmax = 0;
                    double lmax = -1.0;
                    for(index_t k=0; k<3; ++k) {
                        double l = length(rv_[rt_vertex(t,k+1)].p - rv_[rt_vertex(t,k)].p);
                        if(l > lmax) {
                            lmax = l;
                            kmax = k;
                        }
                    }
                    double h = target_size(t, g_t);
                    if(R > h || lmax > refine_params_.split_factor * h) {
                        done = insert_on_edge(t, kmax, to_check);
                    }
                    if(done) {
                        ++nb_fallback;
                    }
                }
                if(!done) {
                    continue;
                }
                ++nb_inserted;
                stack = to_check;
                lawson_flips(stack, to_check);
                for(index_t t2 : to_check) {
                    queue.push_back(t2);
                }
            }
            if(verbose_) {
                Logger::out("skin")
                    << "Refinement: inserted " << nb_inserted << " vertices ("
                    << nb_fallback << " edge splits), flips wanted/refused/done: "
                    << nb_flips_wanted_ << "/" << nb_flips_refused_ << "/"
                    << nb_flips_done_ << ", locate/project/insert failures: "
                    << nb_locate_failures_ << "/" << nb_project_failures_
                    << "/" << nb_insert_failures_ << std::endl;
            }
            repair_transversality();
            return true;
        }

    private:
        const Delaunay& D_;
        const double* points_;
        const double* weights_;
        index_t nb_atoms_;
        double s_;
        bool verbose_;

        // Delaunay combinatorics
        vector<bool> tet_finite_;
        vector<vec3> tet_c_;          // radical center of each tet
        vector<index_t> tri_v_;       // 3 vertices per triangle
        vector<index_t> tri_t_;       // 2 tets per triangle (may be NO_INDEX)
        vector<index_t> tri_e_;       // 3 edges per triangle, e[k]=(v[k],v[k+1])
        vector<index_t> tri_of_tf_;   // triangle of (tet,local facet)
        vector<index_t> edge_v_;      // 2 vertices per edge (sorted)
        vector<bool>    edge_open_;   // true if an infinite tet is around
        vector<index_t> edge_tets_ptr_;
        vector<index_t> edge_tets_;   // tets around edge, cyclic order
        vector<index_t> edge_tris_;   // triangle between tets k and k+1
        vector<index_t> v_edges_ptr_;
        vector<index_t> v_edges_;     // edges incident to vertex

        // Centers
        vector<vec3> c_e_;
        vector<vec3> c_f_;

        // Anchor points of the Delaunay simplices
        vector<vec3>    ad_e_;
        vector<Simplex> sd_e_;
        vector<vec3>    ad_f_;
        vector<Simplex> sd_f_;
        vector<vec3>    ad_t_;
        vector<Simplex> sd_t_;

        // Bounding box and clipped power diagram
        double box_min_[3];
        double box_max_[3];
        vector<bool> tet_in_box_;     // radical center inside the box
        vector<VFace> vf_;            // faces of the clipped diagram
        std::unordered_map<FaceKey, index_t, FaceKeyHash> vf_map_;
        vector<index_t> vf_tri_;      // clipped power edge of each triangle
        vector<index_t> vf_edge_;     // clipped power facet of each edge
        vector<index_t> vf_vertex_;   // clipped power cell of each atom

        // Tetrahedral complex
        vector<Quadric> quadrics_;
        std::unordered_map<VertexKey, index_t, VertexKeyHash> cv_map_;
        vector<vec3>    cv_pos_;      // complex vertices
        vector<double>  cv_val_;      // value of the implicit function
        vector<index_t> ct_v_;        // 4 vertices per complex tet
        vector<index_t> ct_q_;        // quadric (mixed cell) per complex tet
        index_t nb_degenerate_;

        // Surface extraction
        std::unordered_map<Numeric::uint64, index_t> edge_vertex_;

        // Refinable mesh
        vector<RVertex> rv_;
        vector<index_t> rt_v_;        // 3 vertices per triangle
        vector<index_t> rt_adj_;      // 3 adjacent triangles per triangle
        vector<bool>    rt_alive_;
        vector<index_t> rt_carrier_;  // complex tet containing the triangle
        vector<index_t> cv_tets_ptr_; // complex tets incident to each
        vector<index_t> cv_tets_;     //   complex vertex (CRS)
        vector<index_t> tmp_tets_;
        vector<index_t> tmp_tris_;
        vector<index_t> tmp_cand_;
        vector<vec3> tmp_clip_;
        vector<vec3> tmp_poly_;
        vector<index_t> tet_mark_;
        index_t tet_stamp_ = 0;
        index_t nb_flips_wanted_ = 0, nb_flips_refused_ = 0, nb_flips_done_ = 0;
        index_t nb_locate_failures_ = 0, nb_project_failures_ = 0, nb_insert_failures_ = 0;
        SkinRefineParams refine_params_;
    };

    /*********************************************************************/

    SkinSurfaceMesher::SkinSurfaceMesher(
        const Delaunay& delaunay,
        const double* points,
        const double* weights,
        index_t nb_atoms,
        double shrink_factor
    ) : delaunay_(delaunay),
        points_(points),
        weights_(weights),
        nb_atoms_(nb_atoms),
        shrink_(shrink_factor),
        verbose_(true),
        nb_complex_vertices_(0),
        nb_complex_tets_(0),
        nb_degenerate_tets_(0),
        impl_(nullptr) {
        // The quadrics have 1/s and 1/(1-s) terms: keep s away from
        // 0 and 1 (for s=1 the skin surface is the union of the balls,
        // for s=0 it is the convex hull of the shrunk-to-nothing balls).
        shrink_ = std::max(1e-4, std::min(1.0 - 1e-4, shrink_));
    }

    SkinSurfaceMesher::~SkinSurfaceMesher() {
        delete impl_;
    }

    bool SkinSurfaceMesher::compute(Mesh& skin, Mesh* tet_complex) {
        delete impl_;
        impl_ = new SkinSurfaceMesherImpl(
            delaunay_, points_, weights_, nb_atoms_, shrink_, verbose_
        );
        bool result = impl_->run(skin, tet_complex);
        nb_complex_vertices_ = impl_->nb_complex_vertices();
        nb_complex_tets_ = impl_->nb_complex_tets();
        nb_degenerate_tets_ = impl_->nb_degenerate_tets();
        if(!result) {
            delete impl_;
            impl_ = nullptr;
        }
        return result;
    }

    bool SkinSurfaceMesher::subdivide(Mesh& skin, index_t n) {
        if(impl_ == nullptr) {
            Logger::err("skin") << "subdivide(): call compute() first"
                                << std::endl;
            return false;
        }
        return impl_->subdivide(skin, n);
    }

    bool SkinSurfaceMesher::refine(Mesh& skin, const SkinRefineParams& params) {
        if(impl_ == nullptr) {
            Logger::err("skin") << "refine(): call compute() first"
                                << std::endl;
            return false;
        }
        return impl_->refine(skin, params);
    }

    index_t SkinSurfaceMesher::check_isotopy() {
        if(impl_ == nullptr) {
            return NO_INDEX;
        }
        return impl_->count_isotopy_violations();
    }

}
