/*
 *  Generated using Claude
 *
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
 *  av(X)  : the anchor point of nu_X, i.e. the point of nu_X closest to
 *           c(X). sv(X) is the face of nu_X that contains av(X) in its
 *           relative interior (it is nu_{X''} for a coface X'' of X, and
 *           Lemma 16: av(X) = av(sv(X))).
 *
 *  The anchor point of the polyhedral cell mu_{X,X'} is (Lemma 14)
 *     (1-s) ad(X) + s av(X')
 *  and it is identified by the pair (sd(X), sv(X')): if the anchor point
 *  lies on the boundary of mu_{X,X'} then it coincides with the anchor
 *  point of the face mu_{sd(X),sv(X')} and the vertex is "collapsed" as
 *  described in Section 3.2 of the paper. The collapsed vertices generate
 *  degenerate tetrahedra (with repeated vertices) that are simply skipped.
 */

namespace {
    using namespace GEO;

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
    };

    /***********************************************************************/

    /**
     * \brief Internal class that does all the work.
     */
    class Builder {
    public:

        Builder(
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
            }
            {
                Stopwatch W("skin:anchors", verbose_);
                compute_delaunay_anchors();
                if(!compute_voronoi_anchors()) {
                    return false;
                }
            }
            {
                Stopwatch W("skin:complex", verbose_);
                build_tetrahedral_complex();
            }
            {
                Stopwatch W("skin:extract", verbose_);
                extract_surface(skin);
            }
            if(tet_complex != nullptr) {
                get_tetrahedral_complex(*tet_complex);
            }
            if(verbose_) {
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

        /**
         * \brief Computes for each Delaunay simplex delta_X the point
         *  av(X) of the power cell nu_X closest to c(X), and the face
         *  sv(X) of nu_X that contains it (Lemma 16).
         * \details Faces of nu_X are the nu_X'' for cofaces X'' of X.
         * \retval true on success
         * \retval false if the power cell of an atom is unbounded
         */
        bool compute_voronoi_anchors() {

            // Triangles: nu_f is the segment [c_t1, c_t2]
            index_t nf = nb_tris();
            av_f_.resize(nf);
            sv_f_.resize(nf);
            for(index_t f=0; f<nf; ++f) {
                index_t t0 = tri_t_[2*f];
                index_t t1 = tri_t_[2*f+1];
                if(
                    t0 == NO_INDEX || t1 == NO_INDEX ||
                    !tet_finite_[t0] || !tet_finite_[t1]
                ) {
                    // Unbounded power edge, only incident to cells of
                    // non-atom vertices (never used).
                    sv_f_[f] = SF(f);
                    av_f_[f] = c_f_[f];
                    continue;
                }
                vec3 c0 = tet_c_[t0];
                vec3 c1 = tet_c_[t1];
                vec3 D = c1 - c0;
                double DD = length2(D);
                double lambda = (DD == 0.0) ? 0.0 : dot(c_f_[f] - c0, D) / DD;
                if(lambda > 0.0 && lambda < 1.0) {
                    sv_f_[f] = SF(f);
                    av_f_[f] = c_f_[f];
                } else if(lambda <= 0.0) {
                    sv_f_[f] = ST(t0);
                    av_f_[f] = c0;
                } else {
                    sv_f_[f] = ST(t1);
                    av_f_[f] = c1;
                }
            }

            // Edges: nu_e is a convex polygon in the bisector plane,
            // with vertices c_t for the tets t around e, and edges
            // nu_f for the triangles f around e.
            index_t ne = nb_edges();
            av_e_.resize(ne);
            sv_e_.resize(ne);
            for(index_t e=0; e<ne; ++e) {
                if(edge_open_[e]) {
                    sv_e_[e] = SE(e);
                    av_e_[e] = c_e_[e];
                    continue;
                }
                vec3 ce = c_e_[e];
                vec3 d = point(edge_v_[2*e+1]) - point(edge_v_[2*e]);
                index_t m = edge_nb_tets(e);
                bool has_pos = false;
                bool has_neg = false;
                for(index_t k=0; k<m; ++k) {
                    vec3 ck = tet_c_[edge_tet(e,k)];
                    vec3 ck1 = tet_c_[edge_tet(e,(k+1)%m)];
                    double sgn = dot(d, cross(ck1 - ck, ce - ck));
                    if(sgn > 0.0) {
                        has_pos = true;
                    } else {
                        has_neg = true;
                    }
                }
                bool inside = (m >= 3) && (has_pos != has_neg);
                if(inside) {
                    sv_e_[e] = SE(e);
                    av_e_[e] = ce;
                } else {
                    double best_d2 = Numeric::max_float64();
                    for(index_t k=0; k<m; ++k) {
                        index_t f = edge_tri(e,k);
                        double d2 = distance2(ce, av_f_[f]);
                        if(d2 < best_d2) {
                            best_d2 = d2;
                            sv_e_[e] = sv_f_[f];
                            av_e_[e] = av_f_[f];
                        }
                    }
                }
            }

            // Vertices: nu_v is a convex polyhedron with facets nu_e
            // for the edges e incident to v.
            index_t nv = nb_vertices();
            av_v_.resize(nv);
            sv_v_.resize(nv);
            for(index_t v=0; v<nv; ++v) {
                sv_v_[v] = SV(v);
                av_v_[v] = point(v);
                if(!is_atom(v)) {
                    continue;
                }
                vec3 p = point(v);
                double P = weight(v);
                bool inside = true;
                for(index_t k=v_edges_ptr_[v]; k<v_edges_ptr_[v+1]; ++k) {
                    index_t e = v_edges_[k];
                    if(edge_open_[e]) {
                        Logger::err("skin")
                            << "Power cell of atom " << v
                            << " is unbounded (call close_cells() first)"
                            << std::endl;
                        return false;
                    }
                    index_t q = edge_v_[2*e] + edge_v_[2*e+1] - v;
                    // p is inside nu_v iff pi(p_hat,p) < pi(q_hat,p)
                    // for all neighbors q, i.e. -P < |p-q|^2 - Q
                    if(distance2(p, point(q)) - weight(q) + P <= 0.0) {
                        inside = false;
                    }
                }
                if(!inside) {
                    double best_d2 = Numeric::max_float64();
                    for(index_t k=v_edges_ptr_[v]; k<v_edges_ptr_[v+1]; ++k) {
                        index_t e = v_edges_[k];
                        double d2 = distance2(p, av_e_[e]);
                        if(d2 < best_d2) {
                            best_d2 = d2;
                            sv_v_[v] = sv_e_[e];
                            av_v_[v] = av_e_[e];
                        }
                    }
                }
            }
            return true;
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

        Simplex support_v(Simplex X) const {
            switch(X.dim) {
            case 0: return sv_v_[X.id];
            case 1: return sv_e_[X.id];
            case 2: return sv_f_[X.id];
            default: return X;
            }
        }

        vec3 anchor_v(Simplex X) const {
            switch(X.dim) {
            case 0: return av_v_[X.id];
            case 1: return av_e_[X.id];
            case 2: return av_f_[X.id];
            default: return tet_c_[X.id];
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
         *  mu_{X,X'} = (1-s) delta_X (+) s nu_X'.
         * \param[in] X , Xp two Delaunay simplices with X a face of Xp
         *  (or X == Xp)
         * \param[in] q the index of the quadric of the mixed cell being
         *  processed, used to compute the value of the implicit function
         *  at the vertex (if it is created).
         */
        index_t get_vertex(Simplex X, Simplex Xp, index_t q) {
            VertexKey key{ support_d(X).key(), support_v(Xp).key() };
            auto it = cv_map_.find(key);
            if(it != cv_map_.end()) {
                return it->second;
            }
            vec3 p = (1.0 - s_)*anchor_d(X) + s_*anchor_v(Xp);
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
         * \brief Triangulates a hyperbolic facet mu_{e,f} (a rectangle,
         *  Figure 4(b)), split into its four quadrants.
         * \details The rectangle is (1-s) delta_e (+) s nu_f, where nu_f
         *  is the segment [c_ta, c_tb]. Its corners are the vertices
         *  (v_i, t_j) of the mixed complex. Edge anchors are
         *  (v_i, sv(f)) on the two sides parallel to the symmetry axis
         *  of the hyperboloid of e and (sd(e), t_j) on the two sides
         *  parallel to its symmetry plane. The facet anchor is
         *  (sd(e), sv(f)). Quadrant (i,j) is the one that contains
         *  corner (v_i, t_j), it is triangulated with the two triangles
         *  (F, Et_j, Ev_i) and (Et_j, P_ij, Ev_i) : on the edge Et_j-Ev_i,
         *  the distance to one symmetry set increases while the distance
         *  to the other one decreases (monotonicity condition).
         * \param[in] e the edge
         * \param[in] f a triangle incident to e
         * \param[in] ta , tb the two tetrahedra incident to f
         * \param[in] q the quadric of the mixed cell being processed
         * \param[out] quad quad[i][j] receives the two triangles of
         *  quadrant (i,j), i = extremity of e, j = ta (0) or tb (1).
         */
        void hyperbolic_facet(
            index_t e, index_t f, index_t ta, index_t tb, index_t q,
            vector<index_t> quad[2][2]
        ) {
            index_t v[2] = { edge_v_[2*e], edge_v_[2*e+1] };
            index_t t[2] = { ta, tb };
            index_t F = get_vertex(SE(e), SF(f), q);
            index_t Ev[2];
            index_t Et[2];
            for(index_t i=0; i<2; ++i) {
                Ev[i] = get_vertex(SV(v[i]), SF(f), q);
                Et[i] = get_vertex(SE(e), ST(t[i]), q);
            }
            for(index_t i=0; i<2; ++i) {
                for(index_t j=0; j<2; ++j) {
                    index_t P = get_vertex(SV(v[i]), ST(t[j]), q);
                    vector<index_t>& Q = quad[i][j];
                    Q.resize(0);
                    Q.push_back(F);     Q.push_back(Et[j]); Q.push_back(Ev[i]);
                    Q.push_back(Et[j]); Q.push_back(P);     Q.push_back(Ev[i]);
                }
            }
        }

        /**
         * \brief Triangulates the mixed cell mu_v of an atom v
         *  (type 0, shrunk power cell, contains a spherical patch).
         * \details The facets are the polygons mu_{v,e} for the edges e
         *  incident to v. They are circular facets. All the tetrahedra
         *  are joined with the anchor point of the cell (the point of the
         *  cell closest to the center of the sphere).
         */
        void process_vertex_cell(index_t v) {
            index_t q = quadrics_.size();
            quadrics_.push_back(vertex_quadric(v));
            index_t apex = get_vertex(SV(v), SV(v), q);
            vector<index_t> cycle;
            vector<index_t> tris;
            for(index_t k=v_edges_ptr_[v]; k<v_edges_ptr_[v+1]; ++k) {
                index_t e = v_edges_[k];
                index_t F = get_vertex(SV(v), SE(e), q);
                cycle.resize(0);
                tris.resize(0);
                index_t m = edge_nb_tets(e);
                for(index_t j=0; j<m; ++j) {
                    cycle.push_back(get_vertex(SV(v), ST(edge_tet(e,j)), q));
                    cycle.push_back(get_vertex(SV(v), SF(edge_tri(e,j)), q));
                }
                fan(F, cycle, tris);
                join(apex, tris, q);
            }
        }

        /**
         * \brief Triangulates the mixed cell mu_e of an atom edge e
         *  (type 1, prism, contains an hyperboloid patch).
         * \details The two bases mu_{v_i,e} are circular facets, the
         *  side facets mu_{e,f} are hyperbolic facets. The prism is
         *  split by the plane V through the anchor point of the cell,
         *  parallel to its bases (the symmetry plane of the hyperboloid).
         *  V contains the anchor points (sd(e),t) of the edges parallel to
         *  the axis and the anchor points (sd(e),sv(f)) of the side facets,
         *  and the new facet in V is a circular facet (a copy of nu_e).
         *  Each half-prism is triangulated by joining the anchor point
         *  of its base (v_i, sv(e)) with its boundary.
         */
        void process_edge_cell(index_t e) {
            index_t q = quadrics_.size();
            quadrics_.push_back(edge_quadric(e));
            index_t v[2] = { edge_v_[2*e], edge_v_[2*e+1] };
            index_t C = get_vertex(SE(e), SE(e), q);
            index_t apex[2];
            for(index_t i=0; i<2; ++i) {
                apex[i] = get_vertex(SV(v[i]), SE(e), q);
            }
            index_t m = edge_nb_tets(e);

            // The splitting facet V (circular facet)
            vector<index_t> cycle;
            vector<index_t> tris;
            for(index_t k=0; k<m; ++k) {
                cycle.push_back(get_vertex(SE(e), ST(edge_tet(e,k)), q));
                cycle.push_back(get_vertex(SE(e), SF(edge_tri(e,k)), q));
            }
            fan(C, cycle, tris);
            join(apex[0], tris, q);
            join(apex[1], tris, q);

            // The side facets (hyperbolic facets), each one split
            // into two halves by V.
            vector<index_t> quad[2][2];
            for(index_t k=0; k<m; ++k) {
                index_t f = edge_tri(e,k);
                index_t ta = edge_tet(e,k);
                index_t tb = edge_tet(e,(k+1)%m);
                hyperbolic_facet(e, f, ta, tb, q, quad);
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
         * \details The two bases mu_{f,t_j} are circular facets, the
         *  three side facets mu_{e,f} are hyperbolic facets. The prism is
         *  split by the plane V through the anchor point of the cell,
         *  parallel to its bases (the plane of f, that is the symmetry
         *  plane of the hyperboloid). V contains the anchor points
         *  (v_i, sv(f)) and (sd(e), sv(f)), the new facet in V is a
         *  circular facet (a copy of delta_f). Each half-prism is
         *  triangulated by joining the anchor point of its base
         *  (sd(f), t_j) with its boundary.
         */
        void process_tri_cell(index_t f) {
            index_t q = quadrics_.size();
            quadrics_.push_back(tri_quadric(f));
            index_t t[2] = { tri_t_[2*f], tri_t_[2*f+1] };
            index_t C = get_vertex(SF(f), SF(f), q);
            index_t apex[2];
            for(index_t j=0; j<2; ++j) {
                apex[j] = get_vertex(SF(f), ST(t[j]), q);
            }

            // The splitting facet V (circular facet)
            vector<index_t> cycle;
            vector<index_t> tris;
            for(index_t k=0; k<3; ++k) {
                cycle.push_back(get_vertex(SV(tri_v_[3*f+k]), SF(f), q));
                cycle.push_back(get_vertex(SE(tri_e_[3*f+k]), SF(f), q));
            }
            fan(C, cycle, tris);
            join(apex[0], tris, q);
            join(apex[1], tris, q);

            // The side facets (hyperbolic facets), each one split
            // into two halves by V.
            vector<index_t> quad[2][2];
            for(index_t k=0; k<3; ++k) {
                index_t e = tri_e_[3*f+k];
                hyperbolic_facet(e, f, t[0], t[1], q, quad);
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
         * \details The four facets mu_{f,t} are circular facets. All
         *  the tetrahedra are joined with the anchor point of the cell.
         */
        void process_tet_cell(index_t t) {
            index_t q = quadrics_.size();
            quadrics_.push_back(tet_quadric(t));
            index_t apex = get_vertex(ST(t), ST(t), q);
            vector<index_t> cycle;
            vector<index_t> tris;
            for(index_t lf=0; lf<4; ++lf) {
                index_t f = tri_of_tf_[4*t+lf];
                index_t F = get_vertex(SF(f), ST(t), q);
                cycle.resize(0);
                tris.resize(0);
                for(index_t k=0; k<3; ++k) {
                    cycle.push_back(get_vertex(SV(tri_v_[3*f+k]), ST(t), q));
                    cycle.push_back(get_vertex(SE(tri_e_[3*f+k]), ST(t), q));
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
         * \brief Gets (or creates) the mesh vertex at the intersection
         *  between the skin surface and an edge of the tetrahedral
         *  complex.
         * \details The intersection is computed as the root of the
         *  quadric I_X restricted to the edge (there is exactly one by
         *  the monotonicity condition, Observation 18). It is cached, so
         *  that the two cells that share an edge of the complex use the
         *  same vertex (no crack).
         * \param[in] a , b the two extremities of the edge (one inside
         *  and one outside)
         * \param[in] q the quadric of the mixed cell
         */
        index_t edge_intersection(index_t a, index_t b, index_t q, Mesh& M) {
            Numeric::uint64 key = edge_key(a, b);
            auto it = edge_vertex_.find(key);
            if(it != edge_vertex_.end()) {
                return it->second;
            }
            vec3 pa = cv_pos_[a];
            vec3 pb = cv_pos_[b];
            double fa = cv_val_[a];
            double fb = cv_val_[b];
            // I_X(pa + t (pb - pa)) = A t^2 + B t + C, with the
            // constraints f(0) = fa and f(1) = fb.
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
            vec3 p = pa + t*(pb - pa);
            index_t result = M.vertices.create_vertex();
            M.vertices.point(result) = p;
            edge_vertex_[key] = result;
            return result;
        }

        /**
         * \brief Adds a triangle to the mesh, oriented in such a way
         *  that its normal points towards \p outside.
         */
        void add_triangle(
            Mesh& M, index_t v0, index_t v1, index_t v2,
            const vec3& outside, index_t type,
            Attribute<index_t>& type_attr
        ) {
            const vec3& p0 = M.vertices.point(v0);
            const vec3& p1 = M.vertices.point(v1);
            const vec3& p2 = M.vertices.point(v2);
            vec3 n = cross(p1-p0, p2-p0);
            if(dot(n, outside) < 0.0) {
                std::swap(v1, v2);
            }
            index_t f = M.facets.create_triangle(v0, v1, v2);
            type_attr[f] = type;
        }

        /**
         * \brief Extracts the skin surface from the tetrahedral complex
         *  with the marching tetrahedra algorithm (Figure 5).
         */
        void extract_surface(Mesh& M) {
            M.clear();
            M.vertices.set_dimension(3);
            Attribute<index_t> type_attr(
                M.facets.attributes(), "mixed_cell_type"
            );
            for(index_t t=0; t<nb_complex_tets(); ++t) {
                index_t q = ct_q_[t];
                index_t type = quadrics_[q].ell;
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
                    index_t x0 = edge_intersection(in[0], out[0], q, M);
                    index_t x1 = edge_intersection(in[0], out[1], q, M);
                    index_t x2 = edge_intersection(in[0], out[2], q, M);
                    add_triangle(M, x0, x1, x2, dir, type, type_attr);
                } else if(nb_in == 3) {
                    index_t x0 = edge_intersection(in[0], out[0], q, M);
                    index_t x1 = edge_intersection(in[1], out[0], q, M);
                    index_t x2 = edge_intersection(in[2], out[0], q, M);
                    add_triangle(M, x0, x1, x2, dir, type, type_attr);
                } else {
                    // nb_in == 2: quad x00 x01 x11 x10, where xij is
                    // on the edge (in[i], out[j]).
                    index_t x00 = edge_intersection(in[0], out[0], q, M);
                    index_t x01 = edge_intersection(in[0], out[1], q, M);
                    index_t x11 = edge_intersection(in[1], out[1], q, M);
                    index_t x10 = edge_intersection(in[1], out[0], q, M);
                    add_triangle(M, x00, x01, x11, dir, type, type_attr);
                    add_triangle(M, x00, x11, x10, dir, type, type_attr);
                }
            }
            M.facets.connect();
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

        // Anchor points of the power cells
        vector<vec3>    av_v_;
        vector<Simplex> sv_v_;
        vector<vec3>    av_e_;
        vector<Simplex> sv_e_;
        vector<vec3>    av_f_;
        vector<Simplex> sv_f_;

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
    };
}

/*************************************************************************/

namespace GEO {

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
        nb_degenerate_tets_(0) {
        // The quadrics have 1/s and 1/(1-s) terms: keep s away from
        // 0 and 1 (for s=1 the skin surface is the union of the balls,
        // for s=0 it is the convex hull of the shrunk-to-nothing balls).
        shrink_ = std::max(1e-4, std::min(1.0 - 1e-4, shrink_));
    }

    bool SkinSurfaceMesher::compute(Mesh& skin, Mesh* tet_complex) {
        Builder builder(
            delaunay_, points_, weights_, nb_atoms_, shrink_, verbose_
        );
        bool result = builder.run(skin, tet_complex);
        nb_complex_vertices_ = builder.nb_complex_vertices();
        nb_complex_tets_ = builder.nb_complex_tets();
        nb_degenerate_tets_ = builder.nb_degenerate_tets();
        return result;
    }

}
