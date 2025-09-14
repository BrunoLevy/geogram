/*
 *  Copyright (c) 2000-2025 Inria
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
 *     https://www.inria.fr/en/bruno-levy-1
 *
 *     Inria,
 *     Domaine de Voluceau,
 *     78150 Le Chesnay - Rocquencourt
 *     FRANCE
 *
 */

#include <geogram/mesh/mesh_minkowski.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_convex_hull.h>
#include <geogram/mesh/mesh_surface_intersection.h>
#include <geogram/mesh/mesh_topology.h>
#include <geogram/mesh/mesh_fill_holes.h>
#include <geogram/delaunay/delaunay.h>
#include <geogram/numerics/predicates.h>

namespace {
    using namespace GEO;

    /**
     * \brief Convexity of an edge in a 3D mesh
     * \param[in] M a const reference to the mesh
     * \param[in] f1 an edge facet
     * \param[in] le1 a local edge index in \p f1
     * \param[in] angle_tolerance maximum normal angle deviation for
     *   considering that the two adjacent facets are co-planar
     * \retval POSITIVE if the edge is convex
     * \retval ZERO if the two facets adjacent to the edge are coplanar,
     *   up to \p angle_tolerance
     * \retval NEGATIVE if the edge is concave
     */
    Sign edge_convexity(
	const Mesh& M, index_t f1, index_t le1,
	double angle_tolerance = 0.01 * M_PI / 180.0
    ) {
	// Tiny tolerance (0.01 degrees) for nearly co-planar facets (like
	// square facets in CSGBuilder's tesselated spheres).
	index_t n1 = M.facets.nb_vertices(f1);
	index_t f2 = M.facets.adjacent(f1,le1);
	geo_debug_assert(f2 != NO_INDEX);
	index_t n2 = M.facets.nb_vertices(f2);
	index_t le2 = M.facets.find_adjacent(f2,f1);
	vec3 p1 = M.vertices.point(M.facets.vertex(f1,le1));
	vec3 p2 = M.vertices.point(M.facets.vertex(f1,(le1+1)%n1));
	vec3 p3 = M.vertices.point(M.facets.vertex(f1,(le1+2)%n1));
	vec3 p4 = M.vertices.point(M.facets.vertex(f2,(le2+2)%n2));
	if(angle_tolerance != 0.0) {
	    vec3 N1 = cross(p2-p1, p3-p1);
	    vec3 N2 = cross(p4-p1, p2-p1);
	    double a1 = Geom::angle(N1,N2);
	    if(::fabs(a1) < angle_tolerance) {
		return ZERO;
	    }
	}
	return PCK::orient_3d(p1,p2,p3,p4);
    }

    bool mesh_is_convex_3d(
	const Mesh& M, double angle_tolerance = 0.01 * M_PI / 180.0
    ) {
	index_t nb_positive = 0;
	index_t nb_negative = 0;
	for(index_t f: M.facets) {
	    for(index_t le=0; le<M.facets.nb_vertices(f); ++le) {
		index_t g = M.facets.adjacent(f,le);
		geo_debug_assert(g != NO_INDEX);
		if(f > g) {
		    Sign s = edge_convexity(M,f,le,angle_tolerance);
		    nb_positive += (s > 0);
		    nb_negative += (s < 0);
		}
	    }
	}
	return (nb_positive == 0) || (nb_negative == 0);
    }

    bool mesh_is_convex_2d(const Mesh& M) {
	vector<index_t> nxt(M.vertices.nb(), NO_INDEX);
	for(index_t e: M.edges) {
	    index_t v1 = M.edges.vertex(e,0);
	    index_t v2 = M.edges.vertex(e,1);
	    geo_debug_assert(nxt[v1] == NO_INDEX);
	    nxt[v1] = v2;
	}

	index_t nb_positive = 0;
	index_t nb_negative = 0;
	for(index_t v1: M.vertices) {
	    index_t v2 = nxt[v1];
	    index_t v3 = nxt[v2];
	    Sign s = PCK::orient_2d(
		M.vertices.point_ptr(v1),
		M.vertices.point_ptr(v2),
		M.vertices.point_ptr(v3)
	    );
	    nb_positive += (s > 0);
	    nb_negative += (s < 0);
	}
	return (nb_positive == 0) || (nb_negative == 0);
    }

    void compute_minkowski_sum_convex_convex_3d(
	Mesh& result, const Mesh& op1, const Mesh& op2
    ) {
	result.clear();
	result.vertices.set_dimension(3);

	result.vertices.create_vertices(op1.vertices.nb()*op2.vertices.nb());
	for(index_t v1: op1.vertices) {
	    for(index_t v2: op2.vertices) {
		result.vertices.point(v1*op2.vertices.nb()+v2) =
		    op1.vertices.point(v1) + op2.vertices.point(v2) ;
	    }
	}
	compute_convex_hull_3d(result);
	MeshSurfaceIntersection I(result);
	// TODO: make it work without binding original_facet_id
	Attribute<index_t> original_facet_id(
	    result.facets.attributes(), "original_facet_id"
	);
	for(index_t f: result.facets) {
	    original_facet_id[f] = f;
	}
	I.simplify_coplanar_facets();
    }

    /**
     * \brief Reference: Hichem Barki Ph.D. thesis, Algo 2 P. 90
     * A is possibly non-convex
     * B is convex
     */
    class Minkovski {
    public:
	Minkovski(
	    const Mesh& A, const Mesh& B, Mesh& result
	) : A_(A), B_(B), result_(result) {

	    Av_f_.assign(A_.facets.nb(), NO_INDEX);
	    Af_to_Bvcontrib_.resize(A_.facets.nb());

	    // Store one facet incident to each vertex for vertex_is_elevated()
	    for(index_t f: A_.facets) {
		for(index_t v: A_.facets.vertices(f)) {
		    Av_f_[v] = f;
		}
	    }
	}

	void compute() {
	    result_.clear();
	    result_.vertices.set_dimension(3);

	    // Translated facets
	    vector<index_t> afv;
	    for(index_t af: A_.facets) {
		afv.resize(0);
		for(index_t v: A_.facets.vertices(af)) {
		    afv.push_back(v);
		}

		vec3 ap1 = A_.facets.point(af, 0);
		vec3 ap2 = A_.facets.point(af, 1);
		vec3 ap3 = A_.facets.point(af, 2);

		vec3 N = cross(ap2-ap1,ap3-ap1);
		if(length(N) < 1e-6) {
		    continue;
		}

		vector<index_t>& af_bvcontrib = Af_to_Bvcontrib_[af];
		for(index_t bv: B_.vertices) {
		    if(af_bvcontrib.size() == 0) {
			af_bvcontrib.push_back(bv);
		    } else {
			vec3 bp = B_.vertices.point(bv);
			vec3 best_bp_so_far = B_.vertices.point(af_bvcontrib[0]);
			Sign s = N_dot_compare(
			    ap1, ap2, ap3, bp, best_bp_so_far
			);
			if(s >= 0) {
			    if(s > 0) {
				af_bvcontrib.resize(0);
			    }
			    af_bvcontrib.push_back(bv);
			}
		    }
		}
		minkowski_2d(afv, af_bvcontrib);
	    }

            // Corner facets
	    for(index_t bf: B_.facets) {
		vec3 bp1 = B_.facets.point(bf, 0);
		vec3 bp2 = B_.facets.point(bf, 1);
		vec3 bp3 = B_.facets.point(bf, 2);
		vec3 bfN = cross(bp2-bp1,bp3-bp1);
		if(length(bfN) < 1e-6) {
		    continue;
		}
		for(index_t av: A_.vertices) {
		    if(A_vertex_is_elevated(av, bp1, bp2, bp3)) {
			vec3 T = A_.vertices.point(av);
			index_t deg = B_.facets.nb_vertices(bf);
			index_t first_v = result_.vertices.create_vertices(deg);
			index_t new_f = result_.facets.create_polygon(deg);
			for(index_t lv=0; lv<deg; ++lv) {
			    result_.facets.set_vertex(new_f, lv, first_v+lv);
			    result_.vertices.point(first_v+lv) =
				B_.facets.point(bf,lv) + T;
			}
		    }
		}
		// TODO: edges of op1 elevated w.r.t. V
	    }

	    // Edge facets
	    for_each_edge(A_, [&](const Edge& aE) {
		if(!A_facets_have_distinct_contributing_vertices(aE.f, aE.g)) {
		    return;
		}
		if(edge_convexity(A_, aE.f, aE.le) == POSITIVE) {
		    return;
		}
		for_each_edge(B_, [&](const Edge& bE) {
		    // B is almost always a tessellated sphere with
		    // quads made of two triangles. Ignore the diagonal
		    // edges of those quads.
		    if(Geom::angle(bE.Nf, bE.Ng) < 0.01 * M_PI / 180.0) {
			return;
		    }

		    Sign s1 = geo_sgn(dot(aE.dir,bE.Nf));
		    Sign s2 = geo_sgn(dot(aE.dir,bE.Ng));

		    if(s1 == s2) {
			return;
		    }

		    vec3 abN = cross(aE.dir,bE.dir);
		    if(
			s1*dot(cross(aE.Nf,abN),aE.dir) < 0 &&
			s1*dot(cross(abN,aE.Ng),aE.dir) < 0
		    ) {
			create_quad(
			    aE.p1+bE.p1, aE.p1+bE.p2, aE.p2+bE.p2, aE.p2+bE.p1
			);
		    }
		});
	    });

	    tessellate_facets(result_, 3);
	}


    protected:

	struct Edge {
	    Edge(
		const Mesh& mesh_in, index_t f_in, index_t le_in
	    ) : mesh(mesh_in), f(f_in), le(le_in) {
		g = mesh.facets.adjacent(f,le);
		if(!valid()) {
		    return;
		}
		v1 = mesh.facets.vertex(f,le);
		v2 = mesh.facets.vertex(f,(le+1)%mesh.facets.nb_vertices(f));
		p1 = mesh.vertices.point(v1);
		p2 = mesh.vertices.point(v2);
		dir = p2-p1;
		Nf = Geom::mesh_facet_normal(mesh,f);
		Ng = Geom::mesh_facet_normal(mesh,g);
	    }
	    bool valid() {
		return g != NO_INDEX && f > g;
	    }
	    const Mesh& mesh;
	    index_t f;
	    index_t le;
	    index_t g;
	    index_t v1;
	    index_t v2;
	    vec3 p1;
	    vec3 p2;
	    vec3 dir;
	    vec3 Nf;
	    vec3 Ng;
	};

	void for_each_edge(
	    const Mesh& M, std::function<void(const Edge& E)> doit
	) {
	    for(index_t f: M.facets) {
		for(index_t le=0; le<M.facets.nb_vertices(f); ++le) {
		    Edge E(M,f,le);
		    if(E.valid()) {
			doit(E);
		    }
		}
	    }
	}

	void create_quad(vec3 p1, vec3 p2, vec3 p3, vec3 p4) {
	    index_t first_v =result_.vertices.create_vertices(4);
	    index_t new_f = result_.facets.create_polygon(4);
	    for(index_t lv=0; lv<4; ++lv) {
		result_.facets.set_vertex(new_f, lv, first_v+lv);
	    }
	    result_.vertices.point(first_v  ) = p1;
	    result_.vertices.point(first_v+1) = p2;
	    result_.vertices.point(first_v+2) = p3;
	    result_.vertices.point(first_v+3) = p4;
	}

	bool A_facets_have_distinct_contributing_vertices(
	    index_t Af1, index_t Af2
	) const {
	    const vector<index_t>& c1 = Af_to_Bvcontrib_[Af1];
	    const vector<index_t>& c2 = Af_to_Bvcontrib_[Af2];
	    for(index_t v1: c1) {
		if(std::find(c2.begin(), c2.end(), v1) == c2.end()) {
		    return true;
		}
	    }
	    return false;
	}

	bool A_vertex_is_elevated(
	    index_t av, vec3 bp1, vec3 bp2, vec3 bp3
	) const {
	    vec3 ap1 = A_.vertices.point(av);
	    index_t first_f = Av_f_[av];
	    index_t f = first_f;
	    index_t lv = A_.facets.find_vertex(f,av);
	    do {
		index_t N = A_.facets.nb_vertices(f);
		index_t v2 = A_.facets.vertex(f, (lv + 1) % N);
		vec3 ap2 = A_.vertices.point(v2);
		Sign s = N_dot_compare(bp1,bp2,bp3,ap1,ap2);
		if(s < 0) {
		    return false;
		}
		f = A_.facets.adjacent(f,lv);
		lv = A_.facets.find_vertex(f,av);
	    } while(f != first_f);
	    return true;
	}

	void minkowski_2d(
	    const vector<index_t>& av, const vector<index_t>& bv
	) {
	    geo_debug_assert(av.size() >= 3);
	    geo_debug_assert(bv.size() > 0);
	    if(bv.size() == 1) {
		// simple translation
		vec3 T = B_.vertices.point(bv[0]);
		index_t N = av.size();
		index_t first_v = result_.vertices.create_vertices(N);
		index_t new_f = result_.facets.create_polygon(N);
		for(index_t lv=0; lv<N; ++lv) {
		    result_.facets.set_vertex(new_f, lv, first_v+lv);
		    result_.vertices.point(first_v+lv) =
			A_.vertices.point(av[lv]) + T;
		}
	    } else {
		// Super ugly: use Delaunay in 2D orthogonal plane to
		// compute convex hull.
		vec3 p1 = A_.vertices.point(av[0]);
		vec3 p2 = A_.vertices.point(av[1]);
		vec3 p3 = A_.vertices.point(av[2]);
		vec3 NN = normalize(cross(p2-p1,p3-p1));
		vec3 U = normalize(Geom::perpendicular(NN));
		vec3 V = cross(NN,U);
		vector<vec2> uv;
		vector<vec3> xyz;
		uv.reserve(av.size()*bv.size());
		xyz.reserve(av.size()*bv.size());
		for(index_t curav: av) {
		    vec3 ap = A_.vertices.point(curav);
		    for(index_t curbv: bv) {
			vec3 bp = B_.vertices.point(curbv);
			vec3 abp = ap+bp-p1;
			uv.emplace_back(dot(abp,U), dot(abp,V));
			xyz.emplace_back(ap+bp);
		    }
		}
		Delaunay_var delaunay = Delaunay::create(
		    coord_index_t(2), "BDEL2d"
		);
		delaunay->set_keeps_infinite(true);
		delaunay->set_vertices(uv.size(), uv[0].data());
		vector<index_t> nxt(uv.size(), NO_INDEX);
		index_t first = NO_INDEX;
		for(index_t t=delaunay->nb_finite_cells();
		    t<delaunay->nb_cells(); ++t) {
		    index_t v1= NO_INDEX, v2=NO_INDEX;
		    for(index_t lv=0; lv<3; ++lv) {
			if(delaunay->cell_vertex(t,lv) == NO_INDEX) {
			    v1 = delaunay->cell_vertex(t,(lv+1)%3);
			    v2 = delaunay->cell_vertex(t,(lv+2)%3);
			}
		    }
		    first = v2;
		    nxt[v2] = v1;
		}
		vector<vec3> f_xyz;
		index_t v = first;
		do {
		    f_xyz.push_back(xyz[v]);
		    v = nxt[v];
		} while(v != first);

		if(f_xyz.size() < 3) {
		    return;
		}

		index_t N = f_xyz.size();
		index_t first_v = result_.vertices.create_vertices(N);
		index_t new_f = result_.facets.create_polygon(N);
		for(index_t lv=0; lv<N; ++lv) {
		    result_.facets.set_vertex(new_f, lv, first_v+lv);
		    result_.vertices.point(first_v+lv) = f_xyz[lv];
		}
	    }
	}

	/**
	 * \brief Compares the dot product between the normal to a triangle
	 *   and two vectors
	 * \param[in] p1 , p2 , p3 the three vertices of the triangle, that
	 *   define the normal vector N = cross(p2-p1,p3-p1)
	 * \param[in] q1 , q2 the two points to be compared relative to N
	 * \retval POSITIVE if dot(N,q2) > dot(N,q1)
	 * \retval ZERO if dot(N,q2) = dot(N,q1)
	 * \retval NEGATIVE if dot(N,q2) < dot(N,q1)
	 */
	Sign N_dot_compare(
	    const vec3& p1, const vec3& p2, const vec3& p3,
	    const vec3& q1, const vec3& q2
	) const {
	    // TODO: new specialized predicate
	    return PCK::det_3d(p3-p1, p2-p1, q2-q1);
	}

    private:
	const Mesh& A_;
	const Mesh& B_;
	vector<index_t> Av_f_;
	vector<vector<index_t>> Af_to_Bvcontrib_;
	Mesh& result_;
    };

    void compute_minkowski_sum_non_convex_convex_3d(
	Mesh& result, const Mesh& A, const Mesh& B
    ) {
	// TODO: why needed ?
	reorient_connected_components(const_cast<Mesh&>(A));
	reorient_connected_components(const_cast<Mesh&>(B));
	Minkovski mink(A,B,result);
	mink.compute();
    }

    void compute_minkowski_sum_convex_convex_2d(
	Mesh& result, const Mesh& op1, const Mesh& op2
    ) {
	result.clear();
	result.vertices.set_dimension(2);
	result.vertices.create_vertices(op1.vertices.nb()*op2.vertices.nb());
	for(index_t v1: op1.vertices) {
	    for(index_t v2: op2.vertices) {
		result.vertices.point<2>(v1*op2.vertices.nb()+v2) =
		    op1.vertices.point<2>(v1) + op2.vertices.point<2>(v2) ;
	    }
	}
	compute_convex_hull_2d(result);
    }


}

namespace GEO {

    void compute_minkowski_sum_3d(
	Mesh& result, const Mesh& op1, const Mesh& op2
    ) {
	bool op1_is_convex = mesh_is_convex_3d(op1);
	bool op2_is_convex = mesh_is_convex_3d(op2);

	if(op1_is_convex && op2_is_convex) {
	    compute_minkowski_sum_convex_convex_3d(result, op1, op2);
	    return;
	}

	if(!op1_is_convex && op2_is_convex) {
	    compute_minkowski_sum_non_convex_convex_3d(result, op1, op2);
	    return;
	}

	if(op1_is_convex && !op2_is_convex) {
	    compute_minkowski_sum_non_convex_convex_3d(result, op2, op1);
	    return;
	}

	throw(
	    std::logic_error(
		"compute_Minkowski_sum_3d: "
		"non-convex case not implemented yet"
	    )
	);
    }

    void compute_minkowski_sum_2d(
	Mesh& result, const Mesh& op1, const Mesh& op2
    ) {
	if(mesh_is_convex_2d(op1) && mesh_is_convex_2d(op2)) {
	    compute_minkowski_sum_convex_convex_2d(result, op1, op2);
	    return;
	}

	throw(
	    std::logic_error(
		"compute_Minkowski_sum_2d: "
		"non-convex case not implemented yet"
	    )
	);
    }

}
