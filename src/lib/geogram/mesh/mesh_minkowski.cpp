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

    void compute_minkowski_sum_non_convex_convex_3d(
	Mesh& result, const Mesh& op1, const Mesh& op2
    ) {
	// TODO: why needed ?
	reorient_connected_components(const_cast<Mesh&>(op1));
	reorient_connected_components(const_cast<Mesh&>(op2));


	vector<index_t> op1_f_v_contrib(op1.facets.nb(), NO_INDEX);
	vector<index_t> op1_v_f(op1.facets.nb(), NO_INDEX);

	// Store one facet incident to each vertex
	for(index_t f: op1.facets) {
	    for(index_t v: op1.facets.vertices(f)) {
		op1_v_f[v] = f;
	    }
	}

	auto op1_vertex_is_elevated = [&](index_t v, const vec3& V)->bool {
	    double v_dot = dot(op1.vertices.point(v), V);
	    index_t first_f = op1_v_f[v];
	    index_t f = first_f;
	    index_t lv = op1.facets.find_vertex(f,v);
	    do {
		index_t N = op1.facets.nb_vertices(f);
		index_t v2 = op1.facets.vertex(f, (lv + 1) % N);
		double v2_dot = dot(op1.vertices.point(v2),V);
		if(v2_dot > v_dot) {
		    return false;
		}
		f = op1.facets.adjacent(f,lv);
		lv = op1.facets.find_vertex(f,v);
	    } while(f != first_f);
	    return true;
	};


	// Translated facets
	// TODO: planar Minkovski sum if several contributing vertices
	for(index_t f: op1.facets) {
	    vec3 V = Geom::mesh_facet_normal(op1, f);
	    index_t furthest_v = NO_INDEX;
	    double furthest_dot = Numeric::min_float64();
	    for(index_t v: op2.vertices) {
		double this_dot = dot(op2.vertices.point(v),V);
		if(this_dot > furthest_dot) {
		    furthest_v = v;
		    furthest_dot = this_dot;
		}
	    }
	    op1_f_v_contrib[f] = furthest_v;
	    vec3 T = op2.vertices.point(furthest_v);
	    index_t N = op1.facets.nb_vertices(f);
	    index_t first_v = result.vertices.create_vertices(N);
	    index_t new_f = result.facets.create_polygon(N);
	    for(index_t lv=0; lv<N; ++lv) {
		result.facets.set_vertex(new_f, lv, first_v+lv);
		result.vertices.point(first_v+lv) = op1.facets.point(f,lv) + T;
	    }
	}

	// Corner facets
	for(index_t f: op2.facets) {
	    vec3 V = Geom::mesh_facet_normal(op2, f);
	    for(index_t v: op1.vertices) {
		if(op1_vertex_is_elevated(v, V)) {
		    vec3 T = op1.vertices.point(v);
		    index_t N = op2.facets.nb_vertices(f);
		    index_t first_v = result.vertices.create_vertices(N);
		    index_t new_f = result.facets.create_polygon(N);
		    for(index_t lv=0; lv<N; ++lv) {
			result.facets.set_vertex(new_f, lv, first_v+lv);
			result.vertices.point(first_v+lv) =
			    op2.facets.point(f,lv) + T;
		    }
		}
	    }
	    // TODO: edges of op1 elevated w.r.t. V
	}

	// Edge facets
	for(index_t f1: op1.facets) {
	    for(index_t le1=0; le1<op1.facets.nb_vertices(f1); ++le1) {
		index_t g1 = op1.facets.adjacent(f1,le1);

		if(g1 == NO_INDEX || g1 > f1) {
		    continue;
		}

		if(
		    op1_f_v_contrib[f1] == NO_INDEX ||
		    op1_f_v_contrib[g1] == NO_INDEX ||
		    op1_f_v_contrib[f1] == op1_f_v_contrib[g1]
		) {
		    continue;
		}

		if(edge_convexity(op1,f1,le1) == POSITIVE) {
		    continue;
		}

		vec3 V11 = Geom::mesh_facet_normal(op1,f1);
		vec3 V12 = Geom::mesh_facet_normal(op1,g1);
		vec3 p1 = op1.facets.point(f1,le1);
		vec3 p2 = op1.facets.point(
		    f1, (le1 + 1) % op1.facets.nb_vertices(f1)
		);
		vec3 E1 = p2-p1;
		for(index_t f2: op2.facets) {
		    for(index_t le2=0; le2<op2.facets.nb_vertices(f2); ++le2) {
			index_t g2 = op2.facets.adjacent(f2, le2);

			if(g2 == NO_INDEX || g2 > f2) {
			    continue;
			}
			vec3 V21 = Geom::mesh_facet_normal(op2,f2);
			vec3 V22 = Geom::mesh_facet_normal(op2,g2);
			vec3 q1 = op2.facets.point(f2,le2);
			vec3 q2 = op2.facets.point(
			    f2, (le2 + 1) % op2.facets.nb_vertices(f2)
			);

			Sign s = geo_sgn(dot(E1,V21));

			if(s == geo_sgn(dot(E1,V22))) {
			    continue;
			}


			vec3 E2 = q2-q1;
			vec3 Vnf = cross(E1,E2);

			// HERE: "<=" leaves garbage (translated coplanar
			// facets)
			//       "<" missing facets
			if(
			    s*dot(cross(V11,Vnf),E1) < 0 &&
			    s*dot(cross(Vnf,V12),E1) < 0
   		        ) {
			    index_t first_v = result.vertices.create_vertices(4);
			    index_t new_f = result.facets.create_polygon(4);
			    for(index_t lv=0; lv<4; ++lv) {
				result.facets.set_vertex(new_f, lv, first_v+lv);
			    }
			    result.vertices.point(first_v  ) = p1+q1;
			    result.vertices.point(first_v+1) = p1+q2;
			    result.vertices.point(first_v+2) = p2+q2;
			    result.vertices.point(first_v+3) = p2+q1;
			}
		    }
		}
	    }
	}
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
