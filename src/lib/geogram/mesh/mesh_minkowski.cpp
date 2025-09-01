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
	if(mesh_is_convex_3d(op1) && mesh_is_convex_3d(op2)) {
	    compute_minkowski_sum_convex_convex_3d(result, op1, op2);
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
