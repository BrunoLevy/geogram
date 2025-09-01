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

#include <geogram/mesh/mesh_convex_hull.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/delaunay/delaunay.h>
#include <geogram/numerics/predicates.h>

namespace GEO {

    void compute_convex_hull_2d(Mesh& mesh) {
	Delaunay_var delaunay = Delaunay::create(coord_index_t(2), "BDEL2d");
	delaunay->set_keeps_infinite(true);
        delaunay->set_vertices(mesh.vertices.nb(), mesh.vertices.point_ptr(0));
	mesh.edges.clear();
	for(index_t t=delaunay->nb_finite_cells(); t<delaunay->nb_cells(); ++t) {
	    index_t v1= NO_INDEX, v2=NO_INDEX;
	    for(index_t lv=0; lv<3; ++lv) {
		if(delaunay->cell_vertex(t,lv) == NO_INDEX) {
		    v1 = delaunay->cell_vertex(t,(lv+1)%3);
		    v2 = delaunay->cell_vertex(t,(lv+2)%3);
		}
	    }
	    geo_assert(v1 != NO_INDEX && v2 != NO_INDEX);
	    mesh.edges.create_edge(v2,v1);
	}
	mesh.vertices.remove_isolated();
    }

    void compute_convex_hull_3d(Mesh& mesh) {
	Delaunay_var delaunay = Delaunay::create(coord_index_t(3), "PDEL");
	delaunay->set_keeps_infinite(true);
        delaunay->set_vertices(mesh.vertices.nb(), mesh.vertices.point_ptr(0));
	mesh.facets.clear();
	for(index_t t=delaunay->nb_finite_cells(); t<delaunay->nb_cells(); ++t) {
	    index_t v0 = delaunay->cell_vertex(t,0);
	    index_t v1 = delaunay->cell_vertex(t,1);
	    index_t v2 = delaunay->cell_vertex(t,2);
	    index_t v3 = delaunay->cell_vertex(t,3);
	    if(v0 == NO_INDEX) {
		mesh.facets.create_triangle(v3,v2,v1);
	    } else if(v1 == NO_INDEX) {
		mesh.facets.create_triangle(v0,v2,v3);
	    } else if(v2 == NO_INDEX) {
		mesh.facets.create_triangle(v0,v3,v1);
	    } else if(v3 == NO_INDEX) {
		mesh.facets.create_triangle(v0,v1,v2);
	    }
	}
	mesh.vertices.remove_isolated();
	mesh.facets.connect();
    }
}
