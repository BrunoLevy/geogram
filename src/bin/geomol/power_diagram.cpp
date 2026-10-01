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

#include "power_diagram.h"

namespace GEO {

    void PowerDiagram::compute_skeleton(index_t max_v) {
	if(max_v == NO_INDEX) {
	    max_v = nb_vertices();
	}
	skel_ptr_.assign(max_v+1, 0); // +1 because there is a "sentry"
	// Step 1: compute number of tets incident to each vertex
	for(index_t t=0; t<nb_tets(); ++t) {
	    for(index_t lv=0; lv<4; ++lv) {
		index_t v = tet_vertex(t,lv);
		if(v < max_v) {
		    skel_ptr_[v+1]++;
		}
	    }
	}
	// Step 2: compute number of edges incident to each vertex
	// using Euler-Poincaré characteristic (Nicolas Ray's idea):
	// The outer boundary of the set of tets incident to a
	// vertex v is a topological sphere, hence V-E+F=2, where:
	//   V = number of edges incident to v
	//   E = 3F/2
	//   F = number of tetrahedra incident to v
	// Hence, V - 3F/2 + F = 2, or V = 2 + F/2, and finally:
	// Number of incident edges = 2 + (number of incident tets)/2
	for(index_t v=0; v<max_v; ++v) {
	    geo_debug_assert((skel_ptr_[v] & 1) == 0);
	    skel_ptr_[v] = 2 + skel_ptr_[v]/2;
	}
	// Step 3: update row pointers and assign space for skel_h_
	for(index_t v=1; v<=max_v; ++v) {
	    skel_ptr_[v] += skel_ptr_[v-1];
	}
	skel_h_.assign(skel_ptr_[max_v], NO_INDEX);
	// Step 4: insert all halfedges in skel_h_, keeping a single
	// halfedge per oriented edge (see insert_in_skel()).
	for(index_t t: tets()) {
	    for(index_t lv1=0; lv1<4; ++lv1) {
		for(index_t lv2=0; lv2<4; ++lv2) {
		    if(lv1 == lv2) {
			continue;
		    }
		    index_t v1 = tet_vertex(t,lv1);
		    index_t v2 = tet_vertex(t,lv2);
		    if(v1 < max_v && v2 != NO_INDEX) {
			index_t h = make_halfedge_from_t_lv_lv(t, lv1, lv2);
			insert_in_skel(v1,v2,h);
		    }
		}
	    }
	}
    }

    vec3 PowerDiagram::radical_point(
	index_t v0, index_t v1, index_t v2, index_t v3
    ) const {
	vec3 p0 = vertex(v0);
	vec3 p1 = vertex(v1);
	vec3 p2 = vertex(v2);
	vec3 p3 = vertex(v3);
	double h0 = length2(p0) - weight(v0);
	double h1 = length2(p1) - weight(v1);
	double h2 = length2(p2) - weight(v2);
	double h3 = length2(p3) - weight(v3);
	mat3 M = {
	    {p1.x-p0.x, p1.y-p0.y, p1.z-p0.z},
	    {p2.x-p0.x, p2.y-p0.y, p2.z-p0.z},
	    {p3.x-p0.x, p3.y-p0.y, p3.z-p0.z}
	};
	return M.inverse() * (0.5*vec3{h1-h0, h2-h0, h3-h0});
    }

    vec3 PowerDiagram::radical_point(index_t v0, index_t v1, index_t v2) const {
	vec3 p0 = vertex(v0);
	vec3 p1 = vertex(v1);
	vec3 p2 = vertex(v2);
	double w0 = weight(v0);
	double w1 = weight(v1);
	double w2 = weight(v2);
	vec3 U = p1 - p0;
	vec3 V = p2 - p0;
	double UU = dot(U,U);
	double UV = dot(U,V);
	double VV = dot(V,V);
	mat2 M = {
	    { UU, UV },
	    { UV, VV }
	};
	vec2 uv = M.inverse() * (0.5*vec2{w0-w1+UU, w0-w2+VV});
	return p0 + uv.x*U + uv.y*V;
    }

    vec3 PowerDiagram::radical_point(index_t v0, index_t v1) const {
	vec3 p0 = vertex(v0);
	vec3 p1 = vertex(v1);
	double w0 = weight(v0);
	double w1 = weight(v1);
	vec3 U = p1 - p0;
	double UU = length2(U);
	double u = (UU + w0 - w1) / (2.0 * UU);
	return p0 + u * U;
    }

}
