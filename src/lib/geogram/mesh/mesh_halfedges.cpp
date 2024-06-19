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

#include <geogram/mesh/mesh_halfedges.h>

namespace GEO {

    bool MeshHalfedges::move_to_next_around_vertex(Halfedge& H, bool ignore_borders) const {
        geo_debug_assert(halfedge_is_valid(H));
        index_t v = Geom::halfedge_vertex_index_from(mesh_,H); // get the vertex at the origin of H
        index_t f = Geom::halfedge_facet_secondary(mesh_,H); // get the facet at the other side of H (relative to H.facet)
        if(f == NO_FACET) {
            return false; // cannot move
        }
        if(
            ignore_borders == false &&
            facet_region_.is_bound() &&
            facet_region_[H.facet] != facet_region_[f]
        ) {
            return false; // cannot move without crossing a border (end criteria when looping around a vertex, see move_to_*_around_border())
        }
        for(index_t c: mesh_.facets.corners(f)) { // for each corner of the facet f
            index_t pc = mesh_.facets.prev_corner_around_facet(f, c); // get the previous corner clockwise
            if(
                mesh_.facet_corners.vertex(c) == v && // same origin vertex
                mesh_.facet_corners.adjacent_facet(pc) == H.facet // previous facet is adjacent
            ) {
                H.corner = c;
                H.facet = f;
                return true; // successful move
            }
        }
        geo_assert_not_reached;
    }

    bool MeshHalfedges::move_to_prev_around_vertex(Halfedge& H, bool ignore_borders) const {
        geo_debug_assert(halfedge_is_valid(H));
        index_t v = Geom::halfedge_vertex_index_from(mesh_,H); // get the vertex at the origin of H
        index_t pc = mesh_.facets.prev_corner_around_facet(H.facet, H.corner); // get the previous corner clockwise
        index_t f = mesh_.facet_corners.adjacent_facet(pc); // the facet after H.facet clockwise
        if(f == NO_FACET) {
            return false; // cannot move
        }
        if(
            ignore_borders == false &&
            facet_region_.is_bound() &&
            facet_region_[H.facet] != facet_region_[f]
        ) {
            return false; // cannot move without crossing a border (end criteria when looping around a vertex, see move_to_*_around_border())
        }
        for(index_t c: mesh_.facets.corners(f)) {
            if(
                mesh_.facet_corners.vertex(c) == v && // same origin vertex
                mesh_.facet_corners.adjacent_facet(c) == H.facet // previous facet is adjacent
            ) {
                H.corner = c;
                H.facet = f;
                return true; // successful move
            }
        }
        geo_assert_not_reached;
    }

    void MeshHalfedges::move_to_next_around_border(Halfedge& H) const {
        geo_debug_assert(halfedge_is_valid(H));
        geo_debug_assert(halfedge_is_border(H));
        move_to_next_around_facet(H);
        index_t count = 0;
        while(move_to_next_around_vertex(H)) {
            ++count;
            geo_assert(count < 10000);
        }
    }

    void MeshHalfedges::move_to_prev_around_border(Halfedge& H) const {
        geo_debug_assert(halfedge_is_valid(H));
        geo_debug_assert(halfedge_is_border(H));
        index_t count = 0;
        while(move_to_prev_around_vertex(H)) {
            ++count;
            geo_assert(count < 10000);
        }
        move_to_prev_around_facet(H);        
    }
    
    void MeshHalfedges::move_to_opposite(Halfedge& H) const {
        geo_debug_assert(halfedge_is_valid(H));
        index_t v = mesh_.facet_corners.vertex(
            mesh_.facets.next_corner_around_facet(H.facet, H.corner)
        );
        index_t f = mesh_.facet_corners.adjacent_facet(H.corner);
        geo_assert(f != NO_FACET);
        for(index_t c: mesh_.facets.corners(f)) {
            if(mesh_.facet_corners.vertex(c) == v) {
                H.facet = f;
                H.corner = c;
                return;
            }
        }
        geo_assert_not_reached;
    }
}

