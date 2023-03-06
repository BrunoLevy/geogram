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
 
#ifndef GEOGRAM_MESH_MESH_SURFACE_INTERSECTION
#define GEOGRAM_MESH_MESH_SURFACE_INTERSECTION

#include <geogram/basic/common.h>
#include <geogram/basic/numeric.h>
#include <functional>

/**
 * \file geogram/mesh/mesh_surface_intersection.h
 * \brief Functions for computing intersections between surfacic meshes and
 *        boolean operations.
 */

namespace GEO {

    class Mesh;

    /**
     * \brief Parameters for mesh_intersect_surface()
     */
    struct MeshSurfaceIntersectionParams {
        /**
         * \brief detect and fix duplicated vertices in input 
         */
        bool pre_detect_duplicated_vertices = true;
        
        /**
         * \brief detect and fix duplicated facets in input 
         */
        bool pre_detect_duplicated_facets = true;
        
        /** 
         * \brief detect and compute intersections between facets that share 
         *  a facet or an edge. Set to false if input is a set of conformal
         *  meshes.
         */
        bool detect_intersecting_neighbors = true;

        /**
         * \brief connect the facets of the arrangement and split the 
         *  non-manifold edges
         */
        bool post_connect_facets = true;

        /**
         * \brief if set, assign an operand id to each connected component
         *  of input.
         */
        bool per_component_ids = true;

        /**
         * \brief If set, compute constrained Delaunay triangulation
         *  in the intersected triangles. If there are intersections
         *  in coplanar facets, it guarantees uniqueness of their
         *  triangulation.
         */
        bool delaunay = false;

        /**
         * \brief If set, then Delaunay mode uses approximated incircle
         *  predicate (else it uses exact arithmetics)
         */
        bool approx_incircle = false;

        /**
         * \brief Do not order the facets in the AABB. Used for debugging,
         *  for keeping facet ids (need to order the facets before, else
         *  the AABB will take ages).
         */
        bool debug_do_not_order_facets = false;

        /**
         * \brief Enable floating point exceptions, to detect overflows and
         *  underflows (shit happens ! [Forrest Gump]).
         */
        bool debug_enable_FPE = true;

        /**
         * \brief Display information while computing the intersection
         */
        bool verbose = true;
    };

    /**
     * \brief Makes a surface mesh conformal by computing all the intersections.
     * \param[in,out] M the surface mesh
     * \param[in] params parameters
     * \see MeshSurfaceIntersectionParams
     * \details A facet attribute of type index_t named "operand_bit" can 
     *  indicate for each facet to which operand of a n-ary boolean operation
     *  it corresponds to (the same facet might belong to several operands). 
     *  It is taken into account by the two variants of 
     *  mesh_classify_intersections()
     */
    void GEOGRAM_API mesh_intersect_surface(
        Mesh& M, const MeshSurfaceIntersectionParams& params
    );

    /**
     * \brief Classifies the facets of the result of mesh_intersect_surface()
     *  based on a boolean function
     * \param[in,out] M the surface mesh with the result of 
     *  mesh_intersect_surface()
     * \param[in] eqn the boolean function. Each bit of its parameter 
     *  corresponds to an operand (among 32). 
     * \param[in] attribute if an attribute name is specified, then this 
     *  attribute is set for all facets on the boundary of the computed object,
     *  (besides that the mesh is not modified). If an attribute name is not
     *  specified, then all the facets that are not on the boundary of the
     *  computed object are discarded.
     * \param[in] reorder if the intersection was just computed, one does not
     *  need to reorder the facets and one can set this parameter to false.
     */
    void GEOGRAM_API mesh_classify_intersections(
        Mesh& M, std::function<bool(index_t)> eqn,
        const std::string& attribute="", bool reorder=true
    );

    /**
     * \brief Classifies the facets of the result of mesh_intersect_surface()
     *  based on a boolean function
     * \param[in,out] M the surface mesh with the result of 
     *  mesh_intersect_surface()
     * \param[in] expr the boolean function in ASCII. One can use the following
     *  elements, and parentheses:
     *  - Variables: A..Z or x0..x31 
     *  - and:        '&' or '*'
     *  - or:         '|' or '+'
     *  - xor:        '^'
     *  - difference: '-'
     *  - not:        '!' or '~'
     *  Special values for expr: 
     *  - "union" (union of everything)
     *  - "intersection" (intersection of everything).
     * \param[in] attribute if an attribute name is specified, then this 
     *  attribute is set for all facets on the boundary of the computed object,
     *  (besides that the mesh is not modified). If an attribute name is not
     *  specified, then all the facets that are not on the boundary of the
     *  computed object are discarded.
     * \param[in] reorder if the intersection was just computed, one does not
     *  need to reorder the facets and one can set this parameter to false.
     */
    void GEOGRAM_API mesh_classify_intersections(
        Mesh& M, const std::string& expr,
        const std::string& attribute="", bool reorder=true
    );
    
}

#endif

