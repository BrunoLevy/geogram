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

#ifndef GEOGRAM_MESH_MESH_REMESH
#define GEOGRAM_MESH_MESH_REMESH

#include <geogram/basic/common.h>
#include <geogram/basic/numeric.h>

/**
 * \file geogram/mesh/mesh_remesh.h
 * \brief Functions for remeshing
 */

namespace GEO {

    class Mesh;

    /**
     * \brief Remeshes a 'smooth' shape (that is, without management
     *  of sharp features).
     * \param[in] M_in input mesh
     * \param[out] M_out result
     * \param[in] nb_points desired number of points (note: may
     *  generate more points to solve problematic configurations)
     * \param[in] dim dimension in which to do the remesh. Use dim=6
     *  and set_anisotropy(M_in,s) for anisotropic remesh,
     *  dim=3 for isotropic remesh, dim=0 uses M_in.dimension().
     * \param[in] nb_Lloyd_iter number of Lloyd relaxation iterations
     *  (used to initialize Newton iterations with a more homogeneous
     *  distribution)
     * \param[in] nb_Newton_iter number of Newton iterations
     * \param[in] Newton_m number of evaluations used for
     *  Hessian approximation
     * \param[in] adjust if set, call mesh_adjust_surface() to improve
     *  the placement of the points in such a way that the facets of
     *  \p M_out better approximate \p M_in
     * \param[in] adjust_max_edge_distance distance along which
     *  searching for nearest vertex, relative to average
     *  edge length in the neighborhood of the considered
     *  vertex
     *
     * Example 1 - isotropic remesh:
     * \code
     * remesh_smooth(M_in, M_out, 30000, 3) ;
     * \endcode
     *
     * Example 2 - anisotropic remesh:
     * \code
     * set_anisotropy(M_in, 0.04) ;
     * remesh_smooth(M_in, M_out, 30000, 6) ;
     * \endcode
     */
    void GEOGRAM_API remesh_smooth(
        Mesh& M_in, Mesh& M_out,
        index_t nb_points,
        coord_index_t dim = 0,
        index_t nb_Lloyd_iter = 5,
        index_t nb_Newton_iter = 30,
        index_t Newton_m = 7,
        bool adjust = true,
        double adjust_max_edge_distance=0.5
    );

    /**
     * \brief Adjusts a surface mesh in such a way that
     *  minimizes its distance to a reference surface mesh
     * \param[in,out] surface the surface mesh to be adjusted
     * \param[in] reference the reference surface mesh
     * \param[in] max_edge_distance distance along which
     *  searching for nearest vertex, relative to average
     *  edge length in the neighborhood of the considered
     *  vertex
     * \param[in] project_borders if set, in a final post-processing,
     *  project the vertices on the border of the surface onto the
     *  borders of the reference surface. Whereas it improves a bit
     *  the borders, it results in a worse approximation on the facets
     *  adjacent to the border, hence it is off by default
     * \details Internally it uses an AABB, hence the order
     *  of the facets of \p reference can be changed.
     */
    void GEOGRAM_API mesh_adjust_surface(
        Mesh& surface,
        Mesh& reference,
        double max_edge_distance=0.5,
        bool project_borders=false
    );
}

#endif
