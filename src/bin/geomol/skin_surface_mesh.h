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

#ifndef GEOMOL_SKIN_SURFACE_MESH_H
#define GEOMOL_SKIN_SURFACE_MESH_H

#include <geogram/basic/common.h>
#include <geogram/basic/numeric.h>
#include <geogram/delaunay/delaunay.h>
#include <geogram/mesh/mesh.h>

/**
 * \file skin_surface_mesh.h
 * \brief Isotopic meshing of molecular skin surfaces.
 * \details Implementation of:
 *   N.G.H. Kruithof and G. Vegter,
 *   "Meshing skin surfaces with certified topology",
 *   CAD/Graphics 2005, Computational Geometry 36(3), 2007.
 *
 *   The skin surface skn^s(P) of a set of weighted points P (the atoms,
 *   seen as balls of squared radius = weight) with shrink factor s is
 *   decomposed by the mixed complex Mix^s(P) into patches of spheres
 *   (mixed cells of type 0 and 3) and hyperboloids (type 1 and 2).
 *   Each mixed cell is a convex polyhedron (1-s)*delta_X + s*nu_X, where
 *   delta_X is a Delaunay simplex and nu_X its dual power (Voronoi) cell.
 *
 *   The algorithm:
 *    1. computes the anchor points of all the faces of the mixed complex
 *       (Lemmas 13-16 of the paper): the anchor point of a polyhedral cell
 *       mu_{X,X'} = (1-s) delta_X + s nu_X' is (1-s) a(delta_X) + s a(nu_X'),
 *       where a(delta_X) is the point of delta_X closest to the center of X
 *       and a(nu_X') the point of nu_X' closest to the center of X'.
 *    2. triangulates the mixed cells into a tetrahedral complex whose
 *       vertices are the anchor points, such that every edge satisfies the
 *       monotonicity condition (Condition 17 / Lemma 19). Hence every
 *       tetrahedron intersects the skin surface in a topological disk
 *       (Theorem 21).
 *    3. extracts the surface with marching tetrahedra. By Theorem 21 the
 *       result is isotopic to the skin surface. Intersection points are
 *       computed exactly (root of the quadric along the edge).
 *
 *   Unbounded power cells are handled by clipping the power diagram with
 *   a bounding box (see the notes at the top of skin_surface_mesh.cpp),
 *   so that no additional points need to be inserted.
 *
 *   The intermediate tetrahedral complex can be retrieved for debugging.
 */

namespace GEO {

    /**
     * \brief Computes a coarse mesh isotopic to the skin surface of a set
     *  of weighted points.
     * \details The weighted Delaunay triangulation (power diagram) of the
     *  weighted points needs to be computed beforehand by the caller. In
     *  geomol, this is the PowerDiagram (derived from PeriodicDelaunay3d,
     *  that is itself derived from Delaunay), computed with
     *  keeps_infinite set. Infinite (unbounded) power cells are supported:
     *  the power diagram is clipped internally by a bounding box large
     *  enough to contain all the surface, there is no need to insert
     *  additional points (close_cells()). If additional points are
     *  nevertheless present (indices >= nb_atoms), the mixed cells
     *  incident to them are not meshed.
     */
    class SkinSurfaceMesher {
    public:

        /**
         * \brief SkinSurfaceMesher constructor
         * \param[in] delaunay the weighted Delaunay triangulation of the
         *  points, computed with keeps_infinite set (as in geomol).
         * \param[in] points pointer to the coordinates of the points
         *  (3 doubles per point), nb_vertices() of \p delaunay points.
         * \param[in] weights pointer to the weights of the points
         *  (1 double per point). The weight is the squared radius of the
         *  unshrunk ball. The ball of the skin surface has squared radius
         *  shrink_factor * weight.
         * \param[in] nb_atoms number of real atoms (usually
         *  delaunay.nb_vertices()). Points with index greater or equal to
         *  \p nb_atoms are considered as auxiliary points (e.g. inserted
         *  by close_cells()): their cells are not meshed.
         * \param[in] shrink_factor the shrink factor s, in ]0,1[.
         */
        SkinSurfaceMesher(
            const Delaunay& delaunay,
            const double* points,
            const double* weights,
            index_t nb_atoms,
            double shrink_factor
        );

        /**
         * \brief Computes the coarse mesh of the skin surface.
         * \param[out] skin the computed mesh. Facets are oriented with
         *  normals pointing outside the body of the skin surface. A facet
         *  attribute "mixed_cell_type" (index_t, in 0..3) indicates in
         *  which type of mixed cell each facet was generated.
         * \param[out] tet_complex optional. If non-null, receives the
         *  tetrahedral complex (the vertices have an attribute "value"
         *  with the value of the implicit function, and the cells have
         *  an attribute "mixed_cell_type").
         * \retval true on success.
         * \retval false if the triangulation is inconsistent.
         */
        bool compute(Mesh& skin, Mesh* tet_complex = nullptr);

        /**
         * \brief Sets verbosity.
         */
        void set_verbose(bool x) {
            verbose_ = x;
        }

        /**
         * \brief Gets the number of vertices (anchor points) of the
         *  tetrahedral complex generated by the latest call to compute().
         */
        index_t nb_complex_vertices() const {
            return nb_complex_vertices_;
        }

        /**
         * \brief Gets the number of (non-degenerate) tetrahedra of the
         *  tetrahedral complex generated by the latest call to compute().
         */
        index_t nb_complex_tets() const {
            return nb_complex_tets_;
        }

        /**
         * \brief Gets the number of degenerate tetrahedra that were
         *  skipped (collapsed anchor points, see Section 3.2 of the paper).
         */
        index_t nb_degenerate_tets() const {
            return nb_degenerate_tets_;
        }

    private:
        const Delaunay& delaunay_;
        const double* points_;
        const double* weights_;
        index_t nb_atoms_;
        double shrink_;
        bool verbose_;
        index_t nb_complex_vertices_;
        index_t nb_complex_tets_;
        index_t nb_degenerate_tets_;
    };

}

#endif
