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

#ifndef GEOGRAM_MESH_MESH_TETRAHEDRALIZE
#define GEOGRAM_MESH_MESH_TETRAHEDRALIZE

#include <geogram/basic/common.h>

/**
 * \file geogram/mesh/mesh_tetrahedralize.h
 * \brief Functions for filling a surface mesh with tetrahedra.
 */

namespace GEO {

    class Mesh;

    /**
     * \brief Parameters for mesh_tetrahedralize()
     */
    struct MeshTetrahedralizeParameters {
        /**
         * \brief Tentatively fix mesh before tetrahedralization
         * \details Merges duplicated vertices, fills small holes,
         *  removes intersections and extracts external boundary.
         */
        bool preprocess = false;
        /**
         * \brief Maximum distance for merging vertices if preprocess is set
         * \details In percent of bounding box diagonal
         */
        double preprocess_merge_vertices_epsilon = 0.001;
        /**
         * \brief Maximum area for filling a hole if preprocess is set
         * \details In percent of total surface mesh area
         */
        double preprocess_fill_hole_max_area = 0.01;
        /**
         * \brief Inserts additional vertices to improve mesh quality
         */
        bool refine = true;
        /**
         * \brief Desired mesh quality
         * \details it is typically in [1.0, 2.0], it specifies
         *  the desired quality of mesh elements (1.0 means maximum
         *  quality, and generates a higher number of elements).
         */
        double refine_quality = 2.0;
        /**
         * \brief keep internal boundaries and what is inside
         * \details If set, then all internal regions are kept, and
         *  a region cell attribute is created, else only tetrahedra in the
         *  outermost region are kept.
         */
        bool keep_regions = false;
        /**
         * \brief display status messages
         */
        bool verbose = true;
    };

    /**
     * \brief Fills a closed surface mesh with tetrahedra.
     * \details A constrained Delaunay triangulation algorithm
     *  needs to be interfaced (e.g., compiling with tetgen support).
     * \param [in,out] M a reference to the input surface mesh. On exit,
     *  the (optionally pre-processed) same surface mesh filled with
     *  tetrahedra
     * \param [in] params a reference to a MeshTetrahedralizeParameters
     * \retval true if the mesh was successfuly tetrahedralized
     * \retval false otherwise
     * \note needs a constrained Delaunay algorithm to work (geogram needs
     *  to be compiled with mg-tetra or tetgen).
     */
    bool GEOGRAM_API mesh_tetrahedralize(
        Mesh& M, const MeshTetrahedralizeParameters& params
    );

    /**
     * \brief Fills a closed surface mesh with tetrahedra.
     * \details A constrained Delaunay triangulation algorithm
     *  needs to be interfaced (e.g., compiling with tetgen support).
     * \param [in,out] M a reference to a mesh
     * \param [in] preprocess if true, the surface mesh is preprocessed
     *  to fix some defects (small gaps and intersections). If preprocess
     *  is set and borders are detected after preprocessing, then the function
     *  returns false. If preprocess is set to false, then the caller is
     *  supposed to provide a correct set of input constraints (that may have
     *  dangling borders / internal constraints).
     * \param [in] refine if true, inserts additional vertices to improve
     *  the quality of the mesh elements
     * \param[in] quality typically in [1.0, 2.0], specifies
     *  the desired quality of mesh elements (1.0 means maximum
     *  quality, and generates a higher number of elements).
     * \param[in] keep_regions if set, then all internal regions are kept, and
     *  a region cell attribute is created, else only tetrahedra in the
     *  outermost region are kept.
     * \param[in] eps threshold for merging verties if preprocess is set, in
     *  percentage of bounding box diagonal. Use 0 to merge strictly colocated
     *  vertices
     * \retval true if the mesh was successfuly tetrahedralized
     * \retval false otherwise
     * \note needs a constrained Delaunay algorithm to work (geogram needs
     *  to be compiled with mg-tetra or tetgen).
     */
    inline bool GEOGRAM_API mesh_tetrahedralize(
        Mesh& M, bool preprocess=true, bool refine=false, double quality=2.0,
        bool keep_regions=false, double eps = 0.001
    ) {
        MeshTetrahedralizeParameters params;
        params.preprocess = preprocess;
        params.preprocess_merge_vertices_epsilon = eps;
        params.refine = refine;
        params.refine_quality = quality;
        params.keep_regions = keep_regions;
        return mesh_tetrahedralize(M, params);
    }
}

#endif
