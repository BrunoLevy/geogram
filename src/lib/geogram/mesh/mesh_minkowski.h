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

#ifndef GEOGRAM_MESH_MESH_MINKOWSKI
#define GEOGRAM_MESH_MESH_MINKOWSKI

#include <geogram/basic/common.h>

namespace GEO {

    class Mesh;

    /**
     * \brief Computes the Minkowski sum of two 3D meshes
     * \param[in] op1 , op2 two surfacic meshes
     * \param[out] result the Minkowski sum of \p op1 and \p op2
     * \details Not implemented yet
     */
    void GEOGRAM_API compute_minkowski_sum_3d(
	Mesh& result, const Mesh& op1, const Mesh& op2
    );

    /**
     * \brief Computes the Minkowski sum of two 2D meshes, represented as
     *  edges
     * \param[in] op1 , op2 two 2D meshes
     * \param[out] result the Minkowski sum of \p op1 and \p op2
     * \details Not implemented yet
     */
    void GEOGRAM_API compute_minkowski_sum_2d(
	Mesh& result, const Mesh& op1, const Mesh& op2
    );

}

#endif
