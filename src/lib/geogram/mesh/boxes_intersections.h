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

#ifndef GEOGRAM_MESH_BOXES_ISECT
#define GEOGRAM_MESH_BOXES_ISECT

/**
 * \file boxes_intersections.h
 * \brief Functions to compute all intersections in a vector of boxes
 */

#include <geogram/basic/common.h>
#include <geogram/basic/geometry.h>
#include <functional>

namespace GEO {

    /**
     * \brief Reports all intersections in a vector of boxes
     * \param[in] boxes the vector of boxes
     * \param[in] callback will be called for each pair of boxes that
     *  have an intersection in \p boxes. Parameters are the indices
     *  of each pair of boxes that have an intersection
     */
    void GEOGRAM_API boxes_intersections(
	const vector<Box3d>& boxes,
	std::function<void(index_t, index_t)> callback
    );

    /**
     * \brief Reports all intersections in a vector of boxes with groups
     * \details supposed to be more efficient than box_intersections() because
     *  it exploits the structure as groups.
     * \param[in] boxes the vector of boxes
     * \param[in] indices a vector of indices referring to \p boxes
     * \param[in] group pointers referring to \p indices. Has N+1 pointers,
     *  where N is the number of groups. Group g indices are
     *   indices[ [group_ptr[g] ... group_ptr[g+1]) ].
     * \param[in] callback will be called for each pair of boxes that
     *  have an intersection in \p boxes. Parameters are the indices
     *  of each pair of boxes that have an intersection.
     * \parma[in] self intersections if set, compute self-intersections
     *  within each group (default).
     */
    void GEOGRAM_API boxes_intersections_grouped(
	const vector<Box3d>& boxes,
	const vector<index_t>& indices,
	const vector<index_t>& group_ptr,
	std::function<void(index_t, index_t)> callback,
	bool self_intersections = true
    );

}

#endif
