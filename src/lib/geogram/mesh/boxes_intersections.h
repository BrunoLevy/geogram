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
     * \brief Lower-level interface to the underlying algorithm (experts only).
     * \details Reports all pairs (i,p) such that box i contain
     *  the lower-bound point of box p in two sets of boxes I and P. If I and P
     *  are two different sequences, one must call this function twice to get
     *  all boxes intersection, the second time with the arguments swapped.
     *  This is the main algorithm described in:
     *      Fast software for box intersections,
     *      Afra Zoromodian and Herbert Edelsbrunner,
     *      International Journal of Computational Geometry & Applications, 2002
     * I range and P range are modified when calling this function (scrambled).
     * If I range indices and P range indices overlap, then one of them is copied
     * to a local buffer.
     * \param[in] Iboxes pointer to array of boxes refered to by I range
     * \param[in] Ib , Ie pointers to integers sequence refering to \p Iboxes
     * \param[in] Pboxes pointer to array of boxes refered to by P range
     * \param[in] Pb , Pe pointers to integers sequence refering to \p Pboxes
     * \param[in] callback the callback used to report intersections, takes
     *  two integers, i and p
     */
    void GEOGRAM_API boxes_intersections_hybrid_impl(
	const Box3d* Iboxes,
	index_t* Ib,
	index_t* Ie,
	const Box3d* Pboxes,
	index_t* Pb,
	index_t* Pe,
	std::function<void(index_t, index_t)> callback
    );
}

#endif
