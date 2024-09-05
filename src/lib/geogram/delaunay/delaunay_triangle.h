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

#ifndef GEOGRAM_DELAUNAY_DELAUNAY_TRIANGLE
#define GEOGRAM_DELAUNAY_DELAUNAY_TRIANGLE

/**
 * \file geogram/delaunay/delaunay_triangle.h
 * \brief Implementation of Delaunay in 2D using the triangle library
 *  by Jonathan Shewchuk.
 */

#ifdef GEOGRAM_WITH_TRIANGLE

#include <geogram/basic/common.h>
#include <geogram/delaunay/delaunay.h>

extern "C" {
#define REAL double
#define ANSI_DECLARATORS
#define VOID void
#include <geogram/third_party/triangle/triangle.h>
}

namespace GEO {

    /**
     * \brief Implementation of Delaunay using Jonathan Shewchuk's
     *  triangle library.
     */
    class GEOGRAM_API DelaunayTriangle : public Delaunay {
    public:
        /**
         * \brief Creates a new DelaunayTriangle.
         * \details DelaunayTetgen triangulations are only supported for
         * dimension 2. If a different dimension is specified in the
         * constructor, a InvalidDimension exception is thrown.
         * \param[in] dimension dimension of the triangulation
         * \throw InvalidDimension This exception is thrown if dimension is
         * different than 2.
         */
        DelaunayTriangle(coord_index_t dimension = 2);

        /**
         * \copydoc Delaunay::set_vertices()
         */
        void set_vertices(
            index_t nb_vertices, const double* vertices
        ) override;

        /**
         * \copydoc Delaunay::supports_constraints()
         */
        bool supports_constraints() const override;

        /**
         * \brief DelaunayTriangle destructor.
         */
        ~DelaunayTriangle() override;

    protected:

        /**
         * \brief Implementation of set_vertices() used when
         *  no constraint is defined.
         * \param[in] nb_vertices number of vertices
         * \param[in] vertices a const pointer to the
         *  coordinates of the vertices, as a continuous
         *  array of doubles.
         */
        void set_vertices_unconstrained(
            index_t nb_vertices, const double* vertices
        );

        /**
         * \brief Implementation of set_vertices() used when
         *  constraints are defined.
         * \details The constraints are specified by
         *  Delaunay::set_constraints().
         * \param[in] nb_vertices number of vertices
         * \param[in] vertices a const pointer to the
         *  coordinates of the vertices, as a continuous
         *  array of doubles.
         */
        void set_vertices_constrained(
            index_t nb_vertices, const double* vertices
        );


        struct triangulateio triangle_out_ ;
        struct triangulateio triangle_in_ ;
    };
}

#endif

#endif
