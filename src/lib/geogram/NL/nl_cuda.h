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

#ifndef OPENNL_CUDA_EXT_H
#define OPENNL_CUDA_EXT_H

#include "nl_private.h"
#include "nl_matrix.h"
#include "nl_blas.h"

/**
 * \file geogram/NL/nl_cuda.h
 * \brief Internal OpenNL functions that interface with CUDA.
 */

/**
 * \brief Initializes the CUDA extension
 * \details This dynamically loads the CUDA
 *  libraries available in the system (if available) and
 *  retrieves the symbols in there.
 * \retval NL_TRUE if CUDA could be successfully
 *   dynamically loaded and all functions could be
 *   found in it.
 * \retval NL_FALSE otherwise.
 * \note For now, only implemented under Linux in
 *  dynamic libraries mode.
 */
NLboolean nlInitExtension_CUDA(void);

/**
 * \brief Tests whether the CUDA extension is initialized.
 * \retval NL_TRUE if the CUDA extension is initialized.
 * \retval NL_FALSE otherwise.
 */
NLboolean nlExtensionIsInitialized_CUDA(void);


/****************************************************************************/

/**
 * \brief Creates a CUDA on-GPU matrix from an OpenNL CRS matrix.
 * \details Calling nlMultMatrixVector() with the created matrix only works
 *  with vectors that reside on the GPU.
 * \param[in] M the OpenNL CRS matrix to be copied.
 * \return a handle to the CUDA matrix.
 */
NLMatrix nlCUDAMatrixNewFromCRSMatrix(NLMatrix M);

/**
 * \brief Creates a CUDA on-GPU Jacobi preconditioner from an OpenNL CRS matrix.
 * \details Calling nlMultMatrixVector() with the created matrix only works
 *  with vectors that reside on the GPU.
 * \param[in] M the OpenNL CRS matrix.
 * \return a handle to the created CUDA matrix.
 */
NLMatrix nlCUDAJacobiPreconditionerNewFromCRSMatrix(NLMatrix M);

/**
 * \brief Computes a sparse matrix vector product
 * \details Computes \f$ y \leftarrow alpha M x + beta y \f$
 *   As compared to NL abstract matrix API, it has the \p alpha and \p beta
 *   parameters.
 * \param[in] M a matrix created from nlCUDAMAtrixNewFromCRSMatrix()
 * \param[in] x device pointer
 * \param[in,out] y device pointer
 * \param[in] alpha , beta two scalars
 */
void nlCUDAMatrixSpMV(
    NLMatrix M, const double* x, double* y, double alpha, double beta
);

/**
 * \brief Gets a pointer to the BLAS abstraction layer for
 *  BLAS operation on the GPU using CUDA.
 * \return a pointer to the BLAS abstraction layer.
 */
NLBlas_t nlCUDABlas(void);

/****************************************************************************/

#endif
