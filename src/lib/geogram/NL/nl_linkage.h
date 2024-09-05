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

#ifndef OPENL_LINKAGE_H
#define OPENL_LINKAGE_H

/**
 * \file geogram/NL/nl_linkage.h
 * \brief Specify linkage for OpenNL integrated into Geogram
 * \details (uses Geogram defines to select the
 *   right linkage modes for OpenNL symbols)
 */

#ifdef GEO_DYNAMIC_LIBS
#define NL_SHARED_LIBS
#ifdef geogram_EXPORTS
#define NL_EXPORTS
#endif
#endif

#ifdef WIN32
#ifdef NL_SHARED_LIBS
#ifdef NL_EXPORTS
#define NLAPIENTRY __declspec( dllexport )
#else
#define NLAPIENTRY __declspec( dllimport )
#endif
#else
#define NLAPIENTRY
#endif
#else
#ifdef NL_SHARED_LIBS
#define NLAPIENTRY __attribute__ ((visibility("default")))
#else
#define NLAPIENTRY
#endif
#endif

#ifdef __GNUC__
#define NL_DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define NL_DEPRECATED(func) __declspec(deprecated) func
#else
#define NL_DEPRECATED(func) func
#endif

#endif
