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

#ifndef GEOGRAM_BASIC_DETERMINANT
#define GEOGRAM_BASIC_DETERMINANT

#include <geogram/basic/common.h>

/**
 * \file geogram/basic/determinant.h
 * \brief Determinants for small sizes
 */

namespace GEO {

    /************************************************************************/

    /**
     * \brief Computes a two-by-two determinant.
     */
    template <class T>
    inline T det2x2(
        const T& a11, const T& a12,                    
        const T& a21, const T& a22
    ) {                                 
        return a11*a22-a12*a21 ;
    }

    /**
     * \brief Computes a three-by-three determinant.
     */
    template <class T>    
    inline T det3x3(
        const T& a11, const T& a12, const T& a13,                
        const T& a21, const T& a22, const T& a23,                
        const T& a31, const T& a32, const T& a33
    ) {
    return
         a11*det2x2(a22,a23,a32,a33)   
        -a21*det2x2(a12,a13,a32,a33)   
        +a31*det2x2(a12,a13,a22,a23);
    }   


    /**
     * \brief Computes a four-by-four determinant.
     */
    template <class T>    
    inline T det4x4(
        const T& a11, const T& a12, const T& a13, const T& a14,
        const T& a21, const T& a22, const T& a23, const T& a24,               
        const T& a31, const T& a32, const T& a33, const T& a34,  
        const T& a41, const T& a42, const T& a43, const T& a44  
    ) {
        T m12 = a21*a12 - a11*a22;
        T m13 = a31*a12 - a11*a32;
        T m14 = a41*a12 - a11*a42;
        T m23 = a31*a22 - a21*a32;
        T m24 = a41*a22 - a21*a42;
        T m34 = a41*a32 - a31*a42;

        T m123 = m23*a13 - m13*a23 + m12*a33;
        T m124 = m24*a13 - m14*a23 + m12*a43;
        T m134 = m34*a13 - m14*a33 + m13*a43;
        T m234 = m34*a23 - m24*a33 + m23*a43;
        
        return (m234*a14 - m134*a24 + m124*a34 - m123*a44);
    }   
}

#endif
