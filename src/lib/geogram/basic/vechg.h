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

#ifndef GEOGRAM_BASIC_VECHG
#define GEOGRAM_BASIC_VECHG

#include <geogram/basic/common.h>
#include <geogram/basic/vecg.h>
#include <geogram/basic/rationalg.h>

/**
 * \file geogram/basic/vechg.h
 * \brief Generic implementation of geometric vectors in homogeneous coordinates
 */

namespace GEO {


    /************************************************************************/

    /**
     * \brief 2d vector with homogeneous coordinates
     */
    template <class T> class vec2Hg {
    public:
        /** \brief The type of the vector coordinates */
        typedef T value_type;

        vec2Hg() = default;
        
        vec2Hg(const T& x_in, const T& y_in, const T& w_in) :
            x(x_in),
            y(y_in),
            w(w_in) {
        }

        vec2Hg(double x_in, double y_in, double w_in) :
            x(x_in),
            y(y_in),
            w(w_in) {
        }        

        vec2Hg(T&& x_in, T&& y_in, T&& w_in) :
            x(x_in),
            y(y_in),
            w(w_in) {
        }

        vec2Hg(const vec2Hg& rhs) = default;

        vec2Hg(vec2Hg&& rhs) = default;

        template <class T2> explicit vec2Hg(const vecng<2,T2>& rhs) : 
            x(rhs.x),
            y(rhs.y),
            w(1.0) {
        }

        template <class T2> explicit vec2Hg(const vec2Hg<T2>& rhs) : 
            x(rhs.x),
            y(rhs.y),
            w(rhs.w) {
        }
        
        vec2Hg& operator=(const vec2Hg& rhs) = default;
        vec2Hg& operator=(vec2Hg&& rhs) = default;

        T* data() {
            return &x;
        }

        const T* data() const {
            return &x;
        }

        T& operator[](coord_index_t i) {
            geo_debug_assert(i < 2);
            return data()[i];
        }

        const T& operator[](coord_index_t i) const {
            geo_debug_assert(i < 2);
            return data()[i];
        }

        void optimize() {
            Numeric::optimize_number_representation(x);
            Numeric::optimize_number_representation(y);
            Numeric::optimize_number_representation(w);
        }
        
        T x;
        T y;
        T w;
    };

    /************************************************************************/

    template <class T> inline vec2Hg<T> operator-(
        const vec2Hg<T>& p1, const vec2Hg<T>& p2
    ) {
        if(p2.w == p1.w) {
            return vec2Hg<T>(
                p1.x-p2.x,
                p1.y-p2.y,
                p1.w
            );
        }
        return vec2Hg<T>(
            det2x2(p1.x,p1.w,p2.x,p2.w),
            det2x2(p1.y,p1.w,p2.y,p2.w),
            p1.w*p2.w
        );
    }
    
    /************************************************************************/

    /**
     * \brief Comparator class for vec2Hg
     * \detail Used to create maps indexed by vec2Hg or 
     *  SOS symbolic perturbation
     */
    template <class T> class vec2HgLexicoCompare {
    public:
       /**
        * \brief Compares two vec2Hg
        * \retval true if \p v1 is before \p v2 in the lexicographic
        *  order
        * \retval false otherwise
        */
        bool operator()(const vec2Hg<T>& v1, const vec2Hg<T>& v2) const {
            Sign s = Numeric::ratio_compare(v2.x, v2.w, v1.x, v1.w);
            if(s == POSITIVE) {
                return true;
            }
            if(s == NEGATIVE) {
                return false;
            }
            s = Numeric::ratio_compare(v2.y, v2.w, v1.y, v1.w);
            return (s == POSITIVE);
        }
    };
    
    /************************************************************************/
    
    /**
     * \brief 3d vector with homogeneous coordinates
     */
    template <class T> class vec3Hg {
    public:
        /** \brief The type of the vector coordinates */
        typedef T value_type;

        vec3Hg() = default;
        
        vec3Hg(const T& x_in, const T& y_in, const T& z_in, const T& w_in) :
            x(x_in),
            y(y_in),
            z(z_in),
            w(w_in) {
        }

        vec3Hg(T&& x_in, T&& y_in, T&& z_in, T&& w_in) :
            x(x_in),
            y(y_in),
            z(z_in),
            w(w_in) {
        }

        vec3Hg(double x_in, double y_in, double z_in, double w_in) :
            x(x_in),
            y(y_in),
            z(z_in),
            w(w_in) {
        }        
        
        vec3Hg(const vec3Hg& rhs) = default;

        vec3Hg(vec3Hg&& rhs) = default;

        template <class T2> explicit vec3Hg(const vecng<3,T2>& rhs) : 
            x(rhs.x),
            y(rhs.y),
            z(rhs.z),
            w(1.0) {
        }

        template <class T2> explicit vec3Hg(const vec3Hg<T2>& rhs) : 
            x(rhs.x),
            y(rhs.y),
            z(rhs.z),
            w(rhs.w) {
        }
        
        vec3Hg& operator=(const vec3Hg& rhs) = default;
        vec3Hg& operator=(vec3Hg&& rhs) = default;

        T* data() {
            return &x;
        }

        const T* data() const {
            return &x;
        }

        T& operator[](coord_index_t i) {
            geo_debug_assert(i < 3);
            return data()[i];
        }

        const T& operator[](coord_index_t i) const {
            geo_debug_assert(i < 3);
            return data()[i];
        }

        void optimize() {
            Numeric::optimize_number_representation(x);
            Numeric::optimize_number_representation(y);
            Numeric::optimize_number_representation(z);
            Numeric::optimize_number_representation(w);
        }
        
        T x;
        T y;
        T z;
        T w;
    };

    /************************************************************************/
    
    template <class T> inline vec3Hg<T> operator-(
        const vec3Hg<T>& p1, const vec3Hg<T>& p2
    ) {
        if(p1.w == p2.w) {
            return vec3Hg<T>(
                p1.x - p2.x,
                p1.y - p2.y,
                p1.z - p2.z,
                p1.w
            );
        }
        return vec3Hg<T>(
            det2x2(p1.x,p1.w,p2.x,p2.w),
            det2x2(p1.y,p1.w,p2.y,p2.w),
            det2x2(p1.z,p1.w,p2.z,p2.w),            
            p1.w * p2.w
        );
    }

    /************************************************************************/

    /**
     * \brief Comparator class for vec3Hg
     * \detail Used to create maps indexed by vec3Hg or 
     *  SOS symbolic perturbation
     */
    template <class T> class vec3HgLexicoCompare {
    public:
       /**
        * \brief Compares two vec3Hg
        * \retval true if \p v1 is before \p v2 in the lexicographic
        *  order
        * \retval false otherwise
        */
        bool operator()(const vec3Hg<T>& v1, const vec3Hg<T>& v2) const {
            Sign s = Numeric::ratio_compare(v2.x, v2.w, v1.x, v1.w);
            if(s == POSITIVE) {
                return true;
            }
            if(s == NEGATIVE) {
                return false;
            }

            s = Numeric::ratio_compare(v2.y, v2.w, v1.y, v1.w);
            if(s == POSITIVE) {
                return true;
            }
            if(s == NEGATIVE) {
                return false;
            }
        
            s = Numeric::ratio_compare(v2.z, v2.w, v1.z, v1.w);
            return (s == POSITIVE);
        }
    };

    /************************************************************************/

    template <class T> inline vec2Hg<T> mix(
        const rationalg<T>& t,
        const vecng<2,double>& p1, const vecng<2,double>& p2
    ) {
        const T& st_d = t.denom();
        const T& t_n  = t.num();
        T s_n = st_d - t_n;
        return vec2Hg<T>(
            s_n * T(p1.x) + t_n * T(p2.x),
            s_n * T(p1.y) + t_n * T(p2.y),
            st_d
        );
    }
    
    template <class T> inline vec3Hg<T> mix(
        const rationalg<T>& t,
        const vecng<3,double>& p1, const vecng<3,double>& p2
    ) {
        const T& st_d = t.denom();
        const T& t_n  = t.num();
        T s_n = st_d - t_n;
        return vec3Hg<T>(
            s_n * T(p1.x) + t_n * T(p2.x),
            s_n * T(p1.y) + t_n * T(p2.y),
            s_n * T(p1.z) + t_n * T(p2.z),
            st_d
        );
    }


    template <class T> inline vec2Hg<T> mix(
        const rationalg<T>& t, const vec2Hg<T>& p1, const vec2Hg<T>& p2
    ) {
        if(p1.w == p2.w) {
            T sn = t.denom() - t.num();
            T tn = t.num();
            return vec2Hg<T>(
                sn * p1.x + tn * p2.x,
                sn * p1.y + tn * p2.y,
                t.denom() * p1.w
            );
        } else {
            T sn = p2.w*(t.denom() - t.num());
            T tn = p1.w*t.num();
            return vec2Hg<T>(
                sn * p1.x + tn * p2.x,
                sn * p1.y + tn * p2.y,
                t.denom() * p1.w * p2.w
            );
        }
    }

    template <class T> inline vec3Hg<T> mix(
        const rationalg<T>& t, const vec3Hg<T>& p1, const vec3Hg<T>& p2
    ) {
        if(p1.w == p2.w) {
            T sn = t.denom() - t.num();
            T tn = t.num();
            return vec3Hg<T>(
                sn * p1.x + tn * p2.x,
                sn * p1.y + tn * p2.y,
                sn * p1.z + tn * p2.z,
                t.denom() * p1.w
            );
        } else {
            T sn = p2.w*(t.denom() - t.num());
            T tn = p1.w*t.num();
            return vec3Hg<T>(
                sn * p1.x + tn * p2.x,
                sn * p1.y + tn * p2.y,
                sn * p1.z + tn * p2.z,
                t.denom() * p1.w * p2.w
            );
        }
    }


    /************************************************************************/

    namespace Numeric {
        
        template<class T> 
        inline void optimize_number_representation(vec2Hg<T>& v) {
            v.optimize();
        }

        template<class T>
        inline void optimize_number_representation(vec3Hg<T>& v) {
            v.optimize();
        }
        
    }

    /************************************************************************/
}


#endif

