/*
 *  Copyright (c) 2000-2023 Inria
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

#ifndef GEOGRAM_BASIC_RATIONALG
#define GEOGRAM_BASIC_RATIONALG

#include <geogram/basic/common.h>

/**
 * \file geogram/basic/rationalg.h
 * \brief Generic implementation of rational type
 */

namespace GEO {

    /**
     * \brief rationalg (generic rational) is used to compute the
     *  sign of rational fractions exactly.
     * \details rationalg can be used like float and double. It supports
     *  four arithmetic operations (+,-,*,/), comparisons (>,>=,<,<=,==,!=)
     *  and exact sign computation. 
     */
    template <class T> class rationalg {
      public:
        typedef T value_type;

        rationalg() = default;
        
        /**
         * \brief Constructs a new rationalg from a double.
         * \param[in] x the value to initialize this rationalg.
         */
        explicit rationalg(double x) : num_(x), denom_(1.0) {
        }

        /**
         * \brief Constructs a new rationalg from an T.
         * \param[in] x the value to initialize this rationalg.
         */
        explicit rationalg(const T& x) : num_(x), denom_(1.0) {
        }

        /**
         * \brief Constructs a new rationalg from an T
         *  with move semantics
         * \param[in] x the victim T
         */
        explicit rationalg(T&& x) : num_(x), denom_(1.0) {
        }
        
        /**
         * \brief Constructs a new rationalg from two doubles.
         * \param[in] num the numerator
	 * \param[in] denom the denominator
         */
        explicit rationalg(double num, double denom)
	    : num_(num), denom_(denom) {
        }
        
        /**
         * \brief Constructs a new rationalg from two T.
         * \param[in] num the numerator
	 * \param[in] denom the denominator
         */
        explicit rationalg(const T& num, const T& denom)
	    : num_(num), denom_(denom) {
        }

        /**
         * \brief Constructs a new rationalg from two T with
         *  move semantics
         * \param[in] num the numerator
	 * \param[in] denom the denominator
         */
        explicit rationalg(
            T&& num, T&& denom
        ) : num_(num), denom_(denom) {
        }
            
        /**
         * \brief Copy-constructor.
         * \param[in] rhs the rational to be copied
         */
        rationalg(const rationalg<T>& rhs) = default;
        
        /**
         * \brief Move-constructor.
         * \param[in] rhs the rational to be copied
         */
        rationalg(rationalg<T>&& rhs) = default;
        
        /**
         * \brief Assignment operator.
         * \param[in] rhs the rational to be copied
         * \return the new value of this rational (rhs)
         */
        rationalg<T>& operator= (const rationalg<T>& rhs) = default;

        /**
         * \brief Assignment operator with move semantics
         * \param[in] rhs the victim rationalg
         * \return the new value of this rational (rhs)
         */
        rationalg<T>& operator= (rationalg<T>&& rhs) = default;
        
	/**
	 * \brief gets the numerator.
	 * \return a const reference to the numerator.
	 */
	const T& num() const {
	    return num_;
	}

	/**
	 * \brief gets the denominator.
	 * \return a const reference to the denominator.
	 */
	const T& denom() const {
	    return denom_;
	}

	/**
	 * \brief gets the numerator.
	 * \return a reference to the numerator.
	 */
	 T& num() {
	    return num_;
	}

	/**
	 * \brief gets the denominator.
	 * \return a reference to the denominator.
	 */
	 T& denom() {
	    return denom_;
	}

        /**
         * \brief Optimizes the internal representation without changing the
         *  represented value
         */
        void optimize() {
            Numeric::optimize_number_representation(num_);
            Numeric::optimize_number_representation(denom_);
        }
         
        /********************************************************************/

        /**
         * \brief Adds a rationalg to this rationalg
         * \param[in] rhs the rationalg to be added to this rationalg
         * \return the new value of this rationalg
         */
        rationalg<T>& operator+= (const rationalg<T>& rhs) {
	    if(has_same_denom(rhs)) {
		num_ += rhs.num_;
	    } else {
		num_ = num_ * rhs.denom_ + rhs.num_ * denom_;	    
		denom_ *= rhs.denom_;
	    }
	    return *this;
	}

        /**
         * \brief Subtracts a rationalg to this rationalg
         * \param[in] rhs the rationalg to be subtracted
         * \return the new value of this rationalg
         */
        rationalg<T>& operator-= (const rationalg<T>& rhs) {
	    if(has_same_denom(rhs)) {
		num_ -= rhs.num_;
	    } else {
		num_ = num_ * rhs.denom_ - rhs.num_ * denom_;	    
		denom_ *= rhs.denom_;
	    }
	    return *this;
	}

        /**
         * \brief Multiplies this rationalg by a rationalg
         * \param[in] rhs the rationalg to multiply this rationalg by
         * \return the new value of this rationalg
         */
        rationalg<T>& operator*= (const rationalg<T>& rhs) {
	    num_ *= rhs.num_;
	    denom_ *= rhs.denom_;
	    return *this;
	}

        /**
         * \brief Divides this rationalg by a rationalg
         * \param[in] rhs the rationalg to divide this rationalg by
         * \return the new value of this rationalg
         */
        rationalg<T>& operator/= (const rationalg<T>& rhs) {
	    num_ *= rhs.denom_;
	    denom_ *= rhs.num_;
	    return *this;
	}
	
        /**
         * \brief Adds a double to this rationalg
         * \param[in] rhs the double to be added to this rationalg
         * \return the new value of this rationalg
         */
        rationalg<T>& operator+= (double rhs) {
	    num_ += denom_ * T(rhs);
	    return *this;
	}

        /**
         * \brief Subtracts a double from this rationalg
         * \param[in] rhs the double to be subtracted from this rationalg
         * \return the new value of this rationalg
         */
        rationalg<T>& operator-= (double rhs) {
	    num_ -= denom_ * T(rhs);
	    return *this;
	}

        /**
         * \brief Multiplies this rationalg by a double
         * \details If the double is a constant (possibly negative) power of
         *  two (e.g. 0.125, 0.5, 2.0, 4.0 ...), one may use
         *  num().scale_fast() / denom().scale_fast() instead.
         * \param[in] rhs the double to multiply this rationalg with
         * \return the new value of this rationalg
         */
        rationalg<T>& operator*= (double rhs) {
	    num_ *= T(rhs);
	    return *this;
	}

        /**
         * \brief Divides this rationalg by a double
         * \details If the double is a constant (possibly negative) power of
         *  two (e.g. 0.125, 0.5, 2.0, 4.0 ...), one may use
         *  num().scale_fast() / denom().scale_fast() instead.
         * \param[in] rhs the double to multiply this rationalg with
         * \return the new value of this rationalg
         */
        rationalg<T>& operator/= (double rhs) {
	    denom_ *= T(rhs);
	    return *this;
	}
	
        /********************************************************************/

        /**
         * \brief Computes the sum of two rationalg%s
         * \param[in] rhs the rationalg to be added to this rationalg
         * \return the sum of this rationalg and \p rhs
         */
        rationalg<T> operator+ (const rationalg<T>& rhs) const {
	    if(has_same_denom(rhs)) {
		return rationalg(
		    num_ + rhs.num_,
		    denom_
		);
	    }
	    return rationalg(
		num_ * rhs.denom_ + rhs.num_ * denom_,
		denom_ * rhs.denom_
	    );
	}

        /**
         * \brief Computes the difference between two rationalg%s
         * \param[in] rhs the rationalg to be subtracted from
         *  this rationalg
         * \return the difference between this rationalg and \p rhs
         */
        rationalg<T> operator- (const rationalg<T>& rhs) const {
	    if(has_same_denom(rhs)) {
		return rationalg(
		    num_ - rhs.num_,
		    denom_
		);
	    }
	    return rationalg(
		num_ * rhs.denom_ - rhs.num_ * denom_,
		denom_ * rhs.denom_
	    );
	}

        /**
         * \brief Computes the product between two rationalg%s
         * \param[in] rhs the rationalg to be multiplied by
         *  this rationalg
         * \return the product between this rationalg and \p rhs
         */
        rationalg<T> operator* (const rationalg<T>& rhs) const {
	    return rationalg(
		num_ * rhs.num_,
		denom_ * rhs.denom_
	    );
	}

        /**
         * \brief Computes the ratio between two rationalg%s
         * \param[in] rhs the rationalg to be multiplied by
         *  this rationalg
         * \return the ratio between this rationalg and \p rhs
         */
        rationalg<T> operator/ (const rationalg<T>& rhs) const {
	    return rationalg(
		num_ * rhs.denom_,
		denom_ * rhs.num_
	    );
	}

	
        /**
         * \brief Computes the sum of a rationalg and a double.
         * \param[in] rhs the double to be added to this rationalg
         * \return the sum of this rationalg and \p rhs
         */
        rationalg<T> operator+ (double rhs) const {
	    return rationalg(
		num_ + T(rhs) * denom_,
		denom_
	    );
	}

        /**
         * \brief Computes the difference between a rationalg and a double.
         * \param[in] rhs the double to be subtracted from this rationalg
         * \return the difference between this rationalg and \p rhs
         */
        rationalg<T> operator- (double rhs) const {
	    return rationalg(
		num_ - T(rhs) * denom_,
		denom_
	    );
	}

        /**
         * \brief Computes the product between a rationalg and a double.
         * \param[in] rhs the double to be multiplied by this rationalg
         * \return the product between this rationalg and \p rhs
         */
        rationalg<T> operator* (double rhs) const {
	    return rationalg(
		num_ * T(rhs),
		denom_
	    );
	}

        /**
         * \brief Computes the ratio between a rationalg and a double.
         * \param[in] rhs the double to be multiplied by this rationalg
         * \return the ratio between this rationalg and \p rhs
         */
        rationalg<T> operator/ (double rhs) const {
	    return rationalg(
		num_,
		denom_* T(rhs)
	    );
	}
	
        /********************************************************************/

        /**
         * \brief Computes the opposite of this rationalg.
         * \return the opposite of this rationalg
         */
        rationalg<T> operator- () const {
	    return rationalg(
		-num_, 
		denom_
	    );
	}

        /********************************************************************/

        /**
         * \brief Compares two rationalg
         * \return the sign of this expansion minus rhs
         */
        Sign compare(const rationalg<T>& rhs) const {
            if(has_same_denom(rhs)) {
                return Sign(num_.compare(rhs.num_) * denom_.sign());
            }
            return Sign(
                (num_ * rhs.denom_).compare(rhs.num_ * denom_) *
                denom_.sign() * rhs.denom_.sign()
            );
        }

        /**
         * \brief Compares a rationalg with a double
         * \return the sign of this expansion minus rhs
         */
        Sign compare(double rhs) const {
            return Sign(
                num_.compare(T(rhs)*denom_) * denom_.sign()
            );
        }
        
        /**
         * \brief Compares this rationalg with another one.
         * \details Internally computes the sign of the difference
         *  between this rationalg and \p rhs.
         * \return true if this rationalg is greater than \p rhs,
         *  false otherwise
         */
        bool operator> (const rationalg<T>& rhs) const {
            return (int(compare(rhs))>0);
        }

        /**
         * \brief Compares this rationalg with another one.
         * \details Internally computes the sign of the difference
         *  between this rationalg and \p rhs.
         * \return true if this rationalg is greater or equal than \p rhs,
         *  false otherwise
         */
        bool operator>= (const rationalg<T>& rhs) const {
            return (int(compare(rhs))>=0);            
        }

        /**
         * \brief Compares this rationalg with another one.
         * \details Internally computes the sign of the difference
         *  between this rationalg and \p rhs.
         * \return true if this rationalg is smaller than \p rhs,
         *  false otherwise
         */
        bool operator< (const rationalg<T>& rhs) const {
            return (int(compare(rhs))<0);
        }

        /**
         * \brief Compares this rationalg with another one.
         * \details Internally computes the sign of the difference
         *  between this rationalg and \p rhs.
         * \return true if this rationalg is smaller or equal than \p rhs,
         *  false otherwise
         */
        bool operator<= (const rationalg<T>& rhs) const {
            return (int(compare(rhs))<=0);
        }

        /**
         * \brief Compares this rationalg with another one.
         * \details Internally computes the sign of the difference
         *  between this rationalg and \p rhs.
         * \return true if this rationalg is greater than \p rhs,
         *  false otherwise
         */
        bool operator> (double rhs) const {
            return (int(compare(rhs))>0);            
        }

        /**
         * \brief Compares this rationalg with another one.
         * \details Internally computes the sign of the difference
         *  between this rationalg and \p rhs.
         * \return true if this rationalg is greater or equal than \p rhs,
         *  false otherwise
         */
        bool operator>= (double rhs) const {
            return (int(compare(rhs))>=0);            
        }

        /**
         * \brief Compares this rationalg with another one.
         * \details Internally computes the sign of the difference
         *  between this rationalg and \p rhs.
         * \return true if this rationalg is smaller than \p rhs,
         *  false otherwise
         */
        bool operator< (double rhs) const {
            return (int(compare(rhs))<0);            
        }

        /**
         * \brief Compares this rationalg with another one.
         * \details Internally computes the sign of the difference
         *  between this rationalg and \p rhs.
         * \return true if this rationalg is smaller or equal than \p rhs,
         *  false otherwise
         */
        bool operator<= (double rhs) const {
            return (int(compare(rhs))<=0);                        
        }

        /********************************************************************/

        /**
         * \brief Computes an approximation of the stored
         *  value in this rational.
         * \return an approximation of the stored value.
         */
        double estimate() const {
            return num_.estimate() / denom_.estimate();
        }
        
        /**
         * \brief Gets the sign of this rationalg.
         * \return the sign of this rationalg, computed exactly.
         */
        Sign sign() const {
            geo_debug_assert(denom_.sign() != ZERO);
            return Sign(num_.sign() * denom_.sign());
        }

      protected:
        /**
         * \brief Copies a rational into this one.
         * \param[in] rhs a const reference to the rational to be copied
         */
	void copy(const rationalg<T>& rhs) {
	    num_ = rhs.num_;
	    denom_ = rhs.denom_;
	}

	/**
	 * \brief Tests whether a rationalg has trivially the 
         *  same denominator this rationalg.
	 * \details This function is used to implement faster addition, 
	 *  subtraction and tests when it can be quickly determined that both
	 *  operands have the same denominator.
	 * \retval true if it is trivial that \p rhs has the same denominator
	 *  as this rationalg.
	 * \retval false otherwise.
	 */
	bool has_same_denom(const rationalg<T>& rhs) const {
            return denom_ == rhs.denom_;
	}
	
      private:
	T num_;
	T denom_;
    };

    /**************************************************************************/

    /**
     * \brief Computes the sum of a double and a rationalg
     * \param[in] a the double to be added
     * \param[in] b the rationalg to be added
     * \return a rationalg that represents \p a + \p b
     * \relates rationalg
     */
    template <class T>
    inline rationalg<T> operator+ (double a, const rationalg<T>& b) {
        return b + a;
    }

    /**
     * \brief Computes the difference between a double and a rationalg
     * \param[in] a the double
     * \param[in] b the rationalg to be subtracted
     * \return a rationalg that represents \p a - \p b
     * \relates rationalg
     */
    template <class T>    
    inline rationalg<T> operator- (double a, const rationalg<T>& b) {
        rationalg<T> result = b - a;
        result.num().negate();
        return result;
    }

    /**
     * \brief Computes the product of a double and a rationalg
     * \param[in] a the double
     * \param[in] b the rationalg to be multiplied
     * \return a rationalg that represents \p a * \p b
     * \relates rationalg
     */
    template <class T>    
    inline rationalg<T> operator* (double a, const rationalg<T>& b) {
        return b * a;
    }

    /**
     * \brief Computes the ratio between a double and a rationalg
     * \param[in] a the double
     * \param[in] b the rationalg to be divided
     * \return a rationalg that represents \p a / \p b
     * \relates rationalg
     */
    template <class T>    
    inline rationalg<T> operator/ (double a, const rationalg<T>& b) {
        return rationalg<T>(
	    T(a)*b.denom(),
	    b.num()
	);
    }
    
    /**
     * \brief Tests equality between two rationalg%s.
     * \details Implemented by testing whether the difference between
     * \p a and \p b is 0.
     * \return true if \p a and \p b represent exactly the same value, false
     *  otherwise
     * \relates rationalg
     */
    template <class T>    
    inline bool operator== (const rationalg<T>& a, const rationalg<T>& b) {
        return (a.compare(b) == ZERO);
    }

    /**
     * \brief Tests equality between a rationalg and a double.
     * \details Implemented by testing whether the difference between
     * \p a and \p b is 0.
     * \return true if \p a and \p b represent exactly the same value, false
     *  otherwise
     * \relates rationalg
     */
    template <class T>    
    inline bool operator== (const rationalg<T>& a, double b) {
        return (a.compare(b) == ZERO);
    }

    /**
     * \brief Tests equality between a double and a rationalg.
     * \details Implemented by testing whether the difference between
     * \p a and \p b is 0.
     * \return true if \p a and \p b represent exactly the same value, false
     *  otherwise
     * \relates rationalg
     */
    template <class T>    
    inline bool operator== (double a, const rationalg<T>& b) {
        return (b.compare(a) == ZERO);
    }

    /**
     * \brief Tests whether two rationalg%s differ.
     * \details Implemented by testing whether the difference between
     * \p a and \p b is different from 0.
     * \return true if \p a and \p b do not represent the same exact value,
     *  false otherwise
     * \relates rationalg
     */
    template <class T>    
    inline bool operator!= (const rationalg<T>& a, const rationalg<T>& b) {
        return (a.compare(b) != ZERO);
    }

    /**
     * \brief Tests whether a rationalg differs from a double.
     * \details Implemented by testing whether the difference between
     * \p a and \p b is different from 0.
     * \return true if \p a and \p b do not represent the same exact value,
     *  false otherwise
     * \relates rationalg
     */
    template <class T>    
    inline bool operator!= (const rationalg<T>& a, double b) {
        return (a.compare(b) != ZERO);
    }

    /**
     * \brief Tests whether a double differs from a rationalg.
     * \details Implemented by testing whether the difference between
     * \p a and \p b is different from 0.
     * \return true if \p a and \p b do not represent the same exact value,
     *  false otherwise
     * \relates rationalg
     */
    template <class T>    
    inline bool operator!= (double a, const rationalg<T>& b) {
        return (b.compare(a) != ZERO);
    }

    /**************************************************************************/

    /**
     * \brief Specialization of geo_sgn() for rationalg
     * \param x a const reference to a rationalg
     * \return the (exact) sign of x (one of POSITIVE, ZERO, NEGATIVE)
     */
    template <class T> inline Sign geo_sgn(const rationalg<T>& x) {
        return x.sign();
    }

    /**
     * \brief Specialization of geo_cmp() for rationalg
     * \param a , b const references to two rationalg
     * \retval POSITIVE if a > b
     * \retval ZERO if a == b
     * \retval NEGATIVE if a < b
     */
    template <class T> inline Sign geo_cmp(
        const rationalg<T>& a, const rationalg<T>& b
    ) {
        return a.compare(b);
    }
    
    namespace Numeric {

        template <class T> inline void optimize_number_representation(
            rationalg<T>& x
        ) {
            x.optimize();
        }
        
    }

    /**************************************************************************/
    
}

#endif

