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

#ifndef GEOGRAM_NUMERICS_EXPANSION_NT
#define GEOGRAM_NUMERICS_EXPANSION_NT

#include <geogram/basic/common.h>
#include <geogram/numerics/multi_precision.h>
#include <geogram/basic/matrix.h>
#include <geogram/basic/rationalg.h>

/**
 * \file geogram/numerics/expansion_nt.h
 * \brief High-level interface to multi-precision arithmetics
 * \details
 *  This file provides a "number-type" that encapsulates a (low-level)
 *  GEO::expansion object. 
 */

namespace GEO {

    class expansion_nt;
    
   /**
     * \brief Expansion_nt (expansion Number Type) is used to compute the
     *  sign of polynoms exactly.
     * \details Expansion_nt can be used like float and double. It supports
     *  three arithmetic operations (+,-,*), comparisons (>,>=,<,<=,==,!=)
     *  and exact sign computation. expansion_nt is a wrapper around 
     *  an \ref expansion allocated on the heap. When
     *  performance is a concern, the lower-level expansion class may be
     *  used instead.
     */
    class GEOGRAM_API expansion_nt {
    public:

        /**
         * \brief This type is used by the constructor that 
         *  takes two expansion%s.
         */
         enum Operation {
             SUM, DIFF, PRODUCT
         };
         
        /**
         * \brief Constructs an uninitialized expansion_nt.
         */
         expansion_nt() : rep_(nullptr) {
         }
         
        /**
         * \brief Constructs a new expansion_nt from a double.
         * \param[in] x the value to initialize this expansion.
         */
        explicit expansion_nt(double x) {
            rep_ = expansion::new_expansion_on_heap(1);
            rep()[0] = x;
            rep().set_length(1);
        }

        /**
         * \brief Constructs a new expansion_nt from an expansion
         * \details A new expansion is created on the heap and its
         *  content is initialized from \p rhs
         * \param[in] rhs the expansion to be copied
         */
        explicit expansion_nt(const expansion& rhs) {
            rep_ = expansion::new_expansion_on_heap(rhs.length());
            rep().assign(rhs);
        }

        /**
         * \brief Constructs a new expansion_nt from two expansions
         * \details A new expansion is created on the heap and its
         *  content is initialized from \p x \p op \p y. This function
         *  is used by code that combines the low-level API (expansion)
         *  with the high-level number type (expansion_nt). When returning
         *  the result of operations that combine expansion as an expansion_nt,
         *  it makes it possible to avoid copying the result of the 
         *  last operation by directly assigning it to an expansion_nt.
         * \param[in] x , y the two operands
         * \param[in] op one of 
         *  expansion_nt::SUM, expansion_nt::DIFF, expansion_nt::PRODUCT
         */
        explicit expansion_nt(
            Operation op, const expansion& x, const expansion& y
        ) {
            switch(op) {
            case SUM:
                rep_ = expansion::new_expansion_on_heap(
                    expansion::sum_capacity(x,y)
                );
                rep_->assign_sum(x,y);
                break;
            case DIFF:
                rep_ = expansion::new_expansion_on_heap(
                    expansion::diff_capacity(x,y)
                );
                rep_->assign_diff(x,y);
                break;
            case PRODUCT:
                rep_ = expansion::new_expansion_on_heap(
                    expansion::product_capacity(x,y)
                );
                rep_->assign_product(x,y);
                break;
            }
        }

        /**
         * \brief Constructs a new expansion_nt from three expansions
         * \details A new expansion is created on the heap and its
         *  content is initialized from \p x \p op \p y \p op \p z. 
         * This function is used by code that combines the low-level 
         * API (expansion) with the high-level number type (expansion_nt). 
         *  When returning the result of operations that combine expansion 
         *  as an expansion_nt, it makes it possible to avoid copying 
         *  the result of the last operation by directly assigning it 
         *  to an expansion_nt.
         * \param[in] x , y , z the three operands
         * \param[in] op one of expansion_nt::SUM, expansion_nt::PRODUCT
         */
        explicit expansion_nt(
            Operation op,
            const expansion& x, const expansion& y, const expansion& z
        ) {
            switch(op) {
            case SUM:
                rep_ = expansion::new_expansion_on_heap(
                    expansion::sum_capacity(x,y,z)
                );
                rep_->assign_sum(x,y,z);
                break;
            case DIFF:
                geo_assert_not_reached;
                break;
            case PRODUCT:
                rep_ = expansion::new_expansion_on_heap(
                    expansion::product_capacity(x,y,z)
                );
                rep_->assign_product(x,y,z);
                break;
            }
        }

        /**
         * \brief Constructs a new expansion_nt from four expansions
         * \details A new expansion is created on the heap and its
         *  content is initialized from \p x \p op \p y \p op \p z \p op \p t.
         *  This function is used by code that combines the 
         *  low-level API (expansion) with the high-level number type 
         *  (expansion_nt). When returning
         *  the result of operations that combine expansion as an expansion_nt,
         *  it makes it possible to avoid copying the result of the 
         *  last operation by directly assigning it to an expansion_nt.
         * \param[in] x , y , z , t the four operands
         * \param[in] op one of expansion_nt::SUM, expansion_nt::PRODUCT
         */
        explicit expansion_nt(
            Operation op,
            const expansion& x, const expansion& y,
            const expansion& z, const expansion& t
        ) {
            switch(op) {
            case SUM:
                rep_ = expansion::new_expansion_on_heap(
                    expansion::sum_capacity(x,y,z,t)
                );
                rep_->assign_sum(x,y,z,t);
                break;
            case DIFF:
                geo_assert_not_reached;
                break;
            case PRODUCT:
                // HERE: TODO CHECK SIZE
                const expansion& p1 = expansion_product(x,y);
                const expansion& p2 = expansion_product(z,t);
                rep_ = expansion::new_expansion_on_heap(
                    expansion::product_capacity(p1,p2)
                );
                rep_->assign_sum(p1,p2);
                break;
            }
        }
        
        /**
         * \brief Constructs a new expansion_nt from two doubles
         * \details A new expansion is created on the heap and its
         *  content is initialized from \p x \p op \p y. This function
         *  is used by code that combines the low-level API (expansion)
         *  with the high-level number type (expansion_nt). When returning
         *  the result of operations that combine expansion as an expansion_nt,
         *  it makes it possible to avoid copying the result of the 
         *  last operation by directly assigning it to an expansion_nt.
         * \param[in] x , y the two operands
         * \param[in] op one of 
         *  expansion_nt::SUM, expansion_nt::DIFF, expansion_nt::PRODUCT
         */
        explicit expansion_nt(Operation op, double x, double y) {
            switch(op) {
            case SUM:
                rep_ = expansion::new_expansion_on_heap(
                    expansion::sum_capacity(x,y)
                );
                rep_->assign_sum(x,y);
                break;
            case DIFF:
                rep_ = expansion::new_expansion_on_heap(
                    expansion::diff_capacity(x,y)
                );
                rep_->assign_diff(x,y);
                break;
            case PRODUCT:
                rep_ = expansion::new_expansion_on_heap(
                    expansion::product_capacity(x,y)
                );
                rep_->assign_product(x,y);
                break;
            }
        }
        
        /**
         * \brief Copy-constructor.
         * \param[in] rhs the expansion to be copied
         */
        expansion_nt(const expansion_nt& rhs) {
            copy(rhs);
        }

        /**
         * \brief Move-constructor.
         * \details Steals the expansion from \p rhs
         * \param[in] rhs the victim expansion_nt
         */
        expansion_nt(expansion_nt&& rhs) {
            rep_ = nullptr;
            std::swap(rep_, rhs.rep_);
        }
        
        /**
         * \brief Assignment operator.
         * \param[in] rhs the expansion to be copied
         * \return the new value of this expansion (rhs)
         */
        expansion_nt& operator= (const expansion_nt& rhs) {
            if(&rhs != this) {
                cleanup();
                copy(rhs);
            }
            return *this;
        }

        /**
         * \brief Assignment operator with move semantics
         * \param[in] rhs the expansion to be copied
         * \return the new value of this expansion (rhs)
         */
        expansion_nt& operator= (expansion_nt&& rhs) {
            if(&rhs != this) {
                cleanup();
                std::swap(rep_, rhs.rep_);
            }
            return *this;
        }
        
        /**
         * \brief Expansion_nt destructor.
         * \details The stored expansion is deallocated whenever
         *  reference counting reaches 0.
         */
        ~expansion_nt() {
            cleanup();
        }

        /**
         * \brief Optimizes the internal representation without changing the
         *  represented value
         * \details this function can reduce the length of an expansion
         */
        void optimize() {
            rep().optimize();
        }

        /**
         * \brief Flips the sign of an expansion.
         */
        void negate() {
            rep().negate();
        }
        
        /********************************************************************/

        /**
         * \brief Adds an expansion_nt to this expansion_nt
         * \param[in] rhs the expansion_nt to be added to this expansion_nt
         * \return the new value of this expansion_nt
         */
        expansion_nt& operator+= (const expansion_nt& rhs);

        /**
         * \brief Subtracts an expansion_nt to this expansion_nt
         * \param[in] rhs the expansion_nt to be subtracted
         * \return the new value of this expansion_nt
         */
        expansion_nt& operator-= (const expansion_nt& rhs);

        /**
         * \brief Multiplies this expansion_nt by an expansion_nt
         * \param[in] rhs the expansion_nt to multiply this expansion_nt by
         * \return the new value of this expansion_nt
         */
        expansion_nt& operator*= (const expansion_nt& rhs);

        /**
         * \brief Adds a double to this expansion_nt
         * \param[in] rhs the double to be added to this expansion_nt
         * \return the new value of this expansion_nt
         */
        expansion_nt& operator+= (double rhs);

        /**
         * \brief Subtracts a double from this expansion_nt
         * \param[in] rhs the double to be subtracted from this expansion_nt
         * \return the new value of this expansion_nt
         */
        expansion_nt& operator-= (double rhs);

        /**
         * \brief Multiplies this expansion_nt by a double
         * \details If the double is a constant (possibly negative) power of
         *  two (e.g. 0.125, 0.5, 2.0, 4.0 ...), one may use
         *  scale_fast() instead.
         * \param[in] rhs the double to multiply this expansion_nt with
         * \return the new value of this expansion_nt
         */
        expansion_nt& operator*= (double rhs);

        /********************************************************************/

        /**
         * \brief Computes the sum of two expansion_nt%s
         * \param[in] rhs the expansion_nt to be added to this expansion_nt
         * \return the sum of this expansion_nt and \p rhs
         */
        expansion_nt operator+ (const expansion_nt& rhs) const;

        /**
         * \brief Computes the difference between two expansion_nt%s
         * \param[in] rhs the expansion_nt to be subtracted from
         *  this expansion_nt
         * \return the difference between this expansion_nt and \p rhs
         */
        expansion_nt operator- (const expansion_nt& rhs) const;

        /**
         * \brief Computes the product between two expansion_nt%s
         * \param[in] rhs the expansion_nt to be multiplied by
         *  this expansion_nt
         * \return the product between this expansion_nt and \p rhs
         */
        expansion_nt operator* (const expansion_nt& rhs) const;

        /**
         * \brief Computes the sum of an expansion_nt and a double.
         * \param[in] rhs the double to be added to this expansion_nt
         * \return the sum of this expansion_nt and \p rhs
         */
        expansion_nt operator+ (double rhs) const;

        /**
         * \brief Computes the difference between an expansion_nt and a double.
         * \param[in] rhs the double to be subtracted from this expansion_nt
         * \return the difference between this expansion_nt and \p rhs
         */
        expansion_nt operator- (double rhs) const;

        /**
         * \brief Computes the product between an expansion_nt and a double.
         * \param[in] rhs the double to be multiplied by this expansion_nt
         * \return the product between this expansion_nt and \p rhs
         */
        expansion_nt operator* (double rhs) const;

        /********************************************************************/

        /**
         * \brief Computes the opposite of this expansion_nt.
         * \return the opposite of this expansion_nt
         */
        expansion_nt operator- () const;

        /********************************************************************/

        /**
         * \brief Compares two expansion_nt
         * \return the sign of this expansion minus rhs
         */
        Sign compare(const expansion_nt& rhs) const {
            return rep().compare(rhs.rep());
        }

        /**
         * \brief Compares an expansion_nt with a double
         * \return the sign of this expansion minus rhs
         */
        Sign compare(double rhs) const {
            return rep().compare(rhs);
        }
        
        /**
         * \brief Compares this expansion_nt with another one.
         * \details Internally computes the sign of the difference
         *  between this expansion_nt and \p rhs.
         * \return true if this expansion_nt is greater than \p rhs,
         *  false otherwise
         */
        bool operator> (const expansion_nt& rhs) const {
            return (int(compare(rhs))>0);
        }

        /**
         * \brief Compares this expansion_nt with another one.
         * \details Internally computes the sign of the difference
         *  between this expansion_nt and \p rhs.
         * \return true if this expansion_nt is greater or equal than \p rhs,
         *  false otherwise
         */
        bool operator>= (const expansion_nt& rhs) const {
            return (int(compare(rhs))>=0);
        }

        /**
         * \brief Compares this expansion_nt with another one.
         * \details Internally computes the sign of the difference
         *  between this expansion_nt and \p rhs.
         * \return true if this expansion_nt is smaller than \p rhs,
         *  false otherwise
         */
        bool operator< (const expansion_nt& rhs) const {
            return (int(compare(rhs))<0);
        }

        /**
         * \brief Compares this expansion_nt with another one.
         * \details Internally computes the sign of the difference
         *  between this expansion_nt and \p rhs.
         * \return true if this expansion_nt is smaller or equal than \p rhs,
         *  false otherwise
         */
        bool operator<= (const expansion_nt& rhs) const {
            return (int(compare(rhs))<=0);
        }

        /**
         * \brief Compares this expansion_nt with another one.
         * \details Internally computes the sign of the difference
         *  between this expansion_nt and \p rhs.
         * \return true if this expansion_nt is greater than \p rhs,
         *  false otherwise
         */
        bool operator> (double rhs) const {
            return (int(compare(rhs))>0);            
        }

        /**
         * \brief Compares this expansion_nt with another one.
         * \details Internally computes the sign of the difference
         *  between this expansion_nt and \p rhs.
         * \return true if this expansion_nt is greater or equal than \p rhs,
         *  false otherwise
         */
        bool operator>= (double rhs) const {
            return (int(compare(rhs))>=0);            
        }
        
        /**
         * \brief Compares this expansion_nt with another one.
         * \details Internally computes the sign of the difference
         *  between this expansion_nt and \p rhs.
         * \return true if this expansion_nt is smaller than \p rhs,
         *  false otherwise
         */
        bool operator< (double rhs) const {
            return (int(compare(rhs))<0);                        
        }

        /**
         * \brief Compares this expansion_nt with another one.
         * \details Internally computes the sign of the difference
         *  between this expansion_nt and \p rhs.
         * \return true if this expansion_nt is smaller or equal than \p rhs,
         *  false otherwise
         */
        bool operator<= (double rhs) const {
            return (int(compare(rhs))<=0);            
        }

        /********************************************************************/

        /**
         * \brief Computes an approximation of the stored
         *  value in this expansion.
         * \return an approximation of the stored value.
         */
        double estimate() const {
            return rep().estimate();
        }
        
        /**
         * \brief Gets the sign of this expansion_nt.
         * \return the sign of this expansion_nt, computed exactly.
         */
        Sign sign() const {
            return rep().sign();
        }

        /**
         * \brief Gets the length of this expansion.
         * \return the number of components used internally
         *  to represend this expansion
         * \note most client code will not need to use this
         *  (advanced use only).
         */
        index_t length() const {
            return rep().length();
        }

        /**
         * \brief Gets the i-th component of this expansion.
         * \param i index of the component
         * \return the i-th component of this expansion
         * \pre i < length()
         * \note most client code will not need to use this
         *  (advanced use only).
         */
        double component(index_t i) const {
            geo_debug_assert(i < length());
            return rep()[i];
        }
        
        /**
         * \brief Constructs a new expansion_nt from an expansion.
         * \details Used internally
         * \param[in] rep should be a reference-counted expansion, created
         *  by new_expansion_on_heap(). Its reference counter is incremented.
         * \note most client code will not need to use this
         *  (advanced use only).
         */
        expansion_nt(expansion* rep) :
            rep_(rep) {
        }

        /**
         * \brief Gets the internal expansion that represents this
         *  expansion_nt.
         * \return a reference to the expansion that represents
         *  this expansion_nt
         * \note most client code will not need to use this
         *  (advanced use only).
         */
        expansion& rep() {
            return *rep_;
        }

        /**
         * \brief Gets the internal expansion that represents
         *  this expansion_nt.
         * \return a const reference to the expansion that represents
         *  this expansion_nt
         * \note most client code will not need to use this
         *  (advanced use only).
         */
        const expansion& rep() const {
            return *rep_;
        }

        /**
         * \brief Gets a string representation of this expansion
         * \return a string with the length and components or "null"
         *  if this expansion_nt was explicitely set to uninitialized.
         */
        std::string to_string() const {
            return (rep_ == nullptr) ?
                std::string("null") :
                rep_->to_string()   ;
        }
        
    protected:

        /**
         * \brief Copies an expansion into this one.
         * \details current rep_ pointer is supposed to be
         *   uninitialized or freed before calling this function.
         * \param[in] rhs a const reference to the expansion to be copied
         */
        void copy(const expansion_nt& rhs) {
            if(rhs.rep_ == nullptr) {
                rep_ = nullptr;
            } else {
                rep_ = expansion::new_expansion_on_heap(rhs.rep().capacity());
                rep_->set_length(rhs.rep().length());
                for(index_t i=0; i<rep_->length(); ++i) {
                    (*rep_)[i] = rhs.rep()[i];
                }
            }
        }

        /**
         * \brief Cleanups the memory associated with this expansion_nt.
         */
        void cleanup() {
            if(rep_ != nullptr) {
                expansion::delete_expansion_on_heap(rep_);
                rep_ = nullptr;
            }
        }
        
    private:
        expansion* rep_;
        friend expansion_nt operator- (double a, const expansion_nt& b);

        friend expansion_nt expansion_nt_sq_dist(
            const double* a, const double* b, coord_index_t dim
        );

        friend expansion_nt expansion_nt_dot_at(
            const double* a, const double* b, const double* c,
            coord_index_t dim
        );
        // friend class rational_nt;
    };

    /**
     * \brief Computes the sum of a double and an expansion_nt
     * \param[in] a the double to be added
     * \param[in] b the expansion_nt to be added
     * \return an expansion_nt that represents \p a + \p b
     * \relates expansion_nt
     */
    inline expansion_nt operator+ (double a, const expansion_nt& b) {
        return b + a;
    }

    /**
     * \brief Computes the difference between a double and an expansion_nt
     * \param[in] a the double
     * \param[in] b the expansion_nt to be subtracted
     * \return an expansion_nt that represents \p a - \p b
     * \relates expansion_nt
     */
    inline expansion_nt operator- (double a, const expansion_nt& b) {
        expansion_nt result = b - a;
        result.rep().negate();
        return result;
    }

    /**
     * \brief Computes the product of a double and an expansion_nt
     * \param[in] a the double
     * \param[in] b the expansion_nt to be multiplied
     * \return an expansion_nt that represents \p a * \p b
     * \relates expansion_nt
     */
    inline expansion_nt operator* (double a, const expansion_nt& b) {
        return b * a;
    }

    /**
     * \brief Tests equality between two expansion_nt%s.
     * \details Implemented by testing whether the difference between
     * \p a and \p b is 0.
     * \return true if \p a and \p b represent exactly the same value, false
     *  otherwise
     * \relates expansion_nt
     */
    inline bool operator== (const expansion_nt& a, const expansion_nt& b) {
        return a.rep().equals(b.rep());
    }

    /**
     * \brief Tests equality between an expansion_nt and a double.
     * \details Implemented by testing whether the difference between
     * \p a and \p b is 0.
     * \return true if \p a and \p b represent exactly the same value, false
     *  otherwise
     * \relates expansion_nt
     */
    inline bool operator== (const expansion_nt& a, double b) {
        return a.rep().equals(b);
    }

    /**
     * \brief Tests equality between a double and an expansion_nt.
     * \details Implemented by testing whether the difference between
     * \p a and \p b is 0.
     * \return true if \p a and \p b represent exactly the same value, false
     *  otherwise
     * \relates expansion_nt
     */
    inline bool operator== (double a, const expansion_nt& b) {
        return b.rep().equals(a);
    }

    /**
     * \brief Tests whether two expansion_nt%s differ.
     * \details Implemented by testing whether the difference between
     * \p a and \p b is different from 0.
     * \return true if \p a and \p b do not represent the same exact value,
     *  false otherwise
     * \relates expansion_nt
     */
    inline bool operator!= (const expansion_nt& a, const expansion_nt& b) {
        return !a.rep().equals(b.rep());
    }

    /**
     * \brief Tests whether an expansion_nt differs from a double.
     * \details Implemented by testing whether the difference between
     * \p a and \p b is different from 0.
     * \return true if \p a and \p b do not represent the same exact value,
     *  false otherwise
     * \relates expansion_nt
     */
    inline bool operator!= (const expansion_nt& a, double b) {
        return !a.rep().equals(b);
    }

    /**
     * \brief Tests whether a double differs from an expansion_nt.
     * \details Implemented by testing whether the difference between
     * \p a and \p b is different from 0.
     * \return true if \p a and \p b do not represent the same exact value,
     *  false otherwise
     * \relates expansion_nt
     */
    inline bool operator!= (double a, const expansion_nt& b) {
        return !b.rep().equals(a);
    }

    /**
     * \brief Computes an expansion that represents the square distance between
     *  two points.
     * \param[in] a an array of \p dim doubles
     * \param[in] b an array of \p dim doubles
     * \param[in] dim the dimension of the points
     * \return an expansion_nt that represent the exact squared
     *  distance between \p a and \p b
     * \relates expansion_nt
     */
    inline expansion_nt expansion_nt_sq_dist(
        const double* a, const double* b, coord_index_t dim
    ) {
        expansion* result = expansion::new_expansion_on_heap(
            expansion::sq_dist_capacity(dim)
        );
        result->assign_sq_dist(a, b, dim);
        return expansion_nt(result);
    }

    /**
     * \brief Computes an expansion that represents the dot product of
     *  two vectors determined by three points.
     * \param[in] a an array of \p dim doubles
     * \param[in] b an array of \p dim doubles
     * \param[in] c an array of \p dim doubles
     * \param[in] dim the dimension of the points
     * \return an expansion_nt that represents the exact
     *  dot product (\p a - \p c).(\p b - \p c)
     * \relates expansion_nt
     */
    inline expansion_nt expansion_nt_dot_at(
        const double* a, const double* b, const double* c, coord_index_t dim
    ) {
        expansion* result = expansion::new_expansion_on_heap(
            expansion::dot_at_capacity(dim)
        );
        result->assign_dot_at(a, b, c, dim);
        return expansion_nt(result);
    }

    /************************************************************************/

    /**
     * \brief Specialization of geo_sgn() for expansion_nt.
     * \param x a const reference to an expansion_nt
     * \return the (exact) sign of x (one of POSITIVE, ZERO, NEGATIVE)
     */
    template <> inline Sign geo_sgn(const expansion_nt& x) {
        return x.sign();
    }

    /**
     * \brief Specialization of geo_cmp() for expansion_nt.
     * \param x , y const references to two expansion_nt
     * \retval POSITIVE if x > y
     * \retval ZERO if x == y
     * \retval NEGATIVE if x < y
     */
    template <> inline Sign geo_cmp(
        const expansion_nt& x, const expansion_nt& y
    ) {
        return x.compare(y);
    }
    
    /************************************************************************/

    /**
     * \brief Tests whether an expansion_nt is zero.
     * \details Optimized using the low-level API
     * \param[in] x a const reference to the expansion_nt to be tested
     * \retval true if \p x is equal to zero
     * \retval false otherwise
     */
    inline bool expansion_nt_is_zero(const expansion_nt& x) {
        return (x.sign() == GEO::ZERO);
    }

    /**
     * \brief Tests whether an expansion_nt is equal to one.
     * \details Optimized using the low-level API
     * \param[in] x a const reference to the expansion_nt to be tested
     * \retval true if \p x is equal to one
     * \retval false otherwise
     */
    inline bool expansion_nt_is_one(const expansion_nt& x) {
        return x.rep().equals(1.0);
    }


    /**
     * \brief Compares two expansion_nt
     * \details Optimized using the low-level API
     * \param [in] x , y the two expansion_nt to compare
     * \retval POSITIVE if \p x is greater than \p y
     * \retval ZERO if \p x equals \p y
     * \retval NEGATIVE if \p x is smaller than \p y
     */
    inline Sign expansion_nt_compare(
        const expansion_nt& x, const expansion_nt& y
    ) {
        const expansion& diff = expansion_diff(x.rep(), y.rep());
        return diff.sign();
    }

    /**
     * \brief Computes the square of an expansion_nt
     * \details Optimized using the low-level API
     * \param[in] x the expansion_nt to be squared
     * \return \p x * \p x
     */
    inline expansion_nt expansion_nt_square(const expansion_nt& x) {
        expansion_nt result(
            expansion::new_expansion_on_heap(
                expansion::square_capacity(x.rep()
             ))
        );
        result.rep().assign_square(x.rep());
        return result;
    }


    /**
     * \brief Computes a 2x2 determinant
     * \details Specialization using the low-evel API for expansions. 
     *  This gains some performance as compared to using CGAL's 
     *  determinant template with expansion_nt.
     */
    expansion_nt GEOGRAM_API expansion_nt_determinant(
        const expansion_nt& a00,const expansion_nt& a01,  
        const expansion_nt& a10,const expansion_nt& a11
    );
    
    /**
     * \brief Computes a 3x3 determinant
     * \details Specialization using the low-evel API for expansions. 
     *  This gains some performance as compared to using CGAL's determinant 
     *  template with expansion_nt.
     */
    expansion_nt GEOGRAM_API expansion_nt_determinant(
        const expansion_nt& a00,const expansion_nt& a01,const expansion_nt& a02,
        const expansion_nt& a10,const expansion_nt& a11,const expansion_nt& a12,
        const expansion_nt& a20,const expansion_nt& a21,const expansion_nt& a22
    );

    /**
     * \brief Computes a 4x4 determinant
     * \details Specialization using the low-evel API for expansions. 
     *  This gains some performance as compared to using CGAL's determinant 
     *  template with expansion_nt.
     */
    expansion_nt GEOGRAM_API expansion_nt_determinant(
        const expansion_nt& a00,const expansion_nt& a01,
        const expansion_nt& a02,const expansion_nt& a03,
        const expansion_nt& a10,const expansion_nt& a11,
        const expansion_nt& a12,const expansion_nt& a13,
        const expansion_nt& a20,const expansion_nt& a21,
        const expansion_nt& a22,const expansion_nt& a23,
        const expansion_nt& a30,const expansion_nt& a31,
        const expansion_nt& a32,const expansion_nt& a33 
    );

// Make things a bit faster if target OS has large stack size
#ifdef GEO_HAS_BIG_STACK
    
    /**
     * \brief Specialization of det2x2 
     * \details Calls the optimized implementation for expansion_nt
     */
    
    template <> inline expansion_nt det2x2(
        const expansion_nt& a11, const expansion_nt& a12,                    
        const expansion_nt& a21, const expansion_nt& a22
    ) {
        return expansion_nt_determinant(
            a11,a12,
            a21,a22
        );
    }

    /**
     * \brief Specialization of det3x3 
     * \details Calls the optimized implementation for expansion_nt
     */
    
    template <> inline expansion_nt det3x3(
        const expansion_nt& a11, const expansion_nt& a12,
        const expansion_nt& a13,                
        const expansion_nt& a21, const expansion_nt& a22,
        const expansion_nt& a23,                
        const expansion_nt& a31, const expansion_nt& a32,
        const expansion_nt& a33
    ) {
        return expansion_nt_determinant(
            a11,a12,a13,
            a21,a22,a23,
            a31,a32,a33
        );
    }

    /**
     * \brief Specialization of det4x4 
     * \details Calls the optimized implementation for expansion_nt
     */
    
    template <> inline expansion_nt det4x4(
        const expansion_nt& a11, const expansion_nt& a12,
        const expansion_nt& a13, const expansion_nt& a14,
        const expansion_nt& a21, const expansion_nt& a22,
        const expansion_nt& a23, const expansion_nt& a24,               
        const expansion_nt& a31, const expansion_nt& a32,
        const expansion_nt& a33, const expansion_nt& a34,  
        const expansion_nt& a41, const expansion_nt& a42,
        const expansion_nt& a43, const expansion_nt& a44  
    ) {
        return expansion_nt_determinant(
            a11,a12,a13,a14,
            a21,a22,a23,a24,
            a31,a32,a33,a34,
            a41,a42,a43,a44            
        );
    }
    
#endif
    
    /************************************************************************/
}

/**
 * \brief Displays the approximated value of an expansion_nt to a stream.
 * \param[out] os the stream 
 * \param[in] a the expansion_nt to be sent to the stream
 * \return a reference to the stream
 */
inline std::ostream& operator<< (
    std::ostream& os, const GEO::expansion_nt& a
) {
    return os << a.estimate();
}

/**
 * \brief Reads a double precision number from a stream and converts it to
 *  an approximation.
 * \param[in] is the stream 
 * \param[out] a the read expansion_nt
 * \return a reference to the stream
 */
inline std::istream& operator>> ( std::istream& is, GEO::expansion_nt& a) {
    double d;
    is >> d;
    if (is) {
        a = GEO::expansion_nt(d);
    }
    return is;
}

/*****************************************************************************/

namespace GEO {

    /**************************************************************************/

    namespace Numeric {
        
        template<> inline void optimize_number_representation(expansion_nt& x) {
            x.optimize();
        }

        /**
         * \brief Compares two rational numbers given as separate
         *   numerators and denominators. Specialization for exact_nt.
         * \param[in] a_num , a_denom defines a = \p a_num / \p a_denom
         * \param[in] b_num , b_denom defines b = \p b_num / \p b_denom
         * \return the sign of a - b
         */
        template<> Sign GEOGRAM_API ratio_compare(
            const expansion_nt& a_num, const expansion_nt& a_denom,
            const expansion_nt& b_num, const expansion_nt& b_denom
        );
    }
    
    /**************************************************************************/

    typedef rationalg<expansion_nt> rational_nt;
}

#endif
