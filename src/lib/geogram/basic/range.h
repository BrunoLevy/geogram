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

#ifndef GEOGRAM_BASIC_RANGE
#define GEOGRAM_BASIC_RANGE

#include <geogram/basic/numeric.h>
#include <geogram/basic/memory.h>
#include <geogram/basic/assert.h>
#include <geogram/basic/argused.h>

/**
 * \file geogram/basic/range.h
 * \brief C++-20 like helpers for manipulating ranges of integers.
 * \note transform() not implemented yet, I need c++14 for that
 *  (auto return type).
 */

namespace GEO {

    /**
     * \brief Wraps an integer to be used with the range-based for construct.
     * \details Not really an iterator, rather a pseudo-index.
     *   Geogram mostly uses index-based data structures,
     *   where an element is an index in an array (whereas the
     *   STL uses pointer-like semantics). With indices, there
     *   is no indirection (an index is an element), so operator*
     *   does nothing (returns the stored index).
     */
    class index_as_iterator {
    public:
        index_as_iterator(index_t val) : val_(val) {
        }

        void operator++() {
            ++val_;
        }

        bool operator==(index_as_iterator rhs) const {
            return val_ == rhs.val_;
        }

        bool operator!=(index_as_iterator rhs) const {
            return val_ != rhs.val_;
        }

        bool operator<(index_as_iterator rhs) const {
            return val_ < rhs.val_;
        }

        index_t operator*() const {
            return val_;
        }

        index_t operator-(index_as_iterator it) const {
            return val_ - it.val_;
        }

        index_as_iterator operator+(index_t i) const {
            return index_as_iterator(val_ + i);
        }

    private:
        index_t val_;
    };

    /**
     * \brief A generic index_range bounded by two "non-iterators".
     */
    class index_range {
    public:

        typedef index_as_iterator iterator;
        typedef index_as_iterator const_iterator;

        /**
         * \brief index_range constructor
         * \param[in] b first index
         * \param[in] e one position past the last index
         */
        index_range(
	    index_as_iterator b, index_as_iterator e
	) : begin_(b), end_(e) {
        }

        /**
         * \brief gets the first index
         * \return a index_as_iterator corresponding to the first index
         */
        index_as_iterator begin() const {
            return begin_;
        }

        /**
         * \brief gets one position past the last index
         * \return a index_as_iterator corresponding to
         *  one position past the last index
         */
        index_as_iterator end() const {
            return end_;
        }

        /**
         * \brief gets the number of elements in the index_range
         */
        index_t nb() const {
            return end_ - begin_;
        }

        /**
         * \brief direct access to an arbitrary element in the index_range
         * \param[in] i the index of the element
         * \return an index_as_iterator corresponding to the \p i th element
         */
        index_t operator[](index_t i) const {
            geo_debug_assert(i < nb());
            return *(begin_ + i);
        }

    private:
        index_as_iterator begin_;
        index_as_iterator end_;
    };

    /*************************************************************************/

    /**
     * \brief Encapsulates a const pointer to an element in an index_t array
     * \details In debug mode, checks bounds on indirection
     */
    class const_index_ptr_in_array {
    public:
        const_index_ptr_in_array(
            const index_t* ptr, const index_t* begin, const index_t* end
        ) : ptr_(ptr)
#ifdef GEO_DEBUG
          ,begin_(begin),
            end_(end)
#endif
            {
                geo_argused(begin);
                geo_argused(end);
            }

        void operator++() {
            ++ptr_;
        }

        bool operator==(const_index_ptr_in_array rhs) const {
            return ptr_ == rhs.ptr_;
        }

        bool operator!=(const_index_ptr_in_array rhs) const {
            return ptr_ != rhs.ptr_;
        }

        bool operator<(const_index_ptr_in_array rhs) const {
            return ptr_ < rhs.ptr_;
        }

        const index_t& operator*() const  {
            geo_debug_assert(ptr_ >= begin_ && ptr_ < end_);
            return *ptr_;
        }

        index_t operator-(const_index_ptr_in_array it) const {
            return index_t(ptr_ - it.ptr_);
        }

        const_index_ptr_in_array operator+(index_t i) const {
#ifdef GEO_DEBUG
            return const_index_ptr_in_array(ptr_ + i, begin_, end_);
#else
            return const_index_ptr_in_array(ptr_ + i, nullptr, nullptr);
#endif
        }

    private:
        const index_t* ptr_;
#ifdef GEO_DEBUG
        const index_t* begin_;
        const index_t* end_;
#endif
    };

    /*************************************************************************/

    /**
     * \brief Encapsulates a pointer to an element in an index_t array
     * \details In debug mode, checks bounds on indirection
     */
    class index_ptr_in_array {
    public:
        index_ptr_in_array(
            index_t* ptr, index_t* begin, index_t* end
        ) : ptr_(ptr)
#ifdef GEO_DEBUG
          ,begin_(begin),
            end_(end)
#endif
            {
                geo_argused(begin);
                geo_argused(end);
            }

        void operator++() {
            ++ptr_;
        }

        bool operator==(index_ptr_in_array rhs) const {
            return ptr_ == rhs.ptr_;
        }

        bool operator!=(index_ptr_in_array rhs) const {
            return ptr_ != rhs.ptr_;
        }

        bool operator<(index_ptr_in_array rhs) const {
            return ptr_ < rhs.ptr_;
        }

        const index_t& operator*() const  {
            geo_debug_assert(ptr_ >= begin_ && ptr_ < end_);
            return *ptr_;
        }

        index_t& operator*() {
            geo_debug_assert(ptr_ >= begin_ && ptr_ < end_);
            return *ptr_;
        }

        index_t operator-(index_ptr_in_array it) const {
            return index_t(ptr_ - it.ptr_);
        }

        index_ptr_in_array operator+(index_t i) const {
#ifdef GEO_DEBUG
            return index_ptr_in_array(ptr_ + i, begin_, end_);
#else
            return index_ptr_in_array(ptr_ + i, nullptr, nullptr);
#endif
        }

        operator const_index_ptr_in_array() const {
#ifdef GEO_DEBUG
            return const_index_ptr_in_array(ptr_, begin_, end_);
#else
            return const_index_ptr_in_array(ptr_, nullptr, nullptr);
#endif
        }

    private:
        index_t* ptr_;
#ifdef GEO_DEBUG
        index_t* begin_;
        index_t* end_;
#endif
    };

    /*************************************************************************/

    class index_ptr_range {
    public:
        typedef index_ptr_in_array iterator;
        typedef const_index_ptr_in_array const_iterator;

        index_ptr_range(index_t* begin, index_t* end) :
            begin_(begin, begin, end),
            end_(end,begin,end) {
        }

        index_ptr_range(vector<index_t>& V, index_t b, index_t e) :
            begin_(V.data()+b, V.data() + b, V.data() + e),
            end_(V.data()+e, V.data() + b, V.data() + e) {
        }

        iterator begin() {
            return begin_;
        }

        iterator end() {
            return end_;
        }

        const_iterator begin() const {
            return begin_;
        }

        const_iterator end() const {
            return end_;
        }

    private:
        iterator begin_;
        iterator end_;
    };

    /*************************************************************************/

    class const_index_ptr_range {
    public:
        typedef const_index_ptr_in_array iterator;
        typedef const_index_ptr_in_array const_iterator;

        const_index_ptr_range(const index_t* begin, const index_t* end) :
            begin_(begin, begin, end),
            end_(end,begin,end) {
        }

        const_index_ptr_range(const vector<index_t>& V, index_t b, index_t e) :
            begin_(V.data()+b, V.data() + b, V.data() + e),
            end_(V.data()+e, V.data() + b, V.data() + e) {
        }

        const_iterator begin() const {
            return begin_;
        }

        const_iterator end() const {
            return end_;
        }

    private:
        const_iterator begin_;
        const_iterator end_;
    };


    /***********************************************************************/

    #ifndef GOMGEN

    /**
     * \brief An iterator that applies a user-defined function when deferenced
     * \details Used internally by transform_range()
     * \tparam IT iterator type
     * \tparam XFORM functor type
     * \see transform_range()
     */
    template<class IT, typename XFORM> class transformed_iterator {
    public:
	typedef transformed_iterator<IT, XFORM> thisclass;

	transformed_iterator(const IT& it, XFORM xform) :
	    wrapped_(it), xform_(xform) {
	}

	auto operator*() {
	    return xform_(*wrapped_);
	}

	const auto operator*() const {
	    return xform_(*wrapped_);
	}

	bool operator==(const thisclass& rhs) const {
	    return wrapped_ == rhs.wrapped_;
	}

	bool operator!=(const thisclass& rhs) const {
	    return wrapped_ != rhs.wrapped_;
	}

	bool operator<(const thisclass& rhs) const {
	    return wrapped_ < rhs.wrapped_;
	}

        void operator++() {
            ++wrapped_;
        }

    private:
	IT wrapped_;
	XFORM xform_;
    };

    /**
     * \brief An iterator that applies a user-defined function when deferenced
     * \details Used internally by transform_range_ref()
     * \tparam IT iterator type
     * \tparam XFORM functor type
     * \see transform_range_ref()
     */
    template<class IT, typename XFORM> class transformed_iterator_ref {
    public:
	typedef transformed_iterator_ref<IT, XFORM> thisclass;

	transformed_iterator_ref(const IT& it, XFORM xform) :
	    wrapped_(it), xform_(xform) {
	}

	auto& operator*() {
	    return xform_(*wrapped_);
	}

	const auto& operator*() const {
	    return xform_(*wrapped_);
	}

	bool operator==(const thisclass& rhs) const {
	    return wrapped_ == rhs.wrapped_;
	}

	bool operator!=(const thisclass& rhs) const {
	    return wrapped_ != rhs.wrapped_;
	}

	bool operator<(const thisclass& rhs) const {
	    return wrapped_ < rhs.wrapped_;
	}

        void operator++() {
            ++wrapped_;
        }

    private:
	IT wrapped_;
	XFORM xform_;
    };

    /***********************************************************************/

    /**
     * \brief A range composed with a user-defined function
     * \see transform_range()
     * \tparam RANGE range class
     * \tparam XFORM functor type
     */
    template<class RANGE, typename XFORM> class transformed_range {
    public:
	typedef transformed_iterator<typename RANGE::iterator, XFORM> iterator;

	typedef transformed_iterator<
	    typename RANGE::const_iterator, XFORM
	> const_iterator;

	transformed_range(const RANGE& range, XFORM xform) :
	    wrapped_(range), xform_(xform) {
	}

	iterator begin() {
	    return iterator(wrapped_.begin(), xform_);
	}

	iterator end() {
	    return iterator(wrapped_.end(), xform_);
	}

	const_iterator begin() const {
	    return const_iterator(wrapped_.begin(), xform_);
	}

	const_iterator end() const {
	    return const_iterator(wrapped_.end(), xform_);
	}

    private:
	RANGE wrapped_;
	XFORM xform_;
    };

    /***********************************************************************/

    /**
     * \brief A range composed with a user-defined function
     * \details This version is used when user-defined function
     *   returns a reference
     * \see transform_range()
     * \tparam RANGE range class
     * \tparam XFORM functor type
     */
    template<class RANGE, typename XFORM> class transformed_range_ref {
    public:
	typedef transformed_iterator_ref<
	    typename RANGE::iterator, XFORM
	> iterator;

	typedef transformed_iterator_ref<
	    typename RANGE::const_iterator, XFORM
	> const_iterator;

	transformed_range_ref(const RANGE& range, XFORM xform) :
	    wrapped_(range), xform_(xform) {
	}

	iterator begin() {
	    return iterator(wrapped_.begin(), xform_);
	}

	iterator end() {
	    return iterator(wrapped_.end(), xform_);
	}

	const_iterator begin() const {
	    return const_iterator(wrapped_.begin(), xform_);
	}

	const_iterator end() const {
	    return const_iterator(wrapped_.end(), xform_);
	}

    private:
	RANGE wrapped_;
	XFORM xform_;
    };

    /***********************************************************************/

    /**
     * \brief Creates a range that applies a user-defined function to each
     *  element when accessed
     * \param[in] range the range
     * \param[in] xform the transform to be applied to each range element when
     *  accessed. \p xform is supposed to return a value. If \p xform returns
     *  a reference, use transform_range_ref() instead
     * \return a new range object, with special iterators that call \p xform
     *  when deferenced
     */
    template <class RANGE, typename XFORM> inline auto
    transform_range(const RANGE& range, XFORM xform) {
	return transformed_range<RANGE,XFORM>(range, xform);
    }

    /***********************************************************************/

    /**
     * \brief Creates a range that applies a user-defined function to each
     *  element when accessed
     * \param[in] range the range
     * \param[in] xform the transform to be applied to each range element when
     *  accessed. \p xform is supposed to return a reference. If \p xform returns
     *  a value, use transform_range() instead
     * \return a new range object, with special iterators that call \p xform
     *  when deferenced
     */
    template <class RANGE, typename XFORM> inline auto
    transform_range_ref(const RANGE& range, XFORM xform) {
	return transformed_range_ref<RANGE,XFORM>(range, xform);
    }

    /***********************************************************************/

    #endif
}

#endif
