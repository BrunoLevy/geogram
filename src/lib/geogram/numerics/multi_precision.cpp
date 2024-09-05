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

#include <geogram/basic/common.h>

// This makes sure the compiler will not optimize y = a*x+b
// with fused multiply-add, this would break the exact
// predicates.
#ifdef GEO_COMPILER_MSVC
#pragma fp_contract(off)
#endif

#include <geogram/numerics/multi_precision.h>
#include <geogram/numerics/PCK.h>
#include <geogram/basic/process.h>
#include <geogram/basic/logger.h>

namespace {

    using namespace GEO;

#ifdef PCK_STATS
    std::vector<index_t> expansion_length_histo_;
#endif

    /************************************************************************/

    /**
     * \brief An optimized memory allocator for objects of
     *  small size.
     * \details It is used by the high-level class expansion_nt
     *  that allocates expansion objects on the heap. PCK predicates
     *  do not use it (they use the more efficient low-level API
     *  that allocates expansion objects on the stack).
     */
    class Pools {

    public:

        /**
         * \brief Creates a new Pools object
         */
        Pools() : pools_(1024,nullptr) {
            chunks_.reserve(1024);
        }

        /**
         * \brief Pools destructor.
         */
        ~Pools() {
            for(index_t i=0; i<chunks_.size(); ++i) {
                delete[] chunks_[i];
            }
        }

        /**
         * \brief Allocates an element.
         * \param[in] size size in bytes of the element to be allocated
         * \return a pointer to the allocated element
         * \note elements allocated with fast_malloc() should be deallocated
         *  with fast_free()
         */
        void* malloc(size_t size) {
            if(size >= pools_.size()) {
                return ::malloc(size);
            }
            if(pools_[size] == nullptr) {
                new_chunk(size);
            }
            Memory::pointer result = pools_[size];
            pools_[size] = next(pools_[size]);
            return result;
        }

        /**
         * \brief Deallocates an element.
         * \param[in] ptr a pointer to the element to be deallocated
         * \param[in] size number of bytes of the element, as specified
         *   in the call to fast_malloc() that allocated it
         */
        void free(void* ptr, size_t size) {
            if(size >= pools_.size()) {
                ::free(ptr);
                return;
            }
            set_next(Memory::pointer(ptr), pools_[size]);
            pools_[size] = Memory::pointer(ptr);
        }


    protected:
        /**
         * \brief Number of elements in each individual chunk
         *  allocation.
         */
        static const index_t NB_ITEMS_PER_CHUNK = 512;

        /**
         * \brief Allocates a new chunk of elements and prepends
         *  it to the free list for allocations of the specified
         *  size.
         * \param[in] item_size size of the elements to be allocated.
         */
        void new_chunk(size_t item_size) {
            // Allocate chunk
            Memory::pointer chunk =
                new Memory::byte[item_size * NB_ITEMS_PER_CHUNK];
            // Chain items in chunk
            for(index_t i=0; i<NB_ITEMS_PER_CHUNK-1; ++i) {
                Memory::pointer cur_item  = item(chunk, item_size, i);
                Memory::pointer next_item = item(chunk, item_size, i+1);
                set_next(cur_item, next_item);
            }
            // Last item's next is pool's first
            set_next(
                item(chunk, item_size,NB_ITEMS_PER_CHUNK-1),
                pools_[item_size]
            );
            // Set pool's first to first in chunk
            pools_[item_size] = chunk;
            chunks_.push_back(chunk);
        }

    private:

        /**
         * \brief Gets a pointer to the next item
         * \param[in] item a pointer to an item
         * \return a pointer to the next item or
         *  nullptr if we reached the end of the free list
         */
        Memory::pointer next(Memory::pointer item) const {
            return *reinterpret_cast<Memory::pointer*>(item);
        }

        /**
         * \brief Sets the pointer to the next item
         * \param[in] item a pointer to an item
         * \param[in] next a pointer to the next item
         */
        void set_next(
            Memory::pointer item, Memory::pointer next
        ) const {
            *reinterpret_cast<Memory::pointer*>(item) = next;
        }

        /**
         * \brief Gets a pointer to an item by chunk, item_size
         *   and index
         * \pre index < NB_ITEMS_IN_CHUNK
         * \param[in] chunk a pointer to the chunk
         * \param[in] item_size size of the items in chunk
         * \param[in] index index of the item in chunk
         * \return a pointer to the item
         */
        Memory::pointer item(
            Memory::pointer chunk, size_t item_size, index_t index
        ) const {
            geo_debug_assert(index < NB_ITEMS_PER_CHUNK);
            return chunk + (item_size * size_t(index));
        }

        /**
         * \brief The free lists of the pools. Index corresponds
         *  to element size in bytes.
         */
        std::vector<Memory::pointer> pools_;

        /**
         * \brief Pointers to all the allocated chunks.
         * \details Used by the destructor to release all the
         *   allocated memory on exit.
         */
        std::vector<Memory::pointer> chunks_;

    };

    static Pools pools_;

    /************************************************************************/

    /**
     * \brief Computes the sum of two doubles into a length 2 expansion.
     * \details By Jonathan Shewchuk.
     * \param[in] a first argument
     * \param[in] b second argument
     * \param[out] x high-magnitude component of the result
     * \param[out] y low-magnitude component of the result
     * \pre |\p a| > |\p b|
     */
    inline void fast_two_sum(double a, double b, double& x, double& y) {
        x = a + b;
        double bvirt = x - a;
        y = b - bvirt;
    }

#ifdef REMOVE_ME
    /**
     * \brief Computes the difference of two doubles into a length 2 expansion.
     * \details By Jonathan Shewchuk.
     * \param[in] a first argument
     * \param[in] b second argument
     * \param[out] x high-magnitude component of the result
     * \param[out] y low-magnitude component of the result
     * \pre | \p a| > | \p b |
     */
    inline void fast_two_diff(double a, double b, double& x, double& y) {
        x = a - b;
        double bvirt = a - x;
        y = bvirt - b;
    }
#endif

    /**
     * \brief Computes the sum of a length 2 expansion and a double
     *  into a length 3 expansion.
     * \param[in] a1 high-magnitude component of first argument
     * \param[in] a0 low-magnitude component of first argument
     * \param[in] b second argument
     * \param[in] x2 high-magnitude component of the result
     * \param[in] x1 component of the result
     * \param[in] x0 low-magnitude component of the result
     * \details By Jonathan Shewchuk.
     */
    inline void two_one_sum(
        double a1, double a0, double b, double& x2, double& x1, double& x0
    ) {
        double _i;
        two_sum(a0, b, _i, x0);
        two_sum(a1, _i, x2, x1);
    }

    /**
     * \brief Computes the sum of a length 2 expansion and a double
     *  into a length 3 expansion.
     * \param[in] a1 high-magnitude component of first argument
     * \param[in] a0 low-magnitude component of first argument
     * \param[in] b1 high-magnitude component of second argument
     * \param[in] b0 high-magnitude component of second argument
     * \param[in] x3 high-magnitude component of the result
     * \param[in] x2 component of the result
     * \param[in] x1 component of the result
     * \param[in] x0 low-magnitude component of the result
     * \details By Jonathan Shewchuk.
     */
    inline void two_two_sum(
        double a1, double a0, double b1, double b0,
        double& x3, double& x2, double& x1, double& x0
    ) {
        double _j, _0;
        two_one_sum(a1, a0, b0, _j, _0, x0);
        two_one_sum(_j, _0, b1, x3, x2, x1);
    }

#ifndef FP_FAST_FMA

    /**
     * \brief Computes the product between two doubles where
     *  the second one have already been split.
     * \param[in] a first argument
     * \param[in] b second argument
     * \param[in] bhi high-magnitude part of second argument
     * \param[in] blo low-magnitude part of second argument
     * \param[out] x high-magnitude component of the result
     * \param[out] y low-magnitude component of the result
     * \details By Jonathan Shewchuk.
     */
    inline void two_product_presplit(
        double a, double b, double bhi, double blo, double& x, double& y
    ) {
        x = a * b;
        double ahi;
        double alo;
        split(a, ahi, alo);
        double err1 = x - (ahi * bhi);
        double err2 = err1 - (alo * bhi);
        double err3 = err2 - (ahi * blo);
        y = (alo * blo) - err3;
    }

    /**
     * \brief Computes the product between two doubles
     *  where both have already been split.
     * \param[in] a first argument
     * \param[in] ahi high-magnitude part of first argument
     * \param[in] alo low-magnitude part of first argument
     * \param[in] b second argument
     * \param[in] bhi high-magnitude part of second argument
     * \param[in] blo low-magnitude part of second argument
     * \param[out] x high-magnitude component of the result
     * \param[out] y low-magnitude component of the result
     * \details By Jonathan Shewchuk.
     */
    inline void two_product_2presplit(
        double a, double ahi, double alo,
        double b, double bhi, double blo,
        double& x, double& y
    ) {
        x = a * b;
        double err1 = x - (ahi * bhi);
        double err2 = err1 - (alo * bhi);
        double err3 = err2 - (ahi * blo);
        y = (alo * blo) - err3;
    }

#endif

    /**
     * \brief Computes the square of an expansion of length 2.
     * \param[in] a1 high-magnitude component of the argument
     * \param[in] a0 low-magnitude component of the argument
     * \param[out] x an array of six doubles to store the result.
     * \details By Jonathan Shewchuk.
     * An expansion of length two can be squared more quickly than finding the
     *  product of two different expansions of length two, and the result is
     *  guaranteed to have no more than six (rather than eight) components.
     */
    inline void two_square(
        double a1, double a0,
        double* x
    ) {
        double _0, _1, _2;
        double _j, _k, _l;
        square(a0, _j, x[0]);
        _0 = a0 + a0;
        two_product(a1, _0, _k, _1);
        two_one_sum(_k, _1, _j, _l, _2, x[1]);
        square(a1, _j, _1);
        two_two_sum(_j, _1, _l, _2, x[5], x[4], x[3], x[2]);
    }

    /**
     * \brief Computes the product of two expansions of length 2.
     * \param[in] a first argument (array of 2 doubles)
     * \param[in] b second argument (array of 2 doubles)
     * \param[out] x an array of 8 doubles to store the result
     * \details By Jonathan Shewchuk.
     */
    void two_two_product(
        const double* a,
        const double* b,
        double* x
    ) {
        double _0, _1, _2;
        double _i, _j, _k, _l, _m, _n;

        // If the target processor supports the FMA (Fused Multiply Add)
        // instruction, then the product of two doubles into a length-2
        // expansion can be implemented as follows. Thanks to Marc Glisse
        // for the information.
        // Note: under gcc, automatic generations of fma() for a*b+c needs
        // to be deactivated, using -ffp-contract=off, else it may break
        // other functions such as fast_expansion_sum_zeroelim().
#ifdef FP_FAST_FMA
        two_product(a[0],b[0],_i,x[0]);
        two_product(a[1],b[0],_j,_0);
        two_sum(_i, _0, _k, _1);
        fast_two_sum(_j, _k, _l, _2);
        two_product(a[0], b[1], _i, _0);
        two_sum(_1, _0, _k, x[1]);
        two_sum(_2, _k, _j, _1);
        two_sum(_l, _j, _m, _2);
        two_product(a[1], b[1], _j, _0);
        two_sum(_i, _0, _n, _0);
        two_sum(_1, _0, _i, x[2]);
        two_sum(_2, _i, _k, _1);
        two_sum(_m, _k, _l, _2);
        two_sum(_j, _n, _k, _0);
        two_sum(_1, _0, _j, x[3]);
        two_sum(_2, _j, _i, _1);
        two_sum(_l, _i, _m, _2);
        two_sum(_1, _k, _i, x[4]);
        two_sum(_2, _i, _k, x[5]);
        two_sum(_m, _k, x[7], x[6]);
#else
        double a0hi, a0lo;
        split(a[0], a0hi, a0lo);
        double bhi, blo;
        split(b[0], bhi, blo);
        two_product_2presplit(
            a[0], a0hi, a0lo, b[0], bhi, blo, _i, x[0]
        );
        double a1hi, a1lo;
        split(a[1], a1hi, a1lo);
        two_product_2presplit(
            a[1], a1hi, a1lo, b[0], bhi, blo, _j, _0
        );
        two_sum(_i, _0, _k, _1);
        fast_two_sum(_j, _k, _l, _2);
        split(b[1], bhi, blo);
        two_product_2presplit(
            a[0], a0hi, a0lo, b[1], bhi, blo, _i, _0
        );
        two_sum(_1, _0, _k, x[1]);
        two_sum(_2, _k, _j, _1);
        two_sum(_l, _j, _m, _2);
        two_product_2presplit(
            a[1], a1hi, a1lo, b[1], bhi, blo, _j, _0
        );
        two_sum(_i, _0, _n, _0);
        two_sum(_1, _0, _i, x[2]);
        two_sum(_2, _i, _k, _1);
        two_sum(_m, _k, _l, _2);
        two_sum(_j, _n, _k, _0);
        two_sum(_1, _0, _j, x[3]);
        two_sum(_2, _j, _i, _1);
        two_sum(_l, _i, _m, _2);
        two_sum(_1, _k, _i, x[4]);
        two_sum(_2, _i, _k, x[5]);
        two_sum(_m, _k, x[7], x[6]);
#endif
    }

    // [Shewchuk 97]
    // (https://people.eecs.berkeley.edu/~jrs/papers/robustr.pdf)
    // Section 2.8: other operations
    // Compression
    // Note: when converting the algorithms in Shewchuk's article
    // into code, indices in the article go from 1 to m, and in the
    // code they go from 0 to m-1 !!!
    // /!\ there is a bug in the original article,
    // line 14 of the algorigthm should be h_top <= q (small q and not capital Q)

    /**
     * \brief Compresses an expansion
     * \details Modifies in-place an expansion in such a way that it
     *  is shorter. The represented value is not modified.
     * \param[in,out] e a reference to the expansion to be compressed
     */
    void compress_expansion(expansion& e) {
        expansion& h = e;

        index_t m = e.length();
        double Qnew,q;

        index_t bottom = m-1;
        double Q = e[bottom];

        for(int i=int(m)-2; i>=0; --i) {
            fast_two_sum(Q, e[index_t(i)], Qnew, q);
            Q = Qnew;
            if(q != 0.0) {
                h[bottom] = Q;
                --bottom;
                Q = q;
            }
        }
        h[bottom] = Q;

        index_t top = 0;
        for(index_t i=bottom+1; i<m; ++i) {
            fast_two_sum(h[i],Q,Qnew,q);
            Q = Qnew;
            if(q != 0) {
                h[top] = q;
                ++top;
            }
        }
        h[top] = Q;
        h.set_length(top+1);
    }
}

namespace GEO {

    void grow_expansion_zeroelim(
        const expansion& e, double b, expansion& h
    ) {
        double Q, hh;
        double Qnew;
        index_t eindex, hindex;
        index_t elen = e.length();

        hindex = 0;
        Q = b;
        for(eindex = 0; eindex < elen; eindex++) {
            double enow = e[eindex];
            two_sum(Q, enow, Qnew, hh);
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
        }
        if((Q != 0.0) || (hindex == 0)) {
            h[hindex++] = Q;
        }
        h.set_length(hindex);
    }

    void scale_expansion_zeroelim(
        const expansion& e, double b, expansion& h
    ) {
        double Q, sum;
        double hh;
        double product1;
        double product0;
        index_t eindex, hindex;

        // If the target processor supports the FMA (Fused Multiply Add)
        // instruction, then the product of two doubles into a length-2
        // expansion can be implemented as follows. Thanks to Marc Glisse
        // for the information.
        // Note: under gcc, automatic generations of fma() for a*b+c needs
        // to be deactivated, using -ffp-contract=off, else it may break
        // other functions such as fast_expansion_sum_zeroelim().
#ifndef FP_FAST_FMA
        double bhi, blo;
#endif
        index_t elen = e.length();

        // Sanity check: e and h cannot be the same.
        geo_debug_assert(&e != &h);

#ifdef FP_FAST_FMA
        two_product(e[0], b, Q, hh);
#else
        split(b, bhi, blo);
        two_product_presplit(e[0], b, bhi, blo, Q, hh);
#endif

        hindex = 0;
        if(hh != 0) {
            h[hindex++] = hh;
        }
        for(eindex = 1; eindex < elen; eindex++) {
            double enow = e[eindex];
#ifdef FP_FAST_FMA
            two_product(enow, b,  product1, product0);
#else
            two_product_presplit(enow, b, bhi, blo, product1, product0);
#endif
            two_sum(Q, product0, sum, hh);
            if(hh != 0) {
                h[hindex++] = hh;
            }
            fast_two_sum(product1, sum, Q, hh);
            if(hh != 0) {
                h[hindex++] = hh;
            }
        }
        if((Q != 0.0) || (hindex == 0)) {
            h[hindex++] = Q;
        }
        h.set_length(hindex);
    }

    void fast_expansion_sum_zeroelim(
        const expansion& e, const expansion& f, expansion& h
    ) {
        double Q;
        double Qnew;
        double hh;
        index_t eindex, findex, hindex;
        double enow, fnow;
        index_t elen = e.length();
        index_t flen = f.length();

        // sanity check: h cannot be e or f
        geo_debug_assert(&h != &e);
        geo_debug_assert(&h != &f);

        enow = e[0];
        fnow = f[0];
        eindex = findex = 0;
        if((fnow > enow) == (fnow > -enow)) {
            Q = enow;
            enow = e[++eindex];
        } else {
            Q = fnow;
            fnow = f[++findex];
        }
        hindex = 0;
        if((eindex < elen) && (findex < flen)) {
            if((fnow > enow) == (fnow > -enow)) {
                fast_two_sum(enow, Q, Qnew, hh);
                enow = e[++eindex];
            } else {
                fast_two_sum(fnow, Q, Qnew, hh);
                fnow = f[++findex];
            }
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
            while((eindex < elen) && (findex < flen)) {
                if((fnow > enow) == (fnow > -enow)) {
                    two_sum(Q, enow, Qnew, hh);
                    enow = e[++eindex];
                } else {
                    two_sum(Q, fnow, Qnew, hh);
                    fnow = f[++findex];
                }
                Q = Qnew;
                if(hh != 0.0) {
                    h[hindex++] = hh;
                }
            }
        }
        while(eindex < elen) {
            two_sum(Q, enow, Qnew, hh);
            enow = e[++eindex];
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
        }
        while(findex < flen) {
            two_sum(Q, fnow, Qnew, hh);
            fnow = f[++findex];
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
        }
        if((Q != 0.0) || (hindex == 0)) {
            h[hindex++] = Q;
        }
        h.set_length(hindex);
    }

    void fast_expansion_diff_zeroelim(
        const expansion& e, const expansion& f, expansion& h
    ) {
        double Q;
        double Qnew;
        double hh;
        index_t eindex, findex, hindex;
        double enow, fnow;
        index_t elen = e.length();
        index_t flen = f.length();

        // sanity check: h cannot be e or f
        geo_debug_assert(&h != &e);
        geo_debug_assert(&h != &f);

        enow = e[0];
        fnow = -f[0];
        eindex = findex = 0;
        if((fnow > enow) == (fnow > -enow)) {
            Q = enow;
            enow = e[++eindex];
        } else {
            Q = fnow;
            fnow = -f[++findex];
        }
        hindex = 0;
        if((eindex < elen) && (findex < flen)) {
            if((fnow > enow) == (fnow > -enow)) {
                fast_two_sum(enow, Q, Qnew, hh);
                enow = e[++eindex];
            } else {
                fast_two_sum(fnow, Q, Qnew, hh);
                fnow = -f[++findex];
            }
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
            while((eindex < elen) && (findex < flen)) {
                if((fnow > enow) == (fnow > -enow)) {
                    two_sum(Q, enow, Qnew, hh);
                    enow = e[++eindex];
                } else {
                    two_sum(Q, fnow, Qnew, hh);
                    fnow = -f[++findex];
                }
                Q = Qnew;
                if(hh != 0.0) {
                    h[hindex++] = hh;
                }
            }
        }
        while(eindex < elen) {
            two_sum(Q, enow, Qnew, hh);
            enow = e[++eindex];
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
        }
        while(findex < flen) {
            two_sum(Q, fnow, Qnew, hh);
            fnow = -f[++findex];
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
        }
        if((Q != 0.0) || (hindex == 0)) {
            h[hindex++] = Q;
        }
        h.set_length(hindex);
    }

}

/****************************************************************************/

namespace GEO {

    double expansion_splitter_;
    double expansion_epsilon_;

    void expansion::initialize() {
        // Taken from Jonathan Shewchuk's exactinit.
        double half;
        double check, lastcheck;
        int every_other;

        every_other = 1;
        half = 0.5;
        expansion_epsilon_ = 1.0;
        expansion_splitter_ = 1.0;
        check = 1.0;
        // Repeatedly divide `epsilon' by two until it is too small to add to
        // one without causing roundoff.  (Also check if the sum is equal to
        // the previous sum, for machines that round up instead of using exact
        // rounding.  Not that this library will work on such machines anyway.
        do {
            lastcheck = check;
            expansion_epsilon_ *= half;
            if(every_other) {
                expansion_splitter_ *= 2.0;
            }
            every_other = !every_other;
            check = 1.0 + expansion_epsilon_;
        } while((check != 1.0) && (check != lastcheck));
        expansion_splitter_ += 1.0;
    }

    static Process::spinlock expansions_lock = GEOGRAM_SPINLOCK_INIT;

    expansion* expansion::new_expansion_on_heap(index_t capa) {
        Process::acquire_spinlock(expansions_lock);
#ifdef PCK_STATS
        if(capa >= expansion_length_histo_.size()) {
            expansion_length_histo_.resize(capa + 1);
        }
        expansion_length_histo_[capa]++;
#endif
        Memory::pointer addr = Memory::pointer(
            pools_.malloc(expansion::bytes(capa))
        );
        Process::release_spinlock(expansions_lock);
        expansion* result = new(addr)expansion(capa);
        return result;
    }

    void expansion::delete_expansion_on_heap(expansion* e) {
        Process::acquire_spinlock(expansions_lock);
        pools_.free(e, expansion::bytes(e->capacity()));
        Process::release_spinlock(expansions_lock);
    }

    // ====== Initialization from expansion and double ===============

    expansion& expansion::assign_sum(const expansion& a, double b) {
        geo_debug_assert(capacity() >= sum_capacity(a, b));
        grow_expansion_zeroelim(a, b, *this);
        return *this;
    }

    expansion& expansion::assign_diff(const expansion& a, double b) {
        geo_debug_assert(capacity() >= diff_capacity(a, b));
        grow_expansion_zeroelim(a, -b, *this);
        return *this;
    }

    expansion& expansion::assign_product(const expansion& a, double b) {
        // TODO: implement special case where the double argument
        // is a power of two.
        geo_debug_assert(capacity() >= product_capacity(a, b));
        scale_expansion_zeroelim(a, b, *this);
        return *this;
    }

    // =============  expansion sum and difference =========================

    expansion& expansion::assign_sum(
        const expansion& a, const expansion& b
    ) {
        geo_debug_assert(capacity() >= sum_capacity(a, b));
        fast_expansion_sum_zeroelim(a, b, *this);
        return *this;
    }

    expansion& expansion::assign_sum(
        const expansion& a, const expansion& b, const expansion& c
    ) {
        geo_debug_assert(capacity() >= sum_capacity(a, b, c));
        expansion& ab = expansion_sum(a, b);
        this->assign_sum(ab, c);
        return *this;
    }

    expansion& expansion::assign_sum(
        const expansion& a, const expansion& b,
        const expansion& c, const expansion& d
    ) {
        geo_debug_assert(capacity() >= sum_capacity(a, b, c));
        expansion& ab = expansion_sum(a, b);
        expansion& cd = expansion_sum(c, d);
        this->assign_sum(ab, cd);
        return *this;
    }

    expansion& expansion::assign_diff(const expansion& a, const expansion& b) {
        geo_debug_assert(capacity() >= diff_capacity(a, b));
        fast_expansion_diff_zeroelim(a, b, *this);
        return *this;
    }

    // =============  expansion product ==================================

    // Recursive helper function for product implementation
    expansion& expansion::assign_sub_product(
        const double* a, index_t a_length, const expansion& b
    ) {
        geo_debug_assert(
            capacity() >= sub_product_capacity(a_length, b.length())
        );
        if(a_length == 1) {
            scale_expansion_zeroelim(b, a[0], *this);
        } else {
            // "Distillation" (see Shewchuk's paper) is computed recursively,
            // by splitting the list of expansions to sum into two halves.

            const double* a1 = a;
            index_t a1_length = a_length / 2;
            const double* a2 = a1 + a1_length;
            index_t a2_length = a_length - a1_length;

            // Allocate both halves on the stack or on the heap if too large
            // (some platformes, e.g. MacOSX, have a small stack)

            index_t a1b_capa = sub_product_capacity(a1_length, b.length());
            index_t a2b_capa = sub_product_capacity(a2_length, b.length());

            bool a1b_on_heap = (a1b_capa > MAX_CAPACITY_ON_STACK);
            bool a2b_on_heap = (a2b_capa > MAX_CAPACITY_ON_STACK);

            expansion* a1b = a1b_on_heap ?
                new_expansion_on_heap(a1b_capa) :
                new_expansion_on_stack(a1b_capa);

            a1b->assign_sub_product(a1, a1_length, b);

            expansion* a2b = a2b_on_heap ?
                new_expansion_on_heap(a2b_capa) :
                new_expansion_on_stack(a2b_capa);

            a2b->assign_sub_product(a2, a2_length, b);

            this->assign_sum(*a1b, *a2b);

            if(a1b_on_heap) {
                delete_expansion_on_heap(a1b);
            }

            if(a2b_on_heap) {
                delete_expansion_on_heap(a2b);
            }
        }
        return *this;
    }

    expansion& expansion::assign_product(
        const expansion& a, const expansion& b
    ) {
        geo_debug_assert(capacity() >= product_capacity(a, b));
        if(a.length() == 0 || b.length() == 0) {
            x_[0] = 0.0;
            set_length(0);
        } else if(a.length() == 1 && b.length() == 1) {
            two_product(a[0], b[0], x_[1], x_[0]);
            set_length(2);
        } else if(a.length() == 1) {
            scale_expansion_zeroelim(b, a[0], *this);
        } else if(b.length() == 1) {
            scale_expansion_zeroelim(a, b[0], *this);
        } else if(a.length() == 2 && b.length() == 2) {
            two_two_product(a.data(), b.data(), x_);
            set_length(8);
        } else {


            const expansion* pa = &a;
            const expansion* pb = &b;

            if(pa->length() > pb->length()) {
                std::swap(pa, pb);
            }

            // [Shewchuk 97]
            // (https://people.eecs.berkeley.edu/~jrs/papers/robustr.pdf)
            // Section 2.8: other operations
            // Distillation: sum of k values.
            //    Worst case: 1/2*k*(k-1)
            //    But O(k log(k)) if the "summing tree" is well balanced
            //      and using fast_expansion_sum().
            // Recommended way of computing a product:
            //    compute a1*b, a2*b ... ak*b using scale_expansion_zeroelim()
            //    sum them using a well-balanced tree
            // However, there is an extra cost for the recursion (and more
            // importantly, for allocating the intermediary sums, especially
            // when they do not fit on the stack). So when there are less than
            // 16 values to add, we simply accumulate them.

            bool use_balanced_distillation = (pa->length() >= 16);

            if(use_balanced_distillation) {
                // assign_sub_product() is a recursive function that
                // creates a balanced distillation tree on the stack.
                assign_sub_product(pa->data(), pa->length(),*pb);
            } else {
                // trivial implementation: compute all the products
                // P = ak*b and accumulate them into S

                index_t P_capa = product_capacity(*pb, 3.0); // 3.0, or any
                                                             // number that is
                                                             // not a power of 2

                index_t S_capa = capacity(); // same capacity as this,
                                             // enough to store sum.

                bool P_on_heap = (P_capa > MAX_CAPACITY_ON_STACK);
                bool S_on_heap = (S_capa > MAX_CAPACITY_ON_STACK);

                expansion* P = P_on_heap ?
                    new_expansion_on_heap(P_capa) :
                    new_expansion_on_stack(P_capa);

                expansion* S = S_on_heap ?
                    new_expansion_on_heap(S_capa) :
                    new_expansion_on_stack(S_capa);

                expansion* S1 = S;
                expansion* S2 = this;

                if((pa->length()%2) == 0) {
                    std::swap(S1,S2);
                }

                for(index_t i=0; i<pa->length(); ++i) {
                    if(i == 0) {
                        S2->assign_product(*pb, (*pa)[i]);
                    } else {
                        P->assign_product(*pb, (*pa)[i]);
                        S2->assign_sum(*S1,*P);
                    }
                    std::swap(S1,S2);
                }

                geo_assert(S1 == this);

                if(S_on_heap) {
                    delete_expansion_on_heap(S);
                }

                if(P_on_heap) {
                    delete_expansion_on_heap(P);
                }
            }
        }
        return *this;
    }

    expansion& expansion::assign_product(
        const expansion& a, const expansion& b, const expansion& c
    ) {
        const expansion& bc = expansion_product(b, c);
        this->assign_product(a, bc);
        return *this;
    }

    expansion& expansion::assign_square(const expansion& a) {
        geo_debug_assert(capacity() >= square_capacity(a));
        if(a.length() == 1) {
            square(a[0], x_[1], x_[0]);
            set_length(2);
        } else if(a.length() == 2) {
            two_square(a[1], a[0], x_);
            set_length(6);
        } else {
            this->assign_product(a, a);
        }
        return *this;
    }

    // =============  determinants ==========================================

    expansion& expansion::assign_det2x2(
        const expansion& a11, const expansion& a12,
        const expansion& a21, const expansion& a22
    ) {
        const expansion& a11a22 = expansion_product(a11, a22);
        const expansion& a12a21 = expansion_product(a12, a21);
        return this->assign_diff(a11a22, a12a21);
    }

    expansion& expansion::assign_det3x3(
        const expansion& a11, const expansion& a12, const expansion& a13,
        const expansion& a21, const expansion& a22, const expansion& a23,
        const expansion& a31, const expansion& a32, const expansion& a33
    ) {
        // Development w.r.t. first row
        const expansion& c11 = expansion_det2x2(a22, a23, a32, a33);
        const expansion& c12 = expansion_det2x2(a23, a21, a33, a31);
        const expansion& c13 = expansion_det2x2(a21, a22, a31, a32);
        const expansion& a11c11 = expansion_product(a11, c11);
        const expansion& a12c12 = expansion_product(a12, c12);
        const expansion& a13c13 = expansion_product(a13, c13);
        return this->assign_sum(a11c11, a12c12, a13c13);
    }

    expansion& expansion::assign_det_111_2x3(
        const expansion& a21, const expansion& a22, const expansion& a23,
        const expansion& a31, const expansion& a32, const expansion& a33
    ) {
        const expansion& c11 = expansion_det2x2(a22, a23, a32, a33);
        const expansion& c12 = expansion_det2x2(a23, a21, a33, a31);
        const expansion& c13 = expansion_det2x2(a21, a22, a31, a32);
        return this->assign_sum(c11, c12, c13);
    }

    // =============  geometric operations ==================================

    expansion& expansion::assign_sq_dist(
        const double* p1, const double* p2, coord_index_t dim
    ) {
        geo_debug_assert(capacity() >= sq_dist_capacity(dim));
        geo_debug_assert(dim > 0);
        if(dim == 1) {
            double d0, d1;
            two_diff(p1[0], p2[0], d1, d0);
            two_square(d1, d0, x_);
            set_length(6);
        } else {
            // "Distillation" (see Shewchuk's paper) is computed recursively,
            // by splitting the list of expansions to sum into two halves.
            coord_index_t dim1 = dim / 2;
            coord_index_t dim2 = coord_index_t(dim - dim1);
            const double* p1_2 = p1 + dim1;
            const double* p2_2 = p2 + dim1;
            expansion& d1 = expansion_sq_dist(p1, p2, dim1);
            expansion& d2 = expansion_sq_dist(p1_2, p2_2, dim2);
            this->assign_sum(d1, d2);
        }
        return *this;
    }

    expansion& expansion::assign_dot_at(
        const double* p1, const double* p2, const double* p0,
        coord_index_t dim
    ) {
        geo_debug_assert(capacity() >= dot_at_capacity(dim));
        if(dim == 1) {

            double v[2];
            two_diff(p1[0], p0[0], v[1], v[0]);
            double w[2];
            two_diff(p2[0], p0[0], w[1], w[0]);
            two_two_product(v, w, x_);
            set_length(8);
        } else {
            // "Distillation" (see Shewchuk's paper) is computed recursively,
            // by splitting the list of expansions to sum into two halves.
            coord_index_t dim1 = dim / 2;
            coord_index_t dim2 = coord_index_t(dim - dim1);
            const double* p1_2 = p1 + dim1;
            const double* p2_2 = p2 + dim1;
            const double* p0_2 = p0 + dim1;
            expansion& d1 = expansion_dot_at(p1, p2, p0, dim1);
            expansion& d2 = expansion_dot_at(p1_2, p2_2, p0_2, dim2);
            this->assign_sum(d1, d2);
        }
        return *this;
    }

    expansion& expansion::assign_length2(
        const expansion& x, const expansion& y, const expansion& z
    ) {
        const expansion& x2 = expansion_square(x);
        const expansion& y2 = expansion_square(y);
        const expansion& z2 = expansion_square(z);
        this->assign_sum(x2,y2,z2);
        return *this;
    }

    /************************************************************************/

    bool expansion::is_same_as(const expansion& rhs) const {
        if(length() != rhs.length()) {
            return false;
        }
        for(index_t i=0; i<length(); ++i) {
            if(x_[i] != rhs.x_[i]) {
                return false;
            }
        }
        return true;
    }

    bool expansion::is_same_as(double rhs) const {
        if(length() != 1) {
            return false;
        }
        return (x_[0] == rhs);
    }

    Sign expansion::compare(const expansion& rhs) const {
        // Fast path: different signs or both zero
        Sign s1 = sign();
        Sign s2 = rhs.sign();
        if(s1 == ZERO && s2 == ZERO) {
            return ZERO;
        }
        if(s1 != s2) {
            return (int(s1) > int(s2) ? POSITIVE : NEGATIVE);
        }

        // Fast path: same internal representation
        if(is_same_as(rhs)) {
            return ZERO;
        }

        // Compute difference and return sign of difference
        index_t capa = diff_capacity(*this, rhs);
        if(capa > MAX_CAPACITY_ON_STACK) {
            expansion* d = new_expansion_on_heap(capa);
            d->assign_diff(*this, rhs);
            Sign result = d->sign();
            delete_expansion_on_heap(d);
            return result;
        }
        const expansion& d = expansion_diff(*this, rhs);
        return d.sign();
    }

    Sign expansion::compare(double rhs) const {
        // Fast path: different signs or both zero
        Sign s1 = sign();
        Sign s2 = geo_sgn(rhs);
        if(s1 == ZERO && s2 == ZERO) {
            return ZERO;
        }
        if(s1 != s2) {
            return (int(s1) > int(s2) ? POSITIVE : NEGATIVE);
        }

        // Fast path: same internal representation
        if(is_same_as(rhs)) {
            return ZERO;
        }

        // Compute difference and return sign of difference
        index_t capa = diff_capacity(*this, rhs);
        if(capa > MAX_CAPACITY_ON_STACK) {
            expansion* d = new_expansion_on_heap(capa);
            d->assign_diff(*this, rhs);
            Sign result = d->sign();
            delete_expansion_on_heap(d);
            return result;
        }
        const expansion& d = expansion_diff(*this, rhs);
        return d.sign();
    }


/************************************************************************/

    void expansion::show_all_stats() {
#ifdef PCK_STATS
        Logger::out("expansion") << "Stats" << std::endl;
        for(index_t i = 0; i < expansion_length_histo_.size(); ++i) {
            if(expansion_length_histo_[i] != 0) {
                Logger::out("expansion")
                    << "len " << i
                    << " : " << expansion_length_histo_[i] << std::endl;
            }
        }
#endif
    }

    /************************************************************************/

    Sign sign_of_expansion_determinant(
        const expansion& a00,const expansion& a01,
        const expansion& a10,const expansion& a11
    ) {
        const expansion& result = expansion_det2x2(a00, a01, a10, a11);
        return result.sign();
    }

    Sign sign_of_expansion_determinant(
        const expansion& a00,const expansion& a01,const expansion& a02,
        const expansion& a10,const expansion& a11,const expansion& a12,
        const expansion& a20,const expansion& a21,const expansion& a22
    ) {
        // First compute the det2x2
        const expansion& m01 =
            expansion_det2x2(a00, a10, a01, a11);
        const expansion& m02 =
            expansion_det2x2(a00, a20, a01, a21);
        const expansion& m12 =
            expansion_det2x2(a10, a20, a11, a21);

        // Now compute the minors of rank 3
        const expansion& z1 = expansion_product(m01,a22);
        const expansion& z2 = expansion_product(m02,a12).negate();
        const expansion& z3 = expansion_product(m12,a02);

        const expansion& result = expansion_sum3(z1,z2,z3);
        return result.sign();
    }

    Sign sign_of_expansion_determinant(
        const expansion& a00,const expansion& a01,
        const expansion& a02,const expansion& a03,
        const expansion& a10,const expansion& a11,
        const expansion& a12,const expansion& a13,
        const expansion& a20,const expansion& a21,
        const expansion& a22,const expansion& a23,
        const expansion& a30,const expansion& a31,
        const expansion& a32,const expansion& a33
    ) {

        // First compute the det2x2
        const expansion& m01 =
            expansion_det2x2(a10,a00,a11,a01);
        const expansion& m02 =
            expansion_det2x2(a20,a00,a21,a01);
        const expansion& m03 =
            expansion_det2x2(a30,a00,a31,a01);
        const expansion& m12 =
            expansion_det2x2(a20,a10,a21,a11);
        const expansion& m13 =
            expansion_det2x2(a30,a10,a31,a11);
        const expansion& m23 =
            expansion_det2x2(a30,a20,a31,a21);

        // Now compute the minors of rank 3
        const expansion& m012_1 = expansion_product(m12,a02);
        expansion& m012_2 = expansion_product(m02,a12); m012_2.negate();
        const expansion& m012_3 = expansion_product(m01,a22);
        const expansion& m012 = expansion_sum3(m012_1, m012_2, m012_3);

        const expansion& m013_1 = expansion_product(m13,a02);
        expansion& m013_2 = expansion_product(m03,a12); m013_2.negate();

        const expansion& m013_3 = expansion_product(m01,a32);
        const expansion& m013 = expansion_sum3(m013_1, m013_2, m013_3);

        const expansion& m023_1 = expansion_product(m23,a02);
        expansion& m023_2 = expansion_product(m03,a22); m023_2.negate();
        const expansion& m023_3 = expansion_product(m02,a32);
        const expansion& m023 = expansion_sum3(m023_1, m023_2, m023_3);

        const expansion& m123_1 = expansion_product(m23,a12);
        expansion& m123_2 = expansion_product(m13,a22); m123_2.negate();
        const expansion& m123_3 = expansion_product(m12,a32);
        const expansion& m123 = expansion_sum3(m123_1, m123_2, m123_3);

        // Now compute the minors of rank 4
        const expansion& m0123_1 = expansion_product(m123,a03);
        const expansion& m0123_2 = expansion_product(m023,a13);
        const expansion& m0123_3 = expansion_product(m013,a23);
        const expansion& m0123_4 = expansion_product(m012,a33);

        const expansion& z1 = expansion_sum(m0123_1, m0123_3);
        const expansion& z2 = expansion_sum(m0123_2, m0123_4);

        const expansion& result = expansion_diff(z1,z2);
        return result.sign();
    }

    /************************************************************************/

    void expansion::optimize() {
        compress_expansion(*this);
    }

    /************************************************************************/

}
