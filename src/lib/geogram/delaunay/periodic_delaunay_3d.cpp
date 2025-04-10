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

#include <geogram/delaunay/periodic_delaunay_3d.h>
#include <geogram/delaunay/cavity.h>

#include <geogram/mesh/mesh_reorder.h>
#include <geogram/numerics/predicates.h>
#include <geogram/basic/geometry.h>
#include <geogram/basic/stopwatch.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/permutation.h>
#include <geogram/basic/algorithm.h>
#include <geogram/bibliography/bibliography.h>

#include <stack>
#include <algorithm>

#include <mutex>
#include <condition_variable>

// ParallelDelaunayThread class, declared locally, has
// no out-of-line virtual functions. It is not a
// problem since they are only visible from this translation
// unit, but clang will complain.
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wweak-vtables"
#endif

namespace {

    using namespace GEO;

    /**
     * \brief Generates a random integer.
     * \return a random integer between 0 and \p choices - 1
     * \param [in] choices_in number of possible choices for the
     *   random variable (maximum value + 1)
     * \details The function is thread-safe, and uses one seed
     *  per thread.
     */
    GEO::index_t thread_safe_random_(GEO::index_t choices_in) {
#ifdef GARGANTUA
        typedef Numeric::int64 Int;
#else
        typedef long int Int;
#endif
        GEO::signed_index_t choices = signed_index_t(choices_in);
        static thread_local Int randomseed = 1l ;
        if (choices >= 714025l) {
            Int newrandom = (randomseed * 1366l + 150889l) % 714025l;
            randomseed = (newrandom * 1366l + 150889l) % 714025l;
            newrandom = newrandom * (choices / 714025l) + randomseed;
            if (newrandom >= choices) {
                return GEO::index_t(newrandom - choices);
            } else {
                return GEO::index_t(newrandom);
            }
        } else {
            randomseed = (randomseed * 1366l + 150889l) % 714025l;
            return GEO::index_t(randomseed % choices);
        }
    }

    /**
     * \brief Generates a random integer between 0 and 3.
     * \return a random integer between 0 and 3
     * \details The function is thread-safe, and uses one seed
     *  per thread.
     */
    GEO::index_t thread_safe_random_4_() {
        static thread_local long int randomseed = 1l ;
        randomseed = (randomseed * 1366l + 150889l) % 714025l;
        return GEO::index_t(randomseed % 4);
    }

    /**
     * \brief Computes the number of bits set.
     * \param[in] x an integer.
     * \return the number of ones in the binary representation
     *  of the integer.
     */
    inline VBW::index_t pop_count(Numeric::uint32 x) {
#if defined(GEO_COMPILER_GCC_FAMILY)
        return VBW::index_t(Numeric::uint32(__builtin_popcount(x)));
#elif defined(GEO_COMPILER_MSVC)
#if defined(_M_ARM64)
        return VBW::index_t(_CountOneBits(x));
#else
        return VBW::index_t(__popcnt(x));
#endif
#else
        int result = 0;
        for(int b=0; b<32; ++b) {
            result += ((x & 1) != 0);
            x >>= 1;
        }
        return VBW::index_t(result);
#endif
    }

    /************************************************************************/

    // TODO: move these two functions to mesh_reorder.h

    void compute_BRIO_order_periodic_recursive(
        GEO::index_t nb_vertices, const double* vertices,
        GEO::index_t dimension, GEO::index_t stride,
        vector<GEO::index_t>& sorted_indices,
        vector<GEO::index_t>::iterator b,
        vector<GEO::index_t>::iterator e,
        const vec3& period,
        GEO::index_t threshold,
        double ratio,
        GEO::index_t& depth,
        vector<GEO::index_t>* levels
    ) {
        geo_debug_assert(e > b);

        vector<GEO::index_t>::iterator m = b;
        if(GEO::index_t(e - b) > threshold) {
            ++depth;
            m = b + int(double(e - b) * ratio);
            compute_BRIO_order_periodic_recursive(
                nb_vertices, vertices,
                dimension, stride,
                sorted_indices, b, m,
		period,
                threshold, ratio, depth,
                levels
            );
        }

        Hilbert_sort_periodic(
            nb_vertices, vertices,
            sorted_indices,
            dimension,
            stride,
            m,
	    e,
            period
        );

        if(levels != nullptr) {
            levels->push_back(GEO::index_t(e - sorted_indices.begin()));
        }
    }

    void compute_BRIO_order_periodic(
        GEO::index_t nb_vertices, const double* vertices,
        GEO::index_t dimension, GEO::index_t stride,
        vector<GEO::index_t>& sorted_indices,
        vector<GEO::index_t>::iterator first,
        vector<GEO::index_t>::iterator last,
        const vec3& period,
        GEO::index_t threshold = 64,
        double ratio = 0.125,
        vector<GEO::index_t>* levels = nullptr
    ) {
        if(levels != nullptr) {
            levels->clear();
            levels->push_back(GEO::index_t(first - sorted_indices.begin()));
        }
        GEO::index_t depth = 0;
	GEO::random_shuffle(first, last);

        compute_BRIO_order_periodic_recursive(
            nb_vertices, vertices,
            dimension, stride,
            sorted_indices,
            first, last,
            period, threshold, ratio, depth, levels
        );
    }

    /************************************************************************/

    void delaunay_citations() {
        geo_cite_with_info(
            "DBLP:journals/cj/Bowyer81",
            "One of the two initial references to the algorithm, "
            "discovered independently and simultaneously by Bowyer and Watson."
        );
        geo_cite_with_info(
            "journals/cj/Watson81",
            "One of the two initial references to the algorithm, "
            "discovered independently and simultaneously by Bowyer and Watson."
        );
        geo_cite_with_info(
            "DBLP:conf/compgeom/AmentaCR03",
            "Using spatial sorting has a dramatic impact on the performances."
        );
        geo_cite_with_info(
            "DBLP:journals/comgeo/FunkeMN05",
            "Initializing \\verb|locate()| with a non-exact version "
            " (structural filtering) gains (a bit of) performance."
        );
        geo_cite_with_info(
            "DBLP:journals/comgeo/BoissonnatDPTY02",
            "The idea of traversing the cavity from inside "
            " used in GEOGRAM is inspired by the implementation of "
            " \\verb|Delaunay_triangulation_3| in CGAL."
        );
        geo_cite_with_info(
            "DBLP:conf/imr/Si06",
            "The triangulation data structure used in GEOGRAM is inspired "
            "by Tetgen."
        );
        geo_cite_with_info(
            "DBLP:journals/ijfcs/DevillersPT02",
            "Analysis of the different versions of the line walk algorithm "
            " used by \\verb|locate()|."
        );
    }
}

/************************************************************************/

// These two functions are missing when compiling in PSM mode.
#ifdef GEOGRAM_PSM
    namespace GEO {
	namespace PCK {
	    inline Sign det_3d(const vec3& p0, const vec3& p1, const vec3& p2) {
		return det_3d(p0.data(), p1.data(), p2.data());
	    }
	    inline Sign det_4d(
		const vec4& p0, const vec4& p1,	const vec4& p2, const vec4& p3
	    ) {
		return det_4d(p0.data(), p1.data(), p2.data(), p3.data());
	    }
	}
    }
#endif

/************************************************************************/

namespace GEO {

    /**
     * \brief One of the threads of the multi-threaded
     * 3d Delaunay implementation.
     */
    class PeriodicDelaunay3dThread : public GEO::Thread, public Periodic {
    public:
        friend class PeriodicDelaunay3d;

        /**
         * \brief Symbolic value for cell_status_[t] that
         *  indicates that no thread owns t.
         */
        static constexpr index_t NO_THREAD = CellStatusArray::FREE_CELL;

        /**
         * \brief Creates a new PeriodicDelaunay3dThread.
         * \details Each PeriodicDelaunay3dThread has an affected working
         *  zone, i.e. a range of tetrahedra indices in which the
         *  thread is allowed to create tetrahedra.
         * \param[in] master a pointer to the PeriodicDelaunay3d
         *  this thread belongs to
         * \param[in] pool_begin first tetrahedron index of
         *  the working zone of this PeriodicDelaunay3dThread
         * \param[in] pool_end one position past the last tetrahedron
         *  index of the working zone of this PeriodicDelaunay3dThread
         */
        PeriodicDelaunay3dThread(
            PeriodicDelaunay3d* master,
            index_t pool_begin,
            index_t pool_end
        ) :
            master_(master),
            periodic_(master->periodic_),
            period_(master->period_),
            cell_to_v_store_(master_->cell_to_v_store_),
            cell_to_cell_store_(master_->cell_to_cell_store_),
            cell_next_(master_->cell_next_),
            cell_status_(master_->cell_status_),
            has_empty_cells_(false) {

	    max_t_ = master_->cell_next_.size();

	    nb_vertices_ = master_->nb_vertices();
	    nb_vertices_non_periodic_ = master_->nb_vertices_non_periodic_;
	    vertices_ = master_->vertex_ptr(0);
	    weights_ = master_->weights_;
	    dimension_ = master_->dimension();
	    reorder_ = master_->reorder_.data();

	    b_hint_ = NO_TETRAHEDRON;
	    e_hint_ = NO_TETRAHEDRON;

	    nb_rollbacks_ = 0;
	    nb_failed_locate_ = 0;

	    set_pool(pool_begin, pool_end);
	}

	/**
	 * \brief Resets thread statistics
	 */
	void reset_stats() {
	    nb_rollbacks_ = 0;
	    nb_failed_locate_ = 0;
	}

        /**
         * \brief Initializes the pool of tetrahedra for this thread.
         * \param[in] pool_begin first tetrahedron index of
         *  the working zone of this PeriodicDelaunay3dThread
         * \param[in] pool_end one position past the last tetrahedron
         *  index of the working zone of this PeriodicDelaunay3dThread
         */
        void set_pool(index_t pool_begin, index_t pool_end) {
	    pool_begin_ = pool_begin;
	    pool_end_ = pool_end;
            // Initialize free list in memory pool
            first_free_ = pool_begin;
            for(index_t t=pool_begin; t<pool_end-1; ++t) {
                cell_next_[t] = t+1;
            }
            cell_next_[pool_end-1] = END_OF_LIST;
            nb_free_ = pool_end - pool_begin;
            memory_overflow_ = false;
            work_begin_ = NO_INDEX;
            work_rbegin_ = NO_INDEX;
            finished_ = false;
            direction_ = true;
#ifdef GEO_DEBUG
            nb_acquired_tets_ = 0;
#endif
            interfering_thread_ = NO_THREAD;
            nb_tets_to_create_ = 0;
            t_boundary_ = NO_TETRAHEDRON;
            f_boundary_ = NO_INDEX;
            used_tets_end_ = pool_begin;
        }

        /**
         * \brief Tests whether this thread created empty cells.
         * \retval true if this thread created empty cells.
         * \retval false otherwise.
         */
        bool has_empty_cells() {
            return has_empty_cells_;
        }

	/**
	 * \brief Picks a random tetrahedron in this thread's pool.
	 * \retval If no valid tet exists in this thread's pool,
	 *   returns NO_TETRAHEDRON. It can be a real tetrahedron
	 * \retval Otherwise, returns a finite tetrahedon, an infinite
	 *   tetrahedron or a tetrahedon in the free list. Caller needs to check.
	 */
	index_t pick_random_tet() const {
	    // Shit happens [Forrest Gump]
	    if(used_tets_end_ == pool_begin_) {
		return NO_TETRAHEDRON;
	    }
	    return pool_begin_ + thread_safe_random_(
		used_tets_end_ - pool_begin_
	    );
	}

        /**
         * \brief Gets the number of rollbacks.
         * \return the number of rollbacks
         * \details rollbacks occur whenever a point
         *  could not be inserted, due to interferences
         *  from other threads
         */
        index_t nb_rollbacks() const {
            return nb_rollbacks_;
        }

        /**
         * \brief Gets the number of failed locate() calls.
         * \return the number of failed locate() calls
         * \details locate() can fail when it cannot acquire
         *  the tetrahedra that are traversed, due to
         *  interferences from another thread.
         */
        index_t nb_failed_locate() const {
            return nb_failed_locate_;
        }

        /**
         * \brief Gets the number of tetrahedra traversed by
         *  the latest locate() invocation.
         */
        index_t nb_traversed_tets() const {
            return nb_traversed_tets_;
        }

        /**
         * \brief Sets the point index sequence that
         *  should be processed by this thread.
         * \param[in] b index of the first point to insert
         * \param[in] e one position past the index of the
         *   last point to insert
         */
        void set_work(index_t b, index_t e) {
            work_begin_ = b;
            // e is one position past the last point index
            // to insert. Internally we store the last point index
	    // to insert (like rbegin in STL containers). This is
	    // because we manipulate the point sequence to insert
	    // from both ends.
            work_rbegin_ = e-1;
	    // reorder_ may have changed if new vertices were
	    // inserted into it
	    reorder_ = master_->reorder_.data();
        }

        /**
         * \brief Gets the number of remaining points to
         *  be inserted.
         * \return the number of points to be inserted by
         *  this thread
         */
        index_t work_size() const {
            if(work_begin_ == NO_INDEX && work_rbegin_ == NO_INDEX) {
                return 0;
            }
            geo_debug_assert(work_begin_ != NO_INDEX);
            geo_debug_assert(work_rbegin_ != NO_INDEX);
            return std::max(work_rbegin_ - work_begin_ + 1, index_t(0));
        }

        /**
         * \brief Gets the number of threads.
         * \return the number of threads created by
         *  the master PeriodicDelaunay3d of this thread.
         */
        index_t nb_threads() const {
            return index_t(master_->threads_.size());
        }

        /**
         * \brief Gets a thread by index
         * \pre t < nb_threads()
         * \param[in] t index of the thread
         * \return a poiner to the \p t th thread
         */
        PeriodicDelaunay3dThread* thread(index_t t) {
            return static_cast<PeriodicDelaunay3dThread*>(
                master_->threads_[t].get()
            );
        }

        /**
         * \brief Inserts the point sequence allocated to
         *  this thread.
         * \details The point sequence was previously defined
         *  by set_work().
         */
        void run() override {
            has_empty_cells_ = false;
            finished_ = false;

            if(work_begin_ == NO_INDEX || work_rbegin_ == NO_INDEX) {
                return ;
            }

            memory_overflow_ = false;

            // Current hint associated with b
            b_hint_ = NO_TETRAHEDRON;

            // Current hint associated with e
            e_hint_ = NO_TETRAHEDRON;

            // If true, insert in b->e order,
            // else insert in e->b order
            direction_ = true;

            while(
                work_rbegin_ >= work_begin_ &&
                !memory_overflow_ &&
                !has_empty_cells_ &&
                !master_->has_empty_cells_
            ) {
                index_t v = direction_ ? work_begin_ : work_rbegin_ ;
                index_t& hint = direction_ ? b_hint_ : e_hint_ ;

                // Try to insert v and update hint
                bool success = insert(reorder_[v],hint);

                //   Notify all threads that are waiting for
                // this thread to release some tetrahedra.
                send_event();

                if(success) {
                    if(direction_) {
                        ++work_begin_;
                    } else {
                        --work_rbegin_;
                    }
                } else {
                    ++nb_rollbacks_;
                    if(interfering_thread_ != NO_THREAD) {
                        if(id() < interfering_thread_) {
                            // If this thread has a higher priority than
                            // the one that interfered, wait for the
                            // interfering thread to release the tets that
                            // it holds (then the loop will retry to insert
                            // the same vertex).
                            wait_for_event(interfering_thread_);
                        } else {
                            // If this thread has a lower priority than
                            // the interfering thread, try inserting
                            // from the other end of the points sequence.
                            direction_ = !direction_;
                        }
                    }
                }
            }

            if(has_empty_cells_) {
                master_->has_empty_cells_ = true;
            }

            //   Fix by Hiep Vu: wake up threads that potentially missed
            // the previous wake ups.
            mutex_.lock();
	    finished_ = true;
            send_event();
            mutex_.unlock();
        }

        /**
         * \brief Symbolic constant for uninitialized hint.
         * \details Locate functions can be accelerated by
         *  specifying a hint. This constant indicates that
         *  no hint is given.
         */
        static constexpr index_t NO_TETRAHEDRON = NO_INDEX;

        /**
         * \brief Symbolic value for a vertex of a
         *  tetrahedron that indicates a virtual tetrahedron.
         * \details The three other vertices then correspond to a
         *  facet on the convex hull of the points.
         */
        static constexpr index_t VERTEX_AT_INFINITY = NO_INDEX;


        /**
         * \brief Maximum valid index for a tetrahedron.
         * \return the maximum valid index for a tetrahedron
	 *  This includes not only real tetrahedra, but also
	 *  the virtual ones on the border, the conflict
         *  list and the free list.
         */
        index_t max_t() const {
            return max_t_;
        }

	/**
         * \brief Sets the maximum valid index for a tetrahedron.
         * \details Needs to be called when starting the threads, and
	 *  whenever memory allocation occured.
	 * \param[in] max_t the maximum valid index for a tetrahedron.
	 *  This includes not only real tetrahedra,
         *  but also the virtual ones on the border, the conflict
         *  list and the free list.
	 */
	void set_max_t(index_t max_t) {
	    max_t_ = max_t;
	}

        /**
         * \brief Tests whether a given tetrahedron
         *   is a finite one.
         * \details Infinite tetrahedra are the ones
         *   that are incident to the infinite vertex
         *   (index -1)
         * \param[in] t the index of the tetrahedron
         * \retval true if \p t is finite
         * \retval false otherwise
         */
        bool tet_is_finite(index_t t) const {
            return
                cell_to_v_store_[4 * t]     != NO_INDEX &&
                cell_to_v_store_[4 * t + 1] != NO_INDEX &&
                cell_to_v_store_[4 * t + 2] != NO_INDEX &&
                cell_to_v_store_[4 * t + 3] != NO_INDEX ;
        }

        /**
         * \brief Tests whether a tetrahedron is
         *  a real one.
         * \details Real tetrahedra are incident to
         *  four user-specified vertices (there are also
         *  virtual tetrahedra that are incident to the
         *  vertex at infinity, with index -1)
         * \param[in] t index of the tetrahedron
         * \retval true if tetrahedron \p t is a real one
         * \retval false otherwise
         */
        bool tet_is_real(index_t t) const {
            return !tet_is_free(t) && tet_is_finite(t);
        }

        /**
         * \brief Tests whether a tetrahedron is
         *  a real one and has at least a vertex in the core.
         * \details Real tetrahedra are incident to
         *  four user-specified vertices (there are also
         *  virtual tetrahedra that are incident to the
         *  vertex at infinity, with index -1)
         * \param[in] t index of the tetrahedron
         * \retval true if tetrahedron \p t is a real one
         * \retval false otherwise
         */
        bool tet_is_real_non_periodic(index_t t) const {
            return !tet_is_free(t) && tet_is_finite(t) && (
                finite_tet_vertex(t,0) < nb_vertices_non_periodic_ ||
                finite_tet_vertex(t,1) < nb_vertices_non_periodic_ ||
                finite_tet_vertex(t,2) < nb_vertices_non_periodic_ ||
                finite_tet_vertex(t,3) < nb_vertices_non_periodic_ ) ;
        }

        /**
         * \brief Tests whether a tetrahedron is
         *  in the free list.
         * \details Deleted tetrahedra are recycled
         *  in a free list.
         * \param[in] t index of the tetrahedron
         * \retval true if tetrahedron \p t is in
         * the free list
         * \retval false otherwise
         */
        bool tet_is_free(index_t t) const {
            return tet_is_in_list(t);
        }

        /**
         * \brief Tests whether a tetrahedron is contained
         *  by a given linked list.
         * \details Used for debugging purposes.
         * \param[in] t the tetrahedron
         * \param[in] first the first element of the list
         * \retval true if \p t is contained in the list starting
         *   at \p first
         * \retval false otherwise
         */
        bool tet_is_in_list(index_t t, index_t first) const {
            for(
                index_t cur = first; cur != END_OF_LIST;
                cur = tet_next(cur)
            ) {
                if(cur == t) {
                    return true;
                }
            }
            return false;
        }


        /**
         * \brief Finds in the pointset a set of four non-coplanar
         *  points and creates a tetrahedron that connects them.
         * \details This function is used to initiate the incremental
         *  Delaunay construction, it should be called only once.
         * \retval the index of the created tetrahedron
         * \retval NO_TETRAHEDRON if all points were coplanar
         */
        index_t create_first_tetrahedron() {
            index_t iv0,iv1,iv2,iv3;
            if(nb_vertices() < 4) {
                return NO_TETRAHEDRON;
            }

            iv0 = 0;

            iv1 = 1;
            while(
                iv1 < nb_vertices_non_periodic_ &&
                PCK::points_are_identical_3d(
                    non_periodic_vertex_ptr(iv0),
                    non_periodic_vertex_ptr(iv1)
                )
            ) {
                ++iv1;
            }
            if(iv1 == nb_vertices()) {
                return NO_TETRAHEDRON;
            }

            iv2 = iv1 + 1;
            while(
                iv2 < nb_vertices_non_periodic_ &&
                PCK::points_are_colinear_3d(
                    non_periodic_vertex_ptr(iv0),
                    non_periodic_vertex_ptr(iv1),
                    non_periodic_vertex_ptr(iv2)
                )
            ) {
                ++iv2;
            }
            if(iv2 == nb_vertices()) {
                return NO_TETRAHEDRON;
            }

            iv3 = iv2 + 1;
            Sign s = ZERO;
            while(
                iv3 < nb_vertices_non_periodic_ &&
                (s = PCK::orient_3d(
                    non_periodic_vertex_ptr(iv0),
                    non_periodic_vertex_ptr(iv1),
                    non_periodic_vertex_ptr(iv2),
                    non_periodic_vertex_ptr(iv3)
                )) == ZERO
            ) {
                ++iv3;
            }

            if(iv3 == nb_vertices()) {
                return NO_TETRAHEDRON;
            }

            geo_debug_assert(s != ZERO);

            if(s == NEGATIVE) {
                std::swap(iv2, iv3);
            }

            // Create the first tetrahedron
            index_t t0 = new_tetrahedron(iv0, iv1, iv2, iv3);

            // Create the first four virtual tetrahedra surrounding it
            index_t t[4];
            for(index_t f = 0; f < 4; ++f) {
                // In reverse order since it is an adjacent tetrahedron
                index_t v1 = tet_vertex(t0, tet_facet_vertex(f,2));
                index_t v2 = tet_vertex(t0, tet_facet_vertex(f,1));
                index_t v3 = tet_vertex(t0, tet_facet_vertex(f,0));
                t[f] = new_tetrahedron(VERTEX_AT_INFINITY, v1, v2, v3);
            }

            // Connect the virtual tetrahedra to the real one
            for(index_t f=0; f<4; ++f) {
                set_tet_adjacent(t[f], 0, t0);
                set_tet_adjacent(t0, f, t[f]);
            }

            // Interconnect the four virtual tetrahedra along their common
            // faces
            for(index_t f = 0; f < 4; ++f) {
                // In reverse order since it is an adjacent tetrahedron
                index_t lv1 = tet_facet_vertex(f,2);
                index_t lv2 = tet_facet_vertex(f,1);
                index_t lv3 = tet_facet_vertex(f,0);
                set_tet_adjacent(t[f], 1, t[lv1]);
                set_tet_adjacent(t[f], 2, t[lv2]);
                set_tet_adjacent(t[f], 3, t[lv3]);
            }

            release_tets();

            return t0;
        }


        /**
         * \brief Creates a star of tetrahedra filling the conflict zone.
         * \param[in] v the index of the point to be inserted
         * \details This function is used when the Cavity computed
         *  when traversing the conflict zone is OK, that is to say
         *  when its array sizes were not exceeded.
         * \return the index of one the newly created tetrahedron
         */
        index_t stellate_cavity(index_t v) {
            index_t new_tet = NO_INDEX;

            for(index_t f=0; f<cavity_.nb_facets(); ++f) {
                index_t old_tet = cavity_.facet_tet(f);
                index_t lf = cavity_.facet_facet(f);
                index_t t_neigh = tet_adjacent(old_tet, lf);
                index_t v1 = cavity_.facet_vertex(f,0);
                index_t v2 = cavity_.facet_vertex(f,1);
                index_t v3 = cavity_.facet_vertex(f,2);
                new_tet = new_tetrahedron(v, v1, v2, v3);
                set_tet_adjacent(new_tet, 0, t_neigh);
                set_tet_adjacent(
                    t_neigh, find_tet_adjacent(t_neigh,old_tet), new_tet
                );
                cavity_.set_facet_tet(f, new_tet);
            }

            for(index_t f=0; f<cavity_.nb_facets(); ++f) {
                new_tet = cavity_.facet_tet(f);
                index_t neigh1, neigh2, neigh3;
                cavity_.get_facet_neighbor_tets(f, neigh1, neigh2, neigh3);
                set_tet_adjacent(new_tet, 1, neigh1);
                set_tet_adjacent(new_tet, 2, neigh2);
                set_tet_adjacent(new_tet, 3, neigh3);
            }

            return new_tet;
        }


        /**
         * \brief Inserts a point in the triangulation.
         * \param[in] v the index of the point to be inserted
         * \param[in,out] hint the index of a tetrahedron as near as
         *  possible to \p v, or NO_TETRAHEDRON if unspecified. On
         *  exit, the index of one of the tetrahedra incident to
         *  point \p v
         * \retval true if insertion was successful, that is, if there
	 *  was no interference. This includes the situation where the
	 *  point already exists (even if this does not create a new
	 *  vertex)
         * \retval false otherwise
         */
        bool insert(index_t v, index_t& hint) {

            vec3 p = vertex(v);

            Sign orient[4];
            index_t t = locate(v,p,hint,orient);

            //   locate() may fail due to tets already owned by
            // other threads.
            if(t == NO_TETRAHEDRON) {
                ++nb_failed_locate_;
                geo_debug_assert(nb_acquired_tets_ == 0);
                return false;
            }

            //  At this point, t is a valid tetrahedron,
            // and this thread acquired a lock on it.

            // Test whether the point already exists in
            // the triangulation. The point already exists
            // if it's located on three faces of the
            // tetrahedron returned by locate().
            int nb_zero =
                (orient[0] == ZERO) +
                (orient[1] == ZERO) +
                (orient[2] == ZERO) +
                (orient[3] == ZERO) ;

            if(nb_zero >= 3) {
                release_tet(t);
                return true;
            }

            geo_debug_assert(nb_acquired_tets_ == 1);

            index_t t_bndry = NO_TETRAHEDRON;
            index_t f_bndry = NO_INDEX;

            vec4 p_lifted = lifted_vertex(v,p);

            cavity_.clear();

            bool ok = find_conflict_zone(v,p_lifted,t,t_bndry,f_bndry);

            // When in multithreading mode, we cannot allocate memory
            // dynamically and we use a fixed pool. If the fixed pool
            // is full, then we exit the thread (and the missing points
            // are inserted after, in sequential mode).
            if(
                nb_tets_to_create_ > nb_free_ &&
                Process::is_running_threads()
            ) {
                memory_overflow_ = true;
                ok = false;
            }

            if(!ok) {
                //  At this point, this thread did not successfully
                // acquire all the tets in the conflict zone, so
                // we need to rollback.
                release_tets();
                geo_debug_assert(nb_acquired_tets_ == 0);
                return false;
            }

            // The conflict list can be empty if
            //  the triangulation is weighted and v is not visible
            if(tets_to_delete_.size() == 0) {
                release_tets();
                geo_debug_assert(nb_acquired_tets_ == 0);
                has_empty_cells_ = true;
                return true;
            }


            geo_debug_assert(
                nb_acquired_tets_ ==
                tets_to_delete_.size() + tets_to_release_.size()
            );

#ifdef GEO_DEBUG
            // Sanity check: make sure this threads owns all the tets
            // in conflict and their neighbors.
            for(index_t i=0; i<tets_to_delete_.size(); ++i) {
                index_t tdel = tets_to_delete_[i];
                geo_debug_assert(owns_tet(tdel));
                for(index_t lf=0; lf<4; ++lf) {
                    geo_debug_assert(tet_adjacent(tdel,lf) != NO_INDEX);
                    geo_debug_assert(owns_tet(tet_adjacent(tdel,lf)));
                }
            }
#endif
            geo_debug_assert(owns_tet(t_bndry));
            geo_debug_assert(owns_tet(tet_adjacent(t_bndry,f_bndry)));
            geo_debug_assert(
                !tet_is_marked_as_conflict(tet_adjacent(t_bndry,f_bndry))
            );

            //   At this point, this thread owns all the tets in conflict and
            // their neighbors, therefore no other thread can interfere, and
            // we can update the triangulation.

            index_t new_tet = NO_INDEX;
            if(cavity_.OK()) {
                new_tet = stellate_cavity(v);
            } else {
                new_tet = stellate_conflict_zone_iterative(v,t_bndry,f_bndry);
            }

            // Recycle the tetrahedra of the conflict zone.
            for(index_t i=0; i<tets_to_delete_.size()-1; ++i) {
                cell_next_[tets_to_delete_[i]] = tets_to_delete_[i+1];
            }
            cell_next_[tets_to_delete_[tets_to_delete_.size()-1]] =
                first_free_;
            first_free_ = tets_to_delete_[0];
            nb_free_ += nb_tets_in_conflict();

            // Reset deleted tet.
            // This is needed because we update_v_to_cell() in
            // a transient state.
            // Note: update_v_to_cell() is overloaded here,
            // with a check on nb_vertices_non_periodic_,
            // this is why the (-2) does not make everything crash.
            for(index_t i=0; i<tets_to_delete_.size(); ++i) {
                index_t tdel = tets_to_delete_[i];
                set_tet_vertex(tdel, 0, VERTEX_OF_DELETED_TET);
                set_tet_vertex(tdel, 1, VERTEX_OF_DELETED_TET);
                set_tet_vertex(tdel, 2, VERTEX_OF_DELETED_TET);
                set_tet_vertex(tdel, 3, VERTEX_OF_DELETED_TET);
            }

            // Return one of the newly created tets
            hint=new_tet;
            release_tets();
            geo_debug_assert(nb_acquired_tets_ == 0);
            return true;
        }

        /**
         * \brief Determines the list of tetrahedra in conflict
         *  with a given point.
         * \param[in] v the index of the point to be inserted
         * \param[in] t the index of a tetrahedron that contains
         *  \p p, as returned by locate()
         * \param[out] t_bndry a tetrahedron adjacent to the
         *  boundary of the conflict zone
         * \param[out] f_bndry the facet along which t_bndry is
         *  adjacent to the boundary of the conflict zone
         *  The other tetrahedra are linked, and can be traversed
         *  from \p first by using tet_next() until \p last or END_OF_LIST
         *  is reached.
         *  The conflict zone can be empty under two circumstances:
         *  - the vertex \p v already exists in the triangulation
         *  - the triangulation is weighted and \p v is not visible
         *  in either cases, both \p first and \p last contain END_OF_LIST
         * \retval true if all the tetrahedra of the conflict zone and their
         *  neighbors could be acquired by this thread
         * \retval false otherwise
         */
        bool find_conflict_zone(
            index_t v, const vec4& p, index_t t,
            index_t& t_bndry, index_t& f_bndry
        ) {
            nb_tets_to_create_ = 0;

            geo_debug_assert(t != NO_TETRAHEDRON);
            geo_debug_assert(owns_tet(t));

            // Pointer to the coordinates of the point to be inserted
            //const double* p = vertex_ptr(v);

            //  Weighted triangulations can have dangling
            // vertices. Such vertices p are characterized by
            // the fact that p is not in conflict with the
            // tetrahedron returned by locate().
            if(!tet_is_in_conflict(t,v,p)) {
                release_tet(t);
                return true;
            }

            mark_tet_as_conflict(t);

            //   Sanity check: the vertex to be inserted should
            // not correspond to one of the vertices of t.
            geo_debug_assert(v != tet_vertex(t,0));
            geo_debug_assert(v != tet_vertex(t,1));
            geo_debug_assert(v != tet_vertex(t,2));
            geo_debug_assert(v != tet_vertex(t,3));

            // Note: points on edges and on facets are
            // handled by the way tet_is_in_conflict()
            // is implemented, that naturally inserts
            // the correct tetrahedra in the conflict list.

            // Determine the conflict list by greedy propagation from t.
            bool result = find_conflict_zone_iterative(v, p,t);
            t_bndry = t_boundary_;
            f_bndry = f_boundary_;
            return result;
        }


        /**
         * \brief This function is used to implement find_conflict_zone.
         * \details This function detects the neighbors of \p t that are
         *  in the conflict zone and calls itself recursively on them.
         * \param[in] v_in the index of the point to be inserted
         * \param[in] p_in the point to be inserted
         * \param[in] t_in index of a tetrahedron in the conflict zone
         * \pre The tetrahedron \p t was alredy marked as
         *  conflict (tet_is_in_list(t))
         */
        bool find_conflict_zone_iterative(
            index_t v_in, const vec4& p_in, index_t t_in
        ) {
            geo_debug_assert(owns_tet(t_in));
            S_.push_back(SFrame(t_in, v_in, p_in));

            while(S_.size() != 0) {
                index_t t = S_.rbegin()->t;
                index_t v = S_.rbegin()->v;
                vec4    p = S_.rbegin()->p;
                S_.pop_back();

                geo_debug_assert(owns_tet(t));

                for(index_t lf = 0; lf < 4; ++lf) {
                    index_t v2=v;
                    vec4 p2=p;

                    index_t t2 = tet_adjacent(t, lf);

                    // If t2 is already owned by current thread, then
                    // its status was previously determined.
                    if(owns_tet(t2)) {

                        geo_debug_assert(
                            tet_is_marked_as_conflict(t2) ==
                            tet_is_in_conflict(t2,v2,p2)
                        );

                        // If t2 is not in conflict list, then t has a facet
                        // on the border of the conflict zone, and there is
                        // a tet to create.
                        if(!tet_is_marked_as_conflict(t2)) {
                            ++nb_tets_to_create_;
                            cavity_.new_facet(
                                t, lf,
                                tet_vertex(t, tet_facet_vertex(lf,0)),
                                tet_vertex(t, tet_facet_vertex(lf,1)),
                                tet_vertex(t, tet_facet_vertex(lf,2))
                            );
                        }
                        continue;
                    }

                    if(!acquire_tet(t2)) {
                        S_.resize(0);
                        return false;
                    }

                    geo_debug_assert(owns_tet(t2));

                    if(!tet_is_in_conflict(t2,v2,p2)) {
                        mark_tet_as_neighbor(t2);
                        // If t2 is not in conflict list, then t has a facet
                        // on the border of the conflict zone, and there is
                        // a tet to create.
                        ++nb_tets_to_create_;
                    } else {
                        mark_tet_as_conflict(t2);
                        geo_debug_assert(owns_tet(t2));
                        S_.push_back(SFrame(t2,v2,p2));
                        continue;
                    }

                    //  At this point, t is in conflict
                    // and t2 is not in conflict.
                    // We keep a reference to a tet on the boundary
                    t_boundary_ = t;
                    f_boundary_ = lf;
                    cavity_.new_facet(
                        t, lf,
                        tet_vertex(t, tet_facet_vertex(lf,0)),
                        tet_vertex(t, tet_facet_vertex(lf,1)),
                        tet_vertex(t, tet_facet_vertex(lf,2))
                    );
                    geo_debug_assert(tet_adjacent(t,lf) == t2);
                    geo_debug_assert(owns_tet(t));
                    geo_debug_assert(owns_tet(t2));
                }
            }
            return true;
        }

        /**
         * \brief Gets a pointer to a vertex by its global index.
         * \param[in] i global index of the vertex
         * \return a pointer to vertex \p i
         */
        const double* non_periodic_vertex_ptr(index_t i) const {
            geo_debug_assert(i < nb_vertices_non_periodic_);
            return vertices_ + i*3;
        }

        /**
         * \brief Gets the weight of a vertex by its global index.
         * \param[in] i global index of the vertex
         * \return the weight associated with vertex i
         */
        double non_periodic_weight(index_t i) const {
            geo_debug_assert(i < nb_vertices_non_periodic_);
            return (weights_ == nullptr) ? 0.0 : weights_[i];
        }


        /**
         * \brief gets a vertex.
         * \param[in] v the index of the vertex, in 0..nb_vertices_-1
         * \param[out] result a pointer to the 3d coordinates of the vertex.
         * \details In periodic mode, does vertex translation (v is a
         *  virtual vertex index).
         */
        void get_vertex(index_t v, double* result) const {
            if(!periodic_) {
                result[0] = vertices_[3*v];
                result[1] = vertices_[3*v+1];
                result[2] = vertices_[3*v+2];
                return;
            }
            index_t instance = periodic_vertex_instance(v);
            v = periodic_vertex_real(v);
            result[0] = vertices_[3*v];
            result[1] = vertices_[3*v+1];
            result[2] = vertices_[3*v+2];
            result[0] += double(translation[instance][0]) * period_.x;
            result[1] += double(translation[instance][1]) * period_.y;
            result[2] += double(translation[instance][2]) * period_.z;
        }

        /**
         * \brief gets a vertex.
         * \param[in] v the index of the vertex, in 0..nb_vertices_-1
         * \details In periodic mode, does vertex translation (v is a
         *  virtual vertex index).
         * \return the 3d vertex.
         */
        vec3 vertex(index_t v) const {
            vec3 result;
            get_vertex(v, result.data());
            return result;
        }

        /**
         * \brief gets a lifted vertex.
         * \param[in] v the index of the vertex, in 0..nb_vertices_-1
         * \param[out] result the 4d coordinates of the vertex,
         *  shifted on the paraboloid and shifted by the weight.
         * \details In periodic mode, does vertex translation (v is a
         *  virtual vertex index).
         */
        void get_lifted_vertex(index_t v, double* result) const {
            index_t instance = 0;
            if(periodic_) {
                instance = periodic_vertex_instance(v);
                v = periodic_vertex_real(v);
            }
            result[0] = vertices_[3*v];
            result[1] = vertices_[3*v+1];
            result[2] = vertices_[3*v+2];
            result[3] = -non_periodic_weight(v);
            if(periodic_) {
                result[0] += double(translation[instance][0]) * period_.x;
                result[1] += double(translation[instance][1]) * period_.y;
                result[2] += double(translation[instance][2]) * period_.z;
            }
            result[3] +=
                geo_sqr(result[0]) + geo_sqr(result[1]) + geo_sqr(result[2]);
        }

        /**
         * \brief gets a lifted vertex.
         * \param[in] v the index of the vertex, in 0..nb_vertices_-1
         * \details In periodic mode, does vertex translation (v is a
         *  virtual vertex index).
         * \return the 4d vertex, lifted on the paraboloid, and shifted by
         *  the weight.
         */
        vec4 lifted_vertex(index_t v) const {
            vec4 result;
            get_lifted_vertex(v,result.data());
            return result;
        }

        /**
         * \brief gets a lifted vertex.
         * \param[in] v the index of the vertex, in 0..nb_vertices_-1
         * \param[in] p the geometric location of vertex v
         * \details In periodic mode, does vertex translation (v is a
         *  virtual vertex index).
         * \return the 4d vertex, lifted on the paraboloid, and shifted by
         *  the weight.
         */
        vec4 lifted_vertex(index_t v, const vec3& p) {
            return vec4(
                p.x, p.y, p.z,
                geo_sqr(p.x) + geo_sqr(p.y) + geo_sqr(p.z)
                - non_periodic_weight(periodic_vertex_real(v))
            );
        }

        /**
         * \brief Orientation predicate with indices.
         * \param i , j , k , l the four indices of the four vertices.
         * \see PCK::orient_3d()
         */
        Sign orient_3d(index_t i, index_t j, index_t k, index_t l) const {
            // No need to reorder since there is no SOS.
            double V[4][3];
            get_vertex(i,V[0]);
            get_vertex(j,V[1]);
            get_vertex(k,V[2]);
            get_vertex(l,V[3]);
            return PCK::orient_3d(V[0], V[1], V[2], V[3]);
        }

        /**
         * \brief Orientation predicate with indices.
         * \param i , j , k , l the four indices of the four vertices.
         * \see PCK::in_circle_3dlifted_SOS()
         */
        Sign in_circle_3dlifted_SOS(
            index_t i, index_t j, index_t k, index_t l
        ) const {

            // In non-periodic mode, directly access vertices.
            if(!periodic_) {
                const double* pi = non_periodic_vertex_ptr(i);
                const double* pj = non_periodic_vertex_ptr(j);
                const double* pk = non_periodic_vertex_ptr(k);
                const double* pl = non_periodic_vertex_ptr(l);

                double hi = geo_sqr(pi[0]) + geo_sqr(pi[1]) + geo_sqr(pi[2])
                    - non_periodic_weight(i);

                double hj = geo_sqr(pj[0]) + geo_sqr(pj[1]) + geo_sqr(pj[2])
                    - non_periodic_weight(j);

                double hk = geo_sqr(pk[0]) + geo_sqr(pk[1]) + geo_sqr(pk[2])
                    - non_periodic_weight(k);

                double hl = geo_sqr(pl[0]) + geo_sqr(pl[1]) + geo_sqr(pl[2])
                    - non_periodic_weight(l);

                return PCK::in_circle_3dlifted_SOS(
                    pi, pj, pk, pl,
                    hi, hj, hk, hl
                );
            }

            // Note: in periodic mode, SOS mode is lexicographic.
            double V[4][4];
            get_lifted_vertex(i,V[0]);
            get_lifted_vertex(j,V[1]);
            get_lifted_vertex(k,V[2]);
            get_lifted_vertex(l,V[3]);
            return PCK::in_circle_3dlifted_SOS(
                V[0],    V[1],    V[2],    V[3],
                V[0][3], V[1][3], V[2][3], V[3][3]
            );
        }

        /**
         * \brief 4D Orientation predicate with indices.
         * \param i , j , k , l , m the five indices of the four vertices.
         * \see PCK::orient_3dlifted_SOS()
         */
        Sign orient_3dlifted_SOS(
            index_t i, index_t j, index_t k, index_t l, index_t m
        ) const {

            // In non-periodic mode, directly access vertices.
            if(!periodic_) {
                const double* pi = non_periodic_vertex_ptr(i);
                const double* pj = non_periodic_vertex_ptr(j);
                const double* pk = non_periodic_vertex_ptr(k);
                const double* pl = non_periodic_vertex_ptr(l);
                const double* pm = non_periodic_vertex_ptr(m);

                double hi = geo_sqr(pi[0]) + geo_sqr(pi[1]) + geo_sqr(pi[2])
                    - non_periodic_weight(i);

                double hj = geo_sqr(pj[0]) + geo_sqr(pj[1]) + geo_sqr(pj[2])
                    - non_periodic_weight(j);

                double hk = geo_sqr(pk[0]) + geo_sqr(pk[1]) + geo_sqr(pk[2])
                    - non_periodic_weight(k);

                double hl = geo_sqr(pl[0]) + geo_sqr(pl[1]) + geo_sqr(pl[2])
                    - non_periodic_weight(l);

                double hm = geo_sqr(pm[0]) + geo_sqr(pm[1]) + geo_sqr(pm[2])
                    - non_periodic_weight(m);

                return PCK::orient_3dlifted_SOS(
                    pi, pj, pk, pl, pm,
                    hi, hj, hk, hl, hm
                );
            }

            // Periodic mode.
            // Note: in periodic mode, SOS mode is lexicographic.
            double V[5][4];

            /*
              The code below is an inlined version of:
              (gains a little bit)
              get_lifted_vertex(i,V[0]);
              get_lifted_vertex(j,V[1]);
              get_lifted_vertex(k,V[2]);
              get_lifted_vertex(l,V[3]);
              get_lifted_vertex(m,V[4]);
            */

            index_t ii = periodic_vertex_instance(i);
            index_t ij = periodic_vertex_instance(j);
            index_t ik = periodic_vertex_instance(k);
            index_t il = periodic_vertex_instance(l);
            index_t im = periodic_vertex_instance(m);

            i = periodic_vertex_real(i);
            j = periodic_vertex_real(j);
            k = periodic_vertex_real(k);
            l = periodic_vertex_real(l);
            m = periodic_vertex_real(m);

            V[0][0] = vertices_[3*i  ] + double(translation[ii][0]) * period_.x;
            V[0][1] = vertices_[3*i+1] + double(translation[ii][1]) * period_.y;
            V[0][2] = vertices_[3*i+2] + double(translation[ii][2]) * period_.z;
            V[1][0] = vertices_[3*j  ] + double(translation[ij][0]) * period_.x;
            V[1][1] = vertices_[3*j+1] + double(translation[ij][1]) * period_.y;
            V[1][2] = vertices_[3*j+2] + double(translation[ij][2]) * period_.z;
            V[2][0] = vertices_[3*k  ] + double(translation[ik][0]) * period_.x;
            V[2][1] = vertices_[3*k+1] + double(translation[ik][1]) * period_.y;
            V[2][2] = vertices_[3*k+2] + double(translation[ik][2]) * period_.z;
            V[3][0] = vertices_[3*l  ] + double(translation[il][0]) * period_.x;
            V[3][1] = vertices_[3*l+1] + double(translation[il][1]) * period_.y;
            V[3][2] = vertices_[3*l+2] + double(translation[il][2]) * period_.z;
            V[4][0] = vertices_[3*m  ] + double(translation[im][0]) * period_.x;
            V[4][1] = vertices_[3*m+1] + double(translation[im][1]) * period_.y;
            V[4][2] = vertices_[3*m+2] + double(translation[im][2]) * period_.z;

            // Beware the parentheses, they are necessary to ensure that computations
            //               |                               give always the same result.
            //               |________________________________________________________________________
            //                                  |                                                     |
            //                                  v                                                     v
            V[0][3] = -non_periodic_weight(i) + (geo_sqr(V[0][0]) + geo_sqr(V[0][1]) + geo_sqr(V[0][2]));
            V[1][3] = -non_periodic_weight(j) + (geo_sqr(V[1][0]) + geo_sqr(V[1][1]) + geo_sqr(V[1][2]));
            V[2][3] = -non_periodic_weight(k) + (geo_sqr(V[2][0]) + geo_sqr(V[2][1]) + geo_sqr(V[2][2]));
            V[3][3] = -non_periodic_weight(l) + (geo_sqr(V[3][0]) + geo_sqr(V[3][1]) + geo_sqr(V[3][2]));
            V[4][3] = -non_periodic_weight(m) + (geo_sqr(V[4][0]) + geo_sqr(V[4][1]) + geo_sqr(V[4][2]));

            return PCK::orient_3dlifted_SOS(
                V[0],    V[1],    V[2],    V[3],    V[4],
                V[0][3], V[1][3], V[2][3], V[3][3], V[4][3]
            );
        }

        /**
         * \brief Tests whether a given tetrahedron is in conflict with
         *  a given 3d point.
         * \details A real tetrahedron is in conflict with a point whenever
         *  the point is contained by its circumscribed sphere, and a
         *  virtual tetrahedron is in conflict with a point whenever the
         *  tetrahedron formed by its real face and with the point has
         *  positive orientation.
         * \param[in] t the index of the tetrahedron
         * \param[in] v the index of the point
         * \param[in] p a pointer to the coordinates of the point
         * \retval true if point \p p is in conflict with tetrahedron \p t
         * \retval false otherwise
         */
        bool tet_is_in_conflict(index_t t, index_t v, const vec4& p) const {

            geo_argused(p);

            // Lookup tetrahedron vertices

            index_t iv[4];
            for(index_t i=0; i<4; ++i) {
                iv[i] = tet_vertex(t,i);
            }


            // Check for virtual tetrahedra (then in_sphere()
            // is replaced with orient3d())
            for(index_t lf = 0; lf < 4; ++lf) {

                if(iv[lf] == NO_INDEX) {

                    // Facet of a virtual tetrahedron opposite to
                    // infinite vertex corresponds to
                    // the triangle on the convex hull of the points.
                    // Orientation is obtained by replacing vertex lf
                    // with p.
                    iv[lf] = v;

                    // no SOS, we can directly use the PCK predicate
                    Sign sign = orient_3d(iv[0], iv[1], iv[2], iv[3]);

                    if(sign > 0) {
                        return true;
                    }

                    if(sign < 0) {
                        return false;
                    }

                    // If sign is zero, we check the real tetrahedron
                    // adjacent to the facet on the convex hull.
                    geo_debug_assert(tet_adjacent(t, lf) != NO_INDEX);
                    index_t t2 = tet_adjacent(t, lf);
                    geo_debug_assert(!tet_is_virtual(t2));

                    //   If t2 was already visited by this thread, then
                    // it is in conflict if it is already marked.
                    if(owns_tet(t2)) {
                        return tet_is_marked_as_conflict(t2);
                    }

                    //  If t2 was not already visited, then we need to
                    // switch to the in_circum_circle_3d() predicate.

                    index_t iq0 = iv[(lf+1)%4];
                    index_t iq1 = iv[(lf+2)%4];
                    index_t iq2 = iv[(lf+3)%4];

                    return (in_circle_3dlifted_SOS(iq0, iq1, iq2, v) > 0);
                }
            }

            //   If the tetrahedron is a finite one, it is in conflict
            // if its circumscribed sphere contains the point (this is
            // the standard case).

            return (orient_3dlifted_SOS(iv[0], iv[1], iv[2], iv[3], v) > 0);
        }


        /**
         * \brief Finds the tetrahedron that contains a point.
         * \details The tetrahedron is acquired by this thread. If the
         *  tetrahedron could not be acquired, then NO_TETRAHEDRON is returned.
         *  If the point is on a face, edge or vertex,
         *  the function returns one of the tetrahedra incident
         *  to that face, edge or vertex.
         * \param[in] v the index of the vertex to locate
         * \param[in] p the coordinates of the point that correspond to v
         * \param[out] orient a pointer to an array of four Sign%s
         *  or nullptr. If non-nullptr, returns the orientation with respect
         *  to the four facets of the tetrahedron that contains \p p.
         * \retval the index of a tetrahedron that contains \p p.
         *  If the point is outside the convex hull of
         *  the inserted so-far points, then the returned tetrahedron
         *  is a virtual one (first vertex is the "vertex at infinity"
         *  of index -1)
         * \retval NO_TETRAHEDRON if the tetrahedron could not be
         *  acquired by this thread, or if the virtual tetrahedra
         *  were previously removed
         */
        index_t locate(
            index_t& v, vec3& p, index_t hint = NO_TETRAHEDRON,
            Sign* orient = nullptr
        ) {
            geo_argused(v);
            nb_traversed_tets_ = 0;

            // If no hint specified, find a tetrahedron randomly

            if(hint != NO_TETRAHEDRON) {
                if(tet_is_free(hint)) {
                    hint = NO_TETRAHEDRON;
                } else {
                    if( !owns_tet(hint) && !acquire_tet(hint) ) {
                        hint = NO_TETRAHEDRON;
                    }
                    if((hint != NO_TETRAHEDRON) && tet_is_free(hint)) {
                        release_tet(hint);
                        hint = NO_TETRAHEDRON;
                    }
                }
            }

            do {
                while(hint == NO_TETRAHEDRON) {
                    hint = master_->thread(0)->pick_random_tet();
		    // we could also pick from a random thread,
		    // but at initialization only thread0 has tets,
		    // so let us keep thread0 for now
                }
                if(
                    tet_is_free(hint) || (!owns_tet(hint) && !acquire_tet(hint))
                ) {
                    if(owns_tet(hint)) {
                        release_tet(hint);
                    }
                    hint = NO_TETRAHEDRON;
                } else {
                    for(index_t f=0; f<4; ++f) {
                        if(tet_vertex(hint,f) == VERTEX_AT_INFINITY) {
                            index_t new_hint = tet_adjacent(hint,f);
                            if(
                                tet_is_free(new_hint) ||
                                !acquire_tet(new_hint)
                            ) {
                                new_hint = NO_TETRAHEDRON;
                            }
                            release_tet(hint);
                            hint = new_hint;
                            break;
                        }
                    }
                }
            } while(hint == NO_TETRAHEDRON) ;

            index_t t = hint;
            index_t t_pred = NO_TETRAHEDRON;
            Sign orient_local[4];
            if(orient == nullptr) {
                orient = orient_local;
            }


        still_walking:
            {
                if(t_pred != NO_TETRAHEDRON) {
                    release_tet(t_pred);
                }

                if(tet_is_free(t)) {
                    return NO_TETRAHEDRON;
                }

                if(!owns_tet(t) && !acquire_tet(t)) {
                    return NO_TETRAHEDRON;
                }


                if(!tet_is_real(t)) {
                    release_tet(t);
                    return NO_TETRAHEDRON;
                }

                vec3 pv[4];
                pv[0] = vertex(finite_tet_vertex(t,0));
                pv[1] = vertex(finite_tet_vertex(t,1));
                pv[2] = vertex(finite_tet_vertex(t,2));
                pv[3] = vertex(finite_tet_vertex(t,3));

                // Start from a random facet
                index_t f0 = thread_safe_random_4_();
                for(index_t df = 0; df < 4; ++df) {
                    index_t f = (f0 + df) % 4;

                    index_t t_next = tet_adjacent(t,f);

                    //  If the opposite tet is -1, then it means that
                    // we are trying to locate() (e.g. called from
                    // nearest_vertex) within a tetrahedralization
                    // from which the infinite tets were removed.
                    if(t_next == NO_INDEX) {
                        release_tet(t);
                        return NO_TETRAHEDRON;
                    }

                    //   If the candidate next tetrahedron is the
                    // one we came from, then we know already that
                    // the orientation is positive, thus we examine
                    // the next candidate (or exit the loop if they
                    // are exhausted).
                    if(t_next == t_pred) {
                        orient[f] = POSITIVE ;
                        continue ;
                    }

                    //   To test the orientation of p w.r.t. the facet f of
                    // t, we replace vertex number f with p in t (same
                    // convention as in CGAL).
                    // This is equivalent to tet_facet_point_orient3d(t,f,p)
                    // (but less costly, saves a couple of lookups)
                    vec3 pv_bkp = pv[f];
                    pv[f] = p;
                    orient[f] = PCK::orient_3d(
                        pv[0].data(), pv[1].data(), pv[2].data(), pv[3].data()
                    );

                    //   If the orientation is not negative, then we cannot
                    // walk towards t_next, and examine the next candidate
                    // (or exit the loop if they are exhausted).
                    if(orient[f] != NEGATIVE) {
                        pv[f] = pv_bkp;
                        continue;
                    }

                    //  If the opposite tet is a virtual tet, then
                    // the point has a positive orientation relative
                    // to the facet on the border of the convex hull,
                    // thus t_next is a tet in conflict and we are
                    // done.
                    if(tet_is_virtual(t_next)) {
                        release_tet(t);
                        if(!acquire_tet(t_next)) {
                            return NO_TETRAHEDRON;
                        }
                        for(index_t lf = 0; lf < 4; ++lf) {
                            orient[lf] = POSITIVE;
                        }
                        return t_next;
                    }

                    ++nb_traversed_tets_;

                    //   If we reach this point, then t_next is a valid
                    // successor, thus we are still walking.
                    t_pred = t;
                    t = t_next;
                    goto still_walking;
                }
            }

            //   If we reach this point, we did not find a valid successor
            // for walking (a face for which p has negative orientation),
            // thus we reached the tet for which p has all positive
            // face orientations (i.e. the tet that contains p).

#ifdef GEO_DEBUG
            geo_debug_assert(tet_is_real(t));

            vec3 pv[4];
            Sign signs[4];
            pv[0] = vertex(finite_tet_vertex(t,0));
            pv[1] = vertex(finite_tet_vertex(t,1));
            pv[2] = vertex(finite_tet_vertex(t,2));
            pv[3] = vertex(finite_tet_vertex(t,3));
            for(index_t f=0; f<4; ++f) {
                vec3 pv_bkp = pv[f];
                pv[f] = vec3(p.x, p.y, p.z);
                signs[f] = PCK::orient_3d(
		    pv[0].data(), pv[1].data(), pv[2].data(), pv[3].data()
		);
                geo_debug_assert(signs[f] >= 0);
                pv[f] = pv_bkp;
            }
#endif

            return t;
        }


    protected:

        /**
         * \brief Tests whether a tetrahedron was marked as conflict.
         * \pre owns_tet(t)
         * \param[in] t the index of the tetrahedron to be tested
         * \retval true if \p t was marked as conflict
         * \retval false otherwise
         */
        bool tet_is_marked_as_conflict(index_t t) const {
            geo_debug_assert(owns_tet(t));
            return cell_status_.cell_is_marked_as_conflict(t);
        }


        /**
         * \brief Gets the number of tetrahedra in conflict.
         * \return the number of tetrahedra in conflict,
         *  specified by mark_tet_as_conflict()
         */
        index_t nb_tets_in_conflict() const {
            return tets_to_delete_.size();
        }

        /**
         * \brief Marks a tetrahedron as conflict.
         * \details The index of the tetrahedron is also
         *  stored it in the list of conflict tetrahedra.
         * \param[in] t index of the tetrahedron to mark
         * \pre owns_tet(t)
         */
        void mark_tet_as_conflict(index_t t) {
            geo_debug_assert(owns_tet(t));
            tets_to_delete_.push_back(t);
            cell_status_.mark_cell_as_conflict(t);
            geo_debug_assert(owns_tet(t));
            geo_debug_assert(tet_is_marked_as_conflict(t));
        }

        /**
         * \brief Marks a tetrahedron as neighbor of the conflict zone.
         * \details The index of the tetrahedron is also
         *  stored it in the list of tetrahedra to release.
         * \param[in] t index of the tetrahedron to mark
         * \pre owns_tet(t)
         */
        void mark_tet_as_neighbor(index_t t) {
            //   Note: nothing to change in cell_status_[t]
            // since MSB=0 means neigbhor tet.
            tets_to_release_.push_back(t);
        }

        /**
         * \brief Acquires a lock on a tetrahedron and keep
         *  it in the list of acquired tetrahedra.
         * \param[in] t index of the tetrahedron to acquire
         */
        void acquire_and_mark_tet_as_created(index_t t) {
            //  The tet was created in this thread's tet pool,
            // therefore there is no need to use sync
            // primitives to acquire a lock on it.
            geo_debug_assert(cell_status_.cell_thread(t) == NO_THREAD);
            cell_status_.set_cell_status(
		t, CellStatusArray::thread_index_t(id())
	    );

#ifdef GEO_DEBUG
            ++nb_acquired_tets_;
#endif
            tets_to_release_.push_back(t);
        }


        /**
         * \brief Releases all the tetrahedron locks that were
         *  acquired using mark_tet_as_neighbor(),
         *  acquire_and_mark_tet_as_created() and mark_as_conflict().
         */
        void release_tets() {
            for(index_t i=0; i<tets_to_release_.size(); ++i) {
                release_tet(tets_to_release_[i]);
            }
            tets_to_release_.resize(0);
            for(index_t i=0; i<tets_to_delete_.size(); ++i) {
                release_tet(tets_to_delete_[i]);
            }
            tets_to_delete_.resize(0);
        }

        /**
         * \brief Atomically acquires a lock on a tetrahedron.
         * \details When the lock could not be acquired, interfering_thread_
         *  contains the id of the thread that owns the lock.
         * \param[in] t the index of the tetrahedron to acquire
         * \retval true if the lock was successfully acquired
         * \retval false otherwise
         */
        bool acquire_tet(index_t t) {
            geo_debug_assert(t < max_t());
            geo_debug_assert(!owns_tet(t));

            interfering_thread_ = cell_status_.acquire_cell(
                t, CellStatusArray::thread_index_t(id())
            );

            if(interfering_thread_ == NO_THREAD) {
                geo_debug_assert(t == first_free_ || !tet_is_in_list(t));
#ifdef GEO_DEBUG
                ++nb_acquired_tets_;
#endif
                return true;
            }
            return false;
        }

        /**
         * \brief Releases a lock on a tetrahedron, making it
         *  available to the other threads.
         */
        void release_tet(index_t t) {
            geo_debug_assert(t < max_t());
            geo_debug_assert(owns_tet(t));
#ifdef GEO_DEBUG
            --nb_acquired_tets_;
#endif
            cell_status_.release_cell(t);
        }


        /**
         * \brief Tests whether this thread owns a tetrahedron.
         * \param[in] t index of the tetrahedron
         * \retval true if this thread owns t
         * \retval false otherwise
         */
        bool owns_tet(index_t t) const {
            geo_debug_assert(t < max_t());
            return (
		cell_status_.cell_thread(t) ==
		CellStatusArray::thread_index_t(id())
	    );
        }

        /**
         * \brief Tests whether a tetrahedron is
         *  a virtual one.
         * \details Virtual tetrahedra are tetrahedra
         *  incident to the vertex at infinity.
         * \param[in] t index of the tetrahedron
         * \retval true if tetrahedron \p t is virtual
         * \retval false otherwise
         */
        bool tet_is_virtual(index_t t) const {
            return
                !tet_is_free(t) && (
                    cell_to_v_store_[4 * t] == VERTEX_AT_INFINITY ||
                    cell_to_v_store_[4 * t + 1] == VERTEX_AT_INFINITY ||
                    cell_to_v_store_[4 * t + 2] == VERTEX_AT_INFINITY ||
                    cell_to_v_store_[4 * t + 3] == VERTEX_AT_INFINITY) ;
        }


        /**
         * \brief Returns the local index of a vertex by
         *   facet and by local vertex index in the facet.
         * \details
         * tet facet vertex is such that the tetrahedron
         * formed with:
         * - vertex lv
         * - tet_facet_vertex(lv,0)
         * - tet_facet_vertex(lv,1)
         * - tet_facet_vertex(lv,2)
         * has the same orientation as the original tetrahedron for
         * any vertex lv.
         * \param[in] f local facet index, in (0,1,2,3)
         * \param[in] v local vertex index, in (0,1,2)
         * \return the local tetrahedron vertex index of
         *  vertex \p v in facet \p f
         */
        static index_t tet_facet_vertex(index_t f, index_t v) {
            geo_debug_assert(f < 4);
            geo_debug_assert(v < 3);
            return index_t(tet_facet_vertex_[f][v]);
        }

        /**
         * \brief Gets the index of a vertex of a tetrahedron
         * \param[in] t index of the tetrahedron
         * \param[in] lv local vertex (0,1,2 or 3) index in \p t
         * \return the global index of the \p lv%th vertex of tetrahedron \p t
         *  or -1 if the vertex is at infinity
         */
        index_t tet_vertex(index_t t, index_t lv) const {
            geo_debug_assert(t < max_t());
            geo_debug_assert(lv < 4);
            return cell_to_v_store_[4 * t + lv];
        }

        /**
         * \brief Finds the index of the vertex in a tetrahedron.
         * \param[in] t the tetrahedron
         * \param[in] v the vertex
         * \return iv such that tet_vertex(t,v)==iv
         * \pre \p t is incident to \p v
         */
        index_t find_tet_vertex(index_t t, index_t v) const {
            geo_debug_assert(t < max_t());
            //   Find local index of v in tetrahedron t vertices.
            const index_t* T = &(cell_to_v_store_[4 * t]);
            return find_4(T,v);
        }


        /**
         * \brief Gets the index of a vertex of a tetrahedron
         * \param[in] t index of the tetrahedron
         * \param[in] lv local vertex (0,1,2 or 3) index in \p t
         * \return the global index of the \p lv%th vertex of tetrahedron \p t
         * \pre Vertex \p lv of tetrahedron \p t is not at infinity
         */
        index_t finite_tet_vertex(index_t t, index_t lv) const {
            geo_debug_assert(t < max_t());
            geo_debug_assert(lv < 4);
            geo_debug_assert(cell_to_v_store_[4 * t + lv] != NO_INDEX);
            return cell_to_v_store_[4 * t + lv];
        }

        /**
         * \brief Sets a tetrahedron-to-vertex adjacency.
         * \param[in] t index of the tetrahedron
         * \param[in] lv local vertex index (0,1,2 or 3) in \p t
         * \param[in] v global index of the vertex
         */
        void set_tet_vertex(index_t t, index_t lv, index_t v) {
            geo_debug_assert(t < max_t());
            geo_debug_assert(lv < 4);
            geo_debug_assert(owns_tet(t));
            cell_to_v_store_[4 * t + lv] = v;
        }

        /**
         * \brief Gets the index of a tetrahedron adjacent to another one.
         * \param[in] t index of the tetrahedron
         * \param[in] lf local facet (0,1,2 or 3) index in \p t
         * \return the tetrahedron adjacent to \p t accorss facet \p lf
         */
        index_t tet_adjacent(index_t t, index_t lf) const {
            geo_debug_assert(t < max_t());
            geo_debug_assert(lf < 4);
            index_t result = cell_to_cell_store_[4 * t + lf];
            return result;
        }

        /**
         * \brief Sets a tetrahedron-to-tetrahedron adjacency.
         * \param[in] t1 index of the first tetrahedron
         * \param[in] lf1 local facet index (0,1,2 or 3) in t1
         * \param[in] t2 index of the tetrahedron
         *  adjacent to \p t1 across \p lf1
         */
        void set_tet_adjacent(index_t t1, index_t lf1, index_t t2) {
            geo_debug_assert(t1 < max_t());
            geo_debug_assert(t2 < max_t());
            geo_debug_assert(lf1 < 4);
            geo_debug_assert(owns_tet(t1));
            geo_debug_assert(owns_tet(t2));
            cell_to_cell_store_[4 * t1 + lf1] = t2;
        }

        /**
         * \brief Finds the index of the facet across which t1 is
         *  adjacent to t2.
         * \param[in] t1 first tetrahedron
         * \param[in] t2 second tetrahedron
         * \return f such that tet_adjacent(t1,f)==t2
         * \pre \p t1 and \p t2 are adjacent
         */
        index_t find_tet_adjacent(index_t t1, index_t t2) const {
            geo_debug_assert(t1 < max_t());
            geo_debug_assert(t2 < max_t());
            geo_debug_assert(t1 != t2);

            // Find local index of t2 in tetrahedron t1 adajcent tets.
            const index_t* T = &(cell_to_cell_store_[4 * t1]);
            index_t result = find_4(T,t2);

            // Sanity check: make sure that t1 is adjacent to t2
            // only once!
            geo_debug_assert(tet_adjacent(t1,(result+1)%4) != t2);
            geo_debug_assert(tet_adjacent(t1,(result+2)%4) != t2);
            geo_debug_assert(tet_adjacent(t1,(result+3)%4) != t2);
            return result;
        }

        /**
         *  Gets the local facet index incident to an
         * oriented halfedge.
         * \param[in] t index of the tetrahedron
         * \param[in] v1 global index of the first extremity
         * \param[in] v2 global index of the second extremity
         * \return the local index of the facet incident to
         *  the oriented edge \p v1, \p v2.
         */
        index_t get_facet_by_halfedge(index_t t, index_t v1, index_t v2) const {
            geo_debug_assert(t < max_t());
            geo_debug_assert(v1 != v2);
            //   Find local index of v1 and v2 in tetrahedron t
            const index_t* T = &(cell_to_v_store_[4 * t]);

            index_t lv1, lv2;
            lv1 = find_4(T,v1);
            lv2 = find_4(T,v2);
            geo_debug_assert(lv1 != lv2);
            return index_t(halfedge_facet_[lv1][lv2]);
        }


        /**
         *  Gets the local facet indices incident to an
         * oriented halfedge.
         * \param[in] t index of the tetrahedron
         * \param[in] v1 global index of the first extremity
         * \param[in] v2 global index of the second extremity
         * \param[out] f12 the local index of the facet
         *  indicent to the halfedge [v1,v2]
         * \param[out] f21 the local index of the facet
         *  indicent to the halfedge [v2,v1]
         */
        void get_facets_by_halfedge(
            index_t t, index_t v1, index_t v2,
            index_t& f12, index_t& f21
        ) const {
            geo_debug_assert(t < max_t());
            geo_debug_assert(v1 != v2);

            //   Find local index of v1 and v2 in tetrahedron t
            // The following expression is 10% faster than using
            // if() statements (multiply by boolean result of test).
            // Thank to Laurent Alonso for this idea.
            const index_t* T = &(cell_to_v_store_[4 * t]);

            index_t lv1,lv2;

            lv1 = (T[1] == v1) | ((T[2] == v1) * 2) | ((T[3] == v1) * 3);
            lv2 = (T[1] == v2) | ((T[2] == v2) * 2) | ((T[3] == v2) * 3);
            geo_debug_assert(lv1 != 0 || T[0] == v1);
            geo_debug_assert(lv2 != 0 || T[0] == v2);
            geo_debug_assert(lv1 != lv2);

            f12 = index_t(halfedge_facet_[lv1][lv2]);
            f21 = index_t(halfedge_facet_[lv2][lv1]);
        }

        /**
         * \brief Symbolic value of the cell_next_ field
         *  that indicates the end of list in a linked
         *  list of tetrahedra.
         */
        static constexpr index_t END_OF_LIST = NO_INDEX;

        /**
         * \brief Symbolic value of the cell_next_ field
         *  for a tetrahedron that is not in a list.
         */
        static constexpr index_t NOT_IN_LIST = index_t(-2);

        /**
         * \brief Symbolic value for t2v_[] indicating a deleted tetrahedron.
         */
	static constexpr index_t VERTEX_OF_DELETED_TET = index_t(-2);

        /**
         * \brief Gets the number of vertices.
         * \return the number of vertices in this Delaunay
         */
        index_t nb_vertices() const {
            return nb_vertices_;
        }

        /**
         * \brief Tests whether a tetrahedron belongs to a linked
         *  list.
         * \details Tetrahedra can be linked, it is used to manage
         *  both the free list that recycles deleted tetrahedra,
         *  the conflict region and the list of newly created
         *  tetrahedra.
         * \param[in] t the index of the tetrahedron
         * \retval true if tetrahedron \p t belongs to a linked list
         * \retval false otherwise
         */
        bool tet_is_in_list(index_t t) const {
            geo_debug_assert(t < max_t());
            return (cell_next_[t] != NOT_IN_LIST);
        }

        /**
         * \brief Gets the index of a successor of a tetrahedron.
         * \details Tetrahedra can be linked, it is used to manage
         *  both the free list that recycles deleted tetrahedra.
         * \param[in] t the index of the tetrahedron
         * \retval END_OF_LIST if the end of the list is reached
         * \retval the index of the successor of
         *   tetrahedron \t otherwise
         * \pre tet_is_in_list(t)
         */
        index_t tet_next(index_t t) const {
            geo_debug_assert(t < max_t());
            geo_debug_assert(tet_is_in_list(t));
            return cell_next_[t];
        }


        index_t tet_thread(index_t t) const {
            geo_debug_assert(t < max_t());
            return cell_status_.cell_thread(t);
        }

        /**
         * \brief Adds a tetrahedron to a linked list.
         * \details Tetrahedra can be linked, it is used to manage
         *  the free list that recycles deleted tetrahedra.
         * \param[in] t the index of the tetrahedron
         * \param[in,out] first first item of the list or END_OF_LIST if
         *  the list is empty
         * \param[in,out] last last item of the list or END_OF_LIST if
         *  the list is empty
         */
        void add_tet_to_list(index_t t, index_t& first, index_t& last) {
            geo_debug_assert(t < max_t());
            geo_debug_assert(!tet_is_in_list(t));
            geo_debug_assert(owns_tet(t));
            if(last == END_OF_LIST) {
                geo_debug_assert(first == END_OF_LIST);
                first = last = t;
                cell_next_[t] = END_OF_LIST;
            } else {
                cell_next_[t] = first;
                first = t;
            }
        }

        /**
         * \brief Removes a tetrahedron from the linked list it
         *  belongs to.
         * \details Tetrahedra can be linked, it is used to manage
         *  the free list that recycles deleted tetrahedra.
         * \param[in] t the index of the tetrahedron
         */
        void remove_tet_from_list(index_t t) {
            geo_debug_assert(t < max_t());
            geo_debug_assert(tet_is_in_list(t));
            geo_debug_assert(owns_tet(t));
            cell_next_[t] = NOT_IN_LIST;
        }


        /**
         * \brief Creates a new tetrahedron.
         * \details Uses either a tetrahedron recycled
         *  from the free list, or creates a new one by
         *  expanding the two indices arrays.
         * \return the index of the newly created tetrahedron
         */
        index_t new_tetrahedron() {
            // If the memory pool is full, then we expand it.
            // This cannot be done when running multiple threads.
            if(first_free_ == END_OF_LIST) {
                geo_debug_assert(!Process::is_running_threads());

                if(
                    master_->cell_to_v_store_.size() ==
                    master_->cell_to_v_store_.capacity()
                ) {
                    master_->nb_reallocations_++;
                }

                master_->cell_to_v_store_.resize(
                    master_->cell_to_v_store_.size() + 4, NO_INDEX
                );
                master_->cell_to_cell_store_.resize(
                    master_->cell_to_cell_store_.size() + 4, NO_INDEX
                );
                master_->cell_next_.push_back(END_OF_LIST);
                master_->cell_status_.grow();
                ++nb_free_;
                ++max_t_;
                first_free_ = master_->cell_status_.size() - 1;
            }

            acquire_and_mark_tet_as_created(first_free_);
            index_t result = first_free_;

            first_free_ = tet_next(first_free_);
            remove_tet_from_list(result);

            cell_to_cell_store_[4 * result] = NO_INDEX;
            cell_to_cell_store_[4 * result + 1] = NO_INDEX;
            cell_to_cell_store_[4 * result + 2] = NO_INDEX;
            cell_to_cell_store_[4 * result + 3] = NO_INDEX;

            used_tets_end_ = std::max(used_tets_end_, result+1);

            --nb_free_;
            return result;
        }

        /**
         * \brief Creates a new tetrahedron.
         * \details Sets the vertices. Adjacent tetrahedra index are
         *  left uninitialized. Uses either a tetrahedron recycled
         *  from the free list, or creates a new one by
         *  expanding the two indices arrays.
         * \param[in] v1 index of the first vertex
         * \param[in] v2 index of the second vertex
         * \param[in] v3 index of the third vertex
         * \param[in] v4 index of the fourth vertex
         * \return the index of the newly created tetrahedron
         */
        index_t new_tetrahedron(
            index_t v1, index_t v2,
            index_t v3, index_t v4
        ) {
            index_t result = new_tetrahedron();
            cell_to_v_store_[4 * result] = v1;
            cell_to_v_store_[4 * result + 1] = v2;
            cell_to_v_store_[4 * result + 2] = v3;
            cell_to_v_store_[4 * result + 3] = v4;
            return result;
        }

        /**
         * \brief Finds the index of an integer in an array of four integers.
         * \param[in] T a const pointer to an array of four integers
         * \param[in] v the integer to retrieve in \p T
         * \return the index (0,1,2 or 3) of \p v in \p T
         * \pre The four entries of \p T are different and one of them is
         *  equal to \p v.
         */
        static index_t find_4(const index_t* T, index_t v) {
            // The following expression is 10% faster than using
            // if() statements. This uses the C++ norm, that
            // ensures that the 'true' boolean value converted to
            // an int is always 1. With most compilers, this avoids
            // generating branching instructions.
            // Thank to Laurent Alonso for this idea.
            // Note: Laurent also has this version:
            //    (T[0] != v)+(T[2]==v)+2*(T[3]==v)
            // that avoids a *3 multiply, but it is not faster in
            // practice.
            index_t result = index_t(
                (T[1] == v) | ((T[2] == v) * 2) | ((T[3] == v) * 3)
            );
            // Sanity check, important if it was T[0], not explicitly
            // tested (detects input that does not meet the precondition).
            geo_debug_assert(T[result] == v);
            return result;
        }


        /**
         * \brief Finds the index of an integer in an array of four integers.
         * \param[in] T a const pointer to an array of four integers
         * \param[in] v the real vertex to retrieve in \p T
         * \return the index (0,1,2 or 3) of \p v in \p T
         * \pre The four entries of \p T are different and one of them is
         *  equal to \p v.
         */
        index_t find_4_periodic(const index_t* T, index_t v) const {

            // v needs to be a real vertex.
            geo_debug_assert(periodic_vertex_instance(v) == 0);

            geo_debug_assert(
                T[0] != NO_INDEX && T[1] != NO_INDEX &&
		T[2] != NO_INDEX && T[3] != NO_INDEX
            );

            // The following expression is 10% faster than using
            // if() statements. This uses the C++ norm, that
            // ensures that the 'true' boolean value converted to
            // an int is always 1. With most compilers, this avoids
            // generating branching instructions.
            // Thank to Laurent Alonso for this idea.
            // Note: Laurent also has this version:
            //    (T[0] != v)+(T[2]==v)+2*(T[3]==v)
            // that avoids a *3 multiply, but it is not faster in
            // practice.
            index_t result = index_t(
                ( periodic_vertex_real(index_t(T[1])) == v)      |
                ((periodic_vertex_real(index_t(T[2])) == v) * 2) |
                ((periodic_vertex_real(index_t(T[3])) == v) * 3)
            );

            // Sanity check, important if it was T[0], not explicitly
            // tested (detects input that does not meet the precondition).
            geo_debug_assert(periodic_vertex_real(index_t(T[result])) == v);
            return result;
        }


        /**
         * \brief Wakes up all the threads that are waiting for
         *  this thread.
         */
        void send_event() {
            cond_.notify_all();
        }

        /**
         * \brief Waits for a thread.
         * \details Sleeps until thread \p t calls send_event().
         * \param[in] t index of the thread
         * \pre t < nb_threads()
         */
        void wait_for_event(index_t t) {
            // Fixed by Hiep Vu: enlarged critical section (contains
            // now the test (!thrd->finished)
            PeriodicDelaunay3dThread* thrd = thread(t);
            // RAII: ctor locks, dtor unlocks
            std::unique_lock<std::mutex> L(thrd->mutex_);
            if(!thrd->finished_) {
                thrd->cond_.wait(L);
            }
        }

        /****** iterative stellate_conflict_zone *****************/

        /**
         * \brief Used to represent the stack in the
         *  (de-recursified) stellate_conflict_zone_iterative()
         *  function.
         */
        class StellateConflictStack {
        public:

            /**
             * \brief Pushes a new frame onto the stack.
             * \details This also creates the local variables (they are
             *  left uninitialized).
             * \param[in] t1 index of a tetrahedron on the border of
             *  the conflict zone
             * \param[in] t1fbord index of the facet of \p t1 that is
             *  on the border of the conflict zone
             * \param[in] t1fprev index of the facet of \p t1 that we
             *  come from, or NO_INDEX if \p t1 is the first tetrahedron
             */
            void push(index_t t1, index_t t1fbord, index_t t1fprev) {
                store_.resize(store_.size()+1);
                top().t1 = t1;
                top().t1fbord = Numeric::uint8(t1fbord);
                top().t1fprev = Numeric::uint8(t1fprev);
            }

            /**
             * \brief Saves local variables into the current stack frame.
             * \param[in] new_t the index of the newly created tetrahedron
             * \param[in] t1ft2 the facet of t1 that is adjacent to t2
             * \param[in] t2ft1 the facet of t2 that is adjacent to t1
             */
            void save_locals(index_t new_t, index_t t1ft2, index_t t2ft1) {
                geo_debug_assert(!empty());
                top().new_t = new_t;
                top().t1ft2 = Numeric::uint8(t1ft2);
                top().t2ft1 = Numeric::uint8(t2ft1);
            }

            /**
             * \brief Gets the parameters from the current stack frame.
             * \param[out] t1 index of a tetrahedron on the border of
             *  the conflict zone
             * \param[out] t1fbord index of the facet of \p t1 that is
             *  on the border of the conflict zone
             * \param[out] t1fprev index of the facet of \p t1 that we
             *  come from, or NO_INDEX if \p t1 is the first tetrahedron
             */
            void get_parameters(
                index_t& t1, index_t& t1fbord, index_t& t1fprev
            ) const {
                geo_debug_assert(!empty());
                t1      = top().t1;
                t1fbord = index_t(top().t1fbord);
                t1fprev = index_t(top().t1fprev);
            }


            /**
             * \brief Gets the local variables from the current stack frame.
             * \param[out] new_t the index of the newly created tetrahedron
             * \param[out] t1ft2 the facet of t1 that is adjacent to t2
             * \param[out] t2ft1 the facet of t2 that is adjacent to t1
             */
            void get_locals(
                index_t& new_t, index_t& t1ft2, index_t& t2ft1
            ) const {
                geo_debug_assert(!empty());
                new_t = top().new_t;
                t1ft2 = index_t(top().t1ft2);
                t2ft1 = index_t(top().t2ft1);
            }

            /**
             * \brief Pops a stack frame.
             */
            void pop() {
                geo_debug_assert(!empty());
                store_.pop_back();
            }

            /**
             * \brief Tests whether the stack is empty.
             * \retval true if the stack is empty
             * \retval false otherwise
             */
            bool empty() const {
                return store_.empty();
            }

        private:

            /**
             * \brief The parameters and local
             *  variables stored in a stack frame.
             */
            struct Frame {
                // Parameters
                index_t t1;
                index_t new_t;
                Numeric::uint8 t1fbord ;

                // Local variables
                Numeric::uint8 t1fprev ;
                Numeric::uint8 t1ft2   ;
                Numeric::uint8 t2ft1   ;
            };

            /**
             * \brief Gets the top of the stack.
             * \return a modifiable reference to the Frame on
             *  the top of the stack
             * \pre !empty()
             */
            Frame& top() {
                geo_debug_assert(!empty());
                return *store_.rbegin();
            }

            /**
             * \brief Gets the top of the stack.
             * \return a const reference to the Frame on
             *  the top of the stack
             * \pre !empty()
             */
            const Frame& top() const {
                geo_debug_assert(!empty());
                return *store_.rbegin();
            }

            std::vector<Frame> store_;
        };

        /**
         * \brief Creates a star of tetrahedra filling the conflict
         *  zone.
         * \details For each tetrahedron facet on the border of the
         *  conflict zone, a new tetrahedron is created, resting on
         *  the facet and incident to vertex \p v. The function is
         *  called recursively until the entire conflict zone is filled.
         * \param[in] v the index of the point to be inserted
         * \param[in] t1 index of a tetrahedron on the border
         *  of the conflict zone.
         * \param[in] t1fbord index of the facet along which \p t_bndry
         *  is incident to the border of the conflict zone
         * \param[in] t1fprev the facet of \p t_bndry connected to the
         *  tetrahedron that \p t_bndry was reached from, or NO_INDEX
         *  if it is the first tetrahedron.
         * \return the index of one the newly created tetrahedron
         */
        index_t stellate_conflict_zone_iterative(
            index_t v, index_t t1, index_t t1fbord,
            index_t t1fprev = NO_INDEX
        ) {
            //   This function is de-recursified because some degenerate
            // inputs can cause stack overflow (system stack is limited to
            // a few megs). For instance, it can happen when a large number
            // of points are on the same sphere exactly.

            //   To de-recursify, it uses class StellateConflictStack
            // that emulates system's stack for storing functions's
            // parameters and local variables in all the nested stack
            // frames.

            S2_.push(t1, t1fbord, t1fprev);

            index_t new_t;   // the newly created tetrahedron.

            index_t t1ft2;   // traverses the 4 facets of t1.

            index_t t2;      // the tetrahedron on the border of
                             // the conflict zone that shares an
                             // edge with t1 along t1ft2.

            index_t t2fbord; // the facet of t2 on the border of
                             // the conflict zone.

            index_t t2ft1;   // the facet of t2 that is incident to t1.

        entry_point:
            S2_.get_parameters(t1, t1fbord, t1fprev);


            geo_debug_assert(owns_tet(t1));
            geo_debug_assert(tet_adjacent(t1,t1fbord) != NO_INDEX);
            geo_debug_assert(owns_tet(tet_adjacent(t1,t1fbord)));
            geo_debug_assert(tet_is_marked_as_conflict(t1));
            geo_debug_assert(
                !tet_is_marked_as_conflict(tet_adjacent(t1,t1fbord))
            );

            // Create new tetrahedron with same vertices as t_bndry

            new_t = new_tetrahedron(
                tet_vertex(t1,0),
                tet_vertex(t1,1),
                tet_vertex(t1,2),
                tet_vertex(t1,3)
            );

            index_t tbord = tet_adjacent(t1,t1fbord);

            // We generate the tetrahedron with the three vertices
            // of the tet outside the conflict zone and the newly
            // created vertex in the local frame of the tet outside
            // the conflict zone.

            // Replace in new_t the vertex opposite to t1fbord with v
            set_tet_vertex(new_t, t1fbord, v);

            {
                // Connect new_t with t1's neighbor across t1fbord
                set_tet_adjacent(new_t, t1fbord, tbord);
                set_tet_adjacent(tbord, find_tet_adjacent(tbord,t1), new_t);
            }

            //  Lookup new_t's neighbors across its three other
            // facets and connect them
            for(t1ft2=0; t1ft2<4; ++t1ft2) {

                if(t1ft2 == t1fprev || tet_adjacent(new_t,t1ft2) != NO_INDEX) {
                    continue;
                }

                // Get t1's neighbor along the border of the conflict zone
                if(!get_neighbor_along_conflict_zone_border(
                       t1,t1fbord,t1ft2, t2,t2fbord,t2ft1
                   )) {
                    //   If t1's neighbor is not a new tetrahedron,
                    // create a new tetrahedron through a recursive call.
                    S2_.save_locals(new_t, t1ft2, t2ft1);
                    S2_.push(t2, t2fbord, t2ft1);
                    goto entry_point;

                return_point:
                    // This is the return value of the called function.
                    index_t result = new_t;
                    S2_.pop();

                    // Special case: we were in the outermost frame,
                    // then we (truly) return from the function.
                    if(S2_.empty()) {
                        return result;
                    }

                    S2_.get_parameters(t1, t1fbord, t1fprev);
                    S2_.get_locals(new_t, t1ft2, t2ft1);
                    t2 = result;
                }

                set_tet_adjacent(t2, t2ft1, new_t);
                set_tet_adjacent(new_t, t1ft2, t2);
            }

            // Except for the initial call (see "Special case" above),
            // the nested calls all come from the same location,
            // thus there is only one possible return point
            // (no need to push any return address).
            goto return_point;
        }

        /**
         * \brief Finds the neighbor of a tetrahedron on the border of the
         *  conflict zone.
         * \details This function is used by stellate_conflict_zone_iterative()
         * \param[in] t1 a tetrahedron on the border of the conflict zone
         * \param[in] t1fborder the local facet index of \p t1 along which it
         *  is on the border of the conflict zone
         * \param[in] t1ft2 the local facet index of \p t1 that will be
         *  traversed
         * \param[out] t2 a tetrahedron on the border of the conflict zone,
         *  with an edge common to facets \p t1fborder and \p t1ft2 of
         *  tetrahedron \p t1
         * \param[out] t2fborder the local facet index of \p t2 along which it
         *  is on the border of the conflict zone
         * \param[out] t2ft1 the local index of the facet of \p t2 that has a
         *  common edge with facets \p t1fborder and \p t1ft2 of tetrahedron
         *  \p t1
         * \retval true if \p t2 is a newly created tetrahedron
         * \retval false if \p t2 is an old tetrahedron in conflict
         */
        bool get_neighbor_along_conflict_zone_border(
            index_t t1,
            index_t t1fborder,
            index_t t1ft2,
            index_t& t2,
            index_t& t2fborder,
            index_t& t2ft1
        ) const {

            // Note: this function is a bit long for an inline function,
            // but I observed a (modest) performance gain doing so.

            //   Find two vertices that are both on facets new_f and f1
            //  (the edge around which we are turning)
            //  This uses duality as follows:
            //  Primal form (not used here):
            //    halfedge_facet_[v1][v2] returns a facet that is incident
            //    to both v1 and v2.
            //  Dual form (used here):
            //    halfedge_facet_[f1][f2] returns a vertex that both
            //    f1 and f2 are incident to.
            index_t ev1 =
                tet_vertex(t1, index_t(halfedge_facet_[t1ft2][t1fborder]));
            index_t ev2 =
                tet_vertex(t1, index_t(halfedge_facet_[t1fborder][t1ft2]));

            //   Turn around edge [ev1,ev2] inside the conflict zone
            // until we reach again the boundary of the conflict zone.
            // Traversing inside the conflict zone is faster (as compared
            // to outside) since it traverses a smaller number of tets.
            index_t cur_t = t1;
            index_t cur_f = t1ft2;
            index_t next_t = tet_adjacent(cur_t,cur_f);
            while(tet_is_marked_as_conflict(next_t)) {
                geo_debug_assert(next_t != t1);
                cur_t = next_t;
                cur_f = get_facet_by_halfedge(cur_t,ev1,ev2);
                next_t = tet_adjacent(cur_t, cur_f);
            }

            //  At this point, cur_t is in conflict zone and
            // next_t is outside the conflict zone.
            index_t f12,f21;
            get_facets_by_halfedge(next_t, ev1, ev2, f12, f21);
            t2 = tet_adjacent(next_t,f21);
            index_t v_neigh_opposite = tet_vertex(next_t,f12);
            t2ft1 = find_tet_vertex(t2, v_neigh_opposite);
            t2fborder = cur_f;

            //  Test whether the found neighboring tet was created
            //  (then return true) or is an old tet in conflict
            //  (then return false).
            return(t2 != cur_t);
        }

        /**
         * \brief Used by the (de-recursified)
         *   stellate_conflict_zone_iterative() function.
         */
        StellateConflictStack S2_;

        /*************************** debugging ************************/

        /**
         * \brief For debugging purposes, displays a tetrahedron adjacency.
         * \param[in] t index of the tetrahedron to display.
         * \param[in] lf local index (0,1,2 or 3) of the tetrahedron
         *  facet adjacenty to display.
         */
        void show_tet_adjacent(index_t t, index_t lf) const {
            index_t adj = tet_adjacent(t, lf);
            if(adj != NO_INDEX) {
                std::cerr << (tet_is_in_list(adj) ? '*' : ' ');
            }
            std::cerr << adj;
            std::cerr << ' ';
        }


        /**
         * \brief For debugging purposes, displays a tetrahedron.
         * \param[in] t index of the tetrahedron to display.
         */
        void show_tet(index_t t) const {
            std::cerr << "tet"
                      << (tet_is_in_list(t) ? '*' : ' ')
                      << t
                      << ", v=["
                      << tet_vertex(t, 0)
                      << ' '
                      << tet_vertex(t, 1)
                      << ' '
                      << tet_vertex(t, 2)
                      << ' '
                      << tet_vertex(t, 3)
                      << "]  adj=[";
            show_tet_adjacent(t, 0);
            show_tet_adjacent(t, 1);
            show_tet_adjacent(t, 2);
            show_tet_adjacent(t, 3);
            std::cerr << "] ";

            for(index_t f = 0; f < 4; ++f) {
                std::cerr << 'f' << f << ':';
                for(index_t v = 0; v < 3; ++v) {
                    std::cerr << tet_vertex(t, tet_facet_vertex(f,v))
                              << ',';
                }
                std::cerr << ' ';
            }
            std::cerr << std::endl;
        }

    public:

        /**
         * \brief For debugging purposes, tests some combinatorial properties.
         */
        void check_combinatorics(bool verbose) const {
            if(verbose) {
                std::cerr << std::endl;
            }
            bool ok = true;
            std::vector<bool> v_has_tet(nb_vertices(), false);
            for(index_t t = 0; t < max_t(); ++t) {
                if(tet_is_free(t)) {
                    if(verbose) {
                        std::cerr << "-Deleted tet: ";
                        show_tet(t);
                    }
                } else {
                    if(verbose) {
                        std::cerr << "Checking tet: ";
                        show_tet(t);
                    }
                    for(index_t lf = 0; lf < 4; ++lf) {
                        if(tet_adjacent(t, lf) == NO_INDEX) {
                            std::cerr << lf << ":Missing adjacent tet"
                                      << std::endl;
                            ok = false;
                        } else if(tet_adjacent(t, lf) == t) {
                            std::cerr << lf << ":Tet is adjacent to itself"
                                      << std::endl;
                            ok = false;
                        } else {
                            index_t t2 = tet_adjacent(t, lf);
                            bool found = false;
                            for(index_t lf2 = 0; lf2 < 4; ++lf2) {
                                if(tet_adjacent(t2, lf2) == t) {
                                    found = true;
                                }
                            }
                            if(!found) {
                                std::cerr
                                    << lf
                                    << ":Adjacent link is not bidirectional"
                                    << std::endl;
                                ok = false;
                            }
                        }
                    }
                    index_t nb_infinite = 0;
                    for(index_t lv = 0; lv < 4; ++lv) {
                        if(tet_vertex(t, lv) == NO_INDEX) {
                            ++nb_infinite;
                        }
                    }
                    if(nb_infinite > 1) {
                        ok = false;
                        std::cerr << "More than one infinite vertex"
                                  << std::endl;
                    }
                }
                for(index_t lv = 0; lv < 4; ++lv) {
                    index_t v = tet_vertex(t, lv);
                    if(v != NO_INDEX && v != NOT_IN_LIST) {
                        v_has_tet[periodic_vertex_real(v)] = true;
                    }
                }
            }

            index_t nb_v = nb_vertices();
            for(index_t v = 0; v < nb_v; ++v) {
                if(!v_has_tet[v]) {
                    if(verbose) {
                        std::cerr << "Vertex " << v
                                  << " is isolated (duplicated ?)" << std::endl;
                    }
                }
            }
            geo_assert(ok);
            if(verbose) {
                std::cerr << std::endl;
            }
            std::cerr << std::endl << "Delaunay Combi OK" << std::endl;
        }


        /**
         * \brief For debugging purposes, test some geometrical properties.
         */
        void check_geometry(bool verbose) const {
            bool ok = true;
            for(index_t t = 0; t < max_t(); ++t) {
                if(!tet_is_free(t)) {
                    index_t v0 = tet_vertex(t, 0);
                    index_t v1 = tet_vertex(t, 1);
                    index_t v2 = tet_vertex(t, 2);
                    index_t v3 = tet_vertex(t, 3);
                    for(index_t v = 0; v < nb_vertices(); ++v) {
                        vec4 p = lifted_vertex(v);
                        if(v == v0 || v == v1 || v == v2 || v == v3) {
                            continue;
                        }
                        if(tet_is_in_conflict(t, v, p)) {
                            ok = false;
                            if(verbose) {
                                std::cerr << "Tet " << t <<
                                    " is in conflict with vertex " << v
                                          << std::endl;

                                std::cerr << "  offending tet: ";
                                show_tet(t);
                            }
                        }
                    }
                }
            }
            geo_assert(ok);
            std::cerr << std::endl << "Delaunay Geo OK" << std::endl;
        }

    private:
        PeriodicDelaunay3d* master_;
        bool periodic_;
        vec3 period_;
        index_t nb_vertices_;
        const double* vertices_;
        const double* weights_;
        index_t* reorder_;
        index_t dimension_;
	index_t pool_begin_;
	index_t pool_end_;
        index_t max_t_;
        index_t used_tets_end_;

        vector<index_t>& cell_to_v_store_;
        vector<index_t>& cell_to_cell_store_;
        vector<index_t>& cell_next_;
        CellStatusArray& cell_status_;

        index_t first_free_;
        index_t nb_free_;
        bool memory_overflow_;

	/** \brief used by find_conflict_zone_iterative() */
        struct SFrame {

            SFrame() {
            }

            SFrame(
                index_t t_in,
                index_t v_in,
                const vec4& p_in
            ) :
                t(t_in),
                v(v_in),
                p(p_in) {
            }

            SFrame(
                const SFrame& rhs
            ):
                t(rhs.t),
                v(rhs.v),
                p(rhs.p) {
            }

            SFrame& operator=(const SFrame& rhs) {
                t = rhs.t;
                v = rhs.v;
                p = rhs.p;
                return *this;
            }

            index_t t;
            index_t v;
            vec4 p;
        };

        vector<SFrame> S_;
        index_t nb_tets_to_create_;
        index_t t_boundary_; // index of a tet,facet on the bndry
        index_t f_boundary_; // of the conflict zone.

        bool direction_;
        index_t work_begin_;
        index_t work_rbegin_;
        index_t b_hint_;
        index_t e_hint_;
        bool finished_;

        //  Whenever acquire_tet() is unsuccessful, contains
        // the index of the thread that was interfering
        // (shifted to the left by 1 !!)
	CellStatusArray::thread_index_t interfering_thread_;

#ifdef GEO_DEBUG
        index_t nb_acquired_tets_;
#endif

        vector<index_t> tets_to_delete_;
        vector<index_t> tets_to_release_;

        index_t nb_rollbacks_;
        index_t nb_failed_locate_;

        std::condition_variable cond_;
        std::mutex mutex_;

        bool has_empty_cells_;

        /**
         * \brief Gives the indexing of tetrahedron facet
         *  vertices.
         * \details tet_facet_vertex[lf][lv] gives the
         *  local vertex index (in 0,1,2,3) from a
         *  local facet index lf (in 0,1,2,3) and a
         *  local vertex index within the facet (in 0,1,2).
         */
        static char tet_facet_vertex_[4][3];

        /**
         * \brief Gives a local facet index by
         *  halfedge extremities local indices.
         */
        static char halfedge_facet_[4][4];

        /**
         * \brief Optimized representation of triangles on
         *  the border of the cavity, for fast generation of
         *  tetrahedra.
         */
        Cavity cavity_;

        /**
         * \brief Statistics for locate()
         */
        index_t nb_traversed_tets_;
    };


    char PeriodicDelaunay3dThread::halfedge_facet_[4][4] = {
        {4, 2, 3, 1},
        {3, 4, 0, 2},
        {1, 3, 4, 0},
        {2, 0, 1, 4}
    };

    // tet facet vertex is such that the tetrahedron
    // formed with:
    //  vertex lv
    //  tet_facet_vertex[lv][0]
    //  tet_facet_vertex[lv][1]
    //  tet_facet_vertex[lv][2]
    // has the same orientation as the original tetrahedron for
    // any vertex lv.

    char PeriodicDelaunay3dThread::tet_facet_vertex_[4][3] = {
        {1, 2, 3},
        {0, 3, 2},
        {3, 0, 1},
        {1, 0, 2}
    };


    /*************************************************************************/

    PeriodicDelaunay3d::PeriodicDelaunay3d(
        bool periodic, double period
    ) :
        Delaunay(3),
        periodic_(periodic),
        period_(period,period,period),
        weights_(nullptr),
        update_periodic_v_to_cell_(false),
        has_empty_cells_(false),
        nb_reallocations_(0),
        convex_cell_exact_predicates_(true)
    {
        debug_mode_ = CmdLine::get_arg_bool("dbg:delaunay");
        verbose_debug_mode_ = CmdLine::get_arg_bool("dbg:delaunay_verbose");
        debug_mode_ = (debug_mode_ || verbose_debug_mode_);
	benchmark_mode_ = CmdLine::get_arg_bool("dbg:delaunay_benchmark");
        detailed_benchmark_mode_ =
	    CmdLine::get_arg_bool("dbg:detailed_delaunay_benchmark");
        nb_vertices_non_periodic_ = 0;
        delaunay_citations();
    }

    PeriodicDelaunay3d::PeriodicDelaunay3d(
        const vec3& period
    ) :
        Delaunay(3),
        periodic_(true),
        period_(period),
        weights_(nullptr),
        update_periodic_v_to_cell_(false),
        has_empty_cells_(false),
        nb_reallocations_(0),
        convex_cell_exact_predicates_(true)
    {
        debug_mode_ = CmdLine::get_arg_bool("dbg:delaunay");
        verbose_debug_mode_ = CmdLine::get_arg_bool("dbg:delaunay_verbose");
        debug_mode_ = (debug_mode_ || verbose_debug_mode_);
        benchmark_mode_ = CmdLine::get_arg_bool("dbg:delaunay_benchmark");
        detailed_benchmark_mode_ =
	    CmdLine::get_arg_bool("dbg:detailed_delaunay_benchmark");
        nb_vertices_non_periodic_ = 0;
        delaunay_citations();
    }

    void PeriodicDelaunay3d::set_vertices(
        index_t nb_vertices, const double* vertices
    ) {
        has_empty_cells_ = false;

#ifndef GARGANTUA
        {
            Numeric::uint64 expected_max_index =
                Numeric::uint64(nb_vertices) * 7 * 4;
            if(periodic_) {
                expected_max_index *= 2;
            }
            if(expected_max_index > Numeric::uint64(INT32_MAX)) {
                Logger::err("OTM") << "indices will overflow" << std::endl;
                Logger::err("OTM")
                    << "recompile with -DGARGANTUA "
                    << "to activate 64bit indices" << std::endl;
                exit(0);
            }
        }
#endif

        if(periodic_) {
            PCK::set_SOS_mode(PCK::SOS_LEXICO);
        }

        Stopwatch W("BRIO", benchmark_mode_);
        nb_vertices_non_periodic_ = nb_vertices;

        Delaunay::set_vertices(nb_vertices, vertices);
        // Reorder the points
        if(do_reorder_) {
            compute_BRIO_order(
                nb_vertices, vertex_ptr(0), reorder_,
                3, dimension(),
                64, 0.125,
                &levels_
            );
        } else {
            reorder_.resize(nb_vertices);
            for(index_t i = 0; i < nb_vertices; ++i) {
                reorder_[i] = i;
            }
            geo_debug_assert(levels_[0] == 0);
            geo_debug_assert(levels_[levels_.size()-1] == nb_vertices);
        }
    }

    void PeriodicDelaunay3d::set_weights(const double* weights) {
        has_empty_cells_ = false;
        weights_ = weights;
    }

    void PeriodicDelaunay3d::compute() {

	stats_.reset();

	Stopwatch W_tot("total",false);

        has_empty_cells_ = false;

        if(periodic_) {
            reorder_.resize(nb_vertices_non_periodic_);
        }

	{
	    Stopwatch W("DelInternal", detailed_benchmark_mode_);

	    index_t expected_tetra = nb_vertices() * 7;

	    // Everything is allocated here, including for handling
	    // periodic boundary conditions, much later. We need to
	    // allocate sufficient space to have good chances of
	    // inserting most of the additional points in parallel
	    // (in insert_with_BRIO())

	    if(periodic_) {
		expected_tetra = index_t(double(expected_tetra)* 1.2);
	    }

	    // Allocate the tetrahedra
	    cell_to_v_store_.assign(expected_tetra * 4, NO_INDEX);
	    cell_to_cell_store_.assign(expected_tetra * 4, NO_INDEX);
	    cell_next_.assign(expected_tetra,NO_INDEX);
	    cell_status_.resize(expected_tetra);

	    // Create the threads
	    // The maximum number of threads is limited by the number
	    // of bits used by cell_status_ (see delaunay_sync.h)
	    index_t nb_threads = std::min(
		Process::maximum_concurrent_threads(),
		CellStatusArray::MAX_THREADS
	    );
	    index_t pool_size = expected_tetra / nb_threads;
	    if (pool_size == 0) {
		// There are more threads than expected_tetra
		pool_size = 1;
		nb_threads = expected_tetra;
	    }
	    index_t pool_begin = 0;
	    threads_.clear();
	    for(index_t t=0; t<nb_threads; ++t) {
		index_t pool_end =
		    (t == nb_threads - 1) ? expected_tetra
		                          : pool_begin + pool_size;
		threads_.push_back(
		    new PeriodicDelaunay3dThread(this, pool_begin, pool_end)
		);
		pool_begin = pool_end;
	    }


	    // Create first tetrahedron and triangulate first set of points
	    // in sequential mode.

	    PeriodicDelaunay3dThread* thread0 = thread(0);
	    thread0->create_first_tetrahedron();
	    {
		Stopwatch Wmain("DelMain", detailed_benchmark_mode_);
		insert_vertices_with_BRIO("DelMain", levels_);
		stats_.phase_0_t_ = Wmain.elapsed_time();
	    }

	    if(has_empty_cells_) {
		return;
	    }

	    if(periodic_) {
		Stopwatch W12("DelPhaseI-II", detailed_benchmark_mode_);
		handle_periodic_boundaries();
	    }

	    if(has_empty_cells_) {
		return;
	    }
	}

        if(debug_mode_) {
            for(index_t i=0; i<threads_.size(); ++i) {
                std::cerr << i << " : " <<
                    static_cast<PeriodicDelaunay3dThread*>(threads_[i].get())
                    ->max_t() << std::endl;
            }

            thread(0)->check_combinatorics(verbose_debug_mode_);
            thread(0)->check_geometry(verbose_debug_mode_);
        }

	index_t nb_tets = 0;
	{
	    nb_tets = compress();

	    set_arrays(
		nb_tets,
		cell_to_v_store_.data(),
		cell_to_cell_store_.data()
	    );

	    // We need v_to_cell even if CICL is not stored.
	    if(!stores_cicl()) {
		update_v_to_cell();
	    }
	}

        if(periodic_) {
#ifdef GEO_DEBUG
            FOR(v, nb_vertices_non_periodic_) {
                index_t t = v_to_cell_[v];
                geo_assert(t == NO_INDEX || t < nb_tets);
            }
#endif
        }
	stats_.total_t_ = W_tot.elapsed_time();
	if(benchmark_mode_) {
	    Logger::out("Delaunay") << stats_.to_string() << std::endl;
	}
    }

    index_t PeriodicDelaunay3d::compress(bool shrink) {

	Stopwatch W("Compress",detailed_benchmark_mode_);

        //   Compress cell_to_v_store_ and cell_to_cell_store_
        // (remove free and virtual tetrahedra).
        //   Since cell_next_ is not used at this point,
        // we reuse it for storing the conversion array that
        // maps old tet indices to new tet indices
        // Note: tet_is_real() uses the previous value of
        // cell_next(), but we are processing indices
        // in increasing order and since old2new[t] is always
        // smaller or equal to t, we never overwrite a value
        // before needing it.


        PeriodicDelaunay3dThread* thread0 = thread(0);
        vector<index_t>& old2new = cell_next_;
        index_t nb_tets = 0;
        index_t nb_tets_to_delete = 0;

        {
	    // Classify tets in parallel (on very large data sets,
	    // >= 100M points, it gains a little bit of time)
	    parallel_for(0, thread0->max_t(), [&,this](index_t t) {
		if(
                    (keep_infinite_ && !thread0->tet_is_free(t)) ||
                    (periodic_ && thread0->tet_is_real_non_periodic(t)) ||
                    (!periodic_ && thread0->tet_is_real(t))
                ) {
		    old2new[t] = 0; // keep tetrahedron
		} else {
		    old2new[t] = NO_INDEX; // discard tetrahedron
		}
	    });

	    // Compress the tet array
            for(index_t t = 0; t < thread0->max_t(); ++t) {
                if(old2new[t] != NO_INDEX) {
                    if(t != nb_tets) {
                        Memory::copy(
                            &cell_to_v_store_[nb_tets * 4],
                            &cell_to_v_store_[t * 4],
                            4 * sizeof(index_t)
                        );
                        Memory::copy(
                            &cell_to_cell_store_[nb_tets * 4],
                            &cell_to_cell_store_[t * 4],
                            4 * sizeof(index_t)
                        );
                    }
                    old2new[t] = nb_tets;
                    ++nb_tets;
                } else {
                    ++nb_tets_to_delete;
                }
            }

            if(shrink) {
                cell_to_v_store_.resize(4 * nb_tets);
                cell_to_cell_store_.resize(4 * nb_tets);
            }

	    // Apply permutation to cell_to_cell_ array
	    parallel_for(0, 4*nb_tets, [this, &old2new](index_t i) {
                index_t t = cell_to_cell_store_[i];
                geo_debug_assert(t != NO_INDEX);
                t = old2new[t];
                // Note: t can be equal to -1 when a real tet is
                // adjacent to a virtual one (and this is how the
                // rest of Vorpaline expects to see tets on the
                // border).
                geo_debug_assert(!(keep_infinite_ && (t == NO_INDEX)));
                cell_to_cell_store_[i] = t;
            });
        }

        // In "keep_infinite" mode, we reorder the cells in such
        // a way that finite cells have indices [0..nb_finite_cells_-1]
        // and infinite cells have indices [nb_finite_cells_ .. nb_cells_-1]

        if(keep_infinite_) {
            nb_finite_cells_ = 0;
            index_t finite_ptr = 0;
            index_t infinite_ptr = nb_tets - 1;
            for(;;) {
                while(thread0->tet_is_finite(finite_ptr)) {
                    old2new[finite_ptr] = finite_ptr;
                    ++finite_ptr;
                    ++nb_finite_cells_;
                }
                while(!thread0->tet_is_finite(infinite_ptr)) {
                    old2new[infinite_ptr] = infinite_ptr;
                    --infinite_ptr;
                }
                if(finite_ptr > infinite_ptr) {
                    break;
                }
                old2new[finite_ptr] = infinite_ptr;
                old2new[infinite_ptr] = finite_ptr;
                ++nb_finite_cells_;
                for(index_t lf=0; lf<4; ++lf) {
                    std::swap(
                        cell_to_cell_store_[4*finite_ptr + lf],
                        cell_to_cell_store_[4*infinite_ptr + lf]
                    );
                }
                for(index_t lv=0; lv<4; ++lv) {
                    std::swap(
                        cell_to_v_store_[4*finite_ptr + lv],
                        cell_to_v_store_[4*infinite_ptr + lv]
                    );
                }
                ++finite_ptr;
                --infinite_ptr;
            }
	    parallel_for(0, 4*nb_tets, [this, &old2new](index_t i) {
                index_t t = cell_to_cell_store_[i];
                geo_debug_assert(t != NO_INDEX);
                t = old2new[t];
                geo_debug_assert(t != NO_INDEX);
                cell_to_cell_store_[i] = t;
            });
        }


        if(detailed_benchmark_mode_) {
            Logger::out("DelCompress")
                << "max tets " << thread0->max_t()
                << std::endl;

            Logger::out("DelCompress")
                << "Final number of tets " << nb_tets
                << std::endl;
            if(keep_infinite_) {
                Logger::out("DelCompress")
                    << "Removed " << nb_tets_to_delete
                    << " tets (free list)"
		    << " : "
		    << double(nb_tets_to_delete)*100.0/double(nb_tets) << "%"
		    << std::endl;
            } else {
                Logger::out("DelCompress")
                    << "Removed " << nb_tets_to_delete
                    << " tets (free list and infinite)"
		    << " : "
		    << double(nb_tets_to_delete)*100.0/double(nb_tets) << "%"
		    << std::endl;
            }
        }

        for(index_t t=0; t<nb_tets; ++t) {
            cell_next_[t] = PeriodicDelaunay3dThread::NOT_IN_LIST;
        }

	// Disconnect tets that were connected to infinite tets
        if(periodic_) {
	    parallel_for(0, 4*nb_tets, [this,nb_tets](index_t i) {
                if(cell_to_cell_store_[i] >= nb_tets) {
                    cell_to_cell_store_[i] = NO_INDEX;
                }
	    });
#ifdef GEO_DEBUG
            for(index_t i=0; i<4*nb_tets; ++i) {
                geo_debug_assert(cell_to_v_store_[i] != NO_INDEX);
            }
#endif
        }
        return nb_tets;
    }

    index_t PeriodicDelaunay3d::nearest_vertex(const double* p) const {
        // TODO
        return Delaunay::nearest_vertex(p);
    }

    void PeriodicDelaunay3d::set_BRIO_levels(const vector<index_t>& levels) {
        levels_ = levels;
    }

    void PeriodicDelaunay3d::update_v_to_cell() {
        geo_assert(!is_locked_);  // Not thread-safe
        is_locked_ = true;

	// Optimized version for large scale optimal transport,
	// can be removed (all cases treated)
	if(!update_periodic_v_to_cell_ && !keeps_infinite()) {
            v_to_cell_.assign(nb_vertices(), NO_INDEX);
	    parallel_for(0, nb_cells(), [this](index_t c) {
                for(index_t lv = 0; lv < 4; lv++) {
                    index_t v = cell_vertex(c, lv);
		    // discriminates both vertex at infinity (NO_INDEX)
		    // and VERTEX_OF_DELETED_TET (index_t(-2)).
                    if(v < nb_vertices_non_periodic_) {
                        v_to_cell_[v] = c;
		    }
		}
	    });
	    is_locked_ = false; // Do not forget to unlock !
	    return;
	}


        // Note: if keeps_infinite is set, then infinite vertex
        // tet chaining is at t2v_[nb_vertices].

        // Create periodic_v_to_cell_ structure in compressed row
        // storage format.

	// It was used in previous version for handling periodic boundary
	// conditions based on ConvexCell, it is no longer the case, new
	// code solely uses tetrahedra. It is kept here for reference for
	// implementing the distributed version (using ConvexCell can save
	// points tranfers).

        if(update_periodic_v_to_cell_) {
            periodic_v_to_cell_rowptr_.resize(nb_vertices_non_periodic_ + 1);
            periodic_v_to_cell_rowptr_[0] = 0;
            index_t cur = 0;
            for(index_t v=0; v<nb_vertices_non_periodic_; ++v) {
                cur += pop_count(vertex_instances_[v])-1;
                periodic_v_to_cell_rowptr_[v+1] = cur;
            }
            periodic_v_to_cell_data_.assign(cur, NO_INDEX);
        }

        if(keeps_infinite()) {
            geo_assert(!periodic_);
            v_to_cell_.assign(nb_vertices()+1, NO_INDEX);
            for(index_t c = 0; c < nb_cells(); c++) {
                for(index_t lv = 0; lv < 4; lv++) {
                    index_t v = cell_vertex(c, lv);
                    if(v == NO_INDEX) {
                        v = nb_vertices();
                    }
                    v_to_cell_[v] = c;
                }
            }
        } else {
            v_to_cell_.assign(nb_vertices(), NO_INDEX);
            for(index_t c = 0; c < nb_cells(); c++) {
                for(index_t lv = 0; lv < 4; lv++) {
                    index_t v = cell_vertex(c, lv);
                    if(v < nb_vertices_non_periodic_) {
                        v_to_cell_[v] = c;
                    } else if(
                        update_periodic_v_to_cell_ &&
                        v != NO_INDEX &&
			v != PeriodicDelaunay3dThread::VERTEX_OF_DELETED_TET
                    ) {
                        index_t v_real = periodic_vertex_real(v);
                        index_t v_instance = periodic_vertex_instance(v);

                        geo_debug_assert(
                            (vertex_instances_[v_real] & (1u << v_instance)) != 0
                        );

                        index_t slot = pop_count(
                            vertex_instances_[v_real] & ((1u << v_instance)-1)
                        ) - 1;

                        periodic_v_to_cell_data_[
                            periodic_v_to_cell_rowptr_[v_real] + slot
                        ] = c;
                    }
                }
            }
        }

        is_locked_ = false;
    }

    void PeriodicDelaunay3d::update_cicl() {
        // Note: updates CICL information only for the
        // corners that correspond to real vertices
        // (anyway, in the API we will be always
        // starting from a real vertex !)

        geo_assert(!is_locked_);  // Not thread-safe
        is_locked_ = true;
        cicl_.resize(4 * nb_cells());

        for(index_t v = 0; v < nb_vertices_non_periodic_; ++v) {
            index_t t = v_to_cell_[v];
            if(t != NO_INDEX) {
                index_t lv = index(t, v);
                set_next_around_vertex(t, lv, t);
            }
        }

        if(keeps_infinite()) {

            {
                // Process the infinite vertex at index nb_vertices().
                index_t t = v_to_cell_[nb_vertices()];
                if(t != NO_INDEX) {
                    index_t lv = index(t, NO_INDEX);
                    set_next_around_vertex(t, lv, t);
                }
            }

            for(index_t t = 0; t < nb_cells(); ++t) {
                for(index_t lv = 0; lv < 4; ++lv) {
                    index_t v = cell_vertex(t, lv);
                    index_t vv = (v == NO_INDEX) ? nb_vertices() : v;
                    if(v_to_cell_[vv] != t) {
                        index_t t1 = v_to_cell_[vv];
                        index_t lv1 = index(t1, v);
                        index_t t2 = next_around_vertex(t1, lv1);
                        set_next_around_vertex(t1, lv1, t);
                        set_next_around_vertex(t, lv, t2);
                    }
                }
            }


        } else {
            for(index_t t = 0; t < nb_cells(); ++t) {
                for(index_t lv = 0; lv < 4; ++lv) {
                    index_t v = cell_vertex(t, lv);
                    if(v < nb_vertices_non_periodic_ && v_to_cell_[v] != t) {
                        index_t t1 = v_to_cell_[v];
                        index_t lv1 = index(t1, v);
                        index_t t2 = next_around_vertex(t1, lv1);
                        set_next_around_vertex(t1, lv1, t);
                        set_next_around_vertex(t, lv, t2);
                    }
                }
            }
        }

        is_locked_ = false;
    }

    void PeriodicDelaunay3d::get_incident_tets(
	index_t v, IncidentTetrahedra& W
    ) const {

        geo_debug_assert(
            periodic_ || v < nb_vertices_non_periodic_
        );

        W.clear_incident_tets();

        index_t t = NO_INDEX;
        if(v < nb_vertices_non_periodic_) {
            t = v_to_cell_[v];
        } else {
            index_t v_real = periodic_vertex_real(v);
            index_t v_instance = periodic_vertex_instance(v);

            geo_debug_assert(
                (vertex_instances_[v_real] & (1u << v_instance))!=0
            );

            index_t slot = pop_count(
                vertex_instances_[v_real] & ((1u << v_instance)-1)
            ) - 1;

            t = periodic_v_to_cell_data_[
                periodic_v_to_cell_rowptr_[v_real] + slot
            ];
        }

        // Can happen: empty power cell.
        if(t == NO_INDEX) {
            return;
        }

        // TODO: different version if CICL is stored ?
        {
            W.add_incident_tet(t);
            W.S.push(t);
            while(!W.S.empty()) {
                t = W.S.top();
                W.S.pop();
                const index_t* T = &(cell_to_v_store_[4 * t]);
                index_t lv = PeriodicDelaunay3dThread::find_4(T,v);
                index_t neigh = cell_to_cell_store_[4*t + (lv + 1)%4];
                if(neigh != NO_INDEX && !W.has_incident_tet(neigh)) {
                    W.add_incident_tet(neigh);
                    W.S.push(neigh);
                }
                neigh = cell_to_cell_store_[4*t + (lv + 2)%4];
                if(neigh != NO_INDEX && !W.has_incident_tet(neigh)) {
                    W.add_incident_tet(neigh);
                    W.S.push(neigh);
                }
                neigh = cell_to_cell_store_[4*t + (lv + 3)%4];
                if(neigh != NO_INDEX && !W.has_incident_tet(neigh)) {
                    W.add_incident_tet(neigh);
                    W.S.push(neigh);
                }
            }
        }
    }

    inline double dist2(const double* p, const double* q) {
        return
            geo_sqr(p[0]-q[0]) +
            geo_sqr(p[1]-q[1]) +
            geo_sqr(p[2]-q[2]) ;
    }

    /**
     * \brief Copies a Laguerre cell from the triangulation.
     * \param[in] i the index of the vertex of which the Laguerre cell
     *  should be computed.
     * \param[out] C the Laguerre cell.
     * \param[out] W the vector of neighbor vertices indices.
     */
    void PeriodicDelaunay3d::copy_Laguerre_cell_from_Delaunay(
        GEO::index_t i,
        ConvexCell& C,
        IncidentTetrahedra& W
    ) const {
        // Create global vertex indices if not present.
        C.create_vglobal();
        C.clear();

        // Create the vertex at infinity.
        C.create_vertex(vec4(0.0, 0.0, 0.0, 0.0), NO_INDEX);

        GEO::vec3 Pi = vertex(i);
        double wi = weight(i);
        double Pi_len2 = Pi[0]*Pi[0] + Pi[1]*Pi[1] + Pi[2]*Pi[2];

        if(stores_cicl()) {
            // Get neighbors and initialize cell from the tetrahedra in
            // 1-ring neighborhood of vertex.
            GEO::index_t t = GEO::index_t(vertex_cell(i));
            // Special case: Laguerre cell is empty (vertex has
            // no incident tet).
            if(t == NO_INDEX) {
                return;
            }
            do {
                GEO::index_t f = copy_Laguerre_cell_facet_from_Delaunay(
                    i, Pi, wi, Pi_len2, t, C, W
                );
                t = GEO::index_t(next_around_vertex(t,f));
            } while(t != GEO::index_t(vertex_cell(i)));
        } else {
            get_incident_tets(i,W);
            for(index_t t: W) {
                copy_Laguerre_cell_facet_from_Delaunay(
                    i, Pi, wi, Pi_len2, t, C, W
                );
            }
        }
        C.connect_triangles();
    }

    GEO::index_t PeriodicDelaunay3d::copy_Laguerre_cell_facet_from_Delaunay(
        GEO::index_t i,
        const GEO::vec3& Pi,
        double wi,
        double Pi_len2,
        GEO::index_t t,
        ConvexCell& C,
        IncidentTetrahedra& W
    ) const {
        geo_argused(W);

        // Local tet vertex indices from facet
        // and vertex in facet indices.
        static GEO::index_t fv[4][3] = {
            {2,3,1},
            {3,2,0},
            {0,1,3},
            {2,1,0}
        };

        GEO::index_t f = index(t,GEO::index_t(i));
        GEO::index_t jkl[3];  // Global index (in Delaunay) of triangle vertices
        VBW::index_t l_jkl[3];// Local index (in C) of triangle vertices


        // Find or create the three vertices of the facet.
        for(int lfv=0; lfv<3; ++lfv) {

            jkl[lfv] = GEO::index_t(cell_vertex(t, fv[f][lfv]));
            l_jkl[lfv] = VBW::index_t(-1);

            // Vertex already created in C (note:
            // also works for vertex at infinity)
            for(VBW::index_t u=0; u<C.nb_v(); ++u) {
                if(C.v_global_index(u) == jkl[lfv]) {
                    l_jkl[lfv] = VBW::index_t(u);
                    break;
                }
            }

            // vertex not found, create vertex in C
            if(l_jkl[lfv] == VBW::index_t(-1)) {
                l_jkl[lfv] = C.nb_v();
                vec3 Pj = vertex(jkl[lfv]);
		double Pj_len2 = length2(Pj);
                double wj = weight(jkl[lfv]);
                double a = 2.0 * (Pi[0] - Pj[0]);
                double b = 2.0 * (Pi[1] - Pj[1]);
                double c = 2.0 * (Pi[2] - Pj[2]);
                double d = ((wi - Pi_len2) - (wj - Pj_len2));
                C.create_vertex(vec4(a,b,c,d), jkl[lfv]);
            }
        }

        C.create_triangle(l_jkl[0], l_jkl[1], l_jkl[2]);

        return f;
    }

    /*************************************************************************/

    void PeriodicDelaunay3d::insert_vertices(
	const char* phase, index_t b, index_t e
    ) {

	Stopwatch W(phase,detailed_benchmark_mode_);

        if(detailed_benchmark_mode_) {
            Logger::out(phase) << "Inserting "    << (e-b)
			       << " additional vertices" << std::endl;
        }

        has_empty_cells_ = false;

        nb_vertices_ = reorder_.size();
	vector<index_t> levels;

	compute_BRIO_order_periodic(
            nb_vertices_non_periodic_ * 27, // nb of possible periodic vertices
            vertex_ptr(0),
	    3, dimension(),
	    reorder_,
            reorder_.begin() + long(b),
            reorder_.begin() + long(e),
	    period_,
	    64, 0.125,
	    &levels
	);


#ifdef GEO_DEBUG
	// Check that the same vertex was not inserted twice
        for(index_t i=b; i+1<e; ++i) {
            geo_debug_assert(reorder_[i] != reorder_[i+1]);
        }
#endif

	insert_vertices_with_BRIO(phase, levels);
        if(has_empty_cells_) {
            return;
        }
        PeriodicDelaunay3dThread* thread0 = thread(0);
	nb_vertices_ = reorder_.size();
        set_arrays(
            thread0->max_t(),
            cell_to_v_store_.data(),
            cell_to_cell_store_.data()
        );

	if(!strcmp(phase, "insert-I")) {
	    stats_.phase_I_insert_t_ = W.elapsed_time();
	    stats_.phase_I_insert_nb_ = e-b;
	} else if(!strcmp(phase, "insert-II")) {
	    stats_.phase_II_insert_t_ = W.elapsed_time();
	    stats_.phase_II_insert_nb_ = e-b;
	}
    }

    void PeriodicDelaunay3d::insert_vertices_with_BRIO(
	const char* phase, const vector<index_t>& levels
    ) {

        for(index_t t=0; t<threads_.size(); ++t) {
	    thread(t)->reset_stats();
	}

        PeriodicDelaunay3dThread* thread0 = thread(0);

        index_t lvl = 1;
        while(lvl < (levels.size() - 1) && (levels[lvl] - levels[0]) < 1000) {
            ++lvl;
        }

        if(detailed_benchmark_mode_) {
            Logger::out(phase)
                << "Using " << levels.size()-1 << " levels" << std::endl;
            Logger::out(phase)
                << "Levels 0 - " << lvl-1
                << ": bootstraping with first levels in sequential mode"
                << std::endl;
        }

        thread0->set_work(levels[0], levels[lvl]);
        thread0->run();

        if(thread0->has_empty_cells()) {
            has_empty_cells_ = true;
            return;
        }

        index_t nb_sequential_points = 0;
        index_t first_lvl = lvl;

        // Insert points in all BRIO levels
        for(; lvl<levels.size()-1; ++lvl) {

            index_t lvl_b = levels[lvl];
            index_t lvl_e = levels[lvl+1];

            if(detailed_benchmark_mode_) {
                Logger::out(phase) << "Level "
				   << lvl << " : start "
				   << " nbv = "
				   << (lvl_e - lvl_b)
				   << std::endl;
            }

            index_t work_size = (lvl_e - lvl_b)/index_t(threads_.size());

            // Initialize threads
            index_t b = lvl_b;
            for(index_t t=0; t<threads_.size(); ++t) {
                index_t e = (t == threads_.size()-1) ? lvl_e : b+work_size;

                // Copy the indices of the first created tetrahedron
                // and the maximum valid tetrahedron index max_t_
                if(lvl == first_lvl && t!=0) {
                    thread(t)->set_max_t(thread0->max_t());
                }
                thread(t)->set_work(b,e);
                b = e;
            }

	    check_max_t();
	    Process::run_threads(threads_);

            for(index_t t=0; t<this->nb_threads(); ++t) {
                if(thread(t)->has_empty_cells()) {
                    has_empty_cells_ = true;
                    return;
                }
            }

	    // Run threads sequentialy, to insert missing points if
	    // memory overflow was encountered (in sequential mode,
	    // dynamic memory growing works)

	    index_t this_level_nb_sequential_points = 0;

	    for(index_t t=0; t<threads_.size(); ++t) {
		PeriodicDelaunay3dThread* t1 = thread(t);
		this_level_nb_sequential_points += t1->work_size();
		if(t != 0) {
		    // We need to copy max_t_ from previous thread,
		    // since the memory pool may have grown.
		    PeriodicDelaunay3dThread* t2 = thread(t-1);
		    t1->set_max_t(t2->max_t());
		}
		t1->run();
		if(t1->has_empty_cells()) {
		    has_empty_cells_ = true;
		    return;
		}
	    }

	    //  If some tetrahedra were created in sequential mode, then
	    // the maximum valid tetrahedron index was increased by all
	    // the threads in increasing number, so we copy it from the
	    // last thread into thread0 since we use thread0 afterwards
	    // to get max_t()

	    if(this_level_nb_sequential_points != 0) {
		PeriodicDelaunay3dThread* t0 = thread(0);
		PeriodicDelaunay3dThread* tn = thread(
		    this->nb_threads()-1
		);
		t0->set_max_t(tn->max_t());
	    }

	    if(has_empty_cells_) {
		return;
	    }

	    nb_sequential_points += this_level_nb_sequential_points;
        }

        if(detailed_benchmark_mode_) {
            index_t tot_rollbacks = 0 ;
            index_t tot_failed_locate = 0 ;
            for(index_t t=0; t<threads_.size(); ++t) {
		Logger::out(phase)
		    << "thread " << std::setw(3) << t << " : "
		    << std::setw(3) << thread(t)->nb_rollbacks()
		    << " rollbacks  "
		    << std::setw(3) << thread(t)->nb_failed_locate()
		    << " restarted locate"
		    << std::endl;
                tot_rollbacks += thread(t)->nb_rollbacks();
                tot_failed_locate += thread(t)->nb_failed_locate();
            }
	    Logger::out(phase) << "------------------" << std::endl;
            Logger::out(phase) << "total: "
			       << tot_rollbacks << " rollbacks  "
			       << tot_failed_locate << " restarted locate"
			       << std::endl;
	    if(nb_sequential_points == 0) {
		Logger::out(phase) << "All points where inserted in parallel"
				    << std::endl;
	    } else {
		Logger::out(phase) << nb_sequential_points
				   << " points inserted in sequential mode"
				   << std::endl;
	    }
        }

	nb_vertices_ = reorder_.size();
        index_t nb_tets = thread0->max_t();

	for(index_t i=0; i<nb_threads(); ++i) {
	    geo_assert(thread(i)->max_t() <= nb_tets);
	}
    }

    bool PeriodicDelaunay3d::Laguerre_vertex_is_in_conflict_with_plane(
	index_t t, vec4 P
    ) const {

	// Note: facet orientations and signs follow:
	// - ConvexCell
	// - copy_Laguerre_facet_from()

        // Local tet vertex indices from facet and vertex in facet indices.
	// Carefully chosen in such a way that f[(lv+1)%4][2] == lv
	// This is used in the code block below, that handles tets with
	// a vertex at infinity
        static GEO::index_t fv[4][3] = {
            {1,2,3},
            {3,2,0},
            {3,0,1},
            {1,0,2}
        };

	// Particular case, vertex at infinity
	for(index_t lv=0; lv<4; ++lv) {
	    if(cell_vertex(t,lv) == NO_INDEX) {
		index_t li = (lv + 1) % 4; // this vertex is not at infty
		// li is also the index of a facet with the vertex at infty
		// as the last vertex (fv[][] was constructed so), so we
		// are sure that vi, vj, vk are the not at infty
		index_t vi = cell_vertex(t, li);
		index_t vj = cell_vertex(t, fv[li][0]);
		index_t vk = cell_vertex(t, fv[li][1]);
		geo_debug_assert(fv[li][2] == lv); // we know that l == -1
		vec3 pi = vertex(vi);
		vec3 pj = vertex(vj);
		vec3 pk = vertex(vk);
		Sign s = PCK::det_3d(
		    pi-pj,
		    pi-pk,
		    vec3(P.x,P.y,P.z)
		);
		return (s <= 0);
	    }
	}

	index_t v1 = cell_vertex(t,0);
	index_t v2 = cell_vertex(t,1);
	index_t v3 = cell_vertex(t,2);
	index_t v4 = cell_vertex(t,3);

	vec3 p1 = vertex(v1);
	vec3 p2 = vertex(v2);
	vec3 p3 = vertex(v3);
	vec3 p4 = vertex(v4);

	double l1 = weight(v1) - length2(p1);
	double l2 = weight(v2) - length2(p2);
	double l3 = weight(v3) - length2(p3);
	double l4 = weight(v4) - length2(p4);

	Sign s = PCK::det_4d(
	    vec4(2.0*(p1.x-p2.x), 2.0*(p1.y-p2.y), 2.0*(p1.z-p2.z), l1-l2),
	    vec4(2.0*(p1.x-p3.x), 2.0*(p1.y-p3.y), 2.0*(p1.z-p3.z), l1-l3),
	    vec4(2.0*(p1.x-p4.x), 2.0*(p1.y-p4.y), 2.0*(p1.z-p4.z), l1-l4),
	    P
	);

	return (s >= 0);
    }


    void PeriodicDelaunay3d::handle_periodic_boundaries_phase_I() {
        Stopwatch W_classify_I("classify-I", detailed_benchmark_mode_);
        PeriodicDelaunay3dThread* thread0 = thread(0);

	// Lag_cell_status:
	//
	// each bit k in 0..5  indicate that cell has at least one vertex
	//                     in conflict with plane Pk (outside central cube)
	// each bit k in 6..11 indicate that cell has all its vertices
	//                     in conflict with plane Pk (outside central cube)
	// status = 0 -> cell is contained by central cube
	// (status & conflict_mask) != 0 -> cell straddles bndr of central cube
	// (status & all_conflict_mask) != 0 -> cell is outside central cube

	// static Numeric::uint16 conflict_mask     = Numeric::uint16(63u);
	static Numeric::uint16 all_conflict_mask = Numeric::uint16(63u << 6);

	std::atomic<Numeric::uint16>* Lag_cell_status
	    = new std::atomic<Numeric::uint16>[nb_vertices_non_periodic_];

	for(index_t i=0; i<nb_vertices_non_periodic_; ++i) {
	    Lag_cell_status[i] = all_conflict_mask;
	}

	{
	    vec4 cube_face[6] = {
		vec4( 1.0, 0.0, 0.0,  0.0),
		vec4(-1.0, 0.0, 0.0,  period_.x),
		vec4( 0.0, 1.0, 0.0,  0.0),
		vec4( 0.0,-1.0, 0.0,  period_.y),
		vec4( 0.0, 0.0, 1.0,  0.0),
		vec4( 0.0, 0.0,-1.0,  period_.z),
	    };

	    parallel_for(0, thread0->max_t(), [&](index_t t) {
		if(thread0->tet_is_free(t)) {
		    return;
		}
		for(index_t k=0; k<6; ++k) {
		    bool conflict = Laguerre_vertex_is_in_conflict_with_plane(
			t, cube_face[k]
		    );
		    for(index_t lv=0; lv<4; ++lv) {
			index_t v = cell_vertex(t,lv);
			if(v == NO_INDEX) {
			    continue;
			}
			if(conflict) {
			    // set 'conflict' bit k
			    Lag_cell_status[v].fetch_or(
				Numeric::uint16(1u << k),
				std::memory_order_relaxed
			    );
			} else {
			    // reset 'all conflict' bit k
			    Lag_cell_status[v].fetch_and(
				Numeric::uint16(~(1u << (k+6))),
				std::memory_order_relaxed
			    );
			}
		    }
		}
	    }
	    );
	}

	// Count cells inside, crossing, outside
	{
	    stats_.phase_I_nb_inside_ = 0;
	    stats_.phase_I_nb_cross_ = 0;
	    stats_.phase_I_nb_outside_ = 0;
	    for(index_t v=0; v<nb_vertices_non_periodic_; ++v) {
		Numeric::uint16 status = Lag_cell_status[v];
		if(status == 0) {
		    ++stats_.phase_I_nb_inside_;
		} else if((status & all_conflict_mask) != 0) {
		    ++stats_.phase_I_nb_outside_;
		} else {
		    ++stats_.phase_I_nb_cross_;
		}
	    }

	    if(detailed_benchmark_mode_) {
		Logger::out("classify-I") << "Nb cells inside cube: "
					  << stats_.phase_I_nb_inside_
					  << std::endl;
		Logger::out("classify-I") << "Nb cells on boundary: "
					  << stats_.phase_I_nb_cross_
					  << std::endl;
		Logger::out("classify-I") << "Nb cells outside cube: "
					  << stats_.phase_I_nb_outside_
					  << std::endl;
	    }
	}

        // Indicates for each real vertex the instances it has.
        // Each bit of vertex_instances_[v] indicates which instance
        // is used.
        vertex_instances_.assign(nb_vertices_non_periodic_,1);

	for(index_t v=0; v<nb_vertices_non_periodic_; ++v) {
	    Numeric::uint16 status = Lag_cell_status[v];

	    bool status_inside = (status == 0);

	    // In the distributed version, we might need to distinguish
	    // also the following two cases:
	    // bool status_outside = ((status & all_conflict_mask) != 0);
	    // bool status_crossing = !status_inside && !status_outside;

	    // if cell is inside cube, no instance to create
	    if(status_inside) {
		continue;
	    }

	    // Integer translations associated with the six plane equations
	    static int T[6][3]= {
		{-1, 0, 0},
		{ 1, 0, 0},
		{ 0,-1, 0},
		{ 0, 1, 0},
		{ 0, 0,-1},
		{ 0, 0, 1}
	    };

	    // Detect the bounds of the sub-(rubic's) cube overlapped
	    // by the cell.

	    int TXmin = 2, TXmax = -2,
		TYmin = 2, TYmax = -2,
		TZmin = 2, TZmax = -2;

	    FOR(i,6) {
		if((status & Numeric::uint8(1u << i)) != 0) {
		    TXmin = std::min(TXmin, T[i][0]);
		    TXmax = std::max(TXmax, T[i][0]);
		    TYmin = std::min(TYmin, T[i][1]);
		    TYmax = std::max(TYmax, T[i][1]);
		    TZmin = std::min(TZmin, T[i][2]);
		    TZmax = std::max(TZmax, T[i][2]);
		}
	    }

	    for(int TX = TXmin; TX <= TXmax; ++TX) {
		for(int TY = TYmin; TY <= TYmax; ++TY) {
		    for(int TZ = TZmin; TZ <= TZmax; ++TZ) {
			index_t instance = T_to_instance(-TX,-TY,-TZ);
			// Skip instance 0 (it was already inserted !)
			if(instance != 0) {
			    vertex_instances_[v] |= (1u << instance);
			    reorder_.push_back(make_periodic_vertex(v,instance));
			}
		    }
		}
	    }
	}
	delete[] Lag_cell_status;
	stats_.phase_I_classify_t_ = W_classify_I.elapsed_time();
    }

    void PeriodicDelaunay3d::handle_periodic_boundaries_phase_II() {
	Stopwatch W_classify_II("classify-II", detailed_benchmark_mode_);
	PeriodicDelaunay3dThread* thread0 = thread(0);

	// Computes translation_table[][]
	// translation_table[instance2][instance1] transforms
	// instance2 into the frame of instance1
	Numeric::int8 translation_table[27][27];
	for(index_t instance1=0; instance1<27; ++instance1) {
	    int Tx1 = translation[instance1][0];
	    int Ty1 = translation[instance1][1];
	    int Tz1 = translation[instance1][2];
	    for(index_t instance2=0; instance2<27; ++instance2) {
		int Tx2 = translation[instance2][0];
		int Ty2 = translation[instance2][1];
		int Tz2 = translation[instance2][2];

		translation_table[instance2][instance1] =
		    Numeric::int8(instance2);

		if(instance1 == instance2) {
		    continue;
		}

		if(
		    std::abs(Tx2 - Tx1) >= 2 ||
		    std::abs(Ty2 - Ty1) >= 2 ||
		    std::abs(Tz2 - Tz1) >= 2
		) {
		    // Here we could use -1 to encode large displacements,
		    // and issue an error message (for now they are ignored)
		    continue;
		}

		translation_table[instance2][instance1] =
		    Numeric::int8(T_to_instance(Tx2-Tx1,Ty2-Ty1,Tz2-Tz1));
	    }
	}

	std::atomic<Numeric::uint32>* new_vertex_instances =
	    new std::atomic<Numeric::uint32>[vertex_instances_.size()];

	for(index_t i=0; i<vertex_instances_.size(); ++i) {
	    new_vertex_instances[i] = vertex_instances_[i];
	}

	Process::spinlock lock = GEOGRAM_SPINLOCK_INIT;
	parallel_for(0, thread0->max_t(), [&,this](index_t t) {
	    if(!thread0->tet_is_real(t)) {
		return;
	    }
	    // Find the edges v1,v2 such that:
	    //   v1 is a vertex that was inserted in phase-I (instance != 0)
	    //   v2 is a vertex in an instance different from v1
	    for(index_t lv=0; lv<4; ++lv) {
		index_t v1 = thread0->finite_tet_vertex(t, lv);
		index_t v1_instance = periodic_vertex_instance(v1);
		if(v1_instance == 0) {
		    continue;
		}
		for(index_t dlv=1; dlv<4; ++dlv) {
		    index_t v2 = thread0->finite_tet_vertex(t, (lv + dlv)%4);
		    index_t v2_real = periodic_vertex_real(v2);
		    index_t v2_instance = periodic_vertex_instance(v2);

		    // transform v2_instance into the local frame of v1
		    v2_instance = index_t(
			translation_table[v2_instance][v1_instance]
		    );
		    if(v2_instance == v1_instance) {
			continue;
		    }

		    // create the transformed v2_instance if it does not
		    // already exist, and memorize it in the new list of
		    // vertices to create

		    Numeric::uint32 mask = (1u << v2_instance);
		    Numeric::uint32 prev_instances =
			new_vertex_instances[v2_real].fetch_or(
			    mask, std::memory_order_relaxed // only need atomic
			);

		    if((prev_instances & mask) == 0) {
			Process::acquire_spinlock(lock);
			reorder_.push_back(
			    make_periodic_vertex(v2_real, v2_instance)
			);
			Process::release_spinlock(lock);
		    }
		}
	    }
	});
	for(index_t i=0; i<vertex_instances_.size(); ++i) {
	    vertex_instances_[i] = new_vertex_instances[i];
	}
	delete[] new_vertex_instances;
	stats_.phase_II_classify_t_ = W_classify_II.elapsed_time();
    }

    void PeriodicDelaunay3d::handle_periodic_boundaries() {

        // Update pointers so that queries function will work (even in our
        // "transient state").
        PeriodicDelaunay3dThread* thread0 = thread(0);

        set_arrays(
            thread0->max_t(),
            cell_to_v_store_.data(),
            cell_to_cell_store_.data()
        );

	// Test for empty cells
	// TODO: I tested, it really occurs that there is still an empty
	// cell here, why is it not detected before ? To be understood.
        update_v_to_cell();
        for(index_t v=0; v<nb_vertices_non_periodic_; ++v) {
            if(v_to_cell_[v] == NO_INDEX) {
                has_empty_cells_ = true;
                return;
            }
        }

        // Phase I: find the cells that intersect the boundaries, and
        // create periodic vertices for each possible translation.
	{
	    Stopwatch W_phase_I("phase-I",false);
	    handle_periodic_boundaries_phase_I();
	    insert_vertices(
		"insert-I", nb_vertices_non_periodic_, reorder_.size()
	    );
	    stats_.phase_I_t_ = W_phase_I.elapsed_time();
	}

        // Phase II: Insert the real neighbors of the virtual vertices,
        // back-translated to the original position.
	{
	    index_t nb_vertices_phase_I = reorder_.size();
	    Stopwatch W_phase_II("phase-II",false);
	    handle_periodic_boundaries_phase_II();
	    insert_vertices("insert-II", nb_vertices_phase_I, reorder_.size());
	    stats_.phase_II_t_ = W_phase_II.elapsed_time();
	}
    }

    void PeriodicDelaunay3d::check_volume() {
        ConvexCell C;
        C.use_exact_predicates(convex_cell_exact_predicates_);

        Logger::out("Periodic") << "Checking total volume..." << std::endl;
        double sumV = 0;
        IncidentTetrahedra W;

        FOR(v, nb_vertices_non_periodic_) {
            copy_Laguerre_cell_from_Delaunay(v, C, W);
#ifdef GEO_DEBUG
            for(
                VBW::ushort t = C.first_triangle();
                t!=VBW::END_OF_LIST; t=C.next_triangle(t)
            ) {
                for(index_t lv=0; lv<3; ++lv) {
                    // Make sure there is no vertex at infinity
                    geo_debug_assert(
                        C.triangle_v_local_index(t,VBW::index_t(lv)) != 0
                    );
                }
            }
#endif
            C.compute_geometry();
            sumV += C.volume();
        }

        double expectedV = period_.x*period_.y*period_.z;

        Logger::out("Periodic") << "Sum volumes = " << sumV << std::endl;
        Logger::out("Periodic") << "  (expected " <<  expectedV << ")"
                                << std::endl;

        if(::fabs(sumV - expectedV) / expectedV >= 1.0 / 10000.0) {
            Logger::err("Periodic") << "FATAL, volume error is too large"
                                    << std::endl;
            exit(-1);
        }

    }

    void PeriodicDelaunay3d::save_cells(
        const std::string& basename, bool clipped
    ) {
        static int index = 1;

        std::string index_string = String::to_string(index);
        while(index_string.length() < 3) {
            index_string = "0" + index_string;
        }

        std::ofstream out((basename+index_string+".obj").c_str());
        index_t v_off = 1;

        ConvexCell C;
        C.use_exact_predicates(convex_cell_exact_predicates_);
        IncidentTetrahedra W;
        for(index_t vv=0; vv<nb_vertices_non_periodic_; ++vv) {
            copy_Laguerre_cell_from_Delaunay(vv, C, W);
            if(clipped) {
                C.clip_by_plane(vec4( 1.0, 0.0, 0.0,  0.0));
                C.clip_by_plane(vec4(-1.0, 0.0, 0.0,  period_.x));
                C.clip_by_plane(vec4( 0.0, 1.0, 0.0,  0.0));
                C.clip_by_plane(vec4( 0.0,-1.0, 0.0,  period_.y));
                C.clip_by_plane(vec4( 0.0, 0.0, 1.0,  0.0));
                C.clip_by_plane(vec4( 0.0, 0.0,-1.0,  period_.z));
            }
            v_off += C.save(out, v_off, 0.1);
        }
        ++index;
    }

    void PeriodicDelaunay3d::check_max_t() {
	index_t max_t = 0;
	for(index_t i=0; i<nb_threads(); ++i) {
	    max_t = std::max(max_t, thread(i)->max_t());
	}
	for(index_t i=0; i<nb_threads(); ++i) {
	    geo_assert(thread(i)->max_t() == max_t);
	}
    }

/***********************************************************/

    PeriodicDelaunay3d::Stats::Stats() {
	reset();
    }

    void PeriodicDelaunay3d::Stats::reset() {
	Memory::clear(this, sizeof(Stats));
	raw_ = CmdLine::get_arg_bool("dbg:raw_logs");
    }

    std::string PeriodicDelaunay3d::Stats::to_string_raw() const {
	return String::format(
	    "%.1f %.1f %.1f %.1f %d %d %d %.1f %d %.1f %.1f %.1f %d",
	    total_t_,

	    phase_0_t_,

	    phase_I_t_, phase_I_classify_t_,

	    int(phase_I_nb_inside_), int(phase_I_nb_cross_),
	    int(phase_I_nb_outside_),

	    phase_I_insert_t_, int(phase_I_insert_nb_),

	    phase_II_t_, phase_II_classify_t_, phase_II_insert_t_,
	    int(phase_II_insert_nb_)
	);
    }


    std::string PeriodicDelaunay3d::Stats::to_string_pretty() const {
	return String::format(
	    "total   | t:%.1f\n"
	    "phase0  | t:%.1f\n"
	    "phaseI  | t:%.1f t_cls:%.1f t_ins:%.1f nb_ins:%d\n"
	    "        |   in:%d bndry:%d out:%d\n"
	    "phaseII | t:%.1f t_cls:%.1f t_ins:%.1f nb_ins:%d",
	    total_t_,

	    phase_0_t_,

	    phase_I_t_, phase_I_classify_t_,
	    phase_I_insert_t_, int(phase_I_insert_nb_),

	    int(phase_I_nb_inside_), int(phase_I_nb_cross_),
	    int(phase_I_nb_outside_),

	    phase_II_t_, phase_II_classify_t_,
	    phase_II_insert_t_, int(phase_II_insert_nb_)
	);
    }
}
