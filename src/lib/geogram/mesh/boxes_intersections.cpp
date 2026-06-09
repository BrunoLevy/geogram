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

#include <geogram/mesh/boxes_intersections.h>
#include <geogram/basic/process.h>
#include <random>

/*
 * The algorithm implemented here for fast vector<Box> x vector<Box> intersection
 * in the function hybrid() is described in:
 * Fast software for box intersections
 *   Afra Zoromodian and Herbert Edelsbrunner
 *   International Journal of Computational Geometry & Applications
 *   2002
 *
 * Internally is uses a fast median approximation implemented in the
 * approximate_median() function, and described in:
 * Approximating center points with iterative radon points
 *    K. L. Clarkson, D. Eppstein, G. L. Miller, C. Sturtivant, and S- H. Teng.
 *   International Journal of Computational Geometry & Applications
 *   6 (1996) 357–377
 */

namespace {
    using namespace GEO;

#ifdef GARGANTUA
    typedef std::mt19937_64 RandomEngine;
#else
    typedef std::mt19937 RandomEngine;
#endif


    /**
     * \brief comparison between two coordinates with simulation-of-simplicity
     *  symbolic perturbation.
     * \details used to uniquely assign an interval or a point to a given
     *  subrange when splitting ranges of intervals or points. Also used for
     *  sorting boxes.
     * \param[in] a , b the two coordinates to be compared
     * \param[in] ida , idb two ids or indices corresponding to \p a and \p b
     * \retval true if \p a is smaller than \p b. In case of equality, the ids
     *  \p ida and \p idb are used to disambiguate
     * \retval false otherwise
     */
    inline bool lt_sos(double a, double b, index_t ida, index_t idb) {
	return (a < b || (a == b && ida < idb));
    }

    /**
     * \brief Comparator class for sorting boxes along various dimensions
     */
    struct BoxesCompare {
	/**
	 * \brief Compares two boxes
	 * \param[in] ida , idb the indices of the two boxes to be compared
	 * \retval true if box \p ida is before \p idb
	 * \retval false otherwise
	 * \details uses lt_sos()
	 */
	bool operator() (index_t ida, index_t idb) const {
	    double a = boxes[ida].xyz_min[d];
	    double b = boxes[idb].xyz_min[d];
	    return lt_sos(a,b,ida,idb);
	}

	const Box3d* boxes;
	index_t d;
    };

    /************************************************************************/

    /**
     * \brief a range of boxes used to implement box-box intersection algorithm
     * \details Stores a pointer to an array of boxes and a begin-end
     *  range of pointers to box indices
     */
    struct BoxesRange {

	/**
	 * \brief Tests whether a BoxRange is empty
	 * \retval true if this BoxRange contains no box
	 * \retval false otherwise
	 */
	bool empty() const {
	    return e == b;
	}

	/**
	 * \brief Gets the size of a BoxRange
	 * \return the number of boxes in this BoxRange
	 */
	index_t size() const {
	    return index_t(e-b);
	}

	/**
	 * \brief Splits this range into two subranges
	 * \details first subrange contains boxes  e for which
	 *   (precond && predicate(e)) evaluates as true, and second
	 *   subrange contains boxes e for which (precond && predicate(e))
	 *   evaluates as false
	 * \param[in] predicate a function taking the index of a box and
	 *   returning true if box should be moved to first subrange and false
	 *   otherwise
	 * \param[in] precond a precondition anded with the predicate
	 * \return a pair of ranges
	 */
	std::pair<BoxesRange, BoxesRange> split(
	    std::function<bool(index_t)> predicate, bool precond = true
	) {
	    if(!precond) {
		return std::make_pair(
		    BoxesRange{boxes,b,b},
		    BoxesRange{boxes,b,e}
		);
	    }
	    index_t* m = std::partition(b,e,predicate);
	    // a BoxesRange is just three pointers, so there is no harm returning
	    // two of them by value.
	    return std::make_pair(
		BoxesRange{boxes,b,m},
		BoxesRange{boxes,m,e}
	    );
	}


	/**
	 * \brief gets the lower bound of a box for a given coordinate
	 * \param[in] i the index of the box
	 * \param[in] d the coordinate
	 * \return the coordinate \p d of the lower bound of box \p i
	 */
	double xmin(index_t i, index_t d) const {
	    geo_debug_assert(d < 3);
	    return boxes[i].xyz_min[d];
	}

	/**
	 * \brief gets the lower bound of a box for a given coordinate
	 * \param[in] i the index of the box
	 * \param[in] d the coordinate
	 * \return the coordinate \p d of the lower bound of box \p i
	 */
	double xmax(index_t i, index_t d) const {
	    geo_debug_assert(d < 3);
	    return boxes[i].xyz_max[d];
	}

	/**
	 * \brief gets the lower bound of a box for a given coordinate
	 * \param[in] i a pointer to the index of the box
	 * \param[in] d the coordinate
	 * \return the coordinate \p d of the lower bound of box \p i
	 */
	double xmin(index_t* i, index_t d) const {
	    geo_debug_assert(i >= b && i < e);
	    return xmin(*i, d);
	}

	/**
	 * \brief gets the upper bound of a box for a given coordinate
	 * \param[in] i a pointer to the index of the box
	 * \param[in] d the coordinate
	 * \return the coordinate \p d of the upper bound of box \p i
	 */
	double xmax(index_t* i, index_t d) const {
	    geo_debug_assert(d < 3);
	    return xmax(*i,d);
	}

	/**
	 * \brief Tests whether boxes as interval intersect along all coordinates
	 *  from 1 to a given upper bound
	 * \param[in] i pointer to first box index
	 * \param[in] J the BoxesRange in which \p j resides
	 * \param[in] j pointer to second box index
	 * \param[in] d upper bound of coordinates to be tested
	 * \retval true if all coordinate invervals intersect from 1 to d
	 * \retval false otherwise
	 */
	bool II_isect_1tod(
	    index_t* i, const BoxesRange& J, index_t* j, index_t d
	) {
	    if(boxes == J.boxes && *i == *j) { // do not report self-intersection
		return false;
	    }
	    for(index_t dim=1; dim<=d; ++dim) {
		if(xmax(i,dim) < J.xmin(j,dim) || xmin(i,dim) > J.xmax(j,dim)) {
		    return false;
		}
	    }
	    return true;
	}

	/**
	 * \brief Tests whether a box seen as an interval contains a
	 *  (box seen as a) point for a given coordinate
	 * \details does symbolic perturbation on the left bound of the interval,
	 *  like in BoxCompare
	 * \param[in] i pointer to index of first box
	 * \param[in] P the BoxesRange in which \p p resides
	 * \param[in] p pointer to index of second box
	 * \param[in] d coordinate to be tested
	 * \retval true if \p i seen as an interval contains \p p seen as a point
	 *  for coord \p d
	 * \retval false otherwise
	 */
	bool I_contains_P(
	    index_t* i, const BoxesRange& P, index_t* p, index_t d
	) {
	    double ixmin = xmin(i,d);
	    double ixmax = xmax(i,d);
	    double px = P.xmin(p,d);
	    return lt_sos(ixmin, px, *i, *p) && (ixmax >= px);
	}

	const Box3d* boxes; /**< a pointer to the array of 3d boxes */
	index_t* b; /**< a pointer to the first index */
	index_t* e; /**< a pointer to one position past the last index */
    };

    /************************************************************************/

    /**
     * \brief Reports all pairs (i,p) such that box i contains point p
     *  in a set of boxes I seen as intervals and a set of boxes P seen as
     *  points (stabbing)
     * \details see Fast software for box intersections,
     *   Afra Zoromodian and Herbert Edelsbrunner,
     *   International Journal of Computational Geometry & Applications, 2002
     * \param[in] I a set of boxes seen as intervals
     * \param[in] P a set of boxes seen as points
     * \param[in] d maximum dimension to be tested
     * \param[in] report_isect the callback used to report intersections, takes
     *  two integers, i and p
     * \param[in] swap_ip if set, the parameters i,p of the callback are swapped
     */
    void one_way_scan(
	BoxesRange I, BoxesRange P, index_t d,
	std::function<void(index_t,index_t)> report_isect, bool swap_ip = false
    ) {
	std::sort(I.b, I.e, BoxesCompare{I.boxes,0});
	std::sort(P.b, P.e, BoxesCompare{P.boxes,0});
	for(index_t* i = I.b; i != I.e; ++i) {
	    while(P.b != P.e && lt_sos(P.xmin(P.b,0), I.xmin(i,0), *P.b, *i)) {
		++P.b;
	    }
	    for(index_t* p=P.b; p!=P.e && P.xmin(p,0) <= I.xmax(i,0); ++p) {
		if(P.II_isect_1tod(p, I, i, d)) {
		    report_isect(swap_ip ? *p : *i, swap_ip ? *i : *p);
		}
	    }
	}
    }

    /**
     * \brief Reports all pairs (i,p) such that boxes i and p have intersections
     *  in two sets of boxes I and P
     * \details reports the same intersections as obtained by calling both
     *   one_way_scan(I,P) and one_way_scan(P,I).
     *   see Fast software for box intersections,
     *   Afra Zoromodian and Herbert Edelsbrunner,
     *   International Journal of Computational Geometry & Applications, 2002
     * \param[in] I a set of boxes
     * \param[in] P a set of boxes
     * \param[in] d maximum dimension to be tested
     * \param[in] report_isect the callback used to report intersections, takes
     *  two integers, i and p
     * \param[in] swap_ip if set, the parameters i,p of the callback are swapped
     */
    void modified_two_way_scan(
	BoxesRange I, BoxesRange P, index_t d,
	std::function<void(index_t,index_t)> report_isect, bool swap_ip = false
    ) {
	std::sort(I.b, I.e, BoxesCompare{I.boxes,0});
	std::sort(P.b, P.e, BoxesCompare{P.boxes,0});
	while(I.b != I.e && P.b != P.e) {
	    if(lt_sos(I.xmin(I.b,0), P.xmin(P.b,0), *I.b, *P.b)) {
		for(index_t* p=P.b; p!=P.e && P.xmin(p,0)<=I.xmax(I.b,0); ++p) {
		    if(P.II_isect_1tod(p,I,I.b,d) && I.I_contains_P(I.b,P,p,d)) {
			report_isect(swap_ip ? *p : *I.b, swap_ip ? *I.b : *p);
		    }
		}
		++I.b;
	    } else {
		for(index_t* i=I.b; i!=I.e && I.xmin(i,0)<=P.xmax(P.b,0); ++i) {
		    if(I.II_isect_1tod(i,P,P.b,d) && I.I_contains_P(i,P,P.b,d)) {
			report_isect(swap_ip ? *P.b : *i, swap_ip ? *i : *P.b);
		    }
		}
		++P.b;
	    }
	}
    }

    /************************************************************************/

    /**
     * \brief given three boxes seen as points, gets the median one for a
     *  given coordinate
     * \param[in] boxes pointer to an array of boxes
     * \param[in] a , b , c pointers to three indices referring to three boxes
     *  in \p boxes
     * \param[in] dim the dimension along which boxes are sorted
     * \return a pointer to the index of the median box relative to the order
     *  of their lower points along dimension \p dim
     */
    index_t* median_of_three(
	const Box3d* boxes, index_t* a, index_t* b, index_t* c, index_t dim
    ) {
	BoxesCompare C{boxes, dim};
	if(C(*a,*b)) {
	    if(C(*b,*c)) {
		return b;
	    } else if(C(*a,*c)) {
		return c;
	    } else {
		return a;
	    }
	} else if(C(*a,*c)) {
	    return a;
	} else if(C(*b,*c)) {
	    return c;
	} else {
	    return b;
	}
    }

    /************************************************************************/

    /**
     * \brief computes the approximate median of a set of boxes along a given
     *  dimension
     * \param[in] boxes pointer to an array of boxes
     * \param[in] b , e pointers to a sequence of box indices
     * \param[in] d dimension along with boxes are stored
     * \param[in] levels number of levels in the approximation of the median
     * \param[in] rng a random number generator
     * \details see
     *    Approximating center points with iterative radon points
     *      K. L. Clarkson, D. Eppstein, G. L. Miller, C. Sturtivant
     *        and S- H. Teng.
     *      International Journal of Computational Geometry & Applications
     *      6 (1996) 357–377
     */
    index_t* approximate_median(
	const Box3d* boxes, index_t* b, index_t* e, index_t d, int levels,
	RandomEngine& rng
    ) {
	if(levels < 0) {
	    auto N = std::distance(b,e);
	    geo_assert(N >= 1);
	    return b + size_t(
		std::uniform_int_distribution<index_t>(0,index_t(N-1))(rng)
	    );
	}
	return median_of_three(
	    boxes,
	    approximate_median(boxes, b, e, d, levels-1, rng),
	    approximate_median(boxes, b, e, d, levels-1, rng),
	    approximate_median(boxes, b, e, d, levels-1, rng),
	    d
	);
    }

    /***************************************************************************/

    /**
     * \brief Splits a BoxesRange seen as points along a given dimension
     * \details this reorders the elements in the input BoxesRange
     * \param[in] P the BoxesRange seen as points to be split
     * \param[in] d the dimension along which to split \p P
     * \param[in] rng a random number generator
     * \return a triple (P1, x, P2) where P1 and P2 are the constructed
     *  subranges and x the approximate median coordinate.
     */
    std::tuple<BoxesRange, double, BoxesRange> split_points(
	BoxesRange& P, index_t d, RandomEngine& rng
    ) {
	index_t N = P.size();
	int levels = int(0.91 * std::log(double(N)/137.035999206)+1.0);
	levels = (levels <= 0) ? 1 : levels;
	index_t* m = approximate_median(P.boxes, P.b, P.e, d, levels, rng);
	double px_m = P.xmin(m,d);
	// a BoxesRange is just three pointers, so there is no harm returning
	// two of them by value.
	std::pair<BoxesRange, BoxesRange> P1P2 = P.split(
	    [&P, d, px_m](index_t i)->bool {
		return P.xmin(i,d) < px_m;
	    }
	);
	return std::make_tuple(P1P2.first, px_m, P1P2.second);
    }

    /***************************************************************************/

    /**
     * \brief Optimized algorithm that reports all pairs (i,p) such that
     *  boxes i and p have intersections in two sets of boxes I and P. Much
     *  faster than one_way_scan().
     * \details the recursion is initiated as follows
     *   \code
     *   hybrid(
     *	    I,P,2,callback,false,
     *	    -std::numeric_limits<double>::max(),
     *	    std::numeric_limits<double>::max(),
     *	    1000
     *	 );
     *   \endcode
     *   This is the main algorithm described in:
     *      Fast software for box intersections,
     *      Afra Zoromodian and Herbert Edelsbrunner,
     *      International Journal of Computational Geometry & Applications, 2002
     * \pre each p in P belongs to [lo,hi) and each i in I intersects [lo,hi)
     * \param[in] I a set of boxes
     * \param[in] P a set of boxes
     * \param[in] d maximum dimension to be tested
     * \param[in] report_isect the callback used to report intersections, takes
     *  two integers, i and p
     * \param[in] swap_ip if set, the parameters i,p of the callback are swapped
     * \param[in] lo , hi range of coordinates
     * \param[in] cutoff if one of \p I, \p P is smaller than \p cutoff, then
     *  modified_two_way_scan() is used instead.
     * \param[in] rng a random number generator
     */

    void hybrid(
	BoxesRange I, BoxesRange P, index_t d,
	std::function<void(index_t,index_t)> report_isect, bool swap_ip,
	double lo, double hi, index_t cutoff, RandomEngine& rng
    ) {

	static constexpr double inf = -std::numeric_limits<double>::max();
	static constexpr double sup =  std::numeric_limits<double>::max();

        #ifdef GEO_DEBUG // check preconditions
	for(index_t* p = P.b; p != P.e; ++p) { 	// Each p belongs to [lo,hi)
	    geo_debug_assert(P.xmin(p,d) >= lo && P.xmin(p,d) < hi);
	}
	for(index_t* i = I.b; i != I.e; ++i) { 	// Each i intersects [lo,hi)
	    geo_debug_assert(I.xmin(i,d) < hi && I.xmax(i,d) >= lo);
	}
	#endif

	if( I.empty() || P.empty() || lo >= hi ) {
	    return;
	}

	// First hybridization: scan instead of third level of segment tree
	if(d == 0) {
	    one_way_scan(I, P, d, report_isect, swap_ip);
	    return;
	}

	// Second hybridization: cutoffs to switch to scanning
	if(I.size() < cutoff || P.size() < cutoff) {
	    modified_two_way_scan(I, P, d, report_isect, swap_ip);
	    return;
	}

	// Split I into the parts Ispan that span [lo,hi] and the rest Inonspan
	// Ispan correspond to the list of segments that would be stored in
	// current node if using a standard representation of a segment tree.
	auto [Ispan, Inonspan] = I.split(
	    [&I,d,lo,hi](index_t i)->bool{
		return (I.xmin(i,d) < lo && I.xmax(i,d) > hi);
	    },
	    lo != inf && hi != sup
	);

	// Two calls for roots of the segment tree at the next level
	if(!Ispan.empty()) {
	    hybrid(Ispan, P, d-1, report_isect,  swap_ip, inf, sup, cutoff, rng);
	    hybrid(P, Ispan, d-1, report_isect, !swap_ip, inf, sup, cutoff, rng);
	}

	// divide [lo,hi) into [lo, mi) and [mi, hi)
	// Pl is the sef of points contained in the left subsegment [lo,mi)
	// Pr is the sef of points contained in the left subsegment [mi,hi)
	auto [Pl, mi, Pr] = split_points(P, d, rng);

	// Special case: unable to split points (fallback: modified_two_way_scan)
	if(Pl.empty() || Pr.empty()) {
	    modified_two_way_scan(Inonspan, P, d, report_isect, swap_ip);
	    return;
	}

	// Il is the sef of intervals that intersect the left subsegment [lo,mi)
	// but that do not span the entire segment [lo,hi)
	BoxesRange Il = Inonspan.split(
	    [&Inonspan, d, mi](index_t i)->bool {
		return (Inonspan.xmin(i,d) < mi);
	    }
	).first;
	hybrid(Il, Pl, d, report_isect, swap_ip, lo, mi, cutoff, rng);

	// Ir is the sef of intervals that intersect the right subsegment [mi,hi)
	// but that do not span the entire segment [lo,hi)
	// Note that Il and Ir are usually not disjoint.
	BoxesRange Ir = Inonspan.split(
	    [&Inonspan, d, mi](index_t i)->bool {
		return (Inonspan.xmax(i,d) >= mi);
	    }
	).first;
	hybrid(Ir, Pr, d, report_isect, swap_ip, mi, hi, cutoff, rng);
    }

    /**
     * \brief Parallel version of boxes_intersections()
     * \details Used by boxes_intersections() for large datasets
     * \see boxes_intersections()
     */
    void boxes_intersections_parallel(
	const vector<Box3d>& boxes,
	std::function<void(index_t, index_t)> callback
    ) {

	std::vector<index_t> idx(boxes.size());
	for(index_t i=0; i<boxes.size(); ++i) {
	    idx[i] = i;
	}
	std::vector<index_t> pdx(idx);

	// I and P will be subdivided into nI and nP subsets respectively,
	// then intersections will be computed in parallel on each nI*nP couple
	index_t nI = 6;
	index_t nP = 6;


	// Create nP copies of I and nI copies of P, so that
	// each nI*nP thread can manipulate its own copy of I and P
	std::vector<index_t> idx_copies;
	idx_copies.reserve(idx.size()*nP);
	std::vector<index_t> pdx_copies;
	pdx_copies.reserve(pdx.size()*nI);
	for(index_t p=0; p<nP; ++p) {
	    idx_copies.insert(idx_copies.end(), idx.begin(), idx.end());
	}
	for(index_t i=0; i<nI; ++i) {
	    pdx_copies.insert(pdx_copies.end(), pdx.begin(), pdx.end());
	}

	struct Job {
	    BoxesRange I;
	    BoxesRange P;
	    RandomEngine rng;
	    vector<std::pair<index_t, index_t>> intersections;
	};

	index_t I_size = index_t(idx.size());
	index_t I_batch_size = index_t(I_size/nI);
	index_t P_size = index_t(pdx.size());
	index_t P_batch_size = index_t(P_size/nP);

	// Job (i,p) I indices are [ I_ptr(i,p) ... I_ptr(i+1,p) )
	auto I_ptr = [&](index_t i, index_t p)->index_t* {
	    return idx_copies.data() + (
		p*I_size + ((i==nI) ? I_size : (i * I_batch_size))
	    );
	};

	// Job (i,p) P indices are [ P_ptr(i,p) ... P_ptr(i,p+1) )
	auto P_ptr = [&](index_t i, index_t p)->index_t* {
	    return pdx_copies.data() + (
		i*P_size + ((p==nP) ? P_size : (p * P_batch_size))
	    );  // :-) -------^
	};

	// Initialize jobs
	std::random_device rd;
	std::vector<Job> jobs(nI*nP);
	for(index_t p=0; p<nP; ++p) {
	    for(index_t i=0; i<nI; ++i) {
		jobs[p*nI+i].rng = RandomEngine(rd());
		jobs[p*nI+i].I=BoxesRange{boxes.data(),I_ptr(i,p),I_ptr(i+1,p)};
		jobs[p*nI+i].P=BoxesRange{boxes.data(),P_ptr(i,p),P_ptr(i,p+1)};
	    }
	}

	// Let's rock and roll !
	parallel_for(
	    0, nI*nP, [&jobs](index_t j) {
		Job& J = jobs[j];
		hybrid(
		    J.I,J.P,2,
		    [&J](index_t i, index_t j) {
			J.intersections.emplace_back(i,j);
		    },
		    false,
		    -std::numeric_limits<double>::max(),
		     std::numeric_limits<double>::max(),
		    1000,
		    J.rng
		);
	    }
	);

	// We could also have called callback() asynchronously and handled
	// concurrency with a lock but it seems to be faster to store
	// intersections in one vector per thread.
	for(const auto& job: jobs) {
	    for(const auto& ij: job.intersections) {
		callback(ij.first, ij.second);
	    }
	}
    }

}

/******************************************************************************/

namespace GEO {

    void boxes_intersections(
	const vector<Box3d>& boxes,
	std::function<void(index_t, index_t)> callback
    ) {
	if(boxes.size() > 1024) {
	    boxes_intersections_parallel(boxes, callback);
	    return;
	}

	std::random_device rd;
	RandomEngine rng(rd());

	std::vector<index_t> idx(boxes.size());
	for(index_t i=0; i<boxes.size(); ++i) {
	    idx[i] = i;
	}
	std::vector<index_t> pdx(idx);
	BoxesRange I{boxes.data(), idx.data(), idx.data()+idx.size()};
	BoxesRange P{boxes.data(), pdx.data(), pdx.data()+pdx.size()};
	hybrid(
	    I,P,2,callback,false,
	    -std::numeric_limits<double>::max(),
	    std::numeric_limits<double>::max(),
	    1000,
	    rng
	);

    }

}
