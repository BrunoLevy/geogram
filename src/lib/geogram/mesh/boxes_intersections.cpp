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
#include <random>

namespace {
    using namespace GEO;


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
	    if(boxes == J.boxes && *i == *j) {
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

    index_t* approximate_median(
	const Box3d* boxes, index_t* b, index_t* e, index_t d, int levels
    ) {
	static std::random_device rd;
	static std::mt19937_64 random_engine(rd());
	if(levels < 0) {
	    auto N = std::distance(b,e);
	    geo_assert(N >= 1);
	    return b +
		std::uniform_int_distribution<size_t>(0,size_t(N-1))(
		    random_engine
		);
	}
	return median_of_three(
	    boxes,
	    approximate_median(boxes, b, e, d, levels-1),
	    approximate_median(boxes, b, e, d, levels-1),
	    approximate_median(boxes, b, e, d, levels-1),
	    d
	);
    }

    /***************************************************************************/

    std::tuple<BoxesRange, double, BoxesRange> split_points(
	BoxesRange& P, index_t d
    ) {
	index_t N = P.size();
	int levels = int(0.91 * std::log(double(N)/137.035999206)+1.0);
	levels = (levels <= 0) ? 1 : levels;
	index_t* m = approximate_median(P.boxes, P.b, P.e, d, levels);
	double px_m = P.xmin(m,d);
	std::pair<BoxesRange, BoxesRange> P1P2 = P.split(
	    [&P, d, px_m](index_t i)->bool {
		return P.xmin(i,d) < px_m;
	    }
	);
	return std::make_tuple(P1P2.first, px_m, P1P2.second);
    }

    /***************************************************************************/

    void hybrid(
	BoxesRange I, BoxesRange P, index_t d,
	std::function<void(index_t,index_t)> report_isect, bool swap_ip,
	double lo, double hi, index_t cutoff
    ) {

	static constexpr double inf = -std::numeric_limits<double>::max();
	static constexpr double sup =  std::numeric_limits<double>::max();

	#ifdef GEO_DEBUG
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

	if(d == 0) {
	    one_way_scan(I, P, d, report_isect, swap_ip);
	    return;
	}

	if(I.size()  < cutoff || P.size() < cutoff) {
	    modified_two_way_scan(I, P, d, report_isect, swap_ip);
	    return;
	}

	auto [Ispan, Inonspan] = I.split(
	    [&I,d,lo,hi](index_t i)->bool{
		return (I.xmin(i,d) < lo && I.xmax(i,d) > hi);
	    },
	    lo != inf && hi != sup
	);

	if(!Ispan.empty()) {
	    hybrid(Ispan, P, d-1, report_isect,  swap_ip, inf, sup, cutoff);
	    hybrid(P, Ispan, d-1, report_isect, !swap_ip, inf, sup, cutoff);
	}

	auto [P1, px_m, P2] = split_points(P,d);

	// Special case: unable to split points (fallback: modified_two_way_scan)
	if(P1.empty() || P2.empty()) {
	    modified_two_way_scan(Inonspan, P, d, report_isect, swap_ip);
	    return;
	}

	BoxesRange I1 = Inonspan.split(
	    [&Inonspan, d, px_m](index_t b)->bool {
		return (Inonspan.xmin(b,d) < px_m);
	    }
	).first;
	hybrid(I1, P1, d, report_isect, swap_ip, lo, px_m, cutoff);

	I1 = Inonspan.split(
	    [&Inonspan, d, px_m](index_t b)->bool {
		return (Inonspan.xmax(b,d) >= px_m);
	    }
	).first;
	hybrid(I1, P2, d, report_isect, swap_ip, px_m, hi, cutoff);
    }

}

/******************************************************************************/

namespace GEO {

    void boxes_intersections(
	const vector<Box3d>& boxes,
	std::function<void(index_t, index_t)> callback
    ) {
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
	    1000
	);

    }

}
