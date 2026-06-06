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

//  TODO: pass Box array for both sides !

namespace {
    using namespace GEO;


    /**
     * \brief Comparator class for sorting boxes along various dimensions
     */
    struct BoxesCompare {
	/**
	 * \brief Compares two boxes
	 * \param[in] ida , idb the indices of the two boxes to be compared
	 * \retval true if box \p ida is before \p idb
	 * \retval false otherwise
	 * \details compares the coordinate d of the lower bound. In case of
	 *  equality, indices are compared instead.
	 */
	bool operator() (index_t ida, index_t idb) const {
	    double a = boxes[ida].xyz_min[d];
	    double b = boxes[idb].xyz_min[d];
	    return (a < b || (a == b && ida < idb));
	}

	const Box3d* boxes;
	index_t d;
    };


    /**
     * \brief Predicates for box-box intersections
     */
    struct BoxesPredicates {
	/**
	 * \brief Tests whether boxes as interval intersect along all coordinates
	 *  from 1 to a given upper bound
	 * \param[in] pa , pb pointers to the two indices of the two boxes
	 * \param[in] d upper bound of coordinates to be tested
	 * \retval true if all coordinate invervals intersect from 1 to d
	 * \retval false otherwise
	 */
	bool II_isect_1tod(index_t* pa, index_t* pb, index_t d) const {
	    if(*pa == *pb) {
		return false;
	    }
	    for(index_t dim=1; dim<=d; ++dim) {
		if(maxb(pa,dim) < minb(pb,dim) || maxb(pb,dim) < minb(pa,dim)) {
		    return false;
		}
	    }
	    return true;
	}

	/**
	 * \brief Tests whether a box as interval contains a point for a given
	 *  coordinate
	 * \details does symbolic perturbation on the left bound of the interval,
	 *  like in BoxCompare
	 * \param[in] pi , pp pointers to the indices of the two boxes
	 * \param[in] d coordinate to be tested
	 * \retval true if \p pi as an interval contains \p pp for coord \p d
	 * \retval false otherwise
	 */
	bool I_contains_P(index_t* pi, index_t* pp, index_t d) const {
	    double imin = minb(pi,d);
	    double imax = maxb(pi,d);
	    double p = minb(pp,d);
	    return ((imin < p || (imin == p && *pi < *pp))) && (imax >= p);
	}

	/**
	 * \brief gets the lower bound of a box for a given coordinate
	 * \param[in] b a pointer to the index of the box
	 * \param[in] d the coordinate
	 * \return the coordinate \p d of the lower bound of box \p b
	 */
	double minb(index_t* b, index_t d) const {
	    return boxes[*b].xyz_min[d];
	}

	/**
	 * \brief gets the upper bound of a box for a given coordinate
	 * \param[in] b a pointer to the index of the box
	 * \param[in] d the coordinate
	 * \return the coordinate \p d of the upper bound of box \p b
	 */
	double maxb(index_t* b, index_t d) const {
	    return boxes[*b].xyz_max[d];
	}

	const Box3d* boxes;
    };

    /************************************************************************/

    void one_way_scan(
	const Box3d* boxes,
	index_t* i_b, index_t* i_e, // boxes viewed as intervals
	index_t* p_b, index_t* p_e, // boxes viewed as points
	index_t d,
	std::function<void(index_t,index_t)> report_isect,
	bool swap_ip = false
    ) {
	BoxesCompare C{boxes,0};
	BoxesPredicates P{boxes};
	std::sort(i_b, i_e, C);
	std::sort(p_b, p_e, C);
	for(index_t* i = i_b; i != i_e; ++i) {
	    while(p_b != p_e && C(*p_b, *i)) {
		++p_b;
	    }
	    for(index_t* p=p_b; p!=p_e && P.minb(p,0)<=P.maxb(i,0); ++p) {
		if(P.II_isect_1tod(p, i, d)) {
		    report_isect(swap_ip ? *p : *i, swap_ip ? *i : *p);
		}
	    }
	}
    }

    /************************************************************************/

    void modified_two_way_scan(
	const Box3d* boxes,
	index_t* i_b, index_t* i_e, // boxes viewed as intervals
	index_t* p_b, index_t* p_e, // boxes viewed as points
	index_t d,
	std::function<void(index_t,index_t)> report_isect,
	bool swap_ip = false
    ) {
	BoxesCompare C{boxes,0};
	BoxesPredicates P{boxes};
	std::sort(i_b, i_e, C);
	std::sort(p_b, p_e, C);
	while(i_b != i_e && p_b != p_e) {
	    if(C(*i_b, *p_b)) {
		for(index_t* p=p_b; p!=p_e && P.minb(p,0)<=P.maxb(i_b,0); ++p) {
		    if(P.II_isect_1tod(p, i_b, d) && P.I_contains_P(i_b, p, d)) {
			report_isect(swap_ip ? *p : *i_b, swap_ip ? *i_b : *p);
		    }
		}
		++i_b;
	    } else {
		for(index_t* i=i_b; i!=i_e && P.minb(i,0)<=P.maxb(p_b,0); ++i) {
		    if(P.II_isect_1tod(i, p_b, d) && P.I_contains_P(i, p_b, d)) {
			report_isect(swap_ip ? *p_b : *i, swap_ip ? *i : *p_b);
		    }
		}
		++p_b;
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

    /************************************************************************/

    index_t* split_points(
	const Box3d* boxes, index_t* b, index_t* e, index_t d, double& mi
    ) {
	auto N = std::distance(b,e);
	int levels = int(0.91 * std::log(double(N)/137.035999710)+1.0);
	levels = (levels <= 0) ? 1 : levels;
	index_t* m = approximate_median(boxes, b, e, d, levels);
	mi = boxes[*m].xyz_min[d];
	return std::partition(
	    b, e,
	    [boxes, d, mi](index_t b)->bool {
		return boxes[b].xyz_min[d] < mi;
	    }
	);
    }

    /************************************************************************/

    void hybrid(
	const Box3d* boxes,
	index_t* i_b, index_t* i_e,
	index_t* p_b, index_t* p_e,
	index_t d,
	std::function<void(index_t,index_t)> report_isect,
	bool swap_ip,
	double lo, double hi,
	ptrdiff_t cutoff
    ) {
	static constexpr double inf = -std::numeric_limits<double>::max();
	static constexpr double sup =  std::numeric_limits<double>::max();

	#ifdef GEO_DEBUG
	// Each p belongs to [lo,hi)
	for(index_t* p = p_b; p != p_e; ++p) {
	    double x = boxes[*p].xyz_min[d];
	    geo_debug_assert(x >= lo && x < hi);
	}
	// Each i intersects [lo,hi)
	for(index_t* i = i_b; i != i_e; ++i) {
	    double imin = boxes[*i].xyz_min[d];
	    double imax = boxes[*i].xyz_max[d];
	    geo_debug_assert(imin < hi && imax >= lo);
	}
	#endif

	if( i_b == i_e || p_b == p_e || lo >= hi ) {
	    return;
	}

	if(d == 0) {
	    one_way_scan(boxes, i_b, i_e, p_b, p_e, d, report_isect, swap_ip);
	    return;
	}

	if(std::distance(i_b, i_e)<cutoff || std::distance(p_b, p_e)<cutoff) {
	    modified_two_way_scan(
		boxes, i_b, i_e, p_b, p_e, d, report_isect, swap_ip
	    );
	    return;
	}

	index_t* i_span_end = i_b;
	if(lo != inf && hi != sup) {
	    i_span_end = std::partition(
		i_b, i_e, [boxes,d,lo,hi](index_t b)->bool{
		    return (
			boxes[b].xyz_min[d] < lo &&
			boxes[b].xyz_max[d] > hi
		    );
		}
	    );
	}

	if(i_span_end != i_b) {
	    hybrid(
		boxes, i_b, i_span_end, p_b, p_e, d-1, report_isect, swap_ip,
		inf, sup, cutoff
	    );
	    hybrid(
		boxes, p_b, p_e, i_b, i_span_end, d-1, report_isect, !swap_ip,
		inf, sup, cutoff
	    );
	}

	double mi;
	index_t* p_mid = split_points(boxes, p_b, p_e, d, mi);

	// Special case: unable to split points (fallback: modified_two_way_scan)
	if(p_mid == p_b || p_mid == p_e) {
	    modified_two_way_scan(
		boxes, i_span_end, i_e, p_b, p_e, d, report_isect, swap_ip
	    );
	    return;
	}

	index_t* i_mid = std::partition(
	    i_span_end, i_e, [boxes, d, mi](index_t b)->bool {
		return (boxes[b].xyz_min[d] < mi);
	    }
	);

	hybrid(
	    boxes, i_span_end, i_mid, p_b, p_mid, d, report_isect, swap_ip,
	    lo, mi, cutoff
	);

	i_mid = std::partition(
	    i_span_end, i_e, [boxes, d, mi](index_t b)->bool {
		return (boxes[b].xyz_max[d] >= mi);
	    }
	);

	hybrid(
	    boxes, i_span_end, i_mid, p_mid, p_e, d, report_isect, swap_ip,
	    mi, hi, cutoff
	);

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

	/*
	one_way_scan(
	    boxes.data(),
	    idx.data(), idx.data()+idx.size(),
	    pdx.data(), pdx.data()+pdx.size(),
	    2,
	    callback,
	    false
	);
	*/

	/*
	modified_two_way_scan(
	    boxes.data(),
	    idx.data(), idx.data()+idx.size(),
	    pdx.data(), pdx.data()+pdx.size(),
	    2,
	    callback,
	    false
	);
	*/

	hybrid(
	    boxes.data(),
	    idx.data(), idx.data()+idx.size(),
	    pdx.data(), pdx.data()+pdx.size(),
	    2,
	    callback,
	    false,
	    -std::numeric_limits<double>::max(),
	    std::numeric_limits<double>::max(),
	    1000
	);

    }

}
