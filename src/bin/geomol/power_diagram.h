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

#ifndef POWER_DIAGRAM
#define POWER_DIAGRAM

#include <geogram/delaunay/periodic_delaunay_3d.h>
#include <geogram/basic/range.h>

namespace GEO {

    /**
     * \brief A PowerDiagram encoded as a weighted Delaunay triangulation
     * \details Under the hood, uses PeriodicDelaunay3d,
     *   that does periodic-or-not (here not) weighted-or-not (here weighted)
     *   triangulations.
     *   PowerDiagram has functions to navigate the triangulation based on
     *   halfedges.
     *   Each tetrahedron has 12 halfedges. An halfedge is a triplet (T,t,e)
     *   encoded in an index_t, where:
     *      - T is a global tetrahedron index
     *      - t in {0,1,2,3} is a local facet index in the tetrahedron
     *      - e in {0,1,2} is a local edge index in the facet
     *   PowerDiagram also has a function compute_skeleton() that computes
     *   the Delaunauy skeleton, that is, for each vertex v, the list of
     *   halfedges emanating from v. Once compute_skeleton() has been called,
     *   the Delaunay skeleton can be traversed as follows:
     *   \code
     *      for(index_t h: incident_edges(v)) {
     *            do something with h
     *         }
     *      }
     *   \endcode
     */
    class PowerDiagram : public PeriodicDelaunay3d {
    public:

	/**
	 * \brief PowerDiagram constructor
	 */
	PowerDiagram() : PeriodicDelaunay3d(false) {
	    set_keeps_infinite(true);
	}

	/**
	 * \brief gets the number of tetrahedra
	 * \return the total number of tetrahedra, including the infinite ones
	 */
	index_t nb_tets() const {
	    return nb_cells();
	}

	index_range tets() const {
	    return index_range(0, nb_tets());
	}

	index_range vertices() const {
	    return index_range(0, nb_vertices());
	}

	/**
	 * \brief tests whether a tetrahedron is infinite
	 * \retval true if one of the vertices is the vertex at infinity
	 * \retval false if all the vertices are real vertices
	 */
	bool tet_is_infinite(index_t t) const {
	    return (
		tet_vertex(t,0) == NO_INDEX ||
		tet_vertex(t,1) == NO_INDEX ||
		tet_vertex(t,2) == NO_INDEX ||
		tet_vertex(t,3) == NO_INDEX
	    );
	}

	/**
	 * \brief tests whether a tetrahedron is finite
	 * \retval true if all the vertices of this tetrahedron are real vertices
	 * \retval false if one of the vertices is the vertex at infinity
	 */
	bool tet_is_finite(index_t t) const {
	    return !tet_is_infinite(t);
	}

	/**
	 * \brief Gets a vertex of a tetrahedron
	 * \param[in] t a tetrahedron, in 0 .. nb_tets() - 1
	 * \param[in] lv a local vertex index, in 0 .. 3
	 * \return the global vertex index corresponding to vertex \p lv of
	 *  tetrahedron \p t, or NO_INDEX if it is the vertex at infinity
	 */
	index_t tet_vertex(index_t t, index_t lv) const {
	    // what follows is an optimized version of
	    //   return cell_vertex(t,lv);
	    geo_debug_assert(t < nb_tets());
	    geo_debug_assert(lv < 4);
	    return cell_to_v_[t*4+lv];
	}

	/**
	 * \brief Gets a tetrahedron adjacent to another tetrahedron
	 * \param[in] t a tetrahedron, in 0 .. nb_tets() - 1
	 * \param[in] lf a local facet index, in 0 .. 3
	 * \return the tetrahedron adjacent to \p t accross facet \p lf
	 */
	index_t tet_adjacent(index_t t, index_t lf) const {
	    // what follows is an optimized version of
	    //   return cell_adjacent(t,lf);
	    geo_debug_assert(t < nb_tets());
	    geo_debug_assert(lf < 4);
	    return cell_to_cell_[t*4+lf];
	}

	/**
	 * \brief Finds the local index of a vertex in a tetrahedron
	 * \pre \p t is incident to \p v
	 * \param[in] t a tetrahedron, in 0 .. nb_tets() - 1
	 * \param[in] v a global vertex index
	 * \return lv such that tet_vertex(t,lv) == v
	 */
	index_t find_tet_vertex(index_t t, index_t v) const {
            const index_t* T = &(cell_to_v_[4 * t]);
            return find_4(T,v);
	}

	/**
	 * \brief Finds the local facet index accross which a tetrahedron
	 *  is adjacent to another one
	 * \pre \p t1 is adjacent to \p t2
	 * \param[in] t1 a tetrahedron, in 0 .. nb_tets() - 1
	 * \param[in] t2 a tetrahedron, in 0 .. nb_tets() - 1
	 * \return lf such that tet_adjacent(t1,lf) == t2
	 */
	index_t find_tet_adjacent(index_t t1, index_t t2) const {
            const index_t* T = &(cell_to_cell_[4 * t1]);
            return find_4(T,t2);
	}

	/**
	 * \brief Gets a local vertex index from a local facet index
	 *   and local vertex index in the facet
	 * \param[in] lf a local facet index, in 0 .. 3
	 * \param[in] lv a local vertex index in the facet, in 0 .. 2
	 * \return the local vertex index in the tetrahedron, in 0 .. 3
	 */
	static constexpr index_t tet_facet_lv(index_t lf, index_t lv) {
	    geo_debug_assert(lf < 4);
	    geo_debug_assert(lv < 3);
	    return index_t(tet_facet_vertex_[lf][lv]);
	}

	/**
	 * \brief Gets a global vertex index from a tetrahedron,
	 *   a local facet index in the tetrahedron and a local vertex
	 *   index in the facet
	 * \param[in] t a tetrahedron, in 0 .. nb_tets() - 1
	 * \param[in] lf a local facet index, in 0 .. 3
	 * \param[in] lv a local vertex index in the facet, in 0 .. 2
	 * \return the global vertex index
	 */
	index_t tet_facet_vertex(index_t t, index_t lf, index_t lv) const {
	    geo_debug_assert(lf < 4);
	    geo_debug_assert(lv < 3);
	    return tet_vertex(t, tet_facet_lv(lf, lv));
	}

	/**
	 * \brief Makes a global halfedge index from a tetrahedron, a local
	 *  facet index in the tetrahedron and a local edge index in the facet
	 * \param[in] t a tetrahedron, in 0 .. nb_tets() - 1
	 * \param[in] lf a local facet index, in 0 .. 3
	 * \param[in] le a local edge index in the facet, in 0 .. 2
	 * \return the global halfedge index
	 */
	index_t make_halfedge_from_t_lf_le(
	    index_t t, index_t lf, index_t le
	) const {
	    geo_debug_assert(t < nb_tets());
	    geo_debug_assert(lf < 4);
	    geo_debug_assert(le < 3);
	    return (t << 4) | (lf << 2) | le;
	}

	/**
	 * \brief Makes a global halfedge index from a tetrahedron and a local
	 *  halfedge index in the tetrahedron
	 * \param[in] t a tetrahedron, in 0 .. nb_tets() - 1
	 * \param[in] lh a local halfedge index, in 0..15 (only 12 valid values)
	 * \return the global halfedge index
	 */
	index_t make_halfedge_from_t_lh(index_t t, index_t lh) const {
	    geo_debug_assert(t < nb_tets());
	    geo_debug_assert(lh < 16);
	    return (t << 4) | lh;
	}

	/**
	 * \brief Makes a global halfedge index from a tetrahedron and two
	 *  local vertex indices in the tetrahedron
	 * \param[in] t a tetrahedron, in 0 .. nb_tets() - 1
	 * \param[in] lv1 , lv2 the local indices of the extremities of the
	 *  halfedge in the tetrahedron, in 0..3
	 * \return the global halfedge index
	 */
	index_t make_halfedge_from_t_lv_lv(
	    index_t t, index_t lv1, index_t lv2
	) const {
	    geo_debug_assert(lv1 < 4);
	    geo_debug_assert(lv2 < 4);
	    geo_debug_assert(lv1 != lv2);
	    index_t lh = index_t(vv2h_[lv1][lv2]);
	    return make_halfedge_from_t_lh(t,lh);
	}

	/**
	 * \brief Gets the local halfedge index from a global halfedge index
	 * \param[in] h a global halfedge index
	 * \return the local halfedge index, in 0..15 (only 12 valid values)
	 */
	static index_t halfedge_lh(index_t h) {
	    return h & 15u;
	}

	/**
	 * \brief Gets the local facet index from a global halfedge index
	 * \param[in] h a global halfedge index
	 * \return the local facet index, in 0..3
	 */
	static index_t halfedge_lf(index_t h) {
	    return (h & 12u) >> 2;
	}

	/**
	 * \brief Gets the local edge index from a global halfedge index
	 * \param[in] h a global halfedge index
	 * \return the local edge index, in 0..2, relative to the facet
	 *  returned by halfedge_lf(h)
	 */
	static index_t halfedge_le(index_t h) {
	    return h & 3u;
	}

	/**
	 * \brief Gets the tetrahedron index from a global halfedge index
	 * \param[in] h a global halfedge index
	 * \return the global index of the tetrahedron that \p h is adjacent to
	 */
	static index_t halfedge_t(index_t h) {
	    return h >> 4;
	}

	/**
	 * \brief Gets the global vertex index of one of the extremities of an
	 *  halfedge
	 * \param[in] h a global halfedge index
	 * \param[in] lv one of 0, 1
	 * \return the global index of the first or second extremity of \p h,
	 *  depending on \p lv
	 */
	index_t halfedge_v(index_t h, index_t lv) const {
	    geo_debug_assert(lv < 2);
	    index_t t = halfedge_t(h);
	    geo_debug_assert(t < nb_tets());
	    index_t lh = halfedge_lh(h);
	    geo_debug_assert(lh < 16);
	    lv = index_t(h2v_[lh][lv]);
	    geo_debug_assert(lv != ZZ);
	    return tet_vertex(t,lv);
	}

	/**
	 * \brief Flips a halfedge
	 * \param[in] h a global halfedge index
	 * \return the global halfedge index of the unique halfedge in the
	 *  same tetrahedron as \p h but with its extremities swapped
	 */
	index_t halfedge_flip(index_t h) const {
	    index_t t = halfedge_t(h);
	    geo_debug_assert(t < nb_tets());
	    index_t lh = halfedge_lh(h);
	    geo_debug_assert(lh < 16);
	    index_t lv1 = index_t(h2v_[lh][0]);
	    index_t lv2 = index_t(h2v_[lh][1]);
	    return make_halfedge_from_t_lv_lv(t,lv2,lv1);
	}

	/**
	 * \brief Traverses the halfedges incident to the same edge
	 * \param[in] h a global halfedge index
	 * \param[in] v1 , v2 the global vertex indices of the two
	 *  extremities of \p h
	 * \return the next halfedge around the edge (\p v1, \p v2)
	 * \pre v1 == halfedge_v(h,0) && v2 == halfedge_v(h,1)
	 * \details There is also a variant that does not take \p v1 and
	 *  \p v2 as arguments
	 */
	index_t next_halfedge_around_edge(
	    index_t h, index_t v1, index_t v2
	) const {
	    geo_debug_assert(v1 == halfedge_v(h,0));
	    geo_debug_assert(v2 == halfedge_v(h,1));
	    index_t t = halfedge_t(h);
	    geo_debug_assert(t < nb_tets());
	    index_t lf = halfedge_lf(h);
	    index_t t2 = tet_adjacent(t,lf);
	    index_t lv1 = find_tet_vertex(t2,v1);
	    index_t lv2 = find_tet_vertex(t2,v2);
	    return make_halfedge_from_t_lv_lv(t2, lv1, lv2);
	}

	/**
	 * \brief Traverses the halfedges incident to the same edge
	 * \param[in] h a global halfedge index
	 * \return the next halfedge around the edge defined by the
	 *  extremities of \p h
	 */
	index_t next_halfedge_around_edge(index_t h) const {
	    return next_halfedge_around_edge(
		h, halfedge_v(h,0), halfedge_v(h,1)
	    );
	}

	/**
	 * \brief Computes the Delaunay skeleton
	 * \param[in] max_v one position past the maximum vertex index
	 *  for which the Delaunay skeleton should be stored, or NO_INDEX
	 *  if it should be stored for all vertices
	 * \see incident_edges(), skel_begin(), skel_end(), skel_h()
	 */
	void compute_skeleton(index_t max_v = NO_INDEX);

	/**
	 * \short gets the list of halfedges incident to a vertex
	 * \param v a global vertex index, smaller than the parameter max_v
	 *   passed compute_skeleton()
	 * \return a sequence of halfedges
	 * \details the list of incident edges can be traversed as follows:
	 * \code
	 *   for(index_t h: incident_edges(v)) {
	 *         do something with h
	 *      }
	 *   }
	 * \endcode
	 */
	auto incident_edges(index_t v) const {
	    return transform_range(
		index_range(skel_begin(v), skel_end(v)),
		[this](index_t k)->index_t {
		    return skel_h(k);
		}
	    );
	}

	/**
	 * \brief Gets the number of edges starting from a vertex
	 * \param[in] v the vertex
	 */
	index_t nb_incident_edges(index_t v) const {
	    return skel_end(v) - skel_begin(v);
	}

	/**
	 * \brief Computes the radical point of four vertices
	 * \param[in] v1 , v2 , v3 , v4 four vertex indices
	 * \return the point equidistant to the four vertices
	 *   relative to the additively weighted squared distance
	 */
	vec3 radical_point(
	    index_t v0, index_t v1, index_t v2, index_t v3
	) const;

	/**
	 * \brief Computes the radical point of three vertices
	 * \param[in] v1 , v2 , v3 three vertex indices
	 * \return the point in the supporting plane of the three vertices
	 *   equidistant to them relative to the additively weighted
	 *   squared distance
	 */
	vec3 radical_point(index_t v0, index_t v1, index_t v2) const;

	/**
	 * \brief Computes the radical point of two vertices
	 * \param[in] v1 , v2 two vertex indices
	 * \return the point in the supporting line of the two vertices
	 *   equidistant to them relative to the additively weighted
	 *   squared distance
	 */
	vec3 radical_point(index_t v0, index_t v1) const;

    protected:
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
	 * \brief Inserts a halfedge in the Delaunay skeleton, utility function
	 *  for compute_skel()
	 * \details Ignored if a halfedge with the same extremities was already
	 *  present in the skeleton
	 * \param[in] h a global halfedge index
	 * \param[in] v1 , v2 the two global indices of the extremities of \p h
	 */
	void insert_in_skel(index_t v1, index_t v2, index_t h) {
	    for(index_t k = skel_ptr_[v1]; k<skel_ptr_[v1+1]; ++k) {
		if(skel_h_[k] == NO_INDEX) {
		    skel_h_[k] = h;
		    return;
		} else if(v2 == halfedge_v(skel_h_[k],1)) {
		    return;
		}
	    }
	    geo_assert_not_reached;
	}

	/**
	 * \short gets the first element of the list of halfedges incident
	 *  to a vertex
	 * \param v a global vertex index, smaller than the parameter max_v
	 *   passed compute_skeleton()
	 * \see skel_end(), skel_h()
	 */
	index_t skel_begin(index_t v) const {
	    geo_debug_assert(v+1 < skel_ptr_.size());
	    return skel_ptr_[v];
	}

	/**
	 * \short gets one position past the last element of the list of
	 *  halfedges incident to a vertex
	 * \param v a global vertex index, smaller than the parameter max_v
	 *   passed compute_skeleton()
	 * \see skel_begin(), skel_h()
	 */
	index_t skel_end(index_t v) const {
	    geo_debug_assert(v+1 < skel_ptr_.size());
	    index_t b = skel_ptr_[v];
	    index_t e = skel_ptr_[v+1];
	    // strip the NO_INDEX entries that can be there
	    // when too much space was allocated (due to
	    // infinite tetrahedra). But maybe we can still
	    // have the exact number (topological disk plus
	    // count real tetrahedra only)... Keeping that
	    // for now.
	    while(e != b && skel_h_[e-1] == NO_INDEX) {
		--e;
	    }
	    return e;
	}

	/**
	 * \short gets an element of the list of halfedges incident to a vertex
	 * \param k the index of the element
	 * \see skel_begin(), skel_end()
	 */
	index_t skel_h(index_t k) const {
	    geo_debug_assert(k < skel_h_.size());
	    return skel_h_[k];
	}

    private:

	// tet facet vertex is such that the tetrahedron
	// formed with:
	//  vertex lv
	//  tet_facet_vertex[lv][0]
	//  tet_facet_vertex[lv][1]
	//  tet_facet_vertex[lv][2]
	// has the same orientation as the original tetrahedron for
	// any vertex lv.
	static constexpr char tet_facet_vertex_[4][3] = {
	    {1, 2, 3},
	    {0, 3, 2},
	    {3, 0, 1},
	    {1, 0, 2}
	};

	static constexpr char ZZ = 127;

	// maps a local halfedge to the two local vertex indices
	// of its extremities
	static constexpr char h2v_[16][2] = {
	    { 2, 3},
	    { 3, 1},
	    { 1, 2},
	    {ZZ,ZZ},
	    { 3, 2},
	    { 2, 0},
	    { 0, 3},
	    {ZZ,ZZ},
	    { 0, 1},
	    { 1, 3},
	    { 3, 0},
	    {ZZ,ZZ},
	    { 0, 2},
	    { 2, 1},
	    { 1, 0},
	    {ZZ,ZZ}
	};

	static constexpr char vv2h_[4][4] = {
	    {ZZ,  8, 12,  6},
	    {14, ZZ,  2,  9},
	    { 5, 13, ZZ,  0},
	    {10,  1,  4, ZZ}
	};

	// Encodes for each vertex the list of halfedges emanating from
	// this vertex, stored in compressed row storage.
	vector<index_t> skel_ptr_;
	vector<index_t> skel_h_;
    };

}

#endif
