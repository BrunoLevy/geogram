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

#include <geogram_gfx/gui/simple_application.h>
#include <geogram_gfx/full_screen_effects/ambient_occlusion.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/delaunay/periodic_delaunay_3d.h>
#include <geogram/basic/stopwatch.h>

namespace {
    using namespace GEO;

    /**
     * \brief A PowerDiagram encoded as a weighted Delaunay triangulation
     */
    class PowerDiagram : public PeriodicDelaunay3d {
    public:

	/**
	 * \brief PowerDiagram constructor
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
	 *         if(h != NO_INDEX) { // there can be empty slots in the list
	 *            do something with h
	 *         }
	 *      }
	 *   \endcode
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
		tet_vertex(t,3) == NO_INDEX );
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
	static index_t tet_facet_lv(index_t lf, index_t lv) {
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
	 * \see skel_begin(), skel_end(), skel_h()
	 */
	void compute_skeleton(index_t max_v = NO_INDEX) {
	    if(max_v == NO_INDEX) {
		max_v = nb_vertices();
	    }
	    skel_ptr_.assign(max_v+1, 0); // +1 because there is a "sentry"
	    // Step 1: compute number of tets incident to each vertex
	    for(index_t t=0; t<nb_tets(); ++t) {
		for(index_t lv=0; lv<4; ++lv) {
		    index_t v = tet_vertex(t,lv);
		    if(v < max_v) {
			skel_ptr_[v+1]++;
		    }
		}
	    }
	    // Step 2: compute number of edges incident to each vertex
	    // using Euler-Poincaré characteristic (Nicolas Ray's idea):
	    // The outer boundary of the set of tets incident to a
	    // vertex v is a topological sphere, hence V-E+F=2, where:
	    //   V = number of edges incident to v
	    //   E = 3F/2
	    //   F = number of tetrahedra incident to v
	    // Hence, V - 3F/2 + F = 2, or V = 2 + F/2, and finally:
	    // Number of incident edges = 2 + (number of incident tets)/2
	    for(index_t v=0; v<max_v; ++v) {
		geo_debug_assert((skel_ptr_[v] & 1) == 0);
		skel_ptr_[v] = 2 + skel_ptr_[v]/2;
	    }
	    // Step 3: update row pointers and assign space for skel_h_
	    for(index_t v=1; v<=max_v; ++v) {
		skel_ptr_[v] += skel_ptr_[v-1];
	    }
	    skel_h_.assign(skel_ptr_[max_v], NO_INDEX);
	    // Step 4: insert all halfedges in skel_h_, keeping a single
	    // halfedge per oriented edge (see insert_in_skel()).
	    for(index_t t: tets()) {
		for(index_t lv1=0; lv1<4; ++lv1) {
		    for(index_t lv2=0; lv2<4; ++lv2) {
			if(lv1 == lv2) {
			    continue;
			}
			index_t v1 = tet_vertex(t,lv1);
			index_t v2 = tet_vertex(t,lv2);
			if(v1 < max_v && v2 != NO_INDEX) {
			    index_t h = make_halfedge_from_t_lv_lv(t, lv1, lv2);
			    insert_in_skel(v1,v2,h);
			}
		    }
		}
	    }
	}

	/**
	 * \short gets the first element of the list of halfedges incident
	 *  to a vertex
	 * \param v a global vertex index, smaller than the parameter max_v
	 *   passed compute_skeleton()
	 * \return a sequence of halfedges
	 * \details the list of incident edges can be traversed as follows:
	 * \code
	 *   for(index_t h: incident_edges(v)) {
	 *      if(h != NO_INDEX) { // there can be empty slots in the list
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
	    return skel_ptr_[v+1];
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

    /*************************************************************************/

    class Molecule {
    public:

	enum AtomColoring {
	    ATOM_COLORING_CONSTANT, ATOM_COLORING_ATOM, ATOM_COLORING_CHAIN
	};

	Molecule() : diagram_(new PowerDiagram()) {
	}

	bool load(const std::string& filename) {
	    Mesh M;
	    if(!mesh_load(filename, M)) {
		return false;
	    }
	    nb_atoms_ = M.vertices.nb();
	    nb_atoms_reserve_ = 2*nb_atoms_;
	    atom_pos_.reserve(nb_atoms_reserve_);
	    atom_pos_.resize(nb_atoms_);
	    atom_type_.resize(nb_atoms_);
	    atom_chain_.resize(nb_atoms_);
	    Attribute<char> atom_type_attr(
		M.vertices.attributes(), "atom_type"
	    );
	    Attribute<char> atom_chain_attr (
		M.vertices.attributes(), "chain_id"
	    );
	    for(index_t v: M.vertices) {
		atom_pos_[v] = M.vertices.point(v);
		atom_type_[v]= atom_type_attr[v];
		atom_chain_[v] = index_t(atom_chain_attr[v]);
	    }
	    update();
	    return true;
	}

	Box3d bbox() const {
	    Box3d B;
	    B.clear();
	    for(vec3 p: atom_pos_) {
		B.add(p);
	    }
	    return B;
	}

	index_t nb_atoms() const {
	    return nb_atoms_;
	}

	index_range atoms() const {
	    return index_range(0, nb_atoms());
	}

	void update() {
	    atom_weight_.reserve(nb_atoms_reserve_);
	    atom_weight_.resize(nb_atoms());
	    r_min_ =  Numeric::max_float64();
	    r_max_ = -Numeric::max_float64();
	    for(index_t v=0; v<nb_atoms(); ++v) {
		double r = atom_radius(atom_type_[v]);
		r_min_ = std::min(r_min_,r);
		r_max_ = std::max(r_max_,r);
		atom_weight_[v] = weight_factor_*(r*r)/shrink_factor_;
	    }
	    {
		Stopwatch W("delaunay");
		diagram_->set_vertices(nb_atoms(), atom_pos_[0].data());
		diagram_->set_weights(atom_weight_.data());
		diagram_->compute();
		Logger::out("delaunay") << diagram_->nb_tets() << " tetrahedra"
					<< std::endl;
	    }
	    {
		Stopwatch W("close");
		while(close_cells()) {
		    Logger::out("delaunay")
			<< diagram_->nb_tets() << " tetrahedra"
			<< std::endl;
		}
	    }
	    {
		Stopwatch W("skel");
		diagram_->compute_skeleton(nb_atoms_);
	    }
	    {
		tet_dual_.resize(diagram_->nb_tets());
		parallel_for(
		    0, diagram_->nb_tets(), [this](index_t t) {
			if(diagram_->tet_is_finite(t)) {
			    tet_dual_[t] = dual(t);
			}
		    }
		);
	    }
	    Logger::out("delaunay") << diagram_->nb_tets() << " tetrahedra"
				    << std::endl;
	}

	/**
	 * \brief Tests whether a vertex is an atom or an additional point
	 * \param[in] v a global vertex index
	 * \retval true if \v corresponds to an atom of the molecule
	 * \retval false if \v is a point that was inserted to close the infinite
	 *  cells
	 * \see close_cells()
	 */
	bool is_atom(index_t v) {
	    return v < nb_atoms_;
	}

	bool close_cells() {
	    bool changed = false;
	    for(index_t t: diagram_->tets()) {
		if(diagram_->tet_is_finite(t)) {
		    continue;
		}
		for(index_t lf=0; lf<4; ++lf) {
		    if(diagram_->tet_vertex(t,lf) == NO_INDEX) {
			index_t v1 = diagram_->tet_facet_vertex(t,lf,0);
			index_t v2 = diagram_->tet_facet_vertex(t,lf,1);
			index_t v3 = diagram_->tet_facet_vertex(t,lf,2);
			if(is_atom(v1) || is_atom(v2) || is_atom(v3)) {
			    vec3 p1 = diagram_->vertex(v1);
			    vec3 p2 = diagram_->vertex(v2);
			    vec3 p3 = diagram_->vertex(v3);
			    double w1 = diagram_->weight(v1);
			    double w2 = diagram_->weight(v2);
			    double w3 = diagram_->weight(v3);
			    vec3 g = (1.0/3.0)*(p1+p2+p3);
			    vec3 N = normalize(cross(p3-p1,p2-p1));
			    double Ag = length2(g-p1);
			    double Acc = 4.0 * r_max_ * r_max_ / shrink_factor_;
			    while(Acc < Ag) {
				Acc += 0.5;
			    }
			    double Bcc = ::sqrt(Acc - Ag);
			    vec3 cc = g + Bcc * N;
			    double w = weight_factor_ * r_min_ / 5.0;
			    vec3 p = cc - (w/w1)*(p1-cc)
				        - (w/w2)*(p2-cc)
				        - (w/w3)*(p3-cc);
			    atom_pos_.push_back(p);
			    atom_weight_.push_back(w);
			    changed = true;
			}
			break;
		    }
		}
	    }
	    if(changed) {
		Logger::out("close") << "Adding "
				     << atom_pos_.size() - nb_atoms()
				     << " points" << std::endl;

		diagram_ = new PowerDiagram();
		diagram_->set_vertices(atom_pos_.size(), atom_pos_[0].data());
		diagram_->set_weights(atom_weight_.data());
		diagram_->compute();
	    }
	    return changed;
	}

	vec3 dual(index_t t) const {
	    geo_debug_assert(diagram_->tet_is_finite(t));
	    index_t v0 = diagram_->tet_vertex(t,0);
	    index_t v1 = diagram_->tet_vertex(t,1);
	    index_t v2 = diagram_->tet_vertex(t,2);
	    index_t v3 = diagram_->tet_vertex(t,3);

	    vec3 p0 = diagram_->vertex(v0);
	    vec3 p1 = diagram_->vertex(v1);
	    vec3 p2 = diagram_->vertex(v2);
	    vec3 p3 = diagram_->vertex(v3);

	    double h0 = length2(p0)-diagram_->weight(v0);
	    double h1 = length2(p1)-diagram_->weight(v1);
	    double h2 = length2(p2)-diagram_->weight(v2);
	    double h3 = length2(p3)-diagram_->weight(v3);

	    mat3 M = {
		{p1.x-p0.x, p1.y-p0.y, p1.z-p0.z},
		{p2.x-p0.x, p2.y-p0.y, p2.z-p0.z},
		{p3.x-p0.x, p3.y-p0.y, p3.z-p0.z}
	    };

	    return M.inverse() * (0.5*vec3{h1-h0, h2-h0, h3-h0});
	}

	vec3 mixed_vertex(index_t v, index_t t) {
	    return mix(atom_pos_[v], tet_dual_[t], shrink_factor_);
	}

	void draw_shrunk_tet_facet(index_t t, index_t lf, bool flipped = false) {
	    if(flipped) {
		glupVertex(mixed_vertex(diagram_->tet_facet_vertex(t, lf, 2),t));
		glupVertex(mixed_vertex(diagram_->tet_facet_vertex(t, lf, 1),t));
		glupVertex(mixed_vertex(diagram_->tet_facet_vertex(t, lf, 0),t));
	    } else {
		glupVertex(mixed_vertex(diagram_->tet_facet_vertex(t, lf, 0),t));
		glupVertex(mixed_vertex(diagram_->tet_facet_vertex(t, lf, 1),t));
		glupVertex(mixed_vertex(diagram_->tet_facet_vertex(t, lf, 2),t));
	    }
	    ++nb_triangles_;
	}

	void draw_shrunk_power_facet(
	    index_t h0, index_t v1, index_t v2, bool flipped = false
	) {
	    index_t h = h0;
	    index_t t1 = NO_INDEX;
	    index_t t2 = NO_INDEX;
	    do {
		index_t t = diagram_->halfedge_t(h);
		if(t1 == NO_INDEX) {
		    t1 = t;
		} else if(t2 == NO_INDEX) {
		    t2 = t;
		} else {
		    if(flipped) {
			glupVertex(mixed_vertex(v1,t));
			glupVertex(mixed_vertex(v1,t2));
			glupVertex(mixed_vertex(v1,t1));
		    } else {
			glupVertex(mixed_vertex(v1,t1));
			glupVertex(mixed_vertex(v1,t2));
			glupVertex(mixed_vertex(v1,t));
		    }
		    ++nb_triangles_;
		    t2 = t;
		}
		h = diagram_->next_halfedge_around_edge(h, v1, v2);
	    } while(h != h0);
	}

	void draw_quad_facet(index_t h, bool flipped = false) {
	    index_t v1 = diagram_->halfedge_v(h,0);
	    index_t v2 = diagram_->halfedge_v(h,1);
	    index_t t1 = diagram_->halfedge_t(h);
	    index_t t2 = diagram_->tet_adjacent(t1,diagram_->halfedge_lf(h));
	    vec3 p11 = mixed_vertex(v1,t1);
	    vec3 p12 = mixed_vertex(v1,t2);
	    vec3 p21 = mixed_vertex(v2,t1);
	    vec3 p22 = mixed_vertex(v2,t2);
	    if(flipped) {
		glupVertex(p11);
		glupVertex(p21);
		glupVertex(p22);
		glupVertex(p11);
		glupVertex(p22);
		glupVertex(p12);
	    } else  {
		glupVertex(p22);
		glupVertex(p21);
		glupVertex(p11);
		glupVertex(p12);
		glupVertex(p22);
		glupVertex(p11);
	    }
	    nb_triangles_ += 2;
	}

	void draw() {
	    nb_triangles_ = 0;
	    draw_atoms();
	    glCullFace(GL_BACK);
	    glEnable(GL_CULL_FACE);

	    draw_shrunk_tets();
	    draw_shrunk_power_cells();
	    draw_H1_cells();
	    draw_H2_cells();

	    glDisable(GL_CULL_FACE);
	    // draw_power_vertices();
	    // draw_additional_vertices();
	    // draw_Delaunay();

	    // std::cerr << nb_triangles_ << " triangles" << std::endl;
	}

	void draw_shrunk_tets() {
	    glupDisable(GLUP_VERTEX_COLORS);
	    glupSetColor3d(GLUP_FRONT_AND_BACK_COLOR, 0.3, 0.3, 1.0);
	    glupBegin(GLUP_TRIANGLES);
	    for(index_t t: diagram_->tets()) {
		if(
		    is_atom(diagram_->tet_vertex(t,0)) &&
		    is_atom(diagram_->tet_vertex(t,1)) &&
		    is_atom(diagram_->tet_vertex(t,2)) &&
		    is_atom(diagram_->tet_vertex(t,3))
		) {
		    draw_shrunk_tet_facet(t,0);
		    draw_shrunk_tet_facet(t,1);
		    draw_shrunk_tet_facet(t,2);
		    draw_shrunk_tet_facet(t,3);
		}
	    }
	    glupEnd();
	}

	void draw_shrunk_power_cells() {

	    glupDisable(GLUP_VERTEX_COLORS);
	    glupSetColor3d(GLUP_FRONT_AND_BACK_COLOR, 0.0, 1.0, 0.0);

	    glupBegin(GLUP_TRIANGLES);
	    for(index_t v1: atoms()) {
		for(index_t h: diagram_->incident_edges(v1)) {
		    if(h == NO_INDEX) {
			break;
		    }
		    index_t v2 = diagram_->halfedge_v(h,1);
		    draw_shrunk_power_facet(h,v1,v2);
		}
	    }
	    glupEnd();
	}

	void draw_H1_cells() {
	    glupDisable(GLUP_VERTEX_COLORS);
	    glupSetColor3d(GLUP_FRONT_AND_BACK_COLOR, 1.0, 0.0, 0.0);

	    glupBegin(GLUP_TRIANGLES);
	    for(index_t v1: atoms()) {
		for(index_t h0: diagram_->incident_edges(v1)) {
		    if(h0 == NO_INDEX) {
			break;
		    }
		    index_t v2 = diagram_->halfedge_v(h0,1);
		    if(!is_atom(v2) || v1 > v2) {
			continue;
		    }
		    index_t h = h0;
		    do {
			draw_quad_facet(h);
			h = diagram_->next_halfedge_around_edge(h,v1,v2);
		    } while(h != h0);

		    draw_shrunk_power_facet(h, v1, v2, true);
		    h = diagram_->halfedge_flip(h);
		    draw_shrunk_power_facet(h, v2, v1, true);
		}
	    }
	    glupEnd();
	}

	void draw_H2_cells() {
	    glupDisable(GLUP_VERTEX_COLORS);
	    glupSetColor3d(GLUP_FRONT_AND_BACK_COLOR, 1.0, 1.0, 0.0);

	    glupBegin(GLUP_TRIANGLES);
	    for(index_t t: diagram_->tets()) {
		if(!diagram_->tet_is_finite(t)) {
		    continue;
		}
		for(index_t lf=0; lf<4; ++lf) {

		    index_t lv1 = diagram_->tet_facet_lv(lf,0);
		    index_t lv2 = diagram_->tet_facet_lv(lf,1);
		    index_t lv3 = diagram_->tet_facet_lv(lf,2);
		    index_t v1 = diagram_->tet_vertex(t, lv1);
		    index_t v2 = diagram_->tet_vertex(t, lv2);
		    index_t v3 = diagram_->tet_vertex(t, lv3);
		    if(!is_atom(v1) || !is_atom(v2) || !is_atom(v3)) {
			continue;
		    }

		    index_t h1 =
			diagram_->make_halfedge_from_t_lv_lv(t, lv1, lv2);
		    index_t h2 =
			diagram_->make_halfedge_from_t_lv_lv(t, lv2, lv3);
		    index_t h3 =
			diagram_->make_halfedge_from_t_lv_lv(t, lv3, lv1);

		    draw_quad_facet(h1,true);
		    draw_quad_facet(h2,true);
		    draw_quad_facet(h3,true);

		    draw_shrunk_tet_facet(t,lf,true);
		    index_t t2 = diagram_->tet_adjacent(t,lf);
		    index_t lf2 = diagram_->find_tet_adjacent(t2,t);
		    draw_shrunk_tet_facet(t2,lf2,true);
		}
	    }

	    glupEnd();
	}

	void draw_Delaunay() {
	    glupDisable(GLUP_VERTEX_COLORS);
	    glupSetMeshWidth(2.0);
	    glupSetColor3d(GLUP_MESH_COLOR, 0.5, 0.5, 0.5);
	    glupSetColor3d(GLUP_FRONT_AND_BACK_COLOR, 0.5, 0.5, 0.5);
	    glupDisable(GLUP_LIGHTING);
	    glupBegin(GLUP_LINES);
	    for(index_t t: diagram_->tets()) {
		for(index_t lv1=0; lv1<4; ++lv1) {
		    index_t v1 = diagram_->tet_vertex(t,lv1);
		    if(v1 == NO_INDEX) {
			continue;
		    }
		    for(index_t lv2=lv1+1; lv2<4; ++lv2) {
			index_t v2 = diagram_->tet_vertex(t,lv2);
			if(v2 == NO_INDEX) {
			    continue;
			}
			glupVertex3dv(atom_pos_[v1].data());
			glupVertex3dv(atom_pos_[v2].data());
		    }
		}
	    }
	    glupEnd();
	    glupEnable(GLUP_LIGHTING);
	}

	void draw_power_vertices() {
	    glupDisable(GLUP_VERTEX_COLORS);
	    glupSetColor3d(GLUP_FRONT_AND_BACK_COLOR, 0.5, 0.5, 0.5);
	    glupBegin(GLUP_SPHERES);
	    for(index_t t: diagram_->tets()) {
		if(!diagram_->tet_is_finite(t)) {
		    continue;
		}
		glupVertex(vec4(tet_dual_[t], 1.0));
	    }
	    glupEnd();
	}

	void draw_additional_vertices() {
	    glupDisable(GLUP_VERTEX_COLORS);
	    glupSetColor3d(GLUP_FRONT_AND_BACK_COLOR, 1.0, 1.0, 0.0);
	    glupBegin(GLUP_SPHERES);
	    for(index_t v=nb_atoms_; v<atom_pos_.size(); ++v) {
		glupVertex(vec4(atom_pos_[v], 2.0));
	    }
	    glupEnd();
	}


	void draw_atoms() {
	    bool slicing_mode =
		glupIsEnabled(GLUP_CLIPPING) &&
		glupGetClipMode() == GLUP_CLIP_SLICE_CELLS;

	    if(atom_coloring_ == ATOM_COLORING_CONSTANT) {
		glupSetColor3fv(
		    GLUP_FRONT_AND_BACK_COLOR, constant_color_.data()
		);
	    } else {
		glupEnable(GLUP_VERTEX_COLORS);
	    }
	    glupBegin(GLUP_SPHERES);
	    for(index_t v: atoms()) {
		char t = atom_type_[v];
		double R = atom_radius(t);
		vec3 color = atom_color(t);
		if(atom_coloring_ == ATOM_COLORING_ATOM) {
		    glupColor3dv(color.data());
		} else if(atom_coloring_ == ATOM_COLORING_CHAIN) {
		    index_t i = atom_chain_[v] % nb_colormap_entries;
		    double gray = 1.0;
		    if(!slicing_mode) {
			gray = 0.299*color.x + 0.587*color.y + 0.114*color.z;
			gray = 0.8 + 0.2*gray;
		    }
		    glupColor3d(
			gray*colormap[i][0],
			gray*colormap[i][1],
			gray*colormap[i][2]
		    );
		}
		vec3 xyz = atom_pos_[v];
		glupVertex4d(xyz[0], xyz[1], xyz[2], R*double(atom_size_)/10.0);
	    }
	    glupEnd();
	    glupDisable(GLUP_VERTEX_COLORS);
	}

	AtomColoring& atom_coloring() {
	    return atom_coloring_;
	}

	int& atom_size() {
	    return atom_size_;
	}

	vec3f& constant_color() {
	    return constant_color_;
	}

	static double atom_radius(char c) {
	    double result = 1.7;
	    switch(c) {
	    case 'C':
		result = 1.7;
		break;
	    case 'O':
		result = 1.52;
		break;
	    case 'H':
		result = 1.2;
		break;
	    case 'N':
		result = 1.55;
		break;
	    case 'S':
		result = 1.8;
		break;
	    case 'P':
		result = 1.8;
		break;
	    default:
		result = 1.7;
		break;
	    }
	    return result;
	}

	static vec3 atom_color(char c) {
	    vec3 result{0.5, 0.5, 0.5};
	    switch(c) {
	    case 'C':
		result = {0.1, 0.1, 0.1};
		break;
	    case 'H':
		result = {0.9, 0.9, 0.9};
		break;
	    case 'O':
		result = {1.0, 0.0, 0.0};
		break;
	    case 'N':
		result = {0.0, 0.0, 1.0};
		break;
	    case 'S':
		result = {1.0, 1.0, 1.0};
		break;
	    default:
		result = {0.5, 0.5, 0.5};
	    }
	    return result;
	}

    private:
	index_t nb_atoms_reserve_;
	index_t nb_atoms_;
	vector<vec3> atom_pos_;
	vector<char> atom_type_;
	vector<index_t> atom_chain_;

	double r_min_;
	double r_max_;
	double weight_factor_ = 1.0;
	double shrink_factor_ = 0.5;
	vector<double> atom_weight_;
	vector<vec3> tet_dual_;
	SmartPointer<PowerDiagram> diagram_;

	index_t nb_triangles_;
	vec3f constant_color_ = {1.0f, 1.0f, 1.0f};
	AtomColoring atom_coloring_ = ATOM_COLORING_CHAIN;
	int atom_size_ = 10;

	static constexpr double c2 = 0.5;
	static constexpr double c3 = 1.0;
	static constexpr index_t nb_colormap_entries = 6;
	static constexpr double colormap[nb_colormap_entries][3] = {
	    {c2, c2, c3},
	    {c3, c2, c2},
	    {c2, c3, c2},
	    {c3, c2, c3},
	    {c2, c3, c3},
	    {c3, c3, c2}
	};
    };

    /*************************************************************************/

    class GeoMolApplication : public SimpleApplication {
    public:
        GeoMolApplication() : SimpleApplication("GeoMol") {
            set_region_of_interest(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
	    lighting_ = false;
	    effect_ = 1;
	    full_screen_effect_ = new AmbientOcclusionImpl();
	    clip_mode_ = GLUP_CLIP_SLICE_CELLS;
        }

    protected:

        void draw_gui() override {
            SimpleApplication::draw_gui();
        }

        void draw_object_properties() override {
            SimpleApplication::draw_object_properties();
	    ImGui::SliderInt("size", &molecule_.atom_size(), 1, 30);
	    ImGui::Combo(
		"color", (int*)&molecule_.atom_coloring(),
		"constant\0atom\0chain\0\0"
	    );
	    if(molecule_.atom_coloring() == Molecule::ATOM_COLORING_CONSTANT) {
		ImGui::ColorEdit3WithPalette(
		    "constant color", molecule_.constant_color().data()
		);
	    }
        }

        void draw_application_menus() override {
        }

        void draw_scene() override {
	    molecule_.draw();
	    if(clipping_ && clip_mode_ == GLUP_CLIP_SLICE_CELLS) {
		glupClipMode(GLUP_CLIP_STANDARD);
		molecule_.draw();
		glupClipMode(GLUP_CLIP_SLICE_CELLS);
	    }
        }

        void draw_about() override {
            ImGui::Separator();
            if(ImGui::BeginMenu("About...")) {
                ImGui::Text(
                    "     A Simple Molecular Viewer\n"
                );
                ImGui::Separator();
                ImGui::Text(
                    "  Molecular Skin Surface by Matthieu Chavent\n"
                    "\n"
                );
                ImGui::Separator();
                ImGui::Text("\n");
                float sz = float(280.0 * std::min(scaling(), 2.0));
                ImGui::Image(
                    static_cast<ImTextureID>(geogram_logo_texture_),
                    ImVec2(sz, sz)
                );
                ImGui::Text("\n");
                ImGui::Text(
                    "\n"
                    "   GEOGRAM/GLUP Project homepage:\n"
                    "https://github.com/BrunoLevy/geogram\n"
                    "\n"
                    "      The ALICE project, Inria\n"
                );
                ImGui::EndMenu();
            }
        }

        std::string supported_read_file_extensions() override {
            return "pdb";
        }

        std::string supported_write_file_extensions() override {
            return "";
        }

        bool load(const std::string& filename) override {
	    bool result =  molecule_.load(filename);
	    if(result && molecule_.nb_atoms() != 0) {
		Box3d B = molecule_.bbox();
		set_region_of_interest(
		    B.xyz_min[0], B.xyz_min[1], B.xyz_min[2],
		    B.xyz_max[0], B.xyz_max[1], B.xyz_max[2]
		);
	    }
	    return result;
        }

    private:
	Molecule molecule_;
    };
}

int main(int argc, char** argv) {
    GeoMolApplication app;
    app.start(argc, argv);
    return 0;
}
