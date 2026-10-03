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

#ifndef MOLECULE_H
#define MOLECULE_H

#include <geogram/basic/geometry.h>
#include <geogram/basic/range.h>
#include <geogram_gfx/basic/common.h>
#include <geogram_gfx/GLUP/GLUP.h>
#include "power_diagram.h"
#include "TBO.h"

namespace GEO {

    class Molecule {
    public:

	enum AtomColoring {
	    ATOM_COLORING_CONSTANT, ATOM_COLORING_ATOM, ATOM_COLORING_CHAIN
	};
	static constexpr bool FLIPPED = true;

	enum MixedCellType {
	    CELL_TYPE_S0=0, CELL_TYPE_H1=1, CELL_TYPE_H2=2, CELL_TYPE_S3=3
	};

	Molecule();
	~Molecule();

	bool load(const std::string& filename);

	Box3d bbox() const;

	index_t nb_atoms() const {
	    return nb_atoms_;
	}

	index_range atoms() const {
	    return index_range(0, nb_atoms());
	}

	void update();

	/**
	 * \brief Tests whether a vertex is an atom or an additional point
	 * \param[in] v a global vertex index
	 * \retval true if \v corresponds to an atom of the molecule
	 * \retval false if \v is a point that was inserted to close the infinite
	 *  cells
	 * \see close_cells()
	 */
	bool is_atom(index_t v) const {
	    return v < nb_atoms_;
	}

	/**
	 * \brief Inserts additional points in the diagram to close all
	 *  the cells incident to real atoms
	 * \details One needs to call the function until it returns false
	 * \retval true if the diagram was changed
	 * \retval false if all the cells are closed already
	 */
	bool close_cells();

	/**
	 * \brief Copies the cells from the power diagram to the CellInfo
	 *   array (that represents all the cells in compressed row storage
	 *   format).
	 */
	void update_cells();
	void update_S0_cells();
	void update_H1_cells();
	void update_H2_cells();
	void update_S3_cells();

	/**
	 * \brief identifier for a vertex of the mixed complex
	 * \details first index is a primal vertex, second index is a tet
	 */
	typedef std::pair<index_t, index_t> mixed_vertex_id;
	vec3 mixed_vertex(mixed_vertex_id V) const {
	    double s = shrink_factor_;
	    return mix(atom_pos_[V.first], tet_dual_[V.second], s);
	}

	/***********************************************************************/

	void draw();

	AtomColoring& atom_coloring() {
	    return atom_coloring_;
	}

	double& atom_size() {
	    return atom_size_;
	}

	double& shrink_factor() {
	    return shrink_factor_;
	}

	vec3f& constant_color() {
	    return constant_color_;
	}

	bool& draw_mss() {
	    return draw_mss_;
	}

	bool& raytrace() {
	    return raytrace_;
	}

	bool& draw_S0() {
	    return draw_cell_[CELL_TYPE_S0];
	}

	bool& draw_H1() {
	    return draw_cell_[CELL_TYPE_H1];
	}

	bool& draw_H2() {
	    return draw_cell_[CELL_TYPE_H2];
	}

	bool& draw_S3() {
	    return draw_cell_[CELL_TYPE_S3];
	}

	bool& verbose() {
	    return verbose_;
	}

	index_t nb_drawn_triangles() const {
	    return nb_triangles_;
	}

    protected:

	/***********************************************************************/

	void draw_atoms() const;
	void draw_S0_cells() const;
	void draw_H1_cells() const;
	void draw_H2_cells() const;
	void draw_S3_cells() const;

	/***********************************************************************/

	void draw_triangle(
	    mixed_vertex_id V1, mixed_vertex_id V2, mixed_vertex_id V3,
	    bool flipped=false
	) const {
	    if(wireframe_) {
		glupVertex(mixed_vertex(V1));
		glupVertex(mixed_vertex(V2));
		glupVertex(mixed_vertex(V2));
		glupVertex(mixed_vertex(V3));
		glupVertex(mixed_vertex(V3));
		glupVertex(mixed_vertex(V1));
	    }

	    if(flipped) {
		glupVertex(mixed_vertex(V3));
		glupVertex(mixed_vertex(V2));
		glupVertex(mixed_vertex(V1));
	    } else {
		glupVertex(mixed_vertex(V1));
		glupVertex(mixed_vertex(V2));
		glupVertex(mixed_vertex(V3));
	    }
	    ++nb_triangles_;
	}

	/***********************************************************************/

	double atom_radius(index_t v) const {
	    return atom_size_*atom_radius_from_type(atom_type_[v]);
	}

	vec3 atom_color(index_t v) const {
	    return atom_color_from_type(atom_type_[v]);
	}

	static double atom_radius_from_type(char c);
	static vec3 atom_color_from_type(char c);

	/***********************************************************************/

	class CellsInfo {
	public:
	    explicit CellsInfo(Molecule& mol): molecule_(mol) {
		clear();
	    }

	    void clear() {
		cell_facet_ptr.resize(0);
		cell_facet_ptr.push_back(0);
		cell_facet_vertex_ptr.resize(0);
		cell_facet_vertex_ptr.push_back(0);
		cell_facet_vertex.resize(0);
		cell_facet_plane.resize(0);
		cell_eqn.resize(0);
		cell_facet_ptr_tbo.reset();
		cell_facet_plane_tbo.reset();
	    }

	    /*************************************************/

	    void begin_S_cell(vec3 center, double radius) {
		index_t cell_id = cell_eqn.size();
		cell_eqn.emplace_back(
		    vec4f{
			float(center.x), float(center.y), float(center.z),
			float(radius)
		    },
		    vec4f{0.0f, 0.0f, 0.0f, float(cell_id)}
		);
	    }

	    void begin_H_cell(vec3 center, vec3 axis, double R2) {
		index_t cell_id = cell_eqn.size();
		cell_eqn.emplace_back(
		    vec4f{
			float(center.x), float(center.y), float(center.z),
			float(R2)
		    },
		    vec4f{
			float(axis.x), float(axis.y), float(axis.z),
			float(cell_id)
		    }
		);
	    }

	    void end_cell() {
		cell_facet_ptr.push_back(cell_facet_vertex_ptr.size()-1);
	    }

	    void begin_facet() {
	    }

	    void end_facet() {
		cell_facet_vertex_ptr.push_back(cell_facet_vertex.size());
		auto it = cell_facet_vertex.rbegin();
		vec3 p3 = molecule_.mixed_vertex(*it);
		vec3 p2 = molecule_.mixed_vertex(*(it+1));
		vec3 p1 = molecule_.mixed_vertex(*(it+2));
		vec3 n = cross(p2-p1,p3-p1);
		vec4 P{n,-dot(n,p1)};
		cell_facet_plane.push_back(
		    {float(P.x),float(P.y),float(P.z),float(P.w)}
		);
	    }

	    void add_vertex(mixed_vertex_id V) {
		cell_facet_vertex.push_back(V);
	    }

	    /*************************************************/

	    typedef index_as_iterator iterator;
	    typedef index_as_iterator const_iterator;

	    index_as_iterator begin() const {
		return 0;
	    }

	    index_as_iterator end() const {
		return cell_eqn.size();
	    }

	    auto cell_facets(index_t c) const {
		geo_debug_assert(c < nb_cells());
		return index_range(cell_facet_ptr[c], cell_facet_ptr[c+1]);
	    }

	    auto cell_facet_vertices(index_t f) const {
		geo_debug_assert(f+1 < cell_facet_vertex_ptr.size());
		return transform_range(
		    index_range(
			cell_facet_vertex_ptr[f],
			cell_facet_vertex_ptr[f+1]
		    ), [this](index_t v) -> mixed_vertex_id {
			return cell_facet_vertex[v];
		    }
		);
	    }

	    auto cell_planes(index_t c) const {
		geo_debug_assert(c < nb_cells());
		return transform_range(
		    index_range(cell_facet_ptr[c], cell_facet_ptr[c+1]),
		    [this](index_t p) -> vec4f {
			return cell_facet_plane[p];
		    }
		);
	    }

	    /*************************************************/

	    void draw() const {
		for(index_t c: *this) {
		    draw_cell(c);
		}
	    }

	    void draw_cell(index_t c) const {
		glupTexCoord4fv(cell_eqn[c].first.data());
		glupNormal4fv(cell_eqn[c].second.data());
		for(index_t f: cell_facets(c)) {
		    draw_facet(f);
		}
	    }

	    void draw_facet(index_t f) const {
		bool has_v1 = false;
		bool has_v2 = false;
		mixed_vertex_id v1;
		mixed_vertex_id v2;
		for(mixed_vertex_id v: cell_facet_vertices(f)) {
		    if(!has_v1) {
			has_v1 = true;
			v1 = v;
		    } else if(!has_v2) {
			has_v2 = true;
			v2 = v;
		    } else {
			molecule_.draw_triangle(v1,v2,v);
			v2 = v;
		    }
		}
	    }

	    /*************************************************/

	    /** \brief symbolic constant, parameter for add_XXX() */
	    static constexpr bool FLIPPED = true;

	    void add_shrunk_power_facet(index_t h0, bool flipped = false) {
		if(flipped) {
		    h0 = diagram().halfedge_flip(h0);
		}
		index_t v1 = diagram().halfedge_v(h0,0);
		index_t v2 = diagram().halfedge_v(h0,1);
		index_t h = h0;
		begin_facet();
		do {
		    index_t t = diagram().halfedge_t(h);
		    add_vertex({flipped ? v2 : v1,t});
		    h = diagram().next_halfedge_around_edge(h, v1, v2);
		} while(h != h0);
		end_facet();
	    }

	    void add_quad_facet(index_t h, bool flipped = false) {
		index_t v1 = diagram().halfedge_v(h,0);
		index_t v2 = diagram().halfedge_v(h,1);
		index_t t1 = diagram().halfedge_t(h);
		index_t t2 = diagram().tet_adjacent(t1,diagram().halfedge_lf(h));
		begin_facet();
		if(flipped) {
		    add_vertex({v2,t1});
		    add_vertex({v2,t2});
		    add_vertex({v1,t2});
		    add_vertex({v1,t1});
		} else {
		    add_vertex({v1,t1});
		    add_vertex({v1,t2});
		    add_vertex({v2,t2});
		    add_vertex({v2,t1});
		}
		end_facet();
	    }

	    void add_shrunk_tet_facet(
		index_t t, index_t lf, bool flipped=false
	    ) {
		index_t v1 = diagram().tet_facet_vertex(t, lf, 0);
		index_t v2 = diagram().tet_facet_vertex(t, lf, 1);
		index_t v3 = diagram().tet_facet_vertex(t, lf, 2);
		begin_facet();
		if(flipped) {
		    add_vertex({v3,t});
		    add_vertex({v2,t});
		    add_vertex({v1,t});
		} else {
		    add_vertex({v1,t});
		    add_vertex({v2,t});
		    add_vertex({v3,t});
		}
		end_facet();
	    }

	    /*************************************************/

	    const PowerDiagram& diagram() const {
		return *(molecule_.diagram_);
	    }

	private:
	    Molecule& molecule_;

	    // Dual-level compressed row storage
	    // Groumf, I should only store n-sided facets like that, and
	    // do something smarter to speed-up display... Let us think a little
	    // bit more about it...
	    // Well, let us keep moving forward for now!
	    vector<index_t> cell_facet_ptr;
	    vector<vec4f>   cell_facet_plane;
	    vector<index_t> cell_facet_vertex_ptr;
	    vector<mixed_vertex_id> cell_facet_vertex;

	    // TODOC: format
	    vector<std::pair<vec4f, vec4f>> cell_eqn;

	    // For sending to the GPU
	    TextureBufferObject cell_facet_ptr_tbo;
	    TextureBufferObject cell_facet_plane_tbo;
	};

	/***********************************************************************/

    private:
	index_t nb_atoms_;
	vector<vec3> atom_pos_;
	vector<char> atom_type_;
	vector<index_t> atom_chain_;

	double r_min_;
	double r_max_;
	double shrink_factor_ = 0.5;
	vector<double> atom_weight_;
	vector<vec3> tet_dual_;
	SmartPointer<PowerDiagram> diagram_;
	mutable vector<index_t> shrunk_tets_cache_; // tet ids
	mutable vector<index_t> H1_cells_cache_; // halfedge ids
	mutable vector<index_t> H2_cells_cache_; // halfedge ids
	mutable index_t nb_triangles_; // number of drawn triangles

	vec3f constant_color_ = {1.0f, 1.0f, 1.0f};
	AtomColoring atom_coloring_ = ATOM_COLORING_CHAIN;
	double atom_size_ = 1.0;
	mutable bool wireframe_ = 0.0;

	bool draw_mss_ = false;
	bool raytrace_ = false;
	bool draw_cell_[4] = {
	    true, true, false, false
	};
	vec3 cell_color_[4] = {
	    {0.0, 1.0, 0.0},
	    {1.0, 1.0, 0.0},
	    {1.0, 0.0, 1.0},
	    {1.0, 0.0, 0.0}
	};
	bool verbose_ = false;
	GLuint spheres_program_ = 0;
	GLuint hyperboloids_program_ = 0;

	CellsInfo cells_[4];

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
}

#endif
