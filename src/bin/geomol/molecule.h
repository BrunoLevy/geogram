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

	Molecule() = default;

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
	 * \brief identifier for a vertex of the mixed complex
	 * \details first index is a primal vertex, second index is a tet
	 */
	typedef std::pair<index_t, index_t> mixed_vertex_id;
	typedef std::tuple<mixed_vertex_id,mixed_vertex_id,mixed_vertex_id>
	    mixed_trgl;

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

	void draw_S0_cell(index_t v) const;
	void draw_H1_cell(index_t h0) const;
	void draw_H2_cell(index_t t, index_t lf) const;
	void draw_S3_cell(index_t t) const;

	/***********************************************************************/

	void draw_shrunk_tet_facet(
	    index_t t, index_t lf, bool flipped = false
	) const;

	void draw_shrunk_power_facet(index_t h0, bool flipped = false) const;

	void draw_quad_facet(index_t h, bool flipped = false) const;

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

	void send_sphere_parameters(vec3 c, double R) const {
	    glupTexCoord({c,R});
	}

	void send_H_parameters(vec3 c, vec4 axis, double R2) const {
	    /*
	    std::cerr << "C=" << c << "  AXIS=" << axis << "  R2=" << R2
		      << std::endl;
	    */
	    glupTexCoord({c,R2});
	    glupNormal4dv(axis.data());
	}

	/***********************************************************************/

	static double atom_radius(char c);
	static vec3 atom_color(char c);

	/***********************************************************************/

	struct CellsInfo {
	    index_t nb_cells() const {
		return triangles_ptr.size()-1;
	    }

	    index_range cells() {
		return index_range(0, nb_cells());
	    }

	    auto cell_triangles(index_t c) const {
		geo_debug_assert(c < nb_cells());
		return transform_range(
		    index_range(triangles_ptr[c], triangles_ptr[c+1]),
		    [this](index_t t) -> mixed_trgl {
			return triangles[t];
		    }
		);
	    }

	    auto cell_planes(index_t c) const {
		geo_debug_assert(c < nb_cells());
		return transform_range(
		    index_range(planes_ptr[c], planes_ptr[c+1]),
		    [this](index_t p) -> vec4f {
			return planes[p];
		    }
		);
	    }

	    vector<index_t> triangles_ptr;
	    vector<mixed_trgl> triangles;
	    vector<index_t> planes_ptr;
	    vector<vec4f> planes;
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
	    true, true, true, true
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
