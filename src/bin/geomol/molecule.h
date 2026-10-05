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

    /**
     * \brief A molecule that can be rendered as union of spheres or
     *  molecular skin surface.
     */
    class Molecule {
    public:

	enum AtomColoring {
	    ATOM_COLORING_CONSTANT, ATOM_COLORING_ATOM, ATOM_COLORING_CHAIN
	};

	enum MixedCellType {
	    CELL_TYPE_S0=0, CELL_TYPE_H1=1, CELL_TYPE_H2=2, CELL_TYPE_S3=3
	};

	Molecule();
	~Molecule();

	/**
	 * \brief Loads a molecule from a file in PDB format
	 * \param[in] filename the name of a PDB file
	 */
	bool load(const std::string& filename);

	/**
	 * \brief Gets the number of atoms in this molecule
	 */
	index_t nb_atoms() const { return nb_atoms_; }

	/**
	 * \brief Gets the bounding box of the molecule
	 * \return the smallest box that encloses all the centers of the atoms.
	 *  does not take the radii of the atoms into account.
	 */
	Box3d bbox() const;

	/**
	 * \brief Draws the molecule
	 */
	void draw();

	/**
	 * \brief Needs to be called each time the molecule, atom size or
	 *  shrink factor is changed
	 */
	void update();

	/** \brief one of ATOM_COLORING_CONSTANT, ..._ATOM, ..._CHAIN */
	AtomColoring& atom_coloring()  { return atom_coloring_; }

	/** \brief global scaling applied to atom radii */
	double&       atom_size()      { return atom_size_; }

	/** \brief in ]0,1[ (bounds excluded) */
	double&       shrink_factor()  { return shrink_factor_; }

	/** \brief the color for ATOM_COLORING_CONSTANT */
	vec3f&        constant_color() { return constant_color_; }

	/** \brief draw the molecular skin surface */
	bool&         draw_mss()       { return draw_mss_; }

	/** \brief molecular skin surface or draw complex if not set */
	bool&         raytrace()       { return raytrace_; }

	/** \brief displays some statistics when updaring */
	bool&         verbose()        { return verbose_; }

	/**
	 * \brief toggle display mixed complex cells by type
	 * \param[in] type one of 0,1,2,3
	 */
	bool& visible(index_t type) {
	    geo_debug_assert(type < 4);
	    return cells_[type].visible();
	}

	index_t nb_drawn_triangles() const { return nb_triangles_; }

	/**************************************************************/

    protected:

	index_range atoms() const {
	    return index_range(0, nb_atoms());
	}


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
	 * \brief Copies the cells from the power diagram to the
	 *   MixedComplexCells arrays (that represents all the cells
	 *   in compressed row storage format).
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
	    return mix(atom_pos_[V.first], tet_dual_[V.second], shrink_factor_);
	}

	/***********************************************************************/

    protected:

	/** \brief symbolic constant for MixedCells::add_XXX() */
	static constexpr bool FLIPPED = true;

	void create_shaders_if_needed();

	/***********************************************************************/

	void draw_atoms() const;

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

	/**
	 * \brief A familly of cells of the mixed complex.
	 * \details Stores all the polygons in compressed row storage.
	 *  It has a simple API for constructing them and for traversing them,
	 *  as well as a function to compute the clipping planes and sending
	 *  them to the GPU, and a function draw() to render them / raytrace the
	 *  skin surface using a GPU program.
	 *  The cells are constructed as follows, by Molecule::update_X_cells()
	 *   (where X is one of S0,H1,H2,S3):
	 *  \code
	 *     cells.begin_X_cell(...);
	 *        cells.begin_facet();
	 *            cells.add_vertex({v,t});
	 *            cells.add_vertex({v,t});
	 *            ...
	 *        cells.end_facet();
	 *        cells.begin_facet();
	 *            cells.add_vertex({v,t});
	 *            cells.add_vertex({v,t});
	 *            ...
	 *        cells.end_facet();
	 *     cells.end_cell();
	 *     cells.begin_X_cell(...);
	 *        ...
	 *     cells.end_cell();
	 *     ...
	 *  \endcode
	 *  where:
	 *   - cells is a MixedComplexCells object
	 *   - begin_X_cell() is begin_S_cell() (sphere) or begin_H_cell()
	 *      (hyperboloid).
	 *  The cells are traversed as follows:
	 *  \code
	 *    for(index_t c: cells) {
	 *      for(index_t f: cells.facets(c)) {
	 *         for(vec3f p: cells.vertices(f)) {
	 *         }
	 *      }
	 *    }
	 *  \endcode
	 */

	class MixedComplexCells {
	public:
	    explicit MixedComplexCells(Molecule& mol): molecule_(mol) {
		clear();
	    }

	    void clear() {
		cell_facet_ptr_.resize(0);
		cell_facet_ptr_.push_back(0);
		cell_facet_vertex_ptr_.resize(0);
		cell_facet_vertex_ptr_.push_back(0);
		cell_facet_vertex_.resize(0);
		cell_facet_plane_.resize(0);
		cell_eqn_.resize(0);
		cell_facet_ptr_tbo_.reset();
		cell_facet_plane_tbo_.reset();
	    }

	    /*************************************************/

	    void begin_S_cell(vec3 center, double radius) {
		index_t cell_id = cell_eqn_.size();
		cell_eqn_.emplace_back(
		    vec4f{vec3f(center), float(radius)},
		    vec4f{0.0f, 0.0f, 0.0f, float(cell_id)}
		);
	    }

	    void begin_H_cell(vec3 center, vec3 axis, double R2) {
		index_t cell_id = cell_eqn_.size();
		cell_eqn_.emplace_back(
		    vec4f{vec3f(center),float(R2)},
		    vec4f{vec3f(axis), float(cell_id)}
		);
	    }

	    void end_cell() {
		// -1 because count intervals instead of count bounds--------.
		//                                                           v
		index_t total_nb_cell_facets = cell_facet_vertex_ptr_.size()-1;
		cell_facet_ptr_.push_back(total_nb_cell_facets);
	    }

	    void begin_facet() {
	    }

	    void end_facet() {
		cell_facet_vertex_ptr_.push_back(cell_facet_vertex_.size());
		auto it = cell_facet_vertex_.rbegin();
		vec3f p3 = *it;
		vec3f p2 = *(it+1);
		vec3f p1 = *(it+2);
		vec3f n = cross(p2-p1,p3-p1);
		vec4f P{n,-dot(n,p1)};
		cell_facet_plane_.push_back(P);
	    }

	    void add_vertex(mixed_vertex_id V) {
		cell_facet_vertex_.push_back(vec3f(molecule_.mixed_vertex(V)));
	    }

	    /*************************************************/

	    typedef index_as_iterator iterator;
	    typedef index_as_iterator const_iterator;

	    index_t nb_cells() const {
		return cell_eqn_.size();
	    }

	    index_as_iterator begin() const {
		return 0;
	    }

	    index_as_iterator end() const {
		return nb_cells();
	    }

	    auto cell_facets(index_t c) const {
		geo_debug_assert(c < nb_cells());
		return index_range(cell_facet_ptr_[c], cell_facet_ptr_[c+1]);
	    }

	    auto cell_facet_vertices(index_t f) const {
		geo_debug_assert(f+1 < cell_facet_vertex_ptr_.size());
		return transform_range(
		    index_range(
			cell_facet_vertex_ptr_[f],
			cell_facet_vertex_ptr_[f+1]
		    ), [this](index_t v) -> vec3f {
			return cell_facet_vertex_[v];
		    }
		);
	    }

	    auto cell_planes(index_t c) const {
		geo_debug_assert(c < nb_cells());
		return transform_range(
		    index_range(cell_facet_ptr_[c], cell_facet_ptr_[c+1]),
		    [this](index_t p) -> vec4f {
			return cell_facet_plane_[p];
		    }
		);
	    }

	    /*************************************************/

	    // Note: a GLint, not a GLuint (because glUniform1ui(location, val)
	    // does not seem to work to set a texture unit, that seems to be
	    // expected to be a *signed* integer (no idea why).
	    static constexpr GLint FACET_PTR_TEXTURE_UNIT = 4;
	    static constexpr GLint FACET_PLANE_TEXTURE_UNIT = 5;

	    void bind_tbos() const {
		glActiveTexture(GL_TEXTURE0 + FACET_PTR_TEXTURE_UNIT);
		if(cell_facet_ptr_tbo_.TBO() == 0) {
		    cell_facet_ptr_tbo_.create_or_update(
			cell_facet_ptr_.size(), cell_facet_ptr_.data()
		    );
		}
		glActiveTexture(GL_TEXTURE0 + FACET_PLANE_TEXTURE_UNIT);
		if(cell_facet_plane_tbo_.TBO() == 0) {
		    cell_facet_plane_tbo_.create_or_update(
			cell_facet_plane_.size(), cell_facet_plane_.data()
		    );
		}
		glActiveTexture(GL_TEXTURE0);
		cell_facet_ptr_tbo_.bind(GL_TEXTURE0+FACET_PTR_TEXTURE_UNIT);
		cell_facet_plane_tbo_.bind(GL_TEXTURE0+FACET_PLANE_TEXTURE_UNIT);
	    }

	    vec3 color() const {
		if(molecule_.atom_coloring() == ATOM_COLORING_CONSTANT) {
		    vec3f c = molecule_.constant_color();
		    return vec3(double(c.x), double(c.y), double(c.z));
		}
		return color_;
	    }

	    void set_color(const vec3& c) {
		color_ = c;
	    }

	    void set_program(GLuint program) {
		program_ = program;
	    }

	    bool& visible() {
		return visible_;
	    }

	    enum DrawMode { DRAW_MODE_CELLS, DRAW_MODE_MSS };

	    void draw(DrawMode mode = DRAW_MODE_CELLS) const {
		if(!visible_) {
		    return;
		}
		glupDisable(GLUP_VERTEX_COLORS);
		glupSetColor3dv(GLUP_FRONT_AND_BACK_COLOR, color().data());
		if(mode == DRAW_MODE_MSS) {
		    // cell equation is sent through vertex tex coord
		    // and vertex normal, so we need to activate these
		    // vertex attributes.
		    glupEnable(GLUP_TEXTURING);
		    glupEnable(GLUP_VERTEX_NORMALS);
		    glupUseProgram(program_);
		}
		bind_tbos(); // texture buffer object with clipping planes
		glupBegin(GLUP_TRIANGLES);
		for(index_t c: *this) {
		    draw_cell(c);
		}
		glupEnd();
		glupUseProgram(0);
		glupDisable(GLUP_TEXTURING);
		glupDisable(GLUP_VERTEX_NORMALS);
	    }

	    void draw_cell(index_t c) const {
		// send cell equation through tex coord and vertex normal.
		glupTexCoord4fv(cell_eqn_[c].first.data());
		glupNormal4fv(cell_eqn_[c].second.data());
		for(index_t f: cell_facets(c)) {
		    draw_facet(f);
		}
	    }

	    void draw_facet(index_t f) const {
		// triangulates the facet on the fly
		bool has_v1 = false;
		bool has_v2 = false;
		vec3f v1;
		vec3f v2;
		for(vec3f v: cell_facet_vertices(f)) {
		    if(!has_v1) {
			has_v1 = true;
			v1 = v;
		    } else if(!has_v2) {
			has_v2 = true;
			v2 = v;
		    } else {
			glupVertex3fv(v1.data());
			glupVertex3fv(v2.data());
			glupVertex3fv(v.data());
			++molecule_.nb_triangles_;
			v2 = v;
		    }
		}
	    }

	    /*************************************************/

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
	    // Anyway, in the end we'll use simple imposters instead of rendering
	    // the mixed complex cells.
	    vector<index_t> cell_facet_ptr_;
	    vector<vec4f>   cell_facet_plane_;
	    vector<index_t> cell_facet_vertex_ptr_;
	    vector<vec3f> cell_facet_vertex_;

	    /**
	     * \brief Quadric equation
	     * \details Sent through GLUP tex_coord and normal vertex attributes.
	     * - sphere:      {cx,cy,cz,R},  {unused, unused, unused, cell_id}
	     * - hyperboloid: {fx,fy,fz,R2}, {axisx,  axisy,  axisz,  cell_id}
	     */
	    vector<std::pair<vec4f, vec4f>> cell_eqn_;

	    /**
	     * \brief For each cell, clipping planes in compressed row storage
	     */
	    mutable TextureBufferObject cell_facet_ptr_tbo_;
	    mutable TextureBufferObject cell_facet_plane_tbo_;

	    vec3 color_ = {1.0, 1.0, 1.0, 1.0};
	    GLuint program_ = 0;
	    bool visible_ = true;
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
	vector<vec3> tet_dual_; // the radical point dual to each tet
	SmartPointer<PowerDiagram> diagram_;
	mutable index_t nb_triangles_; // number of drawn triangles

	vec3f constant_color_ = {1.0f, 1.0f, 1.0f};
	AtomColoring atom_coloring_ = ATOM_COLORING_CHAIN;
	double atom_size_ = 1.0;
	mutable bool wireframe_ = 0.0;

	bool draw_mss_ = false;
	bool raytrace_ = true;
	bool verbose_ = false;
	GLuint S_program_ = 0; // GPU program for spheres
	GLuint H_program_ = 0; // GPU program for hyperboloids

	MixedComplexCells cells_[4]; // S0,H1,H2 and S3 cells

	// a nice colormap for chain complexes
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
