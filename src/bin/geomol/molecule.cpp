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

#include "molecule.h"
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/basic/stopwatch.h>
#include <geogram_gfx/basic/GLSL.h>
#include "molecule_shaders.h"

namespace GEO {

    Molecule::Molecule() : cells_{
	    MixedComplexCells(*this), MixedComplexCells(*this),
	    MixedComplexCells(*this), MixedComplexCells(*this)
    } {
	cells_[CELL_TYPE_S0].set_color({0.0, 1.0, 0.0});
	cells_[CELL_TYPE_H1].set_color({1.0, 1.0, 0.0});
	cells_[CELL_TYPE_H2].set_color({1.0, 0.0, 1.0});
	cells_[CELL_TYPE_S3].set_color({1.0, 0.0, 0.0});
    }

    Molecule::~Molecule() {
	if(S_program_ != 0) {
	    glDeleteProgram(S_program_);
	}
	if(H_program_ != 0) {
	    glDeleteProgram(H_program_);
	}
	if(S_imposters_program_ != 0) {
	    glDeleteProgram(S_imposters_program_);
	}
	if(H_imposters_program_ != 0) {
	    glDeleteProgram(H_imposters_program_);
	}
	if(empty_VAO_ != 0) {
	    glDeleteVertexArrays(1,&empty_VAO_);
	}
    }

    bool Molecule::load(const std::string& filename) {
	Mesh M;
	if(!mesh_load(filename, M)) {
	    return false;
	}
	nb_atoms_ = M.vertices.nb();
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

    Box3d Molecule::bbox() const {
	Box3d B;
	B.clear();
	for(index_t v: atoms()) {
	    B.add(atom_pos_[v]);
	}
	return B;
    }

    void Molecule::update() {
	atom_weight_.resize(nb_atoms());
	atom_pos_.resize(nb_atoms());
	r_min_ =  Numeric::max_float64();
	r_max_ = -Numeric::max_float64();
	for(index_t v=0; v<nb_atoms(); ++v) {
	    double r = atom_radius(v);
	    r_min_ = std::min(r_min_,r);
	    r_max_ = std::max(r_max_,r);
	    atom_weight_[v] = (r*r)/shrink_factor_;
	}
	{
	    Stopwatch W("delaunay",verbose_);
	    diagram_ = new PowerDiagram();
	    diagram_->set_vertices(nb_atoms(), atom_pos_[0].data());
	    diagram_->set_weights(atom_weight_.data());
	    diagram_->compute();
	    if(verbose_) {
		Logger::out("delaunay") << diagram_->nb_tets()
					<< " tetrahedra"
					<< std::endl;
	    }
	}
	{
	    Stopwatch W("close",verbose_);
	    while(close_cells()) {
		if(verbose_) {
		    Logger::out("delaunay")
			<< diagram_->nb_tets() << " tetrahedra"
			<< std::endl;
		}
	    }
	}
	{
	    Stopwatch W("skel",verbose_);
	    diagram_->compute_skeleton(nb_atoms_);
	}
	{
	    tet_dual_.resize(diagram_->nb_tets());
	    parallel_for(
		0, diagram_->nb_tets(), [this](index_t t) {
		    if(diagram_->tet_is_finite(t)) {
			tet_dual_[t] = diagram_->radical_point(
			    diagram_->tet_vertex(t,0),
			    diagram_->tet_vertex(t,1),
			    diagram_->tet_vertex(t,2),
			    diagram_->tet_vertex(t,3)
			);
		    }
		}
	    );
	}
	if(verbose_) {
	    Logger::out("delaunay") << diagram_->nb_tets() << " tetrahedra"
				    << std::endl;
	}
	update_cells();
    }

    bool Molecule::close_cells() {
	vector<vec3> new_points;
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
			double Ag = distance2(g,p1);
			double Acc = geo_sqr(
			    4.0 * r_max_ * r_max_ / shrink_factor_
			);
			while(Acc < Ag) {
			    Acc += 0.5;
			}
			double Bcc = ::sqrt(Acc - Ag);
			vec3 cc = g + Bcc * N;
			double w = r_min_ / 5.0;
			vec3 p = cc - (w/w1)*(p1-cc)
			    - (w/w2)*(p2-cc)
			    - (w/w3)*(p3-cc);
			// Do not add points directly, because this may
			// cause atom_pos_ reallocation, and wreck
			// diagram_ (that keeps a pointer to it), so we
			// store the new points in a temporary vector
			// instead.
			new_points.push_back(p);
		    }
		    break;
		}
	    }
	}
	if(new_points.size() != 0) {
	    for(vec3 p: new_points) {
		double w = r_min_ / 5.0;
		atom_pos_.push_back(p);
		atom_weight_.push_back(w);
	    }
	    if(verbose_) {
		Logger::out("close") << "Adding "
				     << atom_pos_.size() - nb_atoms()
				     << " points" << std::endl;
	    }
	    diagram_ = new PowerDiagram();
	    geo_debug_assert(atom_weight_.size() == atom_pos_.size());
	    diagram_->set_vertices(atom_pos_.size(), atom_pos_[0].data());
	    diagram_->set_weights(atom_weight_.data());
	    diagram_->compute();
	    return true;
	}
	return false;
    }

    void Molecule::update_cells() {
	update_S0_cells();
	update_H1_cells();
	update_H2_cells();
	update_S3_cells();
    }

    void Molecule::update_S0_cells() {
	// S0 cells are shrunk Voronoi cells, obtained as follows:
	// for each atom
	//   for each edge incident to the atom in the diagram
	//      add the polygonal facet obtained by turning around the edge
	MixedComplexCells& cells = cells_[CELL_TYPE_S0];
	cells.clear();
	for(index_t v: atoms()) {
	    auto incident_edges = diagram_->incident_edges(v);
	    // skip empty power cell
	    if(incident_edges.end() == incident_edges.begin()) {
		continue;
	    }
	    vec3 c = atom_pos_[v];
	    double R = atom_radius(v);
	    cells.begin_S_cell(c,R);
	    for(index_t h0: incident_edges) {
		cells.add_shrunk_power_facet(h0);
	    }
	    cells.end_cell();
	}
    }

    void Molecule::update_H1_cells() {
	// H1 cells are polygonal prisms aligned with the edges of the
	// triangulation. They are obtained as follows:
	// for each atom v1
	//    for each edge incident to v1 in the diagram pointing to
	//    another atom v2 and such that v1 > v2:
	//       for each halfedge around edge (v1,v2)
	//           generate a quad facet
	//    generate the two polygonal caps
	MixedComplexCells& cells = cells_[CELL_TYPE_H1];
	cells.clear();
	for(index_t v1: atoms()) {
	    for(index_t h0: diagram_->incident_edges(v1)) {
		if(h0 == NO_INDEX) {
		    break;
		}
		index_t v2 = diagram_->halfedge_v(h0,1);
		if(!is_atom(v2) || v1 > v2) {
		    continue;
		}
		index_t t  = diagram_->halfedge_t(h0);

		vec3 p1 = mixed_vertex({v1,t});
		vec3 p2 = mixed_vertex({v2,t});

		vec3 c = diagram_->radical_point(v1,v2);
		vec3 axis = normalize(p2 - p1);

		double R2 = distance2(c, diagram_->vertex(v1)) -
		    diagram_->weight(v1);

		cells.begin_H_cell(
		    c,axis,R2,
		    atom_pos_[v1], atom_radius(v1),
		    atom_pos_[v2], atom_radius(v2)
		);

		// generate the quad facets
		index_t h = h0;
		do {
		    cells.add_quad_facet(h);
		    h = diagram_->next_halfedge_around_edge(h,v1,v2);
		} while(h != h0);

		// generate the two polygonal caps
		cells.add_shrunk_power_facet(h, FLIPPED);
		h = diagram_->halfedge_flip(h);
		cells.add_shrunk_power_facet(h, FLIPPED);

		cells.end_cell();
	    }
	}
    }

    void Molecule::update_H2_cells() {
	// H2 cells are triangular prisms extruded from tetrahedra facets.
	// Their two triangular and three quadrangular facets are explicitly
	// constructed.
	MixedComplexCells& cells = cells_[CELL_TYPE_H2];
	cells.clear();
	for(index_t t1: diagram_->tets()) {
	    if(!diagram_->tet_is_finite(t1)) {
		continue;
	    }
	    for(index_t lf=0; lf<4; ++lf) {
		index_t t2 = diagram_->tet_adjacent(t1, lf);
		if(t2 < t1) {
		    continue;
		}

		index_t lv[3] = {
		    PowerDiagram::tet_facet_lv(lf,0),
		    PowerDiagram::tet_facet_lv(lf,1),
		    PowerDiagram::tet_facet_lv(lf,2)
		};

		index_t v[3] = {
		    diagram_->tet_vertex(t1,lv[0]),
		    diagram_->tet_vertex(t1,lv[1]),
		    diagram_->tet_vertex(t1,lv[2])
		};

		if(!is_atom(v[0]) || !is_atom(v[1]) || !is_atom(v[2])) {
		    continue;
		}

		vec3 axis = normalize(tet_dual_[t1] - tet_dual_[t2]);
		vec3 p[3][2];

		for(index_t tlv=0; tlv<3; ++tlv) {
		    p[tlv][0] = mixed_vertex({v[tlv],t1});
		    p[tlv][1] = mixed_vertex({v[tlv],t2});
		}

		vec3 g1 = (1.0/3.0)*(p[0][0]+p[1][0]+p[2][0]);
		double r1 = std::max(
		    std::max(distance2(p[0][0],g1),distance2(p[1][0],g1)),
		    distance2(p[2][0],g1)
		);

		vec3 g2 = (1.0/3.0)*(p[0][1]+p[1][1]+p[2][1]);
		double r2 = r1; // it is a prism!

		vec3 c = diagram_->radical_point(v[0],v[1],v[2]);
		double R2 =
		    distance2(c,diagram_->vertex(v[0])) - diagram_->weight(v[0]);

		cells.begin_H_cell(
		    c,axis,R2,
		    g1,::sqrt(r1), g2,::sqrt(r2)
		);

		cells.begin_facet();
		cells.add_vertex_by_point(vec3f(p[0][0]));
		cells.add_vertex_by_point(vec3f(p[1][0]));
		cells.add_vertex_by_point(vec3f(p[1][1]));
		cells.add_vertex_by_point(vec3f(p[0][1]));
		cells.end_facet();

		cells.begin_facet();
		cells.add_vertex_by_point(vec3f(p[1][0]));
		cells.add_vertex_by_point(vec3f(p[2][0]));
		cells.add_vertex_by_point(vec3f(p[2][1]));
		cells.add_vertex_by_point(vec3f(p[1][1]));
		cells.end_facet();

		cells.begin_facet();
		cells.add_vertex_by_point(vec3f(p[2][0]));
		cells.add_vertex_by_point(vec3f(p[0][0]));
		cells.add_vertex_by_point(vec3f(p[0][1]));
		cells.add_vertex_by_point(vec3f(p[2][1]));
		cells.end_facet();

		cells.begin_facet();
		cells.add_vertex_by_point(vec3f(p[2][0]));
		cells.add_vertex_by_point(vec3f(p[1][0]));
		cells.add_vertex_by_point(vec3f(p[0][0]));
		cells.end_facet();

		cells.begin_facet();
		cells.add_vertex_by_point(vec3f(p[0][1]));
		cells.add_vertex_by_point(vec3f(p[1][1]));
		cells.add_vertex_by_point(vec3f(p[2][1]));
		cells.end_facet();

		cells.end_cell();
	    }
	}
    }

    void Molecule::update_S3_cells() {
	// S3 cells are shrunk tetrahedra.
	// Their four triangular facets are explicitly constructed.
	MixedComplexCells& cells = cells_[CELL_TYPE_S3];
	cells.clear();
	for(index_t t: diagram_->tets()) {

	    index_t v[4] = {
		diagram_->tet_vertex(t,0),
		diagram_->tet_vertex(t,1),
		diagram_->tet_vertex(t,2),
		diagram_->tet_vertex(t,3)
	    };

	    if(
		!is_atom(v[0]) || !is_atom(v[1]) ||
		!is_atom(v[2]) || !is_atom(v[3])
	    ) {
		continue;
	    }

	    vec3 c = tet_dual_[t];
	    vec3 p0 = diagram_->vertex(v[0]);
	    double R2 = (
		distance2(c,p0) - diagram_->weight(v[0])
	    ) * (1.0 - shrink_factor_);

	    // If sphere is imaginary (negative radius) then tet is solid
	    bool carries_surface = (R2 > 0.0);

	    if(!carries_surface) {
		continue;
	    }

	    vec3 p[4] = {
		mixed_vertex({v[0],t}), mixed_vertex({v[1],t}),
		mixed_vertex({v[2],t}), mixed_vertex({v[3],t})
	    };

	    // If tet is entirely contained in sphere then tet is empty
	    carries_surface = false;
	    for(index_t i=0; i<4; ++i) {
		carries_surface = carries_surface || (distance2(c,p[i]) > R2);
	    }

	    if(!carries_surface) {
		continue;
	    }

	    double R = ::sqrt(R2);
	    cells.begin_S_cell(c,R);

	    // we could instead do cells.add_shrunk_tet_facet(t,0..3) but
	    // it is slower because it recomputes the shrunk vertices
	    for(index_t lf=0; lf<4; ++lf) {
		index_t lv1 = PowerDiagram::tet_facet_lv(lf,0);
		index_t lv2 = PowerDiagram::tet_facet_lv(lf,1);
		index_t lv3 = PowerDiagram::tet_facet_lv(lf,2);
		cells.begin_facet();
		cells.add_vertex_by_point(vec3f(p[lv1]));
		cells.add_vertex_by_point(vec3f(p[lv2]));
		cells.add_vertex_by_point(vec3f(p[lv3]));
		cells.end_facet();
	    }

	    cells.end_cell();
	}
    }

    /************************************************************************/

    void Molecule::draw() {
	if(!draw_mss_) {
	    draw_atoms();
	    return;
	}

	// All the rest of this function is for drawing
	// the molecular skin surface
	create_shaders_if_needed();
	nb_triangles_ = 0;

	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);

	MixedComplexCells::DrawMode mode =
	    raytrace_ ? MixedComplexCells::DRAW_MODE_MSS
        	      : MixedComplexCells::DRAW_MODE_CELLS;

	float t1 = float(-1.0/(1.0 - shrink_factor_));
	float t2 = float(1.0 / shrink_factor_);

	if(use_imposters_ && mode == MixedComplexCells::DRAW_MODE_MSS) {
	    draw_spheres_imposters(cells_[CELL_TYPE_S0]);
	} else {
	    cells_[CELL_TYPE_S0].draw(mode);
	}

	if(use_imposters_ && mode == MixedComplexCells::DRAW_MODE_MSS) {
	    GLSL::set_program_uniform_by_name(
		H_imposters_program_, "cAxisPerp", t1, t2
	    );
	    draw_hyperboloids_imposters(cells_[CELL_TYPE_H1]);
	} else {
	    GLSL::set_program_uniform_by_name(H_program_, "cAxisPerp", t1, t2);
	    cells_[CELL_TYPE_H1].draw(mode);
	}

	if(use_imposters_ && mode == MixedComplexCells::DRAW_MODE_MSS) {
	    GLSL::set_program_uniform_by_name(
		H_imposters_program_, "cAxisPerp", t2, t1
	    );
	    draw_hyperboloids_imposters(cells_[CELL_TYPE_H2]);
	} else {
	    GLSL::set_program_uniform_by_name(H_program_, "cAxisPerp", t2, t1);
	    cells_[CELL_TYPE_H2].draw(mode);
	}

	if(use_imposters_ && mode == MixedComplexCells::DRAW_MODE_MSS) {
	    draw_spheres_imposters(cells_[CELL_TYPE_S3]);
	} else {
	    cells_[CELL_TYPE_S3].draw(mode);
	}

	glDisable(GL_CULL_FACE);
    }

    /************************************************************************/

    void Molecule::create_shaders_if_needed() {
	static bool first_time = true;
	if(first_time) {
	    register_geomol_shader_utilities();
	    first_time=false;
	}
	if(S_program_ == 0) {
	    S_program_ = glupCompileProgram(spheres_source);
	    cells_[CELL_TYPE_S0].set_program(S_program_);
	    cells_[CELL_TYPE_S3].set_program(S_program_);
	}
	if(H_program_ == 0) {
	    H_program_ = glupCompileProgram(hyperboloids_source);
	    cells_[CELL_TYPE_H1].set_program(H_program_);
	    cells_[CELL_TYPE_H2].set_program(H_program_);
	}
	if(S_imposters_program_ == 0) {
	    S_imposters_program_ = glupCompileProgram(spheres_imposters_source);
	}
	if(H_imposters_program_ == 0) {
	    H_imposters_program_ = glupCompileProgram(
		hyperboloids_imposters_source
	    );
	}
	if(empty_VAO_ == 0) {
	    glGenVertexArrays(1, &empty_VAO_);
	}
    }

    void Molecule::draw_atoms() const {
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
	    double R = atom_radius(v);
	    vec3 color = atom_color(v);
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
	    glupVertex4d(xyz[0], xyz[1], xyz[2], R*atom_size_);
	}
	glupEnd();
	glupDisable(GLUP_VERTEX_COLORS);
    }


    void Molecule::draw_spheres_imposters(const MixedComplexCells& cells) const {
	if(!cells.visible()) {
	    return;
	}
	glupDisable(GLUP_VERTEX_COLORS);
	glupSetColor3dv(GLUP_FRONT_AND_BACK_COLOR, cells.color().data());
	cells.bind_tbos(S_imposters_program_,true,false);
	GLSL::set_program_uniform_by_name(
	    S_imposters_program_, "show_imposters", show_imposters_
	);
	glupUpdateUniformState();
	glUseProgram(S_imposters_program_);
	glBindVertexArray(empty_VAO_);
	glDrawArrays(GL_TRIANGLES, 0, 6*GLsizei(cells.nb_cells()));
	glBindVertexArray(0);
	glUseProgram(0);
	nb_triangles_ += 2*cells.nb_cells();
    }


    void Molecule::draw_hyperboloids_imposters(
	const MixedComplexCells& cells
    ) const {
	if(!cells.visible()) {
	    return;
	}
	glupDisable(GLUP_VERTEX_COLORS);
	glupSetColor3dv(GLUP_FRONT_AND_BACK_COLOR, cells.color().data());
	cells.bind_tbos(H_imposters_program_,true,true,true);
	GLSL::set_program_uniform_by_name(
	    H_imposters_program_, "show_imposters", show_imposters_
	);
	glupUpdateUniformState();
	glUseProgram(H_imposters_program_);
	glBindVertexArray(empty_VAO_);
	glDrawArrays(GL_TRIANGLES, 0, 6*GLsizei(cells.nb_cells()));
	glBindVertexArray(0);
	glUseProgram(0);
	nb_triangles_ += 2*cells.nb_cells();
    }

    /************************************************************************/

    double Molecule::atom_radius_from_type(char c) {
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

    vec3 Molecule::atom_color_from_type(char c) {
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

    /************************************************************************/

    void Molecule::MixedComplexCells::clear() {
	cell_facet_ptr_.resize(0);
	cell_facet_ptr_.push_back(0);
	cell_facet_vertex_ptr_.resize(0);
	cell_facet_vertex_ptr_.push_back(0);
	cell_facet_vertex_.resize(0);
	cell_facet_plane_.resize(0);
	cell_eqn_1_.resize(0);
	cell_eqn_2_.resize(0);
	cell_imposter_1_.resize(0);
	cell_imposter_2_.resize(0);
	cell_facet_ptr_tbo_.reset();
	cell_facet_plane_tbo_.reset();
	cell_eqn_1_tbo_.reset();
	cell_eqn_2_tbo_.reset();
	cell_imposter_1_tbo_.reset();
	cell_imposter_2_tbo_.reset();
    }

    void Molecule::MixedComplexCells::bind_tbos(
	GLuint program, bool eqn1, bool eqn2, bool imposters
    ) const {
	bind_tbo(cell_facet_ptr_tbo_, cell_facet_ptr_, FACET_PTR_TEX_UNIT);
	bind_tbo(cell_facet_plane_tbo_, cell_facet_plane_, FACET_PLANE_TEX_UNIT);
	if(eqn1) {
	    bind_tbo(cell_eqn_1_tbo_, cell_eqn_1_, CELL_EQN_1_TEX_UNIT);
	}
	if(eqn2) {
	    bind_tbo(cell_eqn_2_tbo_, cell_eqn_2_, CELL_EQN_2_TEX_UNIT);
	}
	if(imposters) {
	    bind_tbo(
		cell_imposter_1_tbo_, cell_imposter_1_, CELL_IMPOSTER_1_TEX_UNIT
	    );
	    bind_tbo(
		cell_imposter_2_tbo_, cell_imposter_2_, CELL_IMPOSTER_2_TEX_UNIT
	    );
	}
	GLSL::set_program_uniform_by_name(
	    program, "facet_ptr_TBO", FACET_PTR_TEX_UNIT
	);
	GLSL::set_program_uniform_by_name(
	    program, "facet_plane_TBO", FACET_PLANE_TEX_UNIT
	);
	if(eqn1) {
	    GLSL::set_program_uniform_by_name(
		program, "cell_eqn_1_TBO", CELL_EQN_1_TEX_UNIT
	    );
	}
	if(eqn2) {
	    GLSL::set_program_uniform_by_name(
		program, "cell_eqn_2_TBO", CELL_EQN_2_TEX_UNIT
	    );
	}
	if(imposters) {
	    GLSL::set_program_uniform_by_name(
		program, "cell_imposter_1_TBO", CELL_IMPOSTER_1_TEX_UNIT
	    );
	    GLSL::set_program_uniform_by_name(
		program, "cell_imposter_2_TBO", CELL_IMPOSTER_2_TEX_UNIT
	    );
	}
    }

    void Molecule::MixedComplexCells::draw(DrawMode mode) const {
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
	bind_tbos(program_); // texture buffer object with clipping planes
	glupBegin(GLUP_TRIANGLES);
	for(index_t c: *this) {
	    draw_cell(c);
	}
	glupEnd();
	glupUseProgram(0);
	glupDisable(GLUP_TEXTURING);
	glupDisable(GLUP_VERTEX_NORMALS);
    }

    void Molecule::MixedComplexCells::draw_cell(index_t c) const {
	// send cell equation through tex coord and vertex normal.
	glupTexCoord4fv(cell_eqn_1_[c].data());
	glupNormal4fv(cell_eqn_2_[c].data());
	for(index_t f: cell_facets(c)) {
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
    }

    void Molecule::MixedComplexCells::add_shrunk_power_facet(
	index_t h0, bool flipped
    ) {
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

    void Molecule::MixedComplexCells::add_quad_facet(index_t h, bool flipped) {
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

    void Molecule::MixedComplexCells::add_shrunk_tet_facet(
	index_t t, index_t lf, bool flipped
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

/**************************************************************/

}
