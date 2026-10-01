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
#include "geomol_shaders.h"

namespace GEO {

    Molecule::~Molecule() {
	if(spheres_program_ != 0) {
	    glDeleteProgram(spheres_program_);
	}
	if(hyperboloids_program_ != 0) {
	    glDeleteProgram(hyperboloids_program_);
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
	for(vec3 p: atom_pos_) {
	    B.add(p);
	}
	return B;
    }


    void Molecule::update() {
	atom_weight_.resize(nb_atoms());
	atom_pos_.resize(nb_atoms());
	r_min_ =  Numeric::max_float64();
	r_max_ = -Numeric::max_float64();
	for(index_t v=0; v<nb_atoms(); ++v) {
	    double r = atom_radius(atom_type_[v]);
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
	shrunk_tets_cache_.resize(0);
	H1_cells_cache_.resize(0);
	H2_cells_cache_.resize(0);
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
			// Do not add points directly, becase this may
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

    /************************************************************************/

    void Molecule::draw() {
	if(!draw_mss_) {
	    draw_atoms();
	    return;
	}

	if(spheres_program_ == 0) {
	    spheres_program_ = glupCompileProgram(
		GLUPES_spheres_source
	    );
	}

	if(hyperboloids_program_ == 0) {
	    hyperboloids_program_ = glupCompileProgram(
		GLUPES_hyperboloids_source
	    );
	}

	nb_triangles_ = 0;

	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);

	if(draw_0_) {
	    draw_shrunk_power_cells();
	}

	if(draw_H1_) {
	    draw_H1_cells();
	}

	if(draw_H2_) {
	    draw_H2_cells();
	}

	if(draw_3_) {
	    draw_shrunk_tets();
	}

	glDisable(GL_CULL_FACE);
    }

    /************************************************************************/

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
	    glupVertex4d(xyz[0], xyz[1], xyz[2], R*atom_size_);
	}
	glupEnd();
	glupDisable(GLUP_VERTEX_COLORS);
    }

    void Molecule::draw_shrunk_power_cells() const {
	glupDisable(GLUP_VERTEX_COLORS);
	glupSetColor3dv(GLUP_FRONT_AND_BACK_COLOR, color_0_.data());
	if(raytrace_) {
	    glupEnable(GLUP_TEXTURING);
	    glupUseProgram(spheres_program_);
	}
	glupBegin(GLUP_TRIANGLES);
	for(index_t v: atoms()) {
	    draw_shrunk_power_cell(v);
	}
	glupEnd();
	glupUseProgram(0);
	glupDisable(GLUP_TEXTURING);
    }

    void Molecule::draw_H1_cells() const {
	// Select the edges incident to two real atoms,
	// and keep only one halfedge per pair (v1 < v2)
	if(H1_cells_cache_.size() == 0) {
	    for(index_t v1: atoms()) {
		for(index_t h: diagram_->incident_edges(v1)) {
		    if(h == NO_INDEX) {
			break;
		    }
		    index_t v2 = diagram_->halfedge_v(h,1);
		    if(!is_atom(v2) || v1 > v2) {
			continue;
		    }
		    H1_cells_cache_.push_back(h);
		    // goto first_one_only;
		}
	    }
	}

	//first_one_only:

	glupDisable(GLUP_VERTEX_COLORS);
	glupSetColor3dv(GLUP_FRONT_AND_BACK_COLOR, color_H1_.data());

	// DEBUG display radical points and supporting edges
	if(false && raytrace_) {
	    glupBegin(GLUP_SPHERES);
	    for(index_t h: H1_cells_cache_) {
		index_t v1 = diagram_->halfedge_v(h,0);
		index_t v2 = diagram_->halfedge_v(h,1);
		vec3 p = diagram_->radical_point(v1,v2);
		glupVertex(vec4{p,0.5});
	    }
	    glupEnd();

	    glupBegin(GLUP_LINES);
	    for(index_t h: H1_cells_cache_) {
		index_t v1 = diagram_->halfedge_v(h,0);
		index_t v2 = diagram_->halfedge_v(h,1);
		glupVertex(diagram_->vertex(v1));
		glupVertex(diagram_->vertex(v2));
	    }
	    glupEnd();
	    return;
	}

	if(raytrace_) {
	    glupEnable(GLUP_TEXTURING);
	    glupEnable(GLUP_VERTEX_NORMALS);
	    glupUseProgram(hyperboloids_program_);
	    GLSL::set_program_uniform_by_name(
		hyperboloids_program_, "cAxisPerp",
		float(-1.0/(1.0 - shrink_factor_)),
		float(1.0 / shrink_factor_)
	    );
	}
	glupBegin(GLUP_TRIANGLES);
	for(index_t h: H1_cells_cache_) {
	    draw_H1_cell(h);
	}
	glupEnd();
	glupUseProgram(0);
	glupDisable(GLUP_TEXTURING);
	glupDisable(GLUP_VERTEX_NORMALS);


	if(raytrace_) {
	    wireframe_ = true;
	    glupSetColor3dv(
		GLUP_FRONT_AND_BACK_COLOR, vec3(0.0, 0.0, 0.0).data()
	    );
	    glupBegin(GLUP_LINES);
	    for(index_t h: H1_cells_cache_) {
		draw_H1_cell(h);
	    }
	    glupEnd();
	    wireframe_ = false;
	}
    }

    void Molecule::draw_H2_cells() const {
	// Select the faces incident to three real atoms,
	// and keep only one facet in each pair (t1 < t2)
	if(H2_cells_cache_.size() == 0) {
	    for(index_t t1: diagram_->tets()) {
		if(!diagram_->tet_is_finite(t1)) {
		    continue;
		}
		for(index_t lf=0; lf<4; ++lf) {
		    index_t t2 = diagram_->tet_adjacent(t1, lf);
		    if(t2 < t1) {
			continue;
		    }
		    index_t v1 = diagram_->tet_facet_vertex(t1, lf, 0);
		    index_t v2 = diagram_->tet_facet_vertex(t1, lf, 1);
		    index_t v3 = diagram_->tet_facet_vertex(t1, lf, 2);
		    if(!is_atom(v1) || !is_atom(v2) || !is_atom(v3)) {
			continue;
		    }
		    H2_cells_cache_.push_back(
			diagram_->make_halfedge_from_t_lf_le(t1, lf, 0)
		    );
		}
	    }
	}

	glupDisable(GLUP_VERTEX_COLORS);
	glupSetColor3dv(GLUP_FRONT_AND_BACK_COLOR, color_H2_.data());
	if(raytrace_) {
	    glupEnable(GLUP_TEXTURING);
	    glupEnable(GLUP_VERTEX_NORMALS);
	    glupUseProgram(hyperboloids_program_);
	    GLSL::set_program_uniform_by_name(
		hyperboloids_program_, "cAxisPerp",
		float(1.0 / shrink_factor_),
		float(-1.0/(1.0 - shrink_factor_))
	    );
	}
	glupBegin(GLUP_TRIANGLES);
	for(index_t h: H2_cells_cache_) {
	    draw_H2_cell(diagram_->halfedge_t(h), diagram_->halfedge_lf(h));
	}
	glupEnd();
	glupUseProgram(0);
	glupDisable(GLUP_TEXTURING);
	glupDisable(GLUP_VERTEX_NORMALS);
    }


    void Molecule::draw_shrunk_tets() const {
	// Select the tetrahedra incident to four real atoms
	if(shrunk_tets_cache_.size() == 0) {
	    for(index_t t: diagram_->tets()) {
		if(
		    is_atom(diagram_->tet_vertex(t,0)) &&
		    is_atom(diagram_->tet_vertex(t,1)) &&
		    is_atom(diagram_->tet_vertex(t,2)) &&
		    is_atom(diagram_->tet_vertex(t,3))
		) {
		    shrunk_tets_cache_.push_back(t);
		}
	    }
	}

	glupSetColor3dv(GLUP_FRONT_AND_BACK_COLOR, color_3_.data());
	glupDisable(GLUP_VERTEX_COLORS);
	if(raytrace_) {
	    glupEnable(GLUP_TEXTURING);
	    glupUseProgram(spheres_program_);
	}
	glupBegin(GLUP_TRIANGLES);
	for(index_t t: shrunk_tets_cache_) {
	    draw_shrunk_tet(t);
	}
	glupEnd();
	glupUseProgram(0);
	glupDisable(GLUP_TEXTURING);
    }

    /************************************************************************/

    void Molecule::draw_shrunk_power_cell(index_t v) const {
	vec3 c = atom_pos_[v];
	double R = atom_radius(atom_type_[v]);
	send_sphere_parameters(c,R);

	for(index_t h: diagram_->incident_edges(v)) {
	    if(h == NO_INDEX) { break; }
	    draw_shrunk_power_facet(h);
	}
    }

    void Molecule::draw_H1_cell(index_t h0) const {
	index_t v1 = diagram_->halfedge_v(h0,0);
	index_t v2 = diagram_->halfedge_v(h0,1);
	index_t t  = diagram_->halfedge_t(h0);

	vec3 p1 = mixed_vertex({v1,t});
	vec3 p2 = mixed_vertex({v2,t});


	vec3 c = diagram_->radical_point(v1,v2);
	vec3 n = normalize(p2 - p1);
	n = dot(p2-c,n)*n;
	vec4 axis(n,-dot(n,p1));

	double R2 = distance2(c,p1) - diagram_->weight(v1);
	send_H_parameters(c,axis,R2);

	index_t h = h0;
	do {
	    draw_quad_facet(h);
	    h = diagram_->next_halfedge_around_edge(h,v1,v2);
	} while(h != h0);
	draw_shrunk_power_facet(h,FLIPPED);
	h = diagram_->halfedge_flip(h);
	draw_shrunk_power_facet(h,FLIPPED);
    }

    void Molecule::draw_H2_cell(index_t t, index_t lf) const {
	index_t lv1 = PowerDiagram::tet_facet_lv(lf,0);
	index_t lv2 = PowerDiagram::tet_facet_lv(lf,1);
	index_t lv3 = PowerDiagram::tet_facet_lv(lf,2);
	index_t v1 = diagram_->tet_vertex(t,lv1);
	index_t v2 = diagram_->tet_vertex(t,lv2);
	index_t v3 = diagram_->tet_vertex(t,lv3);
	index_t t2 = diagram_->tet_adjacent(t,lf);
	index_t lf2 = diagram_->find_tet_adjacent(t2,t);

	vec3 p1 = diagram_->vertex(v1);
	vec3 c = diagram_->radical_point(v1,v2,v3);
	double R2 = distance2(c,p1) - diagram_->weight(v1);
	vec3 axis = tet_dual_[t] - tet_dual_[t2];
	send_H_parameters(c,vec4(axis,0.0),R2);

	index_t h1 = diagram_->make_halfedge_from_t_lv_lv(t, lv1, lv2);
	index_t h2 = diagram_->make_halfedge_from_t_lv_lv(t, lv2, lv3);
	index_t h3 = diagram_->make_halfedge_from_t_lv_lv(t, lv3, lv1);

	draw_quad_facet(h1,FLIPPED);
	draw_quad_facet(h2,FLIPPED);
	draw_quad_facet(h3,FLIPPED);

	draw_shrunk_tet_facet(t,lf,FLIPPED);
	draw_shrunk_tet_facet(t2,lf2,FLIPPED);
    }

    void Molecule::draw_shrunk_tet(index_t t) const {
	vec3 c = tet_dual_[t];
	index_t v0 = diagram_->tet_vertex(t,0);
	vec3 p0 = diagram_->vertex(v0);
	double R2 = distance2(c,p0) - diagram_->weight(v0);
	// TODO: detect also sphere surface completely outside of shrunk tet
	if(R2 < 0.0) {
	    return;
	}
	double R = ::sqrt(R2)*(1.0-shrink_factor_);
	send_sphere_parameters(c,-R);
	draw_shrunk_tet_facet(t,0);
	draw_shrunk_tet_facet(t,1);
	draw_shrunk_tet_facet(t,2);
	draw_shrunk_tet_facet(t,3);
    }

    /************************************************************************/

    void Molecule::draw_shrunk_tet_facet(
	index_t t, index_t lf, bool flipped
    ) const {
	index_t v1 = diagram_->tet_facet_vertex(t, lf, 0);
	index_t v2 = diagram_->tet_facet_vertex(t, lf, 1);
	index_t v3 = diagram_->tet_facet_vertex(t, lf, 2);
	draw_triangle({v1,t},{v2,t},{v3,t},flipped);
    }

    void Molecule::draw_shrunk_power_facet(index_t h0, bool flipped) const {
	index_t v1 = diagram_->halfedge_v(h0,0);
	index_t v2 = diagram_->halfedge_v(h0,1);
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
		draw_triangle({v1,t1}, {v1,t2}, {v1,t}, flipped);
		t2 = t;
	    }
	    h = diagram_->next_halfedge_around_edge(h, v1, v2);
	} while(h != h0);
    }

    void Molecule::draw_quad_facet(index_t h, bool flipped) const {
	index_t v1 = diagram_->halfedge_v(h,0);
	index_t v2 = diagram_->halfedge_v(h,1);
	index_t t1 = diagram_->halfedge_t(h);
	index_t t2 = diagram_->tet_adjacent(t1,diagram_->halfedge_lf(h));
	draw_triangle( {v2,t2}, {v2,t1}, {v1,t1}, flipped);
	draw_triangle( {v1,t2}, {v2,t2}, {v1,t1}, flipped);
    }

    /************************************************************************/

    double Molecule::atom_radius(char c) {
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

    vec3 Molecule::atom_color(char c) {
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

}
