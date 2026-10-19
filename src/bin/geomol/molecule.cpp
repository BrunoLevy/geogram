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
#include <geogram/basic/debug_stream.h>
#include <geogram_gfx/basic/GLSL.h>
#include "molecule_shaders.h"

// TODO:
//  - check -R2 or +R2 ? use same convention everywhere (shaders !)
//  - S0: OK
//  - H1: OK (check whether there are patches completely inside surface)
//  - H2: tighter imposters
//  - S3: tighter imposters if possible: (shrunk tets, we can probably restrain)
//
//   Note: in my notation, I use R2 = |pi-c|^2-wi (used to that in power diag)
//         rather than the opposite used in the articles
//   wi = Ri^2/s
//   t1 = -1/(1-s)   t2 = 1/s
//
//   H1 cells equation:
//   ------------------
// 	 c  = radical_point(i,j)
//       R2 = |pi - c|^2 - wi
//          = |pj - c|^2 - wj
//   t1*X^2 + t2*(Y^2+Z^2) + R2 = 0
//
//   H2 cells equation:
//   ------------------
// 	 c = radical_point(i,j,k)
//       R2 = |pi - c|^2 - wi
//          = |pj - c|^2 - wj
//          = |pk - c|^2 - wi
//   t2*X^2 + t1*(Y^2+Z^2) + R2 = 0
//
//   S3 cells equation:
//   ------------------
//      c  = radical_point(i,j,k,l)
//      R2 = (|pi - c|^2 - wi)*(1-s)
//         = ...
//         = (|pl - c|^2 - wl)*(1-s)

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
	if(false) {
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


    // Given the quadric equation (zero at center and U along axis):
    //    alpha U^2 + beta (V^2+W^2) + R2 = 0
    // The radius along the U axis is given by (when it exists):
    //    r^2(U) = - (alpha U^2 + R2) / beta
    std::pair<double,double> Hcell_radius(
	double alpha, double beta, double R2,
	double u1, double u2
    ) {
	double r2_1 = -(alpha*u1*u1+R2)/beta;
	double r2_2 = -(alpha*u2*u2+R2)/beta;
	double r2_min = std::min(r2_1,r2_2);
	double r2_max = std::max(r2_1,r2_2);
	if(u1*u2 < 0) {
	    double r2_0 = -R2/beta;
	    r2_min = std::min(r2_min,r2_0);
	    r2_max = std::max(r2_max,r2_0);
	}
	return {r2_min, r2_max};
    }

    // TODO: H1 imposters seem to be too small when window is very anisotropic
    // (width >>> height), to be understood
    void Molecule::update_H1_cells() {
	// H1 cells are polygonal prisms aligned with the edges of the
	// triangulation. They are obtained as follows:
	// for each atom v1
	//    for each edge incident to v1 in the diagram pointing to
	//    another atom v2 and such that v1 > v2:
	//       for each halfedge around edge (v1,v2)
	//           generate a quad facet
	//    generate the two polygonal caps
	//
	//   H1 cells equation
	//   -----------------
        //   reminder: wi = Ri^2/s  t1 = -1/(1-s)   t2 = 1/s
	//
        // 	 c  = radical_point(i,j)
        //       R2 = |pi - c|^2 - wi
        //          = |pj - c|^2 - wj
        //   t1*X^2 + t2*(Y^2+Z^2) + R2 = 0

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

		vec3 c = diagram_->radical_point(v1,v2);
		vec3 p1 = atom_pos_[v1];
		vec3 p2 = atom_pos_[v2];
		vec3 axis = normalize(p2 - p1);
		p1 = mix(p1,c,shrink_factor_);
		p2 = mix(p2,c,shrink_factor_);

		double R2 = distance2(c, diagram_->vertex(v1)) -
		    diagram_->weight(v1);

		const double t1 = -1.0/(1.0-shrink_factor_);
		const double t2 =  1.0/shrink_factor_;
		double u1 = dot(p1-c,axis);
		double u2 = dot(p2-c,axis);
		auto [rmin2, rmax2] = Hcell_radius(t1,t2,R2,u1,u2);

		if(discard_empty_cells_ && rmin2 < 0.0 && rmax2 < 0.0) {
		    continue;
		}

		double imposter_r = ::sqrt(rmax2);

		cells.begin_H_cell(c, axis, R2, p1, imposter_r, p2, imposter_r);


		// generate the two polygonal caps
		index_t h = h0;
		cells.add_shrunk_power_facet(h, FLIPPED);
		h = diagram_->halfedge_flip(h);
		cells.add_shrunk_power_facet(h, FLIPPED);

		// generate the quad facets
		h = h0;
		do {
		    index_t t1 = diagram_->halfedge_t(h);
		    index_t lf = diagram_->halfedge_lf(h);
		    index_t t2 = diagram_->tet_adjacent(t1,lf);
		    if(
			diagram_->tet_is_finite(t1) ||
			diagram_->tet_is_finite(t2)
		    ) {
			cells.add_quad_facet(h);
		    }
		    h = diagram_->next_halfedge_around_edge(h,v1,v2);
		} while(h != h0);


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
	for(index_t tet1: diagram_->tets()) {
	    for(index_t lf1=0; lf1<4; ++lf1) {
		index_t tet2 = diagram_->tet_adjacent(tet1, lf1);
		if(tet2 < tet1) {
		    continue;
		}
		index_t lf2 = diagram_->find_tet_adjacent(tet2,tet1);
		bool tet1_is_real = is_atom(diagram_->tet_vertex(tet1,lf1));
		bool tet2_is_real = is_atom(diagram_->tet_vertex(tet2,lf2));
		if(!tet1_is_real && !tet2_is_real) {
		    continue;
		}

		index_t lv[3] = {
		    PowerDiagram::tet_facet_lv(lf1,0),
		    PowerDiagram::tet_facet_lv(lf1,1),
		    PowerDiagram::tet_facet_lv(lf1,2)
		};

		index_t v[3] = {
		    diagram_->tet_vertex(tet1,lv[0]),
		    diagram_->tet_vertex(tet1,lv[1]),
		    diagram_->tet_vertex(tet1,lv[2])
		};

		if(!is_atom(v[0]) || !is_atom(v[1]) || !is_atom(v[2])) {
		    continue;
		}

		vec3 p[3];
		double atom_rmax=0.0;
		for(index_t tlv=0; tlv<3; ++tlv) {
		    p[tlv] = atom_pos_[v[tlv]];
		    atom_rmax = std::max(atom_rmax, atom_radius(v[tlv]));
		}

		vec3 axis = normalize(cross(p[2]-p[0],p[1]-p[0]));
		vec3 c = diagram_->radical_point(v[0],v[1],v[2]);
		double R2 =
		    distance2(c,diagram_->vertex(v[0])) - diagram_->weight(v[0]);

		p[0] = mix(p[0], c, shrink_factor_);
		p[1] = mix(p[1], c, shrink_factor_);
		p[2] = mix(p[2], c, shrink_factor_);

		double u1 = -atom_rmax;
		double u2 = atom_rmax;
		vec3 g1 = c + u1*axis;
		vec3 g2 = c + u2*axis;
		double imposter_r2 = 0.0;
		for(index_t lv=0; lv<3; ++lv) {
		    double r2 = length2((p[lv]-c));
		    imposter_r2 = std::max(imposter_r2,r2);
		}
		const double t1 = -1.0/(1.0-shrink_factor_);
		const double t2 =  1.0/shrink_factor_;
		auto [rmin2, rmax2] = Hcell_radius(t2,t1,R2,u1,u2);
		if(discard_empty_cells_ && rmin2 < 0.0 && rmax2 < 0.0) {
		    continue;
		}
		if(discard_empty_cells_ && rmin2 > imposter_r2) {
		    continue;
		}
		double imposter_r = ::sqrt(imposter_r2);
		cells.begin_H_cell(c, axis, R2, g1, imposter_r, g2, imposter_r);

		index_t h1 =
		    diagram_->make_halfedge_from_t_lv_lv(tet1, lv[0], lv[1]);
		index_t h2 =
		    diagram_->make_halfedge_from_t_lv_lv(tet1, lv[1], lv[2]);
		index_t h3 =
		    diagram_->make_halfedge_from_t_lv_lv(tet1, lv[2], lv[0]);

		cells.add_quad_facet(h1,FLIPPED);
		cells.add_quad_facet(h2,FLIPPED);
		cells.add_quad_facet(h3,FLIPPED);

		if(tet1_is_real) {
		    cells.add_shrunk_tet_facet(tet1,lf1,FLIPPED);
		}
		if(tet2_is_real) {
		    cells.add_shrunk_tet_facet(tet2,lf2,FLIPPED);
		}
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

	    if(discard_empty_cells_ && !carries_surface) {
		continue;
	    }

	    double R = ::sqrt(R2);

	    vec3 imp_c = 0.25*(p[0]+p[1]+p[2]+p[3]);
	    double imp_R2 = 0.0;
	    for(index_t i=0; i<4; ++i) {
		imp_R2 = std::max(imp_R2, length2(p[i]-imp_c));
	    }
	    double imp_R = ::sqrt(imp_R2);

	    cells.begin_S_cell(c,R,imp_c,imp_R);

	    cells.add_shrunk_tet_facet(t,0);
	    cells.add_shrunk_tet_facet(t,1);
	    cells.add_shrunk_tet_facet(t,2);
	    cells.add_shrunk_tet_facet(t,3);

	    cells.end_cell();
	}
    }

    /************************************************************************/

    void Molecule::draw() {
	create_shaders_if_needed();

	if(!draw_mss_) {
	    draw_atoms();
	    return;
	}

	// All the rest of this function is for drawing
	// the molecular skin surface
	nb_triangles_ = 0;

	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	if(draw_mesh_) {
	    glupEnable(GLUP_DRAW_MESH);
	    glupSetMeshWidth(mesh_width_);
	    glupSetColor4fv(GLUP_MESH_COLOR, mesh_color_.data());
	}

	MixedComplexCells::DrawMode mode =
	    raytrace_ ? MixedComplexCells::DRAW_MODE_MSS
        	      : MixedComplexCells::DRAW_MODE_CELLS;

	float t1 = float(-1.0/(1.0 - shrink_factor_));
	float t2 = float(1.0 / shrink_factor_);

	GLSL::set_program_uniform_by_name(
	    S_program_, "full_imposters", full_imposters_
	);

	GLSL::set_program_uniform_by_name(
	    H_program_, "full_imposters", full_imposters_
	);

	bool clip=glupIsEnabled(GLUP_CLIPPING) || glupIsEnabled(GLUP_DRAW_MESH);

	GLSL::set_program_uniform_by_name(S_program_, "replace_imposter", false);
	GLSL::set_program_uniform_by_name(
	    S_program_, "max_clip_planes", clip ? 1024 : 0
	);
	cells_[CELL_TYPE_S0].draw(mode);

	GLSL::set_program_uniform_by_name(
	    H_program_, "max_clip_planes", clip ? 1024 : 2
	);
	GLSL::set_program_uniform_by_name(H_program_, "cAxisPerp", t1, t2);
	cells_[CELL_TYPE_H1].draw(mode);

	GLSL::set_program_uniform_by_name(H_program_, "max_clip_planes", 1024);
	GLSL::set_program_uniform_by_name(H_program_, "cAxisPerp", t2, t1);
	cells_[CELL_TYPE_H2].draw(mode);

	GLSL::set_program_uniform_by_name(S_program_, "max_clip_planes", 1024);
	GLSL::set_program_uniform_by_name(S_program_, "clip_cells", true);
	cells_[CELL_TYPE_S3].draw(mode);

	glDisable(GL_CULL_FACE);
	glupDisable(GLUP_DRAW_MESH);
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
	if(empty_VAO_ == 0) {
	    glGenVertexArrays(1, &empty_VAO_);
	}
    }

    void Molecule::draw_atoms() const {
	const MixedComplexCells& cells = cells_[CELL_TYPE_S0];
	glupDisable(GLUP_VERTEX_COLORS);
	glupSetColor3dv(GLUP_FRONT_AND_BACK_COLOR, cells.color().data());
	cells.bind_tbos();
	GLSL::set_program_uniform_by_name(S_program_, "facet_ptr_TBO", 0);
	GLSL::set_program_uniform_by_name(S_program_, "facet_plane_TBO", 0);
	GLSL::set_program_uniform_by_name(
	    S_program_, "show_imposters", show_imposters_
	);
	GLSL::set_program_uniform_by_name(S_program_, "replace_imposter", false);
	glupUpdateUniformState();
	glUseProgram(S_program_);
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

    void Molecule::MixedComplexCells::bind_tbos() const {
	bool eqn1 = (cell_eqn_1_.size() != 0);
	bool eqn2 = (cell_eqn_2_.size() != 0);
	bool imposters1 = (cell_imposter_1_.size() != 0);
	bool imposters2 = (cell_imposter_2_.size() != 0);

	bind_tbo(cell_facet_ptr_tbo_, cell_facet_ptr_, FACET_PTR_TEX_UNIT);
	bind_tbo(cell_facet_plane_tbo_, cell_facet_plane_, FACET_PLANE_TEX_UNIT);
	if(eqn1) {
	    bind_tbo(cell_eqn_1_tbo_, cell_eqn_1_, CELL_EQN_1_TEX_UNIT);
	}
	if(eqn2) {
	    bind_tbo(cell_eqn_2_tbo_, cell_eqn_2_, CELL_EQN_2_TEX_UNIT);
	}
	if(imposters1) {
	    bind_tbo(
		cell_imposter_1_tbo_, cell_imposter_1_, CELL_IMPOSTER_1_TEX_UNIT
	    );
	}
	if(imposters2) {
	    bind_tbo(
		cell_imposter_2_tbo_, cell_imposter_2_, CELL_IMPOSTER_2_TEX_UNIT
	    );
	}
	GLSL::set_program_uniform_by_name(
	    program_, "facet_ptr_TBO", FACET_PTR_TEX_UNIT
	);
	GLSL::set_program_uniform_by_name(
	    program_, "facet_plane_TBO", FACET_PLANE_TEX_UNIT
	);
	if(eqn1) {
	    GLSL::set_program_uniform_by_name(
		program_, "cell_eqn_1_TBO", CELL_EQN_1_TEX_UNIT
	    );
	}
	if(eqn2) {
	    GLSL::set_program_uniform_by_name(
		program_, "cell_eqn_2_TBO", CELL_EQN_2_TEX_UNIT
	    );
	}
	if(imposters1) {
	    GLSL::set_program_uniform_by_name(
		program_, "cell_imposter_1_TBO", CELL_IMPOSTER_1_TEX_UNIT
	    );
	}
	if(imposters2) {
	    GLSL::set_program_uniform_by_name(
		program_, "cell_imposter_2_TBO", CELL_IMPOSTER_2_TEX_UNIT
	    );
	}
    }

    void Molecule::MixedComplexCells::draw(DrawMode mode) const {
	if(!visible_ || nb_cells() == 0) {
	    return;
	}

	if(mode == DRAW_MODE_MSS) {
	    glupDisable(GLUP_VERTEX_COLORS);
	    glupSetColor3dv(GLUP_FRONT_AND_BACK_COLOR, color().data());
	    bind_tbos();
	    GLSL::set_program_uniform_by_name(
		program_, "show_imposters", molecule_.show_imposters()
	    );
	    glupUpdateUniformState();
	    glUseProgram(program_);
	    glBindVertexArray(molecule_.empty_VAO_);
	    glDrawArrays(GL_TRIANGLES, 0, 6*GLsizei(nb_cells()));
	    glBindVertexArray(0);
	    glUseProgram(0);
	    molecule_.nb_triangles_ += 2*nb_cells();
	    return;
	}

	/* // need to re-wire cells storage and close()
	if(mode == DRAW_MODE_CELLS) {
	    glupDisable(GLUP_VERTEX_COLORS);
	    glupSetColor3dv(GLUP_FRONT_AND_BACK_COLOR, color().data());
	    glupBegin(GLUP_TRIANGLES);
	    for(index_t c: *this) {
		draw_cell(c);
	    }
	    glupEnd();
	    return;
	}
	*/
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

    inline vec4 triangle_plane(vec3 p1, vec3 p2, vec3 p3) {
	vec3 N = normalize(cross(p2-p1, p3-p1));
	return vec4(N, -dot(N,p1));
    }

    void Molecule::MixedComplexCells::add_shrunk_power_facet(
	index_t h0, bool flipped
    ) {
	if(!store_mesh_) {
	    index_t v1 = diagram().halfedge_v(h0,0);
	    index_t v2 = diagram().halfedge_v(h0,1);
	    index_t t = diagram().halfedge_t(h0);
	    vec3 p = molecule_.mixed_vertex({v1,t});
	    vec3 N = normalize(diagram().vertex(v2) - diagram().vertex(v1));
	    add_facet_plane(vec4(N,-dot(N,p)), flipped);
	    return;
	}
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
	index_t lf1 = diagram().halfedge_lf(h);
	index_t v1 = diagram().halfedge_v(h,0);
	index_t v2 = diagram().halfedge_v(h,1);
	index_t t1 = diagram().halfedge_t(h);
	index_t t2 = diagram().tet_adjacent(t1,lf1);
	if(!store_mesh_) {
	    index_t w1 = diagram().tet_facet_vertex(t1,lf1,0);
	    index_t w2 = diagram().tet_facet_vertex(t1,lf1,1);
	    index_t w3 = diagram().tet_facet_vertex(t1,lf1,2);
	    vec3 Nt1 = cross(
		diagram().vertex(w2)-diagram().vertex(w1),
		diagram().vertex(w3)-diagram().vertex(w1)
	    );
	    vec3 N = cross(
		Nt1, diagram().vertex(v2)-diagram().vertex(v1)
	    );
	    N = normalize(N);
	    geo_debug_assert(
		diagram().tet_is_finite(t1) ||
		diagram().tet_is_finite(t2)
	    );
	    vec3 p =
		diagram().tet_is_finite(t1)     ?
		molecule_.mixed_vertex({v1,t1}) :
		molecule_.mixed_vertex({v1,t2}) ;
	    add_facet_plane(vec4(N,-dot(N,p)), flipped);
	    return;
	}
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
	if(!store_mesh_) {
	    vec4 P = triangle_plane(
		molecule_.mixed_vertex({v1,t}),
		molecule_.mixed_vertex({v2,t}),
		molecule_.mixed_vertex({v3,t})
	    );
	    add_facet_plane(P, flipped);
	    return;
	}
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

    void Molecule::MixedComplexCells::save_cell(
	index_t c, const std::string& filename
    ) {
	index_t n=1;
	std::ofstream out(filename.c_str());
	for(index_t f: cell_facets(c)) {
	    for(vec3f p: cell_facet_vertices(f)) {
		out << "v " << p << std::endl;
	    }
	    out << "f ";
	    for(vec3f p: cell_facet_vertices(f)) {
		geo_argused(p);
		out << n << " ";
		++n;
	    }
	    out << std::endl;
	}
    }


}
