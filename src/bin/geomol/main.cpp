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


    class Molecule {
    public:

	enum AtomColoring {
	    ATOM_COLORING_CONSTANT, ATOM_COLORING_ATOM, ATOM_COLORING_CHAIN
	};

	Molecule() : delaunay_(new PeriodicDelaunay3d(false)) {
	    delaunay_->set_keeps_infinite(true);
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
		delaunay_->set_vertices(nb_atoms(), atom_pos_[0].data());
		delaunay_->set_weights(atom_weight_.data());
		delaunay_->compute();
		Logger::out("delaunay") << delaunay_->nb_cells() << " tetrahedra"
					<< std::endl;
	    }
	    {
		Stopwatch W("close");
		while(close_cells()) {
		    Logger::out("delaunay")
			<< delaunay_->nb_cells() << " tetrahedra"
			<< std::endl;
		}
	    }
	    {
		tet_dual_.resize(delaunay_->nb_cells());
		parallel_for(
		    0, delaunay_->nb_cells(), [this](index_t t) {
			if(delaunay_->cell_is_finite(t)) {
			    tet_dual_[t] = dual(t);
			}
		    }
		);
	    }
	    Logger::out("delaunay") << delaunay_->nb_cells() << " tetrahedra"
				    << std::endl;
	}

	bool is_atom(index_t v) {
	    return v < nb_atoms_;
	}


	bool close_cells() {
	    bool changed = false;
	    for(index_t t=0; t<delaunay_->nb_cells(); ++t) {
		if(delaunay_->cell_is_finite(t)) {
		    continue;
		}
		for(index_t lf=0; lf<4; ++lf) {
		    if(delaunay_->cell_vertex(t,lf) == NO_INDEX) {
			index_t lv1 = index_t(tet_facet_vertex_[lf][0]);
			index_t lv2 = index_t(tet_facet_vertex_[lf][1]);
			index_t lv3 = index_t(tet_facet_vertex_[lf][2]);
			index_t v1 = delaunay_->cell_vertex(t,lv1);
			index_t v2 = delaunay_->cell_vertex(t,lv2);
			index_t v3 = delaunay_->cell_vertex(t,lv3);
			if(is_atom(v1) || is_atom(v2) || is_atom(v3)) {
			    vec3 p1 = delaunay_->vertex(v1);
			    vec3 p2 = delaunay_->vertex(v2);
			    vec3 p3 = delaunay_->vertex(v3);
			    double w1 = delaunay_->weight(v1);
			    double w2 = delaunay_->weight(v2);
			    double w3 = delaunay_->weight(v3);
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

		delaunay_ = new PeriodicDelaunay3d(false);
		delaunay_->set_keeps_infinite(true);
		delaunay_->set_vertices(atom_pos_.size(), atom_pos_[0].data());
		delaunay_->set_weights(atom_weight_.data());
		delaunay_->compute();
	    }
	    return changed;
	}

	vec3 dual(index_t t) const {
	    geo_debug_assert(delaunay_->cell_is_finite(t));
	    index_t v0 = delaunay_->cell_vertex(t,0);
	    index_t v1 = delaunay_->cell_vertex(t,1);
	    index_t v2 = delaunay_->cell_vertex(t,2);
	    index_t v3 = delaunay_->cell_vertex(t,3);

	    vec3 p0 = delaunay_->vertex(v0);
	    vec3 p1 = delaunay_->vertex(v1);
	    vec3 p2 = delaunay_->vertex(v2);
	    vec3 p3 = delaunay_->vertex(v3);

	    double h0 = length2(p0)-delaunay_->weight(v0);
	    double h1 = length2(p1)-delaunay_->weight(v1);
	    double h2 = length2(p2)-delaunay_->weight(v2);
	    double h3 = length2(p3)-delaunay_->weight(v3);

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

	void draw_shrunk_tet_facet(index_t t, index_t lf) {
	    for(index_t lv=0; lv<3; ++lv) {
		index_t v = delaunay_->cell_vertex(
		    t, index_t(tet_facet_vertex_[lf][lv])
		);
		glupVertex(mixed_vertex(v, t));
	    }
	}

	void draw_shrunk_voro_facet(index_t t, index_t le) {
	}


	void draw() {
	    draw_atoms();
	    draw_shrunk_tets();
	    // draw_Voronoi_vertices();
	    // draw_additional_vertices();
	    // draw_Delaunay();
	}

	void draw_shrunk_tets() {
	    glupDisable(GLUP_VERTEX_COLORS);
	    glupSetColor3d(GLUP_MESH_COLOR, 0.0, 0.0, 0.0);
	    glupSetMeshWidth(1.0);
	    glupEnable(GLUP_DRAW_MESH);
	    glupSetColor3d(GLUP_FRONT_AND_BACK_COLOR, 0.3, 0.3, 1.0);
	    glupBegin(GLUP_TRIANGLES);
	    for(index_t t=0; t<delaunay_->nb_cells(); ++t) {
		if(
		    is_atom(delaunay_->cell_vertex(t,0)) &&
		    is_atom(delaunay_->cell_vertex(t,1)) &&
		    is_atom(delaunay_->cell_vertex(t,2)) &&
		    is_atom(delaunay_->cell_vertex(t,3))
		) {
		    draw_shrunk_tet_facet(t,0);
		    draw_shrunk_tet_facet(t,1);
		    draw_shrunk_tet_facet(t,2);
		    draw_shrunk_tet_facet(t,3);
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
	    for(index_t t=0; t<delaunay_->nb_cells(); ++t) {
		for(index_t lv1=0; lv1<4; ++lv1) {
		    index_t v1 = delaunay_->cell_vertex(t,lv1);
		    if(v1 == NO_INDEX) {
			continue;
		    }
		    for(index_t lv2=lv1+1; lv2<4; ++lv2) {
			index_t v2 = delaunay_->cell_vertex(t,lv2);
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

	void draw_Voronoi_vertices() {
	    glupDisable(GLUP_VERTEX_COLORS);
	    glupSetColor3d(GLUP_FRONT_AND_BACK_COLOR, 0.5, 0.5, 0.5);
	    glupBegin(GLUP_SPHERES);
	    for(index_t t=0; t<delaunay_->nb_cells(); ++t) {
		if(!delaunay_->cell_is_finite(t)) {
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
	    for(index_t v=0; v<nb_atoms(); ++v) {
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
	SmartPointer<PeriodicDelaunay3d> delaunay_;

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

	static constexpr char tet_facet_vertex_[4][3] = {
	    {1, 2, 3},
	    {0, 3, 2},
	    {3, 0, 1},
	    {1, 0, 2}
	};

    };


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
