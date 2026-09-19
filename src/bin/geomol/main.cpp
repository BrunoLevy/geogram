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

namespace {
    using namespace GEO;

    class Molecule {
    public:

	enum AtomColoring {
	    ATOM_COLORING_CONSTANT, ATOM_COLORING_ATOM, ATOM_COLORING_CHAIN
	};

	Molecule() {
	}

	bool load(const std::string& filename) {
	    Mesh M;
	    if(!mesh_load(filename, M)) {
		return false;
	    }
	    atom_pos_.resize(M.vertices.nb());
	    atom_type_.resize(M.vertices.nb());
	    atom_chain_.resize(M.vertices.nb());
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
	    return true;
	}

	Box3d bbox() const {
	    Box3d B{
		 Numeric::max_float64(),
		 Numeric::max_float64(),
		 Numeric::max_float64(),
		-Numeric::max_float64(),
		-Numeric::max_float64(),
		-Numeric::max_float64()
	    };
	    for(vec3 p: atom_pos_) {
		for(index_t c=0; c<3; ++c) {
		    B.xyz_min[c] = std::min(B.xyz_min[c], p[c]);
		    B.xyz_max[c] = std::max(B.xyz_max[c], p[c]);
		}
	    }
	    return B;
	}

	index_t nb_atoms() const {
	    return atom_pos_.size();
	}

	void draw() {
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
	vector<vec3> atom_pos_;
	vector<char> atom_type_;
	vector<index_t> atom_chain_;

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
