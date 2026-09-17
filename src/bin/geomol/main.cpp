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
		exit(-1);
		return false;
	    }
	    atom_pos.resize(M.vertices.nb());
	    atom_type.resize(M.vertices.nb());
	    atom_chain.resize(M.vertices.nb());
	    Attribute<char> atom_type_attr(
		M.vertices.attributes(), "atom_type"
	    );
	    Attribute<char> atom_chain_attr (
		M.vertices.attributes(), "chain_id"
	    );
	    for(index_t v: M.vertices) {
		atom_pos[v] = M.vertices.point(v);
		atom_type[v]= atom_type_attr[v];
		atom_chain[v] = atom_chain_attr[v];
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
	    for(vec3 p: atom_pos) {
		for(index_t c=0; c<3; ++c) {
		    B.xyz_min[c] = std::min(B.xyz_min[c], p[c]);
		    B.xyz_max[c] = std::max(B.xyz_max[c], p[c]);
		}
	    }
	    return B;
	}

	index_t nb_atoms() const {
	    return atom_pos.size();
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
		double R = 1.0;
		double r=0.5, g=0.5, b=0.5;
		char t = atom_type[v];
		if(t == 'C') {
		    r = g = b = 0.0;
		} else if(t == 'H') {
		    r = g = b = 0.9;
		    R = 0.5;
		} else if(t == 'O') {
		    r = 1.0; g = 0.0; b = 0.0;
		} else if(t == 'N') {
		    r = 0.0; g = 0.0; b = 1.0;
		} else if(t == 'S') {
		    r = 1.0; g = 1.0; b = 0.0;
		} else {
		    r = 1.0; g = 0.0; b = 1.0;
		}
		if(atom_coloring_ == ATOM_COLORING_ATOM) {
		    glupColor3d(r,g,b);
		} else if(atom_coloring_ == ATOM_COLORING_CHAIN) {
		    index_t i = index_t(atom_chain[v]) % nb_colormap_entries;
		    double gray = 1.0;
		    if(!slicing_mode) {
			gray = 0.299*r + 0.587*g + 0.114*b;
			gray = 0.8 + 0.2*gray;
		    }
		    glupColor3d(
			gray*colormap[i][0],
			gray*colormap[i][1],
			gray*colormap[i][2]
		    );
		}
		vec3 xyz = atom_pos[v];
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

    private:
	vector<vec3> atom_pos;
	vector<char> atom_type;
	vector<char> atom_chain;

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
