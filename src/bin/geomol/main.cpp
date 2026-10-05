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
#include <geogram/basic/command_line.h>
#include "molecule.h"

namespace {
    using namespace GEO;

    /*************************************************************************/

    class GeoMolApplication : public SimpleApplication {
    public:
        GeoMolApplication() : SimpleApplication("GeoMol") {
            set_region_of_interest(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
	    lighting_ = false;
	    effect_ = 1;
	    full_screen_effect_ = new AmbientOcclusionImpl();
	    clip_mode_ = GLUP_CLIP_SLICE_CELLS;

	    static constexpr double ROT_STEP = 5.0;   // degrees per key press

	    add_key_func(
		"left",  [this]() { rotate_object(1, -ROT_STEP); }, "turn left"
	    );
	    add_key_func(
		"right", [this]() { rotate_object(1,  ROT_STEP); }, "turn right"
	    );
	    add_key_func(
		"up",    [this]() { rotate_object(0, -ROT_STEP); }, "turn up"
	    );
	    add_key_func(
		"down",  [this]() { rotate_object(0,  ROT_STEP); }, "turn down"
	    );
	    add_key_func(
		"page_up", [this]() { rotate_object(2,  ROT_STEP); }, "roll"
	    );
	    add_key_func(
		"page_down", [this]() { rotate_object(2, -ROT_STEP); },
		"roll back"
	    );
        }

    protected:

	/**
	 * \brief Turns the object around one of the three screen axes.
	 * \param[in] axis 0 for the horizontal of the screen, 1 for its
	 *  vertical, 2 for the axis pointing at the viewer.
	 * \param[in] degrees the angle, in degrees.
	 */
	void rotate_object(index_t axis, double degrees) {
	    double a = degrees * M_PI / 180.0;
	    double ca = ::cos(a);
	    double sa = ::sin(a);
	    index_t i = (axis + 1) % 3;
	    index_t j = (axis + 2) % 3;
	    mat4 R;
	    R.load_identity();
	    R(i,i) =  ca; R(i,j) =  sa;
	    R(j,i) = -sa; R(j,j) =  ca;
	    //  Row-vector convention (v * M), as in OpenGL: a rotation made
	    // in screen space multiplies on the RIGHT of the current one.
	    object_rotation_.set_value(object_rotation_.get_value() * R);
	}

	void declare_args() override {
	    SimpleApplication::declare_args();
	    CmdLine::set_arg("gfx:GLUP_profile","GLUP140");
	}

        void draw_gui() override {
            SimpleApplication::draw_gui();
        }

        void draw_object_properties() override {
            SimpleApplication::draw_object_properties();
	    float size = float(molecule_.atom_size());
	    if(ImGui::SliderFloat("size", &size, 0.01f, 1.99f)) {
		molecule_.atom_size() = double(size);
		molecule_.update();
	    }
	    ImGui::Combo(
		"color", (int*)&molecule_.atom_coloring(),
		"constant\0atom\0chain\0\0"
	    );
	    if(molecule_.atom_coloring() == Molecule::ATOM_COLORING_CONSTANT) {
		ImGui::ColorEdit3WithPalette(
		    "constant color", molecule_.constant_color().data()
		);
	    }
	    ImGui::Checkbox("skin surface", &molecule_.draw_mss());
	    if(molecule_.draw_mss()) {
		float sf = float(molecule_.shrink_factor());
		if(ImGui::SliderFloat("shrink", &sf, 0.05f, 0.95f, "%.2f")) {
		    molecule_.shrink_factor() = double(sf);
		    molecule_.update();
		}
		ImGui::Checkbox("raytrace", &molecule_.raytrace());
		ImGui::Checkbox("0-patches", &molecule_.visible(0));
		ImGui::Checkbox("1-patches", &molecule_.visible(1));
		ImGui::Checkbox("2-patches", &molecule_.visible(2));
		ImGui::Checkbox("3-patches", &molecule_.visible(3));
		ImGui::Checkbox("verbose", &molecule_.verbose());

		std::string trgls_string = String::format(
		    "triangles: %d", molecule_.nb_drawn_triangles()
		);
		ImGui::Text("%s",trgls_string.c_str());
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
