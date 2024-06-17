/*
 *  Copyright (c) 2000-2023 Inria
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

#include <geogram/basic/common.h>

#ifdef GEO_OS_EMSCRIPTEN

int main() {
    std::cout << "Picking not supported (yet) under Emscripten"
              << std::endl;
    return 0;
}

#else

#include <geogram/basic/logger.h>
#include <geogram/basic/vecg.h>
#include <geogram/basic/memory.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_tetrahedralize.h>
#include <geogram/basic/command_line_args.h>
#include <geogram_gfx/gui/simple_mesh_application.h>
#include <geogram_gfx/third_party/glad/glad.h>

#include <string>


// Demo for picking mode : get index of vertex/facet/cell under the cursor

namespace {
    using namespace GEO;
    
    class DemoPicking : public GEO::SimpleMeshApplication {
    public:

        DemoPicking() :
            GEO::SimpleMeshApplication("Geogram Demo Picking"),
            mesh_element_str_("none")
            { 
                vertices_size_ = 3.0f;
                // own variables
                picked_mesh_element_ = index_t(-1);
                mesh_element_ = MESH_FACETS;
                mesh_element_str_ = subelements_type_to_name(mesh_element_);
                button_ = -1;


                // Create a tetrahedralized icosahedron if no geometry is loaded
                mesh_.vertices.set_dimension(3);

                static double points[] = {
                    0,          0.0,       1.175571,
                    1.051462,   0.0,       0.5257311,
                    0.3249197,  1.0,       0.5257311,
                    -0.8506508, 0.618034,  0.5257311,
                    -0.8506508, -0.618034, 0.5257311,
                    0.3249197,  -1.0,      0.5257311,
                    0.8506508,  0.618034,  -0.5257311,
                    0.8506508,  -0.618034, -0.5257311,
                    -0.3249197,  1.0,      -0.5257311,
                    -1.051462,   0.0,      -0.5257311,
                    -0.3249197, -1.0,      -0.5257311,
                    0.0,         0.0,      -1.175571
                };
        
                static index_t facets[] = {
                    0,1,2,
                    0,2,3,
                    0,3,4,
                    0,4,5,
                    0,5,1,
                    1,5,7,
                    1,7,6,
                    1,6,2,
                    2,6,8,
                    2,8,3,
                    3,8,9,
                    3,9,4,
                    4,9,10,
                    4,10,5,
                    5,10,7,
                    6,7,11,
                    6,11,8,
                    7,10,11,
                    8,11,9,
                    9,11,10,
                };

                index_t first_v = mesh_.vertices.create_vertices(12);
                for(index_t v=0; v<12; ++v) {
                    Geom::mesh_vertex_ref(mesh_,first_v+v) =
                        vec3(points[3*v], points[3*v+1], points[3*v+2]) ;
                }

                for(index_t f=0; f<20; ++f) {
                    mesh_.facets.create_triangle(
                        first_v + facets[3*f],
                        first_v + facets[3*f+1],
                        first_v + facets[3*f+2]
                    );
                }

#ifdef GEOGRAM_WITH_TETGEN
                // arg groups "algo" and "tets" are required by
                // mesh_tetrahedralize()
                CmdLine::import_arg_group("algo");
                CmdLine::import_arg_group("tet");
                
                mesh_.facets.connect();
                mesh_tetrahedralize(mesh_);
                mesh_gfx_.set_mesh(&mesh_);
#endif
                
                double xyzmin[3];
                double xyzmax[3];
                get_bbox(mesh_, xyzmin, xyzmax, false);
                set_region_of_interest(
                    xyzmin[0], xyzmin[1], xyzmin[2],
                    xyzmax[0], xyzmax[1], xyzmax[2]
                );
                
            }

    protected:

        void draw_object_properties() override {
            SimpleApplication::draw_object_properties();

            ImGui::Checkbox("##VertOnOff", &show_vertices_);
            ImGui::SameLine();
            ImGui::ColorEdit3WithPalette("Vertices", vertices_color_.data());
            ImGui::SliderFloat("Vertex size", &vertices_size_, 0.1f, 5.0f, "%.1f");

            ImGui::Checkbox("##MeshOnOff", &show_mesh_);
            ImGui::SameLine();
            ImGui::ColorEdit3WithPalette("Mesh", mesh_color_.data());
            ImGui::SliderFloat("Mesh size", &mesh_width_, 0.1f, 2.0f, "%.1f");

            ImGui::Checkbox("##SurfOnOff", &show_surface_);
            ImGui::SameLine();
            ImGui::ColorEdit3WithPalette("Surface", surface_color_.data());

            ImGui::Checkbox("##VolumeOnOff", &show_volume_);
            ImGui::SameLine();
            ImGui::ColorEdit3WithPalette("Volume", volume_color_.data());

            if (ImGui::BeginCombo("Picking mode",mesh_element_str_.c_str()))
            {
                // only suggest values accepted by GEO::MeshGfx::set_picking_mode()
                if (ImGui::Selectable(
                        subelements_type_to_name(MESH_NONE).c_str(),
                        mesh_element_ == MESH_NONE)
                ) {
                    mesh_element_ = MESH_NONE;
                    mesh_element_str_ = subelements_type_to_name(mesh_element_);
                }
                if (ImGui::Selectable(
                        subelements_type_to_name(MESH_VERTICES).c_str(),
                        mesh_element_ == MESH_VERTICES)
                ) {
                    mesh_element_ = MESH_VERTICES;
                    mesh_element_str_ = subelements_type_to_name(mesh_element_);
                    show_vertices_ = true;
                    show_mesh_ = true;
                }
                if (ImGui::Selectable(
                        subelements_type_to_name(MESH_FACETS).c_str(),
                        mesh_element_ == MESH_FACETS)
                ) {
                    mesh_element_ = MESH_FACETS;
                    mesh_element_str_ = subelements_type_to_name(mesh_element_);
                    show_vertices_ = false;
                    show_mesh_ = true;
                    show_surface_ = true;
                    show_volume_ = false;
                }
                if (ImGui::Selectable(
                        subelements_type_to_name(MESH_CELLS).c_str(),
                        mesh_element_ == MESH_CELLS)
                ) {
                    mesh_element_ = MESH_CELLS;
                    mesh_element_str_ = subelements_type_to_name(mesh_element_);
                    show_vertices_ = false;
                    show_mesh_ = true;
                    show_surface_ = false;
                    show_volume_ = true;
                }
                ImGui::EndCombo();
            }
       	}

        void pick(vec2 pos) {
            // double to integer conversion of current cursor position                
            index_t x = index_t(pos.x);
            index_t y = index_t(pos.y); 
            if(x >= get_width() || y >= get_height()) { // if cursor out of the window
                return;
            }

            // change Y axis orientation. glReadPixels() wants pixel coordinates
            // from bottom-left corner
            y = get_height()-1-y;
            
            // instead of rendering colors, mesh_gfx will render indices
            mesh_gfx()->set_picking_mode(mesh_element_);
            
            // rendering
            draw_scene();
            
            // read the index of the picked element using glReadPixels()
            Memory::byte picked_mesh_element_as_pixel[4];
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glPixelStorei(GL_PACK_ROW_LENGTH, 1);
            glReadPixels(
                GLint(x), GLint(y), 1, 1,
                GL_RGBA,GL_UNSIGNED_BYTE,picked_mesh_element_as_pixel
            );
                
            // go back to color rendering mode
            mesh_gfx()->set_picking_mode(MESH_NONE);
            
            // decode index from pixel color
            picked_mesh_element_ =
                index_t(picked_mesh_element_as_pixel[0])         |
                (index_t(picked_mesh_element_as_pixel[1]) << 8)  |
                (index_t(picked_mesh_element_as_pixel[2]) << 16) |
                (index_t(picked_mesh_element_as_pixel[3]) << 24);
            
            if (picked_mesh_element_ != index_t(-1))
                Logger::out("") << mesh_element_str_
                                       << ": " << "index="
                                       << picked_mesh_element_ << std::endl;
        }

        void cursor_pos_callback( double x, double y, int source ) override {
            cursor_pos_ = GEO::vec2(x,y); // store x and y in a private variable
            if((button_ == 0) && (mesh_element_ != MESH_NONE)) {
                pick(cursor_pos_);
            }            
            SimpleMeshApplication::cursor_pos_callback(x,y,source);
        }

        void mouse_button_callback(
            int button, int action, int mods, int source
        ) override {
            if(action == EVENT_ACTION_DOWN) {
                button_ = button;
            } else {
                button_ = -1;
            }
            if((button_ == 0) && (mesh_element_ != MESH_NONE)) {
                pick(cursor_pos_);
                return;
            }
            SimpleMeshApplication::mouse_button_callback(button,action,mods,source);
        }

        static std::string subelements_type_to_name(MeshElementsFlags what) {
            return (what == MESH_NONE) ?
                std::string("none") :
                Mesh::subelements_type_to_name(what);
        }

        
        
    private:
        int button_;
        GEO::vec2 cursor_pos_;
        MeshElementsFlags mesh_element_;
        std::string mesh_element_str_;
        index_t picked_mesh_element_;
    };
}

int main(int argc, char** argv) {
    DemoPicking app;
    app.start(argc, argv);
    return 0;
}

#endif
