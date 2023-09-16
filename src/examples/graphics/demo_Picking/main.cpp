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

#include <geogram_gfx/gui/simple_mesh_application.h>
#include <geogram/basic/vecg.h>
#include <geogram/basic/memory.h>
#include <geogram_gfx/third_party/glad/glad.h>

#include <string>

//   Demo for picking mode : get the facet under the cursor

namespace {
   using namespace GEO;
    
   class DemoPicking : public GEO::SimpleMeshApplication {
    public:
	DemoPicking() : GEO::SimpleMeshApplication(
	    "Geogram Demo Picking"
      )	{
	    picked_mesh_element_ = index_t(-1);
      }

    protected:

		void draw_object_properties() override {
			SimpleApplication::draw_object_properties();
			ImGui::ColorEdit3WithPalette(
				"Surface", surface_color_.data()
			);
			ImGui::Text("Picked facet: %i",picked_mesh_element_);
       	}
        
		void cursor_pos_callback( double x, double y, int source ) override {
			cursor_pos_ = GEO::vec2(x,y); // store x and y in a private variable
			SimpleMeshApplication::cursor_pos_callback(x,y,source);
		}

		void mouse_button_callback(int button, int action, int mods, int source) override {
			if((action==EVENT_ACTION_DOWN) && (button == 0)) { // if left click
				index_t x = index_t(cursor_pos_.x), y = index_t(cursor_pos_.y); // double to integer conversion of current cursor position
				if(x >= get_width() || y >= get_height()) { // if cursor out of the window
					return;
				}
				y = get_height()-1-y; // change Y axis orientation. glReadPixels() wants pixel coordinates from bottom-left corner
				mesh_gfx()->set_picking_mode(MESH_FACETS); // instead of rendering colors, mesh_gfx will render facet indices
				draw_scene(); // rendering
				// read the index of the picked element using glReadPixels()
				Memory::byte picked_mesh_element_as_pixel[4];
				glPixelStorei(GL_PACK_ALIGNMENT, 1);
				glPixelStorei(GL_PACK_ROW_LENGTH, 1);
				glReadPixels(
					GLint(x),GLint(y),1,1,GL_RGBA,GL_UNSIGNED_BYTE,picked_mesh_element_as_pixel
				);
				mesh_gfx()->set_picking_mode(MESH_NONE); // go back to color rendering mode
				// decode facet id from pixel color
				picked_mesh_element_ =
						 index_t(picked_mesh_element_as_pixel[0])        |
						(index_t(picked_mesh_element_as_pixel[1]) << 8)  |
						(index_t(picked_mesh_element_as_pixel[2]) << 16) |
						(index_t(picked_mesh_element_as_pixel[3]) << 24);
			}
			SimpleMeshApplication::mouse_button_callback(button,action,mods,source);
		}
       
   private:
       GEO::vec2 cursor_pos_;
	   index_t picked_mesh_element_;
   };
}

int main(int argc, char** argv) {
    DemoPicking app;
    app.start(argc, argv);
    return 0;
}
