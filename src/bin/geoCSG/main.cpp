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

#include <geogram_gfx/gui/simple_mesh_application.h>
#include <geogram/mesh/mesh_CSG.h>

namespace GEO {
    class CSGApplication : public SimpleMeshApplication {
    public:
        CSGApplication(): SimpleMeshApplication("GeoCSG") {
            use_text_editor_ = true;
            add_key_func("F5", [this](void) { run(); }, "Compile CSG tree");
        }

        /**
         * \copydoc SimpleApplication::load()
         */
	bool load(const std::string& filename) override {
            geo_argused(filename);
            return false;
        }

        /**
         * \copydoc SimpleApplication::save()
         */
	bool save(const std::string& filename) override {
            geo_argused(filename);
            return false;
        }
        
    protected:
        void run() {
            mesh_.clear();

            CSGCompiler CSG;
            CSGMesh_var result = CSG.compile_string(text_editor_.text());
            if(!result.is_null()) {
                mesh_.copy(*result);
            }
            
            double xyzmin[3];
            double xyzmax[3];
            get_bbox(mesh_, xyzmin, xyzmax, false);
            set_region_of_interest(
                xyzmin[0], xyzmin[1], xyzmin[2],
                xyzmax[0], xyzmax[1], xyzmax[2]
            );
            mesh_gfx_.set_mesh(&mesh_);
        }
    };
}

int main(int argc, char** argv) {
    // A SimpleMeshApplication is already a mesh viewer (nothing to do !)
    GEO::CSGApplication app;
    app.start(argc, argv);
    return 0;
}
