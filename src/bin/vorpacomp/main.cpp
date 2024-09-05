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

#include <geogram/basic/common.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_compare.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/basic/logger.h>

int main(int argc, char** argv) {
    using namespace GEO;

    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);

    try {

        CmdLine::import_arg_group("standard");
        CmdLine::declare_arg(
            "tolerance", 0.0,
            "Tolerance for comparing floating points"
        );

        std::vector<std::string> filenames;
        if(!CmdLine::parse(argc, argv, filenames, "mesh1 mesh2")) {
            return 1;
        }

        std::string mesh1_filename = filenames[0];
        std::string mesh2_filename = filenames[1];

        Mesh M1;
        if(!mesh_load(mesh1_filename, M1)) {
            return 1;
        }

        Mesh M2;
        if(!mesh_load(mesh2_filename, M2)) {
            return 1;
        }

        MeshCompareFlags status = mesh_compare(
            M1, M2, MESH_COMPARE_SURFACE_PROPS,
            CmdLine::get_arg_double("tolerance")
        );

        if(status != MESH_COMPARE_OK) {
            Logger::warn("Compare") << "Meshes differ" << std::endl;
            return 2;
        }
    }
    catch(const std::exception& e) {
        std::cerr << "Received an exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
