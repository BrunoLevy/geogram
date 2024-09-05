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
#include <geogram/basic/logger.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/stopwatch.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_manifold_harmonics.h>

// if GEO_DYNAMIC_LIBS is not defined, then there is unreachable code.
#ifdef GEO_COMPILER_MSVC
#pragma warning( disable : 4702)
#endif

int main(int argc, char** argv) {
    using namespace GEO;

    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);

    try {

#ifndef GEO_DYNAMIC_LIBS
        // Manifold Harmonics depend on ARPACK, loaded dynamically by
        // OpenNL, so geogram needs to be compiled with dynamic libs.
        GEO::Logger::err("MH")
            << "Needs geogram compiled with dynamic libs"
            << std::endl;
        GEO::Logger::err("MH")
            << "(see https://github.com/BrunoLevy/geogram/wiki/FAQ)"
            << std::endl;

        return(-1);
#endif

        Stopwatch Wtot("Total time");

        std::vector<std::string> filenames;

        CmdLine::import_arg_group("standard");
        CmdLine::import_arg_group("algo");
        CmdLine::declare_arg(
            "nb_eigens", 30, "number of eigenpairs"
        );

        CmdLine::declare_arg(
            "discretization", "FEM_P1_LUMPED",
            "one of COMBINATORIAL, UNIFORM, FEM_P1, FEM_P1_LUMPED"
        );

        if(
            !CmdLine::parse(
                argc, argv, filenames, "inmesh <outmesh>"
            )
        ) {
            return 1;
        }

        LaplaceBeltramiDiscretization discretization = FEM_P1_LUMPED;

        const std::string& discretization_str =
            CmdLine::get_arg("discretization");

        if(discretization_str == "COMBINATORIAL") {
            discretization = COMBINATORIAL;
        } else if(discretization_str == "UNIFORM") {
            discretization = UNIFORM;
        } else if(discretization_str == "FEM_P1") {
            discretization = FEM_P1;
        } else if(discretization_str == "FEM_P1_LUMPED") {
            discretization = FEM_P1_LUMPED;
        } else {
            Logger::err("MH")
                << discretization_str << ": invalid discretization"
                << std::endl;
            exit(-1);
        }

        if(filenames.size() != 2) {
            Logger::out("Smooth") << "Generating output to out.geogram"
                                  << std::endl;
            filenames.push_back("out.geogram");
        }

        Logger::div("Data I/O");

        Mesh M;

        MeshIOFlags flags;
        flags.reset_element(MESH_CELLS);
        flags.set_attributes(MESH_ALL_ATTRIBUTES);
        if(!mesh_load(filenames[0], M, flags)) {
            return 1;
        }

        mesh_compute_manifold_harmonics(
            M, CmdLine::get_arg_uint("nb_eigens"), discretization
        );

        if(!mesh_save(M, filenames[1], flags)) {
            return 1;
        }

    }
    catch(const std::exception& e) {
        std::cerr << "Received an exception: " << e.what() << std::endl;
        return 1;
    }

    Logger::out("") << "Everything OK, Returning status 0" << std::endl;
    return 0;
}
