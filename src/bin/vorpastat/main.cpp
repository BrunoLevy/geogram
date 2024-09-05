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
#include <geogram/basic/progress.h>
#include <geogram/basic/stopwatch.h>
#include <geogram/basic/process.h>
#include <geogram/basic/file_system.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_distance.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_topology.h>

namespace {

    using namespace GEO;

    /**
     * \return false if distance is greater than 5%
     *  of bbox radius.
     */
    bool measure_distance(Mesh& M1, Mesh& M2) {
        Logger::div("distance");
        Stopwatch total("Hausdorff");

        double bbox_diag = bbox_diagonal(M2);
        double sampling_step = CmdLine::get_arg_percent(
            "stat:sampling_step", bbox_diag
        );

        double sym_dist = 0.0, sym_dist_percent = 0.0;

        {
            Stopwatch W("M1->M2");
            Logger::out("Hausdorff") << "Computing Hausdorff distance M1->M2..."
                                     << std::endl;
            double dist = mesh_one_sided_Hausdorff_distance(
                M1, M2, sampling_step
            );
            sym_dist = std::max(sym_dist, dist);
            double dist_percent = dist / bbox_diag * 100.0;
            Logger::out("Hausdorff") << "Hausdorff distance M1->M2: "
                                     << dist
                                     << " (" << dist_percent << "% bbox diag)"
                                     << std::endl;
        }

        {
            Stopwatch W("M2->M1");
            Logger::out("Hausdorff") << "Computing Hausdorff distance M2->M1..."
                                     << std::endl;
            double dist = mesh_one_sided_Hausdorff_distance(
                M2, M1, sampling_step
            );
            sym_dist = std::max(sym_dist, dist);
            double dist_percent = dist / bbox_diag * 100.0;
            Logger::out("Hausdorff") << "Hausdorff distance M2->M1: "
                                     << dist
                                     << " (" << dist_percent << "% bbox diag)"
                                     << std::endl;
        }

        sym_dist_percent = sym_dist / bbox_diag * 100.0;
        Logger::out("Hausdorff") << "Hausdorff distance M2<->M1: "
                                 << sym_dist
                                 << " (" << sym_dist_percent << "% bbox diag)"
                                 << std::endl;

        return sym_dist_percent < 5.0;
    }
}

int main(int argc, char** argv) {
    using namespace GEO;

    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);

    int result = 0;

    try {

        Stopwatch total("Total time");

        CmdLine::import_arg_group("standard");
        CmdLine::import_arg_group("algo");
        CmdLine::import_arg_group("stat");

        Logger::div("initialization");

        std::vector<std::string> filenames;
        if(!CmdLine::parse(
               argc, argv, filenames, "mesh1 mesh2")
          ) {
            return 1;
        }

        std::string mesh1_filename = filenames[0];
        std::string mesh2_filename = filenames[1];

        Mesh M1;
        if(!mesh_load(mesh1_filename, M1)) {
            return 1;
        }
        mesh_repair(M1, MESH_REPAIR_TRIANGULATE);

        if(
            mesh2_filename == "SPHERE" ||
            mesh2_filename == "DISK"   ||
            mesh2_filename == "TORUS"
        ) {
            bool OK = true;
            index_t nb_cnx_comp = mesh_nb_connected_components(M1);
            signed_index_t nb_borders = mesh_nb_borders(M1);
            signed_index_t Xi = mesh_Xi(M1);
            Logger::out("Topology") << "nb cnx:" << nb_cnx_comp
                                    << " nb_borders:" << nb_borders
                                    << " Xi:" << Xi << std::endl;
            if(mesh2_filename == "SPHERE") {
                OK = ((nb_cnx_comp == 1) && (nb_borders == 0) && (Xi == 2));
            } else if(mesh2_filename == "DISK") {
                OK = ((nb_cnx_comp == 1) && (nb_borders == 1) && (Xi == 1));
            } else if(mesh2_filename == "TORUS") {
                OK = ((nb_cnx_comp == 1) && (nb_borders == 0) && (Xi == 0));
            }
            if(OK) {
                Logger::out("Topology") << "Mesh is a " << mesh2_filename
                                        << " (good !)"
                                        << std::endl;

            } else {
                Logger::err("Topology") << "Mesh is not a " << mesh2_filename
                                        << std::endl;
                result = 3;
            }
        } else {
            Mesh M2;
            if(!mesh_load(mesh2_filename, M2)) {
                return 1;
            }
            mesh_repair(M2, MESH_REPAIR_TRIANGULATE);

            if(!measure_distance(M1, M2)) {
                Logger::warn("Distance")
                    << "Deviation greater than threshold (5%)" << std::endl;
                result = 2;
            }

            if(!meshes_have_same_topology(M1, M2, true)) {
                Logger::warn("Topology")
                    << "Mesh topology differs" << std::endl;
            }
        }
    }
    catch(const std::exception& e) {
        std::cerr << "Received an exception: " << e.what() << std::endl;
        return 1;
    }

    return result;
}
