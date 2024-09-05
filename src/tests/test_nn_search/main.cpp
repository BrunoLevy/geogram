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

#include "nn_search_ANN.h"
#include <geogram/basic/common.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/process.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/stopwatch.h>
#include <geogram/points/nn_search.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>

#if defined(GEO_OS_APPLE) && defined(__arm64__)
#define GEO_APPLE_M1
#endif

int main(int argc, char** argv) {

    using namespace GEO;

    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
    geo_register_NearestNeighborSearch_creator(
        NearestNeighborSearch_ANN, "ANN"
    );
    geo_register_NearestNeighborSearch_creator(
        NearestNeighborSearch_ANN_BruteForce, "ANN_BruteForce"
    );

    try {

        Stopwatch W("Total time");

        CmdLine::import_arg_group("standard");
        CmdLine::import_arg_group("algo");

        CmdLine::declare_arg(
            "algo:nn_check", "ANN", "reference for nn search"
        );
        CmdLine::declare_arg(
            "algo:nn_nb", 10, "number of nearest neighbors"
        );
        CmdLine::declare_arg(
            "by_index", false, "query points by index"
        );

        std::vector<std::string> filenames;
        if(!CmdLine::parse(argc, argv, filenames, "pointsfile")) {
            return 1;
        }

        Mesh M;
        if(!mesh_load(filenames[0], M)) {
            return 1;
        }

        std::string NN1_algo = CmdLine::get_arg("algo:nn_search");
        std::string NN2_algo = CmdLine::get_arg("algo:nn_check");
        bool by_index = CmdLine::get_arg_bool("by_index");


        Logger::out("NN Search") << "Using " << NN1_algo << std::endl;
        Logger::out("NN Search")
            << "Using " << NN2_algo << " for reference"
            << std::endl;

        NearestNeighborSearch_var NN1 = NearestNeighborSearch::create(
            coord_index_t(M.vertices.dimension()), NN1_algo
        );

        NearestNeighborSearch_var NN2 = NearestNeighborSearch::create(
            coord_index_t(M.vertices.dimension()), NN2_algo
        );

        NN1->set_points(M.vertices.nb(), M.vertices.point_ptr(0));
        NN2->set_points(M.vertices.nb(), M.vertices.point_ptr(0));

        index_t nb_neigh = CmdLine::get_arg_uint("algo:nn_nb");
        if(nb_neigh > M.vertices.nb()) {
            Logger::warn("NN Search")
                << "We only have "
                << M.vertices.nb() << " points and "
                << nb_neigh << " neighbors are queried !"
                << std::endl;
            Logger::warn("NN Search")
                << "Using " << M.vertices.nb() << " neighbors."
                << std::endl;
            nb_neigh = M.vertices.nb();
        }

        vector<index_t> neigh1(nb_neigh);
        vector<double> sq_dist1(nb_neigh);

        vector<index_t> neigh2(nb_neigh);
        vector<double> sq_dist2(nb_neigh);

        bool match = true;
        for(index_t i = 0; i < M.vertices.nb(); ++i) {
            const double* q = M.vertices.point_ptr(i);

            neigh1.assign(nb_neigh,NO_INDEX);
            sq_dist1.assign(nb_neigh,0.0);
            neigh2.assign(nb_neigh,NO_INDEX);
            sq_dist2.assign(nb_neigh,0.0);

            if(by_index) {
                NN1->get_nearest_neighbors(
                    nb_neigh, i, neigh1.data(), sq_dist1.data()
                );
                NN2->get_nearest_neighbors(
                    nb_neigh, i, neigh2.data(), sq_dist2.data()
                );
            } else {
                NN1->get_nearest_neighbors(
                    nb_neigh, q, neigh1.data(), sq_dist1.data()
                );
                NN2->get_nearest_neighbors(
                    nb_neigh, q, neigh2.data(), sq_dist2.data()
                );
            }

            for(index_t j=0; j < nb_neigh; ++j) {
                //std::cerr << i << " " << j << "    "
                // << neigh1[j] << " " << neigh2[j] << std::endl;
                geo_assert(neigh1[j] != NO_INDEX);
                geo_assert(neigh2[j] != NO_INDEX);
            }

            bool has_mismatch = false;

            /*
            // Recompute distance between i and nearest neighbors
            // computed by geogram using ANN's distance function,
            // because there can be tiny differences if the compiler
            // does not optimize both functions the same way (happens
            // on Mac/M1) Commented-out for now (does not allow me to
            // get rid of the tolerance below, still need to
            // investigate to understand what's going on with Mac/M1)
            for(index_t j=0; j < nb_neigh; ++j) {
            index_t nn = neigh1[j];
            sq_dist1[j] = annDist(
            M.vertices.dimension(),
            M.vertices.point_ptr(i), M.vertices.point_ptr(nn)
            );
            }
            */

            for(index_t j=0; j < nb_neigh; ++j) {
                // Added tolerance: on Mac/M1 we got tiny differences,
                // I think it is doing auto FMA here and there, to be
                // checked.
#ifdef GEO_APPLE_M1
                if(::fabs(sq_dist1[j] - sq_dist2[j]) > 1e-6) {
#else
                    if(sq_dist1[j] != sq_dist2[j]) {
#endif
                        has_mismatch = true;
                        match = false;
                        Logger::err("Mismatch") << i << "[" << j << "]"
                                                << (sq_dist2[j] - sq_dist1[j])
                                                << " indices: "
                                                << neigh1[j] << " " << neigh2[j]
                                                << std::endl;
                    }
                }

                if(has_mismatch) {
                    {
                        std::ostream& out = Logger::err("Mismatch");
                        out << i << " ref ";
                        for(index_t j=0; j < nb_neigh; ++j) {
                            out << sq_dist1[j] << " ";
                        }
                        out << std::endl;
                    }
                    {
                        std::ostream& out = Logger::err("Mismatch");
                        out << i << " tst ";
                        for(index_t j=0; j < nb_neigh; ++j) {
                            out << sq_dist2[j] << " ";
                        }
                        out << std::endl;
                    }
                }


                /*
                  for(index_t j = 0; j < nb_neigh; ++j) {
                  if(sq_dist1[j] != sq_dist2[j]) {
                  Logger::err("NN Search")
                  << j << "th neighbor mismatches"
                  << std::endl;
                  match = false;
                  }
                  }
                */

            }
            if(match) {
                Logger::out("NN Search")
                    << NN1_algo << " and " << NN2_algo << " match."
                    << std::endl;
            } else {
                Logger::err("NN Search")
                    << NN1_algo << " and " << NN2_algo << " mismatch."
                    << std::endl;
                return 2;
            }
        }
        catch(const std::exception& e) {
            std::cerr << "Received an exception: " << e.what() << std::endl;
            return 1;
        }

        annClose();

        return 0;
    }
