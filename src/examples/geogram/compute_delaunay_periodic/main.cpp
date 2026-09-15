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
#include <geogram/basic/file_system.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/delaunay/periodic_delaunay_3d.h>
#include <algorithm>

namespace {
    using namespace GEO;

    static constexpr index_t dimension = 3;

    bool load_points(
        const std::string& points_filename,
	index_t dim,
        vector<double>& points
    ) {
        Mesh M;
        MeshIOFlags flags;
        flags.reset_element(MESH_FACETS);
        flags.reset_element(MESH_CELLS);
        if(!mesh_load(points_filename, M, flags)) {
            return false;
        }
        M.vertices.set_dimension(dim);
        index_t nb_points = M.vertices.nb();
        points.resize(nb_points * dim);
        Memory::copy(
            points.data(),
            M.vertices.point_ptr(0),
            M.vertices.nb()*dim*sizeof(double)
        );
	if(CmdLine::get_arg_bool("normalize")) {
	    double xyz_min[3];
	    double xyz_max[3];
	    get_bbox(M, xyz_min, xyz_max);
	    index_t N = M.vertices.nb();
	    for(index_t i=0; i<N; ++i) {
		for(index_t coord=0; coord<dim; ++coord) {
		    double c = points.data()[i*dim+coord];
		    c = (c - xyz_min[coord]) / (xyz_max[coord] - xyz_min[coord]);
		    points.data()[i*dim+coord] = c;
		}
	    }
	}
	return true;
    }


    bool save_result(PeriodicDelaunay3d* delaunay, const std::string& basename) {
	if(FileSystem::extension(basename) != "") {
	    Logger::err("Delaunay")
		<< "output basename should not have extension"
		<< std::endl;
	    return false;
	}
	Logger::out("Delaunay") << "Saving result to:" << basename << std::endl;
	bool clipped = !delaunay->periodic();
	delaunay->save_cells(basename, clipped);
	return true;
    }
}

int main(int argc, char** argv) {
    using namespace GEO;
    // Needs to be called once.
    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);

    try {
        Stopwatch Wtot("Total time");

        std::vector<std::string> filenames;

        CmdLine::import_arg_group("standard");
        CmdLine::import_arg_group("algo");

        CmdLine::declare_arg("periodic", false, "periodic boundary condition");
	CmdLine::declare_arg("verbose", false, "logging messages");
	CmdLine::declare_arg(
	    "detailed_verbose", false, "detailed logging messages"
	);
	CmdLine::declare_arg("normalize", false, "normalize coords to [0,1]");

        if(
	    !CmdLine::parse(
		argc, argv, filenames, "pointsfile <output_basename|none>"
	    )
        ) {
            return 1;
        }

        std::string points_filename = filenames[0];
        std::string output_basename =
            filenames.size() >= 2 ? filenames[1] : "none";

        vector<double> points;
        if(!load_points(points_filename, dimension, points)) {
            Logger::err("Delaunay") << "Could not load points" << std::endl;
            return 1;
        }

        index_t nb_points = points.size() / dimension;

        Logger::out("Delaunay")
            << "Loaded " << nb_points << " points" << std::endl;

	if(CmdLine::get_arg_bool("verbose")) {
	    CmdLine::set_arg("dbg:delaunay_benchmark", true);
	}

	if(CmdLine::get_arg_bool("detailed_verbose")) {
	    CmdLine::set_arg("dbg:detailed_delaunay_benchmark", true);
	}

	bool periodic = CmdLine::get_arg_bool("periodic");

	SmartPointer<PeriodicDelaunay3d> delaunay = new PeriodicDelaunay3d(
	    periodic
	);

	delaunay->use_exact_predicates_for_convex_cell(
	    CmdLine::get_arg("algo:predicates") == "exact"
	);

	delaunay->set_keeps_infinite(!periodic);


        double time = 0.0;
        {
            Stopwatch Wdel("Delaunay");
            // Note: this does not transfer ownership of memory, caller
            // is still responsible of the memory of the points (here the
            // vector<double>). No memory is copied, Delaunay just keeps
            // a pointer.
            delaunay->set_vertices(nb_points, points.data());
	    delaunay->compute();
            time = Wdel.elapsed_time();
        }

        Logger::out("Delaunay") << delaunay->nb_cells() << " tetrahedra"
                                << std::endl;

        Logger::out("Delaunay") << double(delaunay->nb_cells()) / time
                                << " tetrahedra / second"
                                << std::endl;

	if(!save_result(delaunay, output_basename)) {
	    Logger::err("Delaunay") << "Could not save result to file "
				    << output_basename
				    << std::endl;
	    return 1;
	}

    } catch(const std::exception& e) {
        std::cerr << "Received an exception: " << e.what() << std::endl;
        return 1;
    }

    Logger::out("") << "Everything OK, Returning status 0" << std::endl;
    return 0;
}
