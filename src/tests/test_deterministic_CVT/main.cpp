/*
 *  Copyright (c) 2000-2024 Inria
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

// Regression test: checks that CVT-based surface remeshing produces
// bit-for-bit identical results regardless of the number of threads used.
// This validates the deterministic reductions implemented in RVD.cpp (enabled
// with GEOGRAM_WITH_DETERMINISTIC_CVT).

#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_remesh.h>
#include <geogram/mesh/mesh_subdivision.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/process.h>

#include <limits>

namespace {
    using namespace GEO;

    // Builds a triangulated box, refined nb_subdiv times with midpoint splits.
    void make_input_mesh(Mesh& M, index_t nb_subdiv) {
        M.clear();
        M.vertices.set_dimension(3);
        for(index_t i=0; i<8; ++i) {
            M.vertices.create_vertex(
                vec3((i&1)?1:-1, (i&2)?1:-1, (i&4)?1:-1).data()
            );
        }
        index_t q[6][4] = {
            {0,2,3,1}, {4,5,7,6}, {0,1,5,4},
            {2,6,7,3}, {0,4,6,2}, {1,3,7,5}
        };
        for(index_t f=0; f<6; ++f) {
            M.facets.create_triangle(q[f][0], q[f][1], q[f][2]);
            M.facets.create_triangle(q[f][0], q[f][2], q[f][3]);
        }
        M.facets.connect();
        for(index_t k=0; k<nb_subdiv; ++k) {
            mesh_split_triangles(M);
        }
    }

    // Remeshes a copy of M_in with nb_points using nb_threads, and returns the
    // resulting vertex coordinates.
    vector<double> remesh(
        const Mesh& M_in, index_t nb_points, index_t nb_threads
    ) {
        Process::set_max_threads(nb_threads);
        Mesh M(3), M_out;
        M.copy(M_in);
        remesh_smooth(M, M_out, nb_points, 3, 20, 30);
        vector<double> result(3 * M_out.vertices.nb());
        for(index_t v=0; v<M_out.vertices.nb(); ++v) {
            const double* p = M_out.vertices.point_ptr(v);
            result[3*v+0] = p[0];
            result[3*v+1] = p[1];
            result[3*v+2] = p[2];
        }
        return result;
    }

    // Counts coordinates that are not bit-for-bit identical.
    index_t count_differences(const vector<double>& a, const vector<double>& b) {
        if(a.size() != b.size()) {
            return index_t(-1);
        }
        index_t nb_diff = 0;
        for(index_t i=0; i<a.size(); ++i) {
            if(std::fabs(a[i]-b[i]) >= std::numeric_limits<double>::denorm_min()) {
                ++nb_diff;
            }
        }
        return nb_diff;
    }
}

int main(int argc, char** argv) {
    using namespace GEO;

    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
    CmdLine::import_arg_group("standard");
    CmdLine::import_arg_group("algo");
    CmdLine::import_arg_group("remesh");

    std::vector<std::string> filenames;
    if(!CmdLine::parse(argc, argv, filenames, "")) {
        return 1;
    }

    Mesh M_in;
    make_input_mesh(M_in, 5); // ~4k facets, enough to create several parts
    const index_t nb_points = 500;

    Logger::out("Determinism")
        << "Input surface: " << M_in.vertices.nb() << " vertices, "
        << M_in.facets.nb() << " facets; remeshing with " << nb_points
        << " points" << std::endl;

    // Reference result computed with a single thread.
    vector<double> ref = remesh(M_in, nb_points, 1);

    index_t thread_counts[] = {1, 2, 3, 4, 8};
    bool ok = true;
    for(index_t k=0; k<sizeof(thread_counts)/sizeof(index_t); ++k) {
        index_t nt = thread_counts[k];
        vector<double> res = remesh(M_in, nb_points, nt);
        index_t nb_diff = count_differences(ref, res);
        if(nb_diff == 0) {
            Logger::out("Determinism")
                << "nb_threads = " << nt << ": OK" << std::endl;
        } else {
            Logger::err("Determinism")
                << "nb_threads = " << nt << ": FAILED (" << nb_diff
                << " differing coordinates)" << std::endl;
            ok = false;
        }
    }

    if(!ok) {
        Logger::err("Determinism")
            << "Remeshing is NOT deterministic across thread counts."
            << std::endl;
        return 1;
    }
    Logger::out("Determinism")
        << "Remeshing is deterministic across all tested thread counts."
        << std::endl;
    return 0;
}
