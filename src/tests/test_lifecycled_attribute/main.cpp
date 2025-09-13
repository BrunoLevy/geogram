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
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_subdivision.h>
#include <geogram/basic/stopwatch.h>
#include <stack>

// Tests lifecycled attribute

namespace {
    using namespace GEO;

    void create_attribute(Mesh& M) {
	if(!CmdLine::get_arg_bool("with_string_attribute")) {
	    return;
	}
	Attribute<std::string> str_id(M.vertices.attributes(),"str_id");
	for(index_t v: M.vertices) {
	    str_id[v] = String::to_string(v);
	}
    }

    void create_geodesic_sphere(Mesh& M, index_t nb_subdivisions, bool quads) {
        static vec3 points[] = {
            {0,          0.0,       1.175571},
	    {1.051462,   0.0,       0.5257311},
	    {0.3249197,  1.0,       0.5257311},
	    {-0.8506508, 0.618034,  0.5257311},
	    {-0.8506508, -0.618034, 0.5257311},
	    {0.3249197,  -1.0,      0.5257311},
	    {0.8506508,  0.618034,  -0.5257311},
	    {0.8506508,  -0.618034, -0.5257311},
	    {-0.3249197,  1.0,      -0.5257311},
	    {-1.051462,   0.0,      -0.5257311},
	    {-0.3249197, -1.0,      -0.5257311},
	    {0.0,         0.0,      -1.175571}
        };

        static index_t facets[][3] = {
            {0,1,2},
	    {0,2,3},
	    {0,3,4},
	    {0,4,5},
	    {0,5,1},
	    {1,5,7},
            {1,7,6},
	    {1,6,2},
	    {2,6,8},
	    {2,8,3},
	    {3,8,9},
	    {3,9,4},
	    {4,9,10},
	    {4,10,5},
	    {5,10,7},
	    {6,7,11},
	    {6,11,8},
	    {7,10,11},
	    {8,11,9},
	    {9,11,10},
        };

	M.clear();
	M.vertices.set_dimension(3);

        M.vertices.create_vertices(12);
        for(index_t v=0; v<12; ++v) {
	    M.vertices.point(v) = points[v];
        }

        for(index_t f=0; f<20; ++f) {
            M.facets.create_triangle(
                facets[f][0],
                facets[f][1],
                facets[f][2]
            );
        }

        M.facets.connect();

	create_attribute(M);

	{
	    Stopwatch W("splitting");
	    for(index_t k=0; k<nb_subdivisions; ++k) {
		if(quads) {
		    mesh_split_quads(M);
		} else {
		    mesh_split_triangles(M);
		}
	    }
	}

	{
	    Stopwatch W("normalizing");
	    for(vec3& p:M.vertices.points()) {
		p = normalize(p);
	    }
	}
    }
}


int main(int argc, char** argv) {
    using namespace GEO;

    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);

    geo_register_attribute_type<std::string>("string");


    CmdLine::import_arg_group("standard");
    CmdLine::declare_arg(
	"nb_subdivisions",4,
	"number of subdivisions from icosahedron"
    );
    CmdLine::declare_arg(
	"quads", false, "use quads instead of triangles"
    );
    CmdLine::declare_arg(
	"with_string_attribute", true, "create a string attribute"
    );
    std::vector<std::string> filenames;
    if(!CmdLine::parse(argc, argv, filenames, "<inputfile>")) {
        return 1;
    }
    try {
       Mesh M;

       if(filenames.size() >= 1) {
	   mesh_load(filenames[0], M);
	   create_attribute(M);
	   index_t nb_subdivisions = CmdLine::get_arg_uint("nb_subdivisions");
	   bool quads = CmdLine::get_arg_bool("quads");
	   Stopwatch W("splitting");
	   for(index_t k=0; k<nb_subdivisions; ++k) {
	       if(quads) {
		   mesh_split_quads(M);
	       } else {
		   mesh_split_triangles(M);
	       }
	   }
       } else {
	   create_geodesic_sphere(
	       M,
	       CmdLine::get_arg_uint("nb_subdivisions"),
	       CmdLine::get_arg_bool("quads")
	   );
       }

    } catch(const std::exception& e) {
        std::cerr << "Received an exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
