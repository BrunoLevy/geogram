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
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_surface_intersection.h>

/*
 * Computes auto-intersections in a mesh.
 */


int main(int argc, char** argv) {
    using namespace GEO;

    // Needs to be called once.
    GEO::initialize();
    Stopwatch W_total("Total time");


    try {

        std::vector<std::string> filenames;

        CmdLine::import_arg_group("standard");
        CmdLine::import_arg_group("algo");
        CmdLine::declare_arg(
            "post",true,"glue triangle and unglue non-manifold edges"
        );
        CmdLine::declare_arg(
            "verbose",true,"makes intersection algorithm more chatty"
        );
        CmdLine::declare_arg(
            "Delaunay",true,"use Delaunay triangulation to remesh intersections"
        );
        CmdLine::declare_arg(
            "detect_intersecting_neighbors",true,
            "test also neighboring triangles for intersection"
        );
        CmdLine::declare_arg(
            "normalize",false,"normalize coordinates during computation"
        );
        CmdLine::declare_arg(
            "remove_internal_shells",true,"remove internal shells"
        );
        CmdLine::declare_arg(
            "simplify_coplanar_facets",true,"simplify coplanar facets"
        );
        CmdLine::declare_arg(
            "coplanar_angle_tolerance",0.0,
            "maximum angle (in degrees) between coplanar facets"
        );
        CmdLine::declare_arg("expr","","Region classification expression");
        CmdLine::declare_arg(
            "monster_threshold",100000,"monster threshold"
        );
        CmdLine::declare_arg(
            "dry_run",false,"Do not insert triangulations in global mesh"
        );
        CmdLine::declare_arg(
           "save_skeleton",false,
           "Save skeleton of intersection in skeleton.geogram"
        );
        
        if(
            !CmdLine::parse(
                argc, argv, filenames, "inputfile <outputfile|none>"
            )
        ) {
            return 1;
        }


        std::string output_filename =
            filenames.size() >= 2 ? filenames[1] : std::string("out.meshb");

        Logger::div("Data I/O");

        Logger::out("I/O") << "Output = " << output_filename << std::endl;

	Mesh A;

	if(!mesh_load(filenames[0],A)) {
	    return 1;
	}

	{
            Logger::div("Intersect");
	    Stopwatch Wintersect("Intersect");
            MeshSurfaceIntersection I(A);
            I.set_verbose(CmdLine::get_arg_bool("verbose"));
            I.set_delaunay(CmdLine::get_arg_bool("Delaunay"));
            I.set_detect_intersecting_neighbors(
                CmdLine::get_arg_bool("detect_intersecting_neighbors")
            );
            I.set_normalize(CmdLine::get_arg_bool("normalize"));
            I.set_radial_sort(
                CmdLine::get_arg_bool("remove_internal_shells") ||
                CmdLine::get_arg_bool("simplify_coplanar_facets") ||
                CmdLine::get_arg("expr") != ""
            );
            I.set_monster_threshold(
                CmdLine::get_arg_uint("monster_threshold")
            );
            I.set_dry_run(
                CmdLine::get_arg_bool("dry_run")
            );

            Mesh skel;
            if(CmdLine::get_arg_bool("save_skeleton")) {
                I.set_build_skeleton(&skel);
            }
            
            I.intersect();

            if(CmdLine::get_arg("expr") != "") {
                I.classify(CmdLine::get_arg("expr"));
            } else if(CmdLine::get_arg_bool("remove_internal_shells")) {
                I.remove_internal_shells();
            }

            if(CmdLine::get_arg_bool("simplify_coplanar_facets")) {
                I.simplify_coplanar_facets(
                    CmdLine::get_arg_double("coplanar_angle_tolerance")
                );
            }

            if(CmdLine::get_arg_bool("save_skeleton")) {
                mesh_save(skel,"skeleton.geogram");
            }
            
        }

        if(CmdLine::get_arg_bool("post")) {
            Logger::div("Post");
	    Stopwatch W_post("Post");
            mesh_repair(
                A,
                MeshRepairMode(
                    MESH_REPAIR_COLOCATE | MESH_REPAIR_DUP_F
                ),
                0.0
            );
	}
	

        Logger::div("Data I/O");
	
        if(output_filename != "none") {
	    mesh_save(A, output_filename);
	}

	
    }
    catch(const std::exception& e) {
        std::cerr << "Received an exception: " << e.what() << std::endl;
        return 1;
    }
    Logger::div("Total time");

    return 0;
}

