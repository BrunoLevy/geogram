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

#include <geogram/mesh/mesh_tetrahedralize.h>
#include <geogram/mesh/mesh_surface_intersection.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_intersection.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh.h>
#include <geogram/delaunay/delaunay.h>
#include <geogram/delaunay/delaunay_tetgen.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/command_line.h>

namespace GEO {

    bool mesh_tetrahedralize(
        Mesh& M, bool preprocess, bool refine,
        double quality, bool keep_regions,
        double epsilon
    ) {
        if(!DelaunayFactory::has_creator("tetgen")) {
            Logger::err("TetMeshing")
                << "Not supported in this version" << std::endl;
            Logger::err("TetMeshing")
                << "(need to recompile with tetgen support)"
                << std::endl;
            return false;
        }

        if(!M.facets.are_simplices()) {
            Logger::err("TetMeshing")
                << "Mesh is not triangulated"
                << std::endl;
            return false;
        }
        
        MeshSurfaceIntersectionParams params;
        
        // in percent of bbox diagonal
        epsilon *= (0.01 * bbox_diagonal(M));            
        
        if(preprocess) {
            mesh_repair(M, MESH_REPAIR_DEFAULT, epsilon);
            mesh_intersect_surface(M, params);
            mesh_classify_intersections(M, "union", "", false);
            /*
            mesh_remove_intersections(M);
            if(CmdLine::get_arg_bool("dbg:tetrahedralize")) {
                mesh_save(M, "tetrahedralize_input_repaired.meshb");
            }
            */
        }

        /*
        if(preprocess) {
            for(index_t c: M.facet_corners) {
                if(M.facet_corners.adjacent_facet(c) == NO_FACET) {
                    Logger::err("TetMeshing")
                        << "Mesh is not closed"
                        << std::endl;
                    return false;
                }
            }
        }
        */

        Logger::out("TetMeshing") << "Tetrahedralizing..." << std::endl;

        Delaunay_var delaunay = Delaunay::create(3,"tetgen");
        delaunay->set_refine(refine);
        delaunay->set_quality(quality);
        delaunay->set_constraints(&M);
	delaunay->set_keep_regions(keep_regions);
	
        try {
            delaunay->set_vertices(0,nullptr); // No additional vertex
        } catch(const Delaunay::InvalidInput& error_report) {

            Logger::warn("Tetrahedralize") << "Encountered error"
                                           << std::endl;
            
            // Try one more time, because tetgen *moves* vertices sometimes
            if(preprocess) {
                Logger::warn("Tetrahedralize")
                    << "Retrying, because tetgen may have moved some vertices"
                    << std::endl;
                
                delaunay.reset();

                delaunay = Delaunay::create(3,"tetgen");
                delaunay->set_refine(refine);
                delaunay->set_quality(quality);
                delaunay->set_constraints(&M);
                delaunay->set_keep_regions(keep_regions);
                
                mesh_repair(M, MESH_REPAIR_DEFAULT, epsilon);
                mesh_intersect_surface(M, params);
                mesh_classify_intersections(M, "union", "", false);

                try {
                    delaunay->set_vertices(0,nullptr); // No additional vertex
                } catch(const Delaunay::InvalidInput& error_report) {
                    Logger::warn("Tetrahedralize")
                        << "Encountered error (nothing we can do it seems...)"
                        << std::endl;
                    return false;
                }
            } else {
                return false;
            }
        }

        vector<double> pts(delaunay->nb_vertices() * 3);
        vector<index_t> tet2v(delaunay->nb_cells() * 4);
        for(index_t v = 0; v < delaunay->nb_vertices(); ++v) {
            pts[3 * v] = delaunay->vertex_ptr(v)[0];
            pts[3 * v + 1] = delaunay->vertex_ptr(v)[1];
            pts[3 * v + 2] = delaunay->vertex_ptr(v)[2];
        }
        for(index_t t = 0; t < delaunay->nb_cells(); ++t) {
            tet2v[4 * t] = index_t(delaunay->cell_vertex(t, 0));
            tet2v[4 * t + 1] = index_t(delaunay->cell_vertex(t, 1));
            tet2v[4 * t + 2] = index_t(delaunay->cell_vertex(t, 2));
            tet2v[4 * t + 3] = index_t(delaunay->cell_vertex(t, 3));
        }

        if(pts.size() == 0 || tet2v.size() == 0) {
            Logger::err("Tetrahedralize") << "Did not generate any tetrahedron"
                                          << std::endl;
            return false;
        }
        
        M.cells.assign_tet_mesh(3, pts, tet2v, true);

	if(keep_regions) {
	    Attribute<index_t> region(M.cells.attributes(), "region");
	    for(index_t t: M.cells) {
		region[t] = delaunay->region(t);
	    }
	}
	
        M.cells.connect();
        M.show_stats("TetMeshing");
        
        return true;
    }
    
}
