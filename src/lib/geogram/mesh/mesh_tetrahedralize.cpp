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
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_fill_holes.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh.h>
#include <geogram/delaunay/delaunay.h>
#include <geogram/delaunay/delaunay_tetgen.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/command_line.h>

namespace GEO {

    bool mesh_tetrahedralize(
        Mesh& M, const MeshTetrahedralizeParameters& parameters
    ) {

        bool verbose = parameters.verbose;
        bool preprocess = parameters.preprocess;
        double epsilon = parameters.preprocess_merge_vertices_epsilon;
        double max_hole_area = parameters.preprocess_fill_hole_max_area;
        bool refine = parameters.refine;
        double quality = parameters.refine_quality;
        bool keep_regions = parameters.keep_regions;
        
        if(!DelaunayFactory::has_creator("tetgen")) {
            Logger::err("TetMeshing")
                << "Not supported in this version" << std::endl;
            Logger::err("TetMeshing")
                << "(need to recompile with tetgen support)"
                << std::endl;
            return false;
        }

        if(!M.facets.are_simplices()) {
            Logger::warn("TetMeshing")
                << "Mesh is not triangulated (triangulating it)"
                << std::endl;
            tessellate_facets(M,3);
        }

        // in percent of bbox diagonal
        epsilon *= (0.01 * bbox_diagonal(M));            

        max_hole_area *= (0.01 * Geom::mesh_area(M));
        
        bool ok = true;
        Delaunay_var delaunay;
        
        for(index_t iter=0; iter<5; ++iter) {
            if(iter != 0 && verbose) {
                Logger::warn("Tetrahedralize")
                    << "Retrying, because tetgen may have moved some vertices"
                    << std::endl;
            }
            
            if(preprocess) {
                mesh_repair(M, MESH_REPAIR_DEFAULT, epsilon);
                fill_holes(M, max_hole_area);
                MeshSurfaceIntersection intersection(M);
                intersection.set_verbose(verbose);
                intersection.intersect();
                intersection.remove_internal_shells();
                if(iter == 0) {
                    intersection.simplify_coplanar_facets();
                }
                mesh_repair(M, MESH_REPAIR_DEFAULT, epsilon);
            }

            if(verbose) {
                Logger::out("TetMeshing") << "Tetrahedralizing..." << std::endl;
            }
            
            delaunay = Delaunay::create(3,"tetgen");
            delaunay->set_refine(refine);
            delaunay->set_quality(quality);
            delaunay->set_constraints(&M);
            delaunay->set_keep_regions(keep_regions);
            
            try {
                ok = true;
                delaunay->set_vertices(0,nullptr); // No additional vertex
                ok = ok &&
                    delaunay->nb_vertices() != 0 &&
                    delaunay->nb_cells() != 0;
                if(ok) {
                    break;
                }
            } catch(const Delaunay::InvalidInput& error_report) {
                geo_argused(error_report);
                if(verbose) {
                    Logger::warn("Tetrahedralize") << "Encountered error"
                                                   << std::endl;
                }
                ok = false;
                if(!preprocess) {
                    break;
                }
            } 
        }

        if(!ok) {
            Logger::err("Tetrahedralize") << "failed" << std::endl;
            return false;
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
            Logger::err("Tetrahedralize")
                << "Did not generate any tetrahedron"
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
        if(verbose) {
            M.show_stats("TetMeshing");
        }
        return true;
    }
    
}
