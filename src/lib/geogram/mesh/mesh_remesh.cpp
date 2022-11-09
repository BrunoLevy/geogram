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

#include <geogram/mesh/mesh_remesh.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_halfedges.h>
#include <geogram/mesh/mesh_reorder.h>
#include <geogram/mesh/mesh_preprocessing.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_AABB.h>
#include <geogram/voronoi/CVT.h>
#include <geogram/NL/nl.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/stopwatch.h>
#include <geogram/basic/progress.h>
#include <geogram/bibliography/bibliography.h>

/****************************************************************************/

namespace GEO {

    void remesh_smooth(
        Mesh& M_in, Mesh& M_out,
        index_t nb_points,
        coord_index_t dim,
        index_t nb_Lloyd_iter,
        index_t nb_Newton_iter,
        index_t Newton_m,
	bool adjust,
	double adjust_max_edge_distance

    ) {

        geo_cite("DBLP:journals/cgf/YanLLSW09");
        geo_cite("DBLP:conf/imr/LevyB12");
	
        if(dim == 0) {
            dim = coord_index_t(M_in.vertices.dimension());
        }

	geo_argused(dim); // See TODO later.
	
        Stopwatch W("Remesh");

        CentroidalVoronoiTesselation CVT(&M_in);
       
        /*
         * TODO: reactivate projection, debug
         vector<vec3> R3_embedding(M_in.vertices.nb());
         for(index_t i=0; i<M_in.vertices.nb(); ++i) {
             R3_embedding[i] = vec3(M_in.vertices.point_ptr(i));
         }
         CentroidalVoronoiTesselation CVT(&M_in, R3_embedding, dim);
        */
       
        if(nb_points == 0) {
            nb_points = M_in.vertices.nb();
        }
        CVT.compute_initial_sampling(nb_points);

        try {
            ProgressTask progress("Lloyd", 100);
            CVT.set_progress_logger(&progress);
            CVT.Lloyd_iterations(nb_Lloyd_iter);
        }
        catch(const TaskCanceled&) {
            // TODO_CANCEL
        }

        if(nb_Newton_iter != 0) {
            try {
                ProgressTask progress("Newton", 100);
                CVT.set_progress_logger(&progress);
                CVT.Newton_iterations(nb_Newton_iter, Newton_m);
            }
            catch(const TaskCanceled&) {
                // TODO_CANCEL
            }
        }

        if(M_in.vertices.dimension() == 6 &&
           CmdLine::get_arg_bool("dbg:save_6d")
        ) {
            Logger::out("Remesh")
                << "Saving source mesh into mesh6.obj6" << std::endl;
            mesh_save(M_in, "mesh6.obj6");
            Logger::out("Remesh")
                << "Saving sampling into points6.txt" << std::endl;
            std::ofstream out("points6.txt");
            out << CVT.delaunay()->nb_vertices() << std::endl;
            for(index_t i = 0; i < CVT.delaunay()->nb_vertices(); i++) {
                for(coord_index_t c = 0; c < 6; c++) {
                    out << CVT.delaunay()->vertex_ptr(i)[c] << " ";
                }
                out << std::endl;
            }
        }

        // Delete auxiliary storage used for each threads (it uses a lot of RAM,
        //   we need this RAM to create the surface now...)
        CVT.RVD()->delete_threads();

        CVT.set_use_RVC_centroids(
            CmdLine::get_arg_bool("remesh:RVC_centroids")
        );
        bool multi_nerve = CmdLine::get_arg_bool("remesh:multi_nerve");

        Logger::out("Remesh") << "Computing RVD..." << std::endl;

        CVT.compute_surface(&M_out, multi_nerve);
        if(CmdLine::get_arg_bool("dbg:save_ANN_histo")) {
            Logger::out("ANN")
                << "Saving histogram to ANN_histo.dat" << std::endl;
            std::ofstream out("ANN_histo.dat");
            CVT.delaunay()->save_histogram(out);
        }

	if(adjust) {
	    mesh_adjust_surface(M_out, M_in, adjust_max_edge_distance);
	}
    }

    /************************************************************************/

    /**
     * \brief Gets the nearest point on a surface along a ray
     * \param[in] AABB the facets of the surface 
     *  as a MeshFacetsAABB
     * \param[in] R1 the ray, both directions are tested to find
     *  the nearest point
     * \param[in] max_dist if the nearest point is further away
     *  than \p max_dist, then the origin of the ray is returned
     */
    inline vec3 nearest_along_bidirectional_ray(
	const MeshFacetsAABB& AABB, const Ray& R1,
	double max_dist
    ) {
	vec3 p = R1.origin;
	vec3 result = p;
	Ray R2(R1.origin, -R1.direction);
	MeshFacetsAABB::Intersection I1;
	MeshFacetsAABB::Intersection I2;
	bool has_I1 = AABB.ray_nearest_intersection(R1,I1);
	bool has_I2 = AABB.ray_nearest_intersection(R2,I2);
	if(has_I1 && !has_I2) {
	    result = I1.p;
	}
	if(!has_I1 && has_I2) {
	    result = I2.p;
	}
	if(has_I1 && has_I2) {
	    if(Geom::distance2(p,I1.p) < Geom::distance2(p,I2.p)) {
		result = I1.p;
	    } else {
		result = I2.p;
	    }
	}
	if(Geom::distance2(result,p) > max_dist*max_dist) {
	    result = p;
	}
	return result;
    }
    
    /************************************************************************/    

    void GEOGRAM_API mesh_adjust_surface(
	Mesh& surface,
	Mesh& reference,
	double max_edge_distance
    ) {
	MeshFacetsAABB AABB(reference);

        // vertex normal
	vector<vec3> Nv(surface.vertices.nb(), vec3(0.0, 0.0, 0.0));

	// average edge length incident to vertex
	vector<double> Lv(surface.vertices.nb(), 0.0);

	vector<index_t> Cv(surface.vertices.nb(), 0.0);
	for(index_t f: surface.facets) {
	    vec3 n = Geom::mesh_facet_normal(surface, f);
	    index_t d = surface.facets.nb_vertices(f);
	    for(index_t lv=0;lv<d;++lv) {
		index_t v1 = surface.facets.vertex(f,lv);
		index_t v2 = surface.facets.vertex(
		    f,(lv==d-1)?0:lv+1
		);
		Nv[v1] += n;
		double l = Geom::distance2(
		    vec3(surface.vertices.point_ptr(v1)),
		    vec3(surface.vertices.point_ptr(v2))	    
		);
		l = ::sqrt(l);
		Lv[v1] += l;
		Lv[v2] += l;
		Cv[v1]++;
		Cv[v2]++;
	    }
	}
	for(index_t v: surface.vertices) {
	    if(Cv[v] != 0) {
		Lv[v] /= double(Cv[v]);
	    }
	}
    
	// nearest point along vertex normal
	vector<vec3> Qv(surface.vertices.nb());
	for(index_t v: surface.vertices) {
	    vec3 p(surface.vertices.point_ptr(v));
	    Qv[v] = nearest_along_bidirectional_ray(
		AABB, Ray(p, Nv[v]),max_edge_distance*Lv[v]
	    );
	}
    
	nlNewContext();
	nlSolverParameteri(NL_LEAST_SQUARES, NL_TRUE);	    
	nlSolverParameteri(
	    NL_NB_VARIABLES, NLint(surface.vertices.nb())
	);
    
	nlBegin(NL_SYSTEM);
	nlBegin(NL_MATRIX);
	for(index_t v: surface.vertices) {
	    // p + lambda_v * Nv = q ---> lambda_v * Nv = q - v
	    for(index_t c=0; c<3; ++c) {
		nlBegin(NL_ROW);
		nlCoefficient(v,Nv[v][c]);
		nlRightHandSide(
		    Qv[v][c] - surface.vertices.point_ptr(v)[c]
		);
		nlEnd(NL_ROW);
	    }
	}
	
	for(index_t f: surface.facets) {
	    index_t d = surface.facets.nb_vertices(f);
	    
	    vec3 Nf(0.0, 0.0, 0.0);
	    vec3 Pf;
	    double Lf=0.0; 
	    
	    for(index_t lv=0; lv<d; ++lv) {
		index_t v= surface.facets.vertex(f,lv);
		Nf += Nv[v];
		Pf += vec3(surface.vertices.point_ptr(v));
		Lf += Lv[v];
	    }
	    Pf = (1.0 / double(d))*Pf;
	    Lf = (1.0 / double(d))*Lf;
	    vec3 Qf = nearest_along_bidirectional_ray(
		AABB, Ray(Pf, Nf), max_edge_distance*Lf
	    );
	    
	    // 1/d(Sum Pv + lambda_v Nv) = Qf
	    // --> Pf + 1/d(Sum lambda_v Nv) = Qf
	    // --> Sum (1/d lambda_v Nv) = Qf - Pf
	    for(index_t c=0; c<3; ++c) {
		nlBegin(NL_ROW);
		for(index_t lv=0; lv<d; ++lv) {
		    index_t v = surface.facets.vertex(f,lv);
		    nlCoefficient(v,Nv[v][c]/double(d));
		}
		nlRightHandSide(Qf[c]-Pf[c]);
		nlEnd(NL_ROW);
	    }
	}
	nlEnd(NL_MATRIX);
	nlEnd(NL_SYSTEM);
	
	nlSolve();
	for(index_t v: surface.vertices) {
	    vec3 p(surface.vertices.point_ptr(v));
	    p = p + nlGetVariable(v)*Nv[v];
	    for(index_t c=0; c<3; ++c) {
		surface.vertices.point_ptr(v)[c] = p[c];
	    }
	}
	nlDeleteContext(nlGetCurrent());
    }
}

