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

#include <geogram/parameterization/mesh_atlas_maker.h>
#include <geogram/parameterization/mesh_LSCM.h>
#include <geogram/parameterization/mesh_ABF.h>
#include <geogram/parameterization/mesh_segmentation.h>
#include <geogram/parameterization/mesh_param_validator.h>
#include <geogram/parameterization/mesh_param_packer.h>
#include <geogram/points/principal_axes.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/basic/progress.h>
#include <deque>
#include <stack>

// TODO: angle threshold does not seem to work, to be debugged...
// TODO: generate number of segments in function of sum of angular defect.

namespace {
    using namespace GEO;

    /**
     * \brief Computes a mesh parameterization by projection onto the
     *  least squares average plane.
     * \param[in] chart the chart to be parameterized.
     * \param[in] corner_tex_coord a reference to the corner tex coord
     *  attribute where to store the computed texture coordinates.
     */
    void GEOGRAM_API chart_parameterize_by_projection(
	Chart& chart, Attribute<double>& corner_tex_coord
    ) {
	Mesh& M = chart.mesh;
	vec3 N;
	vec3 center;

	if(chart.facets.size() == 1) {
	    index_t f = chart.facets[0];
	    center = Geom::mesh_facet_center(M, f);
	    N = Geom::mesh_facet_normal(M,f);
	} else {
	    PrincipalAxes3d LSN;
	    LSN.begin();
	    for(index_t ff=0; ff<chart.facets.size(); ++ff) {
		index_t f=chart.facets[ff];
		for(index_t c: M.facets.corners(f)) {
		    index_t v = M.facet_corners.vertex(c);
		    LSN.add_point(vec3(M.vertices.point_ptr(v)));		
		}
	    }
	    LSN.end();
	    center = LSN.center();
	    N = LSN.normal();
	}
	
	vec3 U = normalize(Geom::perpendicular(N));
	vec3 V = normalize(cross(N,U));

	for(index_t ff=0; ff<chart.facets.size(); ++ff) {
	    index_t f=chart.facets[ff];
	    for(index_t c: M.facets.corners(f)) {
		index_t v = M.facet_corners.vertex(c);
		vec3 p(M.vertices.point_ptr(v));
		p -= center;
		double pu = dot(p,U);
		double pv = dot(p,V);
		corner_tex_coord[2*c]   = pu;
		corner_tex_coord[2*c+1] = pv;
	    }
	}
    }
    
    /**
     * \brief Computes a texture atlas.
     */
    class AtlasMaker {
    public:
	
	AtlasMaker(Mesh& mesh) :
	    mesh_(mesh),
	    hard_angles_threshold_(0.0),
	    chart_(mesh.facets.attributes(),"chart"),
	    vertex_id_(mesh.vertices.attributes(),"id")	{
	    tex_coord_.bind_if_is_defined(
		mesh_.facet_corners.attributes(), "tex_coord"
	    );
	    if(tex_coord_.is_bound()) {
		geo_assert(tex_coord_.dimension() == 2);
	    } else {
		tex_coord_.create_vector_attribute(
		    mesh_.facet_corners.attributes(), "tex_coord", 2
		);
	    }
	    chart_tex_coord_.create_vector_attribute(
		chart_as_mesh_.vertices.attributes(), "tex_coord", 2
	    );
	    for(index_t v: mesh_.vertices) {
		vertex_id_[v] = NO_VERTEX;
	    }
	    chart_parameterizer_ = PARAM_ABF;
	    verbose_ = false;
	}

	~AtlasMaker() {
	   // Destroy attributes
	    if(vertex_id_.is_bound()) {
		vertex_id_.destroy();
	    }
	    if(chart_.is_bound()) {
		chart_.destroy();
	    }
	    Attribute<double> facet_distance;
	    facet_distance.bind_if_is_defined(
		mesh_.facets.attributes(), "distance"
	    );
	    if(facet_distance.is_bound()) {
		facet_distance.destroy();
	    }
	}

	void set_verbose(bool x) {
	    verbose_ = x;
	    validator_.set_verbose(x);
	}
	
	void set_hard_angles_threshold(double x) {
	    hard_angles_threshold_ = x;
	}

	void set_chart_parameterizer(ChartParameterizer param) {
	    chart_parameterizer_ = param;
	}
	
	void make_atlas() {
	    index_t total_f = mesh_.facets.nb();
	    index_t param_f = 0;
	    segment_mesh();
	    for(index_t f: mesh_.facets) {
		while(chart_[f] >= chart_queue_.size()) {
		    chart_queue_.push_back(Chart(mesh_, chart_[f]));
		}
		chart_queue_[chart_[f]].facets.push_back(f);
	    }
	    ProgressTask progress("Atlas",100);
	    progress.progress(0);
	    try {
		while(!chart_queue_.empty()) {
		    Chart& current_chart = chart_queue_.front();
		    if(
			precheck_chart(current_chart) &&
			parameterize_chart(current_chart) &&
			postcheck_chart(current_chart)
		    ) {
			param_f += current_chart.facets.size();
			progress.progress(param_f * 100 / total_f);
		    } else {
			split_chart(current_chart);
		    }
		    chart_queue_.pop_front();
		}
	    } catch(...) {
		Logger::out("Atlas") << "Job canceled" << std::endl;
	    }
	}
	
    protected:

	/* // [BL] temporary code to display current state for 
           // fine-tuning, debugging
	void sanity_check() {
	    std::cerr << "nb_charts_=" << nb_charts_ << std::endl;
	    for(std::deque<Chart>::iterator it=chart_queue_.begin();
		it != chart_queue_.end(); ++it) {
		std::cerr << "Chart: id=" << it->id
			  << " nb facets=" << it->facets.size() << std::endl;
	    }	    
	    for(std::deque<Chart>::iterator it=chart_queue_.begin();
		it != chart_queue_.end(); ++it) {
		Chart& chart = *it;
		for(index_t ff=0; ff<chart.facets.size(); ++ff) {
		    index_t f = chart.facets[ff];
		    geo_assert(chart_[f] == chart.id);
		}
	    }
        } 
        */
	
	bool precheck_chart(Chart& chart) {
	    if(chart.nb_edges_on_border() == 0) {
		return false;
	    }

	    if(
		chart.facets.size() > 3000 &&
		chart.is_sock()
	    ) {
		return false;
	    }
	    
#ifdef GEO_OS_ANDROID
	    return (chart.facets.size() < 3000);
#endif	    
	    return true;
	}

	bool postcheck_chart(Chart& chart) {
	    bool OK = validator_.chart_is_valid(chart);
	    // If a small chart has a problem, then try
	    // simply to project it.
	    if(!OK && chart.facets.size() <= 10) {
		chart_parameterize_by_projection(chart, tex_coord_);
		// Some single-facet charts may fail to be validated if
		// they are too skinny (filling ratio will be too bad),
		// so we force accept if there is a single facet.
		OK = (chart.facets.size() == 1)  ||
		      validator_.chart_is_valid(chart);
	    }
	    return OK;
	}
	
	void split_chart(Chart& chart) {
	    chart_queue_.push_back(Chart(mesh_, chart.id));
	    Chart& chart1 = chart_queue_.back();
	    chart_queue_.push_back(Chart(mesh_, nb_charts_));
	    Chart& chart2 = chart_queue_.back();
	    ++nb_charts_;
	    split_chart_along_principal_axis(chart, chart1, chart2);
	}
	
	bool parameterize_chart(Chart& chart) {

	    if(chart_parameterizer_ == PARAM_PROJECTION) {
	       chart_parameterize_by_projection(chart, tex_coord_);	       
	       return true;
	    }
	   
	    chart_as_mesh_.clear();
	    chart_as_mesh_.vertices.set_dimension(3);
	    
	    // Assign vertices ids and copy vertices
	    index_t cur_vertex = 0;
	    for(index_t ff=0; ff<chart.facets.size(); ++ff) {
		index_t f = chart.facets[ff];
		for(index_t c: chart.mesh.facets.corners(f)) {
		    index_t v = chart.mesh.facet_corners.vertex(c);
		    if(vertex_id_[v] == NO_VERTEX) {
			chart_as_mesh_.vertices.create_vertex(
			    mesh_.vertices.point_ptr(v)
			    );
			vertex_id_[v] = cur_vertex;
			++cur_vertex;
		    }
		}
	    }
	    
	    // Create facets (and triangulate them)
	    for(index_t ff=0; ff<chart.facets.size(); ++ff) {
		index_t f = chart.facets[ff];
		index_t c1 = chart.mesh.facets.corners_begin(f);
		index_t v1 = vertex_id_[chart.mesh.facet_corners.vertex(c1)];
		for(index_t c2 = c1+1;
		    c2+1 < chart.mesh.facets.corners_end(f); ++c2
		) {
		    index_t c3=c2+1;
		    index_t v2=vertex_id_[chart.mesh.facet_corners.vertex(c2)];
		    index_t v3=vertex_id_[chart.mesh.facet_corners.vertex(c3)];
		    geo_assert(v1 < cur_vertex);
		    geo_assert(v2 < cur_vertex);
		    geo_assert(v3 < cur_vertex);		    
		    chart_as_mesh_.facets.create_triangle(v1,v2,v3);
		}
	    }
	    chart_as_mesh_.facets.connect();

	    if(chart_as_mesh_.facets.nb() < 5) {
		mesh_compute_LSCM(
		    chart_as_mesh_, "tex_coord", false, "", verbose_
		);
	    } else {
		switch(chart_parameterizer_) {
		    case PARAM_LSCM:
			mesh_compute_LSCM(
			    chart_as_mesh_, "tex_coord", false, "", verbose_
			);
			break;
		    case PARAM_SPECTRAL_LSCM:
			mesh_compute_LSCM(
			    chart_as_mesh_, "tex_coord", true, "", verbose_
			);
			break;
		    case PARAM_ABF:
			mesh_compute_ABF_plus_plus(
			    chart_as_mesh_, "tex_coord", verbose_
			);
			break;
		    case PARAM_PROJECTION:
		        geo_assert_not_reached;
		        break;
		}
	    }
	    
	    // Copy tex coords
	    for(index_t ff=0; ff<chart.facets.size(); ++ff) {
		index_t f = chart.facets[ff];
		for(index_t c: chart.mesh.facets.corners(f)) {
		    index_t v = vertex_id_[chart.mesh.facet_corners.vertex(c)];
			tex_coord_[2*c] = chart_tex_coord_[2*v];
			tex_coord_[2*c+1] = chart_tex_coord_[2*v+1];
		}		
	    }
	    
	    // Reset vertex ids
	    for(index_t ff=0; ff<chart.facets.size(); ++ff) {
		index_t f = chart.facets[ff];
		for(index_t c: chart.mesh.facets.corners(f)) {
		    vertex_id_[chart.mesh.facet_corners.vertex(c)] = NO_VERTEX;
		}
	    }
	    
	    return true;
	}

	void segment_mesh() {
	    for(index_t f: mesh_.facets) {
		chart_[f] = index_t(-1);
	    }
	    nb_charts_ = 0;
	    for(index_t f: mesh_.facets) {
		std::stack<index_t> S;
		if(chart_[f] == index_t(-1)) {
		    chart_[f] = nb_charts_;
		    S.push(f);
		    do {
			index_t cur_f = S.top();
			S.pop();
			for(index_t c: mesh_.facets.corners(cur_f)) {
			    index_t f2=mesh_.facet_corners.adjacent_facet(c);
			    if(
				f2 != NO_FACET &&
				chart_[f2] != nb_charts_ &&
				Geom::mesh_unsigned_normal_angle(
				    mesh_,cur_f,f2) < hard_angles_threshold_
			    ) {
				chart_[f2] = nb_charts_;
				S.push(f2);
			    }
			}
		    } while(!S.empty());
		    ++nb_charts_;
		}
	    }
	}
	
    private:
	Mesh& mesh_;
	ChartParameterizer chart_parameterizer_;
	ParamValidator validator_;
	double hard_angles_threshold_; // in radians
	Attribute<index_t> chart_;
	Attribute<index_t> vertex_id_;
	Attribute<double> tex_coord_;
	Mesh chart_as_mesh_;
	Attribute<double> chart_tex_coord_;
	std::deque<Chart> chart_queue_;
	index_t nb_charts_;
	bool verbose_;
    };

    /**
     * \brief Tests whether two facets are on the same chart
     * \param[in] mesh a reference to the mesh
     * \param[in] f1 a facet of the mesh
     * \param[in] e1 an edge of \p f1
     * \param[in] tex_coord a reference to the facet corners tex coords
     */
    bool is_same_chart(
	const Mesh& mesh,
	index_t f1, index_t e1, const Attribute<double>& tex_coord
    ) {
	index_t c11 = mesh.facets.corners_begin(f1)+e1;
	index_t c12 = mesh.facets.next_corner_around_facet(f1,c11); 	
	index_t f2  = mesh.facets.adjacent(f1,e1);

	geo_assert(f2 != index_t(-1));
	
	index_t e2  = mesh.facets.find_adjacent(f2,f1);
	index_t c21 = mesh.facets.corners_begin(f2)+e2;
	index_t c22 = mesh.facets.next_corner_around_facet(f2,c21); 		

	return
	    (tex_coord[2*c11  ] == tex_coord[2*c22  ]) &&
	    (tex_coord[2*c11+1] == tex_coord[2*c22+1]) &&
	    (tex_coord[2*c12  ] == tex_coord[2*c21  ]) &&
	    (tex_coord[2*c12+1] == tex_coord[2*c21+1]) ;
	    
    }
    
}

namespace GEO {

    void mesh_make_atlas(
	Mesh& mesh, double hard_angles_threshold, // in degrees
	ChartParameterizer param,
	ChartPacker pack,
	bool verbose 
    ) {
	AtlasMaker atlas(mesh);
	atlas.set_hard_angles_threshold(
	   hard_angles_threshold * M_PI / 180.0
	);
	atlas.set_chart_parameterizer(param);
	atlas.set_verbose(verbose);
	atlas.make_atlas();
	Packer packer;
        // If packer is not PACK_TETRIS,
        // normalize texture coords only -----.
        //                                    v
	packer.pack_surface(mesh, pack != PACK_TETRIS);
	if(pack == PACK_XATLAS) {
	    pack_atlas_using_xatlas(mesh);
	}
    }

    void mesh_get_charts(Mesh& mesh) {
	Attribute<index_t> chart(mesh.facets.attributes(),"chart");
	Attribute<double> tex_coord;
	tex_coord.bind_if_is_defined(
	    mesh.facet_corners.attributes(), "tex_coord"
	);
	if(!tex_coord.is_bound()) {
	    Logger::err("Chart") << "mesh does not have facet corner tex coords"
				 << std::endl;
	    return;
	}
	if(tex_coord.dimension() != 2) {
	    Logger::err("Chart") << "facet corner tex coords not of dimension 2"
				 << std::endl;
	    return;
	}
	chart.fill(index_t(-1));
	std::stack<index_t> S;
	index_t current_chart = 0;
	for(index_t f : mesh.facets) {
	    if(chart[f] == index_t(-1)) {
		chart[f] = current_chart;
		S.push(f);
		while(!S.empty()) {
		    index_t f1 = S.top();
		    S.pop();
		    for(index_t e=0; e<mesh.facets.nb_vertices(f1); ++e) {
			index_t f2 = mesh.facets.adjacent(f1,e);
			if(
			    f2 != index_t(-1) &&
			    chart[f2] == index_t(-1) &&
			    is_same_chart(mesh,f1,e,tex_coord)
			) {
			    chart[f2] = current_chart;
			    S.push(f2);
			}
		    }
		}
		current_chart++;		
	    }
	}
    }
    
}
