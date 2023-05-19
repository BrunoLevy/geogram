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

#include <geogram/mesh/mesh_surface_intersection.h>
#include <geogram/mesh/mesh_surface_intersection_internal.h>
#include <geogram/mesh/triangle_intersection.h>
#include <geogram/mesh/mesh_AABB.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_fill_holes.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/index.h>
#include <geogram/delaunay/CDT_2d.h>
#include <geogram/numerics/predicates.h>
#include <geogram/numerics/expansion_nt.h>
#include <geogram/basic/stopwatch.h>

#include <sstream>
#include <stack>

#ifdef GEO_COMPILER_CLANG
// I'm using long long 
#pragma GCC diagnostic ignored "-Wc++98-compat-pedantic"
#endif

// TODO
// - Understand why I still need to suppress duplicated facets
//   in mesh_tetrahedralize() right after remove_internal_shells().
// - Can we make cube.lua always work, by ensuring unique lifted
//   coordinates ? (indexed by exact point coordinate)
// - Flat triangle generation in fume_extractor.stl (assertion fail)
//     corrected with exact normal computation
//     now assertion fail "point outside triangle"
// - tetrapod.obj: needs multiple call to intersect() before tetgen
//   until there is no intersection
// - multi-component classification
// - integrate boolean op in class
// - parse OpenSCAD .csg files:
//       https://github.com/openscad/openscad/wiki/CSG-File-Format
// - cubes.lua: radial sort complains about coplanar facets
// - fume-extractor: crashes with point outside of triangle error


namespace {
    using namespace GEO;
    /**
     * \brief Removes all facets that have their three vertices aligned
     */
    void remove_degenerate_triangles(Mesh& M) {
        vector<index_t> remove_f(M.facets.nb());
        for(index_t f: M.facets) {
            index_t v1 = M.facets.vertex(f,0);
            index_t v2 = M.facets.vertex(f,1);
            index_t v3 = M.facets.vertex(f,2);            
            const double* p1 = M.vertices.point_ptr(v1);
            const double* p2 = M.vertices.point_ptr(v2);
            const double* p3 = M.vertices.point_ptr(v3);
            remove_f[f] = PCK::aligned_3d(p1,p2,p3);
        }
        M.facets.delete_elements(remove_f);
    }

    /**
     * \brief Enumerates the connected components in a facet attribute
     * \param[in] M a reference to the mesh
     * \param[in] attribute the name of the facet attribute
     * \return the number of found connected components
     */
    index_t get_surface_connected_components(
        Mesh& M, const std::string& attribute = "chart"
    ) {
        Attribute<index_t> chart(M.facets.attributes(), attribute);
        for(index_t f: M.facets) {
            chart[f] = index_t(-1);
        }
        std::stack<index_t> S;
        index_t cur_chart = 0;
        for(index_t f: M.facets) {
            if(chart[f] == index_t(-1)) {
                chart[f] = cur_chart;
                S.push(f);
                while(!S.empty()) {
                    index_t g = S.top();
                    S.pop();
                    for(
                        index_t le=0;
                        le<M.facets.nb_vertices(g); ++le
                    ) {
                        index_t h = M.facets.adjacent(g,le);
                        if(h != index_t(-1) && chart[h] == index_t(-1)) {
                            chart[h] = cur_chart;
                            S.push(h);
                        }
                    }
                }
                ++cur_chart;
            }
        }
        return cur_chart;
    }

    /**
     * \brief Computes the intersection between two mesh triangular facets
     * \details This function is just a wrapper around triangles_intersections()
     *  for Mesh facets.
     * \param[in] M the mesh
     * \param[in] f1 , f2 the two facets
     * \param[out] I a vector of triangle intersections
     * \retval true if there was an intersection
     * \retval false otherwise
     */
    bool mesh_facets_intersect(
        Mesh& M, index_t f1, index_t f2, vector<TriangleIsect>& I
    ) {
        geo_debug_assert(M.facets.nb_vertices(f1) == 3);
        geo_debug_assert(M.facets.nb_vertices(f2) == 3);        
        vec3 p1(M.vertices.point_ptr(M.facets.vertex(f1,0)));
        vec3 p2(M.vertices.point_ptr(M.facets.vertex(f1,1)));
        vec3 p3(M.vertices.point_ptr(M.facets.vertex(f1,2)));
        vec3 q1(M.vertices.point_ptr(M.facets.vertex(f2,0)));
        vec3 q2(M.vertices.point_ptr(M.facets.vertex(f2,1)));
        vec3 q3(M.vertices.point_ptr(M.facets.vertex(f2,2)));
        return triangles_intersections(p1,p2,p3,q1,q2,q3,I);
    }
}


namespace GEO {
    
    MeshSurfaceIntersection::MeshSurfaceIntersection(Mesh& M) :
        lock_(GEOGRAM_SPINLOCK_INIT),
        mesh_(M),
        vertex_to_exact_point_(M.vertices.attributes(), "exact_point"),
        radial_sort_(*this),
        normalize_(true) {
        for(index_t v: mesh_.vertices) {
            vertex_to_exact_point_[v] = nullptr;
        }
        verbose_ = false;
        delaunay_ = true;
        approx_incircle_ = false;
        detect_intersecting_neighbors_ = true;
        use_radial_sort_ = true;
    }

    MeshSurfaceIntersection::~MeshSurfaceIntersection() {
        vertex_to_exact_point_.destroy();
    }

    void MeshSurfaceIntersection::remove_external_shell() {
        vector<index_t> remove_f;
        mark_external_shell(remove_f);
        mesh_.facets.delete_elements(remove_f);
    }
    
    void MeshSurfaceIntersection::remove_internal_shells() {
        vector<index_t> remove_f;
        mark_external_shell(remove_f);
        for(index_t& i: remove_f) {
            i= 1-i;
        }
        mesh_.facets.delete_elements(remove_f);
    }

    
    void MeshSurfaceIntersection::intersect() {

        // Step 1: Preparation
        // -------------------
        
        if(!mesh_.facets.are_simplices()) {
            tessellate_facets(mesh_,3);
        }

        Attribute<index_t> operand_bit;
        operand_bit.bind_if_is_defined(
            mesh_.facets.attributes(), "operand_bit"
        );
        if(!operand_bit.is_bound()) {
            get_surface_connected_components(mesh_,"operand_bit");
            operand_bit.bind(mesh_.facets.attributes(), "operand_bit");
            for(index_t f: mesh_.facets) {
                operand_bit[f] = (index_t(1) << operand_bit[f]) ;
            }
        }

        remove_degenerate_triangles(mesh_);        
        mesh_colocate_vertices_no_check(mesh_);
        mesh_remove_bad_facets_no_check(mesh_);

        // Set symbolic perturbation mode to lexicographic order
        // on point coordinates instead of point indices only,
        // Needed to get compatible triangulations on coplanar faces
        // (example, cubes that touch on a facet).
        PCK::SOSMode SOS_bkp = PCK::get_SOS_mode();
        PCK::set_SOS_mode(PCK::SOS_LEXICO);

        const double SCALING = double(1ull << 20); 
        const double INV_SCALING = 1.0/SCALING;

        if(normalize_) {
            double xyz_min[3];
            double xyz_max[3];
            get_bbox(mesh_, xyz_min, xyz_max);
            normalize_center_ = vec3(
                0.5*(xyz_min[0] + xyz_max[0]),
                0.5*(xyz_min[1] + xyz_max[1]),
                0.5*(xyz_min[2] + xyz_max[2])
            );
            normalize_radius_ = -Numeric::max_float64();
            for(coord_index_t c=0; c<3; ++c) {
                normalize_radius_ = std::max(
                    normalize_radius_, 0.5*(xyz_max[1] - xyz_min[1])
                );
            }
            double s = 1.0/normalize_radius_;
            for(index_t v: mesh_.vertices) {
                double* p = mesh_.vertices.point_ptr(v);
                for(coord_index_t c=0; c<3; ++c) {
                    p[c] = s * (p[c]-normalize_center_[c]);
                }
            }
        }
        
        // Pre-scale everything by 2^20 to avoid underflows
        // (note: this just adds 20 to the exponents of all
        //  coordinates).
        {
            double* p = mesh_.vertices.point_ptr(0);
            index_t N = mesh_.vertices.nb() *
                        mesh_.vertices.dimension();
            for(index_t i=0; i<N; ++i) {
                p[i] *= SCALING;
            }
        }
        
        // Step 2: Get intersections
        // -------------------------
        
        vector<IsectInfo> intersections;
        {
            Stopwatch W("Detect isect");
            MeshFacetsAABB AABB(mesh_,true);
            vector<std::pair<index_t, index_t> > FF;

            // Get candidate pairs of intersecting facets
            AABB.compute_facet_bbox_intersections(
                [&](index_t f1, index_t f2) {
                    // Needed (maybe I should change that in AABB class)
                    if(f1 == f2) {
                        return;
                    }
                    geo_assert(f1 < f2);

                    // Optionally skip facet pairs that
                    // share a vertex or an edge
                    if(
                        !detect_intersecting_neighbors_ && (
                          (mesh_.facets.find_adjacent(f1,f2)!=index_t(-1)) ||
                          (mesh_.facets.find_common_vertex(f1,f2)!=index_t(-1))
                        )
                    ) {
                        return;
                    }
                    FF.push_back(std::make_pair(f1,f2));
                }
            );

            // Compute facet-facet intersections in parallel
            Process::spinlock lock = GEOGRAM_SPINLOCK_INIT;
            parallel_for_slice(
                0,FF.size(), [&](index_t b, index_t e) {
                    vector<TriangleIsect> I;
                    for(index_t i=b; i<e; ++i){
                        index_t f1 = FF[i].first;
                        index_t f2 = FF[i].second;
                        
                        if(mesh_facets_intersect(mesh_,f1, f2, I)) {

                            Process::acquire_spinlock(lock);
                            
                            if(I.size() > 2) {
                                // Coplanar intersection: to generate the edges,
                                // test validity of all possible
                                // pairs of vertices.
                                for(index_t i1=0; i1< I.size(); ++i1) {
                                    for(index_t i2=0; i2<i1; ++i2) {
                                        IsectInfo II = {
                                            f1, f2,
                                            I[i1].first, I[i1].second,
                                            I[i2].first, I[i2].second
                                        };

                                        // Valid edges are the ones where both
                                        // extremities are on the same edge
                                        // of f1 or on the same edge of f2
                                        // (note: it is a *combinatorial*
                                        // convex hull).
                                    
                                        TriangleRegion AB1=regions_convex_hull(
                                            II.A_rgn_f1,II.B_rgn_f1
                                        );
                                        
                                        TriangleRegion AB2=regions_convex_hull(
                                            II.A_rgn_f2, II.B_rgn_f2
                                        );
                                        
                                        if(
                                            region_dim(AB1) == 1 ||
                                            region_dim(AB2) == 1
                                        ) {

                                            intersections.push_back(II);
                                            II.flip();
                                            intersections.push_back(II);
                                        } 
                                    }
                                }
                            } else {
                                // Intersection is either a segment
                                // or a vertex of f2.
                                TriangleRegion A_rgn_f1 = I[0].first;
                                TriangleRegion A_rgn_f2 = I[0].second;
                                
                                TriangleRegion B_rgn_f1 = A_rgn_f1;
                                TriangleRegion B_rgn_f2 = A_rgn_f2;
                                
                                if(I.size() == 2) {
                                    B_rgn_f1 = I[1].first;
                                    B_rgn_f2 = I[1].second;
                                }
                                
                                IsectInfo II = {
                                    f1, f2,
                                    A_rgn_f1, A_rgn_f2,
                                    B_rgn_f1, B_rgn_f2
                                };
                                intersections.push_back(II);
                                II.flip();
                                intersections.push_back(II);
                            }
                            Process::release_spinlock(lock);
                        }
                    }
                }
            );
        }

        // We need to copy the initial mesh, because MeshInTriangle needs
        // to access it in parallel threads, and without a copy, the internal
        // arrays of the mesh can be modified whenever there is a
        // reallocation. Without copying, we would need to insert many
        // locks (each time the mesh is accessed). 
        mesh_copy_.copy(mesh_);
        
        // Step 3: Remesh intersected triangles
        // ------------------------------------
        {
            Stopwatch W("Remesh isect");

            // Sort intersections by f1, so that all intersections between f1
            // and another facet appear as a contiguous sequence.
            std::sort(
                intersections.begin(), intersections.end(),
                [](const IsectInfo& a, const IsectInfo& b) -> bool {
                    return (a.f1 < b.f1) ? true  :
                        (a.f1 > b.f1) ? false :
                        (a.f2 < b.f2) ;
                }
            );

            // Now iterate on all intersections, and identify
            // the [b,e[ intervals that correspond to the same f1 facet.
            // Get starting indices of intersections in same facet.
            vector<index_t> start;
            {
                index_t b=0;            
                while(b < intersections.size()) {
                    start.push_back(b);
                    index_t e = b;
                    while(
                        e < intersections.size() &&
                        intersections[e].f1 == intersections[b].f1
                    ) {
                        ++e;
                    }
                    b = e;
                }
                start.push_back(intersections.size());
            }

            // Display intersection stats
            {
                index_t nb_intersections = intersections.size()/2;
                index_t nb_intersected_triangles = (start.size()-1)/2;
                index_t max_intersections_in_triangle = 0;
                for(index_t i=0; i+1<start.size(); ++i) {
                    max_intersections_in_triangle = std::max(
                        max_intersections_in_triangle,
                        index_t(start[i+1]-start[i])
                    );
                }
                Logger::out("Intersect") << "Intersections: "
                                         << nb_intersections << std::endl;
                Logger::out("Intersect") << "Intersected triangles: "
                                         << nb_intersected_triangles
                                         << std::endl;
                Logger::out("Intersect") << "Max intersections in triangle: "
                                         << max_intersections_in_triangle
                                         << std::endl;
            }
        
            
#define TRIANGULATE_IN_PARALLEL
            
            #ifdef TRIANGULATE_IN_PARALLEL
               parallel_for_slice(
                   0,start.size()-1, [&](index_t k1, index_t k2) {
            #else
               index_t k1 = 0;
               index_t k2 = start.size()-1;
            #endif                   
            
            MeshInTriangle MIT(*this);
            MIT.set_delaunay(delaunay_);
            MIT.set_approx_incircle(approx_incircle_);

            for(index_t k=k1; k<k2; ++k) {
                index_t b = start[k];
                index_t e = start[k+1];
#ifndef TRIANGULATE_IN_PARALLEL                
                if(verbose_) {
                    std::cerr << "Isects in " << intersections[b].f1
                              << " / " << mesh_copy_.facets.nb()              
                              << "    : " << (e-b)
                              << std::endl;
                }
#endif
                MIT.begin_facet(intersections[b].f1);
                for(index_t i=b; i<e; ++i) {
                    const IsectInfo& II = intersections[i];
                    
                    // Each IsectIfo is either an individual vertex
                    // or a segment with two vertices.
                    // Each vertex is represented combinatorially.
                    // The MeshInTriangle knows how to compute the
                    // geometry from the combinatorial information.
                    
                    if(II.is_point()) {
                        MIT.add_vertex(
                            II.f2,
                            II.A_rgn_f1, II.A_rgn_f2
                        );
                    } else {
                        MIT.add_edge(
                            II.f2, 
                            II.A_rgn_f1, II.A_rgn_f2,
                            II.B_rgn_f1, II.B_rgn_f2
                        );
                    }
                }
                MIT.end_facet();
            }
        #ifdef TRIANGULATE_IN_PARALLEL
           });
        #endif
        }
        
        // Step 4: Epilogue
        // ----------------
        
        vector<index_t> has_intersections(mesh_.facets.nb(), 0);
        for(const IsectInfo& II: intersections) {
            has_intersections[II.f1] = 1;
            has_intersections[II.f2] = 1;
        }
        
        mesh_.facets.delete_elements(has_intersections);
        // mesh_remove_bad_facets_no_check(mesh_); // TODO: needed here ?

        if(use_radial_sort_) {
            build_Weiler_model();
        }
        
        PCK::set_SOS_mode(SOS_bkp);

        // Scale-back everything
        {
            double* p = mesh_.vertices.point_ptr(0);
            index_t N = mesh_.vertices.nb() *
                        mesh_.vertices.dimension();
            for(index_t i=0; i<N; ++i) {
                p[i] *= INV_SCALING;
            }
        }

        if(normalize_) {
            for(index_t v: mesh_.vertices) {
                double* p = mesh_.vertices.point_ptr(v);
                for(coord_index_t c=0; c<3; ++c) {
                    p[c] = normalize_radius_*p[c] + normalize_center_[c];
                }
            }
        }
    }
    
    
    vec3HE MeshSurfaceIntersection::exact_vertex(index_t v) const {
        geo_debug_assert(v < mesh_.vertices.nb());
        const vec3HE* p = vertex_to_exact_point_[v];
        if(p != nullptr) {
            return *p;
        }
        const double* xyz = mesh_.vertices.point_ptr(v);
        return vec3HE(xyz[0], xyz[1], xyz[2], 1.0);
    }

    /**
     * \brief Computes a vector of arbitrary length with its direction given
     *   by two points 
     * \param[in] p1 , p2 the two points in homogeneous coordinates
     * \return a vector in cartesian coordinates with the same direction 
     *  and orientation as \p p2 - \p p1
     */
    vec3E MeshSurfaceIntersection::RadialSort::exact_direction(
        const vec3HE& p1, const vec3HE& p2
    ) {
        vec3E U;
        if(p1.w == p2.w) {
            U.x = p2.x - p1.x;
            U.y = p2.y - p1.y;
            U.z = p2.z - p1.z;
            if(p1.w.sign() < 0) {
                U.x.rep().negate();
                U.y.rep().negate();
                U.z.rep().negate();
            }
        } else {
            U.x = det2x2(p2.x, p1.x, p2.w, p1.w);
            U.y = det2x2(p2.y, p1.y, p2.w, p1.w);
            U.z = det2x2(p2.z, p1.z, p2.w, p1.w);
            if(p1.w.sign()*p2.w.sign() < 0) {
                U.x.rep().negate();
                U.y.rep().negate();
                U.z.rep().negate();
            }
        }
        U.x.optimize();
        U.y.optimize();
        U.z.optimize();        
        return U;
    }
    
    index_t MeshSurfaceIntersection::find_or_create_exact_vertex(
        const vec3HE& p
    ) {
        std::map<vec3HE,index_t,vec3HELexicoCompare>::iterator it;
        bool inserted;
        std::tie(it, inserted) = exact_point_to_vertex_.insert(
            std::make_pair(p,index_t(-1))
        );
        if(!inserted) {
            return it->second;
        }
        double w = p.w.estimate();
        vec3 p_inexact(
            p.x.estimate()/w,
            p.y.estimate()/w,
            p.z.estimate()/w
        );
        index_t v = mesh_.vertices.create_vertex(p_inexact.data());
        it->second = v;
        vertex_to_exact_point_[v] = &(it->first);
        return v;
    }

    /************************ Radial sort ***********************************/
    
    void MeshSurfaceIntersection::RadialSort::init(index_t h_ref) {
        degenerate_ = false;
        h_ref_ = h_ref;
        // Clear h_refNorient cache
        refNorient_cache_.resize(0);
        // Precompute U_ref_, V_ref_, N_ref_ for h_refNorient
        U_ref_ = exact_direction(
            mesh_.exact_vertex(mesh_.halfedge_vertex(h_ref,0)),
            mesh_.exact_vertex(mesh_.halfedge_vertex(h_ref,1))
        );
        V_ref_ = exact_direction(
            mesh_.exact_vertex(mesh_.halfedge_vertex(h_ref,0)),
            mesh_.exact_vertex(mesh_.halfedge_vertex(h_ref,2))
        );
        N_ref_ = cross(U_ref_, V_ref_);
        N_ref_.x.optimize();
        N_ref_.y.optimize();
        N_ref_.z.optimize();
    }

    bool MeshSurfaceIntersection::RadialSort::operator()(
        index_t h1, index_t h2
    ) const {
        // h_ref defines the origin of angles
        // the dot product with h_ref's triangle normal
        //  defines a positive side that comes first
        //  (0 to 180 degrees), then a negative side
        //  (180 to 360 degrees excluded)

        // Test on which side h1 and h2 are
        Sign o_ref1 = h_orient(h_ref_, h1);
        Sign o_ref2 = h_orient(h_ref_, h2);

        // If they are on different sides, then
        // h1 < h2 if h1 is on the positive side
        // (that comes first)
        if(o_ref1 * o_ref2 < 0) {
            return o_ref1 > 0;
        }

        // If one of h1's triangle, h2's triangle is
        // coplanar with h_ref
        if(o_ref1 * o_ref2 == 0) {

            // Discriminate 0 or 180 degrees according
            // to dot product with h_ref's triangle normal,
            // that is positive from 0 to 90 degrees and
            //                  from 270 to 360 degrees
            Sign oN_ref1 = h_refNorient(h1);
            Sign oN_ref2 = h_refNorient(h2);
            
            // Both triangles are coplanar with h_ref
            if(o_ref1 == 0 && o_ref2 == 0) {
                
                // Triangles are coplanar with the same normal
                if(oN_ref1 == oN_ref2) {
                    std::cerr << "ZZ " << std::flush;
                    degenerate_ = true;
                    return false;
                }

                // if h1's normal matches href's normal,
                // then h1 comes first, else it is h2
                // that comes first
                return oN_ref1 > 0;
            }

            // Knowing that exactly one of h1,h2 is coplanar with h_ref,
            // If h1 is coplanar with h_ref and has same normal as h_ref,
            // then h1 comes first
            if(o_ref1 == 0 && oN_ref1 > 0) {
                return true;
            }

            // Knowing that exactly one of h1,h2 is coplanar with h_ref,
            // If h2 is coplanar with h_ref and has same normal as h_ref,
            // then h2 comes first
            if(o_ref2 == 0 && oN_ref2 > 0) {
                return false;
            }
        }

        Sign o_12 = h_orient(h1,h2);
        if(o_12 == 0) {
            std::cerr << "** " << std::flush;
            degenerate_ = true;
        }
        return o_12 > 0;
    }

    Sign MeshSurfaceIntersection::RadialSort::h_orient(
        index_t h1, index_t h2
    ) const {
        if(h1 == h2) {
            return ZERO;
        }
        index_t v0 = mesh_.halfedge_vertex(h1,0);
        index_t v1 = mesh_.halfedge_vertex(h1,1);
        index_t w1 = mesh_.halfedge_vertex(h1,2);
        geo_assert(mesh_.halfedge_vertex(h2,0) == v0);
        geo_assert(mesh_.halfedge_vertex(h2,1) == v1);            
        index_t w2 = mesh_.halfedge_vertex(h2,2);

        if(approx_predicates_) {
            vec3 p0(mesh_.target_mesh().vertices.point_ptr(v0));
            vec3 p1(mesh_.target_mesh().vertices.point_ptr(v1));
            vec3 q1(mesh_.target_mesh().vertices.point_ptr(w1));
            vec3 q2(mesh_.target_mesh().vertices.point_ptr(w2));
            return Sign(-PCK::orient_3d(p0,p1,q1,q2));
            // Too stupid, it seems that orient_3d for vec3HE is inverted
            // as compared to vec3 (to be double-checked)
        } 
        const vec3HE& p0 = mesh_.exact_vertex(v0);
        const vec3HE& p1 = mesh_.exact_vertex(v1);
        const vec3HE& q1 = mesh_.exact_vertex(w1);
        const vec3HE& q2 = mesh_.exact_vertex(w2);
        return PCK::orient_3d(p0,p1,q1,q2);
    }
    
    Sign MeshSurfaceIntersection::RadialSort::h_refNorient(index_t h2) const {
        if(h2 == h_ref_) {
            return POSITIVE;
        }
        for(const auto& c: refNorient_cache_) {
            if(c.first == h2) {
                return c.second;
            }
        }

        if(approx_predicates_) {
            index_t v0 = mesh_.halfedge_vertex(h_ref_,0);
            index_t v1 = mesh_.halfedge_vertex(h_ref_,1);
            index_t w1 = mesh_.halfedge_vertex(h_ref_,2);
            index_t w2 = mesh_.halfedge_vertex(h2,2);
            vec3 p0(mesh_.target_mesh().vertices.point_ptr(v0));
            vec3 p1(mesh_.target_mesh().vertices.point_ptr(v1));
            vec3 q1(mesh_.target_mesh().vertices.point_ptr(w1));
            vec3 q2(mesh_.target_mesh().vertices.point_ptr(w2));
            vec3 N1 = cross(p1-p0,q1-p0);
            vec3 N2 = cross(p1-p0,q2-p0);
            return geo_sgn(dot(N1,N2));
        }
        
        vec3E V2 = exact_direction(
            mesh_.exact_vertex(mesh_.halfedge_vertex(h2,0)),
            mesh_.exact_vertex(mesh_.halfedge_vertex(h2,2))
        );
        vec3E N2 = cross(U_ref_, V2);
        N2.x.optimize();
        N2.y.optimize();
        N2.z.optimize();
        Sign result = dot(N_ref_,N2).sign();
        refNorient_cache_.push_back(std::make_pair(h2,result));
        return result;
    }

    bool MeshSurfaceIntersection::radial_sort(
        vector<index_t>::iterator b, vector<index_t>::iterator e
    ) {
        if(index_t(e-b) <= 2) {
            return true;
        }
        radial_sort_.init(*b);
        std::sort(
            b, e,
            [&](index_t h1, index_t h2)->bool {
                return radial_sort_(h1,h2);
            }
        );
        return !radial_sort_.degenerate();
    }

    void MeshSurfaceIntersection::build_Weiler_model() {

        // There can be duplicated facets coming from
        // tesselated co-planar facets.
        // Note: this updates operand_bit attribute
        mesh_remove_bad_facets_no_check(mesh_);

        if(!facet_corner_alpha3_.is_bound()) {
            facet_corner_alpha3_.bind(
                mesh_.facet_corners.attributes(),"alpha3"
            );
        }

        if(!facet_corner_degenerate_.is_bound()) {
            facet_corner_degenerate_.bind(
                mesh_.facet_corners.attributes(),"degenerate"
            );
        }

        // Step 1: duplicate all surfaces and create alpha3 links
        {
            index_t nf = mesh_.facets.nb();
            mesh_.facets.create_triangles(nf);
            for(index_t f1=0; f1<nf; ++f1) {
                index_t f2 = f1+nf;
                mesh_.facets.set_vertex(f2,0,mesh_.facets.vertex(f1,2));
                mesh_.facets.set_vertex(f2,1,mesh_.facets.vertex(f1,1));
                mesh_.facets.set_vertex(f2,2,mesh_.facets.vertex(f1,0));
                sew3(3*f1,  3*f2+1);
                sew3(3*f1+1,3*f2  );
                sew3(3*f1+2,3*f2+2);
            }
        }

        {
            for(index_t c: mesh_.facet_corners) {
                mesh_.facet_corners.set_adjacent_facet(c,index_t(-1));
            }
        }
        
        // A sorted vector of all halfedges
        vector<index_t> H(mesh_.facet_corners.nb());
        for(index_t h: mesh_.facet_corners) {
            H[h] = h;
        }
        
        for(index_t h: mesh_.facet_corners) {
            mesh_.facet_corners.set_adjacent_facet(h, index_t(-1));
        }
        
        // Step 2: find adjacent halfedges by lexicographic sort 
        {
            
            std::sort(
                H.begin(), H.end(),
                [&](index_t h1, index_t h2)->bool {
                    geo_debug_assert(h1 < mesh_.facet_corners.nb());
                    geo_debug_assert(h2 < mesh_.facet_corners.nb());
                    index_t v10 = halfedge_vertex(h1,0);
                    index_t v20 = halfedge_vertex(h2,0);
                    if(v10 < v20) {
                        return true;
                    }
                    if(v10 > v20) {
                        return false;
                    }
                    return (halfedge_vertex(h1,1) < halfedge_vertex(h2,1));
                }
            );
        }
        
        // Step 3: find sequences of adjacent halfedges in the sorted array
        vector<index_t> start;
        {
            for(index_t b=0, e=0; b<H.size(); b=e) {
                start.push_back(b);
                index_t v0 = halfedge_vertex(H[b],0);
                index_t v1 = halfedge_vertex(H[b],1);
                e = b+1;
                while(
                    e < H.size() &&
                    halfedge_vertex(H[e],0) == v0 &&
                    halfedge_vertex(H[e],1) == v1
                ) {
                    ++e;
                }
            }
            start.push_back(H.size());
        }
        
        // Step 4: radial sort
        {
            Logger::out("Radial sort") << "Nb radial edges:"
                                       << start.size()-1 << std::endl;
            Stopwatch W("Radial sort");
            for(index_t k=0; k<start.size()-1; ++k) {
                // std::cerr << k << "/" << start.size()-1 << std::endl;
                vector<index_t>::iterator b =
                    H.begin()+std::ptrdiff_t(start[k]);
                vector<index_t>::iterator e =
                    H.begin()+std::ptrdiff_t(start[k+1]);
                
                bool OK = radial_sort(b,e); // Can return !OK when it
                                            // cannot sort (coplanar facets)
                for(auto it=b; it!=e; ++it) {
                    facet_corner_degenerate_[*it] = !OK;
                }

/*                
                if(!OK) {
                    std::cerr << std::endl;
                    for(auto it1=b; it1!=e; ++it1) {
                        for(auto it2=b; it2!=e; ++it2) {
                            if(it1 != it2) {
                                std::cerr << (it1-b) << " " << (it2-b) << std::endl;
                                radial_sort_.test(*it1, *it2);
                            }
                        }
                    }
                    save_radial("radial",b,e);
                    exit(-1);
                }
*/
            }
        }
        
        // Step 5: create alpha2 links
        {
            for(index_t k=0; k<start.size()-1; ++k) {
                index_t b = start[k];
                index_t e = start[k+1];
                for(index_t i=b; i<e; ++i) {
                    index_t h1 = H[i];
                    index_t h2 = (i+1 == e) ? H[b] : H[i+1];
                    
                    // Do not create alpha2 links if there were coplanar facets
                    if(
                        !facet_corner_degenerate_[h1] &&
                        !facet_corner_degenerate_[h2]
                    ) {
                        sew2(h1,alpha3(h2));
                    }
                }
            }
        }
        
        // Step 6: identify regions
        {
            Attribute<index_t> chart(mesh_.facets.attributes(),"chart");
            for(index_t f: mesh_.facets) {
                chart[f] = index_t(-1);
            }
            index_t cur_chart = 0;
            for(index_t f: mesh_.facets) {
                if(chart[f] == index_t(-1)) {
                    std::stack<index_t> S;
                    chart[f] = cur_chart;
                    S.push(f);
                    while(!S.empty()) {
                        index_t f1 = S.top();
                        S.pop();
                        for(index_t le=0; le<3; ++le) {
                            index_t f2 = mesh_.facets.adjacent(f1,le);
                            if(
                                f2 != index_t(-1) &&
                                chart[f2] == index_t(-1)) {
                                chart[f2] = cur_chart;
                                S.push(f2);
                            }
                        }
                    }
                    ++cur_chart;
                }
            }
            Logger::out("Weiler") << "Found " << cur_chart
                                  << " regions" << std::endl;
        }
    }
    
    void MeshSurfaceIntersection::mark_external_shell(
        vector<index_t>& on_external_shell
    ) {
        auto leftmost = [&](index_t f1, index_t f2) -> index_t {
            if(f1 == index_t(-1)) {
                return f2;
            }
            double xmin1 =  Numeric::max_float64();
            double xmax1 = -Numeric::max_float64();
            double xmin2 =  Numeric::max_float64();
            double xmax2 = -Numeric::max_float64();
            for(index_t lv=0; lv<mesh_.facets.nb_vertices(f1); ++lv) {
                index_t v = mesh_.facets.vertex(f1,lv);
                double x = mesh_.vertices.point_ptr(v)[0];
                xmin1 = std::min(xmin1, x);
                xmax1 = std::max(xmax1, x);                
            }
            for(index_t lv=0; lv<mesh_.facets.nb_vertices(f2); ++lv) {
                index_t v = mesh_.facets.vertex(f2,lv);
                double x = mesh_.vertices.point_ptr(v)[0];
                xmin2 = std::min(xmin2, x);
                xmax2 = std::max(xmax2, x);                
            }
            if(xmin1 < xmin2) {
                return f1;
            }
            if(xmin1 > xmin2) {
                return f2;
            }
            return (xmax1 < xmax2) ? f1 : f2;
        };
        
        on_external_shell.assign(mesh_.facets.nb(), 0);
        index_t leftmost_f = index_t(-1);
        for(index_t f: mesh_.facets) {
            leftmost_f = leftmost(leftmost_f, f);
        }
        vec3 N = Geom::mesh_facet_normal(mesh_, leftmost_f);
        if(N.x < 0) {
            leftmost_f = alpha3_facet(leftmost_f);
        }
        if(false) {
            std::ofstream out("leftmost.obj");
            index_t v1 = mesh_.facets.vertex(leftmost_f,0);
            index_t v2 = mesh_.facets.vertex(leftmost_f,1);
            index_t v3 = mesh_.facets.vertex(leftmost_f,2);
            out << "v " << vec3(mesh_.vertices.point_ptr(v1)) << std::endl;
            out << "v " << vec3(mesh_.vertices.point_ptr(v2)) << std::endl;
            out << "v " << vec3(mesh_.vertices.point_ptr(v3)) << std::endl;
            out << "f 1 2 3" << std::endl;
        }
        
        std::stack<index_t> S;
        on_external_shell[leftmost_f] = 1;
        S.push(leftmost_f);
        while(!S.empty()) {
            index_t f = S.top();
            S.pop();
            for(index_t le=0; le<mesh_.facets.nb_vertices(f); ++le) {
                index_t neigh_f = mesh_.facets.adjacent(f,le);
                if(neigh_f != index_t(-1) && !on_external_shell[neigh_f]) {
                    on_external_shell[neigh_f] = 1;
                    S.push(neigh_f);
                }
            }
        }
    }

    void MeshSurfaceIntersection::save_exact(const std::string& filename) {
        std::ofstream out(filename);
        
    }
    
    /***********************************************************************/
}

namespace {
    using namespace GEO;

    /**
     * \brief A simple parser for boolean expressions
     * \details 
     *  - Variables: A..Z or x0..x31 
     *  - and:        '&' or '*'
     *  - or:         '|' or '+'
     *  - xor:        '^'
     *  - difference: '-'
     */
    class BooleanExprParser {
    public:
        BooleanExprParser(
            const std::string& expr
        ) : expr_(expr) {
        }

        bool eval(index_t x) {
            x_   = x;
            ptr_ = expr_.begin();
            return parse_or();
        }

    protected:

        bool parse_or() {
            bool left = parse_and();
            while(
                cur_char() == '|' ||
                cur_char() == '^' ||
                cur_char() == '+' ||
                cur_char() == '-'
            ) {
                char op = cur_char();
                next_char();
                bool right = parse_and();
                left = (op == '-') ? (left && !right) :
                       (op == '^') ? (left ^   right) :
                                     (left ||  right) ;
            }
            return left;
        }

        bool parse_and() {
            bool left = parse_factor();
            while(cur_char() == '&' || cur_char() == '*') {
                next_char();
                bool right = parse_factor();
                left = left && right;
            }
            return left;
        }

        bool parse_factor() {
            if(cur_char() == '!' || cur_char() == '~' || cur_char() == '-') {
                next_char();
                return !parse_factor();
            }
            if(cur_char() == '(') {
                next_char();
                bool result = parse_or();
                if(cur_char() != ')') {
                    throw std::logic_error(
                        std::string("Unmatched parenthesis: ")+cur_char()
                    );
                }
                next_char();
                return result;
            }
            if((cur_char() >= 'A' && cur_char() <= 'Z') || cur_char() == 'x') {
                return parse_variable();
            }
            throw std::logic_error("Syntax error");
        }

        bool parse_variable() {
            int bit = 0;
            if(cur_char() >= 'A' && cur_char() <= 'Z') {
                bit = int(cur_char()) - int('A');
                next_char();
            } else {
                if(cur_char() != 'x') {
                    throw std::logic_error("Syntax error in variable");
                }
                next_char();
                while(cur_char() >= '0' && cur_char() <= '9') {
                    bit = bit * 10 + (int(cur_char()) - '0');
                    next_char();
                }
            }
            if(bit > 31) {
                throw std::logic_error("Bit larger than 31");
            }
            return ((x_ & (index_t(1u) << bit)) != 0);
        }

        char cur_char() const {
            return *ptr_;
        }
        
        void next_char() {
            if(ptr_ == expr_.end()) {
                throw std::logic_error("Unexpected end of string");
            }
            ptr_++;
        }
        
    private:
        std::string expr_;
        std::string::iterator ptr_;
        index_t x_;
    };

    /**
     * \brief Gets the number of bits set in
     *  a 32 bits integer
     * \param[in] x the integer
     * \return the number of "ones" in the 
     *  binary form of x
     */
    inline index_t nb_bits_set(index_t x) {
        index_t result = 0;
        for(index_t i=0; i<32; ++i) {
            result += (x&1);
            x = x >> 1;
        }
        return result;
    }
}

namespace {
    using namespace GEO;

    // TODO: something less hacky...
    bool facet_on_border(MeshFacetsAABB& AABB, index_t f) {
        vec3 g = Geom::mesh_facet_center(*(AABB.mesh()),f);
        for(index_t i=0; i<1000; ++i) {
            vec3 D(
                Numeric::random_float64(),
                Numeric::random_float64(),
                Numeric::random_float64()
            );
            
            if(!AABB.ray_intersection(
                   Ray(g,D), Numeric::max_float64(), f
            )) {
                return true;
            }
            if(!AABB.ray_intersection(
                   Ray(g,-D), Numeric::max_float64(), f
            )) {
                return true;
            }
        }
        return false;
    }
    
    void mesh_classify_union(
        Mesh& M, 
        const std::string& attribute,
        bool reorder
    ) {
        MeshFacetsAABB AABB(M,reorder);
        index_t nb_charts = get_surface_connected_components(M);        
        Attribute<index_t> chart(M.facets.attributes(), "chart");
        Attribute<bool> selection;
        vector<index_t> delete_f;
        if(attribute != "") {
            selection.bind(M.facets.attributes(), attribute);            
        } else {
            delete_f.assign(M.facets.nb(), 0);
        }

        vector<index_t> chart_facet(nb_charts, index_t(-1));
        for(index_t f: M.facets) {
            index_t c = chart[f];
            if(chart_facet[c] == index_t(-1)) {
                bool f_is_selected = facet_on_border(AABB,f);
                chart_facet[c] = f;
                if(selection.is_bound()) {
                    selection[f] = f_is_selected;
                } else {
                    delete_f[f] = !f_is_selected;
                }
            } else {
                if(selection.is_bound()) {
                    selection[f] = selection[chart_facet[c]];
                } else {
                    delete_f[f] = delete_f[chart_facet[c]];
                }
            }
        } 
        if(!selection.is_bound()) {
            M.facets.delete_elements(delete_f);
        }
        mesh_repair(
            M,
            GEO::MeshRepairMode(
                GEO::MESH_REPAIR_COLOCATE | GEO::MESH_REPAIR_DUP_F
            ),
            0.0
        );
    }
    
}

namespace GEO {
    
    void mesh_classify_intersections(
        Mesh& M, std::function<bool(index_t)> eqn,
        const std::string& attribute,
        bool reorder
    ) {
        MeshFacetsAABB AABB(M,reorder);
        index_t nb_charts = get_surface_connected_components(M);
        Attribute<index_t> chart(M.facets.attributes(), "chart");
        Attribute<index_t> operand_bit(M.facets.attributes(), "operand_bit");
        Attribute<bool> selection;
        vector<index_t> delete_f;
        if(attribute != "") {
            selection.bind(M.facets.attributes(), attribute);            
        } else {
            delete_f.assign(M.facets.nb(), 0);
        }

        vector<index_t> chart_facet(nb_charts, index_t(-1));
        try {
            for(index_t f: M.facets) {
                index_t c = chart[f];
                if(chart_facet[c] == index_t(-1)) {
                    bool f_is_selected = false;
                    chart_facet[c] = f;
                    vec3 g = Geom::mesh_facet_center(M,f);
                    // Picking f's normal is not a good idea,
                    // because for industrial parts it will
                    // encounter many degenerate ray/triangle
                    // intersections.
                    // TODO: we need to detect them and launch
                    // another ray whenever the ray intersects
                    // the surface on a vertex or on an edge.
                    vec3 D(
                        Numeric::random_float64(),
                        Numeric::random_float64(),
                        Numeric::random_float64()
                    );
                    index_t parity = 0;
                    AABB.ray_all_intersections(
                        Ray(g,D),
                        [&](const MeshFacetsAABB::Intersection & I) {
                            if(I.f != f) {
                                parity = parity ^ operand_bit[I.f];
                            }
                        }
                    );
                    if(nb_bits_set(operand_bit[f]) == 1) {
                        // Facet f is on the boundary of the result if
                        // crossing f changes the result of eqn,
                        // in other words, if eqn gives a different
                        // result with and without f's object bit set
                        f_is_selected =
                            eqn(parity |  operand_bit[f]) !=
                            eqn(parity & ~operand_bit[f]) ;
                    } else {
                        // Now if f is on several objects (that is, has
                        // several bit sets), then we determine whether
                        // it is on the boundary of the result by raytracing
                        // in two different directions, and seeing if eqn
                        // gives a different result. 
                        index_t parity2 = 0;
                        AABB.ray_all_intersections(
                            Ray(g,-D),
                            [&](const MeshFacetsAABB::Intersection & I) {
                                if(I.f != f) {
                                    parity2 = parity2 ^ operand_bit[I.f];
                                }
                            }
                        );
                        f_is_selected = (eqn(parity) != eqn(parity2));
                    }
                    if(selection.is_bound()) {
                        selection[f] = f_is_selected;
                    } else {
                        delete_f[f] = !f_is_selected;
                    }
                } else {
                    if(selection.is_bound()) {
                        selection[f] = selection[chart_facet[c]];
                    } else {
                        delete_f[f] = delete_f[chart_facet[c]];
                    }
                }
            }
        } catch(const std::logic_error& e) {
            Logger::err("Classify") << "Error while parsing expression:"
                                    << e.what()
                                    << std::endl;
            return;
        }
        if(!selection.is_bound()) {
            M.facets.delete_elements(delete_f);
        }
        mesh_repair(
            M,
            MeshRepairMode(
                MESH_REPAIR_COLOCATE | MESH_REPAIR_DUP_F
            ),
            0.0
        );
    }

    void mesh_classify_intersections(
        Mesh& M, const std::string& expr,
        const std::string& attribute, bool reorder
    ) {
        BooleanExprParser eqn(expr);
        index_t operand_all_bits;
        {
            index_t max_operand_bit = 0;
            Attribute<index_t> operand_bit;
            operand_bit.bind_if_is_defined(M.facets.attributes(),"operand_bit");
            if(!operand_bit.is_bound()) {
                Logger::err("Classify")
                    << "operand_bit: no such facet attribute"
                    << std::endl;
                return;
            }
            for(index_t f: M.facets) {
                max_operand_bit = std::max(max_operand_bit, operand_bit[f]);
            }
            operand_all_bits = (max_operand_bit << 1)-1;
        }
        
        if(expr == "union") {
            mesh_classify_union(M, attribute, reorder);
            return;
        }
        
        try {
            mesh_classify_intersections(
                M,
                [&](index_t x)->bool {
                    return
                        (expr == "union")        ? (x != 0)                :
                        (expr == "intersection") ? (x == operand_all_bits) : 
                        eqn.eval(x);
                },
                attribute,
                reorder
            );
        } catch(const std::logic_error& e) {
            Logger::err("Classify") << "Error while parsing expression:"
                                    << e.what()
                                    << std::endl;
            return;
        }
    }    
}

/************************************************************************************/

namespace {
    using namespace GEO;
    
    void copy_operand(Mesh& result, const Mesh& operand, index_t operand_id) {
        Attribute<index_t> operand_bit(result.facets.attributes(), "operand_bit");
        index_t v_ofs = result.vertices.create_vertices(operand.vertices.nb());
        for(index_t v: operand.vertices) {
            const double* p_src = operand.vertices.point_ptr(v);            
            double* p_dst = result.vertices.point_ptr(v + v_ofs);
            p_dst[0] = p_src[0];
            p_dst[1] = p_src[1];
            p_dst[2] = p_src[2];            
        }
        for(index_t f1: operand.facets) {
            index_t N = operand.facets.nb_vertices(f1);
            index_t f2 = result.facets.create_polygon(N);
            for(index_t lv=0; lv<N; ++lv) {
                result.facets.set_vertex(
                    f2,lv,operand.facets.vertex(f1,lv) + v_ofs
                );
            }
            operand_bit[f2] = index_t(1) << operand_id;
        }
    }

    
}

namespace GEO {
    
    void mesh_boolean_operation(
        Mesh& result, Mesh& A, Mesh& B, const std::string& operation
    ) {
        if(&result == &A) {
            Attribute<index_t> operand_bit(result.facets.attributes(), "operand_bit");
            for(index_t f: A.facets) {
                operand_bit[f] = index_t(1);
            }
            copy_operand(result,B,1);
        } else if(&result == &B) {
            mesh_boolean_operation(
                result, B, A, (operation=="A-B") ? "B-A" : operation
            );
            return;
        } else {
            result.clear();
            result.vertices.set_dimension(3);
            copy_operand(result,A,0);
            copy_operand(result,B,1);
        }
        MeshSurfaceIntersection I(result);
        I.set_radial_sort(false); // For now classification does not use it
        I.intersect();
        mesh_repair(result); // Merge duplicated facets, reorient, get charts
        mesh_classify_intersections(result, operation, "", false);
        mesh_repair(result); // Final gluing
    }
    
    void mesh_union(Mesh& result, Mesh& A, Mesh& B) {
        mesh_boolean_operation(result, A, B, "A+B");
    }

    void mesh_intersection(Mesh& result, Mesh& A, Mesh& B) {
        mesh_boolean_operation(result, A, B, "A*B");
    }

    void mesh_difference(Mesh& result, Mesh& A, Mesh& B) {
        mesh_boolean_operation(result, A, B, "A-B");
    }
    
    void mesh_remove_intersections(Mesh& M, index_t max_iter) {
        // TODO: same as tet_meshing() (compute union) ?
        for(index_t k=0; k<max_iter; ++k) {
            MeshSurfaceIntersection I(M);
            I.set_radial_sort(false);
            I.intersect();
            mesh_repair(M);            
        }
    }

    bool mesh_facets_have_intersection(Mesh& M, index_t f1, index_t f2) {
        index_t cb1 = M.facets.corners_begin(f1);
        index_t cb2 = M.facets.corners_begin(f2);
        vec3 p0(M.vertices.point_ptr(M.facet_corners.vertex(cb1)));
        vec3 q0(M.vertices.point_ptr(M.facet_corners.vertex(cb2)));
        for(index_t c1 = cb1+1; c1+1<M.facets.corners_end(f1); ++c1) {
            vec3 p1(M.vertices.point_ptr(M.facet_corners.vertex(c1)));
            vec3 p2(M.vertices.point_ptr(M.facet_corners.vertex(c1+1)));
            for(index_t c2 = cb2+1; c2+1<M.facets.corners_end(f2); ++c2) {
                vec3 q1(M.vertices.point_ptr(M.facet_corners.vertex(c2)));
                vec3 q2(M.vertices.point_ptr(M.facet_corners.vertex(c2+1)));
                if(triangles_intersections(p0,p1,p2,q0,q1,q2)) {
                    return true;
                }
            }
        }
        return false;
    }
}


