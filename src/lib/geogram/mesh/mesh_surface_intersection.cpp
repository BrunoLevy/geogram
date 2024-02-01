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
#include <geogram/basic/permutation.h>
#include <geogram/basic/boolean_expression.h>
#include <geogram/basic/debug_stream.h>

#include <sstream>
#include <stack>

#ifdef GEO_COMPILER_CLANG
// I'm using long long 
#pragma GCC diagnostic ignored "-Wc++98-compat-pedantic"
#endif

// If defined, displays status messages and saves files whenever some
// error conditions are met.
// #define MESH_SURFACE_INTERSECTION_DEBUG

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
                    for(index_t le=0; le<M.facets.nb_vertices(g); ++le) {
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

    /**
     * \brief The constant for scaling floating point numbers
     *   in such a way that only the exponent is changed and
     *   set about midrange.
     */
    static constexpr double SCALING = double(1ull << 20);
    
    /**
     * \brief Invert of SCALING
     * \see SCALING
     */
    static constexpr double INV_SCALING = 1.0/SCALING;


    /**
     * \brief Tests whether a segment intersects a triangle
     * \details All points are given with exact homogeneous
     *  coordinates (MeshSurfaceIntersection::ExactPoint)
     * \param[in] P1 , P2 the two extremities of the segment
     * \param[in] q1 , q2 , q3 the three verties of the triangle
     * \param[out] degenerate if set, the segment passes exactly
     *  through one of the vertices, one of the edges or through
     *  the supporting plane of the triangle.
     * \retval true if the segment has an intersection with the
     *  interior of the triangle and is not contained in the
     *  supporting plane of the triangle
     * \retval false otherwise
     */
    template <class POINT> bool segment_triangle_intersection(
        const POINT& P1, const POINT& P2, 
        const POINT& q1,
        const POINT& q2,
        const POINT& q3,
        bool& degenerate
    ) {
        degenerate = false;

        Sign o1 = PCK::orient_3d(P1,q1,q2,q3);
        Sign o2 = PCK::orient_3d(P2,q1,q2,q3);

        // Note: '&&' and not '||' : one of the segment's extremities can be
        // in the plane of the triangle without intersection and without
        // degeneracy
        if(o1 == ZERO && o2 == ZERO) {
            degenerate = true;
            return false;
        }

        if(o1 == o2) {
            return false;
        }
        
        Sign s1 = PCK::orient_3d(P1,P2,q1,q2);
        Sign s2 = PCK::orient_3d(P1,P2,q2,q3);
        Sign s3 = PCK::orient_3d(P1,P2,q3,q1);


        // There is for sure no intersection if two signs
        // differ
        if(s1*s2 < 0 || s2*s3 < 0 || s3*s1 < 0) {
            return false;
        }

        if(s1 == ZERO || s2 == ZERO || s3 == ZERO) {
            degenerate = true;
            return false;
        }

        // Now, if there is an intersection but one of the extremities is
        // in the triangle plane, then it is a degeneracy
        if(o1 == ZERO || o2 == ZERO) {
            degenerate = true;
            return false;
        }
        
        return true;
    }
}


namespace GEO {
    
    MeshSurfaceIntersection::MeshSurfaceIntersection(Mesh& M) :
        lock_(GEOGRAM_SPINLOCK_INIT),
        mesh_(M),
        vertex_to_exact_point_(M.vertices.attributes(), "exact_point"),
        normalize_(false),
        dry_run_(false),
        halfedges_(*this),
        radial_bundles_(*this),
        radial_polylines_(*this)
    {
        for(index_t v: mesh_.vertices) {
            vertex_to_exact_point_[v] = nullptr;
        }
        verbose_ = false;
        fine_verbose_ = false;
        delaunay_ = true;
        detect_intersecting_neighbors_ = true;
        use_radial_sort_ = true;
        monster_threshold_ = index_t(-1);
        // TODO: understand why this breaks co-planarity tests,
        // with exact_nt it should have not changed anything !!
        // Anyway it does not seem to do any good, deactivated
        // for now, kept in the source because it may solve some
        // underflow/overflow cases with expansion_nt.
        rescale_ = false;
        skeleton_ = nullptr;
        interpolate_attributes_ = false;
    }

    MeshSurfaceIntersection::~MeshSurfaceIntersection() {
        vertex_to_exact_point_.destroy();
    }

    void MeshSurfaceIntersection::remove_external_shell() {
        vector<index_t> remove_f;
        mark_external_shell(remove_f);
        mesh_.facets.delete_elements(remove_f);
        mesh_.facets.connect();
    }
    
    void MeshSurfaceIntersection::remove_internal_shells() {
        vector<index_t> remove_f;
        mark_external_shell(remove_f);
        for(index_t& i: remove_f) {
            i= 1-i;
        }
        mesh_.facets.delete_elements(remove_f);
        // mesh_.facets.connect(); // HERE
    }

    void MeshSurfaceIntersection::remove_fins() {
        // WIP
        index_t fins = 0;
        Attribute<index_t> chart(mesh_.facets.attributes(), "chart");
        vector<index_t> remove_f(mesh_.facets.nb(),0);
        for(index_t f1: mesh_.facets) {
            index_t f2 = halfedges_.facet_alpha3(f1);
            if(f2 != NO_INDEX && chart[f1] == chart[f2]) {
                remove_f[f1] = 1;
                ++fins;
            }
        }

        std::cerr << std::endl << ">>>>>>>>>>>" << fins << " fins" << std::endl;

        /*
        for(index_t f1: mesh_.facets) {
            if(remove_f[f1]) {
                continue;
            }
            for(index_t le1=0; le1<3; ++le1) {
                index_t f1n = mesh_.facets.adjacent(f1,le1);
                if(f1n == NO_INDEX || !remove_f[f1n]) {
                    continue;
                }
                index_t f2n = halfedges_.facet_alpha3(f1n);
                index_t v1 = mesh_.facets.vertex(f1,le1);
                index_t v2 = mesh_.facets.vertex(f1,(le1+1)%3);
                index_t le2n = mesh_.facets.find_edge(f2n,v1,v2);
                geo_assert(le2n != NO_INDEX);
                index_t f2 = mesh_.facets.adjacent(f2n,le2n);
                index_t le2 = mesh_.facets.find_edge(f2,v2,v1);
                geo_assert(le2 != NO_INDEX);
                mesh_.facets.set_adjacent(f1,le1,f2);
                mesh_.facets.set_adjacent(f2,le2,f1);
            }
        }
        */
        
        mesh_.facets.delete_elements(remove_f);
    }

    void MeshSurfaceIntersection::intersect_prologue() {
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
        SOS_bkp_ = PCK::get_SOS_mode();
        PCK::set_SOS_mode(PCK::SOS_LEXICO);

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
        if(rescale_) {
            double* p = mesh_.vertices.point_ptr(0);
            index_t N = mesh_.vertices.nb() *
                        mesh_.vertices.dimension();
            for(index_t i=0; i<N; ++i) {
                p[i] *= SCALING;
            }
        }
    }

    void MeshSurfaceIntersection::intersect_get_intersections(
        vector<IsectInfo>& intersections
    ) {
        if(verbose_) {
            Logger::out("Intersect") << "get intersections" << std::endl;
        }
        {
            Stopwatch W("Detect isect",verbose_);

            if(verbose_) {
                Logger::out("Intersect") << "  build AABB" << std::endl;
            }
            MeshFacetsAABB AABB(mesh_,true);
            vector<std::pair<index_t, index_t> > FF;

            // Get candidate pairs of intersecting facets
            if(verbose_) {
                Logger::out("Intersect") << "  get candidate facet pairs"
                                         << std::endl;
            }
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
            if(verbose_) {
                Logger::out("Intersect") << "  compute intersections"
                                         << std::endl;
            }
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

        // Apply a random permutation to the facets, so that
        // each job done in parallel receives statistatically
        // the same amount of hard cases / easy cases (else
        // spatial sorting tends to gather all difficulties
        // in an index range processed by the same thread).
        // TODO: a version of parallel_for() with smarter
        // (dynamic) thread scheduling.
        {
            vector<index_t> reorder(mesh_.facets.nb());
            for(index_t f: mesh_.facets) {
                reorder[f] = f;
            }
            std::random_shuffle(reorder.begin(), reorder.end());
            for(IsectInfo& II: intersections) {
                II.f1 = reorder[II.f1];
                II.f2 = reorder[II.f2];
            }
            Permutation::invert(reorder);
            mesh_.facets.permute_elements(reorder);
        }
    }

    void MeshSurfaceIntersection::intersect_remesh_intersections(
        vector<IsectInfo>& intersections
    ) {
        if(verbose_) {
            Logger::out("Intersect") << "remesh intersections" << std::endl;
        }


        // Keep track of original facet ids: they will be copied to the subfacets
        // whenever a facet is split.
        // NOTE: of course, facet ids are no longer valids once coplanar facets
        // are merged.
        {
            Attribute<index_t> original_facet_id(
                mesh_.facets.attributes(), "original_facet_id"
            );
            for(index_t f: mesh_.facets) {
                original_facet_id[f] = f;
            }
        }
        
        // We need to copy the initial mesh, because MeshInTriangle needs
        // to access it in parallel threads, and without a copy, the internal
        // arrays of the mesh can be modified whenever there is a
        // reallocation. Without copying, we would need to insert many
        // locks (each time the mesh is accessed). 
        mesh_copy_.copy(mesh_);
        
        {
            Stopwatch W("Remesh isect",verbose_);

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
            if(verbose_) {
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

            index_t f_done = 0;
            index_t f_tot = (start.size()-1); 
            
            #ifdef TRIANGULATE_IN_PARALLEL
               parallel_for_slice(
                   0,start.size()-1, [&](index_t k1, index_t k2) {
            #else
               index_t k1 = 0;
               index_t k2 = start.size()-1;
            #endif                   
            
            MeshInTriangle MIT(*this);
            MIT.set_delaunay(delaunay_);
            MIT.set_dry_run(dry_run_);
            
            index_t tid = Thread::current_id();
            
            for(index_t k=k1; k<k2; ++k) {
                index_t b = start[k];
                index_t e = start[k+1];

                if(fine_verbose_) {
                    ++f_done;
                    Logger::out("Isect")
                        << String::format(
                            "[%2d] %5d/%5d    %6d:%3d",
                            int(tid), int(f_done), int(f_tot),
                            int(intersections[b].f1), int(e-b)
                        )
                        << std::endl;
                }

                MIT.begin_facet(intersections[b].f1);
                for(index_t i=b; i<e; ++i) {
                    const IsectInfo& II = intersections[i];
                    
                    // Each IsectInfo is either an individual vertex
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

                if(e-b >= monster_threshold_) {
                    index_t f = intersections[b].f1;
                    MIT.save_constraints(
                        "constraints_"+String::to_string(f)+".geogram"
                    );
                }
    
                // Inserts constraints and creates new vertices in shared mesh
                MIT.commit();

                // For debugging, optionally save "monsters"
                // (that is, triangles that have a huge number
                // of intersections).
                if(e-b >= monster_threshold_) {
                    index_t f = intersections[b].f1;
                    MIT.save("triangulation_"+String::to_string(f)+".geogram");
                    //MIT.save_constraints(
                    //    "constraints_"+String::to_string(f)+".geogram"
                    //);
                    std::ofstream out("facet_"+String::to_string(f)+".obj");
                    for(
                        index_t lv=0; lv<mesh_copy_.facets.nb_vertices(f); ++lv
                    ) {
                        index_t v = mesh_copy_.facets.vertex(f,lv);
                        vec3 p(mesh_copy_.vertices.point_ptr(v));
                        if(rescale_) {
                            p=INV_SCALING*p;
                        }
                        if(normalize_) {
                            p = normalize_radius_*p + normalize_center_;
                        }
                        out << "v " << p << std::endl;
                    }
                    out << "f ";
                    for(
                        index_t lv=0; lv<mesh_copy_.facets.nb_vertices(f); ++lv
                    ) {
                        out << lv+1 << " ";
                    }
                    out << std::endl;
                }

                // Clear it so that it is clean for next triangle.
                MIT.clear();
            }
            if(fine_verbose_) {
                Logger::out("Isect") << String::format("[%2d] done",int(tid))
                                     << std::endl;
            }
        #ifdef TRIANGULATE_IN_PARALLEL
           });
        #endif
        }
    }
    
    void MeshSurfaceIntersection::intersect_epilogue(
        const vector<IsectInfo>& intersections
    ) {

        if(verbose_) {
            Logger::out("Intersect") << "epilogue" << std::endl;
        }
        
        // Vertices coming from intersections may land exactly
        // on an existing vertex (see #111)
        {
            Stopwatch("Colocate newv",verbose_);
            vector<index_t> v2v(mesh_.vertices.nb());
            for(index_t v : mesh_.vertices) {
                v2v[v] = v;
            }
            parallel_for(
                0, mesh_.vertices.nb(),
                [&](index_t v) {
                    // If the vertex is an original vertex, not
                    // coming from an intersection, check whether
                    // it also exists as an intersection
                    if(vertex_to_exact_point_[v] == nullptr) {
                        const double* xyz = mesh_.vertices.point_ptr(v);
                        ExactPoint K(xyz[0], xyz[1], xyz[2], 1.0);
                        auto it = exact_point_to_vertex_.find(K);
                        if(it != exact_point_to_vertex_.end()) {
                            v2v[v] = it->second;
                        }
                    }
                }
            );
            for(index_t c : mesh_.facet_corners) {
                index_t v = v2v[mesh_.facet_corners.vertex(c)];
                mesh_.facet_corners.set_vertex(c, v);
            }
        }

        if(interpolate_attributes_) {
            Attribute<index_t> original_facet_id(
                mesh_.facets.attributes(), "original_facet_id"
            );

            for(index_t c: mesh_.facet_corners) {
                index_t f = c/3;
                index_t f0 = original_facet_id[f];
                // If this is an original facet, there is nothing to do !
                if(f == f0) {
                    continue;
                }

                index_t v = mesh_.facet_corners.vertex(c);
                index_t v1 = mesh_.facets.vertex(f0,0);
                index_t v2 = mesh_.facets.vertex(f0,1);
                index_t v3 = mesh_.facets.vertex(f0,2);

                index_t c1 = mesh_.facets.corner(f0,0);
                index_t c2 = mesh_.facets.corner(f0,1);
                index_t c3 = mesh_.facets.corner(f0,2);

                if(v == v1) {
                    mesh_.facet_corners.attributes().copy_item(c,c1);
                    continue;
                }

                if(v == v2) {
                    mesh_.facet_corners.attributes().copy_item(c,c2);
                    continue;
                }

                if(v == v3) {
                    mesh_.facet_corners.attributes().copy_item(c,c3);
                    continue;
                }
                
                vec3 p1(mesh_.vertices.point_ptr(v1));
                vec3 p2(mesh_.vertices.point_ptr(v2));
                vec3 p3(mesh_.vertices.point_ptr(v3));
                vec3 p(mesh_.vertices.point_ptr(v));

                double a  = Geom::triangle_area(p1,p2,p3);
                double a1 = Geom::triangle_area(p ,p2,p3);
                double a2 = Geom::triangle_area(p1,p ,p3);
                double a3 = Geom::triangle_area(p1,p2,p );
                
                mesh_.facet_corners.attributes().zero_item(c);
                mesh_.facet_corners.attributes().madd_item(c, a1/a, c1);
                mesh_.facet_corners.attributes().madd_item(c, a2/a, c2);
                mesh_.facet_corners.attributes().madd_item(c, a3/a, c3);
            }
        }

        
        // Remove original facets that have intersections.
        {
            vector<index_t> has_intersections(mesh_.facets.nb(), 0);
            for(const IsectInfo& II: intersections) {
                has_intersections[II.f1] = 1;
                has_intersections[II.f2] = 1;
            }
            mesh_.facets.delete_elements(has_intersections);
        }

        // There can be duplicated facets coming from
        // tesselated co-planar facets.
        // Note: this updates operand_bit attribute
        mesh_remove_bad_facets_no_check(mesh_);

#ifdef MESH_SURFACE_INTERSECTION_DEBUG
        std::cerr << "Sanity check: verify that there is no degenerate triangle"
                  << std::endl;
        // Sanity check: do we have facets with their three vertices
        // aligned ? Normally cannot happen since we have eliminated
        // them during intersection, but who knows ?
        // Actually this happens sometimes, and more often when
        // normalize_ is set (and I still do not understand why)
        {
            Attribute<bool> selected(mesh_.facets.attributes(), "selection");
            for(index_t t: mesh_.facets) {
                if(PCK::aligned_3d(
                       exact_vertex(mesh_.facets.vertex(t,0)),
                       exact_vertex(mesh_.facets.vertex(t,1)),
                       exact_vertex(mesh_.facets.vertex(t,2))
                   )) {
                    selected[t] = true;
                    std::cerr << "FACET HAS 3 ALIGNED VERTICES" << std::endl;
                } else {
                    selected[t] = false;
                }
                    
            }
        }
        std::cerr << "There is no degenerate triangle" << std::endl;
#endif

        if(use_radial_sort_) {
            build_Weiler_model();
        }

        PCK::set_SOS_mode(SOS_bkp_);

        // Scale-back everything
        if(rescale_) {
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
    
    void MeshSurfaceIntersection::intersect() {
        if(verbose_) {
            Logger::out("Intersect") << "start." << std::endl;
        }
        intersect_prologue();
        vector<IsectInfo> intersections;
        intersect_get_intersections(intersections);
        intersect_remesh_intersections(intersections);
        intersect_epilogue(intersections);
        if(verbose_) {
            Logger::out("Intersect") << "finished." << std::endl;
        }
    }
    
    MeshSurfaceIntersection::ExactPoint MeshSurfaceIntersection::exact_vertex(
        index_t v
    ) const {
        geo_debug_assert(v < mesh_.vertices.nb());
        const ExactPoint* p = vertex_to_exact_point_[v];
        if(p != nullptr) {
            return *p;
        }
        const double* xyz = mesh_.vertices.point_ptr(v);
        return ExactPoint(xyz[0], xyz[1], xyz[2], 1.0);
    }

    exact::vec3 MeshSurfaceIntersection::RadialSort::exact_direction(
        const ExactPoint& p1, const ExactPoint& p2
    ) {
        exact::vec3 U;
        if(p1.w == p2.w) {
            U.x = p2.x - p1.x;
            U.y = p2.y - p1.y;
            U.z = p2.z - p1.z;
            if(p1.w.sign() < 0) {
                U.x.negate();
                U.y.negate();
                U.z.negate();
            }
        } else {
            U.x = det2x2(p2.x, p1.x, p2.w, p1.w);
            U.y = det2x2(p2.y, p1.y, p2.w, p1.w);
            U.z = det2x2(p2.z, p1.z, p2.w, p1.w);
            if(p1.w.sign()*p2.w.sign() < 0) {
                U.x.negate();
                U.y.negate();
                U.z.negate();
            }
        }
        Numeric::optimize_number_representation(U);
        return U;
    }

    vec3I MeshSurfaceIntersection::RadialSort::exact_direction_I(
        const ExactPoint& p1, const ExactPoint& p2
    ) {
        interval_nt::Rounding rounding;
        interval_nt w1(p1.w);
        interval_nt w2(p2.w);
        vec3I U(
            det2x2(interval_nt(p2.x), w2, interval_nt(p1.x), w1),
            det2x2(interval_nt(p2.y), w2, interval_nt(p1.y), w1),
            det2x2(interval_nt(p2.z), w2, interval_nt(p1.z), w1)
        );
        if(p1.w.sign()*p2.w.sign() < 0) {
            U.x.negate();
            U.y.negate();
            U.z.negate();
        }
        return U;
    }
    
    index_t MeshSurfaceIntersection::find_or_create_exact_vertex(
        const ExactPoint& p
    ) {
        std::map<ExactPoint,index_t,ExactPointCompare>::iterator it;
        bool inserted;
        std::tie(it, inserted) = exact_point_to_vertex_.insert(
            std::make_pair(p,index_t(-1))
        );
        if(!inserted) {
            return it->second;
        }
        vec3 p_inexact = PCK::approximate(p);
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
            mesh_.exact_vertex(mesh_.halfedges_.vertex(h_ref,0)),
            mesh_.exact_vertex(mesh_.halfedges_.vertex(h_ref,1))
        );
        V_ref_ = exact_direction(
            mesh_.exact_vertex(mesh_.halfedges_.vertex(h_ref,0)),
            mesh_.exact_vertex(mesh_.halfedges_.vertex(h_ref,2))
        );
        N_ref_ = cross(U_ref_, V_ref_);
        Numeric::optimize_number_representation(N_ref_);

        // Reference frame in interval arithmetics.
        {
            interval_nt::Rounding rounding;
            
            U_ref_I_.x = interval_nt(U_ref_.x);
            U_ref_I_.y = interval_nt(U_ref_.y);
            U_ref_I_.z = interval_nt(U_ref_.z);
        
            V_ref_I_.x = interval_nt(V_ref_.x);
            V_ref_I_.y = interval_nt(V_ref_.y);
            V_ref_I_.z = interval_nt(V_ref_.z);        

            N_ref_I_.x = interval_nt(N_ref_.x);
            N_ref_I_.y = interval_nt(N_ref_.y);
            N_ref_I_.z = interval_nt(N_ref_.z);
        }
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
        
        index_t v0 = mesh_.halfedges_.vertex(h1,0);
        index_t v1 = mesh_.halfedges_.vertex(h1,1);
        index_t w1 = mesh_.halfedges_.vertex(h1,2);
        geo_assert(mesh_.halfedges_.vertex(h2,0) == v0);
        geo_assert(mesh_.halfedges_.vertex(h2,1) == v1);            
        index_t w2 = mesh_.halfedges_.vertex(h2,2);

        // TODO: double-check that sign corresponds
        // to documentation.
        
        const ExactPoint& p0 = mesh_.exact_vertex(v0);
        const ExactPoint& p1 = mesh_.exact_vertex(v1);
        const ExactPoint& q1 = mesh_.exact_vertex(w1);
        const ExactPoint& q2 = mesh_.exact_vertex(w2);
        return Sign(-PCK::orient_3d(p0,p1,q1,q2));
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

        static PCK::PredicateStats stats("h_refNorient");
        stats.log_invoke();
        
        Sign result = ZERO;
        { 
            interval_nt::Rounding rounding;
            vec3I V2 = exact_direction_I(
                mesh_.exact_vertex(mesh_.halfedges_.vertex(h2,0)),
                mesh_.exact_vertex(mesh_.halfedges_.vertex(h2,2))
            );
            vec3I N2 = cross(U_ref_I_, V2);
            interval_nt d = dot(N_ref_I_, N2);
            interval_nt::Sign2 s = d.sign();
            if(interval_nt::sign_is_non_zero(s)) {
                result = interval_nt::convert_sign(s);
            }
        }

        if(result == ZERO) {
            stats.log_exact();
            exact::vec3 V2 = exact_direction(
                mesh_.exact_vertex(mesh_.halfedges_.vertex(h2,0)),
                mesh_.exact_vertex(mesh_.halfedges_.vertex(h2,2))
            );
            exact::vec3 N2 = cross(U_ref_, V2);
            Numeric::optimize_number_representation(N2);
            result = dot(N_ref_,N2).sign();
        } 
        
        refNorient_cache_.push_back(std::make_pair(h2,result));
        return result;
    }

    void MeshSurfaceIntersection::mark_external_shell(
        vector<index_t>& on_external_shell
    ) {
        // Chart attribute corresponds to volumetric regions
        Attribute<index_t> chart(mesh_.facets.attributes(), "chart");

        // Get nb charts
        index_t nb_charts = 0;
        for(index_t f: mesh_.facets) {
            nb_charts = std::max(nb_charts, chart[f]+1);
        }


        // Get connected components by traversing both alpha2 and alpha3 links,
        // and orient facets coherently
        index_t nb_components = 0;
        vector<index_t> component(mesh_.facets.nb(), index_t(-1));
        {
            for(index_t f:mesh_.facets) {
                if(component[f] == index_t(-1)) {
                    std::stack<index_t> S;
                    component[f] = nb_components;
                    S.push(f);
                    while(!S.empty()) {
                        index_t f1 = S.top();
                        S.pop();

                        {
                            index_t f2 = halfedges_.facet_alpha3(f1);
                            if(component[f2] == index_t(-1)) {
                                component[f2]=component[f1];
                                S.push(f2);
                            }
                        }
                        
                        for(index_t le1=0; le1<3; ++le1) {
                            index_t f2 = mesh_.facets.adjacent(f1,le1);
                            if(f2!=index_t(-1) && component[f2]==index_t(-1)) {
                                #ifdef GEO_DEBUG
                                index_t le2 = mesh_.facets.find_adjacent(f2,f1);
                                geo_debug_assert(
                                    mesh_.facets.vertex(f1,le1) !=
                                    mesh_.facets.vertex(f2,le2)
                                );
                                #endif
                                component[f2] = component[f1];
                                S.push(f2);
                            }
                        }
                    }
                }
                ++nb_components;
            }
        }

        // Compute the volume enclosed by each chart
        
        vector<double> chart_volume(nb_charts,0.0);
        vec3 p0(0.0, 0.0, 0.0);
        for(index_t f: mesh_.facets) {
            index_t v1 = mesh_.facets.vertex(f,0);
            index_t v2 = mesh_.facets.vertex(f,1);
            index_t v3 = mesh_.facets.vertex(f,2);
            vec3 p1(mesh_.vertices.point_ptr(v1));
            vec3 p2(mesh_.vertices.point_ptr(v2));
            vec3 p3(mesh_.vertices.point_ptr(v3));
            chart_volume[chart[f]] += Geom::tetra_signed_volume(p0,p1,p2,p3);
        }
        
        for(index_t c=0; c<chart_volume.size(); ++c) {
            chart_volume[c] = ::fabs(chart_volume[c]);
        }

        // For each component, find the chart that encloses the largest
        // volume (it is the external boundary of the component)
        
        vector<double> max_chart_volume_in_component(nb_components, 0.0);
        vector<index_t> chart_with_max_volume_in_component(
            nb_components, index_t(-1)
        );

        for(index_t f: mesh_.facets) {
            double V = chart_volume[chart[f]];
            if( V >= max_chart_volume_in_component[component[f]]) {
                max_chart_volume_in_component[component[f]] = V;
                chart_with_max_volume_in_component[component[f]] = chart[f];
            }
        }
        
        on_external_shell.resize(mesh_.facets.nb());
        for(index_t f: mesh_.facets) {
            on_external_shell[f] = (
                chart[f] == chart_with_max_volume_in_component[component[f]]
            );
        }
    }

    /*************************************************************************/
    
    void MeshSurfaceIntersection::build_Weiler_model() {

        Attribute<bool> corner_is_on_border(
            mesh_.facet_corners.attributes(), "is_on_border"
        );
        for(index_t c: mesh_.facet_corners) {
            corner_is_on_border[c] = false;
        }
        
        halfedges_.initialize();
        
        // Step 1: Duplicate all surfaces and create alpha3 links
        {
            index_t nf = mesh_.facets.nb();
            mesh_.facets.create_triangles(nf);
            for(index_t f1=0; f1<nf; ++f1) {
                index_t f2 = f1+nf;
                mesh_.facets.set_vertex(f2,0,mesh_.facets.vertex(f1,2));
                mesh_.facets.set_vertex(f2,1,mesh_.facets.vertex(f1,1));
                mesh_.facets.set_vertex(f2,2,mesh_.facets.vertex(f1,0));
                
                // Copy attributes
                mesh_.facets.attributes().copy_item(f2,f1);
                if(interpolate_attributes_) {
                    mesh_.facet_corners.attributes().copy_item(3*f2,  3*f1+2);
                    mesh_.facet_corners.attributes().copy_item(3*f2+1,3*f1+1);
                    mesh_.facet_corners.attributes().copy_item(3*f2+2,3*f1  );
                }

                // Sew halfedges (need to be done *after* copy attributes, 
                // since alpha3 is stored as an .. **attribute** !!!
                halfedges_.sew3(3*f1,  3*f2+1);
                halfedges_.sew3(3*f1+1,3*f2  );
                halfedges_.sew3(3*f1+2,3*f2+2);
            }
        }

        // Step 2: Clear all facet-facet links
        for(index_t c: mesh_.facet_corners) {
            mesh_.facet_corners.set_adjacent_facet(c,NO_INDEX);
        }

        // Step 3: Compute halfedges bundles and radial polylines
        radial_bundles_.initialize();
        radial_polylines_.initialize();

        if(skeleton_ != nullptr) {
            radial_polylines_.get_skeleton(*skeleton_);
        }
        
        // Step 4: Connect manifold edges
        for(index_t bndl: radial_bundles_) {
            if(radial_bundles_.nb_halfedges(bndl) == 1) {
                // If there is only one halfedge in the radial bundle,
                // this is the original surface border, then we
                // create the "hem" by connecting the halfedge on the
                // border to its mate in the duplicated facet
                //
                // Note: if an object has a tiny hole, it has the effect
                // of creating a single shell instead of two shells with
                // opposite orientation, it may be a problem sometimes...
                index_t h = radial_bundles_.halfedge(bndl,0);
                halfedges_.sew2(h,halfedges_.alpha3(h)); 
                corner_is_on_border[h] = true;
                corner_is_on_border[halfedges_.alpha3(h)] = true;
            } else if(radial_bundles_.nb_halfedges(bndl) == 2) {
                // If there are two halfedges in the radial bundle,
                // then it is a standard manifold edge
                index_t h1 = radial_bundles_.halfedge(bndl,0);
                index_t h2 = radial_bundles_.halfedge(bndl,1);
                halfedges_.sew2(h1,halfedges_.alpha3(h2));
            }
        }

        // Step 5: get charts
        // After this step, charts are interconnected triangles with coherent
        // orientation bordered by non-manifold radial edges
        index_t nb_charts = get_surface_connected_components(mesh_, "chart");
        if(verbose_) {
            Logger::out("Intersect")
                << "Found " << nb_charts << " charts" << std::endl;
        }

        // I had before an assertion check that the number of charts is even
        // but I removed it: 
        // the number of charts is not necessarily even, since there
        // can be "fins" (see e.g. "saturn" example that creates 5 charts).
        
        // Step 6: Radial sort
        radial_polylines_.radial_sort();
        
        // Step 7: Create alpha2 links
        for(index_t P: radial_polylines_) {
            for(index_t bndl: radial_polylines_.bundles(P)) {
                if(!radial_bundles_.is_sorted(bndl)) {
                    continue;
                }
                index_t N = radial_bundles_.nb_halfedges(bndl);
                for(index_t i=0; i<N; ++i) {
                    index_t i_next = (i == N-1) ? 0   : i+1;
                    index_t i_prev = (i == 0  ) ? N-1 : i-1;
                    index_t h      = radial_bundles_.halfedge(bndl,i);
                    index_t h_next = radial_bundles_.halfedge(bndl,i_next);
                    index_t h_prev = radial_bundles_.halfedge(bndl,i_prev);
                    halfedges_.sew2(h,halfedges_.alpha3(h_next));
                    halfedges_.sew2(h_prev,halfedges_.alpha3(h));
                }
            }
        }
        
        // Step 8: Identify regions
        {
            index_t nb_regions = get_surface_connected_components(mesh_,"chart");
            if(verbose_) {
                Logger::out("Intersect")
                    << "Found " << nb_regions << " regions" << std::endl;
            }
        }

    }

    
    /***********************************************************************/    

    void MeshSurfaceIntersection::RadialBundles::initialize() {
        // Sorted vector of halfedges
        // In this step, we only insert the ones such that v2 < v1
        // For the ones such that v1 > v2, we can deduce the information
        // from the first half.
        // Note: some of them come from the original halfedges and some of
        // them from the ones we just created by duplicating the faces.
        H_.reserve(mesh_.facet_corners.nb());
        H_.resize(0);
        for(index_t h: mesh_.facet_corners) {
            if(I_.halfedges_.vertex(h,0) < I_.halfedges_.vertex(h,1)) {
                H_.push_back(h);
            }
        }

        // Step 2: Lexicographic sort of the H array, so that "bundles" will be
        // contiguous. By "bundle", I mean the set of halfedges having the same
        // extremities in the same order.
        std::sort(
            H_.begin(), H_.end(),
            [&](index_t h1, index_t h2)->bool {
                geo_debug_assert(h1 < mesh_.facet_corners.nb());
                geo_debug_assert(h2 < mesh_.facet_corners.nb());
                index_t v10 = I_.halfedges_.vertex(h1,0);
                index_t v20 = I_.halfedges_.vertex(h2,0);
                if(v10 < v20) {
                    return true;
                }
                if(v10 > v20) {
                    return false;
                }
                return (I_.halfedges_.vertex(h1,1) < I_.halfedges_.vertex(h2,1));
            }
        );

        // Step 3: find "bundles" of halfedges in the sorted array. Two halfedges
        //  are in the same bundle if they connect the same pair of vertices (in
        //  the same order).
        //  Halfedges of bundle B are H[b],H[b+1]...H[e-1]
        //     where b = bndl_start[B] and e = bndl_start[B+1]
        //     (there are bndl_start.size()-1 bundles)
        bndl_start_.resize(0);
        for(index_t b=0, e=0; b<H_.size(); b=e) {
            bndl_start_.push_back(b);
            index_t v0 = I_.halfedges_.vertex(H_[b],0);
            index_t v1 = I_.halfedges_.vertex(H_[b],1);
            e = b+1;
            while(
                e < H_.size() &&
                I_.halfedges_.vertex(H_[e],0) == v0 &&
                I_.halfedges_.vertex(H_[e],1) == v1
            ) {
                ++e;
            }
        }
        bndl_start_.push_back(H_.size());

        // Step 4: construct second half of the sorted halfedges array from
        // the first half. Remember, at this point, H contains a mixture of
        // original halfedges and new ones, but they are all such that v1 < v2.
        // We now store all the bundles such that v1 > v2 (by "mirroring" the
        // initial ones, that is, traversing alpha3).
        {
            index_t bndl_start_size = bndl_start_.size();
            for(index_t bndl=0; bndl+1<bndl_start_size; ++bndl) {
                index_t b = bndl_start_[bndl];
                index_t e = bndl_start_[bndl+1];
                for(index_t i=b; i<e; ++i) {
                    H_.push_back(I_.halfedges_.alpha3(H_[i]));
                }
                bndl_start_.push_back(H_.size());
            }
        }
        
        // Step 5: chain bundles around vertices
        v_first_bndl_.assign(mesh_.vertices.nb(), NO_INDEX);
        bndl_next_around_v_.assign(bndl_start_.size()-1, NO_INDEX);
        for(index_t bndl = 0; bndl < nb(); ++bndl) {
            index_t v1 = vertex(bndl,0);
            bndl_next_around_v_[bndl] = v_first_bndl_[v1];
            v_first_bndl_[v1] = bndl;
        }

        bndl_is_sorted_.assign(nb(), false);

        if(!facet_chart_.is_bound()) {
            facet_chart_.bind(mesh_.facets.attributes(), "chart");
        }
        
        if(I_.verbose_) {
            Logger::out("Intersect") << "Found " << nb() << " bundles"
                                     << std::endl;
        }
    }

    void MeshSurfaceIntersection::RadialBundles::get_sorted_incident_charts(
        index_t bndl, vector<ChartPos>& chart_pos
    ) {
        chart_pos.resize(0);
        for(index_t h: halfedges(bndl)) {
            chart_pos.push_back(
                std::make_pair(
                    facet_chart_[I_.halfedges_.facet(h)], chart_pos.size()
                )
            );
        }
        std::sort(
            chart_pos.begin(), chart_pos.end(),
            [](const ChartPos& a, const ChartPos& b)->bool {
                return a.first < b.first;
            }
        );
    }
    
    void MeshSurfaceIntersection::RadialPolylines::initialize() {
        B_.resize(0);
        polyline_start_.resize(0);
        polyline_start_.push_back(0);
        vector<bool> bndl_visited(I_.radial_bundles_.nb(),false);
        for(index_t bndl: I_.radial_bundles_) {

            if(bndl_visited[bndl] || I_.radial_bundles_.nb_halfedges(bndl)==2) {
                continue;
            }

            // Find first bundle of the poly line:
            // Traverse bundles on border backwards until a
            // non-manifold vertex is reached or until we have
            // looped to our starting point.
            index_t bndl_first = bndl;
            for(;;) {
                index_t bndl_p =
                    I_.radial_bundles_.prev_along_polyline(bndl_first);
                if(bndl_p == NO_INDEX || bndl_p == bndl) {
                    break;
                }
                bndl_first = bndl_p;
            }

            // Traverse polyline by traversing bundles on border forward
            // until a non-manifold vertex is reached or until we have loped
            // to our starting point.
            index_t bndl_cur = bndl_first;
            for(;;) {
                B_.push_back(bndl_cur);
                bndl_visited[bndl_cur] = true;
                bndl_visited[I_.radial_bundles_.opposite(bndl_cur)] = true;
                index_t bndl_n=I_.radial_bundles_.next_along_polyline(bndl_cur);
                if(bndl_n == NO_INDEX || bndl_n == bndl_first) {
                    break;
                }
                bndl_cur = bndl_n;
            }
            polyline_start_.push_back(B_.size());
        }
        if(I_.verbose_) {
            Logger::out("Intersect") << "Found " <<nb()<< " polylines"
                                     << std::endl;
        }
    }

    void MeshSurfaceIntersection::RadialPolylines::get_skeleton(Mesh& skeleton) {
        skeleton.clear();
        skeleton.vertices.set_dimension(3);
        Attribute<bool> new_v_selection(
            skeleton.vertices.attributes(), "selection"
        );
        Attribute<bool> v_selection(
            mesh_.vertices.attributes(), "selection"
        );
        vector<index_t> v_id(mesh_.vertices.nb(), NO_INDEX);
        for(index_t bndl: B_) {
            for(index_t lv=0; lv<2; ++lv) {
                index_t v = I_.radial_bundles_.vertex(bndl,lv);
                if(v_id[v] == NO_INDEX) {
                    v_id[v] = skeleton.vertices.create_vertex(
                        mesh_.vertices.point_ptr(v)
                    );
                    if(I_.radial_bundles_.nb_bundles_around_vertex(v) != 2) {
                        v_selection[v] = true;
                        new_v_selection[v_id[v]] = true;
                    }
                }
            }
        }
        for(index_t polyline: *this) {
            for(index_t bndl: bundles(polyline)) {
                index_t v1 = I_.radial_bundles_.vertex(bndl,0);
                index_t v2 = I_.radial_bundles_.vertex(bndl,1);
                geo_assert(v_id[v1] != NO_INDEX);
                geo_assert(v_id[v2] != NO_INDEX);
                skeleton.edges.create_edge(v_id[v1], v_id[v2]);
            }
        }
    }

    void MeshSurfaceIntersection::RadialPolylines::radial_sort() {
        if(I_.verbose_) {
            Logger::out("Radial sort")
                << "Nb radial polylines:" << nb() << std::endl;
        }
        Stopwatch W("Radial sort",I_.verbose_);

        index_t nb_sorted = 0;
        index_t nb_to_sort = nb();
        
        parallel_for_slice(
            0, nb(),
            [&](index_t b, index_t e) {
                index_t tid = Thread::current_id();
                RadialSort RS(I_);

                vector<RadialBundles::ChartPos> ref_chart_to_radial_id;
                vector<RadialBundles::ChartPos> cur_chart_to_radial_id;
                vector<index_t> bndl_h;
                    
                for(index_t P = b; P < e; ++P) {

                    index_t bndl_ref = NO_INDEX;
                    index_t N = NO_INDEX;
                    
                    for(index_t bndl: bundles(P)) {
                        bool OK = I_.radial_bundles_.radial_sort(bndl, RS);
                        if(OK) {
                            I_.radial_bundles_.get_sorted_incident_charts(
                                bndl, ref_chart_to_radial_id
                            );
                            for(
                                index_t i=0;
                                i+1<ref_chart_to_radial_id.size(); ++i
                            ) {
                                OK = OK && (
                                    ref_chart_to_radial_id[i].first !=
                                    ref_chart_to_radial_id[i+1].first
                                ); 
                            }
                        }
                        if(OK) {
                            bndl_ref = bndl;
                            N = I_.radial_bundles_.nb_halfedges(bndl);
                            break;
                        }
                    }

                    // Copy order to all other bundles in polyline
                    for(index_t bndl: bundles(P)) {
                        if(bndl == bndl_ref) {
                            continue;
                        }

                        if(I_.radial_bundles_.nb_halfedges(bndl)==1) {
                            continue;
                        }
                        
                        // Necessary condition: reference bndl should be OK
                        // and current bndl should have same nbr of halfedges
                        bool OK = (bndl_ref != NO_INDEX) && (
                            I_.radial_bundles_.nb_halfedges(bndl)==N
                        );

                        // Now check that charts around current bndl are the
                        // same as charts around reference bndl. It can happen
                        // that they differ even if N matches (example21.csg)
                        if(OK) {
                            I_.radial_bundles_.get_sorted_incident_charts(
                                bndl, cur_chart_to_radial_id
                            );
                            for(index_t i=0; i<N; ++i) {
                                OK = OK && (
                                    cur_chart_to_radial_id[i].first ==
                                    ref_chart_to_radial_id[i].first
                                );
                            }
                        }

                        if(OK) {
                            // Here ref_bnfl has a valid order, and bndl has
                            // the same surrounding charts as ref_bndl,
                            // so we can reuse the radial order computed
                            // in ref_bndl
                            bndl_h.assign(N, NO_INDEX);
                            for(index_t i=0; i<N; ++i) {
                                // This halfedge ...
                                index_t h = I_.radial_bundles_.halfedge(
                                    bndl, cur_chart_to_radial_id[i].second
                                );
                                // ... goes here.
                                bndl_h[ref_chart_to_radial_id[i].second] = h;
                            }

                            I_.radial_bundles_.set_sorted_halfedges(bndl,bndl_h);
                        } else {
                            // Else compute the radial sort geometrically
                            OK = I_.radial_bundles_.radial_sort(bndl, RS);
                            // May return !OK (if not using geogram_plus) when it
                            // cannot sort (example_022.csg and example_024.csg)
                            if(!OK) {
                                std::cerr << std::endl
                                          << "FATAL ERROR: "
                                          << "Did not manage to sort a bundle"
                                          << " in Polyline of length "
                                          << nb_bundles(P)
                                          << std::endl;
                                std::cerr
                                    << "(if you reached this point, you may"
                                    << " need geogramplus, contact TESSAEL)"
                                    << std::endl;
                                geo_assert_not_reached;
                            }
                        }
                    }
                    if(I_.fine_verbose_) {
                        ++nb_sorted;
                        if(!(nb_sorted%100)) {
                            Logger::out("Radial sort")
                                << String::format(
                                    "[%2d]  %6d/%6d",
                                    int(tid),int(nb_sorted),int(nb_to_sort)
                                )
                                << std::endl;
                        }
                    }
                }
                if(I_.fine_verbose_) {
                    Logger::out("Radial sort")
                        << String::format("[%2d] done",int(tid))
                        << std::endl;
                }                    
            }
        );
    }
    
    /***********************************************************************/
}


namespace {
    using namespace GEO;
    
    /**
     * \brief Gets the position of the leftmost
     *  bit set in a 32 bits integer
     * \param[in] x the integer
     * \return the position of the leftmost bit
     *  set, or index_t(-1) if the specified integer
     *  is zero.
     */
    inline index_t leftmost_bit_set(index_t x) {
        index_t result = index_t(-1);
        for(index_t i=0; i<32; ++i) {
            if((x&1) != 0) {
                result = i;
            }
            x = x >> 1;
        }
        return result;
    }
}

/***************************************************/

namespace GEO {
    
    void MeshSurfaceIntersection::classify(const std::string& expr) {
        if(verbose_) {
            Logger::out("Weiler") << "Classifying facets" << std::endl;
        }
        
        // Takes as input a Weiler model, with duplicated interfaces,
        // operand bit (that indices for each triangle the set of operands
        // it corresponds to), volumetric alpha3 links and correct facet
        // adjacency links. It computes the operand_inclusion_bits attribute,
        // that indices for each triangle the set of operands that contains 
        // it, then evalues the boolean expression \p expr on all facets, 
        // and keeps the facets on the boundary of the region where \p expr
        // evaluates to true.        
            
        // Chart attribute corresponds to volumetric regions
        Attribute<index_t> chart(
            mesh_.facets.attributes(), "chart"
        );

        // For each facet, bit n set if facet belongs to the boundary
        // of operand n. There can be several bit sets if two operands
        // are tangent (and share facets).
        // For each facet pair (f, g=halfedges_.facet_alpha3(f)),
        // we have operand_bit[f] = operand_bit[g]
        Attribute<index_t> operand_bit(
            mesh_.facets.attributes(), "operand_bit"
        );

        // For each facet, bit n set if facet is inside operand n.
        // For each facet pair (f, g=halfedges_.facet_alpha3(f)),
        // we have nth-bit(operand_bit[f]) != nth-bit(operand_bit[g]),
        // that is, among a facet pair (f,g) on the boundary of operand n,
        // one of f,g is considered to be "outside" the operand, and the
        // other one is considered to be "inside".
        // The one that is considered to be "outside" is the one that belongs
        // to the chart that encloses the largest volume.
        Attribute<index_t> operand_inclusion_bits(
            mesh_.facets.attributes(), "operand_inclusion_bits"
        );
        
        // Get nb charts and nb operands
        index_t nb_charts = 0;
        index_t nb_operands = 0;
        for(index_t f: mesh_.facets) {
            nb_charts = std::max(nb_charts, chart[f]+1);
            nb_operands = nb_operands | operand_bit[f];
        }

        if(nb_operands != 0) {
            nb_operands = leftmost_bit_set(nb_operands) + 1;
        }
        
        if(verbose_) {
            Logger::out("Weiler") << "nb operands=" << nb_operands << std::endl;
        }

        // Get connected components, obtained by traversing all facet
        // adjacency links and volumetric alpha3 links

        index_t nb_components = 0;

        Attribute<index_t> facet_component(
            mesh_.facets.attributes(), "component"
        );
        
        for(index_t f: mesh_.facets) {
            facet_component[f] = NO_INDEX;
        }
        // vector<index_t> facet_component(mesh_.facets.nb(), index_t(-1));

        
        vector<index_t> component_vertex; // one vertex per component
        vector<index_t> component_inclusion_bits; 
        {
            for(index_t f:mesh_.facets) {
                if(facet_component[f] == index_t(-1)) {
                    
                    component_vertex.push_back(mesh_.facets.vertex(f,0));
                    component_inclusion_bits.push_back(0);
                    
                    std::stack<index_t> S;
                    facet_component[f] = nb_components;
                    S.push(f);
                    while(!S.empty()) {
                        index_t f1 = S.top();
                        S.pop();

                        {
                            index_t f2 = halfedges_.facet_alpha3(f1);
                            geo_debug_assert(f2 != index_t(-1));
                            if(facet_component[f2] == index_t(-1)) {
                                facet_component[f2]=facet_component[f1];
                                S.push(f2);
                            }
                        }
                        
                        for(index_t le1=0; le1<3; ++le1) {
                            index_t f2 = mesh_.facets.adjacent(f1,le1);
                            if(
                                f2 != index_t(-1) &&
                                facet_component[f2] == index_t(-1)
                            ) {
                                #ifdef GEO_DEBUG
                                index_t le2 = mesh_.facets.find_adjacent(f2,f1);
                                geo_debug_assert(
                                    mesh_.facets.vertex(f1,le1) !=
                                    mesh_.facets.vertex(f2,le2)
                                );
                                #endif
                                facet_component[f2] = facet_component[f1];
                                S.push(f2);
                            }
                        }
                    }
                    ++nb_components;
                }
            }
        }

        // Compute the volume enclosed by each chart
        vector<double> chart_volume(nb_charts,0.0);
        vec3 p0(0.0, 0.0, 0.0);
        for(index_t f: mesh_.facets) {
            index_t v1 = mesh_.facets.vertex(f,0);
            index_t v2 = mesh_.facets.vertex(f,1);
            index_t v3 = mesh_.facets.vertex(f,2);
            vec3 p1(mesh_.vertices.point_ptr(v1));
            vec3 p2(mesh_.vertices.point_ptr(v2));
            vec3 p3(mesh_.vertices.point_ptr(v3));
            chart_volume[chart[f]] += Geom::tetra_signed_volume(p0,p1,p2,p3);
        }

        // For each component, find the chart that encloses the largest
        // volume (it is the external boundary of the component)
        
        vector<double> max_chart_volume_in_component(nb_components, 0.0);
        vector<index_t> chart_with_max_volume_in_component(
            nb_components, index_t(-1)
        );

        for(index_t f: mesh_.facets) {
            double V = chart_volume[chart[f]];
            if( ::fabs(V) >=
                ::fabs(max_chart_volume_in_component[facet_component[f]])
              ) {
                max_chart_volume_in_component[facet_component[f]] = V;
                chart_with_max_volume_in_component[facet_component[f]] =
                    chart[f];
            }
        }

        
        // If there is more than one component, one needs to check
        // whether some components "float" inside other ones.
        // To do that, we determine the component inclusion bits
        // by launching a ray from a vertex of the component, and
        // checking parity of the number of intersections for each operand.
        
        if(nb_components > 1) {
            if(verbose_) {
                Logger::out("Weiler") << "Classifying " << nb_components
                                   << " components using ray tracing"
                                   << std::endl;
            }


            // We are going to use the copy of the original mesh for
            // raytracing (smaller number of triangles to raytrace, and
            // no vertex with exact coordinates, will be faster). We need
            // to copy facet component id to it.
            {
                Attribute<index_t> original_facet_id(
                    mesh_.facets.attributes(), "original_facet_id"
                );
                Attribute<index_t> facet_component_copy(
                    mesh_copy_.facets.attributes(), "component"
                );

                for(index_t f: mesh_copy_.facets) {
                    facet_component_copy[f] = NO_INDEX;
                }
                
                for(index_t f: mesh_.facets) {
                    index_t original_f = original_facet_id[f];
                    index_t component = facet_component[f];
                    facet_component_copy[original_f] = component;
                    // prefer original vertices for starting raytracing
                    for(index_t lv=0; lv<mesh_.facets.nb_vertices(f); ++lv) {
                        index_t v = mesh_.facets.vertex(f,lv);
                        if(vertex_to_exact_point_[v] == nullptr) { 
                            component_vertex[component] = v;
                        }
                    }
                }

                // TODO: understand how/why it can happen
                // ElectricCircuitElements
                // test_platonic
                for(index_t f: mesh_copy_.facets) {
                    if(facet_component_copy[f] >= nb_components) {
                        facet_component_copy[f] = NO_INDEX;
                    }
                }
                
            }
            
            parallel_for(
                0, nb_components, [&](index_t component) {
                    component_inclusion_bits[component] = classify_component(
                        component, component_vertex[component]
                    );
                }
            );
            if(verbose_) {
                Logger::out("Weiler") << "Done." << std::endl;
            }
        }
        
        // Compute operand inclusion bits for each facet,
        // by propagating component's inclusion bits
        // from component's external shell

        {
            vector<index_t> visited(mesh_.facets.nb(), false);
            std::stack<index_t> S;
        
            for(index_t f: mesh_.facets) {
                if(chart[f] ==
                   chart_with_max_volume_in_component[facet_component[f]]
                  ) {
                    visited[f] = 1;
                    operand_inclusion_bits[f] =
                        component_inclusion_bits[facet_component[f]];
                    S.push(f);
                }
            }
            
            while(!S.empty()) {
                index_t f1 = S.top();
                S.pop();
                {
                    index_t f2 = halfedges_.facet_alpha3(f1);
                    if(f2 != index_t(-1) && !visited[f2]) {
                        visited[f2] = true;
                        S.push(f2);
                        operand_inclusion_bits[f2] =
                            operand_inclusion_bits[f1] ^ operand_bit[f1];
                    }
                }
                for(index_t le=0; le<3; ++le) {
                    index_t f2 = mesh_.facets.adjacent(f1,le);
                    if(f2 != index_t(-1) && !visited[f2]) {
                        visited[f2] = true;
                        S.push(f2);
                        operand_inclusion_bits[f2] = operand_inclusion_bits[f1];
                    }
                }
            }
        }

        // Classify facets based on ther operand inclusion bits and on the
        // boolean expression

        vector<index_t> classify_facet(mesh_.facets.nb(), 0);
        if(expr == "intersection") {
            // If operation is an intersection, return the neighbors of
            // the facets that have all their operand inclusion bit sets.
            index_t all_bits_set = (1u << nb_operands)-1u;
            for(index_t f: mesh_.facets) {
                bool flipped =
                    (max_chart_volume_in_component[facet_component[f]] < 0.0);
                if(flipped) {
                    classify_facet[f] =
                        (operand_inclusion_bits[f] == all_bits_set);
                } else {
                    classify_facet[f] =
                        (
                            operand_inclusion_bits[halfedges_.facet_alpha3(f)] ==
                            all_bits_set
                        );
                }
            }
        } else {
            // For a general operation, return the facets f for which the
            // expression evaluates to false on f and to true on the neighbors
            // of f. Rember: what we want to compute is the *boundary* of the
            // region defined by the boolean expression, that is, the facets
            // for which the result of the boolean expression changes when they
            // are traversed by alpha3.
            try {
                BooleanExpression E(expr == "union" ? "*" : expr);
                for(index_t f: mesh_.facets) {
                    bool flipped =
                        (max_chart_volume_in_component[facet_component[f]]<0.0);
                    index_t f_in_sets = operand_inclusion_bits[f];
                    index_t g_in_sets = operand_inclusion_bits[
                        halfedges_.facet_alpha3(f)
                    ];
                    if(flipped) {
                        classify_facet[f] = (
                            E(f_in_sets) && !E(g_in_sets)
                        );
                    } else {
                        classify_facet[f] = (
                            E(g_in_sets) && !E(f_in_sets)
                        );
                    }
                }
            } catch(...) {
            }
        }

        for(index_t f: mesh_.facets) {
            classify_facet[f] = 1u - classify_facet[f];
        }
        
        mesh_.facets.delete_elements(classify_facet);
        //remove_fins();
        mesh_.facets.connect();
        
        if(verbose_) {
            Logger::out("Weiler") << "Facets classified" << std::endl;
        }
    }

    index_t MeshSurfaceIntersection::classify_component(
        index_t component, index_t v
    ) {
        Attribute<index_t> operand_bit(
            mesh_.facets.attributes(), "operand_bit"
        );
        Attribute<index_t> facet_component(
            mesh_.facets.attributes(), "component"
        );
        
        if(verbose_) {
            Logger::out("Weiler") << " component" << component << std::endl;
        }
        index_t component_inclusion_bits =
            tentatively_classify_component_vertex_fast(component, v);
        index_t nb_retries = 0;
        vector<index_t> component_vertices;
        while(component_inclusion_bits == NO_INDEX) {
            ++nb_retries;
            // geo_assert(nb_retries < 100);
            if(nb_retries >= 100) {
                std::cerr
                    << std::endl
                    << "FATAL ERROR: "
                    << "Did not manage to classify component"
                    << std::endl;
                std::cerr
                    << "(if you reached this point, you may"
                    << " need geogramplus, contact TESSAEL)"
                    << std::endl;
                component_inclusion_bits = 0;
                break;
            }
            
            if(verbose_) {
                Logger::out("Weiler") << "   ... retry"
                                      << std::endl;
            }

            // If first raytracing did not work,
            // get all vertices of the component
            // (here we got them three times but we do not care)
            if(component_vertices.size() == 0) {
                for(index_t f: mesh_.facets) {
                    if(facet_component[f] == component) {
                        component_vertices.push_back(
                            mesh_.facets.vertex(f,0)
                        );
                        component_vertices.push_back(
                            mesh_.facets.vertex(f,1)
                        );
                        component_vertices.push_back(
                            mesh_.facets.vertex(f,2)
                        );
                        
                    }
                }
            }

            v = component_vertices[
                index_t(Numeric::random_int32()) %
                component_vertices.size()
            ];
            component_inclusion_bits = tentatively_classify_component_vertex(
                component, v
            );            
        }
        return component_inclusion_bits;
    }

    index_t MeshSurfaceIntersection::tentatively_classify_component_vertex_fast(
        index_t component, index_t v
    ) {
        
        Attribute<index_t> operand_bit(
            mesh_copy_.facets.attributes(), "operand_bit"
        );

        Attribute<index_t> facet_component(
            mesh_copy_.facets.attributes(), "component"
        );

        
        index_t result = 0;

        // Only works when starting from an original vertex (so that we
        // can use double-precision coordinates everywhere, rather than
        // exact coordinates).
        if(vertex_to_exact_point_[v] != nullptr) {
            return NO_INDEX;
        }
        
        // v is an original vertex, we can use double-precision coords
        // everywhere.
            
        vec3 p1(mesh_.vertices.point_ptr(v));
        vec3 p2 = p1 + vec3(
            1.0e6*(2.0*Numeric::random_float64()-1.0),
            1.0e6*(2.0*Numeric::random_float64()-1.0),
            1.0e6*(2.0*Numeric::random_float64()-1.0)
        );

        // We compute the intersections with mesh_copy_, because
        // - it has a smaller number of facets
        // - all vertices have double-precision coordinates
        for(index_t t: mesh_copy_.facets) {
            
            // TODO: understand how this can happen 
            if(facet_component[t] == NO_INDEX) {
                return NO_INDEX;
            }
                
            // Skip intersections with this component
            if(facet_component[t] == component) {
                continue;
            }
                
            bool degenerate = false;
            
            vec3 q1(
                mesh_copy_.vertices.point_ptr(mesh_copy_.facets.vertex(t,0))
            );
            vec3 q2(
                mesh_copy_.vertices.point_ptr(mesh_copy_.facets.vertex(t,1))
            );
            vec3 q3(
                mesh_copy_.vertices.point_ptr(mesh_copy_.facets.vertex(t,2))
            );
                
            if(segment_triangle_intersection(p1,p2,q1,q2,q3,degenerate)) {
                // If there was an intersection, change the parity
                // relative to the concerned operands.
                result ^= operand_bit[t];
            }
            if(degenerate) {
                return NO_INDEX;
            }
        }

        return result;
    }


    index_t MeshSurfaceIntersection::tentatively_classify_component_vertex(
        index_t component, index_t v
    ) {
        Attribute<index_t> facet_component(
            mesh_.facets.attributes(), "component"
        );
        
        Attribute<index_t> operand_bit(
            mesh_.facets.attributes(), "operand_bit"
        );
        
        index_t result = 0;
        ExactPoint P1 = exact_vertex(v);
        vec3 D(
            1.0e6*(2.0*Numeric::random_float64()-1.0),
            1.0e6*(2.0*Numeric::random_float64()-1.0),
            1.0e6*(2.0*Numeric::random_float64()-1.0)
        );
        ExactPoint P2 = P1;
        P2.x += P2.w*exact::scalar(D.x);
        P2.y += P2.w*exact::scalar(D.y);
        P2.z += P2.w*exact::scalar(D.z);

        for(index_t t: mesh_.facets) {
    
            // Skip intersections with this component
            if(facet_component[t] == component) {
                continue;
            }
            // Test only one facet among each facet pair
            if(t > halfedges_.facet_alpha3(t)) {
                continue;
            }
                
            // Ignore facets that stay in same component
            // (component is same for f and alpha3(f)), this
            // corresponds to facets belonging to "fins".
            // HERE commented-out for now, to be tested
            /*
              if(
              facet_component[halfedges_.facet_alpha3(t)] ==
              facet_component[t]
              ) {
              continue;
              }
            */

            bool degenerate = false;
            ExactPoint p1 = exact_vertex(mesh_.facets.vertex(t,0));
            ExactPoint p2 = exact_vertex(mesh_.facets.vertex(t,1));
            ExactPoint p3 = exact_vertex(mesh_.facets.vertex(t,2));
            
            if(segment_triangle_intersection(P1,P2,p1,p2,p3,degenerate)) {
                // If there was an intersection, change the parity
                // relative to the concerned operands.
                result ^= operand_bit[t];
            }
            if(degenerate) {
                return NO_INDEX;
            }
        }

        return result;
    }
    
    /*************************************************************************/
    
    void MeshSurfaceIntersection::simplify_coplanar_facets(
        double angle_tolerance
    ) {
        if(interpolate_attributes_) {
            Logger::warn("Intersect")
                << "Cannot simplify coplanar facets with interpolated attributes"
                << std::endl;
            return;
        }

        if(verbose_) {
            Logger::out("Intersect") << "Simplifying coplanar facets"
                                     << std::endl;
        }
        Attribute<index_t> facet_group(mesh_.facets.attributes(), "group");
        vector<index_t> group_facet; // one facet per group
        for(index_t f: mesh_.facets) {
            facet_group[f] = index_t(-1);
        }
        Attribute<bool> keep_vertex(mesh_.facets.attributes(), "keep");
        for(index_t v: mesh_.vertices) {
            keep_vertex[v] = false;
        }
        index_t current_group = 0;
        {
            // clear attributes -------------v
            CoplanarFacets coplanar(*this, true, angle_tolerance); 
            for(index_t f: mesh_.facets) {
                if(facet_group[f] == NO_INDEX) {
                    coplanar.get(f, current_group); // This sets facet_group_[f]
                    coplanar.mark_vertices_to_keep();
                    group_facet.push_back(f);
                    ++current_group;
                }
            }
        }

        vector<index_t> remove_f(mesh_.facets.nb(), 0);
        index_t nb_groups = current_group;
        geo_assert(nb_groups == group_facet.size());

        // Triangulate coplanar facet groups in parallel
        Process::spinlock lock = GEOGRAM_SPINLOCK_INIT;
        parallel_for_slice(
            0, nb_groups, [&](index_t b, index_t e) {
                // do not clear attributes -----v
                CoplanarFacets coplanar(*this,false,angle_tolerance); 
                for(index_t group=b; group<e; ++group) {
                    coplanar.get(group_facet[group],group);
                    if(coplanar.nb_facets() < 2) {
                        continue;
                    }

                    coplanar.triangulate();

                    bool OK = true;
                    for(index_t t=0; t<coplanar.CDT.nT(); ++t) {
                        // If one of v1,v2,v3 is NO_INDEX,
                        // it means that v1,v2 or v3 was one of the four
                        // vertices of the external quad.
                        // It means that there was probably an
                        // inside/outside classification error.
                        
                        index_t v1=coplanar.CDT.vertex_id(coplanar.CDT.Tv(t,0));
                        index_t v2=coplanar.CDT.vertex_id(coplanar.CDT.Tv(t,1));
                        index_t v3=coplanar.CDT.vertex_id(coplanar.CDT.Tv(t,2));
                        OK = OK && (v1 != NO_INDEX);
                        OK = OK && (v2 != NO_INDEX);
                        OK = OK && (v3 != NO_INDEX);
                    }

                    if(OK) {
                        coplanar.mark_facets(remove_f);
                        Process::acquire_spinlock(lock);
                        for(index_t t=0; t<coplanar.CDT.nT(); ++t) {
                            index_t v1=
                                coplanar.CDT.vertex_id(coplanar.CDT.Tv(t,0));
                            index_t v2=
                                coplanar.CDT.vertex_id(coplanar.CDT.Tv(t,1));
                            index_t v3=
                                coplanar.CDT.vertex_id(coplanar.CDT.Tv(t,2));
                            index_t new_f =
                                mesh_.facets.create_triangle(v1,v2,v3);
                            facet_group[new_f] = current_group;
                        }
                        Process::release_spinlock(lock);
                    }
                }
            }
        );
        
        remove_f.resize(mesh_.facets.nb(),0);
        mesh_.facets.delete_elements(remove_f);
        mesh_.facets.connect();
    }
    

    
}

/******************************************************************************/

namespace {
    using namespace GEO;
    
    void copy_operand(Mesh& result, const Mesh& operand, index_t operand_id) {
        Attribute<index_t> operand_bit(
            result.facets.attributes(), "operand_bit"
        );
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
        Mesh& result, Mesh& A, Mesh& B,
        const std::string& operation, bool verbose
    ) {
        if(&result == &A) {
            Attribute<index_t> operand_bit(
                result.facets.attributes(), "operand_bit"
            );
            for(index_t f: A.facets) {
                operand_bit[f] = index_t(1);
            }
            copy_operand(result,B,1);
        } else if(&result == &B) {
            mesh_boolean_operation(
                result, B, A, (operation=="A-B") ? "B-A" : operation, verbose
            );
            return;
        } else {
            result.clear();
            result.vertices.set_dimension(3);
            copy_operand(result,A,0);
            copy_operand(result,B,1);
        }
        MeshSurfaceIntersection I(result);
        I.set_radial_sort(true);
        I.set_verbose(verbose);
        I.intersect();
        I.classify(operation);
        I.simplify_coplanar_facets();
    }
    
    void mesh_union(Mesh& result, Mesh& A, Mesh& B, bool verbose) {
        mesh_boolean_operation(result, A, B, "A+B", verbose);
    }

    void mesh_intersection(Mesh& result, Mesh& A, Mesh& B, bool verbose) {
        mesh_boolean_operation(result, A, B, "A*B", verbose);
    }

    void mesh_difference(Mesh& result, Mesh& A, Mesh& B, bool verbose) {
        mesh_boolean_operation(result, A, B, "A-B", verbose);
    }
    
    void mesh_remove_intersections(Mesh& M, index_t max_iter, bool verbose) {
        // TODO: same as tet_meshing() (compute union) ?
        for(index_t k=0; k<max_iter; ++k) {
            MeshSurfaceIntersection I(M);
            I.set_radial_sort(false);
            I.set_verbose(verbose);
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
