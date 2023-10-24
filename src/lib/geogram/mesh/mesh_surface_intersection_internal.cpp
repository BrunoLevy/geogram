/*
 *  COPYRIGHT (c) 2000-2022 Inria
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

#include <geogram/mesh/mesh_surface_intersection_internal.h>
#include <geogram/mesh/mesh_surface_intersection.h>
#include <geogram/basic/debug_stream.h>

namespace {
    using namespace GEO;

    /**
     * \brief Computes the exact intersection between the support
     *  planes of three triangles
     * \param[in] p1 , p2 , p3 the three vertices of the first triangle
     * \param[in] q1 , q2 , q3 the three vertices of the second triangle
     * \param[in] r1 , r2 , r3 the three vertices of the third triangle
     * \param[out] result the exact intersection between the three planes
     *  if it exsists
     * \retval true if the planes have an intersection
     * \retval false otherwise
     */
    bool get_three_planes_intersection(
        MeshSurfaceIntersection::ExactPoint& result,
        const vec3& p1, const vec3& p2, const vec3& p3,
        const vec3& q1, const vec3& q2, const vec3& q3,
        const vec3& r1, const vec3& r2, const vec3& r3
    ) {
        typedef MeshSurfaceIntersection::ExactVec3 ExactVec3;
        
        ExactVec3 N1 = triangle_normal<ExactVec3>(p1,p2,p3);
        ExactVec3 N2 = triangle_normal<ExactVec3>(q1,q2,q3);
        ExactVec3 N3 = triangle_normal<ExactVec3>(r1,r2,r3);

        ExactVec3 B(
            dot(N1,make_vec3<ExactVec3>(p1)),
            dot(N2,make_vec3<ExactVec3>(q1)),
            dot(N3,make_vec3<ExactVec3>(r1))
        );
        
        result.w = det3x3(
            N1.x, N1.y, N1.z,
            N2.x, N2.y, N2.z,
            N3.x, N3.y, N3.z
        );

        if(result.w.sign() == ZERO) {
            return false;
        }
        
        result.x = det3x3(
            B.x, N1.y, N1.z,
            B.y, N2.y, N2.z,
            B.z, N3.y, N3.z
        );

        result.y = det3x3(
            N1.x, B.x, N1.z,
            N2.x, B.y, N2.z,
            N3.x, B.z, N3.z
        );

        result.z = det3x3(
            N1.x, N1.y, B.x,
            N2.x, N2.y, B.y,
            N3.x, N3.y, B.z
        );

        return true;
    }

    /**
     * \brief Computes the exact intersection between the support plane
     *  of a triangle and the support line of a segment
     * \pre The intersection exists
     * \param[in] p1 , p2 , p3 the three vertices of the triangle
     * \param[in] q1 , q2 the two vertices of the segment
     * \return the exact intersection between the plane and the line
     */
    MeshSurfaceIntersection::ExactPoint plane_line_intersection(
        const vec3& p1, const vec3& p2, const vec3& p3,
        const vec3& q1, const vec3& q2
    ) {
        typedef MeshSurfaceIntersection::ExactVec3 ExactVec3;
        typedef MeshSurfaceIntersection::ExactCoord ExactCoord;
        typedef MeshSurfaceIntersection::ExactRational ExactRational;        
        // Moller & Trumbore's algorithm
        // see: https://stackoverflow.com/questions/42740765/
        //  intersection-between-line-and-triangle-in-3d
        ExactVec3 D   = make_vec3<ExactVec3>(q1,q2);
        ExactVec3 E1  = make_vec3<ExactVec3>(p1,p2);
        ExactVec3 E2  = make_vec3<ExactVec3>(p1,p3);
        ExactVec3 AO  = make_vec3<ExactVec3>(p1,q1);
        ExactVec3 N   = cross(E1,E2);
        ExactCoord d  = -dot(D,N);
        geo_debug_assert(d.sign() != ZERO);
        ExactRational t(dot(AO,N),d);
        return mix(t,q1,q2);
    }
}

namespace GEO {
    
    void MeshInTriangle::Vertex::print(std::ostream& out) const {
        if(sym.f1 != index_t(-1)) {
            out << " ( ";
            out << sym.f1;
            out << region_to_string(sym.R1).substr(2);
        }
        if(sym.f2 != index_t(-1)) {
            out << " /\\ ";
            out << sym.f2;
            out << region_to_string(sym.R2).substr(2);
        }
        if(sym.f1 != index_t(-1)) {
            out << " ) ";
        }
    }

    MeshInTriangle::ExactPoint MeshInTriangle::Vertex::compute_geometry() {
        // Case 1: f1 vertex
        if(region_dim(sym.R1) == 0) {
            index_t lv = index_t(sym.R1) - index_t(T1_RGN_P0);
            geo_assert(lv < 3);
            mesh_vertex_index = mesh().facets.vertex(sym.f1,lv);
            vec3 p = mit->mesh_vertex(mesh_vertex_index);
            return ExactPoint(p);
        }

        geo_assert(sym.f1 != index_t(-1) && sym.f2 != index_t(-1));
        
        // Case 2: f2 vertex
        if(region_dim(sym.R2) == 0) {
            index_t lv = index_t(sym.R2) - index_t(T2_RGN_P0);
            geo_assert(lv < 3);
            mesh_vertex_index = mesh().facets.vertex(sym.f2, lv);
            vec3 p = mit->mesh_vertex(mesh_vertex_index);
            return ExactPoint(p);
        }

        // case 3: f1 /\ f2 edge in 3D or f1 edge /\ f2 edge in 3D
        if(
            (region_dim(sym.R1) == 2 || region_dim(sym.R1) == 1) &&
            region_dim(sym.R2) == 1
        ) {
            vec3 p1 = mit->mesh_facet_vertex(sym.f1, 0);
            vec3 p2 = mit->mesh_facet_vertex(sym.f1, 1);
            vec3 p3 = mit->mesh_facet_vertex(sym.f1, 2);
            index_t e = index_t(sym.R2)-index_t(T2_RGN_E0);
            geo_debug_assert(e<3);
            vec3 q1 = mit->mesh_facet_vertex(sym.f2, (e+1)%3);
            vec3 q2 = mit->mesh_facet_vertex(sym.f2, (e+2)%3);
            
            bool seg_seg_two_D = (
                region_dim(sym.R1) == 1 &&
                PCK::orient_3d(p1,p2,p3,q1) == ZERO &&
                PCK::orient_3d(p1,p2,p3,q2) == ZERO) ;
            
            if(!seg_seg_two_D) {
                return plane_line_intersection(p1,p2,p3,q1,q2);
            }
        }
        
        // case 4: f1 edge /\ f2
        if(region_dim(sym.R1) == 1 && region_dim(sym.R2) == 2) {
            index_t e = index_t(sym.R1)-index_t(T1_RGN_E0);
            geo_debug_assert(e<3);
            vec3 p1 = mit->mesh_facet_vertex(sym.f2,0);
            vec3 p2 = mit->mesh_facet_vertex(sym.f2,1);
            vec3 p3 = mit->mesh_facet_vertex(sym.f2,2);
            vec3 q1 = mit->mesh_facet_vertex(sym.f1, (e+1)%3);
            vec3 q2 = mit->mesh_facet_vertex(sym.f1, (e+2)%3);
            return plane_line_intersection(p1,p2,p3,q1,q2);
        }
        
        // case 5: f1 edge /\ f2 edge in 2D
        if(region_dim(sym.R1) == 1 && region_dim(sym.R2) == 1) {
            index_t e1 = index_t(sym.R1) - index_t(T1_RGN_E0);
            geo_debug_assert(e1 < 3);
            index_t e2 = index_t(sym.R2) - index_t(T2_RGN_E0);
            geo_debug_assert(e2 < 3);
            vec2 p1 = mit->mesh_facet_vertex_UV(sym.f1, (e1+1)%3);
            vec2 p2 = mit->mesh_facet_vertex_UV(sym.f1, (e1+2)%3);
            vec2 q1 = mit->mesh_facet_vertex_UV(sym.f2, (e2+1)%3);
            vec2 q2 = mit->mesh_facet_vertex_UV(sym.f2, (e2+2)%3);
            vec3 P1 = mit->mesh_facet_vertex(sym.f1, (e1+1)%3);
            vec3 P2 = mit->mesh_facet_vertex(sym.f1, (e1+2)%3);

            ExactVec2 D1 = make_vec2<ExactVec2>(p1,p2);
            ExactVec2 D2 = make_vec2<ExactVec2>(q1,q2);
            ExactCoord d = det(D1,D2);
            geo_debug_assert(d.sign() != ZERO);
            ExactVec2 AO = make_vec2<ExactVec2>(p1,q1);
            ExactRational t(det(AO,D2),d);
            return mix(t,P1,P2);
        }
        
        // Normally we enumerated all possible cases
        geo_assert_not_reached;
    }

    void MeshInTriangle::Vertex::init_geometry(const ExactPoint& P) {
        point_exact = P;
        Numeric::optimize_number_representation(point_exact);
#ifndef INTERSECTIONS_USE_EXACT_NT
        l = (geo_sqr(P[mit->u_]) + geo_sqr(P[mit->v_])).estimate() /
             geo_sqr(P.w).estimate() ;
#endif        
    }

    MeshInTriangle::MeshInTriangle(MeshSurfaceIntersection& EM) :
        exact_mesh_(EM),
        mesh_(EM.readonly_mesh()),
        f1_(index_t(-1)),
        dry_run_(false)
    {
#ifdef INTERSECTIONS_USE_EXACT_NT
        CDTBase2d::exact_incircle_ = true;
#else
        // Since incircle() with expansions computes approximated 
        // lifted coordinate, we need to activate additional
        // checks for Delaunayization.
        CDTBase2d::exact_incircle_ = false;
#endif
    }


    void MeshInTriangle::begin_facet(index_t f) {
        f1_ = f;

        latest_f2_ = index_t(-1);
        latest_f2_count_ = 0;
        
        vec3 p1 = mesh_facet_vertex(f,0);
        vec3 p2 = mesh_facet_vertex(f,1);
        vec3 p3 = mesh_facet_vertex(f,2);

        geo_debug_assert(!PCK::aligned_3d(p1,p2,p3));
        
        f1_normal_axis_ = PCK::triangle_normal_axis(
            p1,p2,p3
        );
        
        u_ = coord_index_t((f1_normal_axis_ + 1) % 3);
        v_ = coord_index_t((f1_normal_axis_ + 2) % 3);
        for(index_t lv=0; lv<3; ++lv) {
            vertex_.push_back(Vertex(this, f, lv));
        }
        
        CDTBase2d::create_enclosing_triangle(0,1,2);
        
        edges_.push_back(Edge(1,2));
        edges_.push_back(Edge(2,0));
        edges_.push_back(Edge(0,1));
        
        has_planar_isect_ = false;
    }
        
    index_t MeshInTriangle::add_vertex(
        index_t f2, TriangleRegion R1, TriangleRegion R2
    ) {
        geo_debug_assert(f1_ != index_t(-1));
        
        // If the same f2 comes more than twice, then
        // we got a planar facet /\ facet intersection
        // (and it is good to know it, see get_constraints())
        if(f2 != index_t(-1) && f2 == latest_f2_) {
            ++latest_f2_count_;
            if(latest_f2_count_ > 2) {
                has_planar_isect_ = true;
            }
        } else {
            latest_f2_ = f2;
            latest_f2_count_ = 0;
        }
        
        // If vertex is a macro-vertex, return it directly.
        if(region_dim(R1) == 0) {
            return index_t(R1);
        }
        
        // Create the vertex
        vertex_.push_back(Vertex(this, f1_, f2, R1, R2));
        
        // Insert it into the triangulation
        index_t v = CDTBase2d::insert(vertex_.size()-1);
        
        // If it was an existing vertex, return the existing vertex
        if(vertex_.size() > CDTBase2d::nv()) {
            vertex_.pop_back();
        }
        return v;
    }
    
    void MeshInTriangle::add_edge(
        index_t f2,
        TriangleRegion AR1, TriangleRegion AR2,
        TriangleRegion BR1, TriangleRegion BR2
    ) {
        index_t v1 = add_vertex(f2, AR1, AR2);
        index_t v2 = add_vertex(f2, BR1, BR2);
        
        // If both extremities are on the same edge of f1,
        // we do not add the edge, because it will be generated
        // when remeshing the edge of f1
        if(region_dim(regions_convex_hull(AR1,BR1)) == 1) {
            return;
        }
        
        // Generate also the combinatorial information of the edge,
        // that indicates whether both extremities are on the same
        // edge of f2 (useful later to compute the intersections)
        edges_.push_back(Edge(v1,v2,f2,regions_convex_hull(AR2,BR2)));
        
        // Constraints will be added to the triangulation during commit()
    }

    void MeshInTriangle::commit() {

        for(const Edge& E: edges_) {
            CDTBase2d::insert_constraint(E.v1, E.v2);
        }

        if(dry_run_) {
            return;
        }
        
        // Protect global mesh from concurrent accesses
        exact_mesh_.lock();
        
        // Create vertices and facets in target mesh
        for(index_t i=0; i<vertex_.size(); ++i) {
            // Vertex already exists in this MeshInTriangle
            if(vertex_[i].mesh_vertex_index != index_t(-1)) {
                continue;
            }
            vertex_[i].mesh_vertex_index =
                exact_mesh_.find_or_create_exact_vertex(
                    vertex_[i].point_exact
                );
        }
        
        // Create facets in target mesh
        for(index_t t=0; t<CDTBase2d::nT(); ++t) {
            index_t i = CDTBase2d::Tv(t,0);
            index_t j = CDTBase2d::Tv(t,1);
            index_t k = CDTBase2d::Tv(t,2);
            i = vertex_[i].mesh_vertex_index;
            j = vertex_[j].mesh_vertex_index;
            k = vertex_[k].mesh_vertex_index;                    
            index_t new_t = target_mesh().facets.create_triangle(i,j,k);
            // Copy all attributes from initial facet
            target_mesh().facets.attributes().copy_item(new_t, f1_);
        }
        
        // We are done with modification in the mesh
        exact_mesh_.unlock();
    }
    
    void MeshInTriangle::get_constraints(Mesh& M, bool with_edges) const {
        if(M.vertices.nb() == 0) {
            M.vertices.set_dimension(2);
            for(index_t v=0; v<vertex_.size(); ++v) {
                vec2 p = vertex_[v].get_UV_approx();
                M.vertices.create_vertex(p.data());
            }
        }
        if(with_edges && M.edges.nb() == 0) {
            index_t i=0;
            for(const Edge& E: edges_) {
                M.edges.create_edge(E.v1, E.v2); 
                ++i;
            }
        }
    }

    /**
     * \brief Tests the parity of the permutation of a list of
     *  four distinct indices with respect to the canonical order.
     */  
    static bool odd_order(index_t i, index_t j, index_t k) {
        // Implementation: sort the elements (bubble sort is OK for
        // such a small number), and invert parity each time
        // two elements are swapped.
        index_t tab[3] = { i, j, k};
        const int N = 3;
        bool result = false;
        for (int I = 0; I < N - 1; ++I) {
            for (int J = 0; J < N - I - 1; ++J) {
                if (tab[J] > tab[J + 1]) {
                    std::swap(tab[J], tab[J + 1]);
                    result = !result;
                }
            }
        }
        return result;
    }
    
    
    Sign MeshInTriangle::orient2d(index_t vx1,index_t vx2,index_t vx3) const {
        
        trindex K(vx1, vx2, vx3);
        bool inserted;
        std::map<trindex, Sign>::iterator it;
        std::tie(it,inserted) = pred_cache_.insert(std::make_pair(K,ZERO));
        Sign result;
        
        if(inserted) {
            result = PCK::orient_2d_projected(
                vertex_[K.indices[0]].point_exact,
                vertex_[K.indices[1]].point_exact,
                vertex_[K.indices[2]].point_exact,
                f1_normal_axis_
            );
            it->second = result;
        } else {
            result = it->second;
        }

        if(odd_order(vx1,vx2,vx3)) {
            result = Sign(-result);
        }
        
        return result;
    }
    
    Sign MeshInTriangle::incircle(
        index_t v1,index_t v2,index_t v3,index_t v4
    ) const {
        ExactVec2H p1(
            vertex_[v1].point_exact[u_],
            vertex_[v1].point_exact[v_],
            vertex_[v1].point_exact.w
        );
        ExactVec2H p2(
            vertex_[v2].point_exact[u_],
            vertex_[v2].point_exact[v_],
            vertex_[v2].point_exact.w
        );
        ExactVec2H p3(
            vertex_[v3].point_exact[u_],
            vertex_[v3].point_exact[v_],
            vertex_[v3].point_exact.w
        );
        ExactVec2H p4(
            vertex_[v4].point_exact[u_],
            vertex_[v4].point_exact[v_],
            vertex_[v4].point_exact.w
        );
#ifdef INTERSECTIONS_USE_EXACT_NT        
        return PCK::incircle_2d_SOS(p1,p2,p3,p4);
#else
        return PCK::incircle_2d_SOS_with_lengths(
            p1,p2,p3,p4,
            vertex_[v1].l,
            vertex_[v2].l,
            vertex_[v3].l,
            vertex_[v4].l
        );
#endif        
    }

    index_t MeshInTriangle::create_intersection(
        index_t e1, index_t i, index_t j,
        index_t e2, index_t k, index_t l
    ) {
        geo_argused(i);
        geo_argused(j);
        geo_argused(k);
        geo_argused(l);
        ExactPoint I;
        get_edge_edge_intersection(e1,e2,I);
        vertex_.push_back(Vertex(this,I));
        index_t x = vertex_.size()-1;
        CDTBase2d::v2T_.push_back(index_t(-1));
        geo_debug_assert(x == CDTBase2d::nv_);
        ++CDTBase2d::nv_;
        return x;
    }
    
    void MeshInTriangle::get_edge_edge_intersection(
        index_t e1, index_t e2, ExactPoint& I
    ) const {
        index_t f1 = f1_;
        index_t f2 = edges_[e1].sym.f2; 
        index_t f3 = edges_[e2].sym.f2; 
        
        geo_assert(f1 != index_t(-1));
        geo_assert(f2 != index_t(-1));
        geo_assert(f3 != index_t(-1));                        
        
        vec3 P[9] = {
            mesh_facet_vertex(f1,0), mesh_facet_vertex(f1,1),
            mesh_facet_vertex(f1,2),
            mesh_facet_vertex(f2,0), mesh_facet_vertex(f2,1),
            mesh_facet_vertex(f2,2),
            mesh_facet_vertex(f3,0), mesh_facet_vertex(f3,1),
            mesh_facet_vertex(f3,2)
        };
        
        if(!get_three_planes_intersection(
               I,
               P[0], P[1], P[2],
               P[3], P[4], P[5],
               P[6], P[7], P[8]
           )) {
            get_edge_edge_intersection_2D(e1,e2,I);
            return;
        }
    }             

    void MeshInTriangle::get_edge_edge_intersection_2D(
        index_t e1, index_t e2, ExactPoint& I
    ) const {
        const Edge& E1 = edges_[e1];
        const Edge& E2 = edges_[e2];
        
        if(
            region_dim(E1.sym.R2) == 1 &&
            region_dim(E2.sym.R2) == 1
        ) {
            index_t le1 = index_t(E1.sym.R2)-index_t(T2_RGN_E0);
            index_t le2 = index_t(E2.sym.R2)-index_t(T2_RGN_E0);
            geo_assert(le1 < 3);
            geo_assert(le2 < 3);
            
            vec2 p1_uv = mesh_facet_vertex_UV(E1.sym.f2, (le1+1)%3);
            vec2 p2_uv = mesh_facet_vertex_UV(E1.sym.f2, (le1+2)%3);
            vec2 q1_uv = mesh_facet_vertex_UV(E2.sym.f2, (le2+1)%3);
            vec2 q2_uv = mesh_facet_vertex_UV(E2.sym.f2, (le2+2)%3);

            ExactVec2 C1 = make_vec2<ExactVec2>(p1_uv, p2_uv);
            ExactVec2 C2 = make_vec2<ExactVec2>(q2_uv, q1_uv);
            ExactVec2 B  = make_vec2<ExactVec2>(p1_uv, q1_uv);
            ExactCoord d = det(C1,C2);
            geo_debug_assert(d.sign() != ZERO);
            ExactRational t(det(B,C2),d);
            I = mix(
                t,
                mesh_facet_vertex(E1.sym.f2,(le1+1)%3),
                mesh_facet_vertex(E1.sym.f2,(le1+2)%3)
            );
            
        } else {
            geo_assert(
                region_dim(E1.sym.R2) == 1 || region_dim(E2.sym.R2) == 1
            );
            index_t f1 = E1.sym.f2;
            TriangleRegion R1 = E1.sym.R2;
            index_t f2 = E2.sym.f2;
            TriangleRegion R2 = E2.sym.R2;
            if(region_dim(R1) == 1) {
                std::swap(f1,f2);
                std::swap(R1,R2);
            }
            
            index_t e = index_t(R2) - index_t(T2_RGN_E0);
            geo_assert(e < 3);
            
            I = plane_line_intersection(
                mesh_facet_vertex(f1,0),
                mesh_facet_vertex(f1,1),
                mesh_facet_vertex(f1,2),
                mesh_facet_vertex(f2,(e+1)%3),
                mesh_facet_vertex(f2,(e+2)%3)
            );
        }
    }

    void MeshInTriangle::save(const std::string& filename) const {
        Mesh M;
        M.vertices.set_dimension(2);
        for(index_t v=0; v<CDTBase2d::nv(); ++v) {
            vec2 p = vertex_[v].get_UV_approx();
            M.vertices.create_vertex(p.data());
        }
        for(index_t t=0; t<CDTBase2d::nT(); ++t) {
            M.facets.create_triangle(
                CDTBase2d::Tv(t,0),
                CDTBase2d::Tv(t,1),
                CDTBase2d::Tv(t,2)
            );
        }
        
        Attribute<double> tex_coord;
        tex_coord.create_vector_attribute(
            M.facet_corners.attributes(), "tex_coord", 2
        );
        static double triangle_tex[3][2] = {
            {0.0, 0.0},
            {1.0, 0.0},
            {0.0, 1.0}
        };
        for(index_t c: M.facet_corners) {
            tex_coord[2*c]   = triangle_tex[c%3][0];
            tex_coord[2*c+1] = triangle_tex[c%3][1];
        }
        mesh_save(M, filename);
    }
}
