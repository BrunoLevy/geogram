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
#include <geogram/basic/boolean_expression.h>
#include <stack>

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
            dot(N1,ExactVec3(p1)),
            dot(N2,ExactVec3(q1)),
            dot(N3,ExactVec3(r1))
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
        dry_run_(false),
        use_pred_cache_insert_buffer_(false)
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

    void MeshInTriangle::clear() {
        vertex_.resize(0);
        edges_.resize(0);
        f1_ = index_t(-1);
        pred_cache_.clear();
        pred_cache_insert_buffer_.resize(0);
        use_pred_cache_insert_buffer_ = false;
        CDTBase2d::clear();
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

    void MeshInTriangle::begin_insert_transaction() {
        use_pred_cache_insert_buffer_ = true;
    }

    void MeshInTriangle::commit_insert_transaction() {
        for(const auto& it: pred_cache_insert_buffer_) {
            pred_cache_[it.first] = it.second;
        }
        pred_cache_insert_buffer_.resize(0);
        use_pred_cache_insert_buffer_ = false;
    }

    void MeshInTriangle::rollback_insert_transaction() {
        pred_cache_insert_buffer_.resize(0);
        use_pred_cache_insert_buffer_ = false;        
    }
    
    Sign MeshInTriangle::orient2d(index_t vx1,index_t vx2,index_t vx3) const {
        
        trindex K(vx1, vx2, vx3);

        if(use_pred_cache_insert_buffer_) {
            Sign result = PCK::orient_2d_projected(
                vertex_[K.indices[0]].point_exact,
                vertex_[K.indices[1]].point_exact,
                vertex_[K.indices[2]].point_exact,
                f1_normal_axis_
            );
            pred_cache_insert_buffer_.push_back(std::make_pair(K, result));
            if(odd_order(vx1,vx2,vx3)) {
                result = Sign(-result);
            }
            return result;
        }
        
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

    /**********************************************************************/

    ExactCDT2d::ExactCDT2d():
        use_pred_cache_insert_buffer_(false) {
#ifdef INTERSECTIONS_USE_EXACT_NT
        CDTBase2d::exact_incircle_ = true;
#else
        // Since incircle() with expansions computes approximated 
        // lifted coordinate, we need to activate additional
        // checks for Delaunayization.
        CDTBase2d::exact_incircle_ = false;
#endif
    }
    
    ExactCDT2d::~ExactCDT2d() {
    }
    
    void ExactCDT2d::clear() {
        point_.resize(0);
        id_.resize(0);
        pred_cache_.clear();
        pred_cache_insert_buffer_.resize(0);
        use_pred_cache_insert_buffer_ = false;
        cnstr_operand_bits_.resize(0);
        constraints_.resize(0);
        CDTBase2d::clear();
    }
    
    void ExactCDT2d::create_enclosing_quad(
        const ExactPoint& p1, const ExactPoint& p2,
        const ExactPoint& p3, const ExactPoint& p4
    ) {
        geo_assert(nv() == 0);
        geo_assert(nT() == 0);
        add_point(p1);
        add_point(p2);
        add_point(p3);
        add_point(p4);
        CDTBase2d::create_enclosing_quad(0,1,2,3);
    }

    index_t ExactCDT2d::insert(const ExactPoint& p, index_t id, index_t hint) {
        debug_check_consistency();
        add_point(p,id);
        index_t v = CDTBase2d::insert(point_.size()-1, hint);
        // If inserted point already existed in
        // triangulation, then nv() did not increase
        if(point_.size() > nv()) {
            point_.pop_back();
            id_.pop_back();
#ifndef INTERSECTIONS_USE_EXACT_NT
            length_.pop_back();
#endif            
        }
        debug_check_consistency();
        return v;
    }

    void ExactCDT2d::add_point(const ExactPoint& p, index_t id) {
        point_.push_back(p);
        id_.push_back(id);
#ifndef INTERSECTIONS_USE_EXACT_NT
        length_.push_back(
            (geo_sqr(p.x) + geo_sqr(p.y)).estimate() /
            geo_sqr(p.w).estimate() 
        );
#endif            
    }
    
    void ExactCDT2d::begin_insert_transaction() {
        use_pred_cache_insert_buffer_ = true;
    }

    void ExactCDT2d::commit_insert_transaction() {
        for(const auto& it: pred_cache_insert_buffer_) {
            pred_cache_[it.first] = it.second;
        }
        pred_cache_insert_buffer_.resize(0);
        use_pred_cache_insert_buffer_ = false;
    }

    void ExactCDT2d::rollback_insert_transaction() {
        pred_cache_insert_buffer_.resize(0);
        use_pred_cache_insert_buffer_ = false;        
    }
    
    Sign ExactCDT2d::orient2d(index_t i, index_t j, index_t k) const {

        /*
        return PCK::orient_2d(
            point_[i],
            point_[j],
            point_[k]
        );
        */
        
        trindex K(i, j, k);

        if(use_pred_cache_insert_buffer_) {
            Sign result = PCK::orient_2d(
                point_[K.indices[0]],
                point_[K.indices[1]],
                point_[K.indices[2]]
            );
            pred_cache_insert_buffer_.push_back(std::make_pair(K, result));
            if(odd_order(i,j,k)) {
                result = Sign(-result);
            }
            return result;
        }
        
        bool inserted;
        std::map<trindex, Sign>::iterator it;
        std::tie(it,inserted) = pred_cache_.insert(std::make_pair(K,ZERO));
        Sign result;
        
        if(inserted) {
            result = PCK::orient_2d(
                point_[K.indices[0]],
                point_[K.indices[1]],
                point_[K.indices[2]]
            );
            it->second = result;
        } else {
            result = it->second;
        }

        if(odd_order(i,j,k)) {
            result = Sign(-result);
        }
        
        return result;
    }
    
    Sign ExactCDT2d::incircle(index_t i,index_t j,index_t k,index_t l) const {
#ifdef INTERSECTIONS_USE_EXACT_NT        
        return PCK::incircle_2d_SOS(point_[i], point_[j], point_[k], point_[l]);
#else
        return PCK::incircle_2d_SOS_with_lengths(
            point_[i], point_[j], point_[k], point_[l],
            length_[i], length_[j], length_[k], length_[l]
        );
#endif        
    }
    
    index_t ExactCDT2d::create_intersection(
        index_t E1, index_t i, index_t j,
        index_t E2, index_t k, index_t l
    ) {
        
        geo_argused(i);
        geo_argused(j);
        geo_argused(k);
        geo_argused(l);

        // Here we could use i,j,k,l directly, but it is *much better* to take
        // the original extremities of the constrained segments, since they have
        // simpler coordinates ! (i,j,k,l might be themselves vertices created
        // from constraints intersections, whereas constraint extremities can only
        // be initial vertices).
        i = constraints_[E1].indices[0];
        j = constraints_[E1].indices[1];
        k = constraints_[E2].indices[0];
        l = constraints_[E2].indices[1];
        
        ExactVec2H U = point_[j] - point_[i];
        ExactVec2H V = point_[l] - point_[k];
        ExactVec2H D = point_[k] - point_[i];

        ExactRational t(
            det2x2(D.x, D.y, V.x, V.y) * U.w,
            det2x2(U.x, U.y, V.x, V.y) * D.w
        );

        point_.push_back(mix(t, point_[i], point_[j]));
        Numeric::optimize_number_representation(*point_.rbegin());
        
        id_.push_back(index_t(-1));
        index_t x = point_.size()-1;
        
        CDTBase2d::v2T_.push_back(index_t(-1));
        geo_debug_assert(x == CDTBase2d::nv_);
        ++CDTBase2d::nv_;

        return x;
    }

    void ExactCDT2d::classify_triangles(const std::string& expr) {
        
        facet_inclusion_bits_.assign(nT(), 0);

        DList S(*this, DLIST_S_ID);

        // Step 1: get triangles adjacent to the border,
        //   mark them as visited, classify them
        for(index_t t=0; t<nT(); ++t) {
            for(index_t le=0; le<3; ++le) {
                if(Tadj(t,le) == index_t(-1)) {
                    Tset_flag(t, T_VISITED_FLAG);
                    S.push_back(t);
                    break;
                }
            }
        }

        // Step 2: recursive traversal
        while(!S.empty()) {
            index_t t1 = S.pop_back();
            index_t t1_bits = facet_inclusion_bits_[t1];
            for(index_t le=0; le<3; ++le) {
                index_t t2 = Tadj(t1,le); 
                if(
                    t2 != index_t(-1) &&
                    !Tflag_is_set(t2,T_VISITED_FLAG)
                ) {
                    // t2 is included in the same operands as t1,
                    // except for the operands that touch the boundary
                    // between t1 and t2, for which inclusion changes
                    index_t t2_bits = t1_bits;
                    for(
                        index_t ecit = Tedge_cnstr_first(t1,le); 
                        ecit != index_t(-1);
                        ecit = edge_cnstr_next(ecit)
                    ) {
                        index_t cnstr = edge_cnstr(ecit);
                        t2_bits ^= cnstr_operand_bits_[cnstr];
                    }
                    facet_inclusion_bits_[t2] = t2_bits;
                    Tset_flag(t2, T_VISITED_FLAG);
                    S.push_back(t2);
                }
            }
        }
            
        // Step 3: reset visited flag
        for(index_t t=0; t<nT(); ++t) {
            Treset_flag(t, T_VISITED_FLAG);
        }

        // Step 4: mark triangles to be deleted
        if(expr == "intersection") {
            index_t all_bits_set = 0;
            for(index_t e_operand_bits: cnstr_operand_bits_) {
                all_bits_set |= e_operand_bits;
            }
            for(index_t t=0; t<nT(); ++t) {
                if(facet_inclusion_bits_[t] != all_bits_set) {
                    Tset_flag(t, T_MARKED_FLAG);
                }
            }
        } else {
            BooleanExpression E(expr == "union" ? "*" : expr);
            for(index_t t=0; t<nT(); ++t) {
                if(!E(facet_inclusion_bits_[t])) {
                    Tset_flag(t, T_MARKED_FLAG);
                }
            }
        }
        remove_marked_triangles();
    }
    
    void ExactCDT2d::save(const std::string& filename) const {
        Mesh M;
        Attribute<index_t> nb_cnstr(M.edges.attributes(),"nb_cnstr");
        M.vertices.set_dimension(2);
        for(const ExactPoint& P: point_) {
            double w = P.w.estimate();
            vec2 p(P.x.estimate() / w, P.y.estimate() / w);
            M.vertices.create_vertex(p.data());
        }
        for(index_t t=0; t<nT(); ++t) {
            index_t i = Tv(t,0);
            index_t j = Tv(t,1);
            index_t k = Tv(t,2);
            M.facets.create_triangle(i,j,k);

            for(index_t le=0; le<3; ++le) {
                if(Tedge_is_constrained(t,le)) {
                    index_t e = M.edges.create_edge(Tv(t,(le+1)%3), Tv(t,(le+2)%3));
                    nb_cnstr[e] = Tedge_cnstr_nb(t,le);
                }
            }
        }
        M.facets.connect();
        M.vertices.remove_isolated();
        mesh_save(M, filename);
    }

    /*****************************************************************************/

    CoplanarFacets::CoplanarFacets(MeshSurfaceIntersection& I) :
        intersection_(I),
        mesh_(I.target_mesh()),
        facet_group_(I.target_mesh().facets.attributes(),"group"),
        keep_vertex_(I.target_mesh().vertices.attributes(),"keep")
    {
        for(index_t f: mesh_.facets) {
            facet_group_[f] = index_t(-1);
        }
        for(index_t v: mesh_.vertices) {
            keep_vertex_[v] = false;
        }
        f_visited_.assign(mesh_.facets.nb(),false);
        v_visited_.assign(mesh_.vertices.nb(),false);
        v_prev_.resize(mesh_.vertices.nb());
        v_next_.resize(mesh_.vertices.nb());
        v_idx_.resize(mesh_.vertices.nb());
    }

    void CoplanarFacets::get(index_t f, index_t group_id) {
        facets.resize(0);
        vertices.resize(0);
        if(facet_group_[f] == index_t(-1)) {
            // Get facets, first call (facet_group not initialized)
            std::stack<index_t> S;
            facet_group_[f] = group_id;
            S.push(f);
            facets.push_back(f);
            while(!S.empty()) {
                index_t f1 = S.top();
                S.pop();
                for(index_t le1=0; le1<3; ++le1) {
                    index_t f2 = mesh_.facets.adjacent(f1,le1);
                    if(f2 != NO_INDEX && facet_group_[f2] == NO_INDEX) {
                        ExactPoint p1=intersection_.exact_vertex(
                            mesh_.facets.vertex(f1,le1)
                        );
                        ExactPoint p2=intersection_.exact_vertex(
                            mesh_.facets.vertex(f1,(le1+1)%3)
                        );
                        ExactPoint p3=intersection_.exact_vertex(
                            mesh_.facets.vertex(f1,(le1+2)%3)
                        );
                        index_t le2 = mesh_.facets.find_adjacent(f2,f1);
                        ExactPoint p4=intersection_.exact_vertex(
                            mesh_.facets.vertex(f2,(le2+2)%3)
                        );
                        if(triangles_are_coplanar(p1,p2,p3,p4)) {
                            facet_group_[f2] = facet_group_[f1];
                            S.push(f2);
                            facets.push_back(f2);
                        }
                    }
                }
            }
        } else {
            // Get facets, retreive facet group from attribute
            geo_assert(facet_group_[f] == group_id);
            std::stack<index_t> S;
            S.push(f);
            facets.push_back(f);
            f_visited_[f] = true;
            while(!S.empty()) {
                index_t f1 = S.top();
                S.pop();
                for(index_t le1=0; le1<3; ++le1) {
                    index_t f2 = mesh_.facets.adjacent(f1,le1);
                    if(
                        f2!=NO_INDEX && !f_visited_[f2] &&
                        facet_group_[f2] == group_id
                    ) {
                        S.push(f2);
                        facets.push_back(f2);
                        f_visited_[f2] = true;
                    }
                }
            }
            for(index_t ff: facets) {
                f_visited_[ff] = false;
            }
        }
        group_id_ = group_id;
        
        // Initialize vertices_
        for(index_t f1: facets) {
            for(index_t le=0; le<3; ++le) {
                index_t f2 = mesh_.facets.adjacent(f1,le);
                if(f2 == index_t(-1) || facet_group_[f2] != group_id_) {
                    index_t v1 = mesh_.facets.vertex(f1,le);
                    index_t v2 = mesh_.facets.vertex(f1,(le+1)%3);
                    if(!v_visited_[v1]) {
                        v_idx_[v1] = vertices.size();
                        vertices.push_back(v1);
                        v_visited_[v1] = true;
                    }
                    if(!v_visited_[v2]) {
                        v_idx_[v2] = vertices.size();
                        vertices.push_back(v2);
                        v_visited_[v2] = true;
                    }
                }
            }
        }
        
        for(index_t v: vertices) {
            v_prev_[v] = NO_INDEX;
            v_next_[v] = NO_INDEX;
            v_visited_[v] = false;
        }
        
        // Initialize vertices links
        for(index_t f1: facets) {
            for(index_t le=0; le<3; ++le) {
                index_t f2 = mesh_.facets.adjacent(f1,le);
                if(f2 == index_t(-1) || facet_group_[f2] != group_id_) {
                    
                    if(f2 == index_t(-1)) {
                        std::cerr << "INTERNAL BORDER" << std::endl;
                        //geo_assert_not_reached;
                    }
                    
                    index_t v1 = mesh_.facets.vertex(f1,le);
                    index_t v2 = mesh_.facets.vertex(f1,(le+1)%3);
                    if(v_next_[v1] == NO_INDEX) {
                        v_next_[v1] = v2;
                    } else {
                        v_next_[v1] = NON_MANIFOLD;
                    }
                    if(v_prev_[v2] == NO_INDEX) {
                        v_prev_[v2] = v1;
                    } else {
                        v_prev_[v2] = NON_MANIFOLD;
                    }
                }
            }
        }
        
        for(index_t v: vertices) {
            geo_assert(v_prev_[v] != NO_INDEX);
            geo_assert(v_next_[v] != NO_INDEX);
        }            
    }

    void CoplanarFacets::mark_vertices_to_keep() {
        for(index_t v2: vertices) {
            index_t v1 = v_prev_[v2];
            index_t v3 = v_next_[v2];
            if(v1 == NON_MANIFOLD || v3 == NON_MANIFOLD) {
                keep_vertex_[v2] = true;
            } else {
                geo_assert(v1 != NO_INDEX && v3 != NO_INDEX);
                geo_assert(v1 != v2);
                geo_assert(v2 != v3);
                //geo_assert(v3 != v1);
                ExactPoint p1 = intersection_.exact_vertex(v1);
                ExactPoint p2 = intersection_.exact_vertex(v2);
                ExactPoint p3 = intersection_.exact_vertex(v3);
                if(!PCK::aligned_3d(p1,p2,p3)) {
                    keep_vertex_[v2] = true;
                }
            }
        }
    }
    
    void CoplanarFacets::save_borders(const std::string& filename) {
        Mesh borders;
        borders.vertices.set_dimension(3);
        for(index_t v: vertices) {
            borders.vertices.create_vertex(
                mesh_.vertices.point_ptr(v)
            );
        }
        for(index_t v2: vertices) {
            index_t v1 = v_prev_[v2];
            index_t v3 = v_next_[v2];
            if(v1 != NON_MANIFOLD) {
                borders.edges.create_edge(v_idx_[v1],v_idx_[v2]);
            }
            if(v3 != NON_MANIFOLD) {
                borders.edges.create_edge(v_idx_[v2],v_idx_[v3]);
            }
        }
        Attribute<bool> selection(borders.vertices.attributes(), "selection");
        for(index_t v: vertices) {
            selection[v_idx_[v]] = keep_vertex_[v];
        }
        mesh_save(borders,filename);
    }

    void CoplanarFacets::save_facet_group(const std::string& filename) {
        Mesh M;
        Attribute<bool> keep_vertex(M.vertices.attributes(),"keep");
        M.vertices.set_dimension(3);
        for(index_t f: facets) {
            for(index_t lv=0; lv<3; ++lv) {
                index_t v = mesh_.facets.vertex(f,lv);
                v_idx_[v] = index_t(-1);
            }
        }
        for(index_t f: facets) {
            for(index_t lv=0; lv<3; ++lv) {
                index_t v = mesh_.facets.vertex(f,lv);
                if(v_idx_[v] == index_t(-1)) {
                    v_idx_[v] = M.vertices.create_vertex(
                        mesh_.vertices.point_ptr(v)
                    );
                    keep_vertex[v_idx_[v]] = keep_vertex_[v];
                }
            }
            M.facets.create_triangle(
                v_idx_[mesh_.facets.vertex(f,0)],
                v_idx_[mesh_.facets.vertex(f,1)],
                v_idx_[mesh_.facets.vertex(f,2)]
            );
        }

        for(index_t f: facets) {
            for(index_t lv=0; lv<3; ++lv) {
                index_t v = mesh_.facets.vertex(f,lv);
                v_idx_[v] = index_t(-1);
            }
        }
        
        M.facets.connect();
        mesh_save(M,filename);
    }
    
    void CoplanarFacets::triangulate() {
            
        coord_index_t U,V;
        {
            index_t f = facets[0];
            ExactPoint p1=intersection_.exact_vertex(mesh_.facets.vertex(f,0));
            ExactPoint p2=intersection_.exact_vertex(mesh_.facets.vertex(f,1));
            ExactPoint p3=intersection_.exact_vertex(mesh_.facets.vertex(f,2));
            coord_index_t projection_axis = triangle_normal_axis(p1,p2,p3);
            U = coord_index_t((projection_axis+1)%3);
            V = coord_index_t((projection_axis+2)%3);
            if(PCK::orient_2d(
                   ExactVec2H(p1[U],p1[V],p1.w),
                   ExactVec2H(p2[U],p2[V],p2.w),
                   ExactVec2H(p3[U],p3[V],p3.w)
               ) < 0) {
                std::swap(U,V);
            }
        }

        // Compute 2D projected BBOX
        double umin =  Numeric::max_float64();
        double vmin =  Numeric::max_float64();
        double umax = -Numeric::max_float64();
        double vmax = -Numeric::max_float64();
        for(index_t f: facets) {
            for(index_t lv=0; lv<3; ++lv) {
                index_t vx = mesh_.facets.vertex(f,lv);
                double u = mesh_.vertices.point_ptr(vx)[U];
                double v = mesh_.vertices.point_ptr(vx)[V];
                umin = std::min(umin, u);
                umax = std::max(umax, u);
                vmin = std::min(vmin, v);
                vmax = std::max(vmax, v);
            }
        }
        double d = std::max(umax-umin, vmax-vmin);
        d *= 10.0;
        d = std::max(d, 1.0);
        umin-=d;
        vmin-=d;
        umax+=d;
        vmax+=d;
        
        // Create CDT
        CDT.clear();
        CDT.create_enclosing_rectangle(umin, vmin, umax, vmax);
        
        for(index_t v: vertices) {
            if(keep_vertex_[v]) {
                ExactPoint P = intersection_.exact_vertex(v);
                v_idx_[v] = CDT.insert(ExactVec2H(P[U], P[V], P.w), v);
            }
        }

        // Get closed contours and polygonal lines connected to
        // non-manifold vertices
        for(index_t v: vertices) {
            if(keep_vertex_[v] && !v_visited_[v]) {
                vector<index_t> contour;
                index_t w = v;
                do {
                    v_visited_[w] = true;
                    if(keep_vertex_[w]) {
                        contour.push_back(w);
                    }
                    w = v_next_[w];
                } while(w != v && w != NON_MANIFOLD);
                
                if(w == NON_MANIFOLD) {
                    w = v_prev_[v];
                    while(w != NON_MANIFOLD) {
                        v_visited_[w] = true;
                        if(keep_vertex_[w]) {
                            contour.insert(contour.begin(),w);
                        }
                        w = v_prev_[w];
                    }
                } else {
                    contour.push_back(v);
                }
                    
                for(index_t i=0; i+1<contour.size(); ++i) {
                    CDT.insert_constraint(
                        v_idx_[contour[i]],
                        v_idx_[contour[i+1]],
                        index_t(-1)
                    );
                }
            }
        }
        
        // There is a particular case: edges with both extremities as
        // non-manifold vertices
        for(index_t t: facets) {
            for(index_t le=0; le<3; ++le) {
                index_t t2 = mesh_.facets.adjacent(t,le);
                if(t2 != index_t(-1) && facet_group_[t2] == group_id_) {
                    continue;
                }
                index_t v1 = mesh_.facets.vertex(t,le);
                index_t v2 = mesh_.facets.vertex(t,(le+1)%3);
                if(
                    (v_prev_[v1]==NON_MANIFOLD || v_next_[v1]==NON_MANIFOLD) &&
                    (v_prev_[v2]==NON_MANIFOLD || v_next_[v2]==NON_MANIFOLD) 
                ) {
                    CDT.insert_constraint(v_idx_[v1], v_idx_[v2], index_t(-1));
                }
            }
        }

        for(index_t v: vertices) {
            v_visited_[v] = false;
        }

        CDT.remove_external_triangles(true);
    }
        
    coord_index_t CoplanarFacets::triangle_normal_axis(
        const ExactPoint& p1, const ExactPoint& p2, const ExactPoint& p3
    ) {
        ExactPoint U = p2-p1;
        ExactPoint V = p3-p1;
        ExactVec3 N = cross(ExactVec3(U.x,U.y,U.z),ExactVec3(V.x,V.y,V.z));
        if(N.x.sign() == NEGATIVE) {
            N.x.negate();
        }
        if(N.y.sign() == NEGATIVE) {
            N.y.negate();
        }
        if(N.z.sign() == NEGATIVE) {
            N.z.negate();
        }
        if(N.x.compare(N.y) >= 0 && N.x.compare(N.z) >= 0) {
            return 0;
        }
        return (N.y.compare(N.z) >= 0) ? 1 : 2;
    }
    
    bool CoplanarFacets::triangles_are_coplanar(
        const ExactPoint& P1, const ExactPoint& P2,
        const ExactPoint& P3, const ExactPoint& P4
    ) {
        ExactPoint U = P2-P1;
        ExactPoint V = P3-P1;
        ExactPoint W = P4-P1;
        ExactVec3 N1 = cross(ExactVec3(U.x,U.y,U.z),ExactVec3(V.x,V.y,V.z));
        ExactVec3 N2 = cross(ExactVec3(U.x,U.y,U.z),ExactVec3(W.x,W.y,W.z));
        ExactVec3 N12 = cross(N1,N2);
        return (
            (N12.x.sign()==ZERO) &&
            (N12.y.sign()==ZERO) &&
            (N12.z.sign()==ZERO)
        );
    }
    
    /****************************************************************************/
}
