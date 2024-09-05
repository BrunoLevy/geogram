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

#include <geogram/mesh/mesh_AABB.h>
#include <geogram/mesh/mesh_reorder.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/numerics/predicates.h>
#include <geogram/basic/geometry_nd.h>

namespace {

    using namespace GEO;

    /**
     * \brief Finds the nearest point in a mesh facet from a query point.
     * \param[in] M the mesh
     * \param[in] p the query point
     * \param[in] f index of the facet in \p M
     * \param[out] nearest_p the point of facet \p f nearest to \p p
     * \param[out] squared_dist the squared distance between
     *  \p p and \p nearest_p
     */
    void get_point_facet_nearest_point(
        const Mesh& M,
        const vec3& p,
        index_t f,
        vec3& nearest_p,
        double& squared_dist
    ) {
        if(M.facets.nb_vertices(f) == 3) {
            index_t c = M.facets.corners_begin(f);
            const vec3& p1 = Geom::mesh_vertex(M, M.facet_corners.vertex(c));
            const vec3& p2 = Geom::mesh_vertex(M, M.facet_corners.vertex(c+1));
            const vec3& p3 = Geom::mesh_vertex(M, M.facet_corners.vertex(c+2));
            double lambda1, lambda2, lambda3;  // barycentric coords, not used.
            squared_dist = Geom::point_triangle_squared_distance(
                p, p1, p2, p3, nearest_p, lambda1, lambda2, lambda3
            );
        } else {
            squared_dist = Numeric::max_float64();
            index_t c1 = M.facets.corners_begin(f);
            const vec3& p1 = Geom::mesh_vertex(M, M.facet_corners.vertex(c1));
            for(index_t c2 = c1+1; c2+1<M.facets.corners_end(f); ++c2) {
                const vec3& p2 =
                    Geom::mesh_vertex(M, M.facet_corners.vertex(c2));
                index_t c3 = c2+1;
                const vec3& p3 =
                    Geom::mesh_vertex(M, M.facet_corners.vertex(c3));
                double lambda1, lambda2, lambda3;  // barycentric coords,unused.
                vec3 cur_nearest_p;
                double cur_squared_dist = Geom::point_triangle_squared_distance(
                    p, p1, p2, p3, cur_nearest_p, lambda1, lambda2, lambda3
                );
                if(cur_squared_dist < squared_dist) {
                    squared_dist = cur_squared_dist;
                    nearest_p = cur_nearest_p;
                }
            }
        }
    }

    /**
     * \brief Computes the squared distance between a point and a Box.
     * \param[in] p the point
     * \param[in] B the box
     * \return the squared distance between \p p and \p B
     * \pre p is inside B
     */
    double inner_point_box_squared_distance(
        const vec3& p,
        const Box& B
    ) {
        geo_debug_assert(B.contains(p));
        double result = geo_sqr(p[0] - B.xyz_min[0]);
        result = std::min(result, geo_sqr(p[0] - B.xyz_max[0]));
        for(coord_index_t c = 1; c < 3; ++c) {
            result = std::min(result, geo_sqr(p[c] - B.xyz_min[c]));
            result = std::min(result, geo_sqr(p[c] - B.xyz_max[c]));
        }
        return result;
    }

    /**
     * \brief Computes the squared distance between a point and a Box
     *  with negative sign if the point is inside the Box.
     * \param[in] p the point
     * \param[in] B the box
     * \return the signed squared distance between \p p and \p B
     */
    double point_box_signed_squared_distance(
        const vec3& p,
        const Box& B
    ) {
        bool inside = true;
        double result = 0.0;
        for(coord_index_t c = 0; c < 3; c++) {
            if(p[c] < B.xyz_min[c]) {
                inside = false;
                result += geo_sqr(p[c] - B.xyz_min[c]);
            } else if(p[c] > B.xyz_max[c]) {
                inside = false;
                result += geo_sqr(p[c] - B.xyz_max[c]);
            }
        }
        if(inside) {
            result = -inner_point_box_squared_distance(p, B);
        }
        return result;
    }

    /**
     * \brief Computes the squared distance between a point and the
     *  center of a box.
     * \param[in] p the point
     * \param[in] B the box
     * \return the squared distance between \p p and the center of \p B
     */
    double point_box_center_squared_distance(
        const vec3& p, const Box& B
    ) {
        double result = 0.0;
        for(coord_index_t c = 0; c < 3; ++c) {
            double d = p[c] - 0.5 * (B.xyz_min[c] + B.xyz_max[c]);
            result += geo_sqr(d);
        }
        return result;
    }

    /**
     * \brief Tests whether a mesh tetrahedron contains a given point
     * \param[in] M a const reference to the mesh
     * \param[in] t the index of the tetrahedron in \p M
     * \param[in] p a const reference to the point
     * \retval true if the tetrahedron \p t or its boundary contains
     *  the point \p p
     * \retval false otherwise
     */
    bool mesh_tet_contains_point(
        const Mesh& M, index_t t, const vec3& p
    ) {
        const vec3& p0 = Geom::mesh_vertex(M, M.cells.vertex(t,0));
        const vec3& p1 = Geom::mesh_vertex(M, M.cells.vertex(t,1));
        const vec3& p2 = Geom::mesh_vertex(M, M.cells.vertex(t,2));
        const vec3& p3 = Geom::mesh_vertex(M, M.cells.vertex(t,3));

        Sign s[4];
        s[0] = PCK::orient_3d(p, p1, p2, p3);
        s[1] = PCK::orient_3d(p0, p, p2, p3);
        s[2] = PCK::orient_3d(p0, p1, p, p3);
        s[3] = PCK::orient_3d(p0, p1, p2, p);

        return (
            (s[0] >= 0 && s[1] >= 0 && s[2] >= 0 && s[3] >= 0) ||
            (s[0] <= 0 && s[1] <= 0 && s[2] <= 0 && s[3] <= 0)
        );
    }


    /**
     * \brief Tests whether a mesh triangle contains a given point
     * \param[in] M a const reference to the mesh
     * \param[in] t the index of the triangle in \p M
     * \param[in] p a const reference to the point
     * \retval true if the triangle \p t or its boundary contains
     *  the point \p p
     * \retval false otherwise
     */
    bool mesh_triangle_contains_point(
        const Mesh& M, index_t t, const vec2& p
    ) {
        index_t i = M.facets.vertex(t,0);
        index_t j = M.facets.vertex(t,1);
        index_t k = M.facets.vertex(t,2);
        vec2 p0(M.vertices.point_ptr(i));
        vec2 p1(M.vertices.point_ptr(j));
        vec2 p2(M.vertices.point_ptr(k));

        Sign s[3];
        s[0] = PCK::orient_2d(p,  p1, p2);
        s[1] = PCK::orient_2d(p0, p,  p2);
        s[2] = PCK::orient_2d(p0, p1, p );

        return (
            (s[0] >= 0 && s[1] >= 0 && s[2] >= 0 ) ||
            (s[0] <= 0 && s[1] <= 0 && s[2] <= 0 )
        );
    }

    /**
     * \brief Computes the intersection between a ray and a triangle.
     * \param[in] O origin of the ray.
     * \param[in] D direction vector of the ray.
     * \param[in] A , B , C the three vertices of the triangle.
     * \param[out] t the intersection point is O + t D when it exists.
     * \param[out] u , v the intersection point is A + u (B-A) + v (C-A).
     *   when it exists.
     * \param[out] N the normal to the triangle.
     * \retval true if there is an intersection point
     * \retval false otherwise
     */
    bool ray_triangle_intersection(
        const vec3& O, const vec3& D,
        const vec3& A, const vec3& B, const vec3& C,
        double& t, double& u, double& v, vec3& N
    ) {
        // M\"oller and Trumbore,
        // Fast, Minimum Storage Ray-Triangle Intersection,
        // Journal of Graphics Tools, vol. 2,‎ 1997, p. 21–28
        // (with small adaptations: branchless, and reusing the normal vector)
        //
        // Let E1 = B-A; E2 = C-A, write ray eqn (1) and triangle eqn (2), then
        //  write equality between (1) and (2) at intersection point (3):
        //
        // (1) O + tD = A + uE1 + vE2
        // (2) uE1 + vE2 -tD = O-A
        //
        //                [u]
        // (3) [E1|E2|-D] [v] = O-A
        //                [t]
        //
        //  (where [E1|E2|-D] is the 3x3 matrix with E1,E2,-D as its columns)
        //
        //  Using Cramer's formula for the solution of:
        //
        //    [a11 a12 a13][x1]   [b1]
        //    [a12 a22 a23][x2] = [b2]
        //    [a31 a32 a33][x3]   [b3]
        //
        //  gives:
        //
        //        |b1 a12 a13|   |a11 a12 a13|
        //   x1 = |b2 a22 a23| / |a21 a22 a23|
        //        |b3 a32 a33|   |a31 a32 a33|
        //
        //        |a11 b1 a13|   |a11 a12 a13|
        //   x2 = |a21 b2 a23| / |a21 a22 a23|
        //        |a31 b3 a33|   |a31 a32 a33|
        //
        //        |a11 a12 b1|   |a11 a12 a13|
        //   x3 = |a21 a22 b2| / |a21 a22 a23|
        //        |a31 a32 b3|   |a31 a32 a33|
        //
        // Now we get:
        //
        //   u = (O-A,E2,-D) / (E1,E2,-D)
        //   v = (E1,O-A,-D) / (E1,E2,-D)
        //   t = (E1,E2,O-A) / (E1,E2,-D)
        //
        // where (A,B,C) denotes the determinant of the 3x3 matrix
        //  with A,B,C as its column vectors.
        //
        // Now we use the following identities:
        //   (A,B,C) = dot(A,cross(B,C))  (develop the det w.r.t. first column)
        //   (B,A,C) = -(A,B,C)           (swapping two cols changes sign)
        //   (B,C,A) =  (A,B,C)           (circular perm does not change sign)
        //
        // Now we get:
        //
        // u = -(E2,O-A,D)  / (D,E1,E2)
        // v =  (E1,O-A,D)  / (D,E1,E2)
        // t = -(O-A,E1,E2) / (D,E1,E2)
        //
        // Using N=cross(E1,E2); AO = O-A; DAO = cross(D,AO)
        vec3 E1(B-A);
        vec3 E2(C-A);
        N = cross(E1,E2);
        double det = -dot(D,N);
        double invdet = 1.0/det;
        vec3 AO = O - A;
        vec3 DAO = cross(AO,D);
        u =  dot(E2,DAO) * invdet;
        v = -dot(E1,DAO) * invdet;
        t =  dot(AO,N)   * invdet;
        return (
            (fabs(det) >= 1e-20) &&
            (t >= 0.0) &&
            (u >= 0.0) &&
            (v >= 0.0) &&
            ((u+v) <= 1.0)
        );
    }

    inline double max3(double x1, double x2, double x3) {
        return std::max(x1,std::max(x2,x3));
    }

    inline double min3(double x1, double x2, double x3) {
        return std::min(x1,std::min(x2,x3));
    }


    // https://tavianator.com/fast-branchless-raybounding-box-intersections/
    // https://tavianator.com/fast-branchless-raybounding-box-intersections-part-2-nans/
    // http://www.flipcode.com/archives/SSE_RayBox_Intersection_Test.shtml
    // http://psgraphics.blogspot.com/2016/02/ray-box-intersection-and-fmin.html

    /**
     * \brief Tests whether a segment intersects a box.
     * \param[in] q1 the first extremity of the segment.
     * \param[in] dirinv precomputed 1/(q2.x-q1.x), 1/(q2.y-q1.y), 1/(q2.z-q1.z)
     *   where q2 denotes the second extremity of the segment.
     * \param[in] box the box.
     * \param[in] T the maximum acceptable value for the intersection parameter.
     *    Can be used to early-prune boxes while traversing the tree.
     * \retval true if [q1,q2] intersects the box.
     * \retval false otherwise.
     */
    bool ray_box_intersection(
        const vec3& q1, const vec3& dirinv, const Box& box, double T = 1.0
    ) {
        // This version: slab method.
        // Step 1: compute
        // (tx1, tx2) : parameters of intersection with slab {xmin <= x <= xmax}
        // (ty1, ty2) : parameters of intersection with slab {ymin <= y <= ymax}
        // (tz1, tz2) : parameters of intersection with slab {zmin <= z <= zmax}
        //   (note: they are unordered, it is possible that tx1 > tx2)
        // This defines three intervals:
        //  Ix = [ min(tx1,tx2) ... max(tx1,tx2) ]
        //  Iy = [ min(ty1,ty2) ... max(ty1,ty2) ]
        //  Iz = [ min(tz1,tz2) ... max(tz1,tz2) ]
        // The intersection between [q1,q2] and the slab {xmin <= x <= xmax} is
        //  the set of points {q1 + t(q2-q1)} where t in Ix

        // Q: what does it do if one of the fracs is zero ?
        //   normally the tests with inf do what they should
        //   (to be tested)

        double tx1 = dirinv.x*(box.xyz_min[0] - q1.x);
        double tx2 = dirinv.x*(box.xyz_max[0] - q1.x);

        double ty1 = dirinv.y*(box.xyz_min[1] - q1.y);
        double ty2 = dirinv.y*(box.xyz_max[1] - q1.y);

        double tz1 = dirinv.z*(box.xyz_min[2] - q1.z);
        double tz2 = dirinv.z*(box.xyz_max[2] - q1.z);

        // now compute the intersection of the three intervals
        //      Ix /\ Iy /\ Iz
        //   this gives us the range of t that corresponds to points in the
        //   box (because the box is the intersection of the 3 slabs)
        // it starts at the maximum of the left bounds of the 3 intervals
        // it stops at the minimum of the right bounds of the 3 intervals

        double tmin =
            max3(std::min(tx1,tx2), std::min(ty1,ty2), std::min(tz1,tz2));

        double tmax =
            min3(std::max(tx1,tx2), std::max(ty1,ty2), std::max(tz1,tz2));

        // There is no intersection if the interval is empty (tmin > tmax)
        // or if the interval is outside [0,1]
        // Note: the test is tmin <= tmax, because a bbox can be infinitely
        // thin (for instance, the bbox of a triangle orthogonal to one
        // of the axes).

        return (tmax >= 0.0) && (tmin <= tmax) && (tmin <= T);
    }


}

/****************************************************************************/

namespace GEO {

    MeshFacetsAABB::MeshFacetsAABB() {
    }

    MeshFacetsAABB::MeshFacetsAABB(
        Mesh& M, bool reorder
    ) {
        initialize(M, reorder);
    }

    void MeshFacetsAABB::initialize(
        Mesh& M, bool reorder
    ) {
        mesh_ = &M;
        if(mesh_->facets.nb() == 0) {
            return;
        }
        if(reorder) {
            mesh_reorder(*mesh_, MESH_ORDER_MORTON);
        }
        AABB::initialize(
            mesh_->facets.nb(),
            [this](Box& B, index_t f) {
                // Get facet bbox
                index_t c = mesh_->facets.corners_begin(f);
                const double* p = mesh_->vertices.point_ptr(
                    mesh_->facet_corners.vertex(c)
                );
                for(coord_index_t coord = 0; coord < 3; ++coord) {
                    B.xyz_min[coord] = p[coord];
                    B.xyz_max[coord] = p[coord];
                }
                for(++c; c < mesh_->facets.corners_end(f); ++c) {
                    p = mesh_->vertices.point_ptr(
                        mesh_->facet_corners.vertex(c)
                    );
                    for(coord_index_t coord = 0; coord < 3; ++coord) {
                        B.xyz_min[coord] = std::min(B.xyz_min[coord], p[coord]);
                        B.xyz_max[coord] = std::max(B.xyz_max[coord], p[coord]);
                    }
                }
            }
        );
    }

    void MeshFacetsAABB::get_nearest_facet_hint(
        const vec3& p,
        index_t& nearest_f, vec3& nearest_point, double& sq_dist
    ) const {

        // Find a good initial value for nearest_f by traversing
        // the boxes and selecting the child such that the center
        // of its bounding box is nearer to the query point.
        // For a large mesh (20M facets) this gains up to 10%
        // performance as compared to picking nearest_f randomly.
        index_t b = 0;
        index_t e = mesh_->facets.nb();
        index_t n = 1;
        while(e != b + 1) {
            index_t m = b + (e - b) / 2;
            index_t childl = 2 * n;
            index_t childr = 2 * n + 1;
            if(
                point_box_center_squared_distance(p, bboxes_[childl]) <
                point_box_center_squared_distance(p, bboxes_[childr])
            ) {
                e = m;
                n = childl;
            } else {
                b = m;
                n = childr;
            }
        }
        nearest_f = b;

        index_t v = mesh_->facet_corners.vertex(
            mesh_->facets.corners_begin(nearest_f)
        );
        nearest_point = Geom::mesh_vertex(*mesh_, v);
        sq_dist = Geom::distance2(p, nearest_point);
    }

    void MeshFacetsAABB::nearest_facet_recursive(
        const vec3& p,
        index_t& nearest_f, vec3& nearest_point, double& sq_dist,
        index_t n, index_t b, index_t e
    ) const {
        geo_debug_assert(e > b);

        // If node is a leaf: compute point-facet distance
        // and replace current if nearer
        if(b + 1 == e) {
            vec3 cur_nearest_point;
            double cur_sq_dist;
            get_point_facet_nearest_point(
                *mesh_, p, b, cur_nearest_point, cur_sq_dist
            );
            if(cur_sq_dist < sq_dist) {
                nearest_f = b;
                nearest_point = cur_nearest_point;
                sq_dist = cur_sq_dist;
            }
            return;
        }
        index_t m = b + (e - b) / 2;
        index_t childl = 2 * n;
        index_t childr = 2 * n + 1;

        double dl = point_box_signed_squared_distance(p, bboxes_[childl]);
        double dr = point_box_signed_squared_distance(p, bboxes_[childr]);

        // Traverse the "nearest" child first, so that it has more chances
        // to prune the traversal of the other child.
        if(dl < dr) {
            if(dl < sq_dist) {
                nearest_facet_recursive(
                    p,
                    nearest_f, nearest_point, sq_dist,
                    childl, b, m
                );
            }
            if(dr < sq_dist) {
                nearest_facet_recursive(
                    p,
                    nearest_f, nearest_point, sq_dist,
                    childr, m, e
                );
            }
        } else {
            if(dr < sq_dist) {
                nearest_facet_recursive(
                    p,
                    nearest_f, nearest_point, sq_dist,
                    childr, m, e
                );
            }
            if(dl < sq_dist) {
                nearest_facet_recursive(
                    p,
                    nearest_f, nearest_point, sq_dist,
                    childl, b, m
                );
            }
        }
    }

    bool MeshFacetsAABB::ray_intersection(
        const Ray& R, double tmax, index_t ignore_f
    ) const {
        vec3 dirinv(
            1.0/R.direction.x,
            1.0/R.direction.y,
            1.0/R.direction.z
        );
        return ray_intersection_recursive(
            R, dirinv, tmax, ignore_f, 1, 0, mesh_->facets.nb()
        );
    }

    bool MeshFacetsAABB::ray_nearest_intersection(
        const Ray& R, Intersection& I
    ) const {
        index_t f = I.f;
        vec3 dirinv(
            1.0/R.direction.x,
            1.0/R.direction.y,
            1.0/R.direction.z
        );
        ray_nearest_intersection_recursive(
            R, dirinv, I, f, 1, 0, mesh_->facets.nb(), 0
        );
        if(I.f != f) {
            I.p = R.origin + I.t * R.direction;
            return true;
        }
        return false;
    }

    void MeshFacetsAABB::ray_all_intersections(
        const Ray& R,
        std::function<void(const Intersection&)> action
    ) const {
        vec3 dirinv(
            1.0/R.direction.x,
            1.0/R.direction.y,
            1.0/R.direction.z
        );
        ray_all_intersections_recursive(
            R, dirinv, action,
            1, 0, mesh_->facets.nb()
        );
    }

    bool MeshFacetsAABB::ray_intersection_recursive(
        const Ray& R, const vec3& dirinv, double tmax, index_t ignore_f,
        index_t n, index_t b, index_t e
    ) const {
        if(!ray_box_intersection(R.origin, dirinv, bboxes_[n], tmax)) {
            return false;
        }
        if(b + 1 == e) {
            index_t f = b;
            if(f == ignore_f) {
                return false;
            }
            index_t c = mesh_->facets.corners_begin(f);
            const vec3& p1 = Geom::mesh_vertex(
                *mesh_, mesh_->facet_corners.vertex(c)
            );
            ++c;
            while(c+1 != mesh_->facets.corners_end(f)) {
                const vec3& p2 = Geom::mesh_vertex(
                    *mesh_, mesh_->facet_corners.vertex(c)
                );
                const vec3& p3 = Geom::mesh_vertex(
                    *mesh_, mesh_->facet_corners.vertex(c+1)
                );
                vec3 N;
                double t,u,v;
                if(
                    ray_triangle_intersection(
                        R.origin, R.direction, p1, p2, p3, t, u, v, N
                    ) && t < tmax
                ) {
                    return true;
                }
                ++c;
            }
            return false;
        }
        index_t m = b + (e - b) / 2;
        index_t childl = 2 * n;
        index_t childr = 2 * n + 1;
        return (
            ray_intersection_recursive(R,dirinv,tmax,ignore_f, childl,b,m) ||
            ray_intersection_recursive(R,dirinv,tmax,ignore_f, childr,m,e)
        );
    }

    void MeshFacetsAABB::ray_nearest_intersection_recursive(
        const Ray& R, const vec3& dirinv, Intersection& I, index_t ignore_f,
        index_t n, index_t b, index_t e, index_t coord
    ) const {
        if(!ray_box_intersection(R.origin, dirinv, bboxes_[n], I.t)) {
            return;
        }
        if(b + 1 == e) {
            index_t f = b;
            if(f == ignore_f) {
                return;
            }
            index_t c = mesh_->facets.corners_begin(f);
            index_t v1 = mesh_->facet_corners.vertex(c);
            const vec3& p1 = Geom::mesh_vertex(*mesh_, v1);
            ++c;
            while(c+1 != mesh_->facets.corners_end(f)) {
                index_t v2 = mesh_->facet_corners.vertex(c);
                index_t v3 = mesh_->facet_corners.vertex(c+1);
                const vec3& p2 = Geom::mesh_vertex(*mesh_, v2);
                const vec3& p3 = Geom::mesh_vertex(*mesh_, v3);
                vec3 N;
                double t,u,v;
                if(ray_triangle_intersection(
                       R.origin,R.direction,p1,p2,p3,t,u,v,N
                   ) && t<I.t
                  ) {
                    I.t = t;
                    I.u = u;
                    I.v = v;
                    I.N = N;
                    I.i = v1;
                    I.j = v2;
                    I.k = v3;
                    I.f = f;
                }
                ++c;
            }
            return;
        }
        index_t m = b + (e - b) / 2;
        index_t childl = 2 * n;
        index_t childr = 2 * n + 1;
        if(dirinv[coord] < 0.0) {
            ray_nearest_intersection_recursive(
                R, dirinv, I, ignore_f, childr, m, e, (coord+1)%3
            );
            ray_nearest_intersection_recursive(
                R, dirinv, I, ignore_f, childl, b, m, (coord+1)%3
            );
        } else {
            ray_nearest_intersection_recursive(
                R, dirinv, I, ignore_f, childl, b, m, (coord+1)%3
            );
            ray_nearest_intersection_recursive(
                R, dirinv, I, ignore_f, childr, m, e, (coord+1)%3
            );
        }
    }


    void MeshFacetsAABB::ray_all_intersections_recursive(
        const Ray& R, const vec3& dirinv,
        std::function<void(const Intersection&)> action,
        index_t n, index_t b, index_t e
    ) const {
        Intersection I;
        if(!ray_box_intersection(R.origin, dirinv, bboxes_[n], I.t)) {
            return;
        }
        if(b + 1 == e) {
            index_t f = b;
            index_t c = mesh_->facets.corners_begin(f);
            index_t v1 = mesh_->facet_corners.vertex(c);
            const vec3& p1 = Geom::mesh_vertex(*mesh_, v1);
            ++c;
            while(c+1 != mesh_->facets.corners_end(f)) {
                index_t v2 = mesh_->facet_corners.vertex(c);
                index_t v3 = mesh_->facet_corners.vertex(c+1);
                const vec3& p2 = Geom::mesh_vertex(*mesh_, v2);
                const vec3& p3 = Geom::mesh_vertex(*mesh_, v3);
                vec3 N;
                if(ray_triangle_intersection(
                       R.origin,R.direction,p1,p2,p3,I.t,I.u,I.v,I.N
                   )
                  ) {
                    I.i = v1;
                    I.j = v2;
                    I.k = v3;
                    I.f = f;
                    action(I);
                }
                ++c;
            }
            return;
        }
        index_t m = b + (e - b) / 2;
        index_t childl = 2 * n;
        index_t childr = 2 * n + 1;
        ray_all_intersections_recursive(
            R, dirinv, action, childr, m, e
        );
        ray_all_intersections_recursive(
            R, dirinv, action, childl, b, m
        );
    }


/****************************************************************************/

    MeshCellsAABB::MeshCellsAABB() {
    }

    MeshCellsAABB::MeshCellsAABB(Mesh& M, bool reorder) {
        initialize(M, reorder);
    }

    void MeshCellsAABB::initialize(Mesh& M, bool reorder) {
        mesh_ = &M;
        if(reorder) {
            mesh_reorder(*mesh_, MESH_ORDER_MORTON);
        }
        if(mesh_->cells.are_simplices()) {
            AABB::initialize(
                mesh_->cells.nb(),
                [this](Box& B, index_t t) {
                    // Get tet bbox
                    const double* p = mesh_->vertices.point_ptr(
                        mesh_->cells.vertex(t,0)
                    );
                    for(coord_index_t coord = 0; coord < 3; ++coord) {
                        B.xyz_min[coord] = p[coord];
                        B.xyz_max[coord] = p[coord];
                    }
                    for(index_t lv=1; lv<4; ++lv) {
                        p = mesh_->vertices.point_ptr(
                            mesh_->cells.vertex(t,lv)
                        );
                        for(coord_index_t coord = 0; coord < 3; ++coord) {
                            B.xyz_min[coord] = std::min(
                                B.xyz_min[coord], p[coord]
                            );
                            B.xyz_max[coord] = std::max(
                                B.xyz_max[coord], p[coord]
                            );
                        }
                    }
                }
            );
        } else {
            AABB::initialize(
                mesh_->cells.nb(),
                [this](Box& B, index_t c) {
                    // Get cell bbox
                    const double* p = mesh_->vertices.point_ptr(
                        mesh_->cells.vertex(c,0)
                    );
                    for(coord_index_t coord = 0; coord < 3; ++coord) {
                        B.xyz_min[coord] = p[coord];
                        B.xyz_max[coord] = p[coord];
                    }
                    for(index_t lv=1; lv<mesh_->cells.nb_vertices(c); ++lv) {
                        p = mesh_->vertices.point_ptr(
                            mesh_->cells.vertex(c,lv)
                        );
                        for(coord_index_t coord = 0; coord < 3; ++coord) {
                            B.xyz_min[coord] = std::min(
                                B.xyz_min[coord], p[coord]
                            );
                            B.xyz_max[coord] = std::max(
                                B.xyz_max[coord], p[coord]
                            );
                        }
                    }
                }
            );
        }
    }

    index_t MeshCellsAABB::containing_tet_recursive(
        const vec3& p,
        index_t n, index_t b, index_t e
    ) const {

        if(!bboxes_[n].contains(p)) {
            return NO_TET;
        }

        if(e==b+1) {
            if(mesh_tet_contains_point(*mesh_, b, p)) {
                return b;
            } else {
                return NO_TET;
            }
        }

        index_t m = b + (e - b) / 2;
        index_t childl = 2 * n;
        index_t childr = 2 * n + 1;

        index_t result = containing_tet_recursive(
            p, childl, b, m
        );
        if(result == NO_TET) {
            result = containing_tet_recursive(p, childr, m, e);
        }
        return result;
    }

/****************************************************************************/

    MeshFacetsAABB2d::MeshFacetsAABB2d() {
    }

    MeshFacetsAABB2d::MeshFacetsAABB2d(Mesh& M, bool reorder) {
        initialize(M, reorder);
    }

    void MeshFacetsAABB2d::initialize(Mesh& M, bool reorder) {
        bool was_2d = (M.vertices.dimension() == 2);
        if(was_2d) {
            // It is a bit stupid, spatial sort is just implemented
            // in 3D for now, so if input mesh was 2D, we temporarily
            // create z=0 coordinates for all vertices.
            M.vertices.set_dimension(3);
        }
        mesh_ = &M;
        if(reorder) {
            mesh_reorder(*mesh_, MESH_ORDER_MORTON);
        }
        if(mesh_->facets.are_simplices()) {
            AABB::initialize(
                mesh_->facets.nb(),
                [this](Box2d& B, index_t t) {
                    // Get triangle bbox
                    const double* p = mesh_->vertices.point_ptr(
                        mesh_->facets.vertex(t,0)
                    );
                    for(coord_index_t coord = 0; coord < 2; ++coord) {
                        B.xy_min[coord] = p[coord];
                        B.xy_max[coord] = p[coord];
                    }
                    for(index_t lv=1; lv<3; ++lv) {
                        p = mesh_->vertices.point_ptr(
                            mesh_->facets.vertex(t,lv)
                        );
                        for(coord_index_t coord = 0; coord < 2; ++coord) {
                            B.xy_min[coord] = std::min(
                                B.xy_min[coord], p[coord]
                            );
                            B.xy_max[coord] = std::max(
                                B.xy_max[coord], p[coord]
                            );
                        }
                    }
                }
            );
        } else {
            AABB::initialize(
                mesh_->facets.nb(),
                [this](Box2d& B, index_t f) {
                    // Get facet bbox
                    const double* p = mesh_->vertices.point_ptr(
                        mesh_->facets.vertex(f,0)
                    );
                    for(coord_index_t coord = 0; coord < 2; ++coord) {
                        B.xy_min[coord] = p[coord];
                        B.xy_max[coord] = p[coord];
                    }
                    for(index_t lv=1; lv<mesh_->facets.nb_vertices(f); ++lv) {
                        p = mesh_->vertices.point_ptr(
                            mesh_->cells.vertex(f,lv)
                        );
                        for(coord_index_t coord = 0; coord < 2; ++coord) {
                            B.xy_min[coord] = std::min(
                                B.xy_min[coord], p[coord]
                            );
                            B.xy_max[coord] = std::max(
                                B.xy_max[coord], p[coord]
                            );
                        }
                    }
                }
            );
        }
        if(was_2d) {
            M.vertices.set_dimension(2);
        }
    }

    index_t MeshFacetsAABB2d::containing_triangle_recursive(
        const vec2& p,
        index_t n, index_t b, index_t e
    ) const {

        if(!bboxes_[n].contains(p)) {
            return NO_TRIANGLE;
        }

        if(e==b+1) {
            if(mesh_triangle_contains_point(*mesh_, b, p)) {
                return b;
            } else {
                return NO_TRIANGLE;
            }
        }

        index_t m = b + (e - b) / 2;
        index_t childl = 2 * n;
        index_t childr = 2 * n + 1;

        index_t result = containing_triangle_recursive(
            p, childl, b, m
        );
        if(result == NO_TRIANGLE) {
            result = containing_triangle_recursive(p, childr, m, e);
        }
        return result;
    }

/****************************************************************************/

}
