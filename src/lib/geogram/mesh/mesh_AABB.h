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

#ifndef GEOGRAM_MESH_MESH_AABB
#define GEOGRAM_MESH_MESH_AABB

/**
 * \file mesh_AABB.h
 * \brief Axis Aligned Bounding Box trees for accelerating
 *  geometric queries that operate on a Mesh.
 */

#include <geogram/basic/common.h>
#include <geogram/mesh/mesh.h>
#include <geogram/basic/geometry.h>

namespace GEO {


    /**
     * \brief Base class for Axis Aligned Bounding Box trees.
     * \tparam BOX the box class (Box2d or Box3d).
     */
    template <class BOX> class AABB {

    protected:

        /**
         * \brief Initializes this AABB.
         * \param[in] nb number of items.
         * \param[in] get_bbox a function(Box&, index_t) that computes the Box
         *  associated with a given index, in [0..nb-1].
         */
        void initialize(index_t nb, std::function<void(BOX&, index_t)> get_bbox) {
            nb_ = nb;
            bboxes_.resize(max_node_index(1, 0, nb) + 1);
            // +1 because size == max_index + 1 !!!
            init_bboxes_recursive(1, 0, nb_, get_bbox);
        }


        /**
         * \brief Computes all the elements that have a bbox that
         *  intersects a given bbox in a sub-tree of the AABB tree.
         *
         * Note that the tree structure is completely implicit,
         *  therefore the bounds of the (continuous) facet indices
         *  sequences that correspond to the elements contained
         *  in the two nodes are sent as well as the node indices.
         *
         * \param[in] action a function that takes as argument
         *  an index_t (cell index) invoked for all cells that
         *  has a bounding box that overlaps \p box.
         * \param[in] box the query box
         * \param[in] node index of the first node of the AABB tree
         * \param[in] b index of the first facet in \p node
         * \param[in] e one position past the index of the last
         *  facet in \p node
         */
        void bbox_intersect_recursive(
            std::function<void(index_t)> action, const BOX& box,
            index_t node, index_t b, index_t e
        ) const {
            geo_debug_assert(e != b);

            // Prune sub-tree that does not have intersection
            if(!bboxes_overlap(box, bboxes_[node])) {
                return;
            }

            // Leaf case
            if(e == b+1) {
                action(b);
                return;
            }

            // Recursion
            index_t m = b + (e - b) / 2;
            index_t node_l = 2 * node;
            index_t node_r = 2 * node + 1;

            bbox_intersect_recursive(action, box, node_l, b, m);
            bbox_intersect_recursive(action, box, node_r, m, e);
        }

        /**
         * \brief Computes all the pairs of intersecting elements
         *  for two sub-trees of the AABB tree.
         *
         * Note that the tree structure is completely implicit,
         *  therefore the bounds of the (continuous) facet indices
         *  sequences that correspond to the elementss contained
         *  in the two nodes are sent as well as the node indices.
         *
         * \param[in] action a function taking as arguments two
         *  index_t's, invoked of all pairs of elements that have
         *  overlapping bounding boxes.
         * \param[in] node1 index of the first node of the AABB tree
         * \param[in] b1 index of the first facet in \p node1
         * \param[in] e1 one position past the index of the last
         *  facet in \p node1
         * \param[in] node2 index of the second node of the AABB tree
         * \param[in] b2 index of the first facet in \p node2
         * \param[in] e2 one position past the index of the second
         *  facet in \p node2
         */
        void self_intersect_recursive(
            std::function<void(index_t,index_t)> action,
            index_t node1, index_t b1, index_t e1,
            index_t node2, index_t b2, index_t e2
        ) const {
            geo_debug_assert(e1 != b1);
            geo_debug_assert(e2 != b2);

            // Since we are intersecting the AABBTree with *itself*,
            // we can prune half of the cases by skipping the test
            // whenever node2's facet index interval is greated than
            // node1's facet index interval.
            if(e2 <= b1) {
                return;
            }

            // The acceleration is here:
            if(!bboxes_overlap(bboxes_[node1], bboxes_[node2])) {
                return;
            }

            // Simple case: leaf - leaf intersection.
            if(b1 + 1 == e1 && b2 + 1 == e2) {
                action(b1, b2);
                return;
            }

            // If node2 has more elements than node1, then
            //   intersect node2's two children with node1
            // else
            //   intersect node1's two children with node2
            if(e2 - b2 > e1 - b1) {
                index_t m2 = b2 + (e2 - b2) / 2;
                index_t node2_l = 2 * node2;
                index_t node2_r = 2 * node2 + 1;
                self_intersect_recursive(action, node1, b1, e1, node2_l, b2, m2);
                self_intersect_recursive(action, node1, b1, e1, node2_r, m2, e2);
            } else {
                index_t m1 = b1 + (e1 - b1) / 2;
                index_t node1_l = 2 * node1;
                index_t node1_r = 2 * node1 + 1;
                self_intersect_recursive(action, node1_l, b1, m1, node2, b2, e2);
                self_intersect_recursive(action, node1_r, m1, e1, node2, b2, e2);
            }
        }


        /**
         * \brief Computes all the pairs of intersecting elements
         *  for two sub-trees of two AABB trees.
         *
         * Note that the tree structure is completely implicit,
         *  therefore the bounds of the (continuous) facet indices
         *  sequences that correspond to the elementss contained
         *  in the two nodes are sent as well as the node indices.
         *
         * \param[in] action a function taking as arguments two
         *  index_t's, invoked of all pairs of elements that have
         *  overlapping bounding boxes.
         * \param[in] node1 index of the first node of the AABB tree
         * \param[in] b1 index of the first facet in \p node1
         * \param[in] e1 one position past the index of the last
         *  facet in \p node1
         * \param[in] other the second AABB tree
         * \param[in] node2 index of the second node of the second AABB tree
         * \param[in] b2 index of the first facet in \p node2
         * \param[in] e2 one position past the index of the second
         *  facet in \p node2
         */
        void other_intersect_recursive(
            std::function<void(index_t,index_t)> action,
            index_t node1, index_t b1, index_t e1,
            const AABB<BOX>* other,
            index_t node2, index_t b2, index_t e2
        ) const {
            geo_debug_assert(e1 != b1);
            geo_debug_assert(e2 != b2);

            // The acceleration is here:
            if(!bboxes_overlap(bboxes_[node1], other->bboxes_[node2])) {
                return;
            }

            // Simple case: leaf - leaf intersection.
            if(b1 + 1 == e1 && b2 + 1 == e2) {
                action(b1, b2);
                return;
            }

            // If node2 has more elements than node1, then
            //   intersect node2's two children with node1
            // else
            //   intersect node1's two children with node2
            if(e2 - b2 > e1 - b1) {
                index_t m2 = b2 + (e2 - b2) / 2;
                index_t node2_l = 2 * node2;
                index_t node2_r = 2 * node2 + 1;
                other_intersect_recursive(
                    action, node1, b1, e1, other, node2_l, b2, m2
                );
                other_intersect_recursive(
                    action, node1, b1, e1, other, node2_r, m2, e2
                );
            } else {
                index_t m1 = b1 + (e1 - b1) / 2;
                index_t node1_l = 2 * node1;
                index_t node1_r = 2 * node1 + 1;
                other_intersect_recursive(
                    action, node1_l, b1, m1, other, node2, b2, e2
                );
                other_intersect_recursive(
                    action, node1_r, m1, e1, other, node2, b2, e2
                );
            }
        }


        /**
         * \brief Computes the maximum node index in a subtree
         * \param[in] node_index node index of the root of the subtree
         * \param[in] b first facet index in the subtree
         * \param[in] e one position past the last facet index in the subtree
         * \return the maximum node index in the subtree rooted at \p node_index
         */
        static index_t max_node_index(index_t node_index, index_t b, index_t e) {
            geo_debug_assert(e > b);
            if(b + 1 == e) {
                return node_index;
            }
            index_t m = b + (e - b) / 2;
            index_t childl = 2 * node_index;
            index_t childr = 2 * node_index + 1;
            return std::max(
                max_node_index(childl, b, m),
                max_node_index(childr, m, e)
            );
        }

        /**
         * \brief Computes the hierarchy of bounding boxes recursively.
         * \details This function is generic and can be used to compute
         *  a bbox hierarchy of arbitrary elements.
         * \param[in] node_index the index of the root of the subtree
         * \param[in] b first element index in the subtree
         * \param[in] e one position past the last element index in the subtree
         * \param[in] get_bbox a function that takes a Box3d& and an index_t,
         *  that computes the bbox of an element.
         */
        void init_bboxes_recursive(
            index_t node_index,
            index_t b, index_t e,
            std::function<void(BOX&, index_t)> get_bbox
        ) {
            geo_debug_assert(node_index < bboxes_.size());
            geo_debug_assert(b != e);
            if(b + 1 == e) {
                get_bbox(bboxes_[node_index], b);
                return;
            }
            index_t m = b + (e - b) / 2;
            index_t childl = 2 * node_index;
            index_t childr = 2 * node_index + 1;
            geo_debug_assert(childl < bboxes_.size());
            geo_debug_assert(childr < bboxes_.size());
            init_bboxes_recursive(childl, b, m, get_bbox);
            init_bboxes_recursive(childr, m, e, get_bbox);
            geo_debug_assert(childl < bboxes_.size());
            geo_debug_assert(childr < bboxes_.size());
            bbox_union(bboxes_[node_index], bboxes_[childl], bboxes_[childr]);
        }

    protected:
        index_t nb_;
        vector<BOX> bboxes_;
    };

    typedef AABB<Box2d> AABB2d;
    typedef AABB<Box3d> AABB3d;

    /**************************************************************/

    /**
     * \brief Base class for Axis Aligned Bounding Box trees
     *  of mesh elements with 2d boxes.
     */
    class GEOGRAM_API MeshAABB2d : public AABB2d {
    public:
        /**
         * \brief MeshAABB2d constructor.
         */
        MeshAABB2d() : mesh_(nullptr) {
        }

        /**
         * \brief Gets the mesh.
         * \return a const reference to the mesh.
         */
        const Mesh* mesh() const {
            return mesh_;
        }

    protected:
        Mesh* mesh_;
    };

    /**************************************************************/

    /**
     * \brief Base class for Axis Aligned Bounding Box trees
     *  of mesh elements with 3d boxes.
     */
    class GEOGRAM_API MeshAABB3d : public AABB3d {
    public:
        /**
         * \brief MeshAABB3d constructor.
         */
        MeshAABB3d() : mesh_(nullptr) {
        }

        /**
         * \brief Gets the mesh.
         * \return a const reference to the mesh.
         */
        const Mesh* mesh() const {
            return mesh_;
        }

    protected:
        Mesh* mesh_;
    };

    /**************************************************************/

    /**
     * \brief Axis Aligned Bounding Box tree of mesh facets in 3D.
     * \details Used to quickly compute facet intersection and
     *  to locate the nearest facet from 3d query points.
     */
    class GEOGRAM_API MeshFacetsAABB : public MeshAABB3d {
    public:

        /**
         * \brief Stores all the information related with a ray-facet
         *   intersection.
         */
        struct Intersection {
            Intersection() :
                t(Numeric::max_float64()),
                f(index_t(-1)),
                i(index_t(-1)), j(index_t(-1)), k(index_t(-1))
                {
                }
            vec3 p;        /**< the intersection. */
            double t;      /**< the parameter along the intersected ray. */
            index_t f;     /**< the intersected facet. */
            vec3 N;        /**< the normal vector at the intersection. */
            index_t i,j,k; /**< the vertices of the intersected triangle. */
            double u,v;    /**< the barycentric coordinates in the triangle. */
        };


        /**
         * \brief MeshFacetsAABB constructor.
         * \details Creates an uninitialized MeshFacetsAABB.
         */
        MeshFacetsAABB();

        /**
         * \brief Initializes the Axis Aligned Bounding Boxes tree.
         * \param[in] M the input mesh. It can be modified,
         *  and will be triangulated (if
         *  not already a triangular mesh). The facets are
         *  re-ordered (using Morton's order, see mesh_reorder()).
         * \param[in] reorder if not set, Morton re-ordering is
         *  skipped (but it means that mesh_reorder() was previously
         *  called else the algorithm will be pretty unefficient).
         */
        void initialize(Mesh& M, bool reorder = true);


        /**
         * \brief Creates the Axis Aligned Bounding Boxes tree.
         * \param[in] M the input mesh. It can be modified,
         *  and will be triangulated (if
         *  not already a triangular mesh). The facets are
         *  re-ordered (using Morton's order, see mesh_reorder()).
         * \param[in] reorder if not set, Morton re-ordering is
         *  skipped (but it means that mesh_reorder() was previously
         *  called else the algorithm will be pretty unefficient).
         */
        MeshFacetsAABB(Mesh& M, bool reorder = true);

        /**
         * \brief Computes all the pairs of intersecting facets.
         * \param[in] action a function that takes two index_t's
         *  and that is invoked of all pairs of facets that have overlapping
         *  bounding boxes. triangles_intersections() needs to be
         *  called to detect the actual intersections.
         */
        void compute_facet_bbox_intersections(
            std::function<void(index_t, index_t)> action
        ) const {
            self_intersect_recursive(
                action,
                1, 0, mesh_->facets.nb(),
                1, 0, mesh_->facets.nb()
            );
        }

        /**
         * \brief Computes all the intersections between a given
         *  box and the bounding boxes of all the facets.
         * \param[in] action a function that takes an index_t that is
         *  invoked for all facets that have a bounding
         *  box that intersects \p box_in.
         */
        void compute_bbox_facet_bbox_intersections(
            const Box& box_in, std::function<void(index_t)> action
        ) const {
            bbox_intersect_recursive(
                action, box_in, 1, 0, mesh_->facets.nb()
            );
        }

        /**
         * \brief Finds the nearest facet from an arbitrary 3d query point.
         * \param[in] p query point
         * \param[out] nearest_point nearest point on the surface
         * \param[out] sq_dist squared distance between p and the surface.
         * \return the index of the facet nearest to point p.
         */
        index_t nearest_facet(
            const vec3& p, vec3& nearest_point, double& sq_dist
        ) const {
            index_t nearest_facet;
            get_nearest_facet_hint(p, nearest_facet, nearest_point, sq_dist);
            nearest_facet_recursive(
                p,
                nearest_facet, nearest_point, sq_dist,
                1, 0, mesh_->facets.nb()
            );
            return nearest_facet;
        }

        /**
         * \brief Finds the nearest facet from an arbitrary 3d query point.
         * \param[in] p query point
         * \return the index of the facet nearest to point p.
         */
        index_t nearest_facet(const vec3& p) const {
            vec3 nearest_point;
            double sq_dist;
            return nearest_facet(p, nearest_point, sq_dist);
        }

        /**
         * \brief Computes the nearest point and nearest facet from
         * a query point, using user-specified hint.
         *
         * \details The hint is specified as reasonable initial values of
         * (nearest_facet, nearest_point, sq_dist). If multiple queries
         * are done on a set of points that has spatial locality,
         * the hint can be the result of the previous call.
         *
         * \param[in] p query point
         * \param[in,out] nearest_facet the nearest facet so far,
         *   or NO_FACET if not known yet
         * \param[in,out] nearest_point a point in nearest_facet
         * \param[in,out] sq_dist squared distance between p and
         *    nearest_point
         * \note On entry, \p sq_dist needs to be equal to the squared
         *   distance between \p p and \p nearest_point (it is easy to
         *   forget to update it when calling it within a loop).
         */
        void nearest_facet_with_hint(
            const vec3& p,
            index_t& nearest_facet, vec3& nearest_point, double& sq_dist
        ) const {
            if(nearest_facet == NO_FACET) {
                get_nearest_facet_hint(
                    p, nearest_facet, nearest_point, sq_dist
                );
            }
            nearest_facet_recursive(
                p,
                nearest_facet, nearest_point, sq_dist,
                1, 0, mesh_->facets.nb()
            );
        }

        /**
         * \brief Computes the distance between an arbitrary 3d query
         *  point and the surface.
         * \param[in] p query point
         * \return the squared distance between \p p and the surface.
         */
        double squared_distance(const vec3& p) const {
            vec3 nearest_point;
            double result;
            nearest_facet(p, nearest_point, result);
            return result;
        }

        /**
         * \brief Tests whether there exists an intersection between a ray
         *  and the mesh.
         * \param[in] R the ray.
         * \param[in] tmax optional maximum parameter of the intersection along
         *  the ray.
         * \param[in] ignore_f optional facet to be ignored in intersection
         *  tests.
         * \retval true if there was an intersection.
         * \retval false otherwise.
         */
        bool ray_intersection(
            const Ray& R,
            double tmax = Numeric::max_float64(),
            index_t ignore_f = index_t(-1)
        ) const;


        /**
         * \brief Computes the nearest intersection along a ray.
         * \param[in] R the ray
         * \param[in,out] I the intersection. If I.t is set, then intersections
         *  further away than I.t are ignored. If I.f is set, then intersection
         *  with facet f is ignored.
         * \retval true if there was an intersection.
         * \retval false otherwise.
         */
        bool ray_nearest_intersection(const Ray& R, Intersection& I) const;

        /**
         * \brief Tests whether this surface mesh has an intersection
         *  with a segment.
         * \param[in] q1 , q2 the two extremities of the segment.
         * \retval true if there exists an intersection between [q1 , q2]
         *  and a facet of the mesh.
         * \retval false otherwise.
         */
        bool segment_intersection(const vec3& q1, const vec3& q2) const {
            return ray_intersection(Ray(q1, q2-q1), 1.0);
        }

        /**
         * \brief Finds the intersection between a segment and a surface that
         *  is nearest to the first extremity of the segment.
         * \param[in] q1 , q2 the two extremities of the segment.
         * \param[out] t if there was an intersection, it is t*q2 + (1-t)*q1
         * \param[out] f the intersected nearest facet or index_t(-1) if there
         *  was no intersection.
         * \retval true if there exists at least an intersection
         *  between [q1 , q2] and a facet of the mesh.
         * \retval false otherwise.
         */
        bool segment_nearest_intersection(
            const vec3& q1, const vec3& q2, double& t, index_t& f
        ) const {
            Ray R(q1, q2-q1);
            Intersection I;
            I.t = 1.0;
            bool result = ray_nearest_intersection(R,I);
            t = I.t;
            f = I.f;
            return result;
        }

        /**
         * \brief Calls a user function for all ray-facet intersection
         * \param[in] R the ray
         * \param[in] action the function to be called
         */
        void ray_all_intersections(
            const Ray& R,
            std::function<void(const Intersection&)> action
        ) const;

    protected:

        /**
         * \brief Computes a reasonable initialization for
         *  nearest facet search.
         *
         * \details A good initialization makes the algorithm faster,
         *  by allowing early pruning of subtrees that provably
         *  do not contain the nearest neighbor.
         *
         * \param[in] p query point
         * \param[out] nearest_facet a facet reasonably near p
         * \param[out] nearest_point a point in nearest_facet
         * \param[out] sq_dist squared distance between p and nearest_point
         */
        void get_nearest_facet_hint(
            const vec3& p,
            index_t& nearest_facet, vec3& nearest_point, double& sq_dist
        ) const;

        /**
         * \brief The recursive function used by the implementation
         *  of nearest_facet().
         *
         * \details The first call may use get_nearest_facet_hint()
         * to initialize nearest_facet, nearest_point and sq_dist,
         * as done in nearest_facet().
         *
         * \param[in] p query point
         * \param[in,out] nearest_facet the nearest facet so far,
         * \param[in,out] nearest_point a point in nearest_facet
         * \param[in,out] sq_dist squared distance between p and nearest_point
         * \param[in] n index of the current node in the AABB tree
         * \param[in] b index of the first facet in the subtree under node \p n
         * \param[in] e one position past the index of the last facet in the
         *  subtree under node \p n
         */
        void nearest_facet_recursive(
            const vec3& p,
            index_t& nearest_facet, vec3& nearest_point, double& sq_dist,
            index_t n, index_t b, index_t e
        ) const;

        /**
         * \brief The recursive function used by the implementation
         *  of ray_intersection()
         * \param[in] R the ray
         * \param[in] dirinv
         *              precomputed 1/(q2.x-q1.x), 1/(q2.y-q1.y), 1/(q2.z-q1.z)
         * \param[in] max_t the maximum value of t for an intersection
         * \param[in] ignore_f facet index to be ignored in tests
         * \param[in] n index of the current node in the AABB tree
         * \param[in] b index of the first facet in the subtree under node \p n
         * \param[in] e one position past the index of the last facet in the
         *  subtree under node \p n
         * \retval true if their was an intersection
         * \retval false otherwise
         */
        bool ray_intersection_recursive(
            const Ray& R, const vec3& dirinv, double max_t, index_t ignore_f,
            index_t n, index_t b, index_t e
        ) const;

        /**
         * \brief The recursive function used by the implementation
         *  of ray_nearest_intersection()
         * \param[in] R the ray
         * \param[in] dirinv
         *               precomputed 1/(q2.x-q1.x), 1/(q2.y-q1.y), 1/(q2.z-q1.z)
         * \param[in,out] I the parameters of the nearest intersection
         *   computed so-far. All intersections further away than I.t are
         *   ignored.
         * \param[in] ignore_f facet index to be ignored in tests
         * \param[in] n index of the current node in the AABB tree
         * \param[in] b index of the first facet in the subtree under node \p n
         * \param[in] e one position past the index of the last facet in the
         *  subtree under node \p n
         * \param[in] coord the current splitting coordinate, one of 0,1,2
         */
        void ray_nearest_intersection_recursive(
            const Ray& R, const vec3& dirinv, Intersection& I, index_t ignore_f,
            index_t n, index_t b, index_t e, index_t coord
        ) const;


        /**
         * \brief The function used to implement ray_all_intersections()
         * \param[in] R the ray
         * \param[in] dirinv
         *               precomputed 1/(q2.x-q1.x), 1/(q2.y-q1.y), 1/(q2.z-q1.z)
         * \param[in] action the function to be called
         * \param[in] n index of the current node in the AABB tree
         * \param[in] b index of the first facet in the subtree under node \p n
         * \param[in] e one position past the index of the last facet in the
         *  subtree under node \p n
         */
        void ray_all_intersections_recursive(
            const Ray& R, const vec3& dirinv,
            std::function<void(const Intersection&)> action,
            index_t n, index_t b, index_t e
        ) const;

    };

    /***********************************************************************/

    /**
     * \brief Axis Aligned Bounding Box tree of mesh cells.
     * \details Used to quickly find the tetrahedron that contains
     *  a given 3d point.
     */
    class GEOGRAM_API MeshCellsAABB : public MeshAABB3d {
    public:

        /**
         * \brief Symbolic constant for indicating that there
         *  is no containing tetrahedron.
         * \see containing_tet()
         */
        static const index_t NO_TET = index_t(-1);

        /**
         * \brief MeshCellsAABB constructor.
         * \details Creates an uninitialized MeshCellsAABB.
         */
        MeshCellsAABB();

        /**
         * \brief Creates the Axis Aligned Bounding Boxes tree.
         * \param[in] M the input mesh. It can be modified,
         *  The cells are re-ordered (using Morton's order, see mesh_reorder()).
         * \param[in] reorder if not set, Morton re-ordering is
         *  skipped (but it means that mesh_reorder() was previously
         *  called else the algorithm will be pretty unefficient).
         */
        MeshCellsAABB(Mesh& M, bool reorder = true);

        /**
         * \brief Initializes the Axis Aligned Bounding Boxes tree.
         * \param[in] M the input mesh. It can be modified,
         *  The cells are re-ordered (using Morton's order, see mesh_reorder()).
         * \param[in] reorder if not set, Morton re-ordering is
         *  skipped (but it means that mesh_reorder() was previously
         *  called else the algorithm will be pretty unefficient).
         */
        void initialize(Mesh& M, bool reorder = true);

        /**
         * \brief Finds the index of a tetrahedron that contains a query point
         * \param[in] p a const reference to the query point
         * \return the index of one of the tetrahedra that contains \p p or
         *  NO_TET if \p p is outside the mesh.
         * \note The input mesh needs to be tetrahedralized. If the mesh has
         *   arbitrary cells, then one may use instead containing_boxes().
         */
        index_t containing_tet(const vec3& p) const {
            geo_debug_assert(mesh_->cells.are_simplices());
            return containing_tet_recursive(
                p, 1, 0, mesh_->cells.nb()
            );
        }

        /**
         * \brief Computes all the intersections between a given
         *  box and the bounding boxes of all the cells.
         * \param[in] action a function that takes as argument
         *  an index_t (cell index) invoked for all cells that
         *  have a bounding box that intersects \p box_in.
         */
        void compute_bbox_cell_bbox_intersections(
            const Box& box_in,
            std::function<void(index_t)> action
        ) const {
            bbox_intersect_recursive(
                action, box_in, 1, 0, mesh_->cells.nb()
            );
        }

        /**
         * \brief Finds all the cells such that their bounding
         *  box contain a point.
         * \param[in] action a function that takes an index_t
         *  that is invoked for all cells that have a bounding
         *  box that contains \p p.
         */
        void containing_boxes(
            const vec3& p, std::function<void(index_t)> action
        ) const {
            containing_bboxes_recursive(
                action, p, 1, 0, mesh_->cells.nb()
            );
        }

        /**
         * \brief Computes all the pairs of intersecting cells.
         * \param[in] action is a function that takes two index_t's,
         *  invoked of all pairs of cells that have overlapping
         *  bounding boxes. Further processing is necessary to
         *  detect actual cell intersections.
         */
        void compute_cell_bbox_intersections(
            std::function<void(index_t, index_t)> action
        ) const {
            self_intersect_recursive(
                action,
                1, 0, mesh_->cells.nb(),
                1, 0, mesh_->cells.nb()
            );
        }

        /**
         * \brief Computes all the pairs of intersecting cells between this
         *  AABB and another one.
         * \param[in] action is a function that takes two index_t's,
         *  invoked of all pairs of cells that have overlapping
         *  bounding boxes. Further processing is necessary to
         *  detect actual cell intersections.
         * \param[in] other the other AABB.
         */
        void compute_other_cell_bbox_intersections(
            MeshCellsAABB* other,
            std::function<void(index_t, index_t)> action
        ) const {
            other_intersect_recursive(
                action,
                1, 0, mesh_->cells.nb(),
                other,
                1, 0, other->mesh_->cells.nb()
            );
        }


    protected:

        /**
         * \brief The recursive function used by the implementation
         *  of containing_tet().
         * \param[in] p a const reference to the query point
         * \param[in] n index of the current node in the AABB tree
         * \param[in] b index of the first tet in the subtree under node \p n
         * \param[in] e one position past the index of the last tet in the
         *  subtree under node \p n
         * \return the index of one of the tetrahedra that contains \p p, or
         *  NO_TET if \p p is outside the mesh.
         */
        index_t containing_tet_recursive(
            const vec3& p,
            index_t n, index_t b, index_t e
        ) const;


        /**
         * \brief Computes all the cells that have a bbox that
         *  contain a given point in a sub-tree of the AABB tree.
         *
         * Note that the tree structure is completely implicit,
         *  therefore the bounds of the (continuous) facet indices
         *  sequences that correspond to the facets contained
         *  in the two nodes are sent as well as the node indices.
         *
         * \param[in] action a function that takes an index_t that is
         *  invoked for all cells that has a bounding box that
         *  contains \p p.
         * \param[in] p a const reference to the query point
         * \param[in] node index of the first node of the AABB tree
         * \param[in] b index of the first facet in \p node
         * \param[in] e one position past the index of the last
         *  facet in \p node
         */
        void containing_bboxes_recursive(
            std::function<void(index_t)> action,
            const vec3& p,
            index_t node, index_t b, index_t e
        ) const {
            geo_debug_assert(e != b);

            // Prune sub-tree that does not have intersection
            if(!bboxes_[node].contains(p)) {
                return;
            }

            // Leaf case
            if(e == b+1) {
                action(b);
                return;
            }

            // Recursion
            index_t m = b + (e - b) / 2;
            index_t node_l = 2 * node;
            index_t node_r = 2 * node + 1;

            containing_bboxes_recursive(action, p, node_l, b, m);
            containing_bboxes_recursive(action, p, node_r, m, e);
        }
    };

/*******************************************************************/

    /**
     * \brief Axis Aligned Bounding Box tree of mesh facets in 2D.
     * \details Used to quickly find the facet that contains
     *  a given 2d point.
     */
    class GEOGRAM_API MeshFacetsAABB2d : public MeshAABB2d {
    public:

        /**
         * \brief Symbolic constant for indicating that there
         *  is no containing tetrahedron.
         * \see containing_tet()
         */
        static const index_t NO_TRIANGLE = index_t(-1);

        /**
         * \brief MeshFacetsAABB2d constructor.
         * \details Creates an uninitialized MeshFacetsAABB2d.
         */
        MeshFacetsAABB2d();

        /**
         * \brief Creates the Axis Aligned Bounding Boxes tree.
         * \param[in] M the input mesh. It can be modified,
         *  The cells are re-ordered (using Morton's order, see mesh_reorder()).
         * \param[in] reorder if not set, Morton re-ordering is
         *  skipped (but it means that mesh_reorder() was previously
         *  called else the algorithm will be pretty unefficient).
         */
        MeshFacetsAABB2d(Mesh& M, bool reorder = true);

        /**
         * \brief Initializes the Axis Aligned Bounding Boxes tree.
         * \param[in] M the input mesh. It can be modified,
         *  The cells are re-ordered (using Morton's order, see mesh_reorder()).
         * \param[in] reorder if not set, Morton re-ordering is
         *  skipped (but it means that mesh_reorder() was previously
         *  called else the algorithm will be pretty unefficient).
         */
        void initialize(Mesh& M, bool reorder = true);

        /**
         * \brief Finds the index of a facet that contains a query point
         * \param[in] p a const reference to the query point
         * \return the index of one of the facetthat contains \p p or
         *  NO_TRIANGLE if \p p is outside the mesh.
         * \note The input mesh needs to be triangulated. If the mesh has
         *   arbitrary cells, then one may use instead containing_boxes().
         */
        index_t containing_triangle(const vec2& p) const {
            geo_debug_assert(mesh_->facets.are_simplices());
            return containing_triangle_recursive(
                p, 1, 0, mesh_->facets.nb()
            );
        }

        /**
         * \brief Computes all the intersections between a given
         *  box and the bounding boxes of all the facets.
         * \param[in] action a function that takes as argument
         *  an index_t (cell index) invoked for all cells that
         *  have a bounding box that intersects \p box_in.
         */
        void compute_bbox_cell_bbox_intersections(
            const Box2d& box_in,
            std::function<void(index_t)> action
        ) const {
            bbox_intersect_recursive(
                action, box_in, 1, 0, mesh_->facets.nb()
            );
        }

        /**
         * \brief Finds all the cells such that their bounding
         *  box contain a point.
         * \param[in] action a function that takes an index_t
         *  that is invoked for all cells that have a bounding
         *  box that contains \p p.
         */
        void containing_boxes(
            const vec2& p, std::function<void(index_t)> action
        ) const {
            containing_bboxes_recursive(
                action, p, 1, 0, mesh_->facets.nb()
            );
        }

        /**
         * \brief Computes all the pairs of intersecting facets.
         * \param[in] action is a function that takes two index_t's,
         *  invoked of all pairs of cells that have overlapping
         *  bounding boxes. Further processing is necessary to
         *  detect actual cell intersections.
         */
        void compute_facet_bbox_intersections(
            std::function<void(index_t, index_t)> action
        ) const {
            self_intersect_recursive(
                action,
                1, 0, mesh_->facets.nb(),
                1, 0, mesh_->facets.nb()
            );
        }

        /**
         * \brief Computes all the pairs of intersecting cells between this
         *  AABB and another one.
         * \param[in] action is a function that takes two index_t's,
         *  invoked of all pairs of cells that have overlapping
         *  bounding boxes. Further processing is necessary to
         *  detect actual cell intersections.
         * \param[in] other the other AABB.
         */
        void compute_other_cell_bbox_intersections(
            MeshFacetsAABB2d* other,
            std::function<void(index_t, index_t)> action
        ) const {
            other_intersect_recursive(
                action,
                1, 0, mesh_->facets.nb(),
                other,
                1, 0, other->mesh_->facets.nb()
            );
        }


    protected:

        /**
         * \brief The recursive function used by the implementation
         *  of containing_triangle().
         * \param[in] p a const reference to the query point
         * \param[in] n index of the current node in the AABB tree
         * \param[in] b index of the first tet in the subtree under node \p n
         * \param[in] e one position past the index of the last tet in the
         *  subtree under node \p n
         * \return the index of one of the tetrahedra that contains \p p, or
         *  NO_TRIANGLE if \p p is outside the mesh.
         */
        index_t containing_triangle_recursive(
            const vec2& p,
            index_t n, index_t b, index_t e
        ) const;


        /**
         * \brief Computes all the cells that have a bbox that
         *  contain a given point in a sub-tree of the AABB tree.
         *
         * Note that the tree structure is completely implicit,
         *  therefore the bounds of the (continuous) facet indices
         *  sequences that correspond to the facets contained
         *  in the two nodes are sent as well as the node indices.
         *
         * \param[in] action a function that takes an index_t that is
         *  invoked for all cells that has a bounding box that
         *  contains \p p.
         * \param[in] p a const reference to the query point
         * \param[in] node index of the first node of the AABB tree
         * \param[in] b index of the first facet in \p node
         * \param[in] e one position past the index of the last
         *  facet in \p node
         */
        void containing_bboxes_recursive(
            std::function<void(index_t)> action,
            const vec2& p,
            index_t node, index_t b, index_t e
        ) const {
            geo_debug_assert(e != b);

            // Prune sub-tree that does not have intersection
            if(!bboxes_[node].contains(p)) {
                return;
            }

            // Leaf case
            if(e == b+1) {
                action(b);
                return;
            }

            // Recursion
            index_t m = b + (e - b) / 2;
            index_t node_l = 2 * node;
            index_t node_r = 2 * node + 1;

            containing_bboxes_recursive(action, p, node_l, b, m);
            containing_bboxes_recursive(action, p, node_r, m, e);
        }
    };


}

#endif
